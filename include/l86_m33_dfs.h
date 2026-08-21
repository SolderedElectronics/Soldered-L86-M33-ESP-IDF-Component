/**
 * @file l86_m33_dfs.h
 * @brief NMEA and PMTK definitions and enumerations for the L86-M33
 *
 * The NMEA output the module produces is described in the Quectel L86 GNSS
 * Protocol Specification, and the PMTK command set used to configure it in the
 * Quectel PMTK Protocol Specification:
 * https://www.quectel.com/product/gnss-l86/
 *
 * @author Soldered Electronics
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// *****************************************************************************
// Section: Defaults
//
// The module leaves the factory talking NMEA at 9600 baud with a 1 Hz fix rate,
// and the Soldered breakout does not change that.

#define L86_M33_DEFAULT_BAUD_RATE      9600 /**< Baud rate the module boots up with */
#define L86_M33_DEFAULT_RX_BUFFER_SIZE 512  /**< Receive ring buffer, holds a few seconds of NMEA */
#define L86_M33_MIN_RX_BUFFER_SIZE     128  /**< Smallest ring buffer the UART driver accepts */

/** Longest NMEA field kept while parsing, including the string terminator */
#define L86_M33_NMEA_MAX_FIELD_LEN 16

/** Longest command l86_m33_send_command() accepts, without the checksum */
#define L86_M33_MAX_COMMAND_LEN 96

// *****************************************************************************
// Section: Unit conversions
//
// The module reports speed in knots and altitude in metres; everything else is
// derived from that.

#define L86_M33_KNOTS_TO_KMPH 1.852f  /**< One knot in km/h */
#define L86_M33_KNOTS_TO_MPS  0.5144444f /**< One knot in m/s */

/** Mean earth radius in metres, as used for distance and course calculations */
#define L86_M33_EARTH_RADIUS_M 6372795.0

// *****************************************************************************
// Section: PMTK commands
//
// PMTK packets are plain NMEA sentences, so they are sent without a checksum
// here and l86_m33_send_command() appends the one it calculates.

#define L86_M33_PMTK_HOT_START       "$PMTK101" /**< Restart, keeping all assistance data */
#define L86_M33_PMTK_WARM_START      "$PMTK102" /**< Restart, dropping the ephemeris */
#define L86_M33_PMTK_COLD_START      "$PMTK103" /**< Restart, dropping position and time too */
#define L86_M33_PMTK_FULL_COLD_START "$PMTK104" /**< Restart and clear the whole system and user configuration */
#define L86_M33_PMTK_STANDBY         "$PMTK161,0" /**< Enter standby, woken by any serial traffic */
#define L86_M33_PMTK_SET_FIX_CTL     "$PMTK220" /**< Position fix interval in ms */
#define L86_M33_PMTK_SET_PERIODIC    "$PMTK225" /**< Periodic and AlwaysLocate power modes */
#define L86_M33_PMTK_SET_AIC         "$PMTK286" /**< Multi-tone active interference cancellation */
#define L86_M33_PMTK_API_SET_NMEA    "$PMTK314" /**< Which NMEA sentences are output, and how often */
#define L86_M33_PMTK_EASY            "$PMTK869" /**< EASY, the module's self-generated orbit prediction */

/** Shortest position fix interval the module accepts, in ms */
#define L86_M33_FIX_INTERVAL_MIN_MS 100
/** Longest position fix interval the module accepts, in ms */
#define L86_M33_FIX_INTERVAL_MAX_MS 10000

/** Highest output rate a sentence can be set to, in position fixes per sentence */
#define L86_M33_NMEA_RATE_MAX 5

/**
 * @brief Power mode set with PMTK225
 *
 * ::L86_M33_POWER_NORMAL keeps the receiver tracking continuously. The
 * AlwaysLocate modes let the module decide when to sleep and when to wake up
 * based on how much the position is changing, which cuts average power draw a
 * lot at the cost of a position that is only refreshed when the module feels
 * like it.
 */
typedef enum {
    L86_M33_POWER_NORMAL = 0,               /**< Continuous tracking */
    L86_M33_POWER_ALWAYSLOCATE_STANDBY = 8, /**< AlwaysLocate, sleeping in standby */
    L86_M33_POWER_ALWAYSLOCATE_BACKUP = 9,  /**< AlwaysLocate, sleeping in backup, lower power */
} l86_m33_power_mode_t;

/**
 * @brief How much of the assistance data a restart throws away
 *
 * The further down the list, the longer the module takes to get a fix again.
 */
typedef enum {
    L86_M33_RESTART_HOT,       /**< Keep time, position and ephemeris */
    L86_M33_RESTART_WARM,      /**< Keep time and position, drop the ephemeris */
    L86_M33_RESTART_COLD,      /**< Drop all assistance data */
    L86_M33_RESTART_FULL_COLD, /**< Drop assistance data and reset the configuration to defaults */
} l86_m33_restart_t;

// *****************************************************************************
// Section: NMEA
//
// Only the two sentences that carry everything this driver reports are decoded.
// GGA has position, altitude, satellite count and HDOP, RMC has position,
// speed, course, date and time.

/** Sentence the parser is currently working through */
typedef enum {
    L86_M33_SENTENCE_OTHER, /**< Anything not decoded, including the PMTK acknowledgements */
    L86_M33_SENTENCE_GGA,   /**< Global positioning system fix data */
    L86_M33_SENTENCE_RMC,   /**< Recommended minimum specific GNSS data */
} l86_m33_sentence_t;

/**
 * @brief Quality of the fix, out of field 6 of GGA
 *
 * Anything other than ::L86_M33_FIX_NONE means the reported position is a real
 * measurement.
 */
typedef enum {
    L86_M33_FIX_NONE = 0,      /**< No fix, the position fields are empty */
    L86_M33_FIX_GNSS = 1,      /**< Standard GNSS fix */
    L86_M33_FIX_DGNSS = 2,     /**< Differentially corrected fix */
    L86_M33_FIX_ESTIMATED = 6, /**< Dead reckoning, position estimated from the last real fix */
} l86_m33_fix_quality_t;

/**
 * @brief Which NMEA sentences the module sends, set with PMTK314
 *
 * Each field is an output rate in position fixes per sentence, so 1 sends the
 * sentence on every fix, 5 on every fifth fix, and 0 turns it off. Turning off
 * what the application does not read keeps the UART from being flooded at high
 * fix rates. GGA and RMC are the two this driver decodes, so leaving both at 0
 * means nothing gets parsed.
 */
typedef struct {
    uint8_t gll; /**< Geographic position, latitude and longitude */
    uint8_t rmc; /**< Recommended minimum specific GNSS data, decoded by this driver */
    uint8_t vtg; /**< Course over ground and ground speed */
    uint8_t gga; /**< Global positioning system fix data, decoded by this driver */
    uint8_t gsa; /**< GNSS DOP and active satellites */
    uint8_t gsv; /**< GNSS satellites in view */
    uint8_t zda; /**< Date and time */
} l86_m33_nmea_output_t;

/** Output configuration the module starts up with: everything this driver uses, on every fix */
#define L86_M33_NMEA_OUTPUT_DEFAULT()                                                                                  \
    {                                                                                                                  \
        .gll = 1, .rmc = 1, .vtg = 1, .gga = 1, .gsa = 1, .gsv = 5, .zda = 0                                           \
    }

/** Output configuration with only the two sentences this driver decodes left on */
#define L86_M33_NMEA_OUTPUT_MINIMAL()                                                                                  \
    {                                                                                                                  \
        .gll = 0, .rmc = 1, .vtg = 0, .gga = 1, .gsa = 0, .gsv = 0, .zda = 0                                           \
    }

#ifdef __cplusplus
}
#endif
