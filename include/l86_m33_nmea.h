/**
 * @file l86_m33_nmea.h
 * @brief NMEA sentence decoder used by the soldered-l86-m33 component
 *
 * The decoder is fed one character at a time and keeps the last value it
 * successfully decoded for each quantity. It holds no buffers of its own beyond
 * the field currently being read, so it can run straight off the UART without
 * the application having to assemble whole sentences.
 *
 * Values are staged while a sentence is being read and only copied into place
 * once the sentence checksum has been verified, so a corrupted sentence cannot
 * overwrite good data. The application does not normally touch any of this
 * directly; use the accessors in soldered_l86_m33.h instead.
 *
 * @author Soldered Electronics
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "l86_m33_dfs.h"

/**
 * @brief State shared by every decoded quantity
 */
typedef struct {
    bool valid;        /**< Set once the quantity has been decoded at least once */
    int64_t commit_us; /**< esp_timer timestamp of the last time the value was updated */
} l86_m33_meta_t;

/** Position, out of GGA or RMC */
typedef struct {
    l86_m33_meta_t meta;
    double latitude;         /**< Degrees north of the equator, negative for south */
    double longitude;        /**< Degrees east of Greenwich, negative for west */
    double latitude_staged;  /**< Value read out of the sentence being parsed */
    double longitude_staged; /**< Value read out of the sentence being parsed */
} l86_m33_location_t;

/** UTC date, out of RMC */
typedef struct {
    l86_m33_meta_t meta;
    uint16_t year;  /**< Four digit year */
    uint8_t month;  /**< 1 to 12 */
    uint8_t day;    /**< 1 to 31 */
    uint32_t staged; /**< ddmmyy as the module sends it */
} l86_m33_date_t;

/** UTC time of day, out of GGA or RMC */
typedef struct {
    l86_m33_meta_t meta;
    uint8_t hour;        /**< 0 to 23 */
    uint8_t minute;      /**< 0 to 59 */
    uint8_t second;      /**< 0 to 59 */
    uint8_t centisecond; /**< 0 to 99 */
    uint32_t staged;     /**< hhmmsscc, assembled from the sentence field */
} l86_m33_time_t;

/** A quantity the module sends as a decimal number */
typedef struct {
    l86_m33_meta_t meta;
    float value;  /**< Last decoded value */
    float staged; /**< Value read out of the sentence being parsed */
} l86_m33_decimal_t;

/** A quantity the module sends as a whole number */
typedef struct {
    l86_m33_meta_t meta;
    uint32_t value;  /**< Last decoded value */
    uint32_t staged; /**< Value read out of the sentence being parsed */
} l86_m33_integer_t;

/**
 * @brief Decoder state
 *
 * Zero initialized state is a valid starting point, which is what
 * l86_m33_nmea_reset() puts it back to.
 */
typedef struct {
    /* Sentence being assembled */
    char term[L86_M33_NMEA_MAX_FIELD_LEN]; /**< Field currently being read */
    uint8_t term_offset;                   /**< How much of the field has been read */
    uint8_t term_number;                   /**< Which field of the sentence this is, 0 being the talker and type */
    uint8_t parity;                        /**< Running XOR of everything between '$' and '*' */
    bool is_checksum_term;                 /**< The field after '*' holds the checksum, not data */
    bool sentence_has_fix;                 /**< The sentence being read reports a real position */
    uint16_t staged_mask;                  /**< Which quantities the sentence being read has staged a value for */
    l86_m33_sentence_t sentence_type;      /**< Type of the sentence being read */

    /* Decoded quantities */
    l86_m33_location_t location;   /**< Position in degrees */
    l86_m33_date_t date;           /**< UTC date */
    l86_m33_time_t time;           /**< UTC time of day */
    l86_m33_decimal_t speed;       /**< Speed over ground in knots, as the module sends it */
    l86_m33_decimal_t course;      /**< Course over ground in degrees from true north */
    l86_m33_decimal_t altitude;    /**< Altitude above mean sea level in metres */
    l86_m33_decimal_t hdop;        /**< Horizontal dilution of precision */
    l86_m33_integer_t satellites;  /**< Satellites used for the fix */
    l86_m33_integer_t fix_quality; /**< Fix quality, a ::l86_m33_fix_quality_t */

    /* Statistics */
    uint32_t chars_processed;    /**< Characters fed to the decoder */
    uint32_t sentences_with_fix; /**< Decoded sentences that reported a real position */
    uint32_t passed_checksums;   /**< Sentences of any type whose checksum was correct */
    uint32_t failed_checksums;   /**< Sentences of any type whose checksum was wrong */
} l86_m33_nmea_t;

/**
 * @brief Throw away all decoded data and start again from a clean sentence
 *
 * @param[out] nmea Decoder state
 */
void l86_m33_nmea_reset(l86_m33_nmea_t *nmea);

/**
 * @brief Feed one character to the decoder
 *
 * @param[in,out] nmea Decoder state
 * @param[in] c Character straight off the UART
 *
 * @return true if this character completed a decoded sentence whose checksum
 *         was correct, meaning at least one value was just updated
 */
bool l86_m33_nmea_encode(l86_m33_nmea_t *nmea, char c);

/**
 * @brief Work out how long ago a quantity was last updated
 *
 * @param[in] meta State of the quantity
 *
 * @return Milliseconds since the value was last updated, or UINT32_MAX if it
 *         has never been decoded
 */
uint32_t l86_m33_nmea_age_ms(const l86_m33_meta_t *meta);

#ifdef __cplusplus
}
#endif
