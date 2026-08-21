/**
 * @file soldered_l86_m33.h
 * @brief Public API for the soldered-l86-m33 component
 *
 * ESP-IDF driver for the Soldered GNSS GPS L86-M33 Breakout. The module sends
 * NMEA sentences over UART and takes PMTK commands back the same way, so the
 * driver owns a UART port, decodes what arrives on it, and hands the decoded
 * position, time, speed and fix quality to the application.
 *
 * @author Soldered Electronics
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "l86_m33_dfs.h"
#include "l86_m33_nmea.h"

/**
 * @brief Callback invoked by l86_m33_update() when a sentence has been decoded
 *
 * Runs in the context of whichever task called l86_m33_update(), so it is free
 * to read the module and to talk to the rest of the system. It fires once per
 * decoded sentence, which at the default fix rate means a handful of times a
 * second.
 *
 * @param[in] arg The pointer handed to l86_m33_set_fix_callback()
 */
typedef void (*l86_m33_fix_cb_t)(void *arg);

/**
 * @brief How the module is wired up
 *
 * Use ::L86_M33_DEFAULT_CONFIG to fill this in and then change what needs
 * changing.
 */
typedef struct {
    uart_port_t uart_port; /**< UART peripheral to drive the module with */
    int rx_pin;            /**< Pin wired to the TX pin of the module */
    int tx_pin;            /**< Pin wired to the RX pin of the module, or -1 if commands are never sent */
    uint32_t baud_rate;    /**< Baud rate the module is running at, ::L86_M33_DEFAULT_BAUD_RATE out of the box */
    int rx_buffer_size;    /**< Receive ring buffer in bytes, at least ::L86_M33_MIN_RX_BUFFER_SIZE */
} l86_m33_config_t;

/**
 * @brief Configuration for a module wired to the given pins, everything else left at its default
 *
 * @param rx Pin wired to the TX pin of the module
 * @param tx Pin wired to the RX pin of the module
 */
#define L86_M33_DEFAULT_CONFIG(rx, tx)                                                                                 \
    {                                                                                                                  \
        .uart_port = UART_NUM_1, .rx_pin = (rx), .tx_pin = (tx), .baud_rate = L86_M33_DEFAULT_BAUD_RATE,                \
        .rx_buffer_size = L86_M33_DEFAULT_RX_BUFFER_SIZE                                                               \
    }

/**
 * @brief Handle for one L86-M33 module
 *
 * Create one per module. All fields are managed by the driver; treat the struct
 * as opaque and read state through the accessor functions.
 */
typedef struct {
    uart_port_t uart_port;   /**< UART port opened by l86_m33_init() */
    int tx_pin;              /**< Pin commands are sent on, or -1 if there is none */
    l86_m33_nmea_t nmea;     /**< Decoder holding everything read so far */
    l86_m33_fix_cb_t fix_cb; /**< Callback for decoded sentences, or NULL */
    void *fix_arg;           /**< Argument passed to the callback */
} l86_m33_t;

/**
 * @brief Everything the module has reported, read out in one go
 *
 * A quantity whose `_valid` flag is false has not been decoded yet and its
 * value is meaningless. Position, date, time, speed and course only become
 * valid once the module has a fix, which outdoors takes anywhere from a second
 * on a hot start to a minute or so on a cold one.
 */
typedef struct {
    bool location_valid;      /**< Position has been decoded */
    double latitude;          /**< Degrees north of the equator, negative for south */
    double longitude;         /**< Degrees east of Greenwich, negative for west */
    uint32_t location_age_ms; /**< Milliseconds since the position was last updated */

    bool date_valid; /**< Date has been decoded */
    uint16_t year;   /**< Four digit year, UTC */
    uint8_t month;   /**< 1 to 12, UTC */
    uint8_t day;     /**< 1 to 31, UTC */

    bool time_valid;      /**< Time of day has been decoded */
    uint8_t hour;         /**< 0 to 23, UTC */
    uint8_t minute;       /**< 0 to 59, UTC */
    uint8_t second;       /**< 0 to 59, UTC */
    uint8_t centisecond;  /**< 0 to 99, UTC */
    uint32_t datetime_age_ms; /**< Milliseconds since the date and time were last updated */

    bool speed_valid;  /**< Speed over ground has been decoded */
    float speed_kmph;  /**< Speed over ground in km/h */
    bool course_valid; /**< Course over ground has been decoded */
    float course_deg;  /**< Course over ground in degrees from true north */

    bool altitude_valid; /**< Altitude has been decoded */
    float altitude_m;    /**< Altitude above mean sea level in metres */

    bool satellites_valid; /**< Satellite count has been decoded */
    uint32_t satellites;   /**< Satellites used for the fix */
    bool hdop_valid;       /**< HDOP has been decoded */
    float hdop;            /**< Horizontal dilution of precision, smaller is better */

    bool fix_quality_valid;             /**< Fix quality has been decoded */
    l86_m33_fix_quality_t fix_quality;  /**< What kind of fix the module reports */
} l86_m33_data_t;

/**
 * @brief Counters describing what has come in over the UART
 *
 * Useful for telling a wiring problem from a positioning problem: no characters
 * at all means the module is not talking, characters with no passed checksums
 * usually means the baud rate is wrong, and passed checksums with no sentences
 * with a fix simply means the module has not found the satellites yet.
 */
typedef struct {
    uint32_t chars_processed;    /**< Characters received and fed to the decoder */
    uint32_t sentences_with_fix; /**< Decoded sentences that reported a real position */
    uint32_t passed_checksums;   /**< Sentences of any type whose checksum was correct */
    uint32_t failed_checksums;   /**< Sentences of any type whose checksum was wrong */
} l86_m33_stats_t;

/**
 * @brief Open a UART port for the module
 *
 * Installs the UART driver on the port named in `config`, wires it to the given
 * pins at 8N1, and clears the decoder. Nothing is read here; call
 * l86_m33_update() to pull in and decode what the module sends.
 *
 * The module starts sending NMEA on its own as soon as it is powered, so no
 * configuration is needed to get readings out of it.
 *
 * @param[out] dev Handle to initialize
 * @param[in] config How the module is wired up
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument or a
 *         receive buffer below ::L86_M33_MIN_RX_BUFFER_SIZE, or the error
 *         returned by the UART driver
 */
esp_err_t l86_m33_init(l86_m33_t *dev, const l86_m33_config_t *config);

/**
 * @brief Close the UART port
 *
 * @param[in,out] dev Handle previously initialized with l86_m33_init()
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL handle, or the error
 *         returned by uart_driver_delete()
 */
esp_err_t l86_m33_deinit(l86_m33_t *dev);

/**
 * @brief Read whatever the module has sent and decode it
 *
 * Drains the UART receive buffer without blocking and feeds it to the decoder,
 * then calls the callback registered with l86_m33_set_fix_callback() once for
 * every sentence that was decoded.
 *
 * This has to be called often enough that the receive buffer never overflows,
 * which at the default fix rate and buffer size means at least a few times a
 * second. Use l86_m33_update_timeout() instead of a plain delay in a loop that
 * has nothing else to do.
 *
 * @param[in,out] dev Handle
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL handle, or the error
 *         returned by the UART driver
 */
esp_err_t l86_m33_update(l86_m33_t *dev);

/**
 * @brief Keep reading and decoding for a while
 *
 * Same as l86_m33_update(), except that it keeps going for `duration_ms`,
 * blocking on the UART while it waits. Meant as the replacement for a plain
 * delay: the module keeps sending sentences whether or not the application is
 * listening, so time spent in vTaskDelay() is time spent filling up the receive
 * buffer.
 *
 * @param[in,out] dev Handle
 * @param[in] duration_ms How long to keep reading, in ms
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL handle, or the error
 *         returned by the UART driver
 */
esp_err_t l86_m33_update_timeout(l86_m33_t *dev, uint32_t duration_ms);

/**
 * @brief Register a callback for decoded sentences
 *
 * The callback is invoked from l86_m33_update(), not from an interrupt, so it
 * has no restrictions on what it may do. Pass NULL to remove a previously
 * registered callback.
 *
 * @param[in,out] dev Handle
 * @param[in] cb Function to call once per decoded sentence, or NULL
 * @param[in] arg Passed to the callback untouched
 */
void l86_m33_set_fix_callback(l86_m33_t *dev, l86_m33_fix_cb_t cb, void *arg);

/**
 * @brief Check whether the module has reported a position
 *
 * True once a sentence carrying a real fix has been decoded, and it stays true
 * afterwards even if the fix is later lost, since the last known position is
 * still there to be read. Use l86_m33_get_location_age() to tell a fresh
 * position from a stale one.
 *
 * @param[in] dev Handle
 *
 * @return true if there is a position to read
 */
bool l86_m33_available(const l86_m33_t *dev);

/**
 * @brief Read everything the module has reported in one go
 *
 * A snapshot, so all the values belong to the same moment. Nothing here can
 * fail; check the `_valid` flags of ::l86_m33_data_t for what has actually been
 * decoded.
 *
 * @param[in] dev Handle
 * @param[out] data Everything decoded so far
 *
 * @return ESP_OK on success, or ESP_ERR_INVALID_ARG on a NULL argument
 */
esp_err_t l86_m33_get_data(const l86_m33_t *dev, l86_m33_data_t *data);

/**
 * @brief Read the counters describing what has come in over the UART
 *
 * @param[in] dev Handle
 * @param[out] stats Counters
 *
 * @return ESP_OK on success, or ESP_ERR_INVALID_ARG on a NULL argument
 */
esp_err_t l86_m33_get_stats(const l86_m33_t *dev, l86_m33_stats_t *stats);

/**
 * @brief Read the last reported position
 *
 * @param[in] dev Handle
 * @param[out] latitude Degrees north of the equator, negative for south
 * @param[out] longitude Degrees east of Greenwich, negative for west
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if no position has been decoded yet
 */
esp_err_t l86_m33_get_location(const l86_m33_t *dev, double *latitude, double *longitude);

/**
 * @brief Work out how old the last reported position is
 *
 * The module sends a position on every fix, so at the default fix rate this
 * stays under a second while the fix holds. A value that keeps climbing means
 * the fix was lost, or that l86_m33_update() is not being called.
 *
 * @param[in] dev Handle
 * @param[out] age_ms Milliseconds since the position was last updated
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if no position has been decoded yet
 */
esp_err_t l86_m33_get_location_age(const l86_m33_t *dev, uint32_t *age_ms);

/**
 * @brief Read the last reported UTC date
 *
 * @param[in] dev Handle
 * @param[out] year Four digit year, or NULL if not wanted
 * @param[out] month 1 to 12, or NULL if not wanted
 * @param[out] day 1 to 31, or NULL if not wanted
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL handle, or
 *         ESP_ERR_INVALID_STATE if no date has been decoded yet
 */
esp_err_t l86_m33_get_date(const l86_m33_t *dev, uint16_t *year, uint8_t *month, uint8_t *day);

/**
 * @brief Read the last reported UTC time of day
 *
 * The module keeps sending a time of day out of its own clock before it has a
 * fix, which is why the time is only reported here once a sentence with a fix
 * has arrived and the time can be trusted.
 *
 * @param[in] dev Handle
 * @param[out] hour 0 to 23, or NULL if not wanted
 * @param[out] minute 0 to 59, or NULL if not wanted
 * @param[out] second 0 to 59, or NULL if not wanted
 * @param[out] centisecond 0 to 99, or NULL if not wanted
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL handle, or
 *         ESP_ERR_INVALID_STATE if no time has been decoded yet
 */
esp_err_t l86_m33_get_time(const l86_m33_t *dev, uint8_t *hour, uint8_t *minute, uint8_t *second,
                           uint8_t *centisecond);

/**
 * @brief Work out how old the last reported date and time are
 *
 * @param[in] dev Handle
 * @param[out] age_ms Milliseconds since the date and time were last updated
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if no time has been decoded yet
 */
esp_err_t l86_m33_get_datetime_age(const l86_m33_t *dev, uint32_t *age_ms);

/**
 * @brief Read the last reported speed over ground in km/h
 *
 * Speed comes out of the position, so standing still with a weak fix reads as a
 * slow wander rather than as a clean zero.
 *
 * @param[in] dev Handle
 * @param[out] kmph Speed in km/h
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if no speed has been decoded yet
 */
esp_err_t l86_m33_get_speed_kmph(const l86_m33_t *dev, float *kmph);

/**
 * @brief Read the last reported speed over ground in m/s
 *
 * @param[in] dev Handle
 * @param[out] mps Speed in m/s
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if no speed has been decoded yet
 */
esp_err_t l86_m33_get_speed_mps(const l86_m33_t *dev, float *mps);

/**
 * @brief Read the last reported speed over ground in knots, as the module sends it
 *
 * @param[in] dev Handle
 * @param[out] knots Speed in knots
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if no speed has been decoded yet
 */
esp_err_t l86_m33_get_speed_knots(const l86_m33_t *dev, float *knots);

/**
 * @brief Read the last reported course over ground
 *
 * Like speed, this is worked out from the position, so it only means anything
 * while actually moving.
 *
 * @param[in] dev Handle
 * @param[out] degrees Course in degrees clockwise from true north
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if no course has been decoded yet
 */
esp_err_t l86_m33_get_course(const l86_m33_t *dev, float *degrees);

/**
 * @brief Read the last reported altitude above mean sea level
 *
 * GNSS altitude is a good deal noisier than GNSS position, so expect it to
 * wander by tens of metres on a marginal fix.
 *
 * @param[in] dev Handle
 * @param[out] meters Altitude in metres
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if no altitude has been decoded yet
 */
esp_err_t l86_m33_get_altitude(const l86_m33_t *dev, float *meters);

/**
 * @brief Read how many satellites went into the last fix
 *
 * @param[in] dev Handle
 * @param[out] count Number of satellites used
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if no satellite count has been decoded yet
 */
esp_err_t l86_m33_get_satellites(const l86_m33_t *dev, uint32_t *count);

/**
 * @brief Read the horizontal dilution of precision of the last fix
 *
 * A measure of how well the satellites used are spread across the sky, and so
 * of how much to trust the position: under 2 is good, over 5 means the
 * satellites are bunched together and the position may be off by a long way.
 *
 * @param[in] dev Handle
 * @param[out] hdop Dilution of precision, smaller is better
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if no HDOP has been decoded yet
 */
esp_err_t l86_m33_get_hdop(const l86_m33_t *dev, float *hdop);

/**
 * @brief Read what kind of fix the module reports
 *
 * @param[in] dev Handle
 * @param[out] quality Fix quality, a ::l86_m33_fix_quality_t
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if no fix quality has been decoded yet
 */
esp_err_t l86_m33_get_fix_quality(const l86_m33_t *dev, l86_m33_fix_quality_t *quality);

/**
 * @brief Send a command to the module
 *
 * Takes a PMTK sentence without its checksum, such as `"$PMTK220,1000"`, works
 * out the checksum and sends the command with the `*hh` checksum field and the
 * line ending appended. The command set is described in the Quectel PMTK
 * Protocol Specification; the wrappers below cover the commands worth having a
 * name for.
 *
 * The module answers most commands with a `$PMTK001` acknowledgement sentence,
 * which this driver does not decode.
 *
 * @param[in,out] dev Handle
 * @param[in] command Sentence to send, starting with '$' and without a checksum
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument or a
 *         command longer than ::L86_M33_MAX_COMMAND_LEN, ESP_ERR_INVALID_STATE
 *         if the handle was initialized without a transmit pin, or
 *         ESP_ERR_TIMEOUT if the command could not be written
 */
esp_err_t l86_m33_send_command(l86_m33_t *dev, const char *command);

/**
 * @brief Set the power mode
 *
 * ::L86_M33_POWER_NORMAL is what the module boots into. The AlwaysLocate modes
 * hand the timing over to the module, which then sleeps and wakes on its own
 * depending on how much the position is moving, so position updates stop
 * arriving on a fixed schedule.
 *
 * @param[in,out] dev Handle
 * @param[in] mode Mode to switch to, a ::l86_m33_power_mode_t
 *
 * @return ESP_OK on success, or an error from l86_m33_send_command()
 */
esp_err_t l86_m33_set_power_mode(l86_m33_t *dev, l86_m33_power_mode_t mode);

/**
 * @brief Turn AlwaysLocate on or off
 *
 * Shorthand for l86_m33_set_power_mode() with
 * ::L86_M33_POWER_ALWAYSLOCATE_STANDBY and ::L86_M33_POWER_NORMAL.
 *
 * @param[in,out] dev Handle
 * @param[in] enable true to let the module manage its own duty cycle
 *
 * @return ESP_OK on success, or an error from l86_m33_send_command()
 */
esp_err_t l86_m33_set_always_locate(l86_m33_t *dev, bool enable);

/**
 * @brief Turn multi-tone active interference cancellation on or off
 *
 * Notches out narrowband interference, which is worth having when the module
 * sits next to switching regulators, displays or radios. Off by default.
 *
 * @param[in,out] dev Handle
 * @param[in] enable true to turn it on
 *
 * @return ESP_OK on success, or an error from l86_m33_send_command()
 */
esp_err_t l86_m33_set_multi_tone_aic(l86_m33_t *dev, bool enable);

/**
 * @brief Turn EASY, the module's own orbit prediction, on or off
 *
 * The module predicts the satellite orbits for the next few days and keeps the
 * prediction in backup memory, which shortens the time to first fix after a
 * restart. It needs the backup supply to be present, which on this breakout
 * means the battery, and only works with a 1 Hz fix rate.
 *
 * @param[in,out] dev Handle
 * @param[in] enable true to turn it on
 *
 * @return ESP_OK on success, or an error from l86_m33_send_command()
 */
esp_err_t l86_m33_set_easy(l86_m33_t *dev, bool enable);

/**
 * @brief Set how often the module works out a position
 *
 * Between ::L86_M33_FIX_INTERVAL_MIN_MS and ::L86_M33_FIX_INTERVAL_MAX_MS, 1000
 * by default. A faster fix rate means more NMEA per second, so at the low end
 * either raise the baud rate or cut the sentences down with
 * l86_m33_set_nmea_output().
 *
 * @param[in,out] dev Handle
 * @param[in] interval_ms Interval between position fixes in ms
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on an interval outside the
 *         supported range, or an error from l86_m33_send_command()
 */
esp_err_t l86_m33_set_fix_interval(l86_m33_t *dev, uint32_t interval_ms);

/**
 * @brief Choose which NMEA sentences the module sends
 *
 * @param[in,out] dev Handle
 * @param[in] output Output rate per sentence, see ::l86_m33_nmea_output_t
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument or a rate
 *         above ::L86_M33_NMEA_RATE_MAX, or an error from
 *         l86_m33_send_command()
 */
esp_err_t l86_m33_set_nmea_output(l86_m33_t *dev, const l86_m33_nmea_output_t *output);

/**
 * @brief Restart the module
 *
 * How long it then takes to get a fix again depends on how much assistance data
 * was kept; see ::l86_m33_restart_t. Also clears everything the driver has
 * decoded so far, since it is about to be replaced by fresh data.
 *
 * @param[in,out] dev Handle
 * @param[in] mode How much to throw away, a ::l86_m33_restart_t
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on an unknown mode, or an
 *         error from l86_m33_send_command()
 */
esp_err_t l86_m33_restart(l86_m33_t *dev, l86_m33_restart_t mode);

/**
 * @brief Put the module into standby
 *
 * Stops the receiver and the NMEA output, leaving the module drawing about a
 * milliamp. Any traffic on its RX pin wakes it up again, so
 * l86_m33_send_command() with any command, or l86_m33_wake_up(), brings it
 * back.
 *
 * @param[in,out] dev Handle
 *
 * @return ESP_OK on success, or an error from l86_m33_send_command()
 */
esp_err_t l86_m33_standby(l86_m33_t *dev);

/**
 * @brief Wake the module out of standby
 *
 * Sends a single byte, which is all it takes to wake the module; the byte itself
 * is discarded as an unknown sentence.
 *
 * @param[in,out] dev Handle
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL handle,
 *         ESP_ERR_INVALID_STATE if the handle was initialized without a
 *         transmit pin, or ESP_ERR_TIMEOUT if the byte could not be written
 */
esp_err_t l86_m33_wake_up(l86_m33_t *dev);

/**
 * @brief Work out the great circle distance between two positions
 *
 * Distance over the surface of the earth taken as a sphere, which is within a
 * few parts in a thousand of the real thing.
 *
 * @param[in] lat1 Latitude of the first position in degrees
 * @param[in] lng1 Longitude of the first position in degrees
 * @param[in] lat2 Latitude of the second position in degrees
 * @param[in] lng2 Longitude of the second position in degrees
 *
 * @return Distance in metres
 */
double l86_m33_distance_between(double lat1, double lng1, double lat2, double lng2);

/**
 * @brief Work out the initial course from one position to another
 *
 * The direction to set off in to follow the great circle to the second
 * position. On a long route that course keeps changing as the route is
 * followed, so it is worth recalculating as the position updates.
 *
 * @param[in] lat1 Latitude of the position being travelled from, in degrees
 * @param[in] lng1 Longitude of the position being travelled from, in degrees
 * @param[in] lat2 Latitude of the position being travelled to, in degrees
 * @param[in] lng2 Longitude of the position being travelled to, in degrees
 *
 * @return Course in degrees clockwise from true north
 */
double l86_m33_course_to(double lat1, double lng1, double lat2, double lng2);

/**
 * @brief Turn a course in degrees into a compass point
 *
 * @param[in] course Course in degrees clockwise from true north
 *
 * @return One of the 16 compass points as a static string, "N" through "NNW"
 */
const char *l86_m33_cardinal(double course);

#ifdef __cplusplus
}
#endif
