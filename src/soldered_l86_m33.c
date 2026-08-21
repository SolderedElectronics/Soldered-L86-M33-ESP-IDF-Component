/**
 * @file soldered_l86_m33.c
 * @brief Implementation for the soldered-l86-m33 component
 *
 * The NMEA output and the PMTK command set follow the Quectel L86 GNSS Protocol
 * Specification and the Quectel PMTK Protocol Specification:
 * https://www.quectel.com/product/gnss-l86/
 *
 * @author Soldered Electronics
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soldered_l86_m33.h"

static const char *TAG = "L86_M33";

/* How much is pulled off the UART per read while draining the receive buffer */
#define READ_CHUNK_SIZE 64

/* How long a write is given before it is treated as a failure. Commands are
 * short and the UART driver buffers them, so this only ever runs out if the
 * transmit buffer stays full, which means the port is not being clocked out */
#define WRITE_TIMEOUT_MS 100

// *****************************************************************************
// Section: Small helpers

/**
 * @brief Turn degrees into radians
 *
 * @param[in] degrees Angle in degrees
 *
 * @return Angle in radians
 */
static double to_radians(double degrees)
{
    return degrees * M_PI / 180.0;
}

/**
 * @brief Feed a block of received bytes to the decoder
 *
 * @param[in,out] dev Handle
 * @param[in] data Bytes as they came off the UART
 * @param[in] length How many bytes there are
 *
 * @return How many sentences were decoded out of the block
 */
static uint32_t feed_decoder(l86_m33_t *dev, const uint8_t *data, size_t length)
{
    uint32_t sentences = 0;

    for (size_t i = 0; i < length; i++) {
        if (l86_m33_nmea_encode(&dev->nmea, (char)data[i])) {
            sentences++;
        }
    }

    return sentences;
}

/**
 * @brief Read whatever is waiting on the UART and decode it
 *
 * @param[in,out] dev Handle
 * @param[in] wait_ticks How long a single read may block for
 * @param[out] sentences How many sentences were decoded, may be NULL
 *
 * @return ESP_OK on success, or ESP_FAIL if the UART could not be read
 */
static esp_err_t read_and_decode(l86_m33_t *dev, TickType_t wait_ticks, uint32_t *sentences)
{
    uint8_t buffer[READ_CHUNK_SIZE];
    uint32_t decoded = 0;

    while (1) {
        int length = uart_read_bytes(dev->uart_port, buffer, sizeof(buffer), wait_ticks);
        if (length < 0) {
            ESP_LOGE(TAG, "Could not read UART%d", (int)dev->uart_port);
            return ESP_FAIL;
        }

        decoded += feed_decoder(dev, buffer, (size_t)length);

        /* A short read means the receive buffer has been drained */
        if (length < (int)sizeof(buffer)) {
            break;
        }

        /* Anything already waiting is taken without blocking again */
        wait_ticks = 0;
    }

    if (sentences != NULL) {
        *sentences = decoded;
    }

    return ESP_OK;
}

/**
 * @brief Call the registered callback once for every sentence that was decoded
 *
 * @param[in,out] dev Handle
 * @param[in] sentences How many sentences were decoded
 */
static void dispatch_callback(l86_m33_t *dev, uint32_t sentences)
{
    if (dev->fix_cb == NULL) {
        return;
    }

    for (uint32_t i = 0; i < sentences; i++) {
        dev->fix_cb(dev->fix_arg);
    }
}

/**
 * @brief Read a quantity the module sends as a decimal number
 *
 * @param[in] decimal Decoded quantity
 * @param[out] value Value that was read
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument, or
 *         ESP_ERR_INVALID_STATE if the quantity has not been decoded yet
 */
static esp_err_t read_decimal(const l86_m33_decimal_t *decimal, float *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!decimal->meta.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    *value = decimal->value;

    return ESP_OK;
}

// *****************************************************************************
// Section: Lifetime

esp_err_t l86_m33_init(l86_m33_t *dev, const l86_m33_config_t *config)
{
    if (dev == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->rx_buffer_size < L86_M33_MIN_RX_BUFFER_SIZE) {
        ESP_LOGE(TAG, "Receive buffer of %d bytes is below the minimum of %d", config->rx_buffer_size,
                 L86_M33_MIN_RX_BUFFER_SIZE);
        return ESP_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->uart_port = config->uart_port;
    dev->tx_pin = config->tx_pin;

    uart_config_t uart_cfg = {
        .baud_rate = (int)config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* No transmit ring buffer, so writes go straight out, and no event queue,
     * since the application polls with l86_m33_update() */
    esp_err_t err = uart_driver_install(dev->uart_port, config->rx_buffer_size, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not install the UART driver: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(dev->uart_port, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not configure UART%d: %s", (int)dev->uart_port, esp_err_to_name(err));
        uart_driver_delete(dev->uart_port);
        return err;
    }

    err = uart_set_pin(dev->uart_port, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not route UART%d to the given pins: %s", (int)dev->uart_port, esp_err_to_name(err));
        uart_driver_delete(dev->uart_port);
        return err;
    }

    /* Whatever arrived before the driver was ready is a partial sentence */
    uart_flush_input(dev->uart_port);

    return ESP_OK;
}

esp_err_t l86_m33_deinit(l86_m33_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return uart_driver_delete(dev->uart_port);
}

// *****************************************************************************
// Section: Reading the module

esp_err_t l86_m33_update(l86_m33_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t sentences = 0;
    esp_err_t err = read_and_decode(dev, 0, &sentences);
    if (err != ESP_OK) {
        return err;
    }

    dispatch_callback(dev, sentences);

    return ESP_OK;
}

esp_err_t l86_m33_update_timeout(l86_m33_t *dev, uint32_t duration_ms)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t deadline_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;

    do {
        int64_t left_us = deadline_us - esp_timer_get_time();
        if (left_us < 0) {
            left_us = 0;
        }

        /* pdMS_TO_TICKS() rounds down, so a wait shorter than a tick would turn
         * into a busy loop over the UART. Round up to one tick instead. */
        TickType_t wait_ticks = pdMS_TO_TICKS(left_us / 1000);
        if (wait_ticks == 0) {
            wait_ticks = 1;
        }

        uint32_t sentences = 0;
        esp_err_t err = read_and_decode(dev, wait_ticks, &sentences);
        if (err != ESP_OK) {
            return err;
        }

        dispatch_callback(dev, sentences);
    } while (esp_timer_get_time() < deadline_us);

    return ESP_OK;
}

void l86_m33_set_fix_callback(l86_m33_t *dev, l86_m33_fix_cb_t cb, void *arg)
{
    if (dev == NULL) {
        return;
    }

    dev->fix_cb = cb;
    dev->fix_arg = arg;
}

// *****************************************************************************
// Section: Reading decoded data

bool l86_m33_available(const l86_m33_t *dev)
{
    if (dev == NULL) {
        return false;
    }

    return dev->nmea.location.meta.valid;
}

esp_err_t l86_m33_get_data(const l86_m33_t *dev, l86_m33_data_t *data)
{
    if (dev == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const l86_m33_nmea_t *nmea = &dev->nmea;

    memset(data, 0, sizeof(*data));

    data->location_valid = nmea->location.meta.valid;
    data->latitude = nmea->location.latitude;
    data->longitude = nmea->location.longitude;
    data->location_age_ms = l86_m33_nmea_age_ms(&nmea->location.meta);

    data->date_valid = nmea->date.meta.valid;
    data->year = nmea->date.year;
    data->month = nmea->date.month;
    data->day = nmea->date.day;

    data->time_valid = nmea->time.meta.valid;
    data->hour = nmea->time.hour;
    data->minute = nmea->time.minute;
    data->second = nmea->time.second;
    data->centisecond = nmea->time.centisecond;
    data->datetime_age_ms = l86_m33_nmea_age_ms(&nmea->time.meta);

    data->speed_valid = nmea->speed.meta.valid;
    data->speed_kmph = nmea->speed.value * L86_M33_KNOTS_TO_KMPH;
    data->course_valid = nmea->course.meta.valid;
    data->course_deg = nmea->course.value;

    data->altitude_valid = nmea->altitude.meta.valid;
    data->altitude_m = nmea->altitude.value;

    data->satellites_valid = nmea->satellites.meta.valid;
    data->satellites = nmea->satellites.value;
    data->hdop_valid = nmea->hdop.meta.valid;
    data->hdop = nmea->hdop.value;

    data->fix_quality_valid = nmea->fix_quality.meta.valid;
    data->fix_quality = (l86_m33_fix_quality_t)nmea->fix_quality.value;

    return ESP_OK;
}

esp_err_t l86_m33_get_stats(const l86_m33_t *dev, l86_m33_stats_t *stats)
{
    if (dev == NULL || stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    stats->chars_processed = dev->nmea.chars_processed;
    stats->sentences_with_fix = dev->nmea.sentences_with_fix;
    stats->passed_checksums = dev->nmea.passed_checksums;
    stats->failed_checksums = dev->nmea.failed_checksums;

    return ESP_OK;
}

esp_err_t l86_m33_get_location(const l86_m33_t *dev, double *latitude, double *longitude)
{
    if (dev == NULL || latitude == NULL || longitude == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->nmea.location.meta.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    *latitude = dev->nmea.location.latitude;
    *longitude = dev->nmea.location.longitude;

    return ESP_OK;
}

esp_err_t l86_m33_get_location_age(const l86_m33_t *dev, uint32_t *age_ms)
{
    if (dev == NULL || age_ms == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->nmea.location.meta.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    *age_ms = l86_m33_nmea_age_ms(&dev->nmea.location.meta);

    return ESP_OK;
}

esp_err_t l86_m33_get_date(const l86_m33_t *dev, uint16_t *year, uint8_t *month, uint8_t *day)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->nmea.date.meta.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    if (year != NULL) {
        *year = dev->nmea.date.year;
    }
    if (month != NULL) {
        *month = dev->nmea.date.month;
    }
    if (day != NULL) {
        *day = dev->nmea.date.day;
    }

    return ESP_OK;
}

esp_err_t l86_m33_get_time(const l86_m33_t *dev, uint8_t *hour, uint8_t *minute, uint8_t *second, uint8_t *centisecond)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->nmea.time.meta.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    if (hour != NULL) {
        *hour = dev->nmea.time.hour;
    }
    if (minute != NULL) {
        *minute = dev->nmea.time.minute;
    }
    if (second != NULL) {
        *second = dev->nmea.time.second;
    }
    if (centisecond != NULL) {
        *centisecond = dev->nmea.time.centisecond;
    }

    return ESP_OK;
}

esp_err_t l86_m33_get_datetime_age(const l86_m33_t *dev, uint32_t *age_ms)
{
    if (dev == NULL || age_ms == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->nmea.time.meta.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    *age_ms = l86_m33_nmea_age_ms(&dev->nmea.time.meta);

    return ESP_OK;
}

esp_err_t l86_m33_get_speed_kmph(const l86_m33_t *dev, float *kmph)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = read_decimal(&dev->nmea.speed, kmph);
    if (err == ESP_OK) {
        *kmph *= L86_M33_KNOTS_TO_KMPH;
    }

    return err;
}

esp_err_t l86_m33_get_speed_mps(const l86_m33_t *dev, float *mps)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = read_decimal(&dev->nmea.speed, mps);
    if (err == ESP_OK) {
        *mps *= L86_M33_KNOTS_TO_MPS;
    }

    return err;
}

esp_err_t l86_m33_get_speed_knots(const l86_m33_t *dev, float *knots)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return read_decimal(&dev->nmea.speed, knots);
}

esp_err_t l86_m33_get_course(const l86_m33_t *dev, float *degrees)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return read_decimal(&dev->nmea.course, degrees);
}

esp_err_t l86_m33_get_altitude(const l86_m33_t *dev, float *meters)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return read_decimal(&dev->nmea.altitude, meters);
}

esp_err_t l86_m33_get_satellites(const l86_m33_t *dev, uint32_t *count)
{
    if (dev == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->nmea.satellites.meta.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    *count = dev->nmea.satellites.value;

    return ESP_OK;
}

esp_err_t l86_m33_get_hdop(const l86_m33_t *dev, float *hdop)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return read_decimal(&dev->nmea.hdop, hdop);
}

esp_err_t l86_m33_get_fix_quality(const l86_m33_t *dev, l86_m33_fix_quality_t *quality)
{
    if (dev == NULL || quality == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->nmea.fix_quality.meta.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    *quality = (l86_m33_fix_quality_t)dev->nmea.fix_quality.value;

    return ESP_OK;
}

// *****************************************************************************
// Section: Commands

esp_err_t l86_m33_send_command(l86_m33_t *dev, const char *command)
{
    if (dev == NULL || command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->tx_pin < 0) {
        ESP_LOGE(TAG, "No transmit pin was given, so no command can be sent");
        return ESP_ERR_INVALID_STATE;
    }

    size_t length = strlen(command);
    if (length == 0 || length > L86_M33_MAX_COMMAND_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    /* The checksum covers everything between the '$' that opens the sentence and
     * the '*' that closes the data */
    uint8_t checksum = 0;
    for (size_t i = (command[0] == '$' ? 1 : 0); i < length; i++) {
        checksum ^= (uint8_t)command[i];
    }

    char sentence[L86_M33_MAX_COMMAND_LEN + 6];
    int written = snprintf(sentence, sizeof(sentence), "%s*%02X\r\n", command, checksum);
    if (written < 0 || written >= (int)sizeof(sentence)) {
        return ESP_ERR_INVALID_ARG;
    }

    int sent = uart_write_bytes(dev->uart_port, sentence, (size_t)written);
    if (sent != written) {
        ESP_LOGE(TAG, "Could not write the command to UART%d", (int)dev->uart_port);
        return ESP_FAIL;
    }

    /* Wait for the command to actually leave the port, so that a caller sending
     * a command and then cutting power does not lose it */
    esp_err_t err = uart_wait_tx_done(dev->uart_port, pdMS_TO_TICKS(WRITE_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "The command was not sent within %d ms: %s", WRITE_TIMEOUT_MS, esp_err_to_name(err));
    }

    return err;
}

esp_err_t l86_m33_set_power_mode(l86_m33_t *dev, l86_m33_power_mode_t mode)
{
    char command[32];

    snprintf(command, sizeof(command), "%s,%d", L86_M33_PMTK_SET_PERIODIC, (int)mode);

    return l86_m33_send_command(dev, command);
}

esp_err_t l86_m33_set_always_locate(l86_m33_t *dev, bool enable)
{
    return l86_m33_set_power_mode(dev, enable ? L86_M33_POWER_ALWAYSLOCATE_STANDBY : L86_M33_POWER_NORMAL);
}

esp_err_t l86_m33_set_multi_tone_aic(l86_m33_t *dev, bool enable)
{
    char command[32];

    snprintf(command, sizeof(command), "%s,%d", L86_M33_PMTK_SET_AIC, enable ? 1 : 0);

    return l86_m33_send_command(dev, command);
}

esp_err_t l86_m33_set_easy(l86_m33_t *dev, bool enable)
{
    char command[32];

    /* The first argument tells the module to set rather than to report the
     * setting, the second is what to set it to */
    snprintf(command, sizeof(command), "%s,1,%d", L86_M33_PMTK_EASY, enable ? 1 : 0);

    return l86_m33_send_command(dev, command);
}

esp_err_t l86_m33_set_fix_interval(l86_m33_t *dev, uint32_t interval_ms)
{
    if (interval_ms < L86_M33_FIX_INTERVAL_MIN_MS || interval_ms > L86_M33_FIX_INTERVAL_MAX_MS) {
        ESP_LOGE(TAG, "A fix interval of %lu ms is outside the supported %d to %d ms", (unsigned long)interval_ms,
                 L86_M33_FIX_INTERVAL_MIN_MS, L86_M33_FIX_INTERVAL_MAX_MS);
        return ESP_ERR_INVALID_ARG;
    }

    char command[32];

    snprintf(command, sizeof(command), "%s,%lu", L86_M33_PMTK_SET_FIX_CTL, (unsigned long)interval_ms);

    return l86_m33_send_command(dev, command);
}

esp_err_t l86_m33_set_nmea_output(l86_m33_t *dev, const l86_m33_nmea_output_t *output)
{
    if (output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t rates[] = {output->gll, output->rmc, output->vtg, output->gga, output->gsa, output->gsv, output->zda};
    for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
        if (rates[i] > L86_M33_NMEA_RATE_MAX) {
            ESP_LOGE(TAG, "An output rate of %u is above the maximum of %d", rates[i], L86_M33_NMEA_RATE_MAX);
            return ESP_ERR_INVALID_ARG;
        }
    }

    /* The command takes 19 rates, of which only the ones named in
     * l86_m33_nmea_output_t belong to a sentence this module produces. The rest
     * are reserved and have to be sent as zero. */
    char command[80];

    snprintf(command, sizeof(command), "%s,%u,%u,%u,%u,%u,%u,0,0,0,0,0,0,0,0,0,0,0,%u,0", L86_M33_PMTK_API_SET_NMEA,
             output->gll, output->rmc, output->vtg, output->gga, output->gsa, output->gsv, output->zda);

    return l86_m33_send_command(dev, command);
}

esp_err_t l86_m33_restart(l86_m33_t *dev, l86_m33_restart_t mode)
{
    const char *command = NULL;

    switch (mode) {
    case L86_M33_RESTART_HOT:
        command = L86_M33_PMTK_HOT_START;
        break;
    case L86_M33_RESTART_WARM:
        command = L86_M33_PMTK_WARM_START;
        break;
    case L86_M33_RESTART_COLD:
        command = L86_M33_PMTK_COLD_START;
        break;
    case L86_M33_RESTART_FULL_COLD:
        command = L86_M33_PMTK_FULL_COLD_START;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = l86_m33_send_command(dev, command);
    if (err != ESP_OK) {
        return err;
    }

    /* Everything decoded so far describes the module as it was before the
     * restart, and a half read sentence is about to be cut off by it */
    l86_m33_nmea_reset(&dev->nmea);
    uart_flush_input(dev->uart_port);

    return ESP_OK;
}

esp_err_t l86_m33_standby(l86_m33_t *dev)
{
    return l86_m33_send_command(dev, L86_M33_PMTK_STANDBY);
}

esp_err_t l86_m33_wake_up(l86_m33_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->tx_pin < 0) {
        ESP_LOGE(TAG, "No transmit pin was given, so the module cannot be woken up");
        return ESP_ERR_INVALID_STATE;
    }

    const char wake_byte = '\n';
    if (uart_write_bytes(dev->uart_port, &wake_byte, sizeof(wake_byte)) != sizeof(wake_byte)) {
        ESP_LOGE(TAG, "Could not write to UART%d", (int)dev->uart_port);
        return ESP_FAIL;
    }

    return uart_wait_tx_done(dev->uart_port, pdMS_TO_TICKS(WRITE_TIMEOUT_MS));
}

// *****************************************************************************
// Section: Geography

double l86_m33_distance_between(double lat1, double lng1, double lat2, double lng2)
{
    /* Haversine, which stays accurate for two positions close together where the
     * plain spherical law of cosines loses precision */
    double delta_lat = to_radians(lat2 - lat1);
    double delta_lng = to_radians(lng2 - lng1);

    double sin_half_lat = sin(delta_lat / 2.0);
    double sin_half_lng = sin(delta_lng / 2.0);

    double a = sin_half_lat * sin_half_lat +
               cos(to_radians(lat1)) * cos(to_radians(lat2)) * sin_half_lng * sin_half_lng;

    return 2.0 * atan2(sqrt(a), sqrt(1.0 - a)) * L86_M33_EARTH_RADIUS_M;
}

double l86_m33_course_to(double lat1, double lng1, double lat2, double lng2)
{
    double delta_lng = to_radians(lng2 - lng1);
    double lat1_rad = to_radians(lat1);
    double lat2_rad = to_radians(lat2);

    double y = sin(delta_lng) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lng);

    double course = atan2(y, x) * 180.0 / M_PI;

    /* atan2() gives -180 to 180, courses run 0 to 360 */
    if (course < 0.0) {
        course += 360.0;
    }

    return course;
}

const char *l86_m33_cardinal(double course)
{
    static const char *directions[] = {"N",  "NNE", "NE", "ENE", "E",  "ESE", "SE", "SSE",
                                       "S",  "SSW", "SW", "WSW", "W",  "WNW", "NW", "NNW"
                                      };

    /* Each point covers 22.5 degrees, centred on its own direction, so half a
     * step is added before rounding down */
    int index = (int)((course + 11.25) / 22.5) % 16;
    if (index < 0) {
        index += 16;
    }

    return directions[index];
}
