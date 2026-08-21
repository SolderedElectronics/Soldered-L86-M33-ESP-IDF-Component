/**
 * @file main.c
 * @brief Example printing everything the module reports, in one table
 *
 * Prints a row a second holding the position, the fix quality, the number of
 * satellites, the dilution of precision, the date and time, the altitude, the
 * course and the speed, followed by the counters describing what has come in
 * over the UART. A column full of dashes is a value the module has not reported
 * yet.
 *
 * Callbacks are used here rather than polling: the row is printed out of the
 * callback the driver invokes for every decoded sentence, throttled to one row a
 * second.
 *
 * For best results the module needs a clear view of the sky, so it belongs
 * outside or at least at a window.
 *
 * Product used is www.solde.red/333201
 *
 * Connections:
 *   L86-M33      ESP32
 *   TX           GPIO16
 *   RX           GPIO17
 *   5V           5V
 *   GND          GND
 *
 * @author Soldered Electronics
 */

#include <inttypes.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soldered_l86_m33.h"

static const char *TAG = "FULL_DATA";

// Change these to match how your breakout is wired
#define PIN_NUM_RX GPIO_NUM_16 // Goes to the TX pin of the module
#define PIN_NUM_TX GPIO_NUM_17 // Goes to the RX pin of the module

// At most one row per second, however many sentences arrive in between
#define ROW_INTERVAL_US 1000000

/**
 * @brief Turn a fix quality into something readable
 */
static const char *fix_quality_name(l86_m33_fix_quality_t quality)
{
    switch (quality) {
    case L86_M33_FIX_GNSS:
        return "GNSS";
    case L86_M33_FIX_DGNSS:
        return "DGNSS";
    case L86_M33_FIX_ESTIMATED:
        return "EST";
    default:
        return "none";
    }
}

/**
 * @brief Print one row of everything the module has reported
 *
 * Invoked by the driver for every decoded sentence, so the interval between
 * rows is enforced here.
 *
 * @param[in] arg The handle, as handed to l86_m33_set_fix_callback()
 */
static void print_row(void *arg)
{
    static int64_t last_row_us = 0;

    l86_m33_t *gnss = (l86_m33_t *)arg;

    int64_t now_us = esp_timer_get_time();
    if (now_us - last_row_us < ROW_INTERVAL_US) {
        return;
    }
    last_row_us = now_us;

    // A snapshot, so every column on the row belongs to the same moment
    l86_m33_data_t data;
    ESP_ERROR_CHECK(l86_m33_get_data(gnss, &data));

    l86_m33_stats_t stats;
    ESP_ERROR_CHECK(l86_m33_get_stats(gnss, &stats));

    if (data.location_valid) {
        printf("%11.6f %11.6f %6" PRIu32 " ", data.latitude, data.longitude, data.location_age_ms);
    } else {
        printf("%11s %11s %6s ", "---", "---", "---");
    }

    printf("%6s ", data.fix_quality_valid ? fix_quality_name(data.fix_quality) : "---");

    if (data.satellites_valid) {
        printf("%4" PRIu32 " ", data.satellites);
    } else {
        printf("%4s ", "---");
    }

    if (data.hdop_valid) {
        printf("%5.1f ", data.hdop);
    } else {
        printf("%5s ", "---");
    }

    if (data.date_valid && data.time_valid) {
        printf("%04u-%02u-%02u %02u:%02u:%02u ", data.year, data.month, data.day, data.hour, data.minute, data.second);
    } else {
        printf("%10s %8s ", "---", "---");
    }

    if (data.altitude_valid) {
        printf("%8.1f ", data.altitude_m);
    } else {
        printf("%8s ", "---");
    }

    if (data.course_valid) {
        printf("%7.1f %4s ", data.course_deg, l86_m33_cardinal(data.course_deg));
    } else {
        printf("%7s %4s ", "---", "---");
    }

    if (data.speed_valid) {
        printf("%7.2f ", data.speed_kmph);
    } else {
        printf("%7s ", "---");
    }

    printf("%8" PRIu32 " %8" PRIu32 " %8" PRIu32 "\n", stats.chars_processed, stats.sentences_with_fix,
           stats.failed_checksums);
}

void app_main(void)
{
    static l86_m33_t gnss;
    l86_m33_config_t config = L86_M33_DEFAULT_CONFIG(PIN_NUM_RX, PIN_NUM_TX);

    ESP_ERROR_CHECK(l86_m33_init(&gnss, &config));

    // The callback runs from l86_m33_update_timeout() below, in this task, so it
    // is free to do as much work as printing a whole row
    l86_m33_set_fix_callback(&gnss, print_row, &gnss);

    ESP_LOGI(TAG, "Waiting for the module, rows appear once it starts talking");

    printf("\n   Latitude   Longitude    Age    Fix  Sats  HDOP       Date     Time      Alt  Course  Crd   "
           "Speed    Chars      Fix Checksum\n");
    printf("      (deg)       (deg)   (ms)                                          UTC      (m)   (deg)      "
           "(km/h)       RX    Sents     Fail\n");
    printf("-------------------------------------------------------------------------------------------------"
           "-----------------------------------\n");

    while (1) {
        // Everything is printed from the callback, so all this loop has to do is
        // keep feeding the decoder
        ESP_ERROR_CHECK(l86_m33_update_timeout(&gnss, 200));
    }
}
