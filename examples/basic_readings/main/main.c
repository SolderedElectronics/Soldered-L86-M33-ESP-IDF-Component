/**
 * @file main.c
 * @brief Example for reading position, date and time off the module
 *
 * Prints the position and the UTC date and time twice a second. Everything
 * reads as invalid until the module finds enough satellites, which outdoors
 * takes from a second on a hot start up to a minute or so the first time.
 *
 * For best results the module needs a clear view of the sky, so it belongs
 * outside or at least at a window. The onboard antenna is enough on its own;
 * connecting an external one as well makes the fix take much longer. The battery
 * only backs up the clock and the assistance data.
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

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soldered_l86_m33.h"

static const char *TAG = "BASIC_READINGS";

// Change these to match how your breakout is wired
#define PIN_NUM_RX GPIO_NUM_6 // Goes to the TX pin of the module
#define PIN_NUM_TX GPIO_NUM_10 // Goes to the RX pin of the module

// How long to keep reading the module between printouts
#define PRINT_INTERVAL_MS 500

// If nothing has been received by now, the module is not talking to us
#define WIRING_TIMEOUT_MS 5000

void app_main(void)
{
    l86_m33_t gnss;
    l86_m33_config_t config = L86_M33_DEFAULT_CONFIG(PIN_NUM_RX, PIN_NUM_TX);

    // The module starts sending NMEA as soon as it is powered, so opening the
    // UART is all the setup it needs
    ESP_ERROR_CHECK(l86_m33_init(&gnss, &config));

    bool wiring_checked = false;

    while (1) {
        // Reading and decoding for the whole interval instead of sleeping
        // through it keeps the receive buffer from filling up
        ESP_ERROR_CHECK(l86_m33_update_timeout(&gnss, PRINT_INTERVAL_MS));

        double latitude, longitude;
        if (l86_m33_get_location(&gnss, &latitude, &longitude) == ESP_OK) {
            ESP_LOGI(TAG, "Location: %.6f, %.6f", latitude, longitude);
        } else {
            ESP_LOGI(TAG, "Location: no fix yet");
        }

        uint16_t year;
        uint8_t month, day, hour, minute, second;
        if (l86_m33_get_date(&gnss, &year, &month, &day) == ESP_OK &&
                l86_m33_get_time(&gnss, &hour, &minute, &second, NULL) == ESP_OK) {
            ESP_LOGI(TAG, "Date and time: %04u-%02u-%02u %02u:%02u:%02u UTC", year, month, day, hour, minute, second);
        } else {
            ESP_LOGI(TAG, "Date and time: not known yet");
        }

        // Characters arriving with no fix means the module is fine and still
        // looking; no characters at all means it is not being heard
        if (!wiring_checked) {
            l86_m33_stats_t stats;
            ESP_ERROR_CHECK(l86_m33_get_stats(&gnss, &stats));

            if (stats.chars_processed >= 10) {
                wiring_checked = true;
            } else if (esp_log_timestamp() > WIRING_TIMEOUT_MS) {
                ESP_LOGE(TAG, "Nothing received from the module, check the wiring and the baud rate");
                wiring_checked = true;
            }
        }
    }
}
