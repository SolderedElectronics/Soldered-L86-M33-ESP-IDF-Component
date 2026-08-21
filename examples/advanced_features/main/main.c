/**
 * @file main.c
 * @brief Example for the configuration commands the module accepts
 *
 * Sets up multi-tone interference cancellation, EASY orbit prediction, a 2 Hz
 * fix rate and an NMEA output cut down to just the two sentences the driver
 * decodes, then sends one command by hand to show how anything else out of the
 * PMTK command set is sent. Afterwards it prints the position twice a second and
 * switches the module to AlwaysLocate.
 *
 * All of these need the TX pin of the ESP32 wired to the RX pin of the module,
 * which the position readings themselves do not.
 *
 * The full command set is in the Quectel PMTK Protocol Specification.
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

static const char *TAG = "ADVANCED_FEATURES";

// Change these to match how your breakout is wired
#define PIN_NUM_RX GPIO_NUM_16 // Goes to the TX pin of the module
#define PIN_NUM_TX GPIO_NUM_17 // Goes to the RX pin of the module

// Twice a second, up from the 1 Hz the module boots with
#define FIX_INTERVAL_MS 500

// How long to run at the settings above before switching to AlwaysLocate
#define NORMAL_MODE_S 30

void app_main(void)
{
    l86_m33_t gnss;
    l86_m33_config_t config = L86_M33_DEFAULT_CONFIG(PIN_NUM_RX, PIN_NUM_TX);

    ESP_ERROR_CHECK(l86_m33_init(&gnss, &config));

    // Notch out narrowband interference, worth having when the module sits next
    // to switching regulators, displays or radios
    ESP_ERROR_CHECK(l86_m33_set_multi_tone_aic(&gnss, true));

    // Let the module predict the satellite orbits for the next few days and keep
    // the prediction in backup memory, which shortens the time to first fix
    // after a restart. Needs the battery and only works at a 1 Hz fix rate, so
    // it is set before the fix rate is raised below.
    ESP_ERROR_CHECK(l86_m33_set_easy(&gnss, true));

    // Only the two sentences this driver decodes are of any use here, and
    // turning off the rest keeps the UART from being flooded at a faster fix
    // rate
    l86_m33_nmea_output_t output = L86_M33_NMEA_OUTPUT_MINIMAL();
    ESP_ERROR_CHECK(l86_m33_set_nmea_output(&gnss, &output));

    ESP_ERROR_CHECK(l86_m33_set_fix_interval(&gnss, FIX_INTERVAL_MS));

    // Anything without a wrapper is sent as a PMTK sentence without its
    // checksum, which the driver works out and appends. This one asks the module
    // to report its firmware version, which it answers with a $PMTK705 sentence.
    ESP_ERROR_CHECK(l86_m33_send_command(&gnss, "$PMTK605"));

    ESP_LOGI(TAG, "Module configured, reading position for %d s", NORMAL_MODE_S);

    for (int i = 0; i < NORMAL_MODE_S * 2; i++) {
        ESP_ERROR_CHECK(l86_m33_update_timeout(&gnss, 500));

        double latitude, longitude;
        if (l86_m33_get_location(&gnss, &latitude, &longitude) == ESP_OK) {
            uint32_t age_ms = 0;
            ESP_ERROR_CHECK(l86_m33_get_location_age(&gnss, &age_ms));
            ESP_LOGI(TAG, "Location: %.6f, %.6f (%lu ms old)", latitude, longitude, (unsigned long)age_ms);
        } else {
            ESP_LOGI(TAG, "Location: no fix yet");
        }
    }

    // In AlwaysLocate the module decides for itself when to sleep and when to
    // wake up, based on how much the position is moving. Average power draw
    // drops a long way, and in exchange the position is only refreshed when the
    // module feels like it, so the age of the last position is worth watching.
    ESP_LOGI(TAG, "Switching to AlwaysLocate");
    ESP_ERROR_CHECK(l86_m33_set_always_locate(&gnss, true));

    while (1) {
        ESP_ERROR_CHECK(l86_m33_update_timeout(&gnss, 1000));

        double latitude, longitude;
        uint32_t age_ms;
        if (l86_m33_get_location(&gnss, &latitude, &longitude) == ESP_OK &&
                l86_m33_get_location_age(&gnss, &age_ms) == ESP_OK) {
            ESP_LOGI(TAG, "Location: %.6f, %.6f (%lu ms old)", latitude, longitude, (unsigned long)age_ms);
        } else {
            ESP_LOGI(TAG, "Location: no fix yet");
        }
    }
}
