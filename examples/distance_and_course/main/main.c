/**
 * @file main.c
 * @brief Example working out the distance and course from here to somewhere else
 *
 * Prints how far away Osijek is and which way to set off to get there, worked
 * out from the position the module reports. Change the two coordinates below to
 * aim at somewhere else.
 *
 * The same two calculations are what a geofence is built out of: watch the
 * distance to a fixed position and act when it crosses a threshold.
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

static const char *TAG = "DISTANCE_AND_COURSE";

// Change these to match how your breakout is wired
#define PIN_NUM_RX GPIO_NUM_16 // Goes to the TX pin of the module
#define PIN_NUM_TX GPIO_NUM_17 // Goes to the RX pin of the module

// Where we are headed, in this case Osijek, Croatia
#define TARGET_NAME "Osijek"
#define TARGET_LAT  45.5550
#define TARGET_LON  18.6955

// Print how far away the target is once a second
#define PRINT_INTERVAL_MS 1000

void app_main(void)
{
    l86_m33_t gnss;
    l86_m33_config_t config = L86_M33_DEFAULT_CONFIG(PIN_NUM_RX, PIN_NUM_TX);

    ESP_ERROR_CHECK(l86_m33_init(&gnss, &config));

    ESP_LOGI(TAG, "Waiting for a fix, this can take a while on a cold start");

    while (1) {
        ESP_ERROR_CHECK(l86_m33_update_timeout(&gnss, PRINT_INTERVAL_MS));

        double latitude, longitude;
        if (l86_m33_get_location(&gnss, &latitude, &longitude) != ESP_OK) {
            continue;
        }

        double distance_m = l86_m33_distance_between(latitude, longitude, TARGET_LAT, TARGET_LON);
        double course = l86_m33_course_to(latitude, longitude, TARGET_LAT, TARGET_LON);

        ESP_LOGI(TAG, "%s is %.1f km away, head %.1f degrees (%s)", TARGET_NAME, distance_m / 1000.0, course,
                 l86_m33_cardinal(course));

        // Which way we are actually moving, which only means anything while
        // moving fast enough for the noise in the position not to swamp it
        float speed_kmph, heading;
        if (l86_m33_get_speed_kmph(&gnss, &speed_kmph) == ESP_OK && l86_m33_get_course(&gnss, &heading) == ESP_OK) {
            ESP_LOGI(TAG, "Currently going %.1f km/h, heading %.1f degrees (%s)", speed_kmph, heading,
                     l86_m33_cardinal(heading));
        }
    }
}
