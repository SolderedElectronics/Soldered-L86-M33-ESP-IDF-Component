# Soldered GNSS GPS L86-M33 Breakout Component

| ![GNSS GPS L86-M33 Breakout](https://cms.soldered.com/products/333201/media/333201_featured-photo_38c8b2.jpg) |
| :----------------------------------------------------------------------------------------------------------: |
|                        [GNSS GPS L86-M33 Breakout](https://solde.red/333201)                                 |

ESP-IDF driver for the Soldered GNSS GPS L86-M33 Breakout, built around the Quectel L86-M33 GNSS module with its patch antenna on top. Tracks GPS, GLONASS and Galileo, gets a first fix in about 15 s from a warm start and a position update up to ten times a second, and reports position, altitude, speed, course and satellite time over UART.

### Repository Contents

- **/src** - source files (.c)
  - `soldered_l86_m33.c` - the driver
  - `l86_m33_nmea.c` - the NMEA sentence decoder
- **/include** - header files (.h)
  - `soldered_l86_m33.h` - the public API
  - `l86_m33_dfs.h` - NMEA and PMTK definitions and enumerations
  - `l86_m33_nmea.h` - decoder state, used through the API above
- **/examples** - examples for using the library
  - `basic_readings` - read the position and the UTC date and time
  - `full_data` - print everything the module reports as a table, driven by a callback
  - `advanced_features` - interference cancellation, orbit prediction, fix rate, sentence filtering and AlwaysLocate
  - `distance_and_course` - work out how far away somewhere else is and which way it is
- **_other_** - idf_component.yml manifest file for ESP Component Registry

### Usage

The module sends NMEA sentences as soon as it is powered, so opening the UART is all the setup it needs. Wire the TX pin of the module to a receive pin on the ESP32, and its RX pin to a transmit pin if commands are going to be sent:

```c
l86_m33_t gnss;
l86_m33_config_t config = L86_M33_DEFAULT_CONFIG(GPIO_NUM_16, GPIO_NUM_17);
ESP_ERROR_CHECK(l86_m33_init(&gnss, &config));

while (1) {
    // Reads and decodes for 500 ms instead of sleeping through them
    ESP_ERROR_CHECK(l86_m33_update_timeout(&gnss, 500));

    double latitude, longitude;
    if (l86_m33_get_location(&gnss, &latitude, &longitude) == ESP_OK) {
        printf("%.6f, %.6f\n", latitude, longitude);
    }
}
```

**Feeding the decoder:** the module keeps talking whether or not anybody is listening, so `l86_m33_update()` has to be called often enough that the UART receive buffer never overflows - at the default fix rate a few times a second. In a loop that has nothing else to do, `l86_m33_update_timeout()` replaces a plain delay and keeps decoding for the whole wait. A callback registered with `l86_m33_set_fix_callback()` then fires from inside those calls, once per decoded sentence.

**Reading data:** every getter returns `ESP_ERR_INVALID_STATE` until the module has actually reported that quantity, so a fresh start reads as no data rather than as zeroes. `l86_m33_get_data()` reads everything at once into an `l86_m33_data_t` snapshot with a validity flag per field, which is the easier route when several values are used together. Position, date, time, speed and course are only ever updated from a sentence that reported a fix; the satellite count, HDOP and fix quality are updated either way, which is what makes it possible to tell a module that is still searching from one that is not talking at all. `l86_m33_get_location_age()` tells a fresh position from the last known one after the fix was lost, and `l86_m33_get_stats()` returns the character and checksum counters, which is where a wiring or baud rate problem shows up.

**Getting a fix:** the module needs a clear view of the sky, so it belongs outdoors or at least at a window, and the first fix after a cold start can take a minute or more. The patch antenna on the board is all it needs - connecting an external antenna as well makes fixes take much longer. The battery holder backs up the clock and the assistance data so that later starts are warm ones.

**Configuration:** `l86_m33_set_fix_interval()` sets how often a position is worked out, from 100 ms to 10 s, and `l86_m33_set_nmea_output()` sets which sentences come out and how often, which is what keeps the UART from being flooded at a fast fix rate. `l86_m33_set_multi_tone_aic()` notches out narrowband interference from nearby regulators, displays and radios, and `l86_m33_set_easy()` has the module predict the satellite orbits itself to shorten the time to first fix. Anything else out of the PMTK command set is sent with `l86_m33_send_command()`, which takes the sentence without its checksum and appends the one it works out:

```c
ESP_ERROR_CHECK(l86_m33_send_command(&gnss, "$PMTK605"));    // asks for the firmware version
```

**Power:** `l86_m33_set_always_locate()` hands the duty cycling over to the module, which then sleeps and wakes on its own depending on how much the position is moving; updates stop arriving on a fixed schedule, so watch the age of the position rather than expecting a new one. `l86_m33_standby()` stops the receiver altogether, and `l86_m33_wake_up()` brings it back.

**Geography:** `l86_m33_distance_between()` and `l86_m33_course_to()` work out the great circle distance and initial course between two positions, and `l86_m33_cardinal()` turns a course into a compass point. Watching the distance to a fixed position is all a geofence needs.

This component drives the module over UART, which is how the breakout above is wired. The easyC version of the board, [www.solde.red/333213](https://solde.red/333213), answers on I2C instead and is not covered here.

### Original source

This is a port of the [Soldered GNSS L86-M33 Arduino library](https://github.com/SolderedElectronics/Soldered-GNSS-L86-M33-Arduino-Library), whose NMEA parsing comes from [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) by Mikal Hart, which the decoder here follows. Thank you, Mikal Hart.

### Hardware design

You can find hardware design for this board in _GNSS GPS L86-M33 Breakout_ hardware repository.

### Documentation

Access library documentation [here](https://docs.soldered.com/).

### About Soldered

<img src="https://raw.githubusercontent.com/SolderedElectronics/Soldered-Generic-Arduino-Library/dev/extras/Soldered-logo-color.png" alt="soldered-logo" width="500"/>

At Soldered, we design and manufacture a wide selection of electronic products to help you turn your ideas into acts and bring you one step closer to your final project. Our products are intented for makers and crafted in-house by our experienced team in Osijek, Croatia. We believe that sharing is a crucial element for improvement and innovation, and we work hard to stay connected with all our makers regardless of their skill or experience level. Therefore, all our products are open-source. Finally, we always have your back. If you face any problem concerning either your shopping experience or your electronics project, our team will help you deal with it, offering efficient customer service and cost-free technical support anytime. Some of those might be useful for you:

- [Web Store](https://www.soldered.com/shop)
- [Tutorials & Projects](https://soldered.com/learn)
- [Documentation](https://docs.soldered.com)

### Open-source license

Soldered invests vast amounts of time into hardware & software for these products, which are all open-source. Please support future development by buying one of our products.

Check license details in the LICENSE file. Long story short, use these open-source files for any purpose you want to, as long as you apply the same open-source licence to it and disclose the original source. No warranty - all designs in this repository are distributed in the hope that they will be useful, but without any warranty. They are provided "AS IS", therefore without warranty of any kind, either expressed or implied. The entire quality and performance of what you do with the contents of this repository are your responsibility. In no event, Soldered (TAVU) will be liable for your damages, losses, including any general, special, incidental or consequential damage arising out of the use or inability to use the contents of this repository.

## Have fun!

And thank you from your fellow makers at Soldered Electronics.
