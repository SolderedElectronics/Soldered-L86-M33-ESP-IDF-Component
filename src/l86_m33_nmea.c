/**
 * @file l86_m33_nmea.c
 * @brief NMEA sentence decoder for the soldered-l86-m33 component
 *
 * Sentence layout follows the Quectel L86 GNSS Protocol Specification, which in
 * turn follows NMEA 0183.
 *
 * @author Soldered Electronics
 */

#include <stdlib.h>
#include <string.h>
#include "esp_timer.h"
#include "l86_m33_nmea.h"

/* Which quantities a sentence has staged a value for, so that only those get
 * committed when its checksum turns out to be correct */
#define STAGED_LOCATION    (1U << 0)
#define STAGED_DATE        (1U << 1)
#define STAGED_TIME        (1U << 2)
#define STAGED_SPEED       (1U << 3)
#define STAGED_COURSE      (1U << 4)
#define STAGED_ALTITUDE    (1U << 5)
#define STAGED_HDOP        (1U << 6)
#define STAGED_SATELLITES  (1U << 7)
#define STAGED_FIX_QUALITY (1U << 8)

/* Position, date, time, speed and course only mean something when the sentence
 * they came out of reported a fix. The module keeps sending the rest either way,
 * so those are committed regardless. */
#define STAGED_NEEDS_FIX (STAGED_LOCATION | STAGED_DATE | STAGED_TIME | STAGED_SPEED | STAGED_COURSE)

// *****************************************************************************
// Section: Field parsing

/**
 * @brief Turn one hex digit into its value
 *
 * @param[in] c Hex digit, upper or lower case
 *
 * @return Value of the digit, or 0 for anything that is not a hex digit
 */
static uint8_t hex_to_int(char c)
{
    if (c >= '0' && c <= '9') {
        return (uint8_t)(c - '0');
    }
    if (c >= 'A' && c <= 'F') {
        return (uint8_t)(c - 'A' + 10);
    }
    if (c >= 'a' && c <= 'f') {
        return (uint8_t)(c - 'a' + 10);
    }

    return 0;
}

/**
 * @brief Turn a latitude or longitude field into degrees
 *
 * NMEA sends these as degrees and minutes run together with no separator,
 * `ddmm.mmmm` for a latitude and `dddmm.mmmm` for a longitude, so the split
 * between the two is found from where the decimal point sits rather than from a
 * fixed offset.
 *
 * @param[in] term Field to parse
 *
 * @return Position in degrees, always positive; the hemisphere is a separate field
 */
static double parse_degrees(const char *term)
{
    const char *point = strchr(term, '.');
    if (point == NULL) {
        point = term + strlen(term);
    }

    /* Two digits of whole minutes sit in front of the decimal point, and
     * everything before those is degrees */
    if (point - term < 2) {
        return 0.0;
    }

    char whole_degrees[4] = {0};
    size_t length = (size_t)(point - term) - 2;
    if (length >= sizeof(whole_degrees)) {
        length = sizeof(whole_degrees) - 1;
    }
    memcpy(whole_degrees, term, length);

    return atof(whole_degrees) + atof(point - 2) / 60.0;
}

/**
 * @brief Turn a time of day field into hhmmsscc
 *
 * The field is `hhmmss.sss`, with the fractional part rounded down to
 * centiseconds since that is as fine as the module ever reports.
 *
 * @param[in] term Field to parse
 *
 * @return Time as a single number, hours in the highest two digits
 */
static uint32_t parse_time(const char *term)
{
    uint32_t whole = (uint32_t)strtoul(term, NULL, 10);
    uint32_t centiseconds = 0;

    const char *point = strchr(term, '.');
    if (point != NULL) {
        if (point[1] >= '0' && point[1] <= '9') {
            centiseconds = (uint32_t)(point[1] - '0') * 10;
            if (point[2] >= '0' && point[2] <= '9') {
                centiseconds += (uint32_t)(point[2] - '0');
            }
        }
    }

    return whole * 100 + centiseconds;
}

// *****************************************************************************
// Section: Sentence handling

/**
 * @brief Work out which sentence the field at position 0 belongs to
 *
 * The field holds the talker followed by the sentence type, and the talker
 * changes with which constellations went into the fix, so only the last three
 * characters are looked at.
 *
 * @param[in] term Field to look at
 *
 * @return Sentence type, ::L86_M33_SENTENCE_OTHER for anything not decoded here
 */
static l86_m33_sentence_t sentence_type(const char *term)
{
    size_t length = strlen(term);
    if (length < 5) {
        return L86_M33_SENTENCE_OTHER;
    }

    const char *type = term + length - 3;
    if (strcmp(type, "GGA") == 0) {
        return L86_M33_SENTENCE_GGA;
    }
    if (strcmp(type, "RMC") == 0) {
        return L86_M33_SENTENCE_RMC;
    }

    return L86_M33_SENTENCE_OTHER;
}

/**
 * @brief Take one field apart and stage whatever it holds
 *
 * Empty fields, which is what the module sends for everything it does not know
 * yet, stage nothing and so leave the last known value alone.
 *
 * @param[in,out] nmea Decoder state
 */
static void parse_term(l86_m33_nmea_t *nmea)
{
    if (nmea->term_number == 0) {
        nmea->sentence_type = sentence_type(nmea->term);
        return;
    }

    if (nmea->sentence_type == L86_M33_SENTENCE_OTHER || nmea->term[0] == '\0') {
        return;
    }

    if (nmea->sentence_type == L86_M33_SENTENCE_GGA) {
        switch (nmea->term_number) {
        case 1: /* UTC time of day */
            nmea->time.staged = parse_time(nmea->term);
            nmea->staged_mask |= STAGED_TIME;
            break;
        case 2: /* Latitude */
            nmea->location.latitude_staged = parse_degrees(nmea->term);
            nmea->staged_mask |= STAGED_LOCATION;
            break;
        case 3: /* Hemisphere the latitude is in */
            if (nmea->term[0] == 'S') {
                nmea->location.latitude_staged = -nmea->location.latitude_staged;
            }
            break;
        case 4: /* Longitude */
            nmea->location.longitude_staged = parse_degrees(nmea->term);
            nmea->staged_mask |= STAGED_LOCATION;
            break;
        case 5: /* Hemisphere the longitude is in */
            if (nmea->term[0] == 'W') {
                nmea->location.longitude_staged = -nmea->location.longitude_staged;
            }
            break;
        case 6: /* Fix quality, 0 when the module has no position */
            nmea->fix_quality.staged = (uint32_t)strtoul(nmea->term, NULL, 10);
            nmea->staged_mask |= STAGED_FIX_QUALITY;
            nmea->sentence_has_fix = (nmea->fix_quality.staged != L86_M33_FIX_NONE);
            break;
        case 7: /* Satellites used for the fix */
            nmea->satellites.staged = (uint32_t)strtoul(nmea->term, NULL, 10);
            nmea->staged_mask |= STAGED_SATELLITES;
            break;
        case 8: /* Horizontal dilution of precision */
            nmea->hdop.staged = strtof(nmea->term, NULL);
            nmea->staged_mask |= STAGED_HDOP;
            break;
        case 9: /* Altitude above mean sea level, in metres */
            nmea->altitude.staged = strtof(nmea->term, NULL);
            nmea->staged_mask |= STAGED_ALTITUDE;
            break;
        default:
            break;
        }

        return;
    }

    switch (nmea->term_number) {
    case 1: /* UTC time of day */
        nmea->time.staged = parse_time(nmea->term);
        nmea->staged_mask |= STAGED_TIME;
        break;
    case 2: /* Status, 'A' for a usable position and 'V' for a warning */
        nmea->sentence_has_fix = (nmea->term[0] == 'A');
        break;
    case 3: /* Latitude */
        nmea->location.latitude_staged = parse_degrees(nmea->term);
        nmea->staged_mask |= STAGED_LOCATION;
        break;
    case 4: /* Hemisphere the latitude is in */
        if (nmea->term[0] == 'S') {
            nmea->location.latitude_staged = -nmea->location.latitude_staged;
        }
        break;
    case 5: /* Longitude */
        nmea->location.longitude_staged = parse_degrees(nmea->term);
        nmea->staged_mask |= STAGED_LOCATION;
        break;
    case 6: /* Hemisphere the longitude is in */
        if (nmea->term[0] == 'W') {
            nmea->location.longitude_staged = -nmea->location.longitude_staged;
        }
        break;
    case 7: /* Speed over ground, in knots */
        nmea->speed.staged = strtof(nmea->term, NULL);
        nmea->staged_mask |= STAGED_SPEED;
        break;
    case 8: /* Course over ground, in degrees from true north */
        nmea->course.staged = strtof(nmea->term, NULL);
        nmea->staged_mask |= STAGED_COURSE;
        break;
    case 9: /* UTC date */
        nmea->date.staged = (uint32_t)strtoul(nmea->term, NULL, 10);
        nmea->staged_mask |= STAGED_DATE;
        break;
    default:
        break;
    }
}

/**
 * @brief Mark a quantity as decoded, as of now
 *
 * @param[out] meta State of the quantity
 * @param[in] now_us Timestamp to record as the moment of the update
 */
static void commit_meta(l86_m33_meta_t *meta, int64_t now_us)
{
    meta->valid = true;
    meta->commit_us = now_us;
}

/**
 * @brief Copy the values staged by a verified sentence into place
 *
 * @param[in,out] nmea Decoder state
 */
static void commit_sentence(l86_m33_nmea_t *nmea)
{
    int64_t now_us = esp_timer_get_time();
    uint16_t mask = nmea->staged_mask;

    /* A sentence without a fix carries no position, and the time it carries
     * comes out of the module's own clock rather than from the satellites */
    if (!nmea->sentence_has_fix) {
        mask &= (uint16_t)~STAGED_NEEDS_FIX;
    }

    if (mask & STAGED_LOCATION) {
        nmea->location.latitude = nmea->location.latitude_staged;
        nmea->location.longitude = nmea->location.longitude_staged;
        commit_meta(&nmea->location.meta, now_us);
    }

    if (mask & STAGED_DATE) {
        nmea->date.day = (uint8_t)(nmea->date.staged / 10000);
        nmea->date.month = (uint8_t)((nmea->date.staged / 100) % 100);
        nmea->date.year = (uint16_t)(2000 + nmea->date.staged % 100);
        commit_meta(&nmea->date.meta, now_us);
    }

    if (mask & STAGED_TIME) {
        nmea->time.hour = (uint8_t)(nmea->time.staged / 1000000);
        nmea->time.minute = (uint8_t)((nmea->time.staged / 10000) % 100);
        nmea->time.second = (uint8_t)((nmea->time.staged / 100) % 100);
        nmea->time.centisecond = (uint8_t)(nmea->time.staged % 100);
        commit_meta(&nmea->time.meta, now_us);
    }

    if (mask & STAGED_SPEED) {
        nmea->speed.value = nmea->speed.staged;
        commit_meta(&nmea->speed.meta, now_us);
    }

    if (mask & STAGED_COURSE) {
        nmea->course.value = nmea->course.staged;
        commit_meta(&nmea->course.meta, now_us);
    }

    if (mask & STAGED_ALTITUDE) {
        nmea->altitude.value = nmea->altitude.staged;
        commit_meta(&nmea->altitude.meta, now_us);
    }

    if (mask & STAGED_HDOP) {
        nmea->hdop.value = nmea->hdop.staged;
        commit_meta(&nmea->hdop.meta, now_us);
    }

    if (mask & STAGED_SATELLITES) {
        nmea->satellites.value = nmea->satellites.staged;
        commit_meta(&nmea->satellites.meta, now_us);
    }

    if (mask & STAGED_FIX_QUALITY) {
        nmea->fix_quality.value = nmea->fix_quality.staged;
        commit_meta(&nmea->fix_quality.meta, now_us);
    }
}

/**
 * @brief Handle a field that has just been read to its end
 *
 * @param[in,out] nmea Decoder state
 *
 * @return true if this field was a correct checksum closing a decoded sentence
 */
static bool term_complete(l86_m33_nmea_t *nmea)
{
    if (!nmea->is_checksum_term) {
        parse_term(nmea);
        return false;
    }

    uint8_t checksum = (uint8_t)((hex_to_int(nmea->term[0]) << 4) | hex_to_int(nmea->term[1]));
    if (checksum != nmea->parity) {
        nmea->failed_checksums++;
        return false;
    }

    nmea->passed_checksums++;

    if (nmea->sentence_type == L86_M33_SENTENCE_OTHER) {
        return false;
    }

    if (nmea->sentence_has_fix) {
        nmea->sentences_with_fix++;
    }

    commit_sentence(nmea);

    return true;
}

// *****************************************************************************
// Section: Public functions

void l86_m33_nmea_reset(l86_m33_nmea_t *nmea)
{
    if (nmea == NULL) {
        return;
    }

    memset(nmea, 0, sizeof(*nmea));
}

bool l86_m33_nmea_encode(l86_m33_nmea_t *nmea, char c)
{
    if (nmea == NULL) {
        return false;
    }

    nmea->chars_processed++;

    switch (c) {
    case ',':
        /* Field separators are part of what the checksum covers, the '*' that
         * closes the data is not */
        nmea->parity ^= (uint8_t)c;
    /* Fall through */
    case '*':
    case '\r':
    case '\n': {
        nmea->term[nmea->term_offset] = '\0';
        bool new_data = term_complete(nmea);
        nmea->term_number++;
        nmea->term_offset = 0;
        nmea->is_checksum_term = (c == '*');
        return new_data;
    }

    case '$':
        /* Start of a sentence, so drop whatever was being read */
        nmea->term_number = 0;
        nmea->term_offset = 0;
        nmea->parity = 0;
        nmea->is_checksum_term = false;
        nmea->sentence_has_fix = false;
        nmea->staged_mask = 0;
        nmea->sentence_type = L86_M33_SENTENCE_OTHER;
        return false;

    default:
        /* Fields longer than the buffer get cut short rather than allowed to run
         * over it, which only ever happens on a corrupted sentence and is caught
         * by the checksum */
        if (nmea->term_offset < sizeof(nmea->term) - 1) {
            nmea->term[nmea->term_offset++] = c;
        }
        if (!nmea->is_checksum_term) {
            nmea->parity ^= (uint8_t)c;
        }
        return false;
    }
}

uint32_t l86_m33_nmea_age_ms(const l86_m33_meta_t *meta)
{
    if (meta == NULL || !meta->valid) {
        return UINT32_MAX;
    }

    return (uint32_t)((esp_timer_get_time() - meta->commit_us) / 1000);
}
