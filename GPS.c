#include "GPS.h"

#include <math.h>
#include <regex.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "usart.h"

// @formatter:off
const char* GPS_GPGGA_REGEX_STRING =
    "\\$GPGGA,"                             //     GPS position indicator
        "([[:digit:]]*)\\.?"                //  1) Time hours, minutes, and seconds (combined)
        "([[:digit:]]*),"                   //  2) Time microseconds
        "([[:digit:]]*\\.?[[:digit:]]*),"   //  3) Latitude (DDMM.MMMMM)
        "([NS]?),"                          //  4) Latitude N/S
        "([[:digit:]]*\\.?[[:digit:]]*),"   //  5) Longitude (DDDMM.MMMMM)
        "([EW]?),"                          //  6) Longitude E/W
        "([[:digit:]]?),"                   //  7) Quality indicator
        "([[:digit:]]{2}),"                 //  8) Number of satellites used
        "([[:digit:]]*\\.?[[:digit:]]*),"   //  9) Horizontal dilution of precision (HDOP)
        "(-?[[:digit:]]*\\.?[[:digit:]]*)," // 10) Antenna altitude
        "([MF]?),"                          // 11) Antenna altitude units (M/F)
        "(-?[[:digit:]]*\\.?[[:digit:]]*)," // 12) Geoidal separation
        "([MF]?),"                          // 13) Geoidal separation units (M/F)
        "([[:digit:]]*\\.?[[:digit:]]*),"   // 14) Age of correction
        "([[:digit:]]*)"                    // 15) Correction station ID
        "\\*([[:alnum:]]{2})"               // 16) Checksum
;
// @formatter:on

// @formatter:off
const char* GPS_GPVTG_REGEX_STRING =
    "\\$GPVTG,"                             //     GPS course and ground speed
        "([[:digit:]]*\\.?[[:digit:]]*),"   //  1) Course over ground (degrees true)
        "(T?),"                             //  2) Degrees true
        "([[:digit:]]*\\.?[[:digit:]]*),"   //  3) Course over ground (degrees magnetic)
        "(M?),"                             //  4) Degrees magnetic
        "([[:digit:]]*\\.?[[:digit:]]*),"   //  5) Speed over ground (knots)
        "(N?),"                             //  6) Speed knots
        "([[:digit:]]*\\.?[[:digit:]]*),"   //  7) Speed over ground (kph)
        "(K?),"                             //  8) Speed kilometers per hour
        "([NADE]?)"                         //  9) Mode (not valid, autonomous, differential, estimated/dead reckoning)
        "\\*([[:alnum:]]{2})"               // 10) Checksum
;
// @formatter:on

// @formatter:off
const char* GPS_GPZDA_REGEX_STRING =
    "\\$GPZDA,"                             //     GPS time and date indicator
        "([[:digit:]]*)\\.?"                //  1) Time hours, minutes, and seconds (combined)
        "([[:digit:]]*),"                   //  2) Time microseconds
        "([[:digit:]]{2})?,"                //  3) Time day
        "([[:digit:]]{2})?,"                //  4) Time month
        "([[:digit:]]{4})?,"                //  5) Time year
        "(-?[[:digit:]]{2})?,"              //  6) Time local zone hours
        "(-?[[:digit:]]{2})?,"              //  7) Time local zone minutes
        "\\*([[:alnum:]]{2})"               //  8) Checksum
;
// @formatter:on

double convertDegMinToDecDeg(float degMin)
{
    double min = 0.0;
    double decDeg = 0.0;

    //get the minutes, fmod() requires double
    min = fmod((double) degMin, 100.0);

    //rebuild coordinates in decimal degrees
    degMin = (int) (degMin / 100);
    decDeg = degMin + (min / 60);

    return decDeg;
}

void GPS_Init(GPS_t* gps, UART_HandleTypeDef* uart)
{
    memset(gps, 0, sizeof(GPS_t));

    regcomp(&(gps->regex_gpgga), GPS_GPGGA_REGEX_STRING, REG_EXTENDED);
    regcomp(&(gps->regex_gpvtg), GPS_GPVTG_REGEX_STRING, REG_EXTENDED);
    regcomp(&(gps->regex_gpzda), GPS_GPZDA_REGEX_STRING, REG_EXTENDED);

    HAL_UART_Receive_IT(uart, &(gps->buffer.char_interrupt), 1);
}

void GPS_CallBack(GPS_t* gps, UART_HandleTypeDef* uart)
{
    // The most recent character has been stored in
    // gps->buffer.char_interrupt; handle it and set
    // up to capture the next one.

#if (1 == GPS_DEBUG_PRINT_ENABLED)
    GPS_DEBUG_PRINT(&(gps->buffer.char_interrupt), 1);
#endif

    gps->buffer.updated_ms = HAL_GetTick();
    if (gps->buffer.char_interrupt != '\0' && gps->buffer.next_index < GPS_BUFFER_SIZE - 1)
    {
        gps->buffer.chars[gps->buffer.next_index] = gps->buffer.char_interrupt;
        ++gps->buffer.next_index;
    }

    // Listen for the next character
    HAL_UART_Receive_IT(uart, &(gps->buffer.char_interrupt), 1);
}

void GPS_Process(GPS_t* gps, UART_HandleTypeDef* uart)
{
    uint32_t current_ms = HAL_GetTick();
    uint8_t must_clear_buffer = (gps->buffer.next_index >= GPS_BUFFER_SIZE - 1);

    if (current_ms > (gps->buffer.updated_ms + GPS_MS_BEFORE_CHECK) &&
        gps->buffer.next_index > 0)
    { // It has been long enough since an update, and there is data available
        int gpgga_result = GPS_Process_GPGGA(gps, current_ms);
        int gpvtg_result = GPS_Process_GPVTG(gps, current_ms);
        int gpzda_result = GPS_Process_GPZDA(gps, current_ms);

        if (0 == gpgga_result &&
            0 == gpvtg_result &&
            0 == gpzda_result)
        {
            must_clear_buffer = 1;
        }
    }

    if (must_clear_buffer)
    {
        memset(&(gps->buffer.chars), 0, GPS_BUFFER_SIZE);
        gps->buffer.next_index = 0;
        gps->buffer.updated_ms = current_ms;
    }

    // Ensure that the interrupt is listening for the next character.
    HAL_UART_Receive_IT(uart, &(gps->buffer.char_interrupt), 1);
}

int GPS_Process_GPGGA(GPS_t* gps, uint32_t current_ms)
{
    char* string = (char*)(gps->buffer.chars);
    int result = regexec(
        &(gps->regex_gpgga),
        string,
        GPS_GPGGA_NUM_FIELDS,
        gps->regmatch_gpgga,
        0
    );

    if (0 == result)
    { // All captures successful, can parse
        GPGGA_t* gpgga = &(gps->gpgga);
        regmatch_t* matches = gps->regmatch_gpgga;
        gpgga->updated_ms = current_ms;

        size_t index = 1; // index 0 is the entire string!
        char* substring = (char*)&(string[matches[index].rm_so]);
        char* end = NULL;

        //  1) Time hours, minutes, and seconds (combined)
        uint32_t hours_minutes_seconds = atoll(substring);
        gpgga->utc_h = (hours_minutes_seconds / 10000) % 100;
        gpgga->utc_m = (hours_minutes_seconds /   100) % 100;
        gpgga->utc_s = (hours_minutes_seconds /     1) % 100;
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  2) Time microseconds
        gpgga->utc_us = atol(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  3) Latitude (DDMM.MMMMM)
        gpgga->lat = convertDegMinToDecDeg(atoff(substring));
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  4) Latitude N/S
        if ('N' == *substring || 'S' == *substring)
        {
            gpgga->lat_dir = *substring;
        }
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  5) Longitude (DDDMM.MMMMM)
        gpgga->lon = convertDegMinToDecDeg(atoff(substring));
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  6) Longitude E/W
        if ('E' == *substring || 'W' == *substring)
        {
            gpgga->lon_dir = *substring;
        }
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  7) Quality indicator
        gpgga->quality = atol(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  8) Number of satellites used
        gpgga->num_sats = atol(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  9) Horizontal dilution of precision (HDOP)
        gpgga->hdop = atoff(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        // 10) Antenna altitude
        gpgga->alt = atoff(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        // 11) Antenna altitude units (M/F)
        if ('M' == *substring || 'F' == *substring)
        {
            gpgga->alt_unit = *substring;
        }
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        // 12) Geoidal separation
        gpgga->geo = atoff(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        // 13) Geoidal separation units (M/F)
        if ('M' == *substring || 'F' == *substring)
        {
            gpgga->geo_unit = *substring;
        }
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        // 14) Age of correction
        gpgga->aoc = atoi(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        // 15) Correction station ID
        end = (char*)&(string[matches[index].rm_eo]);
        memcpy(gpgga->station, substring, (end - substring));
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        // 16) Checksum
        end = (char*)&(string[matches[index].rm_eo]);
        memcpy(gpgga->check, substring, (end - substring));
        //++index;
        //substring = (char*)&(string[matches[index].rm_so]);
    }

    return result;
}

int GPS_Process_GPVTG(GPS_t* gps, uint32_t current_ms)
{
    char* string = (char*)(gps->buffer.chars);
    int result = regexec(
        &(gps->regex_gpvtg),
        string,
        GPS_GPVTG_NUM_FIELDS,
        gps->regmatch_gpvtg,
        0
    );

    if (0 == result)
    { // All captures successful, can parse
        GPVTG_t* gpvtg = &(gps->gpvtg);
        regmatch_t* matches = gps->regmatch_gpvtg;
        gpvtg->updated_ms = current_ms;

        size_t index = 1; // index 0 is the entire string!
        char* substring = (char*)&(string[matches[index].rm_so]);
        char* end = NULL;

        //  1) Course over ground (degrees true)
        gpvtg->course_t = atoi(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  2) Degrees true
        if ('T' == *substring)
        {
            gpvtg->course_t_c = *substring;
        }
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  3) Course over ground (degrees magnetic)
        gpvtg->course_m = atoi(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  4) Degrees magnetic
        if ('M' == *substring)
        {
            gpvtg->course_m_c = *substring;
        }
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  5) Speed over ground (knots)
        gpvtg->speed_kt = atoi(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  6) Speed knots
        if ('N' == *substring)
        {
            gpvtg->speed_kt_c = *substring;
        }
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  7) Speed over ground (kph)
        gpvtg->speed_km = atoi(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  8) Speed kilometers per hour
        if ('K' == *substring)
        {
            gpvtg->speed_km_c = *substring;
        }
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  9) Mode (not valid, autonomous, differential, estimated/dead reckoning)
        gpvtg->mode = *substring;
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        // 10) Checksum
        end = (char*)&(string[matches[index].rm_eo]);
        memcpy(gpvtg->check, substring, (end - substring));
        //++index;
        //substring = (char*)&(string[matches[index].rm_so]);
    }

    return result;
}

int GPS_Process_GPZDA(GPS_t* gps, uint32_t current_ms)
{
    char* string = (char*)(gps->buffer.chars);
    int result = regexec(
        &(gps->regex_gpzda),
        string,
        GPS_GPGGA_NUM_FIELDS,
        gps->regmatch_gpzda,
        0
    );

    if (0 == result)
    { // All captures successful, can parse
        GPZDA_t* gpzda = &(gps->gpzda);
        regmatch_t* matches = gps->regmatch_gpzda;
        gpzda->updated_ms = current_ms;

        size_t index = 1; // index 0 is the entire string!
        char* substring = (char*)&(string[matches[index].rm_so]);
        char* end = NULL;

        //  1) Time hours, minutes, and seconds (combined)
        uint32_t hours_minutes_seconds = atoll(substring);
        gpzda->utc_h = (hours_minutes_seconds / 10000) % 100;
        gpzda->utc_m = (hours_minutes_seconds /   100) % 100;
        gpzda->utc_s = (hours_minutes_seconds /     1) % 100;
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  2) Time microseconds
        gpzda->utc_us = atol(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  3) Time day
        gpzda->utc_day = atoi(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  4) Time month
        gpzda->utc_month = atoi(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  5) Time year
        gpzda->utc_year = atoi(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  6) Time local zone hours
        gpzda->utc_local_hours = atoi(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  7) Time local zone minutes
        gpzda->utc_local_minutes = atoi(substring);
        ++index;
        substring = (char*)&(string[matches[index].rm_so]);

        //  8) Checksum
        end = (char*)&(string[matches[index].rm_eo]);
        memcpy(gpzda->check, substring, (end - substring));
        //++index;
        //substring = (char*)&(string[matches[index].rm_so]);
    }

    return result;
}
