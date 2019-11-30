#include "GPS.h"

#include <math.h>
#include <regex.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "usart.h"



#if GPS_GPGGA_ENABLED
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
#endif // GPS_GPGGA_ENABLED

#if GPS_GPRMC_ENABLED
// @formatter:off
const char* GPS_GPRMC_REGEX_STRING =
    "\\$GPRMC,"                             //     GPS recommended minimum data
        "([[:digit:]]*)\\.?"                //  1) Time hours, minutes, and seconds (combined)
        "([[:digit:]]*),"                   //  2) Time fractional seconds
        "([AV]?),"                          //  3) Warning A/V
        "([[:digit:]]*\\.?[[:digit:]]*),"   //  4) Latitude (DDMM.MMMMM)
        "([NS]?),"                          //  5) Latitude N/S
        "([[:digit:]]*\\.?[[:digit:]]*),"   //  6) Longitude (DDDMM.MMMMM)
        "([EW]?),"                          //  7) Longitude E/W
        "([[:digit:]]*\\.?[[:digit:]]*),"   //  8) Speed over ground (knots)
        "([[:digit:]]*\\.?[[:digit:]]*),"   //  9) Course over ground (degrees true)
        "([[:digit:]]*),"                   // 10) Time day, month, year (combined)
        "([[:digit:]]*\\.?[[:digit:]]*),"   // 11) Magnetic variation degrees
        "([EW]?),"                          // 12) Magnetic variation E/W
        "([NADE]?)"                         // 13) Mode (not valid, autonomous, differential, estimated/dead reckoning)
        "\\*([[:alnum:]]{2})"               // 14) Checksum
;
// @formatter:on
#endif // GPS_GPRMC_ENABLED

#if GPS_GPVTG_ENABLED
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
#endif // GPS_GPVTG_ENABLED

#if GPS_GPZDA_ENABLED
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
#endif // GPS_GPZDA_ENABLED

double convertDegMinToDecDeg(float deg_min, uint8_t is_negative)
{
    double min = 0.0;
    double dec_deg = 0.0;

    //get the minutes, fmod() requires double
    min = fmod((double) deg_min, 100.0);

    //rebuild coordinates in decimal degrees
    deg_min = (int) (deg_min / 100);
    dec_deg = deg_min + (min / 60);

    if (is_negative)
    {
        dec_deg = -dec_deg;
    }

    return dec_deg;
}

void GPS_Init(GPS_t* gps, UART_HandleTypeDef* uart)
{
    memset(gps, 0, sizeof(GPS_t));

#if GPS_GPGGA_ENABLED
    regcomp(&(gps->regex_gpgga), GPS_GPGGA_REGEX_STRING, REG_EXTENDED);
#endif // GPS_GPGGA_ENABLED

#if GPS_GPRMC_ENABLED
    regcomp(&(gps->regex_gprmc), GPS_GPRMC_REGEX_STRING, REG_EXTENDED);
#endif // GPS_GPRMC_ENABLED

#if GPS_GPVTG_ENABLED
    regcomp(&(gps->regex_gpvtg), GPS_GPVTG_REGEX_STRING, REG_EXTENDED);
#endif // GPS_GPVTG_ENABLED

#if GPS_GPZDA_ENABLED
    regcomp(&(gps->regex_gpzda), GPS_GPZDA_REGEX_STRING, REG_EXTENDED);
#endif // GPS_GPZDA_ENABLED

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

void GPS_Process(uint32_t current_ms, GPS_t* gps, UART_HandleTypeDef* uart)
{
    uint8_t must_clear_buffer = (gps->buffer.next_index >= GPS_BUFFER_SIZE - 1);

    if (gps->buffer.next_index > 0)
    { // There is data available
        int result_bool = 0; // nonzero if something failed

#if GPS_GPGGA_ENABLED
        result_bool = GPS_Process_GPGGA(gps, current_ms) || result_bool;
#endif // GPS_GPGGA_ENABLED

#if GPS_GPRMC_ENABLED
        result_bool = GPS_Process_GPRMC(gps, current_ms) || result_bool;
#endif // GPS_GPRMC_ENABLED

#if GPS_GPVTG_ENABLED
        result_bool = GPS_Process_GPVTG(gps, current_ms) || result_bool;
#endif // GPS_GPVTG_ENABLED

#if GPS_GPZDA_ENABLED
        result_bool = GPS_Process_GPZDA(gps, current_ms) || result_bool;
#endif // GPS_GPZDA_ENABLED

        if (0 == result_bool)
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

#if GPS_GPGGA_ENABLED
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

        size_t index = 0; // index 0 is the entire string!
        char* substring = NULL;
        char* end = NULL;

        //  1) Time hours, minutes, and seconds (combined)
        substring = (char*)&(string[matches[++index].rm_so]);
        uint32_t hours_minutes_seconds = atoll(substring);
        gpgga->utc_h = (hours_minutes_seconds / 10000) % 100;
        gpgga->utc_m = (hours_minutes_seconds /   100) % 100;
        gpgga->utc_s = (hours_minutes_seconds /     1) % 100;

        //  2) Time microseconds
        substring = (char*)&(string[matches[++index].rm_so]);
        gpgga->utc_us = atol(substring);

        //  3) Latitude (DDMM.MMMMM)
        //  4) Latitude N/S
        substring = (char*)&(string[matches[++index].rm_so]);
        gprmc->lat_valid = (matches[index].rm_eo > matches[index].rm_so);
        gprmc->lat = convertDegMinToDecDeg(
            atoff(substring),
            'S' == (string[matches[++index].rm_so])
        );

        //  5) Longitude (DDDMM.MMMMM)
        //  6) Longitude E/W
        substring = (char*)&(string[matches[++index].rm_so]);
        gprmc->lon_valid = (matches[index].rm_eo > matches[index].rm_so);
        gprmc->lon = convertDegMinToDecDeg(
            atoff(substring),
            'W' == (string[matches[++index].rm_so])
        );

        //  7) Quality indicator
        substring = (char*)&(string[matches[++index].rm_so]);
        gpgga->quality = atol(substring);

        //  8) Number of satellites used
        substring = (char*)&(string[matches[++index].rm_so]);
        gpgga->num_sats = atol(substring);

        //  9) Horizontal dilution of precision (HDOP)
        substring = (char*)&(string[matches[++index].rm_so]);
        gpgga->hdop = atoff(substring);

        // 10) Antenna altitude
        substring = (char*)&(string[matches[++index].rm_so]);
        gpgga->alt = atoff(substring);

        // 11) Antenna altitude units (M/F)
        substring = (char*)&(string[matches[++index].rm_so]);
        if ('M' == *substring || 'F' == *substring)
        {
            gpgga->alt_unit = *substring;
        }

        // 12) Geoidal separation
        substring = (char*)&(string[matches[++index].rm_so]);
        gpgga->geo = atoff(substring);

        // 13) Geoidal separation units (M/F)
        substring = (char*)&(string[matches[++index].rm_so]);
        if ('M' == *substring || 'F' == *substring)
        {
            gpgga->geo_unit = *substring;
        }

        // 14) Age of correction
        substring = (char*)&(string[matches[++index].rm_so]);
        gpgga->aoc = atoi(substring);

        // 15) Correction station ID
        substring = (char*)&(string[matches[++index].rm_so]);
        end = (char*)&(string[matches[index].rm_eo]);
        memcpy(gpgga->station, substring, (end - substring));

        // 16) Checksum
        substring = (char*)&(string[matches[++index].rm_so]);
        end = (char*)&(string[matches[index].rm_eo]);
        memcpy(gpgga->check, substring, (end - substring));
    }

    return result;
}
#endif // GPS_GPGGA_ENABLED

#if GPS_GPRMC_ENABLED
int GPS_Process_GPRMC(GPS_t* gps, uint32_t current_ms)
{
    char* string = (char*)(gps->buffer.chars);
    int result = regexec(
        &(gps->regex_gprmc),
        string,
        GPS_GPRMC_NUM_FIELDS,
        gps->regmatch_gprmc,
        0
    );

    if (0 == result)
    { // All captures successful, can parse
        GPRMC_t* gprmc = &(gps->gprmc);
        regmatch_t* matches = gps->regmatch_gprmc;
        gprmc->updated_ms = current_ms;

        size_t index = 0; // index 0 is the entire string!
        char* substring = NULL;
        char* end = NULL;

        //  1) Time hours, minutes, and seconds (combined)
        substring = (char*)&(string[matches[++index].rm_so]);
        uint32_t hours_minutes_seconds = atoll(substring);
        gprmc->utc_h = (hours_minutes_seconds / 10000) % 100;
        gprmc->utc_m = (hours_minutes_seconds /   100) % 100;
        gprmc->utc_s = (hours_minutes_seconds /     1) % 100;

        //  2) Time fractional seconds
        substring = (char*)&(string[matches[++index].rm_so]);
        gprmc->speed_kt = atoi(substring);

        //  3) Warning A/V
        substring = (char*)&(string[matches[++index].rm_so]);
        if ('A' == *substring || 'V' == *substring)
        {
            gprmc->nav_warn = *substring;
        }

        //  4) Latitude (DDMM.MMMMM)
        //  5) Latitude N/S
        substring = (char*)&(string[matches[++index].rm_so]);
        gprmc->lat_valid = (matches[index].rm_eo > matches[index].rm_so);
        gprmc->lat = convertDegMinToDecDeg(
            atoff(substring),
            'S' == (string[matches[++index].rm_so])
        );

        //  6) Longitude (DDDMM.MMMMM)
        //  7) Longitude E/W
        substring = (char*)&(string[matches[++index].rm_so]);
        gprmc->lon_valid = (matches[index].rm_eo > matches[index].rm_so);
        gprmc->lon = convertDegMinToDecDeg(
            atoff(substring),
            'W' == (string[matches[++index].rm_so])
        );

        //  8) Speed over ground (knots)
        substring = (char*)&(string[matches[++index].rm_so]);
        gprmc->speed_kt = atoff(substring);

        //  9) Course over ground (degrees true)
        substring = (char*)&(string[matches[++index].rm_so]);
        gprmc->course_t = atoff(substring);

        // 10) Time day, month, year (combined)
        substring = (char*)&(string[matches[++index].rm_so]);
        uint32_t day_mon_year = atoll(substring);
        gprmc->utc_day  = (day_mon_year / 10000) % 100;
        gprmc->utc_mon  = (day_mon_year /   100) % 100;
        gprmc->utc_year = (day_mon_year /     1) % 100 + 2000;

        // 11) Magnetic variation degrees
        substring = (char*)&(string[matches[++index].rm_so]);
        gprmc->var = atoff(substring);

        // 12) Magnetic variation E/W
        substring = (char*)&(string[matches[++index].rm_so]);
        if ('E' == *substring || 'W' == *substring)
        {
            gprmc->var_c = *substring;
        }

        // 13) Mode (not valid, autonomous, differential, estimated/dead reckoning)
        substring = (char*)&(string[matches[++index].rm_so]);
        if ('N' == *substring || 'A' == *substring ||
            'D' == *substring || 'E' == *substring)
        {
            gprmc->mode = *substring;
        }

        // 14) Checksum
        substring = (char*)&(string[matches[++index].rm_so]);
        end = (char*)&(string[matches[index].rm_eo]);
        memcpy(gprmc->check, substring, (end - substring));
    }

    return result;
}
#endif // GPS_GPRMC_ENABLED

#if GPS_GPVTG_ENABLED
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

        size_t index = 0; // index 0 is the entire string!
        char* substring = NULL;
        char* end = NULL;

        //  1) Course over ground (degrees true)
        substring = (char*)&(string[matches[++index].rm_so]);
        gpvtg->course_t = atoff(substring);

        //  2) Degrees true
        substring = (char*)&(string[matches[++index].rm_so]);
        if ('T' == *substring)
        {
            gpvtg->course_t_c = *substring;
        }

        //  3) Course over ground (degrees magnetic)
        substring = (char*)&(string[matches[++index].rm_so]);
        gpvtg->course_m = atoff(substring);

        //  4) Degrees magnetic
        substring = (char*)&(string[matches[++index].rm_so]);
        if ('M' == *substring)
        {
            gpvtg->course_m_c = *substring;
        }

        //  5) Speed over ground (knots)
        substring = (char*)&(string[matches[++index].rm_so]);
        gpvtg->speed_kt = atoff(substring);

        //  6) Speed knots
        substring = (char*)&(string[matches[++index].rm_so]);
        if ('N' == *substring)
        {
            gpvtg->speed_kt_c = *substring;
        }

        //  7) Speed over ground (kph)
        substring = (char*)&(string[matches[++index].rm_so]);
        gpvtg->speed_km = atoff(substring);

        //  8) Speed kilometers per hour
        substring = (char*)&(string[matches[++index].rm_so]);
        if ('K' == *substring)
        {
            gpvtg->speed_km_c = *substring;
        }

        //  9) Mode (not valid, autonomous, differential, estimated/dead reckoning)
        substring = (char*)&(string[matches[++index].rm_so]);
        gpvtg->mode = *substring;

        // 10) Checksum
        substring = (char*)&(string[matches[++index].rm_so]);
        end = (char*)&(string[matches[index].rm_eo]);
        memcpy(gpvtg->check, substring, (end - substring));
    }

    return result;
}
#endif // GPS_GPVTG_ENABLED

#if GPS_GPZDA_ENABLED
int GPS_Process_GPZDA(GPS_t* gps, uint32_t current_ms)
{
    char* string = (char*)(gps->buffer.chars);
    int result = regexec(
        &(gps->regex_gpzda),
        string,
        GPS_GPZDA_NUM_FIELDS,
        gps->regmatch_gpzda,
        0
    );

    if (0 == result)
    { // All captures successful, can parse
        GPZDA_t* gpzda = &(gps->gpzda);
        regmatch_t* matches = gps->regmatch_gpzda;
        gpzda->updated_ms = current_ms;

        size_t index = 0; // index 0 is the entire string!
        char* substring = NULL;
        char* end = NULL;

        //  1) Time hours, minutes, and seconds (combined)
        substring = (char*)&(string[matches[++index].rm_so]);
        uint32_t hours_minutes_seconds = atoll(substring);
        gpzda->utc_h = (hours_minutes_seconds / 10000) % 100;
        gpzda->utc_m = (hours_minutes_seconds /   100) % 100;
        gpzda->utc_s = (hours_minutes_seconds /     1) % 100;

        //  2) Time microseconds
        substring = (char*)&(string[matches[++index].rm_so]);
        gpzda->utc_us = atol(substring);

        //  3) Time day
        substring = (char*)&(string[matches[++index].rm_so]);
        gpzda->utc_day = atoi(substring);

        //  4) Time month
        substring = (char*)&(string[matches[++index].rm_so]);
        gpzda->utc_month = atoi(substring);

        //  5) Time year
        substring = (char*)&(string[matches[++index].rm_so]);
        gpzda->utc_year = atoi(substring);

        //  6) Time local zone hours
        substring = (char*)&(string[matches[++index].rm_so]);
        gpzda->utc_local_hours = atoi(substring);

        //  7) Time local zone minutes
        substring = (char*)&(string[matches[++index].rm_so]);
        gpzda->utc_local_minutes = atoi(substring);

        //  8) Checksum
        substring = (char*)&(string[matches[++index].rm_so]);
        end = (char*)&(string[matches[index].rm_eo]);
        memcpy(gpzda->check, substring, (end - substring));
    }

    return result;
}
#endif // GPS_GPZDA_ENABLED
