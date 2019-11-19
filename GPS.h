#ifndef _GPS_H_
#define _GPS_H_

#include <stddef.h>
#include <stdint.h>
#include <regex.h>

#include "usart.h"

#ifndef GPS_BUFFER_SIZE
#define GPS_BUFFER_SIZE 512
#endif

#ifndef GPS_MS_BEFORE_CHECK
#define GPS_MS_BEFORE_CHECK 50
#endif

#ifndef GPS_DEBUG_PRINT_ENABLED
#define GPS_DEBUG_PRINT_ENABLED 0
#endif

#ifndef GPS_DEBUG_PRINT
#define GPS_DEBUG_PRINT(string_uint8_ptr, string_len_bytes) // do nothing by default
#endif

/*!
 * @brief Buffer for GPS string data.
 *
 * @details As the data streams in, it will be stored here for processing later.
 */
typedef struct
{
    uint32_t    updated_ms; //! Last ticks (in ms) that this was updated

    uint8_t     chars[GPS_BUFFER_SIZE]; //! Input character buffer
    uint8_t     char_interrupt;         //! Input character from interrupt
    uint16_t    next_index;             //! Next input buffer index
} GPS_Buffer_t;

/*!
 * @brief Contains GPS GPGGA data.
 *
 * @details
 * This is the Global Positioning System fix data.
 *
 * @see https://docs.novatel.com/oem7/Content/Logs/GPGGA.htm
 * @see http://navspark.mybigcommerce.com/content/NMEA_Format_v0.1.pdf
 * @see http://lefebure.com/articles/nmea-gga/
 */
typedef struct
{
    uint32_t    updated_ms; //! Last ticks (in ms) that this was updated

    uint8_t     utc_h;      //! UTC hour
    uint8_t     utc_m;      //! UTC minute
    uint8_t     utc_s;      //! UTC second
    uint16_t    utc_us;     //! UTC microsecond

    float       lat;        //! Latitude
    char        lat_dir;    //! Latitude direction (N/S); null if @a lat invalid
    float       lon;        //! Longitude
    char        lon_dir;    //! Longitude direction (E/W); null if @a lon invalid

    uint8_t     quality;    //! Quality indicator
    uint8_t     num_sats;   //! Number of satellites used
    float       hdop;       //! Horizontal dilution of precision

    float       alt;        //! Mean sea level altitude
    char        alt_unit;   //! Unit for @a alt; null if @a alt invalid

    float       geo;        //! Geoidal separation
    char        geo_unit;   //! Unit for @a geo; null if @a geo invalid

    int8_t      aoc;        //! Age of correction; -1 if invalid
    char        station[4]; //! Differential base station ID
    char        check[2];   //! Checksum for message

} GPGGA_t;

#define GPS_GPGGA_NUM_FIELDS (16+1)

/*!
 * @brief Contains GPS GPVTG data.
 *
 * @details
 * This is the Global Positioning System course and ground speed data.
 *
 * @see https://docs.novatel.com/oem7/Content/Logs/GPVTG.htm
 * @see http://navspark.mybigcommerce.com/content/NMEA_Format_v0.1.pdf
 */
typedef struct
{
    uint32_t    updated_ms; //! Last ticks (in ms) that this was updated

    float       course_t;   //! Course over ground, degrees True
    char        course_t_c; //! Label for @a course_t; null if @a course_t invalid

    float       course_m;   //! Course over ground, degrees Magnetic
    char        course_m_c; //! Label for @a course_m; null if @a course_m invalid

    float       speed_kt;   //! Speed over ground, knots
    char        speed_kt_c; //! Label for @a speed_t; null if @a speed_t invalid

    float       speed_km;   //! Speed over ground, kilometers per hour
    char        speed_km_c; //! Label for @a speed_t; null if @a speed_t invalid

    char        mode;       //! Mode indicator; null if invalid

    char        check[2];   //! Checksum for message
} GPVTG_t;

#define GPS_GPVTG_NUM_FIELDS (10+1)

/*!
 * @brief Contains GPS GPZDA data.
 *
 * @details
 * This is the Global Positioning System time and date information.
 *
 * @see https://docs.novatel.com/oem7/Content/Logs/GPZDA.htm
 * @see http://navspark.mybigcommerce.com/content/NMEA_Format_v0.1.pdf
 */
typedef struct
{
    uint32_t    updated_ms; //! Last ticks (in ms) that this was updated

    uint8_t     utc_h;      //! UTC hour
    uint8_t     utc_m;      //! UTC minute
    uint8_t     utc_s;      //! UTC second
    uint16_t    utc_us;     //! UTC microsecond

    uint8_t     utc_day;    //! UTC day
    uint8_t     utc_month;  //! UTC month
    uint16_t    utc_year;   //! UTC year

    int8_t      utc_local_hours;    //! UTC local zone hours
    int8_t      utc_local_minutes;  //! UTC local zone minutes

    char        check[2];   //! Checksum for message
} GPZDA_t;

#define GPS_GPZDA_NUM_FIELDS (8+1)

/*!
 * @brief Describes latest captured GPS data and buffers upcoming data.
 *
 * @details
 * Stores incoming characters streamed from GPS in @a buffer, and the
 * latest converted GPGGA and GPVTG data in the appropriate structures.
 */
typedef struct
{
    GPS_Buffer_t    buffer;

    regex_t         regex_gpgga;
    regex_t         regex_gpvtg;
    regex_t         regex_gpzda;

    regmatch_t      regmatch_gpgga[GPS_GPGGA_NUM_FIELDS];
    regmatch_t      regmatch_gpvtg[GPS_GPVTG_NUM_FIELDS];
    regmatch_t      regmatch_gpzda[GPS_GPVTG_NUM_FIELDS];

    GPGGA_t         gpgga;
    GPVTG_t         gpvtg;
    GPZDA_t         gpzda;
} GPS_t;

/*!
 * @brief Sets up a GPS to work with a UART.
 *
 * @details
 * Clears out the entire @a gps item and grabs a test character
 * from the @a uart.
 *
 * @param gps   Pointer to the GPS_t that will be used for the captured data.
 * @param uart  Pointer to the UART_HandleTypeDef that will stream the GPS data.
 *
 * @retval void
 */
void GPS_Init(      GPS_t* gps, UART_HandleTypeDef* uart);

/*!
 * @brief Handle incoming characters from the uart to the GPS.
 *
 * @details
 * Handle buffering and recording character data into the buffer of the @a gps,
 * so that it can be processed later.
 *
 * @param gps   Pointer to the GPS_t that will be used for the captured data.
 * @param uart  Pointer to the UART_HandleTypeDef that will stream the GPS data.
 *
 * @retval void
 */
void GPS_CallBack(  GPS_t* gps, UART_HandleTypeDef* uart);

/*!
 * @brief Process buffered characters into GPS data.
 *
 * @details
 * Use global regular expressions and string conversions to store data into the
 * @a gps in the GPGGA_t and GPVTG_t structures it contains.
 *
 * @note Not reentrant.
 *
 * @todo Make reentrant
 *
 * @param gps   Pointer to the GPS_t that provides the captured data and
 *              will store the converted structs.
 *
 * @retval void
 */
void GPS_Process(   GPS_t* gps, UART_HandleTypeDef* uart);

int GPS_Process_GPGGA(GPS_t* gps, uint32_t current_ms);

int GPS_Process_GPVTG(GPS_t* gps, uint32_t current_ms);

int GPS_Process_GPZDA(GPS_t* gps, uint32_t current_ms);

#endif
