#ifndef NEO8M_H
#define NEO8M_H

#include "stm32h7xx_hal.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"

typedef enum {
  NEO8M_OK = 0,
  NEO8M_ERROR,
  NEO8M_TIMEOUT,
  NEO8M_INVALID_PARAM,
  NEO8M_NO_FIX,
  NEO8M_BUFFER_FULL,
  NEO8M_CHECKSUM_ERROR
} neo8m_status_t;

typedef enum {
  NEO8M_FIX_NONE = 0,
  NEO8M_FIX_INVALID,
  NEO8M_FIX_2D,
  NEO8M_FIX_3D
} neo8m_fixquality_t;

typedef struct
{
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t millisecond;
} neo8m_time_t;

typedef struct {
  uint8_t day;
  uint8_t month;
  uint16_t year;
} neo8m_date_t;

typedef struct 
{
  double latitude;  // degrees (-90 to 90)
  double longitude; //degree (-180 to 180)
  float altitude; //meters above sea level
  float speed; //km/h
  float course; //degree (0-360)
  uint8_t satellites; //number of satellites
  float hdop; //horizontal dilution of precision
  neo8m_fixquality_t fix; //fix quality
  neo8m_time_t time;
  neo8m_date_t date;
  bool valid; // data valid flag
} neo8m_data_t;

typedef struct
{
  UART_HandleTypeDef *uart;

  //rx buffer
  uint8_t rx_buffer[256];
  uint16_t rx_index;

  //parsed data
  neo8m_data_t data;

  //state
  bool initialized;
  uint32_t last_update;
} neo8m_handle_t;

typedef struct
{
  UART_HandleTypeDef *huart;
  uint32_t baud_rate;
  uint32_t timeout_ms;
} neo8m_config_t;

/**
 * @brief initialize neo-8m gps module
 * 
 * @param gps 
 * @param config 
 * @return neo8m_status_t 
 */
neo8m_status_t neo8m_init(neo8m_handle_t *gps, const neo8m_config_t *config);

/**
 * @brief process incoming uart data
 * 
 * @param gps 
 * @param byte 
 * @return neo8m_status_t 
 */
neo8m_status_t neo8m_process(neo8m_handle_t *gps, uint8_t byte);

/**
 * @brief check if gps has valid fix
 * 
 * @param gps 
 * @return true 
 * @return false 
 */
bool neo8m_has_fix(const neo8m_handle_t *gps);

/**
 * @brief get parse GPs data
 * 
 */
const neo8m_data_t* neo8m_get_data(const neo8m_handle_t *gps);

/**
 * @brief start uart reception in interrupt mode
 * 
 */
neo8m_status_t neo8m_start_receive_it(neo8m_handle_t *gps);

/**
 * @brief start uart reception in dma mode
 * 
 */
 neo8m_status_t neo8m_start_receive_dma(neo8m_handle_t *gps, uint8_t *buffer, uint32_t size);

/**
 * @brief check if data is fresh (update recently)
 * 
 */
bool neo8m_is_data_fresh(const neo8m_handle_t *gps, uint32_t max_age_ms);

/**
 * @brief calculate distance between two coordinates (Haversine formular)
 * 
 */
float neo8m_calculate_distance(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief calculate bearing between two coordinates
 * 
 */
float neo8m_calculate_bearing(double lat1, double lon1, double lat2, double lon2);

#endif