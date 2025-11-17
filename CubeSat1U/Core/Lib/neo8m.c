#include "neo8m.h"
#include "string.h"
#include"math.h"

//private define
#define NMEA_MAX_LENGTH     82
#define EARTH_RADIUS_KM     6371.0
#define DEG_TO_RAD          0.017453292519943295
#define RAD_TO_DEG          57.29577951308232

static bool neo8m_parse_nmea(neo8m_handle_t *gps, const char *nmea);
static bool neo8m_parse_gga(neo8m_handle_t *gps, const char *sentence);
static bool neo8m_parse_rmc(neo8m_handle_t *gps, const char *sentence);
static uint8_t neo8m_calculate_check_sum(const char *sentence);
static bool neo8m_verify_check_sum(const char *sentence);
static double neo8m_parse_coordinate (const char *c, char dir);
static void neo8m_parse_time(const char *time_str, neo8m_time_t *time);
static void neo8m_parse_date(const char *date_str, neo8m_date_t *date);


neo8m_status_t neo8m_init(neo8m_handle_t *gps, const neo8m_config_t *config)
{
  if (gps == NULL || config->huart == NULL)
  {
    return NEO8M_INVALID_PARAM;
  }

  //initialize handle
  gps->uart = config->huart;
  gps->rx_index = 0;
  gps->initialized = true;
  gps->last_update = 0;

  //clear data
  memset(&gps->data, 0, sizeof(gps));
  memset(gps->rx_buffer, 0, sizeof(gps->rx_buffer));

  return NEO8M_OK;
  
}

neo8m_status_t neo8m_process(neo8m_handle_t *gps, uint8_t byte)
{
  if (!gps->initialized)
  {
    return NEO8M_ERROR;
  }

  //add byte to buffer
  if (gps->rx_index >= sizeof(gps->rx_buffer) - 1)
  {
    gps->rx_index = 0;
    return NEO8M_BUFFER_FULL;
  }
  //ghi byte & tang chi so index
  gps->rx_buffer[gps->rx_index++] = byte;

  //check for end of nmea sentence
  if (byte == '\n' && gps->rx_index >= 2 && gps->rx_buffer[gps->rx_index - 2] == '\r')
  {
    //end string
    gps->rx_buffer[gps->rx_index - 2] = '\0';
    //goi ham phan tich chuoi
    if (neo8m_parse_nmea(gps, (char*)gps->rx_buffer))
    {
      gps->last_update = HAL_GetTick();
    }
    //reset for next sentence
  gps->rx_index = 0;
  }
  return NEO8M_OK;  
  
}

//kiem tra xem da xac dinh duoc vi tri toa do hay chua
bool neo8m_has_fix(const neo8m_handle_t *gps)
{
  if (!gps->initialized)
  {
    return false;
  }
  return (gps->data.fix >= NEO8M_FIX_2D && gps->data.valid);
  
}

const neo8m_data_t* neo8m_get_data(const neo8m_handle_t *gps)
{
  return &gps->data;
}

//nhận dữ liệu uart ở chế độ ngắt
neo8m_status_t neo8m_start_receive_it(neo8m_handle_t *gps)
{
  if (!gps->initialized)
  {
    return NEO8M_ERROR;
  }
  
  //start single byte reception
  if (HAL_UART_Receive_IT(gps->uart, &gps->rx_buffer[0], 1) != HAL_OK)
  {
    return NEO8M_ERROR;
  }
  return NEO8M_OK;
  
}

neo8m_status_t neo8m_start_receive_dma(neo8m_handle_t *gps, uint8_t *buffer, uint32_t size)
{
  if (!gps->initialized)
  {
    return NEO8M_ERROR;
  }
  if (HAL_UART_Receive_DMA(gps->uart, buffer, size) != HAL_OK)
  {
    return NEO8M_ERROR;
  }

  return NEO8M_OK;  
}

bool neo8m_is_data_fresh(const neo8m_handle_t *gps, uint32_t max_age_ms)
{
  if (!gps->initialized)
  {
    return false;
  }
  return (HAL_GetTick() - gps->last_update) < max_age_ms;
  
}

float neo8m_calculate_distance(double lat1, double lon1, double lat2, double lon2)
{
  double dlat = (lat2 - lat1) * DEG_TO_RAD;
  double dlon = (lon2 - lon1) * DEG_TO_RAD;

  double a = sin(dlat/2) * sin(dlat/2) + sin(dlon/2) * sin(dlon/2) * cos(lat1) * cos(lat2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));

  return EARTH_RADIUS_KM * c * 1000; //return in meters
}

float neo8m_calculate_bearing(double lat1, double lon1, double lat2, double lon2)
{
  lat1 *= DEG_TO_RAD;
  lat2 *= DEG_TO_RAD;
  double dlon = (lon2 - lon1) * DEG_TO_RAD;

  double y = sin(dlon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);

  float bearing = atan2(y,x) * RAD_TO_DEG;

  //nomalize to 0 - 360
  if (bearing < 0)
  {
    bearing += 360;
  }
  return bearing;
  
}

static bool neo8m_parse_nmea(neo8m_handle_t *gps, const char *nmea)
{
  if (nmea[0] != '$') return false;

  //verify checksum
  if (!neo8m_verify_check_sum(nmea))
  {
    return false;
  }

  //parse specific sentence types
  if (strstr(nmea, "$GPGGA") || strstr(nmea, "$GNGGA"))
  {
    return neo8m_parse_gga(gps, nmea);
  }
  if (strstr(nmea, "$GPRMC") || strstr(nmea, "$GNRMC"))
  {
    return neo8m_parse_rmc(gps, nmea);
  }
  return false;  
}

static bool neo8m_parse_gga(neo8m_handle_t *gps, const char *sentence)
{
  char buffer[NMEA_MAX_LENGTH];
  strncpy(buffer, sentence, sizeof(buffer) - 1);
  char *token = strtok(buffer, ",");
  uint8_t field = 0;

  char time_str[16] = {0};
  char lat_str[16] = {0};
  char lat_dir = 'N';
  char lon_str[16] = {0};
  char lon_dir = 'E';
  uint8_t fix_quality = 0;
  uint8_t satellites = 0;
  char hdop_str[8] = {0};
  char alt_str[16] = {0};

  while (token != NULL)
  {
    switch (field)
    {
    case 1: strncpy(time_str, token, sizeof(time_str) - 1); break;
    case 2: strncpy(lat_str, token, sizeof(lat_str) - 1); break;
    case 3: lat_dir = token[0]; break;
    case 4: strncpy(lon_str, token, sizeof(lon_str) - 1); break;
    case 5: lon_dir = token[0]; break;
    case 6: fix_quality = atoi(token); break;
    case 7: satellites = atoi(token); break;
    case 8: strncpy(hdop_str, token, sizeof(hdop_str) - 1); break;
    case 9: strncpy(alt_str, token, sizeof(alt_str) - 1); break;
    
    default:
      break;
    }
    token = strtok(NULL, ",");
    field++;
  }
  //update data
  if (strlen(time_str) >= 6)
  {
    neo8m_parse_time(time_str, &gps->data.time);
  }
  if (strlen(lat_str) > 0 && strlen(lon_str) > 0)
  {
    gps->data.latitude = neo8m_parse_coordinate(lat_str, lat_dir);
    gps->data.longitude = neo8m_parse_coordinate(lon_str, lon_dir);
  }
  gps->data.satellites = satellites;
  gps->data.hdop = atof(hdop_str);
  gps->data.altitude = atof(alt_str);

  if (fix_quality > 0)
  {
    gps->data.fix = (fix_quality == 1) ? NEO8M_FIX_3D : NEO8M_FIX_2D;
    gps->data.valid = true;
  } else {
    gps->data.fix = NEO8M_FIX_NONE;
    gps->data.valid = false;
  }
  return true;  
}

static bool neo8m_parse_rmc(neo8m_handle_t *gps, const char *sentence)
{
  char buffer[NMEA_MAX_LENGTH];
  strncpy(buffer, sentence, sizeof(buffer) - 1);

  char *token = strtok(buffer, ",");
  uint8_t field = 0;
  char time_str[16] = {0};
  char status = 'V';
  char lat_str[16] = {0};
  char lat_dir = 'N';
  char lon_str[16] = {0};
  char lon_dir = 'E';
  char speed_str[16] = {0};
  char course_str[16] = {0};
  char date_str[16] =  {0};

  while (token != NULL)
  {
    switch (field)
    {
    case 1: strncpy(time_str, token, sizeof(time_str) - 1); break;
    case 2: status = token[0]; break;
    case 3: strncpy(lat_str, token, sizeof(lat_str) - 1); break;
    case 4: lat_dir = token[0]; break;
    case 5: strncpy(lon_str, token, sizeof(lon_str) - 1); break;
    case 6: lon_dir = token[0]; break;
    case 7: strncpy(speed_str, token, sizeof(speed_str) - 1); break;
    case 8: strncpy(course_str, token, sizeof(course_str) - 1); break;
    case 9: strncpy(date_str, token, sizeof(date_str) - 1); break;
    
    default:
      break;
    }
    token = strtok(NULL, ",");
    field++;
  }

  //update data
  if (status == 'A')
  {
    if (strlen(time_str) >= 6)
    {
      neo8m_parse_time(time_str, &gps->data.time);
    }
    if (strlen(date_str) >= 6)
    {
      neo8m_parse_date(date_str, &gps->data.date);
    }
    if (strlen(lat_str) > 0 && strlen(lon_str) > 0)
    {
      gps->data.latitude = neo8m_parse_coordinate(lat_str, lat_dir);
      gps->data.longitude = neo8m_parse_coordinate(lon_str, lon_dir);
    }

    gps->data.speed = atof(speed_str) * 1.852f;
    gps->data.course = atof(course_str);
    gps->data.valid = true;

    return true;    
  }
  return false;
  
}

static uint8_t neo8m_calculate_check_sum(const char *sentence)
{
  uint8_t checksum = 0;

  //start after '$', and before '*'
  for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
        checksum ^= sentence[i];
  }
  return checksum;
  
}

static bool neo8m_verify_check_sum(const char *sentence)
{
  const char *asterisk = strchr(sentence, '*');
  if (asterisk == NULL)
  {
    return false;
  }
  uint8_t calculated = neo8m_calculate_check_sum(sentence);
  uint8_t received = (uint8_t) strtol(asterisk + 1, NULL, 16);

  return (calculated == received);
  
}

static double neo8m_parse_coordinate(const char *coord, char dir)
{
  if (strlen(coord) < 4)
  {
    return 0.0;
  }
  //nmea format: ddmm.mmmm or dddm.mmmm
  double value = atof(coord);
  //extract degrees (first 2 or 3 digits)
  int degrees;
  if (value >= 10000) //longtitude
  {
    degrees = (int) (value/100);
  } else { //latitude
    degrees = (int) (value/100);
  }
  // Extract minutes
    double min = value - (degrees * 100);
    
    // Convert to decimal degrees
    double result = degrees + (min / 60.0);
    
    // Apply direction
    if (dir == 'S' || dir == 'W') {
        result = -result;
    }
    
    return result;
}

static void neo8m_parse_time(const char *time_str, neo8m_time_t *time)
{
  //format: hhmmss.sss
  if (strlen(time_str) < 6)
  {
    return;
  }
  char buf[3];
  buf[0] = time_str[0];
  buf[1] = time_str[1];
  buf[2] = '\0';
  time->hour = atoi(buf);

  buf[0] = time_str[2];
  buf[1] = time_str[3];
  time->minute = atoi(buf);

  buf[0] = time_str[4];
  buf[1] = time_str[5];
  time->second = atoi(buf);

  //milliseconds if available
  if (strlen(time_str) > 7)
  {
    time->millisecond = (uint16_t)(atof(time_str + 6) * 1000);
  }
}

static void neo8m_parse_date(const char *date_str, neo8m_date_t *date)
{
  //format: ddmmyy
  if (strlen(date_str) < 6) return;
  char buf[3];
  buf[0] = date_str[0];
  buf[1] = date_str[1];
  buf[2] = '\0';
  date->day = atoi(buf);

  buf[0] = date_str[2];
  buf[1] = date_str[3];
  date->month = atoi(buf);

  buf[0] = date_str[4];
  buf[1] = date_str[5];
  date->year = 2000 + atoi(buf);
}