#ifndef WEERSTATION_TYPES_H
#define WEERSTATION_TYPES_H

typedef enum
{
  WindDir_N = 0, WindDir_NNE, WindDir_NE, WindDir_ENE, WindDir_E, WindDir_ESE, WindDir_SE, WindDir_SSE,
  WindDir_S, WindDir_SSW, WindDir_SW, WindDir_WSW, WindDir_W, WindDir_WNW, WindDir_NW, WindDir_NNW,
  WindDir_MAX
} WindDir;

typedef struct
{
  float   temp_c;
  float   hum_rh;
  float   wind_kmh;
  float   gust_kmh;
  float   rain_cum_mm;
  WindDir wind_dir;  
} SensorData;


typedef struct
{
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t year;   // minus 2000
  uint8_t mon;
  uint8_t day;
} DcfData;

typedef enum
{
  MsgType_Sensor = 10,
  MsgType_DCF    = 11
} MsgType;

#endif //  WEERSTATION_TYPES_H
