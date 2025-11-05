#ifndef AXIS_H
#define AXIS_H

#include "stdio.h"
#include "stdint.h"
#include "stm32h7xx_hal.h"

typedef struct 
{
  int16_t x;
  int16_t y;
  int16_t z;
}axis3_t;

typedef struct
{
  float x;
  float y;
  float z;
}faxis3_t;


#endif