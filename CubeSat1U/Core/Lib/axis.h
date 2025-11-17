#ifndef AXIS_H
#define AXIS_H

#include "stdio.h"
#include "stdint.h"
#include "stm32h7xx_hal.h"

typedef struct 
{
  int32_t x;
  int32_t y;
  int32_t z;
}axis3_t;

typedef struct
{
  float x;
  float y;
  float z;
}faxis3_t;


#endif