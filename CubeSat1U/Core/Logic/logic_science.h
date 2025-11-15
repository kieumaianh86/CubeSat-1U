#ifndef LOGIC_SCIENCE_H
#define LOGIC_SCIENCE_H

#include "axis.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "neo8m.h"

typedef enum {
  SCIENCE_OK = 0,
  SCIENCE_ERROR,
  SCIENCE_IN_PROGRESS
} science_status_t;


#endif