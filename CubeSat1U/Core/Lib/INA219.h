#ifndef INA219_H
#define INA219_H

#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "stdint.h"

typedef enum {
    Address_gnd_gnd = 0x40,
    Address_gnd_vcc = 0x41,
    Address_gnd_sda = 0x42,
    Address_gnd_scl = 0x43,
    Address_vcc_gnd = 0x44,
    Address_vcc_vcc = 0x45,
    Address_vcc_sda = 0x46,
    Address_vcc_scl = 0x47,
    Address_sda_gnd = 0x48,
    Address_sda_vcc = 0x49,
    Address_sda_sda = 0x4A,
    Address_sda_scl = 0x4B,
    Address_scl_gnd = 0x4C,
    Address_scl_vcc = 0x4D,
    Address_scl_sda = 0x4E,
    Address_scl_scl = 0x4F
} ina_address_A0_A1;

//Registor AddressAddress
#define ina_reg_config              0x00
#define ina_reg_shuntvoltage        0x01
#define ina_reg_bus_voltage         0x02
#define ina_reg_power               0x03
#define ina_reg_current             0x04
#define ina_reg_calibration         0x05

#define ina_config_reset            0x8000

typedef enum {
    BRNG_16V = 0x0000,
    BRNG_32V = 0x2000
} ina_config_busvoltagerange;

typedef enum {
    ina_pga_1_40 = 0x0000,
    ina_pga_2_80 = 0x0800,
    ina_pga_4_160 = 0x1000,
    ina_pga_8_320 = 0x1800
} ina_pga_gain_range;

typedef enum {
    ina_config_badc_9bit = 0x0000,
    ina_config_badc_10bit = 0x0080,
    ina_config_badc_11bit = 0x0100,
    ina_config_badc_12bit = 0x0180,
    ina_config_badc_12bit_2s_1060us = 0x0480,
    ina_config_badc_12bit_4s_2130us = 0x0500,
    ina_config_badc_12bit_8s_4260us = 0x0580,
    ina_config_badc_12bit_16s_8150us = 0x0600,
    ina_config_badc_12bit_32s_17020us = 0x0680,
    ina_config_badc_12bit_64s_35050us = 0x0700,
    ina_config_badc_12bit_128s_68100us = 0x0780
} ina_config_badc;                             //Bit 7-1010

typedef enum {
    ina_config_shuntadc_9bit_84us = 0x0000,
    ina_config_shuntadc_10bit_146us = 0x0008,
    ina_config_shuntadc_11bit_276us = 0x0010,
    ina_config_shuntadc_12bit_532us = 0x0018,
    ina_config_shuntadc_12bit_2s_1060us = 0x0048,
    ina_config_shuntadc_12bit_4s_2130us = 0x0050,
    ina_config_shuntadc_12bit_8s_4260us = 0x0058,
    ina_config_shuntadc_12bit_16s_8150us = 0x0060,
    ina_config_shuntadc_12bit_32s_17020us = 0x0068,
    ina_config_shuntadc_12bit_64s_35050us = 0x0070,
    ina_config_shuntadc_12bit_128s_68100us = 0x0780
} ina_config_shuntadc;

typedef enum {
    ina_config_powerdown = 0x00,
    ina_config_shuntvoltage_triggered = 0x01,
    ina_config_busvoltage_triggered = 0x02,
    ina_config_shuntandbus_triggered = 0x03,
    ina_config_adc_off = 0x04,
    ina_config_shuntvoltage_continous = 0x05,
    ina_config_busvoltage_continous = 0x06,
    ina_config_shuntandbus_continous = 0x07
} ina_config_operatingmode;

#define ina_config_mode_mask (0x07)

typedef union {
    uint16_t data;
    uint8_t bytes[2];
}reg16_t;

typedef enum {
    INA219_OK,
    INA219_ERROR,
    INA219_I2C_ERROR,
    INA219_INVALID_PARAM,
    INA219_BUS_OVER,
    INA219_BUS_UNDER,
    INA219_SHUNT_OVER
} INA219_STATUS_t;

typedef enum {
    battery_START,
    battery_OK,
    battery_LOW,
    battery_CRITICAL,
    battery_EMERGENCY
} battery_state;

typedef struct {
    I2C_HandleTypeDef *ina219_i2c;
    uint8_t Address;

    uint16_t BusVoltage;
    int16_t ShuntVoltage;
    int16_t Current;
    uint16_t Power;

    battery_state check;
} ina219_t;


INA219_STATUS_t ina219_ReadBusVoltage(ina219_t *ina219);
INA219_STATUS_t ina219_ReadCurrent(ina219_t *ina219);
INA219_STATUS_t ina219_ReadShuntVoltage(ina219_t *ina219);
INA219_STATUS_t ina219_ReadPower(ina219_t *ina219);


INA219_STATUS_t ina219_reset(ina219_t *ina219);
INA219_STATUS_t ina219_setCalibration(ina219_t *ina219, uint16_t Calibration);
INA219_STATUS_t ina219_getConfig(ina219_t *ina219, uint16_t *Config);
INA219_STATUS_t ina219_setConfig(ina219_t *ina219, uint16_t Config);
INA219_STATUS_t ina219_Config(ina219_t *ina219);
INA219_STATUS_t set_PowerMode(ina219_t *ina219, ina_config_operatingmode mode);
INA219_STATUS_t ina219_init(ina219_t *ina219, I2C_HandleTypeDef *hi2c, uint8_t address);
float ina219_BatteryLife(ina219_t *ina219);
battery_state Checkbattery(ina219_t *ina219);

#define RSHUNT 0.1
#define MAX_EXPERT_CURRENT 2
#define CURRENT_LSB 65 //CURRENT LSB = MAX EXPERT CURRENT / RSHUNT
#define POWER_LSB 1300 //POWER LSB = 20 * CURRENT LSB
#define SHUNTVOLTAGE_LSB 10
#define CALIBRATION 6301 // CALIBRATION = trunc(0.04096/(CURRENT LSB * RSHUNT))
#define BATTERY_MAX 4200
#define BATTERY_MIN 3000
#define BUS_THRESHOLD_OVER 4250
#define BUS_THRESHOLD_UNDER 3000
#define BATTERY_EMPTY 0
#define BATTERY_FULL 100
#define BATTERY_OK_THRESHOLD 70
#define BATTERY_LOW_THRESHOLD 30
#define BATTERY_CRITICAL_THRESHOLD 10
#define MILLI_TO_MICRO 1000
#define PERCENT 100

#endif