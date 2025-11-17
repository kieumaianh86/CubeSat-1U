#ifndef INA219_H
#define INA219_H

#include "stdio.h"
#include "stm32h7xx_hal.h"

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
const uint8_t ina_reg_config          =      (0x00);
const uint8_t ina_reg_shuntvoltage    =      (0x01);
const uint8_t ina_reg_bus_voltage     =      (0x02);
const uint8_t ina_reg_power           =      (0x03);
const uint8_t ina_reg_current         =      (0x04);
const uint8_t ina_reg_calibration     =      (0x05);

const uint8_t ina_config_reset        =      (0x8000);

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

typedef struct {
    I2C_HandleTypeDef *ina219_i2c;
    uint8_t Address;
    battery_state check;
} ina219_t;

typedef enum {
    battery_START,
    battery_OK,
    battery_LOW
} battery_state;

uint16_t ina219_ReadBusVoltage(ina219_t *ina219);
int16_t ina219_ReadCurrent(ina219_t *ina219);
int16_t ina219_ReadShuntVoltage(ina219_t *ina219);
float ina219_ReadPower(ina219_t *ina219);


void ina219_reset(ina219_t *ina219);
void ina219_setCalibration(ina219_t *ina219, uint16_t Calibration);
uint16_t ina219_getConfig(ina219_t *ina219);
void ina219_setConfig(ina219_t *ina219, uint16_t Config);
void ina219_Config(ina219_t *ina219);
void set_PowerMode(ina219_t *ina219, ina_config_operatingmode mode);
uint8_t ina219_init(ina219_t *ina219, I2C_HandleTypeDef *hi2c, uint8_t address);
float ina219_BatteryLife(ina219_t *ina219);
battery_state Checkbattery(ina219_t *ina219, float BatteryThresold);

typedef struct{
    float current_lsb;
    float power_lsb;
} lsb_t;

bool isFirst;

#endif