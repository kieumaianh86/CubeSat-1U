#include "INA219.h"

INA219_STATUS_t read_16bit(ina219_t *ina219, uint8_t Register, uint16_t *value) {
    if(!ina219) return INA219_INVALID_PARAM;
    if(!ina219->ina219_i2c) return INA219_I2C_ERROR;

    reg16_t reg;

    if (HAL_I2C_Mem_Read(ina219 -> ina219_i2c, ina219 -> Address << 1, Register, 1, reg.bytes, 2, 1000) != HAL_OK)
    {
        return INA219_I2C_ERROR;
    }
    else {
        *value = reg.data;
        return INA219_OK;
    }
}

INA219_STATUS_t write_16bit(ina219_t *ina219, uint8_t Register, uint16_t value){
    if(!ina219) return INA219_INVALID_PARAM;
    if(!ina219->ina219_i2c) return INA219_I2C_ERROR;
    reg16_t reg;
    reg.data = value;
    if(HAL_I2C_Mem_Write(ina219->ina219_i2c, ina219->Address << 1, Register, 1, reg.bytes, 2, 1000) != HAL_OK)
    {
        return INA219_I2C_ERROR;
    }
    return INA219_OK;
}

INA219_STATUS_t ina219_ReadBusVoltage(ina219_t *ina219) {
    if(!ina219) return INA219_INVALID_PARAM;

    uint16_t result_raw = 0;
    if(read_16bit(ina219, ina_reg_bus_voltage, &result_raw) != INA219_OK) return INA219_I2C_ERROR;
    ina219->BusVoltage = ((result_raw >> 3) * 4);
    //Bus Voltage Register contents must be shifted right by three bits
    //result must be multiplied by the Bus Voltage LSB of 4mV
    if (ina219->BusVoltage > BUS_THRESHOLD_OVER) return INA219_BUS_OVER;
    if (ina219->BusVoltage < BUS_THRESHOLD_UNDER) return INA219_BUS_UNDER;

    return INA219_OK;
}

INA219_STATUS_t ina219_ReadCurrent(ina219_t *ina219) {
    if(!ina219) return INA219_INVALID_PARAM;

    uint16_t result_raw = 0;
    if(read_16bit(ina219, ina_reg_current, &result_raw) != INA219_OK) return INA219_I2C_ERROR;
    int16_t result = (int16_t)(result_raw * CURRENT_LSB * MILLI_TO_MICRO);
    ina219->Current = result;
    return INA219_OK;
}

INA219_STATUS_t ina219_ReadShuntVoltage(ina219_t *ina219) {
    if(!ina219) return INA219_INVALID_PARAM;

    uint16_t result_raw = 0;
    if(read_16bit(ina219, ina_reg_shuntvoltage, &result_raw) != INA219_OK) return INA219_I2C_ERROR;
    int16_t result = (int16_t)(result_raw * SHUNTVOLTAGE_LSB / MILLI_TO_MICRO);
    ina219->ShuntVoltage = result;
    if(ina219->ShuntVoltage > MAX_EXPERT_CURRENT * MILLI_TO_MICRO) return INA219_SHUNT_OVER;
    
    return INA219_OK;
}

INA219_STATUS_t ina219_ReadPower(ina219_t *ina219) {
    if(!ina219) return INA219_ERROR;

    uint16_t result_raw = 0;
    if(read_16bit(ina219, ina_reg_power, &result_raw) != INA219_OK) return INA219_I2C_ERROR;
    ina219->Power = result_raw * POWER_LSB;
    return INA219_OK;
}

INA219_STATUS_t ina219_reset(ina219_t *ina219) {
    if(!ina219) return INA219_INVALID_PARAM;

    if(write_16bit(ina219, ina_reg_config, ina_config_reset) != INA219_OK) return INA219_I2C_ERROR;
    
    HAL_Delay(1);
    return INA219_OK;
}

INA219_STATUS_t ina219_setCalibration(ina219_t *ina219, uint16_t calibration) {
    if(!ina219) return INA219_INVALID_PARAM;

    if(write_16bit(ina219, ina_reg_calibration, calibration) != INA219_OK) return INA219_I2C_ERROR;
    return INA219_OK;
}

INA219_STATUS_t ina219_getConfig(ina219_t *ina219, uint16_t *Config) {
    if(!ina219) return INA219_INVALID_PARAM;

    if(read_16bit(ina219, ina_reg_config, Config) != INA219_OK) return INA219_I2C_ERROR;
    return INA219_OK;
}

INA219_STATUS_t ina219_setConfig(ina219_t *ina219, uint16_t Config) {
    if(!ina219) return INA219_INVALID_PARAM;
    
    if(write_16bit(ina219, ina_reg_config, Config) != INA219_OK) return INA219_I2C_ERROR;
    
    return INA219_OK;
}

// Example: Set Config
INA219_STATUS_t ina219_Config(ina219_t *ina219) {
    if(!ina219) return INA219_INVALID_PARAM;

    uint16_t Config = BRNG_32V | ina_pga_8_320
                                 | ina_config_badc_12bit 
                                 | ina_config_shuntadc_12bit_532us 
                                 | ina_config_shuntandbus_continous;
    // BRND_32V, ina_pga_8_320, adc 12bit and shunt and bus continous mode is default
    // modify the real value
    if(ina219_setCalibration(ina219, CALIBRATION) != INA219_OK) return INA219_ERROR;
    if(ina219_setConfig(ina219, Config) != INA219_OK) return INA219_ERROR;
    
    return INA219_OK;
}

INA219_STATUS_t set_PowerMode(ina219_t *ina219, ina_config_operatingmode mode) {
    if(!ina219) return INA219_INVALID_PARAM;

    if(mode > ina_config_shuntandbus_continous) return INA219_INVALID_PARAM;

    uint16_t config;
    if(ina219_getConfig(ina219, &config) != INA219_OK) return INA219_ERROR;
    
    config = (config & ~ina_config_mode_mask) | (mode & ina_config_mode_mask);

    return ina219_setConfig(ina219, config);
}

INA219_STATUS_t ina219_init(ina219_t *ina219, I2C_HandleTypeDef *hi2c, uint8_t address) {
    if(!ina219 || !hi2c || !address) return INA219_INVALID_PARAM;
    
    ina219->ina219_i2c = hi2c;
    ina219->Address = address;

    // CHECK I2C DEVICE 
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, address << 1, 3, 2);
    if(status != HAL_OK) return INA219_I2C_ERROR;

    ina219->check = battery_START;

    // Reset INA219
    if(ina219_reset(ina219) != INA219_OK) return INA219_ERROR;

    // Config INA219
    if(ina219_Config(ina219) != INA219_OK) return INA219_ERROR;

    return INA219_OK;
}

float ina219_BatteryLife(ina219_t *ina219) {
    if(!ina219) return 0;

    if(ina219_ReadBusVoltage(ina219) != INA219_OK) return 0;

    uint16_t Vbus = ina219->BusVoltage;
    float percentLife = (float)(Vbus - BATTERY_MIN) / (float)(BATTERY_MAX - BATTERY_MIN) * PERCENT;
    if (percentLife < BATTERY_EMPTY) {
        percentLife = BATTERY_EMPTY;
    }
    if(percentLife > BATTERY_FULL) {
        percentLife = BATTERY_FULL;
    }
    return percentLife;
}

//check battery
battery_state Checkbattery(ina219_t *ina219) {
    if(!ina219) return battery_START;

    float percentBattery = ina219_BatteryLife(ina219);
    
    if(percentBattery >= BATTERY_OK_THRESHOLD) {
        ina219->check = battery_OK;
    }
    else if(percentBattery >= BATTERY_LOW_THRESHOLD) {
        ina219->check = battery_LOW;
    }
    else if(percentBattery >= BATTERY_CRITICAL_THRESHOLD) {
        ina219->check = battery_CRITICAL;
    }
    else {
        ina219->check = battery_EMERGENCY;
    }
    
    return ina219->check;
}

