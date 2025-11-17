#include "INA219.h"

battery_state check;

float Rshunt = 0.1;  //Ohm

//1 cell pin 
float battery_max = 4200.0f;
float battery_min = 3000.0f;

// insert Max Expert Current
float MaxExpertCurrent = 2.0f;

lsb_t LSB;
void calculate_LSB(void) {
    LSB.current_lsb = MaxExpertCurrent /32768.0f;
    if(LSB.current_lsb *1e6f < 6.25f) {
        LSB.current_lsb = 6.5f / 1e6f;
    }
    LSB.power_lsb = 20.0f * LSB.current_lsb;
}
//calculate current_lsb and power_lsb

uint16_t calculate_calibration() {
    float result = 0.04096f / (LSB.current_lsb * Rshunt);
    result = truncf(result);
    return (uint16_t)result;
}
//Calculate calibrationcalibration

uint16_t read_16bit(ina219_t *ina219, uint8_t Register) {
    uint8_t value[2];
    HAL_I2C_Mem_Read(ina219->ina219_i2c, ina219->Address << 1, Register, 1, value, 2, 1000);
    //Read 2 byte from Register and add into Value array
    //Stop if error or after waiting 1000ms
    return ((value[0] << 8) | value[1]);
    //Shift value[0] by 8 bits and OR it with value[1]
}

HAL_StatusTypeDef write_16bit(ina219_t *ina219, uint8_t Register, uint16_t value){
    uint8_t data[2];
    data[0] = (value << 8) & 0xff;
    data[1] = value & 0xff;
    return HAL_I2C_Mem_Write(ina219->ina219_i2c, ina219->Address << 1, Register, 1, data, 2, 1000);
}

uint16_t ina219_ReadBusVoltage(ina219_t *ina219) {
    uint16_t result = read_16bit(ina219, ina_reg_bus_voltage);
    return (result >> 3) * 4;
    //Bus Voltage Register contents must be shifted right by three bits
    //result must be multiplied by the Bus Voltage LSB of 4mV
}

int16_t ina219_ReadCurrent(ina219_t *ina219) {
    int16_t result_raw = read_16bit(ina219, ina_reg_current);
    return (int16_t)((float)result_raw * LSB.current_lsb * 1000.0f);
    // current mA
}

int16_t ina219_ReadShuntVoltage(ina219_t *ina219) {
    int16_t result_raw = read_16bit(ina219, ina_reg_shuntvoltage); //have sign
    return (int16_t)((float)result_raw * 0.01f + 0.5f);
    // Shunt Voltage must be multiplied by the Shunt Voltage LSB of 10uV
    // Multiplied 0.01 : uV -> mV
}

float ina219_ReadPower(ina219_t *ina219) {
    uint16_t result_raw = read_16bit(ina219, ina_reg_power);
    return result_raw * LSB.power_lsb; //mW
}

void ina219_reset(ina219_t *ina219) {
    write_16bit(ina219, ina_reg_config, ina_config_reset);
    HAL_Delay(1);
}

void ina219_setCalibration(ina219_t *ina219, uint16_t calibration) {
    write_16bit(ina219, ina_reg_calibration, calibration);
}

uint16_t ina219_getConfig(ina219_t *ina219) {
    uint16_t result = read_16bit(ina219, ina_reg_config);
    return result;
}

void ina219_setConfig(ina219_t *ina219, uint16_t Config) {
    write_16bit(ina219, ina_reg_config, Config);
}

// Example: Set Config
void ina219_Config(ina219_t *ina219) {
    uint16_t Config = BRNG_32V | ina_pga_8_320 |
                                 ina_config_badc_12bit 
                                 | ina_config_shuntadc_12bit_532us 
                                 | ina_config_shuntandbus_continous;
    // BRND_32V, ina_pga_8_320, adc 12bit and shunt and bus continous mode is default
    // modify the realreal value
    calculate_LSB();
    uint16_t calibration = calculate_calibration();
    ina219_setCalibration(ina219, calibration);
    ina219_setConfig(ina219, Config);
}

void set_PowerMode(ina219_t *ina219, ina_config_operatingmode mode) {
    uint16_t config = ina219_getConfig(ina219);
    // get now config
    switch(mode) {
        case ina_config_powerdown:
        config = (config & ~ina_config_mode_mask) | (ina_config_powerdown & ina_config_mode_mask);
        ina219_setConfig(ina219, config);
        break;

        case ina_config_shuntvoltage_triggered:
        config = (config & ~ina_config_mode_mask) | (ina_config_shuntvoltage_triggered & ina_config_mode_mask);
        ina219_setConfig(ina219, config);
        break;

        case ina_config_busvoltage_triggered:
        config = (config & ~ina_config_mode_mask) | (ina_config_busvoltage_triggered & ina_config_mode_mask);
        ina219_setConfig(ina219, config);
        break;

        case ina_config_shuntandbus_triggered:
        config = (config & ~ina_config_mode_mask) | (ina_config_shuntandbus_triggered & ina_config_mode_mask);
        ina219_setConfig(ina219, config);
        break;

        case ina_config_adc_off:
        config = (config & ~ina_config_mode_mask) | (ina_config_adc_off & ina_config_mode_mask);
        ina219_setConfig(ina219, config);
        break;

        case ina_config_shuntvoltage_continous:
        config = (config & ~ina_config_mode_mask) | (ina_config_shuntvoltage_continous & ina_config_mode_mask);
        ina219_setConfig(ina219, config);
        break;

        case ina_config_busvoltage_continous:
        config = (config & ~ina_config_mode_mask) | (ina_config_busvoltage_continous & ina_config_mode_mask);
        ina219_setConfig(ina219, config);
        break;

        case ina_config_shuntandbus_continous:
        config = (config & ~ina_config_mode_mask) | (ina_config_shuntandbus_continous & ina_config_mode_mask);
        ina219_setConfig(ina219, config);
        break;

        default:
        break;
    }
}

uint8_t ina219_init(ina219_t *ina219, I2C_HandleTypeDef *hi2c, uint8_t address) {
    
    isFirst = false;
    ina219->ina219_i2c = hi2c;
    ina219->Address = address;

    uint8_t ina219_isReady = HAL_I2C_IsDeviceReady(hi2c, address << 1, 3, 2);

    if(ina219_isReady == HAL_OK) {
        ina219->check = battery_START;
        ina219_reset(ina219);
        ina219_Config(ina219);

        return 1;
    }
    else{
        return 0;
    }
}

float ina219_BatteryLife(ina219_t *ina219) {
    uint16_t Vbus = ina219_ReadBusVoltage(ina219);
    float percentLife = (Vbus - battery_min) / (battery_max - battery_min) * 100.0f;
    if (percentLife < 0.0f) {
        percentLife = 0.0f;
    }
    if(percentLife > 100.0f) {
        percentLife = 100.0f;
    }
    return percentLife;
}

//check battery
battery_state Checkbattery(ina219_t *ina219, float BatteryThresold) {
    float percentBattery = ina219_BatteryLife(ina219);
    switch(ina219->check) {
        case battery_START:
        if(percentBattery >= BatteryThresold) {
            ina219->check = battery_OK;
        }
        else {
            ina219->check = battery_LOW;
        }
        break;

        case battery_OK:
        if(percentBattery >= BatteryThresold) {
            ina219->check = battery_OK;
        }
        else {
            ina219->check = battery_LOW;
        }
        break;
        
        case battery_LOW:
        if(percentBattery >= BatteryThresold) {
            ina219->check = battery_OK;
        }
        else {
            ina219->check = battery_LOW;
        }
        break;

        default:
        ina219->check = battery_START;
    }
    return ina219->check;
}
