#include "bmp388_oop.h"
#include <math.h>

static HAL_StatusTypeDef BMP388_ReadRegs(BMP388_Device *self, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(self->hi2c, self->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

static HAL_StatusTypeDef BMP388_WriteRegs(BMP388_Device *self, uint8_t reg, const uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Write(self->hi2c, self->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, 100);
}

static HAL_StatusTypeDef BMP388_SoftReset(BMP388_Device *self) {
    uint8_t rst_cmd = BMP388_SOFTRESET;
    uint8_t status = 0;
    
    HAL_StatusTypeDef rslt = BMP388_ReadRegs(self, BMP3_REG_STATUS, &status, 1);
    if ((rslt == HAL_OK) && (status & BMP3_CMD_RDY)) {
        rslt = BMP388_WriteRegs(self, BMP3_REG_CMD, &rst_cmd, 1);
        if (rslt == HAL_OK) {
            HAL_Delay(5);
            uint8_t err = 0;
            rslt = BMP388_ReadRegs(self, BMP3_REG_ERR_REG, &err, 1);
            if (err & 0x01) {
                return HAL_ERROR;
            }
        }
    }
    return rslt;
}

static HAL_StatusTypeDef BMP388_GetCalibData(BMP388_Device *self) {
    uint8_t calib_buff[BMP3_CALIBDATA_LEN] = {0};
    HAL_StatusTypeDef rslt = BMP388_ReadRegs(self, BMP3_REG_CALIB_DATA, calib_buff, BMP3_CALIBDATA_LEN);
    if (rslt != HAL_OK) return rslt;
    
    uint16_t raw_par_t1 = ((uint16_t)calib_buff[1] << 8) | (uint16_t)calib_buff[0];
    uint16_t raw_par_t2 = ((uint16_t)calib_buff[3] << 8) | (uint16_t)calib_buff[2];
    int8_t   raw_par_t3 = (int8_t)calib_buff[4];
    int16_t  raw_par_p1 = ((int16_t)calib_buff[6] << 8) | (int16_t)calib_buff[5];
    int16_t  raw_par_p2 = ((int16_t)calib_buff[8] << 8) | (int16_t)calib_buff[7];
    int8_t   raw_par_p3 = (int8_t)calib_buff[9];
    int8_t   raw_par_p4 = (int8_t)calib_buff[10];
    uint16_t raw_par_p5 = ((uint16_t)calib_buff[12] << 8) | (uint16_t)calib_buff[11];
    uint16_t raw_par_p6 = ((uint16_t)calib_buff[14] << 8) | (uint16_t)calib_buff[13];
    int8_t   raw_par_p7 = (int8_t)calib_buff[15];
    int8_t   raw_par_p8 = (int8_t)calib_buff[16];
    int16_t  raw_par_p9 = ((int16_t)calib_buff[18] << 8) | (int16_t)calib_buff[17];
    int8_t   raw_par_p10 = (int8_t)calib_buff[19];
    int8_t   raw_par_p11 = (int8_t)calib_buff[20];
    
    self->calib.par_t1 = (float)raw_par_t1 / 0.00390625f;
    self->calib.par_t2 = (float)raw_par_t2 / 1073741824.0f;
    self->calib.par_t3 = (float)raw_par_t3 / 281474976710656.0f;
    self->calib.par_p1 = ((float)raw_par_p1 - 16384.0f) / 1048576.0f;
    self->calib.par_p2 = ((float)raw_par_p2 - 16384.0f) / 536870912.0f;
    self->calib.par_p3 = (float)raw_par_p3 / 4294967296.0f;
    self->calib.par_p4 = (float)raw_par_p4 / 137438953472.0f;
    self->calib.par_p5 = (float)raw_par_p5 / 0.125f;
    self->calib.par_p6 = (float)raw_par_p6 / 64.0f;
    self->calib.par_p7 = (float)raw_par_p7 / 256.0f;
    self->calib.par_p8 = (float)raw_par_p8 / 32768.0f;
    self->calib.par_p9 = (float)raw_par_p9 / 281474976710656.0f;
    self->calib.par_p10 = (float)raw_par_p10 / 281474976710656.0f;
    self->calib.par_p11 = (float)raw_par_p11 / 36893488147419103232.0f;
    
    return HAL_OK;
}

static float BMP388_CompensateTemp(BMP388_Device *self, uint32_t raw_temp) {
    float partial_data1 = (float)(raw_temp - self->calib.par_t1);
    float partial_data2 = (float)(partial_data1 * self->calib.par_t2);
    return partial_data2 + (partial_data1 * partial_data1) * self->calib.par_t3;
}

static float BMP388_CompensatePress(BMP388_Device *self, float temp, uint32_t raw_press) {
    float partial_data1 = self->calib.par_p6 * temp;
    float partial_data2 = self->calib.par_p7 * (temp * temp);
    float partial_data3 = self->calib.par_p8 * (temp * temp * temp);
    float partial_out1 = self->calib.par_p5 + partial_data1 + partial_data2 + partial_data3;
    
    partial_data1 = self->calib.par_p2 * temp;
    partial_data2 = self->calib.par_p3 * (temp * temp);
    partial_data3 = self->calib.par_p4 * (temp * temp * temp);
    float partial_out2 = (float)raw_press * (self->calib.par_p1 + partial_data1 + partial_data2 + partial_data3);
    
    partial_data1 = (float)raw_press * (float)raw_press;
    partial_data2 = self->calib.par_p9 + self->calib.par_p10 * temp;
    partial_data3 = partial_data1 * partial_data2;
    float partial_data4 = partial_data3 + ((float)raw_press * (float)raw_press * (float)raw_press) * self->calib.par_p11;
    
    return partial_out1 + partial_out2 + partial_data4;
}

static HAL_StatusTypeDef BMP388_Init_Impl(BMP388_Device *self) {
    uint8_t chip_id = 0;
    
    HAL_StatusTypeDef rslt = BMP388_ReadRegs(self, BMP3_REG_CHIP_ID, &chip_id, 1);
    if (rslt != HAL_OK || chip_id != BMP3_CHIP_ID) {
        return HAL_ERROR;
    }
    
    rslt = BMP388_SoftReset(self);
    if (rslt != HAL_OK) return rslt;
    
    rslt = BMP388_GetCalibData(self);
    if (rslt != HAL_OK) return rslt;
    
    self->osr = (BMP388_OVERSAMPLING_2X << 3) | BMP388_OVERSAMPLING_8X;
    rslt = BMP388_WriteRegs(self, BMP3_REG_OSR, &self->osr, 1);
    if (rslt != HAL_OK) return rslt;
    
    self->odr = BMP3_ODR_50_HZ;
    rslt = BMP388_WriteRegs(self, BMP3_REG_ODR, &self->odr, 1);
    if (rslt != HAL_OK) return rslt;
    
    self->iir = BMP3_IIR_FILTER_COEFF_3 << 1;
    rslt = BMP388_WriteRegs(self, BMP3_REG_CONFIG, &self->iir, 1);
    if (rslt != HAL_OK) return rslt;
    
    uint8_t pwr_ctrl = 0x01 | 0x02 | (0x03 << 4);
    rslt = BMP388_WriteRegs(self, BMP3_REG_PWR_CTRL, &pwr_ctrl, 1);
    if (rslt != HAL_OK) return rslt;
    
    return HAL_OK;
}

static HAL_StatusTypeDef BMP388_ReadTempPress_Impl(BMP388_Device *self, float *temperature, float *pressure) {
    uint8_t raw_data[6] = {0};
    
    HAL_StatusTypeDef rslt = BMP388_ReadRegs(self, BMP3_REG_DATA_0, raw_data, 6);
    if (rslt != HAL_OK) return rslt;
    
    uint32_t raw_press = ((uint32_t)raw_data[2] << 16) | ((uint32_t)raw_data[1] << 8) | raw_data[0];
    uint32_t raw_temp = ((uint32_t)raw_data[5] << 16) | ((uint32_t)raw_data[4] << 8) | raw_data[3];
    
    *temperature = BMP388_CompensateTemp(self, raw_temp);
    *pressure = BMP388_CompensatePress(self, *temperature, raw_press);
    
    return HAL_OK;
}

static float BMP388_FindAltitude_Impl(BMP388_Device *self, float ground_pressure, float pressure) {
    return 44330.0f * (1.0f - powf(pressure / ground_pressure, 0.1903f));
}

void BMP388_Create(BMP388_Device *self, I2C_HandleTypeDef *hi2c, uint8_t dev_addr) {
    self->hi2c = hi2c;
    self->dev_addr = dev_addr;
    
    self->Init = BMP388_Init_Impl;
    self->ReadTempPress = BMP388_ReadTempPress_Impl;
    self->FindAltitude = BMP388_FindAltitude_Impl;
}
