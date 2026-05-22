#include "qmc5883_oop.h"
#include <stdio.h>

static HAL_StatusTypeDef QMC5883P_ReadRegs(QMC5883_Device *self, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(self->hi2c, self->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

static HAL_StatusTypeDef QMC5883P_WriteRegs(QMC5883_Device *self, uint8_t reg, const uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Write(self->hi2c, self->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, 100);
}

static HAL_StatusTypeDef QMC5883_Init_Impl(QMC5883_Device *self) {
    uint8_t chip_id = 0;
    
    HAL_StatusTypeDef rslt = QMC5883P_ReadRegs(self, QMC5883P_REG_CHIP_ID, &chip_id, 1);
    if (rslt != HAL_OK || chip_id != QMC5883P_ID) {
        printf("QMC5883P Chip ID Error: 0x%02X\r\n", chip_id);
        return HAL_ERROR;
    }
    
    uint8_t val = 0x80;
    rslt = QMC5883P_WriteRegs(self, QMC5883P_REG_CTRL2, &val, 1);
    if (rslt != HAL_OK) return rslt;
    HAL_Delay(10);
    
    val = 0x01;
    rslt = QMC5883P_WriteRegs(self, QMC5883P_REG_CTRL2, &val, 1);
    if (rslt != HAL_OK) return rslt;
    
    val = 0x0F;
    rslt = QMC5883P_WriteRegs(self, QMC5883P_REG_CTRL1, &val, 1);
    if (rslt != HAL_OK) return rslt;
    
    return HAL_OK;
}

static HAL_StatusTypeDef QMC5883_ReadRaw_Impl(QMC5883_Device *self, QMC5883_MagData *data) {
    uint8_t buffer[6] = {0};
    
    HAL_StatusTypeDef rslt = QMC5883P_ReadRegs(self, QMC5883P_REG_XOUT_L, buffer, 6);
    if (rslt != HAL_OK) return rslt;
    
    int16_t raw_mx = (int16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
    int16_t raw_my = (int16_t)(((uint16_t)buffer[3] << 8) | buffer[2]);
    int16_t raw_mz = (int16_t)(((uint16_t)buffer[5] << 8) | buffer[4]);
    
    const float sensitivity = 15000.0f;
    data->x = (float)raw_mx / sensitivity;
    data->y = (float)raw_my / sensitivity;
    data->z = (float)raw_mz / sensitivity;
    
    return HAL_OK;
}

static HAL_StatusTypeDef QMC5883_ReadCalibrated_Impl(QMC5883_Device *self, QMC5883_MagData *data) {
    HAL_StatusTypeDef rslt = self->ReadRaw(self, data);
    if (rslt != HAL_OK) return rslt;
    
    data->x -= self->offset_x;
    data->y -= self->offset_y;
    data->z -= self->offset_z;
    
    return HAL_OK;
}

static void QMC5883_Calibrate_Impl(QMC5883_Device *self, uint32_t cal_time_ms) {
    QMC5883_MagData current_mag_data;
    
    float mag_max_x = -1000.0f, mag_min_x = 1000.0f;
    float mag_max_y = -1000.0f, mag_min_y = 1000.0f;
    float mag_max_z = -1000.0f, mag_min_z = 1000.0f;
    
    printf("QMC5883P Calibrating...\r\n");
    uint32_t start_time = HAL_GetTick();
    
    while ((HAL_GetTick() - start_time) < cal_time_ms) {
        if (self->ReadRaw(self, &current_mag_data) == HAL_OK) {
            if (current_mag_data.x > mag_max_x) mag_max_x = current_mag_data.x;
            if (current_mag_data.x < mag_min_x) mag_min_x = current_mag_data.x;
            
            if (current_mag_data.y > mag_max_y) mag_max_y = current_mag_data.y;
            if (current_mag_data.y < mag_min_y) mag_min_y = current_mag_data.y;
            
            if (current_mag_data.z > mag_max_z) mag_max_z = current_mag_data.z;
            if (current_mag_data.z < mag_min_z) mag_min_z = current_mag_data.z;
        }
        HAL_Delay(50);
    }
    
    self->offset_x = (mag_max_x + mag_min_x) / 2.0f;
    self->offset_y = (mag_max_y + mag_min_y) / 2.0f;
    self->offset_z = (mag_max_z + mag_min_z) / 2.0f;
    
    printf("QMC5883P Offsets: X:%d/100, Y:%d/100, Z:%d/100\r\n", 
           (int)(self->offset_x * 100.0f), (int)(self->offset_y * 100.0f), (int)(self->offset_z * 100.0f));
}

void QMC5883_Create(QMC5883_Device *self, I2C_HandleTypeDef *hi2c, uint8_t dev_addr) {
    self->hi2c = hi2c;
    self->dev_addr = dev_addr;
    self->offset_x = 0.0f;
    self->offset_y = 0.0f;
    self->offset_z = 0.0f;
    
    self->Init = QMC5883_Init_Impl;
    self->ReadRaw = QMC5883_ReadRaw_Impl;
    self->ReadCalibrated = QMC5883_ReadCalibrated_Impl;
    self->Calibrate = QMC5883_Calibrate_Impl;
}
