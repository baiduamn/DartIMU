#ifndef __TEN_AXIS_IMU_H
#define __TEN_AXIS_IMU_H

#include "main.h"
#include "icm45686_oop.h"
#include "bmp388_oop.h"
#include "qmc5883_oop.h"

struct TenAxisIMU;

typedef struct TenAxisIMU {
    ICM45686_Device imu;
    BMP388_Device baro;
    QMC5883_Device mag;
    
    float q[4];
    float yaw;
    float pitch;
    float roll;
    
    float baseline_pressure;
    float raw_pressure;
    float raw_temperature;
    float altitude;
    
    float accel[3];
    float gyro[3];
    float imu_temp;
    QMC5883_MagData mag_raw;
    QMC5883_MagData mag_cal;
    
    uint32_t last_update_ticks;
    
    HAL_StatusTypeDef (*Init)(struct TenAxisIMU *self);
    HAL_StatusTypeDef (*Update)(struct TenAxisIMU *self);
    void (*GetEuler)(struct TenAxisIMU *self, float *yaw, float *pitch, float *roll);
    float (*GetAltitude)(struct TenAxisIMU *self);
} TenAxisIMU;

void TenAxisIMU_Create(TenAxisIMU *self, 
                       SPI_HandleTypeDef *hspi_imu, GPIO_TypeDef *cs_port_imu, uint16_t cs_pin_imu,
                       I2C_HandleTypeDef *hi2c_sensors);

#endif /* __TEN_AXIS_IMU_H */
