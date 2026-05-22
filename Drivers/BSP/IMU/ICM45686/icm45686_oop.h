#ifndef __ICM45686_OOP_H
#define __ICM45686_OOP_H

#include "main.h"
#include "inv_imu_driver.h"

struct ICM45686_Device;

typedef struct ICM45686_Device {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    
    inv_imu_device_t imu_dev;
    float gyro_offset[3];
    
    int (*Init)(struct ICM45686_Device *self);
    int (*ReadData)(struct ICM45686_Device *self, float accel[3], float gyro[3], float *temp);
    void (*Calibrate)(struct ICM45686_Device *self);
} ICM45686_Device;

void ICM45686_Create(ICM45686_Device *self, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

#endif /* __ICM45686_OOP_H */
