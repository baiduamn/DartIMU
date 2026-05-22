#ifndef __QMC5883_OOP_H
#define __QMC5883_OOP_H

#include "main.h"

#define QMC5883P_ADDR               (0x2C << 1)

#define QMC5883P_REG_CHIP_ID        0x00
#define QMC5883P_REG_XOUT_L         0x01
#define QMC5883P_REG_XOUT_H         0x02
#define QMC5883P_REG_YOUT_L         0x03
#define QMC5883P_REG_YOUT_H         0x04
#define QMC5883P_REG_ZOUT_L         0x05
#define QMC5883P_REG_ZOUT_H         0x06
#define QMC5883P_REG_STATUS         0x09
#define QMC5883P_REG_CTRL1          0x0A
#define QMC5883P_REG_CTRL2          0x0B

#define QMC5883P_ID                 0x80

typedef struct {
    float x;
    float y;
    float z;
} QMC5883_MagData;

struct QMC5883_Device;

typedef struct QMC5883_Device {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
    
    float offset_x;
    float offset_y;
    float offset_z;
    
    HAL_StatusTypeDef (*Init)(struct QMC5883_Device *self);
    HAL_StatusTypeDef (*ReadRaw)(struct QMC5883_Device *self, QMC5883_MagData *data);
    HAL_StatusTypeDef (*ReadCalibrated)(struct QMC5883_Device *self, QMC5883_MagData *data);
    void (*Calibrate)(struct QMC5883_Device *self, uint32_t cal_time_ms);
} QMC5883_Device;

void QMC5883_Create(QMC5883_Device *self, I2C_HandleTypeDef *hi2c, uint8_t dev_addr);

#endif /* __QMC5883_OOP_H */
