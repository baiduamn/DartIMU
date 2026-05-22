#ifndef __BMP388_OOP_H
#define __BMP388_OOP_H

#include "main.h"

#define BMP388_I2C_ADDR_PRIMARY   (0x76 << 1)
#define BMP388_I2C_ADDR_SECONDARY (0x77 << 1)

#define BMP3_CHIP_ID              0x50
#define BMP3_CMD_RDY              0x10
#define BMP388_SOFTRESET          0xB6

#define BMP388_NO_OVERSAMPLING    0x00
#define BMP388_OVERSAMPLING_2X    0x01
#define BMP388_OVERSAMPLING_4X    0x02
#define BMP388_OVERSAMPLING_8X    0x03
#define BMP388_OVERSAMPLING_16X   0x04
#define BMP388_OVERSAMPLING_32X   0x05

#define BMP3_IIR_FILTER_DISABLE   0x00
#define BMP3_IIR_FILTER_COEFF_1   0x01
#define BMP3_IIR_FILTER_COEFF_3   0x02
#define BMP3_IIR_FILTER_COEFF_7   0x03
#define BMP3_IIR_FILTER_COEFF_15  0x04
#define BMP3_IIR_FILTER_COEFF_31  0x05
#define BMP3_IIR_FILTER_COEFF_63  0x06
#define BMP3_IIR_FILTER_COEFF_127 0x07

#define BMP3_ODR_200_HZ           0x00
#define BMP3_ODR_100_HZ           0x01
#define BMP3_ODR_50_HZ            0x02
#define BMP3_ODR_25_HZ            0x03
#define BMP3_ODR_12_5_HZ          0x04
#define BMP3_ODR_6_25_HZ          0x05

#define BMP3_CALIBDATA_LEN        21

typedef enum {
    BMP3_REG_CHIP_ID     = 0x00,
    BMP3_REG_ERR_REG     = 0x02,
    BMP3_REG_STATUS      = 0x03,
    BMP3_REG_DATA_0      = 0x04,
    BMP3_REG_DATA_3      = 0x07,
    BMP3_REG_PWR_CTRL    = 0x1B,
    BMP3_REG_OSR         = 0x1C,
    BMP3_REG_ODR         = 0x1D,
    BMP3_REG_CONFIG      = 0x1F,
    BMP3_REG_CALIB_DATA  = 0x31,
    BMP3_REG_CMD         = 0x7E,
} BMP388_OOP_Regs;

typedef struct {
    float par_t1;
    float par_t2;
    float par_t3;
    float par_p1;
    float par_p2;
    float par_p3;
    float par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float par_p8;
    float par_p9;
    float par_p10;
    float par_p11;
} BMP388_CalibData;

struct BMP388_Device;

typedef struct BMP388_Device {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
    
    uint8_t osr;
    uint8_t iir;
    uint8_t odr;
    
    BMP388_CalibData calib;
    
    HAL_StatusTypeDef (*Init)(struct BMP388_Device *self);
    HAL_StatusTypeDef (*ReadTempPress)(struct BMP388_Device *self, float *temperature, float *pressure);
    float (*FindAltitude)(struct BMP388_Device *self, float ground_pressure, float pressure);
} BMP388_Device;

void BMP388_Create(BMP388_Device *self, I2C_HandleTypeDef *hi2c, uint8_t dev_addr);

#endif /* __BMP388_OOP_H */
