#include "icm45686_oop.h"
#include "delay.h"
#include <stdio.h>

static ICM45686_Device *g_active_icm = NULL;

static int icm45686_read_regs(uint8_t reg, uint8_t* buf, uint32_t len) {
    if (!g_active_icm) return -1;
    
    reg |= 0x80;
    HAL_GPIO_WritePin(g_active_icm->cs_port, g_active_icm->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(g_active_icm->hspi, &reg, 1, 100);
    HAL_SPI_Receive(g_active_icm->hspi, buf, len, 1000);
    HAL_GPIO_WritePin(g_active_icm->cs_port, g_active_icm->cs_pin, GPIO_PIN_SET);
    return 0;
}

static int icm45686_write_regs(uint8_t reg, const uint8_t* buf, uint32_t len) {
    if (!g_active_icm) return -1;
    
    for (uint32_t i = 0; i < len; i++) {
        HAL_GPIO_WritePin(g_active_icm->cs_port, g_active_icm->cs_pin, GPIO_PIN_RESET);
        uint8_t r = reg + i;
        HAL_SPI_Transmit(g_active_icm->hspi, &r, 1, 100);
        HAL_SPI_Transmit(g_active_icm->hspi, (uint8_t *)&buf[i], 1, 100);
        HAL_GPIO_WritePin(g_active_icm->cs_port, g_active_icm->cs_pin, GPIO_PIN_SET);
    }
    return 0;
}

static int ICM45686_Init_Impl(ICM45686_Device *self) {
    int rc = 0;
    uint8_t whoami = 0;
    inv_imu_int_pin_config_t int_pin_config;
    inv_imu_int_state_t int_config;
    
    g_active_icm = self;
    
    if (self->cs_port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (self->cs_port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (self->cs_port == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    
    GPIO_InitTypeDef GPIO_Initure = {0};
    GPIO_Initure.Pin   = self->cs_pin;
    GPIO_Initure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull  = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(self->cs_port, &GPIO_Initure);
    HAL_GPIO_WritePin(self->cs_port, self->cs_pin, GPIO_PIN_SET);
    
    HAL_Delay(100);
    
    self->imu_dev.transport.read_reg   = icm45686_read_regs;
    self->imu_dev.transport.write_reg  = icm45686_write_regs;
    self->imu_dev.transport.sleep_us   = delay_us;
    self->imu_dev.transport.serif_type = UI_SPI4;
    
    drive_config0_t drive_config0;
    drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
    rc = inv_imu_write_reg(&(self->imu_dev), DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
    if (rc != 0) return rc;
    delay_us(2);
    
    rc = inv_imu_get_who_am_i(&(self->imu_dev), &whoami);
    if (rc != 0 || whoami != INV_IMU_WHOAMI) {
        printf("ICM45686 WHOAMI Error: 0x%02X\r\n", whoami);
        return -1;
    }
    
    rc = inv_imu_soft_reset(&(self->imu_dev));
    if (rc != 0) return rc;
    
    int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
    int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
    int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
    rc = inv_imu_set_pin_config_int(&(self->imu_dev), INV_IMU_INT1, &int_pin_config);
    if (rc != 0) return rc;
    
    memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
    int_config.INV_UI_DRDY = INV_IMU_ENABLE;
    rc = inv_imu_set_config_int(&(self->imu_dev), INV_IMU_INT1, &int_config);
    if (rc != 0) return rc;
    
    rc |= inv_imu_set_accel_fsr(&(self->imu_dev), ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G);
    rc |= inv_imu_set_gyro_fsr(&(self->imu_dev), GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS);
    
    rc |= inv_imu_set_accel_frequency(&(self->imu_dev), ACCEL_CONFIG0_ACCEL_ODR_1600_HZ);
    rc |= inv_imu_set_gyro_frequency(&(self->imu_dev), GYRO_CONFIG0_GYRO_ODR_1600_HZ);
    
    rc |= inv_imu_set_accel_ln_bw(&(self->imu_dev), IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
    rc |= inv_imu_set_gyro_ln_bw(&(self->imu_dev), IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);
    
    rc |= inv_imu_select_accel_lp_clk(&(self->imu_dev), SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);
    if (rc != 0) return rc;
    
    rc |= inv_imu_set_accel_mode(&(self->imu_dev), PWR_MGMT0_ACCEL_MODE_LN);
    rc |= inv_imu_set_gyro_mode(&(self->imu_dev), PWR_MGMT0_GYRO_MODE_LN);
    if (rc != 0) return rc;
    
    self->Calibrate(self);
    
    return 0;
}

static int ICM45686_ReadData_Impl(ICM45686_Device *self, float accel[3], float gyro[3], float *temp) {
    inv_imu_sensor_data_t d;
    g_active_icm = self;
    
    int rc = inv_imu_get_register_data(&(self->imu_dev), &d);
    if (rc != 0) return rc;
    
    accel[0] = (float)(d.accel_data[0] * 4.0f / 32.768f);
    accel[1] = (float)(d.accel_data[1] * 4.0f / 32.768f);
    accel[2] = (float)(d.accel_data[2] * 4.0f / 32.768f);
    
    gyro[0] = (float)(d.gyro_data[0] * 1000.0f / 32768.0f) - self->gyro_offset[0];
    gyro[1] = (float)(d.gyro_data[1] * 1000.0f / 32768.0f) - self->gyro_offset[1];
    gyro[2] = (float)(d.gyro_data[2] * 1000.0f / 32768.0f) - self->gyro_offset[2];
    
    *temp = 25.0f + (d.temp_data / 128.0f);
    
    return 0;
}

static void ICM45686_Calibrate_Impl(ICM45686_Device *self) {
    float accel[3], gyro[3], temp;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int samples = 200;
    
    printf("ICM45686 Calibrating (Hold still)...\r\n");
    g_active_icm = self;
    
    for (int i = 0; i < 50; i++) {
        self->ReadData(self, accel, gyro, &temp);
        HAL_Delay(2);
    }
    
    for (int i = 0; i < samples; i++) {
        self->ReadData(self, accel, gyro, &temp);
        sum_gx += gyro[0];
        sum_gy += gyro[1];
        sum_gz += gyro[2];
        HAL_Delay(2);
    }
    
    self->gyro_offset[0] = (float)(sum_gx / samples);
    self->gyro_offset[1] = (float)(sum_gy / samples);
    self->gyro_offset[2] = (float)(sum_gz / samples);
    
    printf("ICM45686 Offsets: X:%d/100, Y:%d/100, Z:%d/100 dps\r\n", 
           (int)(self->gyro_offset[0] * 100.0f), (int)(self->gyro_offset[1] * 100.0f), (int)(self->gyro_offset[2] * 100.0f));
}

void ICM45686_Create(ICM45686_Device *self, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    self->hspi = hspi;
    self->cs_port = cs_port;
    self->cs_pin = cs_pin;
    
    self->gyro_offset[0] = 0.0f;
    self->gyro_offset[1] = 0.0f;
    self->gyro_offset[2] = 0.0f;
    
    self->Init = ICM45686_Init_Impl;
    self->ReadData = ICM45686_ReadData_Impl;
    self->Calibrate = ICM45686_Calibrate_Impl;
}
