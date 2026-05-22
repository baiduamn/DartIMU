#include "TenAxisIMU.h"
#include <math.h>
#include <stdio.h>

extern volatile uint32_t nowtime;

#define MAHONY_KP  1.5f
#define MAHONY_KI  0.002f

static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, 
                             float ax, float ay, float az, 
                             float mx, float my, float mz, float dt) {
    float recipNorm;
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

    if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
    } else {
        halfwx = 0.0f;
        halfwy = 0.0f;
        halfwz = 0.0f;
    }

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;

        if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
        } else {
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);
        }

        if (MAHONY_KI > 0.0f) {
            integralFBx += MAHONY_KI * halfex * dt;
            integralFBy += MAHONY_KI * halfey * dt;
            integralFBz += MAHONY_KI * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += MAHONY_KP * halfex;
        gy += MAHONY_KP * halfey;
        gz += MAHONY_KP * halfez;
    }

    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 * recipNorm;
    q[1] = q1 * recipNorm;
    q[2] = q2 * recipNorm;
    q[3] = q3 * recipNorm;
}

static HAL_StatusTypeDef TenAxisIMU_Init_Impl(TenAxisIMU *self) {
    printf("Initializing TenAxisIMU...\r\n");
    
    if (self->imu.Init(&(self->imu)) != 0) {
        printf("ICM45686 Init failed\r\n");
        return HAL_ERROR;
    }
    
    if (self->mag.Init(&(self->mag)) != HAL_OK) {
        printf("QMC5883P Init failed\r\n");
        return HAL_ERROR;
    }
    
    if (self->baro.Init(&(self->baro)) != HAL_OK) {
        printf("BMP388 Init failed\r\n");
        return HAL_ERROR;
    }
    
    printf("Calibrating baseline pressure...\r\n");
    float temp_t, temp_p;
    float sum_p = 0.0f;
    for (int i = 0; i < 20; i++) {
        self->baro.ReadTempPress(&(self->baro), &temp_t, &temp_p);
        sum_p += temp_p;
        HAL_Delay(15);
    }
    self->baseline_pressure = sum_p / 20.0f;
    printf("Baseline Pressure: %d Pa\r\n", (int)self->baseline_pressure);
    
    self->q[0] = 1.0f;
    self->q[1] = 0.0f;
    self->q[2] = 0.0f;
    self->q[3] = 0.0f;
    self->yaw = 0.0f;
    self->pitch = 0.0f;
    self->roll = 0.0f;
    self->altitude = 0.0f;
    self->last_update_ticks = nowtime;
    
    return HAL_OK;
}

static HAL_StatusTypeDef TenAxisIMU_Update_Impl(TenAxisIMU *self) {
    if (self->imu.ReadData(&(self->imu), self->accel, self->gyro, &(self->imu_temp)) != 0) {
        return HAL_ERROR;
    }
    
    self->mag.ReadCalibrated(&(self->mag), &(self->mag_cal));
    self->mag.ReadRaw(&(self->mag), &(self->mag_raw));
    
    self->baro.ReadTempPress(&(self->baro), &(self->raw_temperature), &(self->raw_pressure));
    self->altitude = self->baro.FindAltitude(&(self->baro), self->baseline_pressure, self->raw_pressure);
    
    uint32_t now = nowtime;
    uint32_t delta_ticks = now - self->last_update_ticks;
    self->last_update_ticks = now;
    
    float dt = (float)delta_ticks * 0.0001f;
    if (dt > 0.05f) dt = 0.05f;
    if (dt < 0.00001f) dt = 0.00001f;
    
    float gx_rad = self->gyro[0] * 3.14159265f / 180.0f;
    float gy_rad = self->gyro[1] * 3.14159265f / 180.0f;
    float gz_rad = self->gyro[2] * 3.14159265f / 180.0f;
    
    MahonyAHRSupdate(self->q, gx_rad, gy_rad, gz_rad, 
                     self->accel[0], self->accel[1], self->accel[2], 
                     self->mag_cal.x, self->mag_cal.y, self->mag_cal.z, dt);
                     
    float q0 = self->q[0], q1 = self->q[1], q2 = self->q[2], q3 = self->q[3];
    self->yaw   = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / 3.14159265f;
    self->pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * 180.0f / 3.14159265f;
    self->roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180.0f / 3.14159265f;
    
    return HAL_OK;
}

static void TenAxisIMU_GetEuler_Impl(TenAxisIMU *self, float *yaw, float *pitch, float *roll) {
    *yaw = self->yaw;
    *pitch = self->pitch;
    *roll = self->roll;
}

static float TenAxisIMU_GetAltitude_Impl(TenAxisIMU *self) {
    return self->altitude;
}

void TenAxisIMU_Create(TenAxisIMU *self, 
                       SPI_HandleTypeDef *hspi_imu, GPIO_TypeDef *cs_port_imu, uint16_t cs_pin_imu,
                       I2C_HandleTypeDef *hi2c_sensors) {
    ICM45686_Create(&(self->imu), hspi_imu, cs_port_imu, cs_pin_imu);
    BMP388_Create(&(self->baro), hi2c_sensors, BMP388_I2C_ADDR_PRIMARY);
    QMC5883_Create(&(self->mag), hi2c_sensors, QMC5883P_ADDR);
    
    self->Init = TenAxisIMU_Init_Impl;
    self->Update = TenAxisIMU_Update_Impl;
    self->GetEuler = TenAxisIMU_GetEuler_Impl;
    self->GetAltitude = TenAxisIMU_GetAltitude_Impl;
}
