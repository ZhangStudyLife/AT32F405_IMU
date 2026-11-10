#ifndef LSM6DSV16X_H
#define LSM6DSV16X_H
#include "at32f402_405.h"

void imu_spi_init(void);
uint8_t imu_reg_read(uint8_t reg);
void imu_reg_write(uint8_t reg, uint8_t value);
int imu_read(int16_t gyro[3], int16_t accel[3]);
#endif
