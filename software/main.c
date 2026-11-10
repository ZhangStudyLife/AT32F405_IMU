#include "lsm6dsv16x.h"

volatile uint32_t heartbeat_ms;
volatile uint32_t imu_id, imu_error, sample_count;
volatile uint8_t imu_config[8];
volatile int16_t gyro_raw[3], accel_raw[3];
volatile float gyro_dps[3], accel_g[3];

void SysTick_Handler(void) { heartbeat_ms++; }

static void delay_ms(uint32_t ms)
{
    uint32_t start = heartbeat_ms;
    while ((uint32_t)(heartbeat_ms - start) < ms);
}

int main(void)
{
    int16_t gyro[3], accel[3];
    unsigned i;
    system_core_clock_update();
    SysTick_Config(system_core_clock / 1000);
    imu_spi_init();
    delay_ms(100);
    imu_id = imu_reg_read(0x0f);
    if (imu_id != 0x70) { imu_error = 1; while (1); }
    imu_reg_write(0x12, 0x01); /* software reset */
    for (i = 0; i < 100 && (imu_reg_read(0x12) & 1); i++)
        delay_ms(1);
    if (imu_reg_read(0x12) & 1) { imu_error = 2; while (1); }
    imu_reg_write(0x12, 0x44); /* BDU + address increment */
    imu_reg_write(0x15, 0x01); /* gyro +/-250 dps */
    imu_reg_write(0x17, 0x00); /* accel +/-2 g */
    imu_reg_write(0x10, 0x06); /* accel 120 Hz, high performance */
    imu_reg_write(0x11, 0x06); /* gyro 120 Hz, high performance */
    for (i = 0; i < 8; i++)
        imu_config[i] = imu_reg_read(0x10 + i);
    if (imu_config[0] != 6 || imu_config[1] != 6 ||
        imu_config[2] != 0x44 || imu_config[5] != 1 || imu_config[7] != 0)
        { imu_error = 3; while (1); }
    delay_ms(100);
    while (1)
    {
        if (imu_read(gyro, accel))
        {
            for (i = 0; i < 3; i++)
            {
                gyro_raw[i] = gyro[i];
                accel_raw[i] = accel[i];
                gyro_dps[i] = gyro[i] * 0.00875f;
                accel_g[i] = accel[i] * 0.000061f;
            }
            sample_count++;
        }
        delay_ms(1);
    }
}
