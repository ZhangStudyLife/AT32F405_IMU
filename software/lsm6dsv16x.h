#ifndef LSM6DSV16X_H
#define LSM6DSV16X_H
#include "platform.h"
typedef struct {
    uint32_t time_us, edge_cycles;
    int16_t gyro[3], accel[3];
} ImuSample;
typedef struct {
    uint32_t id, phase, error, dma_timeouts, dma_errors, drdy_count, overlaps;
    uint32_t config_index, fallback_reads, last_status;
} SensorStatus;
extern SensorStatus sensor_status;
void sensor_init(void);
void sensor_stop(void);
int sensor_poll(ImuSample *sample); /* 1=new frame, 0=pending, -1=fault */
#ifdef IMU_TEST
extern volatile uint32_t test_fault; /* 1=no samples, 2=DMA never completes */
#endif
#endif
