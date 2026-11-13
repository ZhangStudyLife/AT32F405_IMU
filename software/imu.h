#ifndef IMU_H
#define IMU_H
#include "lsm6dsv16x.h"
#include "calibration.h"
typedef struct {
    uint32_t frames, time_us, valid, calibrated, missed, late, no_data, invalid;
    uint32_t period_last, period_min, period_max, warmup;
    uint32_t maintenance, save_result;
    float gyro[3], accel[3], filtered_gyro[3], filtered_accel[3], q[4], euler[3];
} IMUState;
enum { CMD_NONE, CMD_GYRO, CMD_ACCEL, CMD_COLLECT, CMD_SOLVE, CMD_ABORT,
       CMD_SAVE, CMD_PROFILE_RESET, CMD_RESTART, CMD_ERASE_CAL };
extern volatile uint32_t boot_error;
void IMU_Init(void);
void IMU_Process(void);
const IMUState *IMU_GetState(void); /* main context only */
int IMU_Command(uint32_t command);
#endif
