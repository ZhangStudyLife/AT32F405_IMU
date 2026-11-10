#ifndef IMU_H
#define IMU_H
#include "lsm6dsv16x.h"
#include "calibration.h"
typedef struct {
    uint32_t frames, time_us, valid, calibrated, missed, late, no_data, invalid;
    uint32_t period_last, period_min, period_max, warmup;
    float gyro[3], accel[3], filtered_gyro[3], filtered_accel[3], q[4], euler[3];
} IMUState;
enum { CMD_NONE, CMD_GYRO, CMD_ACCEL, CMD_COLLECT, CMD_SOLVE, CMD_ABORT,
       CMD_SAVE, CMD_PROFILE_RESET, CMD_RESTART, CMD_ERASE_CAL };
typedef struct {
    uint32_t magic, version, size, sequence, now_us, heartbeat, core_hz;
    int32_t command_result;
    uint32_t maintenance, save_result, profile_elapsed_us, test_build;
    SensorStatus sensor;
    IMUState imu;
    CalStatus calibration;
    CalParams params;
    Profile timing[PROF_COUNT];
    uint64_t idle_cycles;
} DebugSnapshot;
extern volatile uint32_t debug_request, debug_ack, imu_command;
extern volatile DebugSnapshot debug_snapshot;
extern volatile uint32_t boot_error;
void IMU_Init(void);
void IMU_Process(void);
const IMUState *IMU_GetState(void); /* main context only; debugger uses mailbox */
void IMU_DebugPublish(void);
#endif
