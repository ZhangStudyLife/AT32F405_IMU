#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <stdint.h>
enum { CAL_IDLE, CAL_READY, CAL_WAIT, CAL_COLLECTING, CAL_SOLVING, CAL_DONE, CAL_FAILED };
enum { CAL_OK, CAL_BUSY, CAL_TOO_FEW, CAL_MOVING, CAL_DUPLICATE, CAL_SINGULAR,
       CAL_BAD_FIT, CAL_TIMEOUT, CAL_BAD_COMMAND };
typedef struct {
    uint32_t flags; /* bit0=gyro calibrated, bit1=accel calibrated */
    float gyro_bias[3], accel_bias[3], matrix[3][3];
} CalParams;
typedef struct {
    uint32_t mode, state, poses, samples, stable, total_samples, error, solve_steps;
    float fit_rms, max_norm_error;
} CalStatus;
extern CalParams cal_params;
extern CalStatus cal_status;
void Cal_Init(void);
int Cal_StartGyro(void);
int Cal_StartAccel(void);
int Cal_Collect(void);
int Cal_Solve(void);
void Cal_Abort(void);
void Cal_Update(const float accel[3], const float gyro[3]);
void Cal_Step(void); /* one bounded solver step */
void Cal_Apply(const float accel[3], const float gyro[3], float a[3], float g[3]);
#ifdef IMU_TEST
int Cal_TestPose(const float accel[3]);
#endif
#endif

