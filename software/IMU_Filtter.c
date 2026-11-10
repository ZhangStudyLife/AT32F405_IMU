/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
/* AT32 optimization: shared coefficients and in-place 1 kHz output.
 * Original transfer functions and first-frame DC seeding retained. */
#pragma STDC FP_CONTRACT OFF
#include "IMU_Filtter.h"
#include <math.h>
#include <string.h>

static float notch[4], lowpass[4], pt2_k; /* b0=b2, b1, a1, a2 */
static struct { float n1, n2, l1, l2, a1, a2; } state[3];
static unsigned initialized;

void IMUFilter_Init(void)
{
    float w = 2.0f * 3.14159265359f * 300.0f / 1000.0f;
    float alpha = sinf(w) / (2.0f * (300.0f / 180.0f));
    float a0 = 1.0f + alpha;
    notch[0] = 1.0f / a0;
    notch[1] = notch[2] = (-2.0f * cosf(w)) / a0;
    notch[3] = (1.0f - alpha) / a0;
    w = 2.0f * 3.14159265359f * 80.0f / 1000.0f;
    alpha = sinf(w) / (2.0f * 0.70710678f);
    a0 = 1.0f + alpha;
    lowpass[0] = (1.0f - cosf(w)) * 0.5f / a0;
    lowpass[1] = (1.0f - cosf(w)) / a0;
    lowpass[2] = (-2.0f * cosf(w)) / a0;
    lowpass[3] = (1.0f - alpha) / a0;
    w = 2.0f * 3.14159265359f * (25.0f * 1.553773974f) * (1.0f / 1000.0f);
    pt2_k = w / (w + 1.0f);
    memset(state, 0, sizeof(state));
    initialized = 0;
}

void IMUFilter_Update(float gyro[3], float accel[3])
{
    unsigned i;
    if (!initialized) {
        for (i = 0; i < 3; i++) {
            state[i].n1 = gyro[i] * (1.0f - notch[0]);
            state[i].n2 = gyro[i] * (notch[0] - notch[3]);
            state[i].l1 = gyro[i] * (1.0f - lowpass[0]);
            state[i].l2 = gyro[i] * (lowpass[0] - lowpass[3]);
            state[i].a1 = state[i].a2 = accel[i];
        }
        initialized = 1;
        return;
    }
    for (i = 0; i < 3; i++) {
        float n = notch[0] * gyro[i] + state[i].n1;
        float g = lowpass[0] * n + state[i].l1;
        state[i].n1 = notch[1] * gyro[i] - notch[2] * n + state[i].n2;
        state[i].n2 = notch[0] * gyro[i] - notch[3] * n;
        state[i].l1 = lowpass[1] * n - lowpass[2] * g + state[i].l2;
        state[i].l2 = lowpass[0] * n - lowpass[3] * g;
        state[i].a1 += pt2_k * (accel[i] - state[i].a1);
        state[i].a2 += pt2_k * (state[i].a1 - state[i].a2);
        gyro[i] = g;
        accel[i] = state[i].a2;
    }
}
