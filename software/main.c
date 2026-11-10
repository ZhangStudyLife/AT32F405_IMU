#include "imu.h"

int main(void)
{
    uint32_t start;
    if (!platform_init()) boot_error=1;
    else IMU_Init();
    for (;;) {
        start=cycles();
        if (!boot_error) IMU_Process();
        IMU_DebugPublish();
        profile_add(PROF_PROCESS,cycles()-start);
        platform_idle();
    }
}
