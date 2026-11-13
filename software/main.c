#include "imu.h"
#include "uart.h"

int main(void)
{
    uint32_t start, maintenance;
    if (!platform_init()) boot_error=1;
    IMU_Init(); UART_Init();
    for (;;) {
        start=cycles();
        maintenance=IMU_GetState()->maintenance;
        if (!boot_error) IMU_Process();
        UART_Process();
        if (!maintenance && !IMU_GetState()->maintenance)
            profile_add(PROF_PROCESS,cycles()-start);
        platform_idle();
    }
}
