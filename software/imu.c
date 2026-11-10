#include "imu.h"
#include "MahonyAhrs.h"
#include "IMU_Filtter.h"
#include "storage.h"
#include <math.h>
#include <string.h>

volatile uint32_t debug_request, debug_ack, imu_command, boot_error;
volatile DebugSnapshot debug_snapshot;
static IMUState state;
static MahonyAhrs_t ahrs;
static uint32_t last_sample, profile_since, last_solver_us, solve_start, save_start;
static uint32_t maintenance, save_result, stale, seeded, erase_requested;
static int command_result;

const IMUState *IMU_GetState(void) { return &state; }

static void restart(void)
{
    state.valid=state.warmup=0;
    seeded=last_sample=stale=0;
    IMUFilter_Init();
    MahonyAhrs_Init(&ahrs);
    sensor_init();
}

void IMU_Init(void)
{
    Cal_Init();
    storage_load();
    restart();
    profile_since=micros();
}

/* Initial tilt in native sensor axes, including the inverted orientation. */
static void seed_attitude(const float a[3])
{
    float roll=atan2f(a[1],a[2]);
    float pitch=atan2f(-a[0],sqrtf(a[1]*a[1]+a[2]*a[2]));
    float cr=cosf(roll*0.5f), sr=sinf(roll*0.5f);
    float cp=cosf(pitch*0.5f), sp=sinf(pitch*0.5f);
    ahrs.q0=cr*cp; ahrs.q1=sr*cp; ahrs.q2=cr*sp; ahrs.q3=-sr*sp;
}

#ifdef IMU_TEST
static int synthetic_calibration(unsigned kind)
{
    unsigned i,n=kind==100 ? 128:(kind==101 ? 12:3);
    Cal_Abort(); Cal_StartAccel();
    for(i=0;i<n;i++) {
        float a[3], z=kind==101 ? 0:1-2*((float)i+0.5f)/(float)n;
        float angle=kind==101 ? 6.283185307f*(float)i/(float)n : 2.39996323f*(float)i;
        float x=cosf(angle)*sqrtf(1-z*z), y=sinf(angle)*sqrtf(1-z*z);
        a[2]=z/1.01f;
        a[1]=(y-0.015f*a[2])/0.98f;
        a[0]=(x-0.02f*a[1]+0.01f*a[2])/1.02f;
        a[0]+=0.03f; a[1]-=0.02f; a[2]+=0.01f;
        if(Cal_TestPose(a)!=CAL_OK) return -10;
    }
    solve_start=cycles();
    return Cal_Solve();
}
#endif

static void commands(void)
{
    uint32_t cmd=imu_command;
    if(!cmd || maintenance) return;
    imu_command=0;
    command_result=CAL_OK;
    switch(cmd) {
    case CMD_GYRO: command_result=Cal_StartGyro(); break;
    case CMD_ACCEL: command_result=Cal_StartAccel(); break;
    case CMD_COLLECT: command_result=Cal_Collect(); break;
    case CMD_SOLVE:
        command_result=Cal_Solve(); solve_start=cycles(); break;
    case CMD_ABORT: Cal_Abort(); break;
    case CMD_SAVE:
    case CMD_ERASE_CAL:
        if(cal_status.state>=CAL_READY && cal_status.state<=CAL_SOLVING) {
            command_result=CAL_BUSY; break;
        }
        state.valid=0; maintenance=1; save_result=0;
        erase_requested=cmd==CMD_ERASE_CAL;
        sensor_stop(); save_start=cycles(); storage_start(erase_requested);
        break;
    case CMD_PROFILE_RESET:
        profile_reset(); profile_since=micros();
        state.period_min=state.period_max=0; break;
    case CMD_RESTART: Cal_Abort(); restart(); break;
#ifdef IMU_TEST
    case 100: case 101: case 102: command_result=synthetic_calibration(cmd); break;
#endif
    default: command_result=CAL_BAD_COMMAND; break;
    }
}

void IMU_Process(void)
{
    ImuSample sample;
    float dt;
    uint32_t start,period,cal_before;
    unsigned i;
    int result;
    commands();
    if(maintenance) {
        result=storage_poll();
        if(result) {
            profile_add(PROF_SAVE,cycles()-save_start);
            save_result=result>0 ? 1:2;
            if(result>0 && erase_requested) Cal_Init();
            maintenance=0; restart();
        }
        return;
    }
    start=cycles();
    result=sensor_poll(&sample);
    profile_add(PROF_DRIVER,cycles()-start);
    if(!last_sample && sensor_status.phase==6) last_sample=micros();
    if(result==1) {
        period=last_sample ? sample.time_us-last_sample:1000;
        last_sample=sample.time_us;
        stale=0;
        state.time_us=sample.time_us;
        state.period_last=period;
        if(!state.period_min || period<state.period_min) state.period_min=period;
        if(period>state.period_max) state.period_max=period;
        if(period>1500) state.missed+=(period+500)/1000-1;
        if(!period || period>=10000) {
            state.invalid++; state.valid=0; seeded=0; return;
        }
        dt=period*0.000001f;
        start=cycles();
        for(i=0;i<3;i++) {
            state.gyro[i]=sample.gyro[i]*0.070f;
            state.accel[i]=sample.accel[i]*0.000488f;
        }
        period=cycles()-start;
        start=cycles();
        cal_before=cal_status.state;
        Cal_Update(state.accel,state.gyro);
        if(cal_before!=CAL_DONE && cal_status.state==CAL_DONE && cal_status.mode==1)
            MahonyAhrs_ResetBias(&ahrs);
        profile_add(PROF_CAL_SAMPLE,cycles()-start);
        start=cycles();
        Cal_Apply(state.accel,state.gyro,state.filtered_accel,state.filtered_gyro);
        profile_add(PROF_PREPROCESS,period+cycles()-start);
        start=cycles();
        IMUFilter_Update(state.filtered_gyro,state.filtered_accel);
        profile_add(PROF_FILTER,cycles()-start);
        if(!seeded) { seed_attitude(state.filtered_accel); seeded=1; }
        start=cycles();
        MahonyAhrs_Update(&ahrs,
            state.filtered_gyro[0],state.filtered_gyro[1],state.filtered_gyro[2],
            state.filtered_accel[0],state.filtered_accel[1],state.filtered_accel[2],dt);
        profile_add(PROF_MAHONY,cycles()-start);
        start=cycles();
        MahonyAhrs_GetEulerDegrees(&ahrs,state.euler);
        state.q[0]=ahrs.q0; state.q[1]=ahrs.q1;
        state.q[2]=ahrs.q2; state.q[3]=ahrs.q3;
        state.frames++;
        if(state.warmup<1000) state.warmup++;
        state.valid=state.warmup==1000;
        state.calibrated=cal_params.flags;
        profile_add(PROF_EULER,cycles()-start);
        period=cycles()-sample.edge_cycles;
        profile_add(PROF_FRAME,period);
        if(state.valid && period>system_core_clock/2000U) state.late++;
    }
    if(result<0 || (last_sample && (uint32_t)(micros()-last_sample)>10000)) {
        if(!stale) { state.no_data++; stale=1; }
        state.valid=0;
    }
    if(cal_status.state==CAL_SOLVING && (uint32_t)(micros()-last_solver_us)>=1000) {
        last_solver_us=micros();
        start=cycles(); Cal_Step();
        profile_add(PROF_CAL_SLICE,cycles()-start);
        if(cal_status.state!=CAL_SOLVING) profile_add(PROF_CAL_TOTAL,cycles()-solve_start);
    }
}

void IMU_DebugPublish(void)
{
    uint32_t request=debug_request,mask;
    if(request==debug_ack) return;
    debug_snapshot.sequence++;
    __DMB();
    debug_snapshot.magic=0x41545449;
    debug_snapshot.version=2;
    debug_snapshot.size=sizeof(DebugSnapshot);
    debug_snapshot.now_us=micros();
    debug_snapshot.heartbeat=heartbeat_ms;
    debug_snapshot.core_hz=system_core_clock;
    debug_snapshot.command_result=command_result;
    debug_snapshot.maintenance=maintenance;
    debug_snapshot.save_result=save_result;
    debug_snapshot.profile_elapsed_us=micros()-profile_since;
#ifdef IMU_TEST
    debug_snapshot.test_build=1;
#endif
    debug_snapshot.imu=state;
    debug_snapshot.calibration=cal_status;
    debug_snapshot.params=cal_params;
    mask=__get_PRIMASK(); __disable_irq();
    debug_snapshot.sensor=sensor_status;
    memcpy((void *)debug_snapshot.timing,profiles,sizeof(profiles));
    debug_snapshot.idle_cycles=idle_cycles;
    __set_PRIMASK(mask);
    __DMB();
    debug_snapshot.sequence++;
    __DMB();
    debug_ack=request;
}
