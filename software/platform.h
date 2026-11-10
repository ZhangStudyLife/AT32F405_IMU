#ifndef PLATFORM_H
#define PLATFORM_H
#include "at32f402_405.h"
#include <stdint.h>

extern volatile uint32_t heartbeat_ms;
extern uint64_t idle_cycles;
int platform_init(void);
void platform_idle(void);
static __inline uint32_t cycles(void) { return DWT->CYCCNT; }
static __inline uint32_t micros(void) { return TMR2->cval; }

enum { PROF_ACQUIRE, PROF_DRIVER, PROF_PREPROCESS, PROF_FILTER, PROF_MAHONY,
       PROF_EULER, PROF_CAL_SAMPLE, PROF_CAL_SLICE, PROF_CAL_TOTAL,
       PROF_FRAME, PROF_PROCESS, PROF_IRQ, PROF_SAVE, PROF_COUNT };
typedef struct {
    uint32_t count, last, min, max;
    uint64_t total;
} Profile;
extern Profile profiles[PROF_COUNT];
void profile_add(unsigned stage, uint32_t elapsed);
void profile_reset(void);
#endif
