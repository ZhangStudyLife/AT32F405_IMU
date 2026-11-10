#include "platform.h"
#include <string.h>

volatile uint32_t heartbeat_ms;
Profile profiles[PROF_COUNT];
uint64_t idle_cycles;

void platform_idle(void)
{
    uint32_t start;
    /* An enabled pending IRQ wakes WFI even with PRIMASK set. Account sleep
       before unmasking, so interrupt service is counted as active CPU time. */
    __disable_irq();
    start = cycles();
    __DSB();
    __WFI();
    idle_cycles += (uint32_t)(cycles() - start);
    __enable_irq();
}

void profile_reset(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    memset(profiles, 0, sizeof(profiles));
    idle_cycles = 0;
    __set_PRIMASK(mask);
}

void SysTick_Handler(void)
{
    static unsigned ticks;
    uint32_t start = cycles();
    if (++ticks == 10) { ticks = 0; heartbeat_ms++; }
    profile_add(PROF_IRQ, cycles() - start);
}

int platform_init(void)
{
    uint32_t start;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    flash_psr_set(FLASH_WAIT_CYCLE_6);
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
    /* HICK / 6 = 8 MHz; 8 * 108 / 1 / 4 = 216 MHz. */
    crm_pll_config(CRM_PLL_SOURCE_HICK, 108, 1, CRM_PLL_FP_4);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    start = cycles();
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
        if ((uint32_t)(cycles() - start) > 800000U) goto timebase;
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_2);
    crm_apb2_div_set(CRM_APB2_DIV_1);
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    start = cycles();
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
        if ((uint32_t)(cycles() - start) > 21600000U) goto timebase;
timebase:
    crm_auto_step_mode_enable(FALSE);
    system_core_clock_update();
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    tmr_32_bit_function_enable(TMR2, TRUE);
    tmr_base_init(TMR2, 0xffffffffU, system_core_clock / 1000000U - 1U);
    tmr_counter_enable(TMR2, TRUE);
    /* 100 us wakeups handle startup deadlines / missing DRDY without spinning. */
    SysTick_Config(system_core_clock / 10000U);
    NVIC_SetPriority(SysTick_IRQn, 2);
    return system_core_clock == 216000000U;
}
