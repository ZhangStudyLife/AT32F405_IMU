#include "uart.h"
#include "imu.h"
#include <string.h>

UARTConfig uart_config={2000000,1000,1,0};
static UARTConfig next_config;
static uint8_t tx[160], request[18];
static volatile uint8_t rx[256];
static volatile uint32_t rx_wrap, rx_fault;
static uint32_t rx_read, partial_us, tx_started, tx_limit, switch_us, last_frame;
static uint32_t tx_dropped, tx_errors, rx_errors, crc_errors;
static unsigned tx_length, request_length, pending, decimation, enabled=1;
enum { RUN, SWITCH_ACK, SWITCH_DELAY, SAVE_DRAIN, SAVE_WAIT };
static unsigned phase;
static uint32_t report_us;

static uint32_t get32(const void *p)
{
    uint32_t value;
    memcpy(&value,p,4);
    return value;
}

static void put32(void *p, uint32_t value) { memcpy(p,&value,4); }

static uint16_t crc16(const uint8_t *p, unsigned size)
{
    uint16_t crc=0xffff;
    unsigned i;
    while(size--) {
        crc^=(uint16_t)*p++ << 8;
        for(i=0;i<8;i++) crc=(uint16_t)((crc<<1)^((crc & 0x8000) ? 0x1021:0));
    }
    return crc;
}

static unsigned frame_size(unsigned mask)
{
    return 16+((mask&1)?12:0)+((mask&2)?16:0)+((mask&4)?24:0)+((mask&8)?24:0);
}

int UART_ConfigCheck(const UARTConfig *c)
{
    static const uint16_t rates[]={10,25,50,100,200,250,500,1000};
    unsigned i;
    if((c->baud!=1000000 && c->baud!=2000000 && c->baud!=3000000 && c->baud!=6000000) ||
       !c->mask || c->mask>15 || c->reserved) return UART_BAD_ARGUMENT;
    for(i=0;i<sizeof(rates)/sizeof(rates[0]);i++) if(c->rate==rates[i]) break;
    if(i==sizeof(rates)/sizeof(rates[0])) return UART_BAD_ARGUMENT;
    return frame_size(c->mask)*c->rate*10U>c->baud*4U/5U ? UART_BANDWIDTH:CAL_OK;
}

void DMA1_Channel4_IRQHandler(void)
{
    uint32_t flags=DMA1->sts;
    if(flags & DMA1_FDT4_FLAG) rx_wrap+=sizeof(rx);
    if(flags & DMA1_DTERR4_FLAG) rx_fault=1;
    DMA1->clr=DMA1_GL4_FLAG;
}

static uint32_t received(void)
{
    uint32_t key=__get_PRIMASK(), count;
    __disable_irq();
    count=rx_wrap+sizeof(rx)-DMA1_CHANNEL4->dtcnt;
    /* DMA can wrap while interrupts are masked. Consume its pending wrap here. */
    if(DMA1->sts & DMA1_FDT4_FLAG) {
        DMA1->clr=DMA1_FDT4_FLAG;
        rx_wrap+=sizeof(rx);
        count=rx_wrap+sizeof(rx)-DMA1_CHANNEL4->dtcnt;
    }
    __set_PRIMASK(key);
    __DMB();
    return count;
}

static void receive_reset(void)
{
    uint32_t key=__get_PRIMASK();
    __disable_irq();
    USART2->ctrl1_bit.ren=0;
    DMA1_CHANNEL4->ctrl_bit.chen=0;
    (void)USART2->sts;
    (void)USART2->dt;
    DMA1->clr=DMA1_GL4_FLAG;
    DMA1_CHANNEL4->dtcnt=sizeof(rx);
    rx_read=rx_wrap=rx_fault=partial_us=0;
    NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
    __DMB();
    DMA1_CHANNEL4->ctrl_bit.chen=1;
    USART2->ctrl1_bit.ren=1;
    __set_PRIMASK(key);
}

static void baud_apply(void)
{
    USART2->ctrl1=0;
    USART2->baudr=(system_core_clock/2U+uart_config.baud/2U)/uart_config.baud;
    USART2->ctrl2=0;                       /* One stop bit. */
    USART2->ctrl3=(1U<<6)|(1U<<7);         /* RX/TX DMA, no flow control. */
    USART2->ctrl1=(1U<<13)|(1U<<3)|(1U<<2); /* Enable, TX/RX, 8 data bits. */
    receive_reset();
}

void UART_Init(void)
{
    gpio_init_type gpio;
    dma_init_type dma;
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK,TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK,TRUE);
    crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK,TRUE);
    gpio_default_para_init(&gpio);
    gpio.gpio_pins=GPIO_PINS_2|GPIO_PINS_3;
    gpio.gpio_mode=GPIO_MODE_MUX;
    gpio.gpio_pull=GPIO_PULL_UP;
    gpio.gpio_drive_strength=GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA,&gpio);
    gpio_pin_mux_config(GPIOA,GPIO_PINS_SOURCE2,GPIO_MUX_7);
    gpio_pin_mux_config(GPIOA,GPIO_PINS_SOURCE3,GPIO_MUX_7);
    dmamux_enable(DMA1,TRUE);
    dma_reset(DMA1_CHANNEL3);
    dma_reset(DMA1_CHANNEL4);
    dma_default_para_init(&dma);
    dma.peripheral_base_addr=(uint32_t)&USART2->dt;
    dma.memory_inc_enable=TRUE;
    dma.memory_data_width=DMA_MEMORY_DATA_WIDTH_BYTE;
    dma.peripheral_data_width=DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma.priority=DMA_PRIORITY_MEDIUM;
    dma.direction=DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma.memory_base_addr=(uint32_t)tx;
    dma_init(DMA1_CHANNEL3,&dma);
    dma.direction=DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma.memory_base_addr=(uint32_t)rx;
    dma.loop_mode_enable=TRUE;
    dma.buffer_size=sizeof(rx);
    dma_init(DMA1_CHANNEL4,&dma);
    dmamux_init(DMA1MUX_CHANNEL3,DMAMUX_DMAREQ_ID_USART2_TX);
    dmamux_init(DMA1MUX_CHANNEL4,DMAMUX_DMAREQ_ID_USART2_RX);
    dma_interrupt_enable(DMA1_CHANNEL4,DMA_FDT_INT|DMA_DTERR_INT,TRUE);
    NVIC_SetPriority(DMA1_Channel4_IRQn,3);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    if(boot_error) uart_config.baud=115200;
    baud_apply();
}

static uint8_t state_flags(const IMUState *s)
{
    return (uint8_t)((s->valid?1:0)|((cal_params.flags&3U)<<1)|
        ((cal_status.state>=CAL_READY && cal_status.state<=CAL_SOLVING)?8:0)|
        (s->maintenance?16:0)|((sensor_status.error || boot_error ||
          (sensor_status.phase==6 && (!s->warmup || (uint32_t)(micros()-s->time_us)>=10000)))?32:0)|
        ((s->warmup<1000)?64:0)|(enabled?128:0));
}

static void send_frame(unsigned type, uint32_t sequence, unsigned length)
{
    uint16_t crc;
    tx[0]=0xa5; tx[1]=0x5a; tx[2]=(uint8_t)type; tx[3]=(uint8_t)length;
    put32(tx+4,sequence);
    crc=crc16(tx+2,length+6);
    memcpy(tx+8+length,&crc,2);
    tx_length=length+10;
    tx_limit=(tx_length*10000000U+uart_config.baud-1U)/uart_config.baud+1000U;
    DMA1_CHANNEL3->ctrl_bit.chen=0;
    DMA1->clr=DMA1_GL3_FLAG;
    DMA1_CHANNEL3->dtcnt=tx_length;
    USART2->sts=~USART_TDC_FLAG;
    tx_started=micros();
    __DMB();
    DMA1_CHANNEL3->ctrl_bit.chen=1;
}

static void reply(unsigned result, unsigned extra)
{
    tx[8]=request[8]; tx[9]=(uint8_t)result;
    send_frame(0x11,get32(request+4),extra+2);
    request_length=0;
}

static void receive_command(void)
{
    uint32_t end=received();
    unsigned budget=64, size, i;
    uint16_t crc;
    if((uint32_t)(end-rx_read)>sizeof(rx)) {
        rx_errors++; rx_read=end; partial_us=0; return;
    }
    while(budget && rx_read!=end) {
        if(rx[rx_read&255]!=0xa5) goto skip;
        if((uint32_t)(end-rx_read)<4) goto partial;
        if(rx[(rx_read+1)&255]!=0x5a || rx[(rx_read+2)&255]!=0x10) goto skip;
        size=rx[(rx_read+3)&255]+10U;
        if(size<11 || size>sizeof(request)) { rx_errors++; goto skip; }
        if((uint32_t)(end-rx_read)<size) goto partial;
        if(size>budget) return;
        for(i=0;i<size;i++) request[i]=rx[(rx_read+i)&255];
        if((uint32_t)(received()-rx_read)>sizeof(rx)) {
            rx_errors++; rx_read=received(); partial_us=0; return;
        }
        memcpy(&crc,request+size-2,2);
        budget-=size-1; /* CRC/copy work also consumes the 64-byte budget. */
        if(crc!=crc16(request+2,size-4)) { crc_errors++; goto skip; }
        rx_read+=size; partial_us=0; request_length=size;
        return;
partial:
        if(!partial_us) partial_us=micros();
        if((uint32_t)(micros()-partial_us)<10000) return;
        rx_errors++;
skip:
        rx_read++; budget--; partial_us=0;
    }
}

/* Shared by query 08 and the one-second unsolicited health frame. */
static unsigned diagnostic(uint8_t *p, const IMUState *s)
{
    const uint32_t values[]={
        1,boot_error,system_core_clock,sensor_status.phase,sensor_status.error,
        sensor_status.id,sensor_status.drdy_count,sensor_status.fallback_reads,
        sensor_status.last_status,GPIOB->idt&1U,
        s->warmup ? micros()-s->time_us:0xffffffffU,state_flags(s),
        sensor_status.config_index,s->frames
    };
    memcpy(p,values,sizeof(values));
    return sizeof(values);
}

static void command(const IMUState *s)
{
    unsigned result=CAL_OK, extra=0;
    uint32_t key;
    switch(request[8]) {
    case 1: {
        if(request[3]!=1) { result=UART_BAD_ARGUMENT; break; }
        key=__get_PRIMASK(); __disable_irq();
        const uint32_t values[]={
            1,uart_config.baud,uart_config.rate,uart_config.mask,state_flags(s),
            s->frames,s->time_us,heartbeat_ms,sensor_status.id,sensor_status.phase,
            sensor_status.error,s->missed,s->late,s->no_data,s->invalid,
            sensor_status.dma_timeouts,sensor_status.dma_errors,sensor_status.overlaps,
            tx_dropped,tx_errors,rx_errors,crc_errors,
            cal_status.mode,cal_status.state,cal_status.poses,cal_status.samples,
            cal_status.stable,cal_status.total_samples,cal_status.error,
            cal_status.solve_steps,s->save_result
        };
        memcpy(tx+10,values,sizeof(values));
        __set_PRIMASK(key);
        memcpy(tx+10+sizeof(values),&cal_status.fit_rms,4);
        memcpy(tx+14+sizeof(values),&cal_status.max_norm_error,4);
        extra=sizeof(values)+8;
        break;
    }
    case 2:
        if(boot_error) { result=UART_NOT_READY; break; }
        if(request[3]!=8) { result=UART_BAD_ARGUMENT; break; }
        next_config.baud=get32(request+9);
        memcpy(&next_config.rate,request+13,2);
        next_config.mask=request[15]; next_config.reserved=0;
        result=(unsigned)UART_ConfigCheck(&next_config);
        if(result) break;
        pending=decimation=0;
        if(next_config.baud!=uart_config.baud) phase=SWITCH_ACK;
        else uart_config=next_config;
        break;
    case 3:
        if(request[3]!=2 || request[9]>1) { result=UART_BAD_ARGUMENT; break; }
        enabled=request[9]; pending=decimation=0;
        break;
    case 4:
        if(request[3]!=2 || request[9]<1 || request[9]>5) {
            result=UART_BAD_ARGUMENT; break;
        }
        result=(unsigned)IMU_Command(request[9]);
        break;
    case 5:
        if(boot_error) { result=UART_NOT_READY; break; }
        if(request[3]!=1) { result=UART_BAD_ARGUMENT; break; }
        phase=SAVE_DRAIN; switch_us=micros(); pending=0; return;
    case 6:
        if(request[3]!=2 || request[9]>=PROF_COUNT) {
            result=UART_BAD_ARGUMENT; break;
        }
        tx[10]=request[9];
        key=__get_PRIMASK(); __disable_irq();
        put32(tx+11,system_core_clock);
        put32(tx+15,micros()-profile_since);
        memcpy(tx+19,&idle_cycles,8);
        memcpy(tx+27,&profiles[request[9]],sizeof(Profile));
        __set_PRIMASK(key);
        extra=17+sizeof(Profile);
        break;
    case 7:
        if(request[3]!=1) { result=UART_BAD_ARGUMENT; break; }
        result=(unsigned)IMU_Command(CMD_PROFILE_RESET);
        break;
    case 8:
        if(request[3]!=1) { result=UART_BAD_ARGUMENT; break; }
        extra=diagnostic(tx+10,s);
        break;
    case 9:
        if(request[3]!=1) { result=UART_BAD_ARGUMENT; break; }
        memcpy(tx+10,&cal_params,sizeof(cal_params));
        extra=sizeof(cal_params);
        break;
    case 10:
        if(request[3]!=1) { result=UART_BAD_ARGUMENT; break; }
        result=(unsigned)IMU_Command(CMD_RESTART);
        break;
#ifdef IMU_TEST
    case 0xf0: /* Test firmware only: no samples / stalled sensor DMA / restart. */
        if(request[3]!=2 || request[9]>2) { result=UART_BAD_ARGUMENT; break; }
        test_fault=request[9];
        if(!test_fault) result=(unsigned)IMU_Command(CMD_RESTART);
        break;
#endif
    default: result=CAL_BAD_COMMAND; break;
    }
    reply(result,extra);
}

static void process(void)
{
    const IMUState *s=IMU_GetState();
    uint8_t *p;
    unsigned result;
    if(rx_fault || (USART2->sts & (USART_PERR_FLAG|USART_FERR_FLAG|USART_NERR_FLAG|USART_ROERR_FLAG))) {
        rx_errors++; receive_reset();
    }
    if(s->frames!=last_frame) {
        last_frame=s->frames;
        if(enabled && phase==RUN && ++decimation>=1000U/uart_config.rate) {
            if(pending) tx_dropped++;
            pending=1; decimation=0;
        }
    }
    if(tx_length) {
        if((DMA1->sts & DMA1_DTERR3_FLAG) || (uint32_t)(micros()-tx_started)>=tx_limit) {
            DMA1_CHANNEL3->ctrl_bit.chen=0;
            DMA1->clr=DMA1_GL3_FLAG;
            tx_length=0; tx_errors++;
            crm_periph_reset(CRM_USART2_PERIPH_RESET,TRUE);
            crm_periph_reset(CRM_USART2_PERIPH_RESET,FALSE);
            baud_apply();
            if(phase==SWITCH_ACK) phase=RUN; /* Failed ACK: retain the old baud. */
        } else if((DMA1->sts & DMA1_FDT3_FLAG) && (USART2->sts & USART_TDC_FLAG)) {
            DMA1_CHANNEL3->ctrl_bit.chen=0;
            DMA1->clr=DMA1_GL3_FLAG;
            tx_length=0;
        }
    }
    if(phase==SWITCH_ACK) {
        if(!tx_length) { switch_us=micros(); phase=SWITCH_DELAY; }
        return;
    }
    if(phase==SWITCH_DELAY) {
        if((uint32_t)(micros()-switch_us)>=20000) {
            uart_config=next_config;
            baud_apply(); phase=RUN; pending=decimation=0;
        }
        return;
    }
    if(phase==SAVE_DRAIN) {
        if(tx_length || !(USART2->sts & USART_TDC_FLAG)) {
            if((uint32_t)(micros()-switch_us)>=5000U) {
                phase=RUN; reply(UART_SAVE_FAILED,0);
            }
            return;
        }
        result=(unsigned)IMU_Command(CMD_SAVE);
        if(result) { phase=RUN; reply(result,0); }
        else phase=SAVE_WAIT;
        return;
    }
    if(phase==SAVE_WAIT) {
        if(!s->maintenance) {
            receive_reset(); phase=RUN; pending=decimation=0;
            reply(s->save_result==1 ? CAL_OK:UART_SAVE_FAILED,0);
        }
        return;
    }
    if(!request_length) receive_command();
    if(tx_length) return;
    if(request_length) { command(s); return; }
    if((boot_error || !enabled || sensor_status.phase!=6 || !s->warmup ||
        (uint32_t)(micros()-s->time_us)>=10000U) &&
       (uint32_t)(micros()-report_us)>=1000000U) {
        report_us=micros();
        send_frame(0x12,heartbeat_ms,diagnostic(tx+8,s));
        return;
    }
    if(!pending) return;
    pending=0;
    if(sensor_status.phase!=6 || (uint32_t)(micros()-s->time_us)>=10000) return;
    put32(tx+8,s->time_us);
    tx[12]=state_flags(s); tx[13]=uart_config.mask;
    p=tx+14;
    if(uart_config.mask&1) { memcpy(p,s->euler,12); p+=12; }
    if(uart_config.mask&2) { memcpy(p,s->q,16); p+=16; }
    if(uart_config.mask&4) {
        memcpy(p,s->gyro,12); memcpy(p+12,s->accel,12); p+=24;
    }
    if(uart_config.mask&8) {
        memcpy(p,s->filtered_gyro,12); memcpy(p+12,s->filtered_accel,12); p+=24;
    }
    send_frame(1,s->frames,(unsigned)(p-tx-8));
}

void UART_Process(void)
{
    uint32_t start=cycles();
    unsigned maintenance=phase>=SAVE_DRAIN;
    process();
    if(!maintenance && phase<SAVE_DRAIN) profile_add(PROF_UART,cycles()-start);
}
