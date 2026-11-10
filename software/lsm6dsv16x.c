#include "lsm6dsv16x.h"
#include <string.h>

SensorStatus sensor_status;
static uint8_t tx[13], rx[13];
static volatile uint32_t pending, edge_us, edge_cycles, dma_done, dma_error, dma_end_cycles;
static uint32_t bus_active, bus_start_us, deadline, frame_us, frame_cycles, burst;
static const uint8_t config[][2] = {
    {0x12,0x44}, {0x10,0x10}, {0x11,0x10}, {0x62,0x01},
    {0x15,0x04}, {0x17,0x03}, {0x11,0x19}, {0x10,0x19}, {0x0d,0x02}
};
#ifdef IMU_TEST
volatile uint32_t test_fault;
#endif

void EXINT0_IRQHandler(void)
{
    uint32_t start = cycles();
    exint_flag_clear(EXINT_LINE_0);
    if (pending) sensor_status.overlaps++;
    edge_us = micros();
    edge_cycles = cycles();
    pending = 1;
    sensor_status.drdy_count++;
    profile_add(PROF_IRQ, cycles() - start);
}

void DMA1_Channel1_IRQHandler(void)
{
    uint32_t start = cycles();
    dma_error = dma_flag_get(DMA1_DTERR1_FLAG) != RESET;
    dma_done = dma_flag_get(DMA1_FDT1_FLAG) != RESET;
    dma_end_cycles = cycles();
    dma_flag_clear(DMA1_GL1_FLAG);
    profile_add(PROF_IRQ, cycles() - start);
}

static void bus_end(void)
{
    dma_channel_enable(DMA1_CHANNEL1, FALSE);
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    gpio_bits_set(GPIOA, GPIO_PINS_4);
    bus_active = 0;
}

void sensor_stop(void)
{
    NVIC_DisableIRQ(EXINT0_IRQn);
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    bus_end();
    sensor_status.phase = 7;
    pending = 0;
}

static void bus_start(uint8_t reg, uint8_t value, unsigned len, int read)
{
    memset(tx, 0, sizeof(tx));
    tx[0] = reg | (read ? 0x80 : 0);
    tx[1] = value;
    dma_done = dma_error = 0;
    dma_flag_clear(DMA1_GL1_FLAG | DMA1_GL2_FLAG);
    dma_data_number_set(DMA1_CHANNEL1, len);
    dma_data_number_set(DMA1_CHANNEL2, len);
    bus_start_us = micros();
    bus_active = 1;
    gpio_bits_reset(GPIOA, GPIO_PINS_4);
#ifdef IMU_TEST
    if (test_fault == 2) return;
#endif
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
    dma_channel_enable(DMA1_CHANNEL2, TRUE);
}

static int bus_poll(void)
{
    if (dma_error || (uint32_t)(micros() - bus_start_us) >= 250) {
        if (dma_error) sensor_status.dma_errors++;
        else sensor_status.dma_timeouts++;
        bus_end();
        return -1;
    }
    if (!dma_done || spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) != RESET) return 0;
    bus_end();
    return 1;
}

void sensor_init(void)
{
    gpio_init_type gpio;
    spi_init_type spi;
    dma_init_type dma;
    exint_init_type ext;
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
    sensor_stop();
    spi_i2s_reset(SPI1);
    gpio_default_para_init(&gpio);
    gpio_bits_set(GPIOA, GPIO_PINS_4);
    gpio.gpio_pins = GPIO_PINS_4;
    gpio.gpio_mode = GPIO_MODE_OUTPUT;
    gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio);
    gpio.gpio_pins = GPIO_PINS_5 | GPIO_PINS_6 | GPIO_PINS_7;
    gpio.gpio_mode = GPIO_MODE_MUX;
    gpio_init(GPIOA, &gpio);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE5, GPIO_MUX_5);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE6, GPIO_MUX_5);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_5);
    gpio.gpio_pins = GPIO_PINS_0;
    gpio.gpio_mode = GPIO_MODE_INPUT;
    gpio.gpio_pull = GPIO_PULL_DOWN;
    gpio_init(GPIOB, &gpio);
    spi_default_para_init(&spi);
    spi.master_slave_mode = SPI_MODE_MASTER;
    spi.mclk_freq_division = SPI_MCLK_DIV_32;
    spi.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
    spi.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(SPI1, &spi);
    dmamux_enable(DMA1, TRUE);
    dma_reset(DMA1_CHANNEL1);
    dma_reset(DMA1_CHANNEL2);
    dma_default_para_init(&dma);
    dma.memory_base_addr = (uint32_t)rx;
    dma.peripheral_base_addr = (uint32_t)&SPI1->dt;
    dma.memory_inc_enable = TRUE;
    dma.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma.priority = DMA_PRIORITY_HIGH;
    dma.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init(DMA1_CHANNEL1, &dma);
    dma.memory_base_addr = (uint32_t)tx;
    dma.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init(DMA1_CHANNEL2, &dma);
    dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_SPI1_RX);
    dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_SPI1_TX);
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT | DMA_DTERR_INT, TRUE);
    spi_i2s_dma_receiver_enable(SPI1, TRUE);
    spi_i2s_dma_transmitter_enable(SPI1, TRUE);
    spi_enable(SPI1, TRUE);
    scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, SCFG_PINS_SOURCE0);
    exint_default_para_init(&ext);
    ext.line_select = EXINT_LINE_0;
    ext.line_enable = TRUE;
    ext.line_mode = EXINT_LINE_INTERRUPUT;
    ext.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&ext);
    exint_flag_clear(EXINT_LINE_0);
    NVIC_ClearPendingIRQ(EXINT0_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    NVIC_SetPriority(EXINT0_IRQn, 2);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 2);
    NVIC_EnableIRQ(EXINT0_IRQn);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    pending = burst = 0;
    sensor_status.id = sensor_status.error = sensor_status.config_index = 0;
    sensor_status.phase = 0;
    deadline = micros() + 100000;
}

int sensor_poll(ImuSample *sample)
{
    int done = 0;
    unsigned i;
    if (sensor_status.phase == 7) return -1;
    if (bus_active) {
        done = bus_poll();
        if (!done) return 0;
        if (done < 0) { sensor_status.error = 4; sensor_status.phase = 7; return -1; }
    }
    switch (sensor_status.phase) {
    case 0:
        if ((int32_t)(micros() - deadline) < 0) return 0;
        sensor_status.phase = 1;
        bus_start(0x0f, 0, 2, 1);
        break;
    case 1:
        sensor_status.id = rx[1];
        if (rx[1] != 0x70) { sensor_status.error = 1; sensor_status.phase = 7; return -1; }
        sensor_status.phase = 2;
        bus_start(0x12, 1, 2, 0);
        deadline = micros() + 100000;
        break;
    case 2:
        sensor_status.phase = 3;
        bus_start(0x12, 0, 2, 1);
        break;
    case 3:
        if (rx[1] & 1) {
            if ((int32_t)(micros() - deadline) >= 0) {
                sensor_status.error = 2; sensor_status.phase = 7; return -1;
            }
            bus_start(0x12, 0, 2, 1);
        } else {
            sensor_status.phase = 4;
            bus_start(config[0][0], config[0][1], 2, 0);
        }
        break;
    case 4:
        /* HAODR mode/ODR changes need settling; no busy delay. */
        if (done) deadline = micros() + 1000;
        if ((int32_t)(micros() - deadline) < 0) return 0;
        sensor_status.phase = 5;
        bus_start(config[sensor_status.config_index][0], 0, 2, 1);
        break;
    case 5:
        if (rx[1] != config[sensor_status.config_index][1]) {
            sensor_status.error = 3; sensor_status.phase = 7; return -1;
        }
        if (++sensor_status.config_index == sizeof(config)/sizeof(config[0])) {
            sensor_status.phase = 6;
            pending = 0;
            deadline = micros();
        } else {
            sensor_status.phase = 4;
            bus_start(config[sensor_status.config_index][0], config[sensor_status.config_index][1], 2, 0);
        }
        break;
    case 6:
        if (done && burst) {
            burst = 0;
            for (i = 0; i < 3; i++) {
                sample->gyro[i] = (int16_t)(rx[1+2*i] | (rx[2+2*i]<<8));
                sample->accel[i] = (int16_t)(rx[7+2*i] | (rx[8+2*i]<<8));
            }
            sample->time_us = frame_us;
            sample->edge_cycles = frame_cycles;
            profile_add(PROF_ACQUIRE, dma_end_cycles - frame_cycles);
            return 1;
        }
        if (done && (rx[1] & 3) == 3) {
            pending = 0;
            burst = 1;
            bus_start(0x22, 0, 13, 1);
            return 0;
        }
#ifdef IMU_TEST
        if (test_fault == 1) return 0;
#endif
        if ((int32_t)(micros() - deadline) < 0) return 0;
        if (pending || gpio_input_data_bit_read(GPIOB, GPIO_PINS_0) != RESET) {
            frame_us = pending ? edge_us : micros();
            frame_cycles = pending ? edge_cycles : cycles();
            deadline = micros() + 100;
            bus_start(0x1e, 0, 2, 1);
        }
        break;
    }
    return 0;
}
