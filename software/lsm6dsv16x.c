#include "lsm6dsv16x.h"

static uint8_t transfer(uint8_t value)
{
    while (spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET);
    spi_i2s_data_transmit(SPI1, value);
    while (spi_i2s_flag_get(SPI1, SPI_I2S_RDBF_FLAG) == RESET);
    return (uint8_t)spi_i2s_data_receive(SPI1);
}

void imu_spi_init(void)
{
    gpio_init_type gpio;
    spi_init_type spi;
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
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
    spi_default_para_init(&spi);
    spi.master_slave_mode = SPI_MODE_MASTER;
    spi.mclk_freq_division = SPI_MCLK_DIV_16;
    spi.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
    spi.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(SPI1, &spi);
    spi_enable(SPI1, TRUE);
}

uint8_t imu_reg_read(uint8_t reg)
{
    uint8_t value;
    gpio_bits_reset(GPIOA, GPIO_PINS_4);
    transfer(reg | 0x80);
    value = transfer(0);
    gpio_bits_set(GPIOA, GPIO_PINS_4);
    return value;
}

void imu_reg_write(uint8_t reg, uint8_t value)
{
    gpio_bits_reset(GPIOA, GPIO_PINS_4);
    transfer(reg);
    transfer(value);
    gpio_bits_set(GPIOA, GPIO_PINS_4);
}

int imu_read(int16_t gyro[3], int16_t accel[3])
{
    uint8_t data[12];
    unsigned i;
    if ((imu_reg_read(0x1e) & 3) != 3)
        return 0;
    gpio_bits_reset(GPIOA, GPIO_PINS_4);
    transfer(0x22 | 0x80); /* OUTX_L_G through OUTZ_H_A */
    for (i = 0; i < sizeof(data); i++)
        data[i] = transfer(0);
    gpio_bits_set(GPIOA, GPIO_PINS_4);
    for (i = 0; i < 3; i++)
    {
        gyro[i] = (int16_t)(data[2*i] | (data[2*i+1] << 8));
        accel[i] = (int16_t)(data[6+2*i] | (data[7+2*i] << 8));
    }
    return 1;
}
