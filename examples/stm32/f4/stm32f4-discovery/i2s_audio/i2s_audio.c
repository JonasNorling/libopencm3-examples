/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Jonas Norling <jonas.norling@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencmsis/core_cm3.h>

static const uint8_t DAC_I2C_ADDR = 0x94 >> 1;
static const uint32_t DAC_I2C = I2C1;
static const uint32_t DAC_I2S = SPI3;
static const uint32_t DAC_DMA_STREAM = DMA_STREAM5;
static const uint32_t DAC_DMA_CHANNEL = DMA_SxCR_CHSEL_0;
static const uint32_t MIC_I2S = SPI2;
static const uint32_t MIC_DMA_STREAM = DMA_STREAM3;
static const uint32_t MIC_DMA_CHANNEL = DMA_SxCR_CHSEL_0;

#define DAC_BUFFER_LEN 1024
static int16_t dac_buffer[DAC_BUFFER_LEN];

#define MIC_BUFFER_LEN 128
/* Align DMA buffers so we can treat them as 32-bit entities */
static uint16_t mic_buffer1[MIC_BUFFER_LEN] __attribute__((aligned(4)));
static uint16_t mic_buffer2[MIC_BUFFER_LEN] __attribute__((aligned(4)));

/*
 * Write a register in the DAC over the I2C bus.
 */
static void write_dac_register(uint8_t reg, uint8_t value)
{
	i2c_send_start(DAC_I2C);
	while (!((I2C_SR1(DAC_I2C) & I2C_SR1_SB)
		&& (I2C_SR2(DAC_I2C) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	/* Send the address, wait for it to be transfered, then also read SR2
	 * in accordance with the STM32 manual. */
	i2c_send_7bit_address(DAC_I2C, DAC_I2C_ADDR, I2C_WRITE);
	while (!(I2C_SR1(DAC_I2C) & I2C_SR1_ADDR));
	volatile uint8_t r __attribute__((unused)) = I2C_SR2(DAC_I2C);
	/* Send the register number, wait for the transfer to finish. */
	i2c_send_data(DAC_I2C, reg);
	while (!(I2C_SR1(DAC_I2C) & I2C_SR1_BTF));
	/* Send the register value, wait for the transfer to finish. */
	i2c_send_data(DAC_I2C, value);
	while (!(I2C_SR1(DAC_I2C) & (I2C_SR1_BTF | I2C_SR1_TxE)));
	i2c_send_stop(DAC_I2C);
}

static void config_clocks(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

	/* Enable clocks for the peripherals we'll be using */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_SPI2);
	rcc_periph_clock_enable(RCC_SPI3);
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_DMA1);
}

static void config_gpio(void)
{
	/* GPIO pins connected to LEDs (PD12, PD13, PD14, PD15) */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
			GPIO12 | GPIO13 | GPIO14 | GPIO15);
	gpio_clear(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);

	/* GPIO pin for DAC reset */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);

	/* I2S3 alternate pin function mapping (DAC) */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO7 | GPIO10 | GPIO12);
	gpio_set_af(GPIOC, GPIO_AF6, GPIO7 | GPIO10 | GPIO12);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
	gpio_set_af(GPIOA, GPIO_AF6, GPIO4);

	/* I2S2 alternate pin function mapping (MEMS microphone) */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
	gpio_set_af(GPIOC, GPIO_AF5, GPIO3);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	gpio_set_af(GPIOB, GPIO_AF5, GPIO10);

	/* I2C1 alternate pin function mapping (DAC control) */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO6 | GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO6 | GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ,
			GPIO6 | GPIO9);
}

static void config_dac_i2s(void)
{
	/* Set up I2S for 48kHz 16-bit stereo and turn I2S on.
	 * The input to the PLLI2S is 1MHz. Division factors are from table 126
	 * in the data sheet.
	 *
	 * This gives us 1.536MHz SCLK = 16 bits * 2 channels * 48000 Hz
	 * and 12.288 MHz MCLK = 256 * 48000 Hz. */

	spi_reset(DAC_I2S);

	RCC_PLLI2SCFGR = (3 << 28) | (258 << 6);
	RCC_CR |= RCC_CR_PLLI2SON;

	const uint32_t i2sdiv = 3 | SPI_I2SPR_ODD;
	SPI_I2SPR(DAC_I2S) = SPI_I2SPR_MCKOE | i2sdiv;
	SPI_I2SCFGR(DAC_I2S) |= SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SE |
			(SPI_I2SCFGR_I2SCFG_MASTER_TRANSMIT << SPI_I2SCFGR_I2SCFG_LSB);
}

static void config_dac_i2c(void)
{
	/* Configure the I2C bus based on an APB1 frequency of 12MHz. The
	 * CCR value used here lands us in the right frequency ballbark. */
	i2c_reset(DAC_I2C);
	i2c_set_clock_frequency(DAC_I2C, I2C_CR2_FREQ_42MHZ);
	i2c_set_ccr(DAC_I2C, 500);
	i2c_peripheral_enable(DAC_I2C);
}

static void config_dac(void)
{
	/* Reset the DAC and configure the I2S bus and I2C bus. */
	gpio_clear(GPIOD, GPIO4);
	config_dac_i2s();
	config_dac_i2c();
	gpio_set(GPIOD, GPIO4);

	/* Turn off speaker output, turn on headphone output */
	write_dac_register(0x04, 0xAF);

	/* Auto detect I2S clock */
	write_dac_register(0x05, 0x81);

	/* Work as an I2S slave */
	write_dac_register(0x06, 0x04);

	/* Set volume */
	write_dac_register(0x20, 0xe0);
	write_dac_register(0x21, 0xe0);

	/* Power up the DAC */
	write_dac_register(0x02, 0x9E);
}

static void config_microphone_i2s(void)
{
	/* The MEMS microphone sends a PDM bitstream at 1.0 to 3.25 MHz.
	 * Here it is clocked at 3.072MHz, which is 48kHz * 64, so we get 64
	 * bits from the microphone for each sample we send to the DAC. This
	 * relationship is exact: both I2S masters are clocked from the same
	 * PLL and their prescalers have a fixed relationship. */

	spi_reset(MIC_I2S);

	const uint32_t i2sdiv = 14;
	SPI_I2SPR(MIC_I2S) = i2sdiv;
	SPI_I2SCFGR(MIC_I2S) |= SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SE |
			(SPI_I2SCFGR_I2SCFG_MASTER_RECEIVE << SPI_I2SCFGR_I2SCFG_LSB);
}

static void config_dma(void)
{
	/* Configure the DMA engine to stream data to the DAC. */
	dma_stream_reset(DMA1, DAC_DMA_STREAM);
	dma_set_peripheral_address(DMA1, DAC_DMA_STREAM,
				   (intptr_t)&SPI_DR(DAC_I2S));
	dma_set_memory_address(DMA1, DAC_DMA_STREAM, (intptr_t)dac_buffer);
	dma_set_number_of_data(DMA1, DAC_DMA_STREAM, DAC_BUFFER_LEN);
	dma_channel_select(DMA1, DAC_DMA_STREAM, DAC_DMA_CHANNEL);
	dma_set_transfer_mode(DMA1, DAC_DMA_STREAM,
			DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	dma_set_memory_size(DMA1, DAC_DMA_STREAM, DMA_SxCR_MSIZE_16BIT);
	dma_set_peripheral_size(DMA1, DAC_DMA_STREAM, DMA_SxCR_PSIZE_16BIT);
	dma_enable_memory_increment_mode(DMA1, DAC_DMA_STREAM);
	dma_enable_circular_mode(DMA1, DAC_DMA_STREAM);
	dma_enable_stream(DMA1, DAC_DMA_STREAM);

	/* Configure the DMA engine to stream data from the MEMS microphone. */
	dma_stream_reset(DMA1, MIC_DMA_STREAM);
	dma_set_peripheral_address(DMA1, MIC_DMA_STREAM, 
				   (intptr_t)&SPI_DR(MIC_I2S));
	dma_set_memory_address(DMA1, MIC_DMA_STREAM, (intptr_t)mic_buffer1);
	dma_set_memory_address_1(DMA1, MIC_DMA_STREAM, (intptr_t)mic_buffer2);
	dma_set_number_of_data(DMA1, MIC_DMA_STREAM, MIC_BUFFER_LEN);
	dma_channel_select(DMA1, MIC_DMA_STREAM, MIC_DMA_CHANNEL);
	dma_set_transfer_mode(DMA1, MIC_DMA_STREAM,
			DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	dma_set_memory_size(DMA1, MIC_DMA_STREAM, DMA_SxCR_MSIZE_16BIT);
	dma_set_peripheral_size(DMA1, MIC_DMA_STREAM, DMA_SxCR_PSIZE_16BIT);
	dma_enable_memory_increment_mode(DMA1, MIC_DMA_STREAM);
	dma_enable_double_buffer_mode(DMA1, MIC_DMA_STREAM);
	dma_enable_transfer_complete_interrupt(DMA1, MIC_DMA_STREAM);
	dma_enable_stream(DMA1, MIC_DMA_STREAM);

	/* Enable interrupts on finished DMA transfers from the microphone. */
	nvic_enable_irq(NVIC_DMA1_STREAM3_IRQ);
}

static float lowpass_old(float x)
{
	// http://www.micromodeler.com/dsp/
	// 4'th order butterworth, cutoff at 0.003 Fs
	const float c[] = {
		0.00012614203661854813, 0.00025228407323709626, 0.00012614203661854813,
		1.965419503383591, -0.9657687175161379,// b0, b1, b2, a1, a2
		0.00006103515625, 0.0001220703125, 0.00006103515625,
		1.9853245870936216, -0.9856773379451891// b0, b1, b2, a1, a2
	};

	static float s[4];
	float next_s[4];

	next_s[0] = x + c[3] * s[0] + c[4] * s[1];
	next_s[1] = s[0];
	const float y1 = c[0] * next_s[0] + c[1] * s[0] + c[2] * s[1];

	next_s[2] = y1 + c[8] * s[2] + c[9] * s[3];
	next_s[3] = s[2];
	const float y2 = c[5] * next_s[2] + c[6] * s[2] + c[7] * s[3];

	s[0] = next_s[0];
	s[1] = next_s[1];
	s[2] = next_s[2];
	s[3] = next_s[3];
	
	return y2;
}

static float lowpass(float x)
{
	/* 4'th order butterworth, cutoff at 0.003 Fs.
	 * Coefficients from http://www.micromodeler.com/dsp/ */
	{
		const float b[] = { 0.00012614203661854813,
				    0.00025228407323709626, 0.00012614203661854813 };
		const float a[] = { 1.965419503383591, -0.9657687175161379 };

		static float s[2];
		float next_s[2];

		next_s[0] = x + a[0] * s[0] + a[1] * s[1];
		next_s[1] = s[0];
		x = b[0] * next_s[0] + b[1] * s[0] + b[2] * s[1];
		s[0] = next_s[0];
		s[1] = next_s[1];
	}

	{
		const float b[] = { 0.00006103515625,
				    0.0001220703125, 0.00006103515625 };
		const float a[] = { 1.9853245870936216, -0.9856773379451891 };

		static float s[2];
		float next_s[2];

		next_s[0] = x + a[0] * s[0] + a[1] * s[1];
		next_s[1] = s[0];
		x = b[0] * next_s[0] + b[1] * s[0] + b[2] * s[1];
		s[0] = next_s[0];
		s[1] = next_s[1];
	}
	
	return x;
}

static float dcblock(float x)
{
	const float a = 0.995f;
	static float oldx, oldy;
        oldy = x - oldx + a * oldy;
	oldx = x;
	return oldy;
}

/* Interrupt for finished transfer from the microphone */
void dma1_stream3_isr(void)
{
	static unsigned outpt = 0;
	unsigned outsample;
	unsigned inword;
	unsigned inbit;
	float y;

	gpio_clear(GPIOD, GPIO12); /* Green LED */
	gpio_set(GPIOD, GPIO13); /* Amber LED */
	
	dma_clear_interrupt_flags(DMA1, MIC_DMA_STREAM, DMA_TCIF);

	/*
	 * The PDM data from the microphone needs to be converted to PCM samples
	 * that can be used by the DAC. To do this we
	 *  1. Scale the bitstream to a stream of rail-to-rail 16-bit samples.
	 *  2. Lowpass filter the samples with a cutoff well below the Nyquist
	 *     frequency of the target sample rate.
	 *  3. Decimate by picking out every 64'th sample.
	 *  4. Highpass filter with a cutoff below hearing to remove any DC
	 *     bias.
	 *
	 * See ST AN3998 and the Wikipedia article on PDM.
	 */

	const uint32_t* in = dma_get_target(DMA1, MIC_DMA_STREAM) ?
			(const uint32_t*)mic_buffer2 :
			(const uint32_t*)mic_buffer1;

	for (outsample = 0; outsample < sizeof(mic_buffer1) / 8; outsample++) {
		for (inword = 0; inword < 2; inword++) {
			for (inbit = 0; inbit < 32; inbit++) {
				const float x = ((*in >> inbit) & 1) - 1.0f;
				y = lowpass(x);
			}
			in++;
		}

		y = dcblock(y);

		/* Store stereo samples, downsampling by 64 */
		dac_buffer[outpt++ % DAC_BUFFER_LEN] = y * 0x8000;
		dac_buffer[outpt++ % DAC_BUFFER_LEN] = y * 0x8000;
	}

	if (!(SPI_CR2(DAC_I2S) & SPI_CR2_TXDMAEN)) {
		/* The data stream to the DAC hasn't been enabled yet, so
		 * start it now. */
		spi_enable_tx_dma(DAC_I2S);
	}

	gpio_clear(GPIOD, GPIO13); /* Amber LED */
	gpio_set(GPIOD, GPIO12); /* Green LED */
}

int main(void)
{
	config_clocks();
	config_gpio();
	config_dac();
	config_microphone_i2s();
	config_dma();

	/* Start data flow from the microphone. The data flow to the DAC is
	 * started after the first block comes back from the microphone to give
	 * it some head start. */
        spi_enable_rx_dma(MIC_I2S);

	while (1) {
		/* Go to sleep until an interrupt happens. */
		__WFI();
		gpio_toggle(GPIOD, GPIO15); /* Blue LED */
	}

	return 0;
}
