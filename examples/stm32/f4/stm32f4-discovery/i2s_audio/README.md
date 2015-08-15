# README

This is an example program demonstating the use of I2S, I2C and DMA on
STM32F4. It runs on an STM32F4 Discovery board, using the on-board
MEMS microphone and audio DAC.

The program implements a passthrough between the microphone and the
headphone output, doing the minimum amount of DSP to translate the
microphone's 1-bit PDM stream into PCM samples for the DAC. There is
plenty of white noise generated in the process, probably from aliasing
in the decimation, but the point here is to demonstrate I2S without
focusing too much of the application specific math.


## Hardware and pin mappings on the Discovery board

U7: CS43L22 audio DAC
 - I2C address 0x94
 - Connected to I2S3

U9: MP45DT02 microphone

- PD4             - CS43L22 /RESET
- PC7 (I2S3_MCK)  - CS43L22 MCLK
- PC10 (I2S3_CK)  - CS43L22 SCLK
- PC12 (I2S3_SD)  - CS43L22 SDIN
- PA4 (I2S3_WS)   - CS43L22 LRCK

- PB6 (I2C1_SCL)  - CS43L22 SCL (pull-up on board)
- PB9 (I2C1_SDA)  - CS43L22 SDA (pull-up on board)

- PC3 (I2S2_SD)   - MP45DT02 DOUT
- PB10 (I2S2_CK)  - MP45DT02 CLK

- PD12 (TIM4_CH1) - LED4
- PD13 (TIM4_CH2) - LED5
- PD14 (TIM4_CH3) - LED6
- PD15 (TIM4_CH4) - LED7
