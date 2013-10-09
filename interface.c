/*
Controls communication with encoders and shift registers
Used to read buttons and write to LEDs
*/

#include "interface.h"

/**
Initialise all the GPIO pins being used
They connect to the encoders (buttons) and shift registers (LEDs)
*/
void Interface_Init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// Initialize buttons
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStruct.GPIO_Pin = Encoder1_Q0 | Encoder1_Q1 | Encoder1_Q2 | Encoder1_EN;
	GPIO_Init(Encoder1_Port, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = Encoder2_Q0 | Encoder2_Q1 | Encoder2_Q2 | Encoder2_EN;
	GPIO_Init(Encoder2_Port, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = Encoder1_Q0 | Encoder3_Q1 | Encoder3_Q2 | Encoder3_EN;
	GPIO_Init(Encoder1_Port, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	// Initialize LEDs
	GPIO_InitStruct.GPIO_Pin = Shifter1_DS | Shifter1_CLK;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStruct.GPIO_Pin = Shifter1_DS | Shifter1_CLK;
	GPIO_Init(Shifter1_Port, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = Shifter2_DS | Shifter2_CLK;
	GPIO_Init(Shifter2_Port, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = Shifter3_DS | Shifter3_CLK;
	GPIO_Init(Shifter3_Port, &GPIO_InitStruct);
}

/**
Write to the LEDs by writing to the shift register
Instead of using an SPI peripheral, a GPIO pin is manually toggled to
simulate the clock line
*/
void Interface_SetFunctionLED(uint8_t bits)
{
	int i=0;
	uint8_t mask = 0b10000000;
	for(; i<8; i++)
	{
		if(mask & bits)
			GPIO_SetBits(Shifter1_Port, Shifter1_DS);
		else
			GPIO_ResetBits(Shifter1_Port, Shifter1_DS);

		mask = mask>>1;

		GPIO_SetBits(Shifter1_Port, Shifter1_CLK);
		GPIO_ResetBits(Shifter1_Port, Shifter1_CLK);
	}
	GPIO_SetBits(Shifter1_Port, Shifter1_CLK);
	GPIO_ResetBits(Shifter1_Port, Shifter1_CLK);
}

/**
Write to the LEDs by writing to the shift register
Instead of using an SPI peripheral, a GPIO pin is manually toggled to
simulate the clock line

This function is used for the second shift register as pins allocations are different
on each shift register
*/
void Interface_SetRegister2(uint8_t bits)
{
	int i=0;
	uint8_t mask = 0b10000000;
	for(; i<8; i++)
	{
		if(mask & bits)
			GPIO_SetBits(Shifter2_Port, Shifter2_DS);
		else
			GPIO_ResetBits(Shifter2_Port, Shifter2_DS);

		mask = mask>>1;

		GPIO_SetBits(Shifter2_Port, Shifter2_CLK);
		GPIO_ResetBits(Shifter2_Port, Shifter2_CLK);
	}
}

/**
Write to the LEDs by writing to the shift register
Instead of using an SPI peripheral, a GPIO pin is manually toggled to
simulate the clock line

This function is used for the second shift register as pins allocations are different
on each shift register
*/
void Interface_SetRegister3(uint8_t bits)
{
	int i=0;
	uint8_t mask = 0b10000000;
	for(; i<8; i++)
	{
		if(mask & bits)
			GPIO_SetBits(Shifter3_Port, Shifter3_DS);
		else
			GPIO_ResetBits(Shifter3_Port, Shifter3_DS);

		mask = mask>>1;

		GPIO_SetBits(Shifter3_Port, Shifter3_CLK);
		GPIO_ResetBits(Shifter3_Port, Shifter3_CLK);
	}
}

/**
Write to the 4x4 LED matrix that controls sound playback
*/
void Interface_SetLEDMatrix(uint16_t bits)
{
	Interface_SetRegister2(bits & 0x00FF);
	Interface_SetRegister3(bits>>8);
}

/**
Determine if any button from 4x4 playback grid has been pressed
*/
int Interface_ButtonPadPressed()
{
	if(GPIO_ReadInputDataBit(Encoder1_Port, Encoder1_EN) == 1 | GPIO_ReadInputDataBit(Encoder2_Port, Encoder2_EN))
	{
		return 1;
	}
	return 0;
}

/**
Read an effect button
*/
uint8_t Interface_ReadFunctionButton()
{
	uint8_t button_pressed = 0;
	if(GPIO_ReadInputDataBit(Encoder3_Port, Encoder3_Q0) == 1)
	{
		button_pressed = button_pressed | 0b001;
	}
	if(GPIO_ReadInputDataBit(Encoder3_Port, Encoder3_Q1) == 1)
	{
		button_pressed = button_pressed | 0b010;
	}
	if(GPIO_ReadInputDataBit(Encoder3_Port, Encoder3_Q2) == 1)
	{
		button_pressed = button_pressed | 0b100;
	}

	return button_pressed;
}

/**
Determine which button from 4x4 playback grid has been pressed
*/
uint8_t Interface_ReadButtonPad()
{
	uint8_t button_pressed = 0;

	if (GPIO_ReadInputDataBit(Encoder2_Port, Encoder2_EN) == 1)
		button_pressed = button_pressed | 0b1000;

	if (GPIO_ReadInputDataBit(Encoder2_Port, Encoder2_Q0) == 1 || GPIO_ReadInputDataBit(Encoder1_Port, Encoder1_Q0) == 1)
	{
		button_pressed = button_pressed | 0b001;
	}
	if (GPIO_ReadInputDataBit(Encoder2_Port, Encoder2_Q1) == 1 || GPIO_ReadInputDataBit(Encoder1_Port, Encoder1_Q1) == 1)
	{
		button_pressed = button_pressed | 0b010;
	}
	if (GPIO_ReadInputDataBit(Encoder2_Port, Encoder2_Q2) == 1 || GPIO_ReadInputDataBit(Encoder1_Port, Encoder1_Q2) == 1)
	{
		button_pressed = button_pressed | 0b100;
	}

	return button_pressed;
}
