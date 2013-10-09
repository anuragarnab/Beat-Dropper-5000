#ifndef interface
#define interface

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"

#define Encoder1_Port	GPIOD
#define Encoder1_Q0		GPIO_Pin_3
#define Encoder1_Q1		GPIO_Pin_7
#define Encoder1_Q2		GPIO_Pin_5
#define Encoder1_EN		GPIO_Pin_6

#define Encoder2_Port	GPIOB
#define Encoder2_Q0		GPIO_Pin_3
#define Encoder2_Q1		GPIO_Pin_4
#define Encoder2_Q2		GPIO_Pin_5
#define Encoder2_EN		GPIO_Pin_7

#define Encoder3_Port	GPIOC
#define Encoder3_Q0		GPIO_Pin_6
#define Encoder3_Q1		GPIO_Pin_8
#define Encoder3_Q2		GPIO_Pin_9
#define Encoder3_EN		GPIO_Pin_11

// Swapped shifter 1 and 2 around
#define Shifter1_Port 	GPIOC
#define Shifter1_DS		GPIO_Pin_14
#define Shifter1_CLK	GPIO_Pin_15

#define Shifter3_Port 	GPIOE
#define Shifter3_DS		GPIO_Pin_2
#define Shifter3_CLK	GPIO_Pin_4


// Swapping shifter 1 and 2 around
#define Shifter2_Port 	GPIOD
#define Shifter2_DS		GPIO_Pin_0
#define Shifter2_CLK	GPIO_Pin_2

void Interface_Init();
void Interface_SetFunctionLED(uint8_t bits);
void Interface_SetRegister2(uint8_t bits);
void Interface_SetRegister3(uint8_t bits);
void Interface_SetLEDMatrix(uint16_t bits);

uint8_t Interface_ReadFunctionButton();
int Interface_ButtonPadPressed();
uint8_t Interface_ReadButtonPad();

#endif
