#ifndef LCD
#define LCD


#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>-
#include "string.h"


#define    POWER_UP        0x33
#define    FOURBIT_MODE    0X32

#define    EIGHTBIT_MODE   0x38 // as well as two line mode
#define    DISPLAY	       0xF

#define    TWOLINE_MODE    0x28
#define    SETUP_CURSOR    0x0C
#define    SETUP_CURSOR_BLINKING    0xF

#define    CLEAR           0x01
#define    CURSOR_HOME     0x02
#define    LINE_TWO        0xC0

void init_GPIO(void);
void delay_ms(int milli);
void pulse_enable(void);
void LCD_Command(unsigned char command);
void LCD_CommandSerial(unsigned char command);
void init_LCD_Screen(void);
void write_to_screen_characters(unsigned char character);
void write_to_screen_string(unsigned char *instring);
void print_double_to_screen(uint16_t data_in);
void print_pot_to_screen(uint16_t data_in);


#endif
