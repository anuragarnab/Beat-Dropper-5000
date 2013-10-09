#include "LCD.h"

//********************************************************************************************
//* CONNECTIONS:                                                                             *
//*------------------------------------------------------------------------------------------*
//* LCD PINS   | NAME                      | CONNECTED TO STM32F4 Discovery Port E           *
//*------------------------------------------------------------------------------------------*
//* 1............VSS.......................GND                       *
//* 2............VDD.......................+5V                       *
//* 3............CONTRAST..................POT 5K                    * Potentiometer Pins: pin 1 to V+, wiper (2nd pin) to pin 3 of LCD, pin 3 to GND
//* 4............RS  - Register Select.....PE3                       *
//* 5............RW  - Read/Write..........GND                       *
//* 6............E   - Enable..............PE5                       *
//* 7............DB0  - Data line 0........GND                       *
//* 8............DB1  - Data line 1........GND                       *
//* 9............DB2  - Data line 2........GND                       *
//* 10...........DB3  - Data line 3........GND                       *
//* 11...........DB4  - Data line 4........PE10                      *
//* 12...........DB5  - Data line 5........PE11                      *
//* 13...........DB6  - Data line 6........PE12                      *
//* 14...........DB7  - Data line 7........PE13                      *
//* 15...........BACKLIGHT POSITIVE........VDD                       *
//* 16...........BACKLIGHT NEGATIVE........VSS                       *
//********************************************************************

uint16_t Register_Select = GPIO_Pin_3;
uint16_t Enable = GPIO_Pin_5;

uint16_t DB4 = GPIO_Pin_10;
uint16_t DB5 = GPIO_Pin_11;
uint16_t DB6 = GPIO_Pin_12;
uint16_t DB7 = GPIO_Pin_13;

void init_GPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStruct.GPIO_Pin = Register_Select |Enable| DB4 | DB5| DB6 | DB7 ; // we want to configure 6 pins (defined above)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// pins are output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// Sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOE, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits
}

void delay_ms(int milli)
{
	int delay = milli * 17612; // approximate loops per ms at 168 MHz, Debug config
	for(; delay != 0; delay--);
}

/*
 * pulse_enable() - a method to toggle the Enable (PE5) as data is written to the LCD on the high to low transition of the signal
 * Enable pin is initially low (0)
 */
void pulse_enable(void)
{
	delay_ms(1);
	GPIOE->ODR ^=  ( 1 << 5 ); // Sets enable high
	delay_ms(1);
	GPIOE->ODR ^=  ( 1 << 5 ); // Sets enable low
}

/*
 *
 */
void LCD_CommandSerial(unsigned char command)
{

	GPIO_ResetBits(GPIOE, Register_Select); // Register Select line low

	unsigned char upper_nibble = command & 0xF0 ; // Put upper nibble (DB7 to DB4 bits) on data lines
	GPIOE->ODR = upper_nibble << 6;
	pulse_enable();

	unsigned char lower_nibble = command & 0xF ; // Put lower nibble (DB3 to DB0 bits) on data lines
	GPIOE->ODR = lower_nibble << 10;
	pulse_enable();

}

/*
 * init_LCD_Screen - a method to initialize the LCD screen using the commands passed into LCD_Command
 * Note 4 bit mode is selected
 */
void init_LCD_Screen(void){

	delay_ms(100);

	LCD_CommandSerial (POWER_UP);                   // Power up initialization for the lcd 'final note recommendation'
	LCD_CommandSerial (FOURBIT_MODE);               // Set LCD into 4 bit mode
	LCD_CommandSerial (SETUP_CURSOR_BLINKING);      // Turn display on and set up cursor
	LCD_CommandSerial (TWOLINE_MODE);               // Set up 2 lines and character size
	LCD_CommandSerial (CLEAR);                      // Clear display
}

/*
 * write_to_screen_characters - strings are written to the LCD screen as characters, this method writes each character to the LCD screen
 */
void write_to_screen_characters(unsigned char character)
{
	unsigned char upper_nibble = character & 0xF0 ; // Put upper nibble (DB7 to DB4 bits) on data lines
	GPIOE->ODR = upper_nibble << 6;
	GPIO_SetBits(GPIOE, Register_Select); // Use this pin as Register Select, needs to be high

	pulse_enable();

	unsigned char lower_nibble = character & 0xF ; // Put lower nibble (DB3 to DB0 bits) on data lines
	GPIOE->ODR = lower_nibble << 10;
	GPIO_SetBits(GPIOE, Register_Select); // Use this pin as Register Select, needs to be high

	pulse_enable();
}

/*
* write_to_screen_strings - strings are passed to this method by the user
* The method checks if the string requires one or two lines and adjusts the cursor pointer accordling
* The string should have a limit of up to 32 ASCII characters
* For full character set consult LCD1.pdf in dropbox literature folder for LCD screen
* The method breaks up the string into characters and passes this to write_to_screen_characters
*/
void write_to_screen_string(unsigned char *instring)
{
	unsigned char count = 0;
	int length = 0;

	length = (unsigned) strlen(instring);
	char *line_2 = ' ' ;

	while (instring[count]) // Until the null terminator is reached
	{
		if (count > 15)
		{
			GPIO_ResetBits(GPIOE, Register_Select);
			/*****/
			LCD_CommandSerial(0xC0);
			for (count ; count < length; count++)
			{
				write_to_screen_characters(instring[count]);
			}
		}
		else
		{
			write_to_screen_characters(instring[count]); // Write each character to LCD
			count++;
		}

	}
}

/**
 * Prints numbers to the screen
 */
void print_double_to_screen(uint16_t data_in)
{
	unsigned char characters[4] = {0};

	int array_length = sizeof(characters)/sizeof(characters[0]);
	int base = 1000;

	unsigned char num = '\0';
	uint16_t temp = 0;


	int j;

	for (j = 0 ; j < array_length ; j++)
	{
		characters[j] = data_in/base + 48;
		temp = data_in/base;
		data_in = data_in - (temp * base);
		base = base/10;
	}

	int i;

	for(i = 0; i < array_length; ++i)
	{
		write_to_screen_characters(characters[i]);
	}
}

/**
 * Prints potentiometer readings to screen
 */
void print_pot_to_screen(uint16_t data_in)
{
	unsigned char characters[3] = {0};

	int array_length = sizeof(characters)/sizeof(characters[0]);
	int base = 1000;

	unsigned char num = '\0';
	uint16_t temp = 0;


	int j;

	for (j = 0 ; j < array_length ; j++)
	{
		characters[j] = data_in/base + 48;
		temp = data_in/base;
		data_in = data_in - (temp * base);
		base = base/10;
	}

	int i;

	for(i = 0; i < array_length; ++i)
	{
		write_to_screen_characters(characters[i]);
	}
}
