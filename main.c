#include "stm32f4xx.h"
#include "stm32f4_discovery_audio_codec/stm32f4_discovery_audio_codec.h"

#include "stm32f4_discovery.h"
#include "STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h"
#include "STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h"
#include <math.h>

#include "sounds/snare.h"
#include "sounds/fxTom.h"
#include "sounds/hihat.h"
#include "sounds/kich.h"
#include "equaliser.h"

#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/semphr.h"

#include "main.h"
#include "effects.h"

#include "LCD.h"
#include "interface.h"

int16_t playbackBuffer [32768];

uint64_t u64IdleTicksCnt=0; // Counts when the OS has no task to execute.
uint64_t tickTime=0;        // Counts OS ticks (default = 1000Hz).

xSemaphoreHandle xSemaphore = NULL;
xSemaphoreHandle xSemLCD = NULL;

xTaskHandle xModifyBuffer = NULL;

void GPIO_A0_Init(void);
void potInit(void);

void vPlaybackTask( void * pvparameters);
void vModifyBuffer (void * pvparameters);
void vButtonTask (void * pvparameters);
void vPulse_EnableTask (void * pvparameters);
void vPotTask (void * pvparameters);

// 8 Buttons' states - each bit indicate respective button's ON/OFF state
xSemaphoreHandle xButtonInterrupts = NULL;
uint16_t buttonState = 0;

//Freestyle mode task/variables
xTaskHandle xFreestyleLED = NULL;
xTaskHandle xFreestyleRead = NULL;
void vFlashFreestyleLEDTask(void * pvparameters);

// Potentiometer reading variables
uint16_t ADC3ConvertedValue[5] = {0,0,0,0,0};
uint16_t ADC3ConvertedValuePrev[5] = {65535,65535,65535,65535,65535};
static const uint8_t POT_THRESHOLD = 100; //the amount by which the adc must vary before
                                          //the pot gets recognised as being turned
static const uint16_t EFF_TURNOFF = 500; //when the adc is below this amount, the
                                         //currently selected effect is turned off

// Effects variables
uint8_t selected_eff;
uint8_t active_eff[9] = {0,0,0,0,0,0,0,0};
uint16_t eff_params[9] = {2000, 5000 ,5000 ,0,0,2000,3000};//initial vals (gets over ridden on potTask's first call)
uint16_t eff_minval[9] = {1000, 10   ,10   ,0,0,1000,1000};
uint16_t eff_maxval[9] = {6000, 10000,10000,0,0,5000,7000};
                       // ECHO   LPF   HPF      VIB  REVRB

// Equalizer variables
EQSTATE eq;
float eq_minval[3] = {0.0,0.0,0.0};//lg,mg,hg
float eq_maxval[3] = {1.0,1.0,1.0};//lg,mg,hg

// Tempo variables
static const uint16_t MIN_PB_DELAY = 100; //minimum playback task delay (>=1)
static const uint16_t MAX_PB_DELAY = 1000; //maximum playback task delay (<2^32)
uint16_t playback_delay = 500; //interval delay in ms
uint8_t beats = 3;

// Indicates whether a sound is playing (1) or not (0)
uint8_t status = 0;

/*
Useful for debugging
*/
void HardFault_Handler (void) {
	STM_EVAL_LEDOn(LED3);
	STM_EVAL_LEDOn(LED4);
	STM_EVAL_LEDOn(LED5);
	STM_EVAL_LEDOn(LED6);
}


int main(void)
{
	SystemInit();
	uint8_t effectState = 0;
	
	// Initialise low pass filter and equaliser
	LPfilter_reset();
	init_3band_state(&eq, 880, 5000, 44100);
	eq.lg = 1.2;  
	eq.mg = 0;    
	eq.hg = 0;    

	init_GPIO();
	init_LCD_Screen();
	delay_ms(100);
	LCD_CommandSerial(CLEAR);
	write_to_screen_string("  Beat Dropper       5000 ");
	delay_ms(500);

	selected_eff = NO_EFFECT;
	potInit(); // initialises pot pins, adc3 and dma

	Interface_Init();
	Interface_SetLEDMatrix(0xA5A5);
	Interface_SetFunctionLED(0xFF);

	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
    STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);

    vSemaphoreCreateBinary( xSemaphore );
    vSemaphoreCreateBinary( xSemLCD );
    vSemaphoreCreateBinary(xButtonInterrupts);

	uint16_t
	i = 0;
	for (; i < 32768; ++i) {
		if (i < SOUNDSIZE) {
			playbackBuffer [i] = snare[i];
		}
		else {
			playbackBuffer [i] = 0;
		}
	}

	EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);
	EVAL_AUDIO_Init(OUTPUT_DEVICE_HEADPHONE, 100, 22050); // Halve it since its mono

	xTaskCreate( vPlaybackTask, ( signed char * ) "Play Task", 100, NULL, 1, NULL );
	xTaskCreate( vModifyBuffer, ( signed char * ) "Modify Task", 150, NULL, 1, &xModifyBuffer );
	xTaskCreate( vButtonTask, ( signed char * ) "Button Task", 130, NULL, 1, NULL );
	xTaskCreate( vPotTask, ( signed char * ) "Potentiometer Task", 100, NULL, 1, NULL );
	vTaskSuspend(xModifyBuffer );
    vTaskStartScheduler(); // This should never return.

    while(1)
    {
    }

    return 1;
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void)
{
	return 1;
}

/*
 * Called when buffer has been played out
 * Releases semaphore and wakes up task which modifies the buffer
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size)
{
	vTaskResume( xModifyBuffer);
	xSemaphoreGive(xSemaphore);
	status = 0;
}

void EVAL_AUDIO_HalfTransfer_CallBack(uint32_t pBuffer, uint32_t Size)
{
}

void EVAL_AUDIO_Error_CallBack(void* pData)
{
	while(1);
}

uint32_t Codec_TIMEOUT_UserCallback(void)
{
	while(1);
	return 1;
}

/*
 * Plays sound in a loop
 * "delay" determine the tempo - how often it plays
 * A semaphore is used for synchronisation so that you can only play when the previous sound has completed
 * The "playbackComplete" callback has the other end of the semaphore
 */
void vPlaybackTask( void *pvparameters )
{
	for (;;){
		while( xSemaphoreTake( xSemaphore, (portTickType) 254 ) == pdFALSE) {} ;

		EVAL_AUDIO_Play((uint16_t*)(playbackBuffer), 16384);
		status = 1;
		vTaskDelay(playback_delay / portTICK_RATE_MS);
	}
}

/*
 * Loads sound into the playback buffer
 * First loads sound into the array
 * Then applies effects to it if they are selected
 *
 * Display active effect on LCD screen after modifying playback buffer
 */
void vModifyBuffer (void * pvparameters){

	const uint16_t * sourceSound;
	uint16_t endNumber;
	uint16_t i = 0;
	uint8_t number = 1;
	float ringFreq = 1000.0;
	uint8_t effectState = 0;

	uint8_t k;
	for(k=0; k<8; ++k)
	{
		active_eff[i] = 0;
	}
	Interface_SetFunctionLED(effectState);

	while (1) {

		switch (number) {
			case (0):
					Interface_SetLEDMatrix(buttonState & 0x000F);

					for (i=0; i < 32768; ++i)
					{
						playbackBuffer [i] = 0;
						if(buttonState & 0x0001 && i<SOUNDSIZE)
							playbackBuffer [i] += (int16_t) snare[i]/4;

						if(buttonState & 0x0002 && i < SOUNDSIZE2)
							playbackBuffer [i] += (int16_t) fxTom[i]/4;

						if(buttonState & 0x0004 && i < SOUNDSIZE3)
							playbackBuffer [i] += (int16_t) kick[i]/4;

						if(buttonState & 0x0008 && i < SOUNDSIZE4)
							playbackBuffer [i] += (int16_t) hihat[i]/4;

						if (i < 100 && i > 0){
							playbackBuffer [i] = playbackBuffer[i] / (100 -i);
						}
					}

					STM_EVAL_LEDOn(LED3);
					STM_EVAL_LEDOff(LED4);
					STM_EVAL_LEDOff(LED5);
					STM_EVAL_LEDOff(LED6);
					break;
			case (1):
					Interface_SetLEDMatrix(buttonState & 0x00F0);

					for (i=0; i < 32768; ++i)
					{
						playbackBuffer [i] = 0;
						if(buttonState & 0x0010 && i<SOUNDSIZE)
							playbackBuffer [i] += (int16_t) snare[i]/4;

						if(buttonState & 0x0020 && i < SOUNDSIZE2)
							playbackBuffer [i] += (int16_t) fxTom[i]/4;

						if(buttonState & 0x0040 && i < SOUNDSIZE3)
							playbackBuffer [i] += (int16_t) kick[i]/4;

						if(buttonState & 0x0080 && i < SOUNDSIZE4)
							playbackBuffer [i] += (int16_t) hihat[i]/4;

						if (i < 100 && i > 0){
							playbackBuffer [i] = playbackBuffer[i] / (100 -i);
						}
					}


					STM_EVAL_LEDOff(LED3);
					STM_EVAL_LEDOn(LED4);
					STM_EVAL_LEDOff(LED5);
					STM_EVAL_LEDOff(LED6);
					break;

			case (2):
					Interface_SetLEDMatrix(buttonState & 0x0F00);

					for (i=0; i < 32768; ++i)
					{
						playbackBuffer [i] = 0;
						if(buttonState & 0x0100 && i<SOUNDSIZE)
							playbackBuffer [i] += (int16_t) snare[i]/4;

						if(buttonState & 0x0200 && i < SOUNDSIZE2)
							playbackBuffer [i] += (int16_t) fxTom[i]/4;

						if(buttonState & 0x0400 && i < SOUNDSIZE3)
							playbackBuffer [i] += (int16_t) kick[i]/4;

						if(buttonState & 0x0800 && i < SOUNDSIZE4)
							playbackBuffer [i] += (int16_t) hihat[i]/4;

						if (i < 100 && i > 0){
							playbackBuffer [i] = playbackBuffer[i] / (100 -i);
						}
					}

					STM_EVAL_LEDOff(LED3);
					STM_EVAL_LEDOff(LED4);
					STM_EVAL_LEDOn(LED6);
					STM_EVAL_LEDOff(LED5);
					break;

			case (3):
					Interface_SetLEDMatrix(buttonState & 0xF000);

					for (i=0; i < 32768; ++i)
					{
						playbackBuffer [i] = 0;
						if(buttonState & 0x1000 && i<SOUNDSIZE)
							playbackBuffer [i] += (int16_t) snare[i]/4;

						if(buttonState & 0x2000 && i < SOUNDSIZE2)
							playbackBuffer [i] += (int16_t) fxTom[i]/4;

						if(buttonState & 0x4000 && i < SOUNDSIZE3)
							playbackBuffer [i] += (int16_t) kick[i]/4;

						if(buttonState & 0x8000 && i < SOUNDSIZE4)
							playbackBuffer [i] += (int16_t) hihat[i]/4;

						if (i < 100 && i > 0){
							playbackBuffer [i] = playbackBuffer[i] / (100 -i);
						}
					}
					STM_EVAL_LEDOff(LED3);
					STM_EVAL_LEDOff(LED4);
					STM_EVAL_LEDOff(LED6);
					STM_EVAL_LEDOn(LED5);
					break;
		} // end switch

		++number;
		if (number > beats){
			number = 0;
		}

		i = 0;
		
		//
		// Apply any active effects
		//

		if (active_eff[LPF]){
			LPfilter_computeCoeff (eff_params[LPF], 0.5f);
			lpf(playbackBuffer);
		}
		if (active_eff[HPF]){
			hpf(playbackBuffer, eff_params[HPF]);
		}
		if (active_eff[EQ3]){
			equaliser3(playbackBuffer, eq);
		}

		if (active_eff[ECHO]){
			echo(playbackBuffer, eff_params[ECHO]);
		}
		else if (active_eff[VIBRATO]){   // Can't have echo and vibrato
			vibrato(playbackBuffer, eff_params[VIBRATO]);
		}
		else if (active_eff[REVERB]){
			reverb (playbackBuffer, eff_params[REVERB]);
		}


		if (active_eff[RING]){
			ringFreq += 4000;
			if (ringFreq > 32000){
				ringFreq = 1000.0;
			}

			ringModulation(playbackBuffer, ringFreq);
		}
	
	
		//
		// Display active effect on LCD screen
		//
		
		LCD_CommandSerial(CLEAR);
		write_to_screen_string("bpm: ");
		print_double_to_screen( (60.0 / (playback_delay / 1000.0)) );

		if (selected_eff == ECHO && active_eff[ECHO]){
			LCD_CommandSerial(LINE_TWO);
			write_to_screen_string("echo: ");
			print_double_to_screen( eff_params[ECHO] );
		}
		if (selected_eff == LPF && active_eff[LPF]){
			LCD_CommandSerial(LINE_TWO);
			write_to_screen_string("Low pass: ");
			print_double_to_screen( eff_params[LPF] );
		}
		if (selected_eff == HPF && active_eff[HPF] ){
			LCD_CommandSerial(LINE_TWO);
			write_to_screen_string("High pass: ");
			print_double_to_screen( eff_params[HPF] );
		}
		if (selected_eff == EQ3 && active_eff[EQ3]){
			LCD_CommandSerial(LINE_TWO);
			write_to_screen_string("L");
			print_pot_to_screen( (uint16_t) (eq.lg * 100) );
			write_to_screen_string(" M");
			print_pot_to_screen( (uint16_t) (eq.mg * 100) );
			write_to_screen_string(" H");
			print_pot_to_screen( (uint16_t) (eq.hg * 100) );
		}
		if (selected_eff == VIBRATO && active_eff[VIBRATO]){
			LCD_CommandSerial(LINE_TWO);
			write_to_screen_string("Vibrato: ");
			print_double_to_screen( eff_params[VIBRATO] );
		}
		if (selected_eff == REVERB && active_eff[REVERB] ){
			LCD_CommandSerial(LINE_TWO);
			write_to_screen_string("Reverb: ");
			print_double_to_screen( eff_params[REVERB] );
		}

		vTaskSuspend( xModifyBuffer );
	}
}

/**
 * Debounces button in the while loop
 * Button only counts as pressed once it is released
 *
 * This is task to detect button matrix only.
 */
void vButtonTask( void *pvparameters )
{
	uint8_t freestyle = 0;
	uint8_t effectState = 0;
	uint8_t wasPressed = 0;
	uint8_t buttonValue = 0;
	int i = 0;

	for (;;)
	{
		while (Interface_ButtonPadPressed()) {
			if(!wasPressed)
				buttonValue = Interface_ReadButtonPad();
			wasPressed = 1;
		}
		while (GPIO_ReadInputDataBit(Encoder3_Port, Encoder3_EN) == 1) {
			if(!wasPressed)
			{
				selected_eff = Interface_ReadFunctionButton();
				wasPressed = 2;
			}
		}

		while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11) == 1)
		{
			wasPressed = 3;
		}

		if(xSemaphoreTake(xButtonInterrupts, 30)== pdTRUE)
		{
			if (wasPressed == 1) {
				buttonState = buttonState^(1<<buttonValue);	// Toggle the "button value"th bit of buttonState using XOR
				Interface_SetLEDMatrix(buttonState);        // Uncommenting this would set LEDs to indicate the buttonState, ignoring times

				wasPressed = 0;
			}

			if (wasPressed == 2) {
				// This should cover toggling all the below functions
				active_eff[selected_eff] = active_eff[selected_eff] ^ 1;

				// Indicate which functions are 'on'
				effectState = 0;
				for(i=0; i<8; i++)
				{
					if(active_eff[i] != 0)
						effectState = effectState | (1<<i);
				}
				Interface_SetFunctionLED(effectState);

				if(selected_eff == 7)
				{
					if(active_eff[7] == 1 && freestyle == 0)
					{
						freestyle = 1;
						xTaskCreate( vFlashFreestyleLEDTask, (signed char * ) "Freestyle", 80, NULL, 2, &xFreestyleLED );
					}
					else if(freestyle == 1)
					{
						vTaskSuspend(xFreestyleLED);
						vTaskDelete(xFreestyleLED);
						Interface_SetFunctionLED(effectState | 0x80);
						active_eff[7] = 1;
						freestyle = 0;
					}
				}

		    	wasPressed = 0;
			}

			if(wasPressed == 3)
			{
				++beats;
				if (beats > 3){
				beats = 0;
				}
			wasPressed = 0;
			}
			xSemaphoreGive(xButtonInterrupts);

		}

		vTaskDelay(30 / portTICK_RATE_MS);
	}
}

/*
 * Light up LEDs for Freestyle mode
 */
void vFlashFreestyleLEDTask(void *pvparameters){
	uint8_t toggler = 0;
	uint8_t effectState = 0;
	int i = 0;

	for(;;)
	{
		effectState = 0;
		for(i=0; i<8; i++)
		{
			if(active_eff[i] != 0)
				effectState = effectState | (1<<i);
		}
		Interface_SetFunctionLED(effectState ^ toggler);
		toggler = toggler^0x80;
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

/*
 * Checks pot values and makes changes accordingly
 */
void vPotTask( void *pvparameters )
{
	for (;;)
	{
		int16_t pot_diff;

		// Tempo control
		pot_diff = ADC3ConvertedValuePrev[0] - ADC3ConvertedValue[0];
		if (pot_diff > POT_THRESHOLD || pot_diff < -POT_THRESHOLD) //pot changes slightly by itself
		{
			ADC3ConvertedValuePrev[0] = ADC3ConvertedValue[0];
			playback_delay = ((float)ADC3ConvertedValuePrev[0]/4095)*(MAX_PB_DELAY-MIN_PB_DELAY)+MIN_PB_DELAY;
		}

		// Effect param control
		if (selected_eff!=NO_EFFECT || selected_eff!=RING)
		{
			pot_diff = ADC3ConvertedValuePrev[1] - ADC3ConvertedValue[1];
			if (pot_diff > POT_THRESHOLD || pot_diff < -POT_THRESHOLD) //pot changes slightly by itself
			{
				ADC3ConvertedValuePrev[1] = ADC3ConvertedValue[1];

				if (ADC3ConvertedValue[1] > EFF_TURNOFF)
				{
					eff_params[selected_eff] = ((float)(ADC3ConvertedValuePrev[1]-EFF_TURNOFF)/(4095-EFF_TURNOFF))
							*(eff_maxval[selected_eff]-eff_minval[selected_eff])+eff_minval[selected_eff];
					active_eff[selected_eff] = 1;
				}
				else
					active_eff[selected_eff] = 0;
			}
		}

		// Equalizer param control
		if (active_eff[EQ3])
		{
			// Equalizer low gain changed?
			pot_diff = ADC3ConvertedValuePrev[2] - ADC3ConvertedValue[2];
			if (pot_diff > POT_THRESHOLD || pot_diff < -POT_THRESHOLD)
			{
				ADC3ConvertedValuePrev[2] = ADC3ConvertedValue[2];
				eq.lg = ((float)ADC3ConvertedValuePrev[2]/4095)*(eq_maxval[0]-eq_minval[0])+eq_minval[0];
			}

			// Equalizer mid gain changed?
			pot_diff = ADC3ConvertedValuePrev[3] - ADC3ConvertedValue[3];
			if (pot_diff > POT_THRESHOLD || pot_diff < -POT_THRESHOLD)
			{
				ADC3ConvertedValuePrev[3] = ADC3ConvertedValue[3];
				eq.mg = ((float)ADC3ConvertedValuePrev[3]/4095)*(eq_maxval[1]-eq_minval[1])+eq_minval[1];
			}

			// Equalizer high gain changed?
			pot_diff = ADC3ConvertedValuePrev[4] - ADC3ConvertedValue[4];
			if (pot_diff > POT_THRESHOLD || pot_diff < -POT_THRESHOLD)
			{
				ADC3ConvertedValuePrev[4] = ADC3ConvertedValue[4];
				eq.hg = ((float)ADC3ConvertedValuePrev[4]/4095)*(eq_maxval[2]-eq_minval[2])+eq_minval[2];
			}
		}

		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

/*
 * Initialises GPIO, ADC and DMA for the potentiometers
 */
void potInit (void)
{
	ADC_InitTypeDef       ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	DMA_InitTypeDef       DMA_InitStruct;
	GPIO_InitTypeDef      GPIO_InitStruct;

	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);//ADC3 is connected to the APB2 peripheral bus

	/* DMA2 Stream0 channel0 configuration **************************************/
	DMA_InitStruct.DMA_Channel = DMA_Channel_2;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;//ADC3's data register
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = 5;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//Reads 16 bit values
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//Stores 16 bit values
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStruct);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/* Configure GPIO pins ******************************************************/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;// PC0, PC1, PC2, PC3
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//The pins are configured in analog mode
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;//We don't need any pull up or pull down
	GPIO_Init(GPIOC, &GPIO_InitStruct);//Initialize GPIOC pins with the configuration
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;//PA1
	GPIO_Init(GPIOA, &GPIO_InitStruct);//Initialize GPIOA pins with the configuration

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);

	/* ADC3 Init ****************************************************************/
	ADC_DeInit();
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit number giving a maximum value of 4095
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;//The scan is configured in multiple channels
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;//The conversion is continuous, the input data is converted more than once
	ADC_InitStruct.ADC_ExternalTrigConv = 0;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//Data converted will be shifted to right
	ADC_InitStruct.ADC_NbrOfConversion = 5;
	ADC_Init(ADC3, &ADC_InitStruct);//Initialize ADC with the configuration

	/* Select the channels to be read from **************************************/
	ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_144Cycles);//PC0
	ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 2, ADC_SampleTime_144Cycles);//PC1
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_144Cycles);//PC2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_144Cycles);//PC3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, ADC_SampleTime_144Cycles);//PA1

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

	/* Enable ADC3 DMA */
	ADC_DMACmd(ADC3, ENABLE);

	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);

	/* Start ADC3 Software Conversion */
	ADC_SoftwareStartConv(ADC3);
}

/*
 * This FreeRTOS callback function gets called once per tick (default = 1000Hz)
 */
void vApplicationTickHook( void ) {
    ++tickTime;
}

/*
 * Continually send "silence" to the speaker when not playing
 */
void vApplicationIdleHook( void ) {

	uint8_t i = 0;
	uint16_t k = 0xFFFF;

    ++u64IdleTicksCnt;
    if (status == 0){

    	if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE)){
    		SPI_I2S_SendData(CODEC_I2S, i);
    			if (i > 16){
    				i = 0;
    			}
    	}

    }
}

/*
 * A required FreeRTOS function.
 */
void vApplicationMallocFailedHook( void ) {
    configASSERT( 0 );  // Latch on any failure / error.
}
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName)
{

}
