/*
 * ui.c
 *
 *  Created on: Dec 18, 2018
 *      Author: jeffsnyder
 */
#include "main.h"
#include "ui.h"
#include "audiostream.h"
#include "hrtim.h"

uint8_t buttonValues[NUM_BUTTONS];
uint8_t buttonValuesPrev[NUM_BUTTONS];
uint32_t buttonCounters[NUM_BUTTONS];
uint8_t buttonPressed[NUM_BUTTONS];
uint8_t buttonReleased[NUM_BUTTONS];

uint8_t colorsForPresets[6][3] = {{120, 120, 120},{50,150, 20}, {200,0,20}, {0,70,120}, {150, 150, 0}, {100, 0, 100}};
GeneraPreset currentPreset = 0;
GeneraPreset previousPreset = PresetNil;
uint8_t loadingPreset = 0;

uint8_t samplerRecording;



//these are some scaling arrays to try to match the brightness levels of the RGB leds
uint16_t redScaled[17] = {2000, 6550, 6660, 6750, 6840, 7000, 7080, 7170, 7250, 7350, 7550, 7900, 8200, 8500, 9000, 9200, 9800};
//uint16_t greenScaled[17] = {0, 8, 9, 10, 11, 13, 16, 22, 30, 40, 60, 110, 180, 300, 400, 700, 900};
uint16_t greenScaled[17] = {210, 580, 600, 610, 660, 700, 740, 790, 850, 950, 1010, 1150, 1230, 1350, 1460, 1500, 2000};
uint16_t blueScaled[17] = {210, 580, 600, 610, 700, 740, 790, 820, 850, 950, 1010, 1150, 1230, 1320, 1420, 1500, 1800};


void RGB_LED_setColor(uint8_t Red, uint8_t Green, uint8_t Blue) //inputs between 0-255
{
	float floatyPoint;
	uint8_t intPart;
	float fractPart;
	float endValue;

	floatyPoint = ((float)Red) * 0.0625f;
	intPart = (uint8_t)floatyPoint;
	fractPart = floatyPoint - ((float)intPart);
	endValue = (redScaled[intPart] * 1.2f * (1.0f - fractPart)) + (redScaled[intPart + 1] * 1.2f* (fractPart));

	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP2xR = (uint16_t)endValue;

	floatyPoint = ((float)Green) * 0.0625f;
	intPart = (uint8_t)floatyPoint;
	fractPart = floatyPoint - ((float)intPart);
	endValue = (greenScaled[intPart] * (1.0f - fractPart)) + (greenScaled[intPart + 1] * (fractPart));

	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP2xR =(uint16_t)endValue;

	floatyPoint = ((float)Blue) * 0.0625f;
	intPart = (uint8_t)floatyPoint;
	fractPart = floatyPoint - ((float)intPart);
	endValue = (blueScaled[intPart] * (1.0f - fractPart)) + (blueScaled[intPart + 1] * (fractPart));

	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR =(uint16_t)endValue;

}



void setLED_B(uint8_t brightness) //inputs between 0-255
{
	if (brightness > 5)
	{
		HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP2xR =((uint16_t)brightness) << 6;
	}
	else
	{
		HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP2xR = 200;
	}
}


void setLED_C(uint8_t brightness)//inputs between 0-255
{
	if (brightness > 5)
	{
		HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR =((uint16_t)brightness) << 6;
	}
	else
	{
		HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = 200;
	}
}


void setLED_D(uint8_t brightness)//inputs between 0-255
{
	if (brightness > 5)
	{
		HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP2xR =((uint16_t)brightness) << 6;
	}
	else
	{
		HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP2xR = 200;
	}
}



uint8_t buttonState[10];

void buttonCheck(void)
{
	if (codecReady)
	{
		buttonValues[0] = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);//A
		buttonValues[1] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6); //B
		buttonValues[2] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7); //C
		buttonValues[3] = !HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6); //D


		for (int i = 0; i < NUM_BUTTONS; i++)
		{

			if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] < 1))
			{
				buttonCounters[i]++;
			}
			else if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] >= 1))
			{
				if (buttonValues[i] == 1)
				{
					buttonPressed[i] = 1;
				}
				else if (buttonValues[i] == 0)
				{
					buttonReleased[i] = 1;
				}
				buttonValuesPrev[i] = buttonValues[i];
				buttonCounters[i] = 0;
			}
		}

		// make some if statements if you want to find the "attack" of the buttons (getting the "press" action)
		// we'll need if statements for each button  - maybe should go to functions that are dedicated to each button?

		// A button changes presets
		if (buttonPressed[0] == 1)
		{

			previousPreset = currentPreset;
			if (currentPreset >= PresetNil - 1) currentPreset = 0;
			else currentPreset++;
			loadingPreset = 1;
			//writeCurrentPresetToFlash();
			displayColorsForCurrentPreset();
			buttonPressed[0] = 0;

		}

		// B button
		if (buttonPressed[1] == 1)
		{
			distortionMode = !distortionMode;
			setLED_B(distortionMode * 255);
			buttonPressed[1] = 0;
		}

		// C button
		if (buttonPressed[2] == 1)
		{

		}

		// D button
		if (buttonPressed[2] == 1)
		{

		}


	}
}

void displayColorsForCurrentPreset(void)
{
	RGB_LED_setColor(colorsForPresets[currentPreset][0], colorsForPresets[currentPreset][1], colorsForPresets[currentPreset][2]);
}


