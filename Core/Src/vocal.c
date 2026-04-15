/*
 * vocal.c
 *
 *  Created on: Dec 27, 2023
 *      Author: jeffsnyder
 */


#include "main.h"
#include "leaf.h"
#include "codec.h"
#include "i2c.h"
#include "gpio.h"
#include "spi.h"
#include "parameters.h"
#include "audiostream.h"
#include "vocal.h"

tVoc vocal[NUM_STRINGS_PER_BOARD];
int prevTractLength[NUM_STRINGS_PER_BOARD] = {22, 22};
int32_t prevActualTractLength[NUM_STRINGS_PER_BOARD] = {22, 22};
float vocalDefaults[12] = {0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.3019f, 0.1764f, 0.7764f, 0.8155f};
void __ATTR_ITCMRAM audioInitVocal()
{
	for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
	{
		tVoc_init(&vocal[v], 22, 65, &leaf);
	}
}

void __ATTR_ITCMRAM audioFreeVocal()
{
	;
}

void __ATTR_ITCMRAM audioSwitchToVocal()
{
	for (int i = 0; i < 12; i++)
	{
		tExpSmooth_setFactor(knobSmoothers[i], 0.01f);

		if (voice == 60)
		{
			tExpSmooth_setValAndDest(knobSmoothers[i], vocalDefaults[i]);
		}
		else
		{
			tExpSmooth_setValAndDest(knobSmoothers[i], loadedKnobParams[i]);
		}

		tExpSmooth_setValAndDest(knobSmoothers[i], vocalDefaults[i]);
		knobFrozen[i] = 1;
	}
}
void __ATTR_ITCMRAM audioFrameVocal(uint16_t buffer_offset)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	uint32_t tempCountFrame = DWT->CYCCNT;
	int32_t current_sample = 0;

	if (resetStringInputs)
	{
		for (int i = 0; i < numStringsThisBoard; i++)
		{
			//note off
			tADSRT_clear(fenvelopes[i]);
			previousStringInputs[i] = 0;
		}
		resetStringInputs = 0;
		newPluck = 1;
	}

	if (newPluck)
	{
		for (int i = 0; i < numStringsThisBoard; i++)
		{
			if ((previousStringInputs[i] == 0) && (stringInputs[i] > 0))
			{
				float amplitz = stringInputs[i] * 0.000015259021897f;

				stringOctave[i] = octave;
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

				//tADSRT_on(&fenvelopes[i], amplitz);
				stringFrequencies[i] = mtof(stringMIDIPitches[i]+ stringOctave[i]);

				tADSRT_setSustain(fenvelopes[i], 1.0f);// * randomFactors[currentRandom]);

				tADSRT_on(fenvelopes[i], amplitz);

			}
			else if ((previousStringInputs[i] > 0) && (stringInputs[i] == 0))
			{
				//note off

				tADSRT_off(fenvelopes[i]);
			}
			previousStringInputs[i] = stringInputs[i];
		}
		newPluck = 0;
	}

	for (int i = 0; i < numStringsThisBoard; i++)
	{
		float doublecompute = knobScaled[1] > 0.5f;
		float oneMinusDoubleCompute = 1.0f-doublecompute;
		float newTractLength = (knobScaled[0] * 100.0f) * (1.50f * oneMinusDoubleCompute +1.0f) ;

		if ((newTractLength > (prevTractLength[i] + 10))|| (newTractLength < (prevTractLength[i] - 10)))
		{
			int32_t squishedTract = (newTractLength*0.168f) ;
			if ( squishedTract != prevActualTractLength[i])
			{
				tVoc_set_tractLength(vocal[i],squishedTract   + 2);
				prevActualTractLength[i] = squishedTract;
			}
			prevTractLength[i] = newTractLength;
		}

		tVoc_setDoubleComputeFlag(vocal[i], doublecompute);

		tVoc_setTurbulenceNoiseGain(vocal[i], knobScaled[4]);
		tVoc_setAspirationNoiseGain(vocal[i], knobScaled[5]);
		tVoc_setAspirationNoiseFilterFreq(vocal[i], knobScaled[6]);
		tVoc_setAspirationNoiseFilterQ(vocal[i], knobScaled[7]);
		tVoc_set_tongue_shape_and_touch(vocal[i], knobScaled[8], knobScaled[9],knobScaled[10],knobScaled[11]);
		tVoc_set_velum(vocal[i], (0.4f * knobScaled[3]) + 0.01f);
		tVoc_rescaleDiameter(vocal[i], (knobScaled[2] * 3.0f) + 0.0245f);
	}

		//mono operation, no need to compute right channel. Also for loop iterating by 2 instead of 1 to avoid if statement.
	for (int i = 0; i < HALF_BUFFER_SIZE; i+=2)
	{
		current_sample = (int32_t)(audioTickVocal() * TWO_TO_23);
		audioOutBuffer[buffer_offset + i] = current_sample;
		audioOutBuffer[buffer_offset + i + 1] = current_sample;
	}
/*
	if (switchStrings)
	{
		switchStringModel(switchStrings);
	}
	switchStrings = 0;
	*/
	timeFrame = DWT->CYCCNT - tempCountFrame;
	frameLoadPercentage = (float)timeFrame * frameLoadMultiplier;
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}


float __ATTR_ITCMRAM audioTickVocal(void)
{
	float tempSamp = 0.0f;
	for (int i = 0; i < 12; i++)
	{
		knobScaled[i] = tExpSmooth_tick(knobSmoothers[i]);
	}

	float volumeSmoothed = tExpSmooth_tick(volumeSmoother);



	for (int i = 0; i < numStringsThisBoard; i++)
	{
		tempSamp += tVoc_tick(vocal[i]) * tADSRT_tickNoInterp(fenvelopes[i]);
		stringFrequencies[i] = mtofTableLookup(stringMIDIPitches[i]+ stringOctave[i]);
		tVoc_setFreq(vocal[i], stringFrequencies[i]);
		tVoc_set_tenseness(vocal[i], volumeSmoothed);

		//Lfloat tongue = 12.0f + (16.0f * knobScaled[0]);
		//tVoc_set_tongue_shape(&vocal[i], tongue, 2.9f * knobScaled[1] + 0.1f);

	}
	//float outVol = 0.0265625f - (0.2467348f * volumeSmoothed) + (1.253049f * volumeSmoothed * volumeSmoothed);
	//float outVol = 0.008315613f + 0.3774075f*volumeSmoothed - 1.785774f*volumeSmoothed*volumeSmoothed + 4.218241f*volumeSmoothed*volumeSmoothed*volumeSmoothed - 0.8576009f*volumeSmoothed*volumeSmoothed*volumeSmoothed*volumeSmoothed - 0.9656285f*volumeSmoothed*volumeSmoothed*volumeSmoothed*volumeSmoothed*volumeSmoothed;
	float outVol = 0.006721744f + 0.4720157f*volumeSmoothed - 2.542849f*volumeSmoothed*volumeSmoothed + 6.332339f*volumeSmoothed*volumeSmoothed*volumeSmoothed - 3.271672f*volumeSmoothed*volumeSmoothed*volumeSmoothed*volumeSmoothed;

	tempSamp *= outVol;
	tempSamp *= masterVolFromBrain;
	return tanhf(tempSamp);
}
