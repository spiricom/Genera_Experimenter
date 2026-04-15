/*
 * string1.c
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
#include "string1.h"
#include "string2.h"
#include "string4.h"
#include "synth.h"

tSimpleLivingString3 livStr[NUM_STRINGS_PER_BOARD];
tPickupNonLinearity pu[NUM_STRINGS_PER_BOARD];
tExpSmooth pitchSmootherS[NUM_STRINGS_PER_BOARD];
float string1Defaults[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.3019f, 0.1764f, 0.7764f, 0.8155f};
void __ATTR_ITCMRAM audioInitString1()
{
	if (whichStringModelLoaded != String1Loaded)
	{

		if (whichStringModelLoaded == String2Loaded)
		{
			audioFreeString2();
		}
		if (whichStringModelLoaded == String4Loaded)
		{
			audioFreeString4();
		}
		else if (whichStringModelLoaded == SynthLoaded)
		{
			audioFreeSynth();
		}

		for (int v = 0; v < numStringsThisBoard; v++)
		{

			tSimpleLivingString3_initToPool(&livStr[v], 4, 220.0f, 17000.0f,
														 0.99999f, 0.0f, 0.01f,
													 0.01f, 0, &mediumPool);
			tSimpleLivingString3_setTargetLev(livStr[v], 0.047059f);
			tSimpleLivingString3_setLevSmoothFactor(livStr[v], 0.0301913f);
			tSimpleLivingString3_setLevStrength(livStr[v], 0.0f);
			tSimpleLivingString3_setLevMode(livStr[v], 1);
			tPickupNonLinearity_init(&pu[v], &leaf);
			tExpSmooth_init(&pitchSmootherS[v], 64.0f, 0.6f, &leaf);
		}
		whichStringModelLoaded = String1Loaded;
	}

}


void __ATTR_ITCMRAM audioFreeString1()
{
	for (int v = 0; v < numStringsThisBoard; v++)
	{
		tExpSmooth_free(&pitchSmootherS[v]);
		tSimpleLivingString3_free(&livStr[v]);
		tPickupNonLinearity_free(&pu[v]);
	}
}

void __ATTR_ITCMRAM audioSwitchToString1()
{

	audioInitString1();
	//load string1 default params:
	for (int i = 0; i < 12; i++)
	{
		tExpSmooth_setFactor(knobSmoothers[i], 0.001f);

		if (voice == 63)
		{
			tExpSmooth_setValAndDest(knobSmoothers[i], string1Defaults[i]);
		}
		else
		{
			tExpSmooth_setValAndDest(knobSmoothers[i], loadedKnobParams[i]);
		}

		knobFrozen[i] = 1;
	}
	tVZFilter_setFreq(noiseFilt2, 3332.0f); //based on testing with knob values
	audioFrameFunction = audioFrameString1;
	presetReady = 1;
}



void __ATTR_ITCMRAM audioFrameString1(uint16_t buffer_offset)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	uint32_t tempCountFrame = DWT->CYCCNT;
	int32_t current_sample = 0;

	if (resetStringInputs)
	{
		for (int i = 0; i < numStringsThisBoard; i++)
		{
			//note off
			lsDecay[i] = 0;
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

				stringOctave[i] = octave;
				float note = stringMIDIPitches[i] + stringOctave[i];
				//sourceValues[MIDI_KEY_SOURCE_OFFSET][v] = (note[v] - midiKeySubtractor) * midiKeyDivisor;

				if (note < 0.0f)
				{
					note = 0.0f;
				}
				if (note > 127.0f)
				{
					note = 127.0f;
				}
				if (isnan(note))
				{
					note = 64.0f;
				}

				tExpSmooth_setValAndDest(pitchSmootherS[i], mtof(note));
				float finalFreq = tExpSmooth_tick(pitchSmootherS[i]);
				tSimpleLivingString3_setFreq(livStr[i], finalFreq);


				float amplitz = stringInputs[i] * 0.000015259021897f;

				//then it's the string synth
				//tSimpleLivingString3_setDecay(&livStr[i], 20.0f);
				tSimpleLivingString3_pluck(livStr[i], amplitz, LEAF_clip(0.0f, ((pluckPos * randomFactors[currentRandom]) * knobScaled[2]) + (pluckPos * (1.0f - knobScaled[2])),1.0f));
				currentRandom++;
				lsDecay[i] = 1;

			}
			else if ((previousStringInputs[i] > 0) && (stringInputs[i] == 0))
			{
				//note off
				//tSimpleLivingString3_setDecay(&livStr[i], 0.1f);
				lsDecay[i] = 0;
			}
			previousStringInputs[i] = stringInputs[i];
		}
		newPluck = 0;
	}
	//mono operation, no need to compute right channel. Also for loop iterating by 2 instead of 1 to avoid if statement.
	for (int i = 0; i < HALF_BUFFER_SIZE; i+=2)
	{
		int iplusbuffer = buffer_offset + i;
		current_sample = (int32_t)(audioTickString1() * TWO_TO_23);
		current_sample = LEAF_clip((-1 * TWO_TO_23) + 1, current_sample, TWO_TO_23 - 1);

		audioOutBuffer[buffer_offset + i] = current_sample;
		audioOutBuffer[buffer_offset + i + 1] = current_sample * -1;
	}

	/*
	if (switchStrings)
	{
		switchStringModel(switchStrings);
	}
	switchStrings = 0;
	*/
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	timeFrame = DWT->CYCCNT - tempCountFrame;
	frameLoadPercentage = (float)timeFrame * frameLoadMultiplier;

}



float __ATTR_ITCMRAM audioTickString1(void)
{
	float temp = 0.0f;
	float note[numStringsThisBoard];

	float volumeSmoothed = tExpSmooth_tick(volumeSmoother);

	for (int i = 0; i < 12; i++)
	{
		knobScaled[i] = tExpSmooth_tick(knobSmoothers[i]);
	}
	pluckPos = knobScaled[9];

	for (int i = 0; i < numStringsThisBoard; i++)
	{
		note[i] = stringMIDIPitches[i] + stringOctave[i];
		//sourceValues[MIDI_KEY_SOURCE_OFFSET][v] = (note[v] - midiKeySubtractor) * midiKeyDivisor;

		if (note[i] < 0.0f)
		{
			note[i] = 0.0f;
		}
		if (note[i] > 127.0f)
		{
			note[i] = 127.0f;
		}
		if (isnan(note[i]))
		{
			note[i] = 64.0f;
		}

		//float finalFreq = mtof(note[i]);
		float dampFreq = 15778.3f;
		float decay = 0.1f;
		if (lsDecay[i])
		{
			decay = (knobScaled[10] * 800.0f) + 10.0f;
		}
		tSimpleLivingString3_setPickupPoint(livStr[i], knobScaled[8]);
		tSimpleLivingString3_setDecay(livStr[i], decay);
		tSimpleLivingString3_setDampFreq(livStr[i], dampFreq);
		tSimpleLivingString3_setLevStrength(livStr[i], knobScaled[0] * 0.0352872f);

		livStr[i]->rippleGain = knobScaled[5] * -0.03f;
		livStr[i]->invOnePlusr = 1.0f / (1.0f + livStr[i]->rippleGain);
		livStr[i]->rippleDelay = knobScaled[11];
		tExpSmooth_setDest(pitchSmootherS[i], mtof(note[i]));
		float finalFreq = tExpSmooth_tick(pitchSmootherS[i]);
		tSimpleLivingString3_setFreq(livStr[i], finalFreq);
		float barDelta = fabsf(barInMIDI[0]-prevBarInMIDI[0]);
		if (barDelta > 0.2f)
		{
			barDelta = 0.0f; //to avoid noise on open string glitches
		}
		tExpSmooth_setDest(barSlideSmoother[i], barDelta);
		barDelta = tExpSmooth_tick(barSlideSmoother[i]);
		barDelta = tEnvelopeFollower_tick(barNoiseSmoother[i], barDelta);
		prevBarInMIDI[0] = barInMIDI[0];
		//tVZFilter_setFreq(&noiseFilt, faster_mtof(knobScaled[5] * 128.0f));

		float filtNoise = tVZFilter_tickEfficient(noiseFilt2, tNoise_tick(myNoise));
		//filtNoise = tVZFilter_tickEfficient(&noiseFilt2, filtNoise);
		float slideNoise = filtNoise * barDelta * knobScaled[1] * 10.0f;

		temp += tPickupNonLinearity_tick(pu[i], tSimpleLivingString3_tick(livStr[i], slideNoise));
	}

	//float outVol = 0.0265625f - (0.2467348f * volumeSmoothed) + (1.253049f * volumeSmoothed * volumeSmoothed);

	float outVol = 0.006721744f + 0.4720157f*volumeSmoothed - 2.542849f*volumeSmoothed*volumeSmoothed + 6.332339f*volumeSmoothed*volumeSmoothed*volumeSmoothed - 3.271672f*volumeSmoothed*volumeSmoothed*volumeSmoothed*volumeSmoothed;


	//temp = input;
	temp *= outVol * masterVolFromBrain;

	return temp;
}



