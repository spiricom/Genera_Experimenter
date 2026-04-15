/*
 * string3.c
 *
 *  Created on: Jan 11, 2024
 *      Author: jeffsnyder
 */


/*
 * string2.c
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
#include "string3.h"


tStiffString stringsC[NUM_STRINGS_PER_BOARD];
float prevStiffness[NUM_STRINGS_PER_BOARD];
float prevSquishedStiffness[NUM_STRINGS_PER_BOARD];

float prevPuPos[NUM_STRINGS_PER_BOARD];
float prevSquishedPuPos[NUM_STRINGS_PER_BOARD];
float finalPuPos[NUM_STRINGS_PER_BOARD];

float string3Defaults[12] = {0.06f, 0.99f, 0.99f, 0.9f, 0.6f, 0.6f, 0.0f, 0.0f, 0.9f, 0.85f, 0.84f, 0.905f};

//add tDynamicSmoother for pitch

void __ATTR_ITCMRAM audioInitString3()
{

	for (int v = 0; v < numStringsThisBoard; v++)
	{
		tStiffString_init(&stringsC[v], 70, &leaf);
	}
}


void __ATTR_ITCMRAM audioFreeString3()
{
	for (int v = 0; v < numStringsThisBoard; v++)
	{
		tStiffString_free(&stringsC[v]);
	}
}

void __ATTR_ITCMRAM audioSwitchToString3()
{
	for (int i = 0; i < 12; i++)
	{
		tExpSmooth_setFactor(knobSmoothers[i], 0.001f);

		if (voice == 59)
		{
			tExpSmooth_setValAndDest(knobSmoothers[i], string3Defaults[i]);
		}
		else
		{
			tExpSmooth_setValAndDest(knobSmoothers[i], loadedKnobParams[i]);
		}
		knobFrozen[i] = 1;
	}
}

void __ATTR_ITCMRAM audioFrameString3(uint16_t buffer_offset)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		uint32_t tempCountFrame = DWT->CYCCNT;
		int32_t current_sample = 0;

		if (resetStringInputs)
		{
			for (int i = 0; i < numStringsThisBoard; i++)
			{
				//note off
				//tTString_mute(&strings[i]);
				previousStringInputs[i] = 0;
			}
			resetStringInputs = 0;
			newPluck = 1;
		}
		float theNote[NUM_STRINGS_PER_BOARD];
		for (int i = 0; i < numStringsThisBoard; i++)
		{
			theNote[i] = stringMIDIPitches[i] + stringOctave[i];

			if (theNote[i] < 0.0f)
			{
				theNote[i] = 0.0f;
			}
			if (theNote[i] > 127.0f)
			{
				theNote[i] = 127.0f;
			}
			if (isnan(theNote[i]))
			{
				theNote[i] = 64.0f;
			}

			float finalFreq = mtofTableLookup(theNote[i]);
			//float openStringFreq = mtofTableLookup(theNote[i]-barInMIDI[i]);
			tStiffString_setFreqNoUpdate(stringsC[i], finalFreq);

			float openStringFreq = mtofTableLookup(theNote[i]-barInMIDI[i]);
			volatile float ratioOfOpenStringToNote=(finalFreq/ openStringFreq);
			volatile float ratioOfNoteToOpenString = openStringFreq/ finalFreq;


			float newStiffness = (knobScaled[0] * 100.0f);
			if ((newStiffness > (prevStiffness[i] + 1.0f))|| (newStiffness < (prevStiffness[i] - 1.0f)))
			{
				float squishedStiffness = (newStiffness*0.01f) ;
				if ( squishedStiffness != prevSquishedStiffness[i])
				{
					tStiffString_setStiffnessNoUpdate(stringsC[i],squishedStiffness );
					prevSquishedStiffness[i] = squishedStiffness;
				}
				prevStiffness[i] = newStiffness;
			}


			float newpuPos = (knobScaled[10] * 200.0f);
			if ((newpuPos > (prevPuPos[i] + 1.0f))|| (newpuPos < (prevPuPos[i] - 1.0f)))
			{
				float squishedPuPos = (newpuPos*0.005f) ;
				if ( squishedPuPos != prevSquishedPuPos[i])
				{
					finalPuPos[i] = squishedPuPos;
					float pickupKnobVal = (0.5f - finalPuPos[i] * 0.5f);
					float pickupKnobVal2 = pickupKnobVal;
					if (ratioOfNoteToOpenString < pickupKnobVal)
					{
						volatile float difference = pickupKnobVal - ratioOfNoteToOpenString;

						pickupKnobVal2 = ratioOfNoteToOpenString-difference;
					}
					float pickuppos = (pickupKnobVal * knobScaled[1]) + (pickupKnobVal2 * ratioOfOpenStringToNote) * (1.0f - knobScaled[1]);
					tStiffString_setPickupPosNoUpdate(stringsC[i], (pickuppos));
					prevSquishedPuPos[i] = squishedPuPos;
				}
				prevPuPos[i] = newpuPos;
			}





			float pluckKnobVal = (0.4f - knobScaled[11] * 0.4f) + 0.1f;
			float pluckKnobVal2 = pluckKnobVal;
			if (ratioOfNoteToOpenString < pluckKnobVal)
			{
				volatile float difference = pluckKnobVal - ratioOfNoteToOpenString;

				pluckKnobVal2 = ratioOfNoteToOpenString-difference;
			}
			float pluckpos = (pluckKnobVal * knobScaled[2]) + (pluckKnobVal2 * ratioOfOpenStringToNote) * (1.0f - knobScaled[2]);
			tStiffString_setPluckPosNoUpdate(stringsC[i], LEAF_clip(0.1f,pluckpos, 0.9f));

			float dampKnob = (1.0f - knobScaled[9]);

			float mainDecay = (1.0f - knobScaled[8]);
			if (mainDecay < 0.01f)
			{
				mainDecay = 0.0f;
			}
			tStiffString_setDecayNoUpdate(stringsC[i], mainDecay * mainDecay * mainDecay * 0.001f);
			tStiffString_setDecayHighFreqNoUpdate(stringsC[i], (dampKnob*dampKnob*dampKnob)  * 0.00025f);
			tStiffString_updateOscillators(stringsC[i]);
			tStiffString_updateOutputWeights(stringsC[i]);
		}
		//mono operation, no need to compute right channel. Also for loop iterating by 2 instead of 1 to avoid if statement.
		for (int i = 0; i < HALF_BUFFER_SIZE; i+=2)
		{
			int iplusbuffer = buffer_offset + i;
			current_sample = (int32_t)(audioTickString3() * TWO_TO_23);
			audioOutBuffer[iplusbuffer] = current_sample;
			audioOutBuffer[iplusbuffer + 1] = current_sample;
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


float __ATTR_ITCMRAM audioTickString3(void)
{
	float temp = 0.0f;


	float volumeSmoothed = tExpSmooth_tick(volumeSmoother);

	for (int i = 0; i < 12; i++)
	{
		knobScaled[i] = tExpSmooth_tick(knobSmoothers[i]);
	}

	/*
	  specialModeMacroNames[1][0] = "DecayTime ";
	  specialModeMacroNames[1][1] = "Tone      ";
	  specialModeMacroNames[1][2] = "PluckPos  ";
	  specialModeMacroNames[1][3] = "PickupPos ";
	  specialModeMacroNames[1][4] = "SlideNois ";
	  specialModeMacroNames[1][5] = "Stiffness ";
	  specialModeMacroNames[1][6] = "FB Amp    ";
	  specialModeMacroNames[1][7] = "FB Speed  ";
	  specialModeMacroNames[1][8] = "PU Filter ";
	  specialModeMacroNames[1][9] = "Harmonic  ";
	  specialModeMacroNames[1][10] = "HarmPosX  ";
	  specialModeMacroNames[1][11] = "HarmPosY  ";
	  specialModeMacroNames[1][12] = "PUModRate ";
	  specialModeMacroNames[1][13] = "PUModAmp  ";
	  specialModeMacroNames[1][14] = "Ph Harm G ";
	  specialModeMacroNames[1][15] = "PUFilterQ ";
	  specialModeMacroNames[1][16] = "PeakF Q   ";
	  specialModeMacroNames[1][17] = "PeakF Frq ";
	  specialModeMacroNames[1][18] = "Tension G ";
	  specialModeMacroNames[1][19] = "Tension S ";
	  */
	if (newPluck)
	{
		float theNote[NUM_STRINGS_PER_BOARD];
		for (int i = 0; i < numStringsThisBoard; i++)
		{
			if ((previousStringInputs[i] == 0) && (stringInputs[i] > 0))
			{
				Lfloat str3Amp = stringInputs[i] * 0.000015259021897f;
				stringOctave[i] = octave;

				theNote[i] = stringMIDIPitches[i] + stringOctave[i];
				if (theNote[i] < 0.0f)
				{
					theNote[i] = 0.0f;
				}
				if (theNote[i] > 127.0f)
				{
					theNote[i] = 127.0f;
				}
				if (isnan(theNote[i]))
				{
					theNote[i] = 64.0f;
				}
				float finalFreq = mtofTableLookup(theNote[i]);
				tStiffString_setFreq(stringsC[i], finalFreq);
				tStiffString_pluck(stringsC[i], str3Amp);

			}
			else if ((previousStringInputs[i] > 0) && (stringInputs[i] == 0))
			{
				//note off
				tStiffString_mute(stringsC[i]);
				//tTString_mute(&strings[i]);
			}
			else
			{

			}
			previousStringInputs[i] = stringInputs[i];
		}
		newPluck = 0;
	}
	for (int i = 0; i < numStringsThisBoard; i++)
	{

		temp += tStiffString_tick(stringsC[i]) * 0.5f;
	}
	//float outVol = 0.0265625f - (0.2467348f * volumeSmoothed) + (1.253049f * volumeSmoothed * volumeSmoothed);
	float outVol = 0.006721744f + 0.4720157f*volumeSmoothed - 2.542849f*volumeSmoothed*volumeSmoothed + 6.332339f*volumeSmoothed*volumeSmoothed*volumeSmoothed - 3.271672f*volumeSmoothed*volumeSmoothed*volumeSmoothed*volumeSmoothed;

	temp *= outVol * masterVolFromBrain;
	temp = tanhf(temp);
	return LEAF_clip(-1.0f, temp * 0.98f, 1.0f);
}
