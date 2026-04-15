


#include "main.h"
#include "leaf.h"
#include "codec.h"
#include "i2c.h"
#include "gpio.h"
#include "spi.h"
#include "parameters.h"
#include "audiostream.h"
#include "string4.h"
#include "string2.h"
#include "string1.h"
#include "synth.h"

tPattiString stringsP[NUM_STRINGS_PER_BOARD];
float string4Defaults[20] = {0.7f, 0.8f, 0.8f, 0.6f, 0.25f, 0.15f, 0.0f, 0.0f, 0.1f, 0.0f, 0.9f, 0.2f, 0.2f, 0.2f, 0.4f, 0.0f, 0.8f, 0.0f, 0.5f, 0.5f};

Lfloat string4Params[10][3][3] =
{


		{
				{416.0f, 8000.0f, 1.6f},{827.0f, 11560.0f, 0.72f},{1649.0f, 11760.0f, 0.64f}
		},
		{
				{370.0f, 9350.0f, 1.0f},{740.0f, 10870.0f, 0.72f},{1470.0f, 11760.0f, 0.7f}
		},
		{
				{330.0f, 7745.0f, 0.9f}, {660.0f, 9200.0f, 0.88f},{1330.0f, 11760.0f, 0.56f}
		},
		{
				{308.0f, 9500.0f, 1.6f},{615.0f, 9020.0f, 1.4},{1236.0f, 11760.0f, 1.4f}
		},
		{
				{247.0f, 7880.0f, 2.3f}, {495.0f, 9270.0f, 1.1}, {1000.0f, 11760.0f, 0.64f}
		},
		{
				{208.0f, 5733.0f, 2.4f}, {416.0f, 5800.0f, 1.3f}, {837.0f, 11670.0f, 0.5f}
		},
		{
				{185.0f, 5550.0f, 2.6f}, {368.0f, 7100.0f, 1.74f}, {740.0f, 10300.0f, 0.67f}
		},
		{
				{164.0f, 6341.0f, 2.1f}, {327.0f, 5500.0f, 1.8f}, {655.0f, 9177.0f, 0.45f}
		},
		{
				{146.0f, 6190.0f, 3.0f}, {293.0f, 4100.0f, 1.9f}, {586.0f, 8000.0f, 0.7f}
		},
		{
				{123.0f, 5000.0f, 3.1f}, {247.0f, 6700.0f, 2.8f}, {486.0f, 6900.0f, 0.66f}
		}
};



void __ATTR_ITCMRAM audioInitString4()
{
	if (whichStringModelLoaded != String4Loaded)
	{
		if (whichStringModelLoaded == String1Loaded)
		{
			audioFreeString1();
		}
		if (whichStringModelLoaded == String2Loaded)
		{
			audioFreeString2();
		}
		else if (whichStringModelLoaded == SynthLoaded)
		{
			audioFreeSynth();
		}
		for (int v = 0; v < numStringsThisBoard; v++)
		{
			tPattiString_initToPool(&stringsP[v], &mediumPool);

			 //tPattiString_setWoundOrUnwound(stringsP[v],((firstString+v) > 3)); //string 5 is first wound string (4 in zero-based counting)
		}

		whichStringModelLoaded = String4Loaded;
	}
}


void __ATTR_ITCMRAM audioFreeString4()
{
	for (int v = 0; v < numStringsThisBoard; v++)
	{
		//tPattiString_free(&stringsP[v]);
	}
}

void __ATTR_ITCMRAM audioSwitchToString4()
{
	//load string2 default params:
	audioInitString4();
	for (int i = 0; i < 20; i++)
	{
		tExpSmooth_setFactor(knobSmoothers[i], 0.001f);
		if (voice == 62)
		{
			tExpSmooth_setValAndDest(knobSmoothers[i], string4Defaults[i]);
		}
		else
		{
			tExpSmooth_setValAndDest(knobSmoothers[i], loadedKnobParams[i]);
		}
		knobFrozen[i] = 1;
	}
	audioFrameFunction = audioFrameString4;
	presetReady = 1;
}



void __ATTR_ITCMRAM audioFrameString4(uint16_t buffer_offset)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		uint32_t tempCountFrame = DWT->CYCCNT;
		int32_t current_sample = 0;


		if (resetStringInputs)
		{
			for (int i = 0; i < numStringsThisBoard; i++)
			{
				//note off
				tPattiString_mute(stringsP[i]);
				previousStringInputs[i] = 0;
			}
			resetStringInputs = 0;
			newPluck = 1;
		}
		for (int i = 0; i < numStringsThisBoard; i++)
		{

			//tPattiString_setFullStringFreq(stringsP[i], 370.f);
			//tPattiString_setPickupWidth(stringsP[i], 0.01f);
			//tPattiString_setPickupPos(stringsP[i], 0.9f);
		}
		//mono operation, no need to compute right channel. Also for loop iterating by 2 instead of 1 to avoid if statement.
		for (int i = 0; i < HALF_BUFFER_SIZE; i+=2)
		{
			int iplusbuffer = buffer_offset + i;
			current_sample = (int32_t)(audioTickString4() * TWO_TO_23);
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
		timeFrame = DWT->CYCCNT - tempCountFrame;
		frameLoadPercentage = (float)timeFrame * frameLoadMultiplier;
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

uint32_t thisFrameCount4 = 0;
float __ATTR_ITCMRAM audioTickString4(void)
{
	float temp = 0.0f;
	float theNote[NUM_STRINGS_PER_BOARD];

	float volumeSmoothed = tExpSmooth_tick(volumeSmoother);

	for (int i = 0; i < 20; i++)
	{
		knobScaled[i] = tExpSmooth_tick(knobSmoothers[i]);
	}
	for (int i = 0; i < numStringsThisBoard; i++)
	{

		//tPattiString_setFullStringFreq(stringsP[i], 370.f);
		//tPattiString_setPickupWidth(stringsP[i], 0.5f * knobScaled[0]);
		//tPattiString_setPickupPos(stringsP[i], 1.0f - (0.5f * knobScaled[1]));

		if (knobScaled[0] < 0.3f)
		{
			tPattiString_setFullStringFreq(stringsP[i], 370.0f); //open string 1
			tPattiString_setPickupWidth(stringsP[i], 0.04583333f); //1.1"
			tPattiString_setPickupPos(stringsP[i], 0.92183333f); //1.9" -JS should check that we're actually centering the bridge on this point, I think it might not be the case
			tPattiString_setPluckPos(stringsP[i], 0.65f);//0.89583333f); //2.5" from bridge
		}
		else if ((knobScaled[0] >= 0.3f) && (knobScaled[0] < 0.6f))
		{
			tPattiString_setFullStringFreq(stringsP[i], 185.0f); //open string 7
			tPattiString_setPickupWidth(stringsP[i], 0.04583333f); //1.1"
			tPattiString_setPickupPos(stringsP[i], 0.92183333f); //1.9" -JS should check that we're actually centering the bridge on this point, I think it might not be the case
			tPattiString_setPluckPos(stringsP[i], 0.89583333f); //2.5" from bridge
		}
		else
		{
			tPattiString_setFullStringFreq(stringsP[i], 123.0f); //open string 10
			tPattiString_setPickupWidth(stringsP[i], 0.04583333f); //1.1"
			tPattiString_setPickupPos(stringsP[i], 0.92183333f); //1.9" -JS should check that we're actually centering the bridge on this point, I think it might not be the case
			tPattiString_setPluckPos(stringsP[i], 0.89583333f); //2.5" from bridge
		}
		//tPattiString_setPickupWidth(stringsP[i], 0.04583333f); //1.1"
		//tPattiString_setPickupPos(stringsP[i], 0.92183333f); //1.9" -JS should check that we're actually centering the bridge on this point, I think it might not be the case
		stringsP[i]->pluckShape = (knobScaled[1]*20.0f)+ 1.0;
		tPattiString_setPickupFilterAmount(stringsP[i], knobScaled[2]);
		tPattiString_setNonlinearityAmount(stringsP[i], knobScaled[3]);
		tPattiString_setVerticalGain(stringsP[i], knobScaled[4]*2.0f);
		tPattiString_setHorizontalGain(stringsP[i], knobScaled[5]*2.0f);
		tPattiString_setDecay(stringsP[i], knobScaled[6] * 10.0f);
		tPattiString_setBrightness(stringsP[i], knobScaled[7]);
		tPattiString_setNonlinearScalingV(stringsP[i], (knobScaled[8] * 2.0f)+ 1.0f);
		tPattiString_setNonlinearScalingH(stringsP[i], (knobScaled[9] * 2.0f) + 1.0f);
		stringsP[i]->pickupMixAmount = knobScaled[10];
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
		for (int i = 0; i < numStringsThisBoard; i++)
		{
			if ((previousStringInputs[i] == 0) && (stringInputs[i] > 0))
			{
				float amplitz = stringInputs[i] * 0.000015259021897f;
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
				tPattiString_setFreq(stringsP[i], finalFreq);
				tPattiString_pluck(stringsP[i],  amplitz);

			}
			else if ((previousStringInputs[i] > 0) && (stringInputs[i] == 0))
			{
				//note off
				tPattiString_mute(stringsP[i]);
			}
			previousStringInputs[i] = stringInputs[i];
		}
		newPluck = 0;
	}
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
		float openStringFreq = mtofTableLookup(theNote[i]-barInMIDI[i]);
		//tTString_setWindingsPerInch(stringsP[i],LEAF_map(openStringFreq, 123.0f, 247.0f, 70.0f, 120.0f));
#if 0
		if (thisFrameCount4 == 0)
		{

			float thisString = (firstString + i);
			float thisStringProportion = thisString * invNumStrings;
			float thisHarmonic = (knobScaled[10] * (1.0f - thisStringProportion)) + (knobScaled[11] * thisStringProportion);
			float harmonic = (thisHarmonic * 8.0f) + 2.0f;
			float inHarm = LEAF_map(theNote[i], 20.0f, 76.0f, 0.0001f, 0.00001f);
			if (knobScaled[5] > 0.05f)
			{
				inHarm = LEAF_clip(0.00000001f, inHarm * knobScaled[5], 0.01f);
				tTString_setHarmonicity(stringsP[i], inHarm, finalFreq);
				tTString_setInharmonic(stringsP[i], 1);
				tTString_setHarmonic(stringsP[i],harmonic);
			}
			else
			{
				tTString_setInharmonic(stringsP[i], 0);
				tTString_setHarmonic(stringsP[i],(uint32_t)harmonic);
			}

		}
#endif





		//tTString_setPickupAmount(&strings[i], knobScaled[7]);
		//tTString_setBarPosition(stringsP[i],barInMIDI[i]);
		//tTString_setBarDrive(&strings[i],knobScaled[4]);



		//tTString_setFeedbackStrength(stringsP[i],knobScaled[6]);
		//tTString_setFeedbackReactionSpeed(stringsP[i],knobScaled[7]);

		//tTString_setRippleDepth(stringsP[i],knobScaled[9]);


		//tTString_setTensionSpeed(&strings[i],knobScaled[19]);






		tPattiString_setFreq(stringsP[i], finalFreq);


		//Lfloat decayScaling = fastPowf(2.0f, knobScaled[0] * 4.0f - 2.0f); //0.5-2.0f
		//Lfloat filterScaling = fastPowf(2.0f, knobScaled[1] * 4.0f - 2.0f); //0.5-2.0f

		uint32_t which = 0;
		float alpha = 0.0f;
		float oneMinusAlpha = 1.0f;
		for (int i = 0; i < 9; i++)
		{
			//find the closest open string frequency and interpolate
			if (string4Params[i][0][0] > openStringFreq)
			{
				if (string4Params[i+1][0][0] < openStringFreq)
				{
					which = i;
					alpha = LEAF_mapToZeroToOneOutput(openStringFreq, string4Params[i][0][0],string4Params[i+1][0][0]);
					oneMinusAlpha = 1.0f - alpha;
				}
				else if (i == 8)
				{
					which = 9;
				}
			}
		}

		float barHeight = barInMIDI[i] / 12.0f;

		uint32_t barHeightInt = (uint32_t)barHeight;
		float barHeightAlpha = barHeight - barHeightInt;
		float barHeightOneMinusAlpha = 1.0f - barHeightAlpha;
		uint32_t barHeightIntPlusOne = LEAF_clip(0, barHeightInt + 1, 2);

		uint32_t whichPlusOne = LEAF_clip(0, which+1, 9);
		float filterFreq = 0.0f;
		float decayTime = 0.0f;

		if ((which == 0) && (alpha < 0.001f))
		{
			//it's higher than measurements
			float freqRatio1 = string4Params[0][barHeightInt][1] / string4Params[0][barHeightInt][0];
			float freqRatio2 = string4Params[0][barHeightIntPlusOne][1] / string4Params[0][barHeightIntPlusOne][0];

			filterFreq = (freqRatio1 * barHeightOneMinusAlpha + freqRatio2 * barHeightAlpha) * openStringFreq;

			float decayRatio1 = string4Params[0][barHeightInt][2] / string4Params[0][barHeightInt][0];
			float decayRatio2 = string4Params[0][barHeightIntPlusOne][2] / string4Params[0][barHeightIntPlusOne][0];

			decayTime = (decayRatio1 * barHeightOneMinusAlpha + decayRatio2 * barHeightAlpha) * openStringFreq;
		}

		if (which == 9)
		{
			//it's lower than measurements
			float freq1 = string4Params[9][barHeightInt][1];
			float freq2 = string4Params[9][barHeightIntPlusOne][1];
			filterFreq = (freq1 * barHeightOneMinusAlpha + freq2 * barHeightAlpha);
			float decay1 = string4Params[9][barHeightInt][2];
			float decay2 = string4Params[9][barHeightIntPlusOne][2];
			decayTime = (decay1 * barHeightOneMinusAlpha + decay2 * barHeightAlpha) ;
		}

		else
		{
			Lfloat filterFreq1 = string4Params[which][barHeightInt][1] * oneMinusAlpha + string4Params[whichPlusOne][barHeightInt][1] * alpha;
			Lfloat filterFreq2 = string4Params[which][barHeightIntPlusOne][1] * oneMinusAlpha + string4Params[barHeightIntPlusOne][barHeightInt][1] * alpha;
			filterFreq = filterFreq1 * barHeightOneMinusAlpha + filterFreq2 * barHeightAlpha;
			Lfloat decayTime1 = string4Params[which][barHeightInt][2] * oneMinusAlpha + string4Params[whichPlusOne][barHeightInt][2] * alpha;
			Lfloat decayTime2 = string4Params[which][barHeightIntPlusOne][2] * oneMinusAlpha + string4Params[barHeightIntPlusOne][barHeightInt][2] * alpha;
			decayTime = decayTime1 * barHeightOneMinusAlpha + decayTime2 * barHeightAlpha;
		}


		//tTString_setDecayInSeconds(stringsP[i],decayTime * decayScaling);
		//tTString_setFilterFreqDirectly(stringsP[i], filterFreq * filterScaling);
		float inSample = 0.0f;
		temp += tPattiString_tick(stringsP[i], inSample) * 2.0f;
	}
	//float outVol = 0.0265625f - (0.2467348f * volumeSmoothed) + (1.253049f * volumeSmoothed * volumeSmoothed);
	float outVol = 0.006721744f + 0.4720157f*volumeSmoothed - 2.542849f*volumeSmoothed*volumeSmoothed + 6.332339f*volumeSmoothed*volumeSmoothed*volumeSmoothed - 3.271672f*volumeSmoothed*volumeSmoothed*volumeSmoothed*volumeSmoothed;
	thisFrameCount4 = (thisFrameCount4 + 1) & 63;
	temp *= outVol * masterVolFromBrain;
	temp = tanhf(temp);
	return LEAF_clip(-1.0f, temp * 0.98f, 1.0f);
}
