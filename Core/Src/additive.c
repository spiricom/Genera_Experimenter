/*
 * additive.c
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

#include "additive.h"


#define NUM_OVERTONES 15


tCycle additive[NUM_STRINGS_PER_BOARD][NUM_OVERTONES];
tADSRT additiveEnv[NUM_STRINGS_PER_BOARD][NUM_OVERTONES];
float additivePickupPos[NUM_STRINGS_PER_BOARD];
float dAp[3][2][NUM_OVERTONES];
float dAi[3][2][NUM_OVERTONES];
float dBs[3][2][NUM_OVERTONES];

float invNumOvertones;
float decayAf1[NUM_OVERTONES];
float decayBs[NUM_OVERTONES];
float decayAfParts[NUM_OVERTONES];
float totalGain_s[3][3];
float gainNormalizers_s[3][3];
float totalGain[2] = {0.0f, 0.0f};
float gainNormalizers[2] = {0.0f, 0.0f};

tExpSmooth tensionAdd[NUM_STRINGS_PER_BOARD];

float partials[18] = {0.073f, 0.164f, 0.19f, 0.181f, 0.151f, 0.103f, 0.0684f, 0.053f, 0.046f, 0.032f, 0.024f, 0.021f, 0.0198f, 0.011f, 0.0114f, 0.0137f, 0.0139f, 0.0101f};
float partialsHigh[18] = {0.092f, 0.312f, 0.311f, 0.299f, 0.3125f, 0.13f, 0.054f, 0.028f, 0.012f, 0.0077f, 0.0018f, 0.003f, 0.0024f, 0.002415f, 0.00145f, 0.00103f, 0.0001f, 0.0014f};
float partialDecays[18] = {49.54f, 41.0f, 32.7f, 19.0f, 16.65f, 12.2f, 8.25f, 6.55f, 6.59f, 4.8f, 4.549f, 3.75f, 3.54f, 2.7f, 1.9f, 2.05f, 1.9f, 1.65f};
float partialDecaysHigh[18] = {15.7f, 11.8f, 9.7f, 8.95f, 7.3f, 4.6f, 4.9f, 4.32f, 2.9f, 2.34f, 1.23f, 2.5f, 2.211f, 1.94f, 1.77f, 0.71f, 0.1f, 0.05f};

float pickupWeights[NUM_STRINGS_PER_BOARD][NUM_OVERTONES];
float finalGains[NUM_STRINGS_PER_BOARD][NUM_OVERTONES];
float gainSum[NUM_STRINGS_PER_BOARD];
float invGainSum[NUM_STRINGS_PER_BOARD];

float stringPartialGains[3][3][18] =
{
		{
				{0.039156f, 0.08295f, 0.100061f, 0.09995f, 0.04556f, 0.0177f, 0.04025f, 0.055531f, 0.051128f, 0.031538f, 0.016563f, 0.0112f, 0.010f, 0.0085f, 0.0076f, 0.0065f, 0.00752f, 0.00042f},
				{0.151324f, 0.162346f, 0.061834f, 0.06991f, 0.0562f, 0.012482f, 0.014779f, 0.02f, 0.005216f, 0.010173f, 0.0f, 0.0f},
				{0.457762f, 0.17526f, 0.06247f, 0.05275f, 0.01362f, 0.0f, 0.002040f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
		},
		{
				{0.030473f, 0.054897f, 0.065547f, 0.056177f, 0.02957f, 0.02767f, 0.028027f, 0.034652f, 0.026227f, 0.0144f, 0.006086f, 0.0027f, 0.0023f, 0.002f, 0.00167f, 0.0012f, 0.0015f, 0.0001112f},
				{0.102318f, 0.102264f, 0.04869f, 0.056057f, 0.025617f, 0.005731f, 0.04339f, 0.00369f, 0.004151f, 0.002016f, 0.001106f, 0.00528f, 0.00206f, 0.001003f, 0.00487f, 0.001016f, 0.00096f, 0.00218f},
				{0.2387f, 0.080883f, 0.01353f, 0.013997f, 0.004522f, 0.00f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
		},
		{
				{0.05561f, 0.10098f, 0.12085f, 0.095168f, 0.04267f, 0.0135f, 0.014507f, 0.015348f, 0.010175f, 0.005007f, 0.002371f, 0.000964f, 0.009175f, 0.008007f, 0.001554f, 0.0007884f},
				{0.15764f, 0.225206f, 0.0948f, 0.023891f, 0.007234f, 0.001718f, 0.001173f, 0.00106f, 0.001278f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
				{0.3303f, 0.0314f, 0.002494f, 0.002405f, 0.00362f, 0.000148f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
		}
};
float stringDecays[3][3][18] =
{
		{
				{35.67f, 33.9234f, 18.64f, 15.892f, 12.619274f, 9.99546f, 7.434127f, 5.322585f, 5.60995f, 4.847801f, 3.186f, 2.8237f, 1.4f, 1.2333f, 0.76f, 0.64444f, 0.462f, 0.31222f},
				{12.83f, 8.75f, 7.2341f, 5.26f, 4.023f, 2.43639f, 1.3249f, 2.28643f, 2.086553f, 1.18f, 0.9f, 0.8f, 0.765f, 0.569f, 0.694f, 0.369f, 0.7382f, 0.235f},
				{6.422086f, 2.9616f, 1.636757f, 1.549297f, 0.924518f, 0.5f, 0.249887f, 0.1f, 0.05f, 0.05f, 0.05f, 0.05f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f}
		},
		{
				{23.38936f, 21.62787f, 14.968f, 12.094512f, 7.996f, 6.746f, 7.17174f, 4.94f, 4.28555f, 3.08621f, 2.5488f, 2.04907f, 1.99443f, 1.6783f, 1.43f, 1.22453f, 1.14f, 0.8421f},
				{15.018f, 10.320317f, 7.221724f, 6.184f, 3.760794f, 1.94116f, 2.561338f, 2.223991f, 1.574f, 1.0494f, 1.43f, 0.5994f, 0.54f, 0.4009f, 0.34f, 0.2554f, 0.162f, 0.12f},
				{5.977f, 4.47291f, 1.56f, 1.611f, 1.211950f, 0.9f, 0.8f, 0.6f, 0.5f, 0.4f, 0.3f, 0.2f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f}
		},
		{
				{15.64294f, 11.2199f, 8.171f, 7.371655f, 5.559978f, 4.685f, 1.91163f, 2.886f, 2.8612f, 2.1999f, 1.774915f, 0.63721, 0.5233f, 0.49f, 0.4233f, 0.305f, 0.276f, 0.153f},
				{8.17129f, 7.534f, 4.435f, 3.148471f, 1.8866f, 1.17f, 1.13f, 0.9495f, 0.912f, 0.6f, 0.5f, 0.4f, 0.3f, 0.2f, 0.1f, 0.05f, 0.03f, 0.02f},
				{4.635f, 2.298f, 1.1369f, 0.924581f, 0.599f, 0.53725f, 0.4f, 0.3f, 0.2f, 0.2f, 0.2f, 0.1f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f}
		}
};

float stringFundamentals[3][3] =
{
		{124.0f, 248.0f, 496.0f},
		{204.0f, 408.0f, 816.0f},
		{370.0f, 740.0f, 1480.0f}
};


float additiveDefaults[12] = {0.1f, 0.50f, 0.5f, 0.95f, 0.0f, 0.1f, 0.2f, 0.96f, 0.3f, 0.1764f, 0.99f, 0.4f};


void __ATTR_ITCMRAM audioInitAdditive()
{
	invNumOvertones = 1.0f / NUM_OVERTONES;
	for (int i = 0; i < NUM_OVERTONES; i++)
	{
		totalGain[0] += partials[i];
		totalGain[1] += partialsHigh[i];
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				totalGain_s[j][k] += stringPartialGains[j][k][i];
			}
		}
	}
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			gainNormalizers_s[i][j] = 1.0f / totalGain_s[i][j];
		}
	}
	gainNormalizers[0] = 1.0f / totalGain[0];
	gainNormalizers[1] = 1.0f / totalGain[1];
	for (int i = 0; i < NUM_OVERTONES; i++)
	{
		partials[i] = partials[i] * gainNormalizers[0];
		partialsHigh[i] = partialsHigh[i] * gainNormalizers[1];
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				stringPartialGains[j][k][i] *= gainNormalizers_s[j][k];
			}
		}
	}
	for (int i = 0; i < NUM_OVERTONES; i++)
	{
		  decayAfParts[i] = (((1.0f/partialDecaysHigh[i])-(1.0f/partialDecays[i])) * 0.004048582995951f);  // divided by 370-123 = 247. value is 1/247
		  decayAf1[i] = decayAfParts[i] * 123.0f;
		  decayBs[i] = (1.0f/partialDecays[i]) - decayAf1[i];
		  for (int j = 0; j < 3; j++)
		  {
			  for (int k = 0; k < 2; k++)
			  {
				  float tempDivisor = (stringFundamentals[j][k+1] -  stringFundamentals[j][k]);
				  dAp[j][k][i] = ((1.0f / stringDecays[j][k+1][i]) - (1.0f / stringDecays[j][k][i])) / tempDivisor;

				  dAi[j][k][i] = dAp[j][k][i] * stringFundamentals[j][k];

				  dBs[j][k][i] = (1.0f/stringDecays[j][k][i]) - dAi[j][k][i];
			  }
		  }
	}
	for (int i = 0; i < NUM_STRINGS_PER_BOARD; i++)
	{
		for (int j = 0; j < NUM_OVERTONES; j++)
		{
			tCycle_init(&additive[i][j], &leaf);
			tADSRT_init(&additiveEnv[i][j], 5.0f, partialDecays[j] * 1000.0f, 0.0f, 150.0f, decayExpBuffer, DECAY_EXP_BUFFER_SIZE, &leaf);
			tExpSmooth_init(&tensionAdd[i], 0.0f, 0.001f, &leaf);
			tExpSmooth_setDest(tensionAdd[i], 0.0f);
		}
		//tExpSmooth_init(&stringFreqSmoothers[i],1.0f, 0.05f, &leaf);
	}
}


void __ATTR_ITCMRAM audioFreeAdditive()
{

}

void __ATTR_ITCMRAM audioSwitchToAdditive()
{
	tVZFilter_setFrequencyAndResonance(noiseFilt,1760.0f, 2.5f);
	tVZFilter_setFrequencyAndResonance(noiseFilt2,61.0f, 2.5f);
	for (int i = 0; i < 12; i++)
	{
		tExpSmooth_setFactor(knobSmoothers[i], 0.001f);
		if (voice == 61)
		{
			tExpSmooth_setValAndDest(knobSmoothers[i], additiveDefaults[i]);
		}
		else
		{
			tExpSmooth_setValAndDest(knobSmoothers[i], loadedKnobParams[i]);
		}
		knobFrozen[i] = 1;
	}
	for (int i = 0; i < numStringsThisBoard; i++)
	{
		tADSRT_setSustain(fenvelopes[i], 0.0f);
	}

}

void __ATTR_ITCMRAM audioFrameAdditive(uint16_t buffer_offset)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	uint32_t tempCountFrame = DWT->CYCCNT;
	int32_t current_sample = 0;

	if (resetStringInputs)
	{
		for (int i = 0; i < numStringsThisBoard; i++)
		{
			//note off
			for (int j = 0; j < NUM_OVERTONES; j++)
			{
				tADSRT_clear(additiveEnv[i][j]);
			}
			tADSRT_clear(fenvelopes[i]);
			previousStringInputs[i] = 0;
		}
		resetStringInputs = 0;
		newPluck = 1;
	}

	for (int i = 0; i < numStringsThisBoard; i++)
	{
		additivePickupPos[i] = (knobScaled[3] * 0.4f) + 0.1f;
		for (int j = 0; j < NUM_OVERTONES; j++)
		{
			Lfloat x0 = additivePickupPos[i] * PI;
			pickupWeights[i][j] = sinf((j + 1) * x0);
		}
	}

	if (newPluck)
	{
		for (int i = 0; i < numStringsThisBoard; i++)
		{
			if ((previousStringInputs[i] == 0) && (stringInputs[i] > 0))
			{
				float amplitz = stringInputs[i] * 0.000015259021897f;
				tExpSmooth_setVal(tensionAdd[i], amplitz);
				stringOctave[i] = octave;
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

				tADSRT_setDecay(fenvelopes[i], 50.0f);
				tADSRT_on(fenvelopes[i], amplitz);
				stringFrequencies[i] = mtofTableLookup(stringMIDIPitches[i]+ stringOctave[i]);
				//float thisDecay = 1.0f / ((decayAfParts[j] * stringFrequencies[i]) + decayBs[j]);
				float thisDecay;
				//float thisGain = map(LEAF_clip(132.0f, stringFrequencies[i], 370.0f), 123.0f, 370.0f, partials[j], partialsHigh[j]);
				float d1;
				float d2;
				float thisGain;
				int thisString = i + firstString;
				float stringFade;
				float fakedFreq = stringFrequencies[i] * (((1.0f - knobScaled[11]) * 3.5f) + 0.5f);
				float height2 = 0.0f;
				float height1 = 0.0f;
				if (thisString < 6)
				{
					stringFade = (float)thisString * 0.2f;
					height2 = LEAF_clip(0.0f, LEAF_map(fakedFreq, stringFundamentals[2][0], stringFundamentals[2][2], 0.0f, 2.0f), 1.99f);
					height1 = LEAF_clip(0.0f, LEAF_map(fakedFreq, stringFundamentals[1][0], stringFundamentals[1][2], 0.0f, 2.0f), 1.99f);
				}
				else
				{
					stringFade = (float)(thisString - 6.0f) * 0.2f;
					height2 = LEAF_clip(0.0f, LEAF_map(fakedFreq, stringFundamentals[1][0], stringFundamentals[1][2], 0.0f, 2.0f), 1.99f);
					height1 = LEAF_clip(0.0f, LEAF_map(fakedFreq, stringFundamentals[0][0], stringFundamentals[0][2], 0.0f, 2.0f), 1.99f);
				}
				float oneMinusStringFade = 1.0f - stringFade;
				//float fakedFreq = stringFrequencies[i];
				//float height2 = LEAF_clip(0.0f,(barInMIDI[0] *  0.083333333333333f) + stringOctave[i] + (knobScaled[2] * 2.0f), 1.99f);
				//float height1 = height2;
				int height1Int = floor(height1);
				float height1Float = height1 - height1Int;
				int height2Int = floor(height2);
				float height2Float = height2 - height2Int;
				for (int j = 0; j < NUM_OVERTONES; j++)
				{
					if (thisString < 6)
					{
						float x1 =  (stringPartialGains[1][height1Int][j] * (1.0f - height1Float)) + (stringPartialGains[1][height1Int + 1][j] * height1Float);
						float x2 =  (stringPartialGains[2][height2Int][j] * (1.0f - height2Float)) + (stringPartialGains[2][height2Int + 1][j] * height2Float);
						thisGain = (x1 * stringFade) + (x2 * oneMinusStringFade);


						if (height2 < 1.0f)
						{
							d2 = 1.0f / ((dAp[2][0][j] * LEAF_clip(stringFundamentals[2][0], fakedFreq, stringFundamentals[2][1])) + dBs[2][0][j]);
						}
						else
						{
							d2 = 1.0f / ((dAp[2][1][j] * LEAF_clip(stringFundamentals[2][1], fakedFreq, stringFundamentals[2][2])) + dBs[2][1][j]);
						}

						if (height1 < 1.0f)
						{
							d1 = 1.0f / ((dAp[1][0][j] * LEAF_clip(stringFundamentals[1][0], fakedFreq, stringFundamentals[1][1])) + dBs[1][0][j]);
						}
						else
						{
							d1 = 1.0f / ((dAp[1][1][j] * LEAF_clip(stringFundamentals[1][1], fakedFreq, stringFundamentals[1][2])) + dBs[1][1][j]);
						}

						thisDecay = (d1 * stringFade) + (d2 * oneMinusStringFade);
					}
					else
					{


						float x1 =  stringPartialGains[0][height1Int][j] + (stringPartialGains[1][height1Int + 1][j] * height1Float);
						float x2 =  stringPartialGains[1][height2Int][j] + (stringPartialGains[2][height2Int + 1][j] * height2Float);
						thisGain = (x1 * stringFade) + (x2 * oneMinusStringFade);

						if (height2 < 1.0f)
						{
							d2 = 1.0f / ((dAp[1][0][j] * LEAF_clip(stringFundamentals[1][0], fakedFreq, stringFundamentals[1][1])) + dBs[1][0][j]);
						}
						else
						{
							d2 = 1.0f / ((dAp[1][1][j] * LEAF_clip(stringFundamentals[1][1], fakedFreq, stringFundamentals[1][2])) + dBs[1][1][j]);
						}

						if (height1 < 1.0f)
						{
							d1 = 1.0f / ((dAp[0][0][j] * LEAF_clip(stringFundamentals[0][0], fakedFreq, stringFundamentals[0][1])) + dBs[0][0][j]);
						}
						else
						{
							d1 = 1.0f / ((dAp[0][1][j] * LEAF_clip(stringFundamentals[0][1], fakedFreq, stringFundamentals[0][2])) + dBs[0][1][j]);
						}

						thisDecay = (d1 * stringFade) + (d2 * oneMinusStringFade);
					}
					thisDecay *= 2000.0f * knobScaled[10];
					tADSRT_setDecay(additiveEnv[i][j], thisDecay);// * randomFactors[currentRandom]);
					currentRandom++;
					tADSRT_on(additiveEnv[i][j], amplitz * thisGain);
					finalGains[i][j] = thisGain;
					currentRandom++;
				}
			}
			else if ((previousStringInputs[i] > 0) && (stringInputs[i] == 0))
			{
				//note off
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				for (int j = 0; j < NUM_OVERTONES; j++)
				{
					tADSRT_off(additiveEnv[i][j]);
				}
				tADSRT_off(fenvelopes[i]);
			}
			previousStringInputs[i] = stringInputs[i];
		}
		newPluck = 0;
	}


		//mono operation, no need to compute right channel. Also for loop iterating by 2 instead of 1 to avoid if statement.
	for (int i = 0; i < HALF_BUFFER_SIZE; i+=2)
	{
		current_sample = (int32_t)(audioTickAdditive() * TWO_TO_23);
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


float __ATTR_ITCMRAM audioTickAdditive(void)
{
	float tempSamp = 0.0f;
	for (int i = 0; i < 12; i++)
	{
		knobScaled[i] = tExpSmooth_tick(knobSmoothers[i]);
	}


	float filtNoise = tVZFilter_tickEfficient(noiseFilt, tNoise_tick(myNoise));
	filtNoise += tVZFilter_tickEfficient(noiseFilt2, tNoise_tick(myNoise));
	filtNoise *= 2.0f;
	float stretch = knobScaled[0];
	stretch = (stretch*stretch*stretch*stretch) * 0.3f;
	float oneMinusPickup = (1.0f - knobScaled[4]);
	float pickup = knobScaled[4];
	float freqWeightKnob = knobScaled[1];
	float oneMinusFreqWeightKnob = 1.0f - knobScaled[1];
	float volumeSmoothed = tExpSmooth_tick(volumeSmoother);
	//float Env2 = 0.0f;
	for (int i = 0; i < numStringsThisBoard; i++)
	{
		float thisTension = tExpSmooth_tick(tensionAdd[i]);
		thisTension = knobScaled[5] * thisTension;
		float tensionSpeed = (1.0f - knobScaled[6]);
		tensionSpeed = tensionSpeed*tensionSpeed*tensionSpeed*tensionSpeed;
		tExpSmooth_setFactor(tensionAdd[i], 0.01f * tensionSpeed + 0.0001f);

		float theMIDI = (stringMIDIPitches[i]+ stringOctave[i]) + thisTension;

		float noiseEnv = tADSRT_tick(fenvelopes[i]); //noise envelope
		tempSamp += filtNoise * noiseEnv *  knobScaled[2];
		stringFrequencies[i] = mtofTableLookup(theMIDI);
		invGainSum[i] = 1.0f;
		if(gainSum[i] > 0.0001f)
		{
			invGainSum[i] = 1.0f / gainSum[i];
		}
		gainSum[i] = 0.0f;
		for (int j = 0; j < NUM_OVERTONES; j++)
		{
			float thisEnv = tADSRT_tick(additiveEnv[i][j]);
			float tempFreq = (stringFrequencies[i] * (j+1) * ((stretch * j) + 1.0f));// * ((Env2 * knobScaled[5])+ 1.0f);
			//float tempFreq = 0.0f;
			float tempGain = ((tempFreq - 15000.0f) * 0.00025f);
			//float tempGain = LEAF_map(tempFreq, 15000.0f, 19000.0f, 0.0, 1.0f);
			tempGain = LEAF_clip(0.0f, (1.0f-tempGain), 1.0f);
			tCycle_setFreq(additive[i][j], tempFreq);
			float upRamp = (j * invNumOvertones);
			float downRamp = 1.0f - (j * invNumOvertones);
			float freqWeight = (upRamp * freqWeightKnob) + (downRamp * oneMinusFreqWeightKnob);
			gainSum[i] += freqWeight * finalGains[i][j];
			float thisWeight = oneMinusPickup + pickupWeights[i][j] * pickup;
			tempSamp += tCycle_tick(additive[i][j]) * thisEnv * thisWeight * freqWeight * invGainSum[i];
		}
	}

	//float outVol = 0.0265625f - (0.2467348f * volumeSmoothed) + (1.253049f * volumeSmoothed * volumeSmoothed);
	float outVol = 0.006721744f + 0.4720157f*volumeSmoothed - 2.542849f*volumeSmoothed*volumeSmoothed + 6.332339f*volumeSmoothed*volumeSmoothed*volumeSmoothed - 3.271672f*volumeSmoothed*volumeSmoothed*volumeSmoothed*volumeSmoothed;


	tempSamp *= 0.99f;
	tempSamp *= outVol;
	tempSamp *= masterVolFromBrain;
	return tempSamp;
}
