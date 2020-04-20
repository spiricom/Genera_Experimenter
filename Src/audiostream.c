/*
 * audiostream.c
 *
 *  Created on: Aug 30, 2019
 *      Author: jeffsnyder
 */


/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "leaf.h"
#include "codec.h"
#include "gpio.h"

//the audio buffers are put in the D2 RAM area because that is a memory location that the DMA has access to.
int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;

void audioFrame(uint16_t buffer_offset);
float audioTick(void);
float map(float value, float istart, float istop, float ostart, float ostop);
float processString(int whichString, float input);
void attackDetectMedian(int whichString, float input);
void attackDetect2(int whichString, float input);
float SFXRhodesTick(int whichString);
void SFXRhodesAlloc(void);


HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;



int whichBoard = 1;


uint8_t codecReady = 0;
float sample = 0.0f;
float rightIn = 0.0f;

#define NUM_SAWTOOTHS 4
tSVF myLowpass[2];
tSawtooth mySaw[2][NUM_SAWTOOTHS];
tCycle mySine[2];
tHighpass dcBlock[2];
tEnvelopeFollower myFollower[2];
tExpSmooth pitchSmoother[2];
tMedianFilter median[2];
tNoise noise[2];
tThreshold threshold[2];
tADSR envelope[2];
tSlide fastSlide[2];
tSlide slowSlide[2];
tSVF lowpass[2];
tRampUpDown updownRamp[2];

uint16_t stringPositions[2];
float stringMappedPositions[2];
float stringFrequencies[2];
float stringMIDIVersionOfFrequencies[2];
uint32_t currentMIDInotes[2];
uint32_t previousMIDInotes[2];
uint8_t stringTouchLH[2] = {0,0};
uint8_t stringTouchRH[2] = {0,0};
float openStringFrequencies[4] = {41.204f, 55.0f, 73.416f, 97.999f};
float octave = 1.0f;

float noiseFloor = -40.0f;
float status[2] = {0.0f, 0.0f};
int outOfThreshPositiveChange[2] = {0,0};
int delayCounter[2] = {0,0};
float currentMaximum[2] = {-120.f,-120.f};
int sahArmed[2] = {0,0};
int previousOutOfThresh[2] = {0,0};
int attackDelay = 400;
 int offLockout[2] = {0,0};
 int offLockoutDelay = 1000;
 float finalFreqs[2];
int noteOnHappened[2] = {0,0};
int noteOffHappened[2] = {0,0};
float noteOnAmplitudes[2] = {0.0f, 0.0f};
int outOfThresh[2] = {0,0};

float detuneAmounts[NUM_SAWTOOTHS] = {0.997f, 0.999f, 1.001f, 1.003f};

// frets are measured at 3 7 12 and 19
float fretMeasurements[4][4] ={
		{25002.0f, 25727.0f, 25485.0f, 25291.0f},
		{17560.0f, 18006.0f, 17776.0f, 17704.0f},
		{10071.0f, 10314.0f, 10075.0f, 10063.0f},
		{2864.0f, 2689.0f, 2610.0f, 2529.0f}
	};


float fretScaling[4] = {0.9f, 0.66666666666f, 0.5f, 0.25f};







//MEMPOOLS
#define SMALL_MEM_SIZE 5000
char smallMemory[SMALL_MEM_SIZE];

#define MEDIUM_MEM_SIZE 500000
char mediumMemory[MEDIUM_MEM_SIZE] __ATTR_RAM_D1;

//#define LARGE_MEM_SIZE 33554432 //32 MBytes - size of SDRAM IC
//char largeMemory[LARGE_MEM_SIZE] __ATTR_SDRAM;

tMempool smallPool;
//tMempool largePool;


/**********************************************/

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;


void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn)
{
	// Initialize LEAF.

	whichBoard = whichBoard * 2; // get the correct board offset (2 strings per board)

	LEAF_init(SAMPLE_RATE, AUDIO_FRAME_SIZE, mediumMemory, MEDIUM_MEM_SIZE, &randomNumber);

	tMempool_init (&smallPool, smallMemory, SMALL_MEM_SIZE);
	//tMempool_init (&largePool, largeMemory, LARGE_MEM_SIZE);

	HAL_Delay(10);

	for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		audioOutBuffer[i] = 0;
	}

	/*
	for (int i = 0; i < 2; i++)
	{
		tSawtooth_init(&mySaw[i]);
		tEnvelopeFollower_init(&myFollower[i], 0.01f, 0.9995f);
		tHighpass_init(&dcBlock[i], 250.0f);
		tExpSmooth_init(&pitchSmoother[i], 80.0f, 0.01f);
		tMedianFilter_init(&median[i], 80);
		tNoise_init(&noise[i], WhiteNoise);
		tThreshold_init(&threshold[i],1.0f, 2.5f);
		tADSR_init(&envelope[i], 0.0f, 100.0f, 0.4f, 20.0f);
		tADSR_setLeakFactor(&envelope[i], 0.999999f);
		tSlide_init(&fastSlide[i],1.0f,4410.0f);
		tSlide_init(&slowSlide[i],1.0f,4410.0f);
		tSVF_init(&lowpass[i], SVFTypeLowpass, 1000.0f, 1.0f);
	}
	*/


	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < NUM_SAWTOOTHS; j++)
		{
			tSawtooth_init(&mySaw[i][j]);
		}
		tSVF_init(&myLowpass[i], SVFTypeLowpass, 5000.0f, 0.5f);
		tCycle_init(&mySine[i]);
		tHighpass_init(&dcBlock[i], 3000.0f);
		tExpSmooth_init(&pitchSmoother[i], 80.0f, 0.002f);
		tNoise_init(&noise[i], WhiteNoise);
		tThreshold_init(&threshold[i],0.5f, 4.0f);
		tADSR_init(&envelope[i], 0.0f, 100.0f, 0.6f, 20.0f);
		tADSR_setLeakFactor(&envelope[i], 0.999998f);
		tSlide_init(&fastSlide[i],1.0f,1110.0f);
		tSlide_init(&slowSlide[i],500.0f,1.0f);
		tSVF_init(&lowpass[i], SVFTypeLowpass, 4000.0f, 1.0f);
		tRampUpDown_init(&updownRamp[i], 0.0f, 104.0f, 1); //5000 samples should be 104 ms
	}


	SFXRhodesAlloc();

	HAL_Delay(1);

	// set up the I2S driver to send audio data to the codec (and retrieve input as well)
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);

	// with the CS4271 codec IC, the SAI Transmit and Receive must be happening before the chip will respond to
	// I2C setup messages (it seems to use the masterclock input as it's own internal clock for i2c data, etc)
	// so while we used to set up codec before starting SAI, now we need to set up codec afterwards, and set a flag to make sure it's ready

	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

}


float inputSamples[2];
void audioFrame(uint16_t buffer_offset)
{
	int i;

	//if the codec isn't ready, keep the buffer as all zeros
	//otherwise, start computing audio!

	if (codecReady)
	{

		for (i = 0; i < (HALF_BUFFER_SIZE); i = i + 2)
		{
			inputSamples[1] = (audioInBuffer[buffer_offset + i] << 8) * INV_TWO_TO_31;
			inputSamples[0] = (audioInBuffer[buffer_offset + i + 1] << 8) * INV_TWO_TO_31;

			audioOutBuffer[buffer_offset + i] = (int32_t)(audioTick() * TWO_TO_23);
			audioOutBuffer[buffer_offset + i + 1] = 0;
		}

	}
}
float intoThresh1 = 0.0f;
float noteOnAmplitude[2] = {0.0f, 0.0f};

float audioTick()
{
	sample = 0.0f;
	for (int whichString = 0; whichString < 2; whichString ++)
	{
		//sample = audioIn;
		//sample = processString(0, audioIn);

		//sample = audioIn;
		attackDetect2(whichString, inputSamples[whichString]);
		processString(whichString, inputSamples[whichString]);
		float tempSample = 0.0f;
		//sample += tNoise_tick(&noise[whichString]);// * tADSR_tick(&envelope[whichString]);
		for (int i = 0; i < NUM_SAWTOOTHS; i++)
		{
			tempSample += tSawtooth_tick(&mySaw[whichString][i]) * tADSR_tick(&envelope[whichString]);
			tempSample += tCycle_tick(&mySine[whichString]) * tADSR_tick(&envelope[whichString]);
			tempSample = tSVF_tick(&myLowpass[whichString], tempSample);
		}
		sample += tempSample;
		//sample +=  SFXRhodesTick(whichString);


		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, outOfThreshPositiveChange[1] & 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, sahArmed[1] & 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, outOfThresh[1] & 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, stringTouchRH[1] & 1);


		sample = tanhf(sample * 1.6f);
		//sample = LEAF_clip(-1.0f, dbtoa(intoThresh1) * 0.5f, 1.0f);
		//sample = 0.0f;
		//sample = smoothed;

		//sample = tRampUpDown_sample(&updownRamp[0]);
		//sample = powf(sample, 0.5f);
		//sample += outOfThreshPositiveChange[0];
		//sample -= noteOnHappened[0];
	}
	return sample;

}



float previousRampSmoothed[2] = {0.0f, 0.0f};
int downCounter[2] = {0,0};
void attackDetect2(int whichString, float input)
{
	float intoThresh = 0.0f;
	float dbSmoothed = 0.0f;
	float dbSmoothed2 = 0.0f;
	float smoothed = 0.0f;
	float smoothed2 = 0.0f;
	float tempAbs = 0.0f;
	float increment = 0.005f;
	//float increment = 0.000755857898715f;
	//float increment = 0.000455857898715f;
	//float increment = 0.000255857898715f;
	float rampsmoothed = 0.0f;

	noteOnHappened[whichString] = 0;
	input = tHighpass_tick(&dcBlock[whichString], input);
	//input = tSVF_tick(&lowpass[whichString], input);
	tempAbs = fabsf(input);

	smoothed = tSlide_tick(&fastSlide[whichString], tempAbs);
	smoothed2  = tSlide_tick(&slowSlide[whichString], smoothed);
	dbSmoothed = LEAF_clip(-54.0f, atodb(smoothed), 12.0f);

	dbSmoothed2 = LEAF_clip(-54.0f, atodb(smoothed2), 12.0f);
	intoThresh = dbSmoothed - dbSmoothed2;

	if (whichString == 0)
	{
		intoThresh1 = intoThresh;
	}
	outOfThresh[whichString] = tThreshold_tick(&threshold[whichString], intoThresh);
	if ((outOfThresh[whichString] > 0) && (previousOutOfThresh[whichString] == 0))
	{
		outOfThreshPositiveChange[whichString] = 1;
	}

	else
	{
		outOfThreshPositiveChange[whichString] = 0;
	}

	previousOutOfThresh[whichString] = outOfThresh[whichString];



	//if you didn't get an attack within the last 1323 samples, and you got one now
	if ((status[whichString] <= 0.0f) && (outOfThreshPositiveChange[whichString] == 1))
	{
		status[whichString] = 1.0f;
		currentMaximum[whichString] = 0.0f;
		delayCounter[whichString] = 50;
		sahArmed[whichString] = 1;
	}
	else if (status[whichString] > 0.0f)
	{
		status[whichString] = status[whichString] - (increment);
	}
	tRampUpDown_setDest(&updownRamp[whichString], tempAbs);
	rampsmoothed = tRampUpDown_tick(&updownRamp[whichString]);

	//update the maximum of the samples since last reset
	if (rampsmoothed > currentMaximum[whichString])
	{
		currentMaximum[whichString] = rampsmoothed;
	}

	if (delayCounter[whichString] > 0)
	{
		delayCounter[whichString]--;
	}

	if ((intoThresh < (previousRampSmoothed[whichString] - 0.004f)) && (sahArmed[whichString] == 1))
	{
		downCounter[whichString]++;
	}
	else if ((intoThresh >= (previousRampSmoothed[whichString]) - 0.1f) && (sahArmed[whichString] == 1))
	{
		downCounter[whichString] = 0;
	}

	if ((sahArmed[whichString] == 1) && (delayCounter[whichString] == 0))
	//if you waited some number of samples to ride to the peak, then now make a noteOn event
	//if ((delayCounter[whichString] == 0) && (sahArmed[whichString] == 1))
	{
		//float tempAmp = map(currentMaximum, -60.0f, 6.0f, 0.0f, 1.0f);
		float tempAmp = powf(currentMaximum[whichString], 0.5f);
		tADSR_on(&envelope[whichString], 0.5f);
		noteOnAmplitude[whichString] = tempAmp;
		noteOnHappened[whichString] = 1;
		sahArmed[whichString] = 0;
		downCounter[whichString] = 0;
		offLockout[whichString] = offLockoutDelay;
	}
	previousRampSmoothed[whichString] = intoThresh;

	if (offLockout[whichString] > 0)
	{
		offLockout[whichString]--;
	}

}
float knobParams[2];
float joy_x;
float joy_y;
int LHmuteCounter[2] = {0,0};
int LHstabilityCounter[2] = {0,0};
int RHmuteCounter[2] = {0,0};
float stringFreqs[2] = {0.0f, 0.0f};
float processString(int whichString, float input)
{
	int whichStringOfFour = whichString + whichBoard;

	stringTouchLH[whichString] = (SPI_RX[8] >> whichStringOfFour) & 1;
	stringTouchRH[whichString] = (SPI_RX[8] >> (whichStringOfFour + 4)) & 1;

	noteOffHappened[whichString] = 0;


	stringPositions[whichString] =  ((uint16_t)SPI_RX[whichStringOfFour * 2] << 8) + ((uint16_t)SPI_RX[(whichStringOfFour * 2) + 1] & 0xff);




	if (stringPositions[whichString] == 65535)
	{
		stringFrequencies[whichString] = openStringFrequencies[whichStringOfFour];
		stringMIDIVersionOfFrequencies[whichString] = LEAF_frequencyToMidi(stringFrequencies[whichString]);
	}
	else
	{
		stringMappedPositions[whichString] = map((float)stringPositions[whichString], fretMeasurements[1][whichStringOfFour], fretMeasurements[2][whichStringOfFour], fretScaling[1], fretScaling[2]);
		stringFrequencies[whichString] = ((1.0 / stringMappedPositions[whichString])) * openStringFrequencies[whichStringOfFour];
		stringMIDIVersionOfFrequencies[whichString] = LEAF_frequencyToMidi(stringFrequencies[whichString]);
	}

	//check for muting
	if (stringTouchLH[whichString])
	{
		LHmuteCounter[whichString]++;
	}
	else
	{
		LHmuteCounter[whichString] = 0;
	}

	if (stringTouchRH[whichString])
	{
		RHmuteCounter[whichString]++;
	}
	else
	{
		RHmuteCounter[whichString] = 0;
	}
	if (((LHmuteCounter[whichString] > 1000) && (stringPositions[whichString] == 65535)) || (RHmuteCounter[whichString] > 400))
	{
		if (offLockout[whichString] == 0)
		{
			tADSR_off(&envelope[whichString]);
			noteOffHappened[whichString] = 1;
		}
	}
	else
	{
		//tEnvelopeFollower_decayCoeff(&myFollower[whichString], 0.99999f);
	}
	if ((stringTouchLH[whichString] && (stringPositions[whichString] != 65535)) || ((stringTouchLH[whichString] == 0) && (stringPositions[whichString] == 65535)))
	{
		LHstabilityCounter[whichString]++;
	}
	else
	{
		LHstabilityCounter[whichString] = 0;
	}
	//if (LHstabilityCounter[whichString] > 8000)
	{
		tExpSmooth_setDest(&pitchSmoother[whichString], mtof((round(stringMIDIVersionOfFrequencies[whichString]))));
	}
		float myFreq = tExpSmooth_tick(&pitchSmoother[whichString]) * octave;
		for (int i = 0; i < NUM_SAWTOOTHS; i++)
		{
			tSawtooth_setFreq(&mySaw[whichString][i], myFreq * detuneAmounts[i]);
			tCycle_setFreq(&mySine[i], myFreq * 0.5f);
		}

		joy_x = ((SPI_RX[9] << 8) + (SPI_RX[10])) * 0.00025f;
		joy_y = ((SPI_RX[11] << 8) + (SPI_RX[12])) * 0.00025f;
		knobParams[0] = faster_mtof(joy_x * 230.0f) ; //brightness
		tSVF_setFreq(&myLowpass[whichString], LEAF_clip(50.0f, (knobParams[0] + (myFreq * 3.0f)) * tADSR_tick(&envelope[whichString]), 18000.0f));


		stringFreqs[whichString] =myFreq;

	return 0.0f;

}


///FM RHODES ELECTRIC PIANO SYNTH
#define NUM_VOC_VOICES 2

tCycle FM_sines[NUM_VOC_VOICES][6];
float FM_freqRatios[4][6] = {{1.0f, 1.0001f, 1.0f, 3.0f, 1.0f, 1.0f}, {2.0f, 2.0001f, .99999f, 3.0f, 5.0f, 8.0f},  {1.0f, 2.0f, 1.0f, 7.0f, 3.0f, 4.0f}, {1.0f, 2.0f, 1.0f, 7.0f, 3.0f, 4.0f}};
float FM_indices[4][6] = {{1016.0f, 0.0f, 120.0f, 32.0f, 208.0f, 168.0f}, {100.0f, 100.0f, 300.0f, 300.0f, 10.0f, 5.0f}, {500.0f, 50.0f, 500.0f, 10.0f,0.0f, 0.0f}, {50.0f, 128.0f, 1016.0f, 528.0f, 4.0f, 0.0f}};
float FM_decays[4][6] = {{64.0f, 2000.0f, 3000.0f, 3400.0f, 3200.0f, 3100.0f}, {2000.0f, 300.0f, 800.0f, 3000.0f, 340.0f, 50.0f}, {20.0f, 50.0f, 50.0f, 10.0f, 30.0f, 20.0f}, {584.0f, 1016.0f, 1016.0f, 1000.0f, 600.0f, 500.0f}};
float FM_sustains[4][6] = {{0.9f, 0.9f, 0.9f, 0.9f, 0.7f, 0.7f}, {0.5f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f}, {0.3f, 0.3f, 0.3f, 0.3f, 0.0f, 0.0f},{0.5f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f}};
float FM_attacks[4][6] = {{7.0f, 7.0f, 7.0f, 7.0f, 7.0f, 7.0f}, {7.0f, 7.0f, 7.0f, 7.0f, 7.0f,7.0f}, {10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f},{1000.0f, 680.0f, 250.0f, 1300.0f, 750.0f, 820.0f}};
tADSR FM_envs[NUM_VOC_VOICES][6];
float feedback_output = 0.0f;

tCycle tremolo;

int Rsound = 2;

char* soundNames[4];

//FM Rhodes
void SFXRhodesAlloc()
{
	soundNames[0] = "DARK  ";
	soundNames[1] = "LIGHT ";
	soundNames[2] = "BASS  ";
	soundNames[3] = "PAD   ";
	for (int i = 0; i < NUM_VOC_VOICES; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			tCycle_initToPool(&FM_sines[i][j], &smallPool);
			tADSR_initToPool(&FM_envs[i][j], FM_attacks[Rsound][j], FM_decays[Rsound][j], FM_sustains[Rsound][j], 100.0f, &smallPool);
			tADSR_setLeakFactor(&FM_envs[i][j], 0.999987f);
		}
	}
	tCycle_initToPool(&tremolo, &smallPool);
	tCycle_setFreq(&tremolo, 3.0f);


}


float SFXRhodesTick(int whichString)
{
	joy_x = ((SPI_RX[9] << 8) + (SPI_RX[10])) * 0.00025f;
	joy_y = ((SPI_RX[11] << 8) + (SPI_RX[12])) * 0.00025f;
	knobParams[0] = (joy_x * 2.0f) + 0.7f; //brightness
	knobParams[1] = joy_y; //brightness
	float sample = 0.0f;

	//tCycle_setFreq(&tremolo, knobParams[2]);

	if (noteOnHappened[whichString])
	{
		for (int j = 0; j < 6; j++)

		{
			tADSR_on(&FM_envs[whichString][j], noteOnAmplitude[whichString] * 2.0f);// * 0.0078125f);
		}

	}
	if (noteOffHappened[whichString])
	{
		for (int j = 0; j < 6; j++)
			{
				tADSR_off(&FM_envs[whichString][j]);
			}
	}

		float myFrequency = stringFreqs[whichString];
		tCycle_setFreq(&FM_sines[whichString][5], (myFrequency  * FM_freqRatios[Rsound][5]) + (FM_indices[Rsound][5] * feedback_output * knobParams[0]));
		feedback_output = tCycle_tick(&FM_sines[whichString][5]);
		tCycle_setFreq(&FM_sines[whichString][4], (myFrequency  * FM_freqRatios[Rsound][4]) + (FM_indices[Rsound][4] * feedback_output * knobParams[0] * tADSR_tick(&FM_envs[whichString][5])));
		tCycle_setFreq(&FM_sines[whichString][3], (myFrequency  * FM_freqRatios[Rsound][3]) + (FM_indices[Rsound][3] * knobParams[0] * tCycle_tick(&FM_sines[whichString][4]) * tADSR_tick(&FM_envs[whichString][4])));
		tCycle_setFreq(&FM_sines[whichString][2], (myFrequency  * FM_freqRatios[Rsound][2]) + (FM_indices[Rsound][2] * knobParams[0] * tCycle_tick(&FM_sines[whichString][3]) * tADSR_tick(&FM_envs[whichString][3])));
		tCycle_setFreq(&FM_sines[whichString][1], myFrequency  * FM_freqRatios[Rsound][1]);
		tCycle_setFreq(&FM_sines[whichString][0],( myFrequency  * FM_freqRatios[Rsound][0]) + (FM_indices[Rsound][0] * knobParams[0] * tCycle_tick(&FM_sines[whichString][1]) * tADSR_tick(&FM_envs[whichString][1])));



		sample += tCycle_tick(&FM_sines[whichString][2]) * tADSR_tick(&FM_envs[whichString][2]);
		sample += tCycle_tick(&FM_sines[whichString][0]) * tADSR_tick(&FM_envs[whichString][0]);

	float tremoloSignal = ((tCycle_tick(&tremolo) * 0.5f) + 0.5f) * knobParams[1];
	sample = sample * (tremoloSignal + (1.0f - knobParams[1]));

	sample *= 1.0f;
	return sample;
	//sample = LEAF_shaper(sample, 1.0f);
	//sample *= 0.8f;
}


float map(float value, float istart, float istop, float ostart, float ostop)
{
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
	;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	;
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  ;
}


void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	audioFrame(HALF_BUFFER_SIZE);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	audioFrame(0);
}


void attackDetectMedian(int whichString, float input)
{
	float intoThresh = 0.0f;
	float medianOut = 0.0f;
	float clippedDbSmoothed = 0.0f;
	float dbSmoothed = 0.0f;
	float smoothed = 0.0f;
	float tempAbs = 0.0f;

	//float increment = 0.000755857898715f;
	float increment = 0.000455857898715f;

	input = tHighpass_tick(&dcBlock[whichString], input);
	input = tSVF_tick(&lowpass[whichString], input);
	tempAbs = fabsf(input);

	smoothed = tSlide_tick(&fastSlide[whichString], tempAbs);
	dbSmoothed = atodb(smoothed);
	clippedDbSmoothed = LEAF_clip(noiseFloor, dbSmoothed, 6.0f);
	medianOut = tMedianFilter_tick(&median[whichString], clippedDbSmoothed);
	intoThresh = clippedDbSmoothed - medianOut;


	int outOfThresh = tThreshold_tick(&threshold[whichString], intoThresh);
	if ((outOfThresh > 0) && (previousOutOfThresh[whichString] == 0))
	{
		outOfThreshPositiveChange[whichString] = 1;
	}

	else
	{
		outOfThreshPositiveChange[whichString] = 0;
	}

	previousOutOfThresh[whichString] = outOfThresh;



	//if you didn't get an attack within the last 1323 samples, and you got one now
	if ((status[whichString] <= 0.0f) && (outOfThreshPositiveChange[whichString] == 1))
	{
		status[whichString] = 1.0f;
		currentMaximum[whichString] = noiseFloor;
		delayCounter[whichString] = attackDelay;
		sahArmed[whichString] = 1;
	}
	else if (status[whichString] > 0.0f)
	{
		status[whichString] = status[whichString] - (increment);
	}

	//update the maximum of the samples since last reset
	if (clippedDbSmoothed > currentMaximum[whichString])
	{
		currentMaximum[whichString] = clippedDbSmoothed;
	}

	if (delayCounter[whichString] > 0)
	{
		delayCounter[whichString]--;
	}

	//if you waited 140 samples to ride to the peak, then now make a noteOn event
	if ((delayCounter[whichString] == 0) && (sahArmed[whichString] == 1))
	{
		//float tempAmp = map(currentMaximum, -60.0f, 6.0f, 0.0f, 1.0f);
		float tempAmp = dbtoa(currentMaximum[whichString]);
		tempAmp = LEAF_clip(0.0f, tempAmp, 1.0f);
		tADSR_on(&envelope[whichString], tempAmp);
		offLockout[whichString] = offLockoutDelay;
		sahArmed[whichString] = 0;
	}

	if (offLockout[whichString] > 0)
	{
		offLockout[whichString]--;
	}
}
