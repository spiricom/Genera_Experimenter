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
#include "adc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sai.h"
#include "AVLtree.h"

#define NUM_STRINGS NUM_ADC_CHANNELS

#define ATODB_TABLE_SIZE 512
#define ATODB_TABLE_SIZE_MINUS_ONE 511

#define LHMUTE_COUNTLIM 128
#define RHMUTE_COUNTLIM 256

float atodbTable[ATODB_TABLE_SIZE];


//the audio buffers are put in the D2 RAM area because that is a memory location that the DMA has access to.
int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t ADC_values[NUM_ADC_CHANNELS * ADC_BUFFER_SIZE] __ATTR_RAM_D2;


void audioFrame(uint16_t buffer_offset);
float audioTick(void);
float map(float value, float istart, float istop, float ostart, float ostop);
float processString(int whichString);

HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

volatile int64_t cycleCountVals[4];
float cycleCountAverages[4][3];

uint8_t codecReady = 0;

volatile int lockOutCountdown = 0;

#define NUM_SAWTOOTHS 1
tSVF myLowpass;
tSawtooth mySaw[NUM_SAWTOOTHS];
tCycle mySine;
tExpSmooth pitchSmoother;
tNoise noise;
tADSR pitchEnvelope;
tADSR noiseEnvelope;
tADSR filterEnvelope;

tSimplePoly poly;

uint16_t stringPositions[NUM_STRINGS];
float stringMappedPositions[NUM_STRINGS];
float stringFrequencies[NUM_STRINGS];
float stringMIDIVersionOfFrequencies[NUM_STRINGS];
uint32_t currentMIDInotes[NUM_STRINGS];
uint32_t previousMIDInotes[NUM_STRINGS];
uint8_t stringTouchLH[NUM_STRINGS] = {0,0,0,0};
uint8_t stringTouchRH[NUM_STRINGS] = {0,0,0,0};
float openStringFrequencies[NUM_STRINGS] = {41.204f, 55.0f, 73.416f, 97.999f};
float octave = 1.0f;

float knobParams[2];
float joy_x;
float joy_y;
int LHmuteCounter[NUM_STRINGS] = {0,0,0,0};
int LHstabilityCounter[NUM_STRINGS] = {0,0,0,0};
int RHmuteCounter[NUM_STRINGS] = {0,0,0,0};
float stringFreqs[NUM_STRINGS] = {0.0f, 0.0f, 0.0f, 0.0f};

int ADC_Ready = 0;
float detuneAmounts[NUM_SAWTOOTHS] = {0.997f, 0.999f, 1.001f, 1.003f};

// frets are measured at 3 7 12 and 19   need to redo these measurements with an accurately set capo
float fretMeasurements[NUM_STRINGS][4] ={
		{25002.0f, 25727.0f, 25485.0f, 25291.0f},
		{17560.0f, 18006.0f, 17776.0f, 17704.0f},
		{10071.0f, 10314.0f, 10075.0f, 10063.0f},
		{2864.0f, 2689.0f, 2610.0f, 2529.0f}
	};


float fretScaling[NUM_STRINGS] = {0.9f, 0.66666666666f, 0.5f, 0.25f};

tExpSmooth gainSmoothed;

int waitTimeOver = 0;

//MEMPOOLS
#define SMALL_MEM_SIZE 80000
char smallMemory[SMALL_MEM_SIZE];

#define MEDIUM_MEM_SIZE 519000
char mediumMemory[MEDIUM_MEM_SIZE] __ATTR_RAM_D1;


char largeMemory[LARGE_MEM_SIZE] __ATTR_SDRAM;

tMempool mediumPool;
tMempool largePool;


tPluckDetectorInt pluck[NUM_STRINGS];
tLockhartWavefolder folder;

LEAF leaf;

/**********************************************/

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;


void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn)
{
	// Initialize LEAF.

	LEAF_init(&leaf, SAMPLE_RATE, AUDIO_FRAME_SIZE, smallMemory, SMALL_MEM_SIZE, &randomNumber);

	tMempool_init (&mediumPool, mediumMemory, MEDIUM_MEM_SIZE, &leaf);
	tMempool_init (&largePool, largeMemory, LARGE_MEM_SIZE, &leaf);

	HAL_Delay(10);

	for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		audioOutBuffer[i] = 0;
	}


	LEAF_generate_atodb(atodbTable, ATODB_TABLE_SIZE);

	tSimplePoly_init(&poly,1, &leaf);
	tSimplePoly_setNumVoices(&poly,1);
	poly->recover_stolen = 0;
	tExpSmooth_init(&gainSmoothed, 0.0f, 0.01f, &leaf);
	tLockhartWavefolder_init(&folder, &leaf);

	for (int i = 0; i < NUM_STRINGS; i++)
	{
		tPluckDetectorInt_initToPool(&pluck[i], &mediumPool);

	}


	for (int j = 0; j < NUM_SAWTOOTHS; j++)
	{
		tSawtooth_init(&mySaw[j], &leaf);
	}

	//the monophonic synth voice
	tSVF_init(&myLowpass, SVFTypeLowpass, 5000.0f, 0.5f, &leaf);
	tCycle_init(&mySine, &leaf);
	tExpSmooth_init(&pitchSmoother, 80.0f, 0.002f, &leaf);
	tNoise_init(&noise, PinkNoise, &leaf);
	tADSR_init(&pitchEnvelope, 6.0f, 200.0f, 0.8f, 20.0f, &leaf);
	tADSR_setLeakFactor(&pitchEnvelope, 0.999997f);
	tADSR_init(&noiseEnvelope, 0.0f, 20.0f, 0.0f, 8.0f, &leaf);
	tADSR_init(&filterEnvelope, 1.0f, 500.0f, 0.25f, 100.0f, &leaf);

	HAL_Delay(1);

	//clear the buffer
	for (int i = 0; i < (AUDIO_BUFFER_SIZE); i++)
	{
		audioOutBuffer[i] = 0;
	}

	// set up the I2S driver to send audio data to the codec (and retrieve input as well)
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);

	// with the CS4271 codec IC, the SAI Transmit and Receive must be happening before the chip will respond to
	// I2C setup messages (it seems to use the masterclock input as it's own internal clock for i2c data, etc)
	// so while we used to set up codec before starting SAI, now we need to set up codec afterwards, and set a flag to make sure it's ready

	//init the AH attack detector

	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

}

int ADC_notStarted = 1;
int frameCount = 0;

void audioFrame(uint16_t buffer_offset)
{
	int i;
	//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_9);
	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
	if (ADC_notStarted)
	{
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_values,NUM_ADC_CHANNELS * ADC_BUFFER_SIZE);
		ADC_notStarted = 0;
	}
	//if the codec isn't ready, keep the buffer as all zeros
	//otherwise, start computing audio!
	frameCount++;
	if (codecReady)
	{

		for (i = 0; i < (HALF_BUFFER_SIZE); i = i + 2)
		{
			audioOutBuffer[buffer_offset + i] = (int32_t)(audioTick() * TWO_TO_23);
			//audioOutBuffer[buffer_offset + i + 1] = 0; // don't need to since buffer was cleared
		}

	}
}

float audioTick()
{
	float tempSample = 0.0f;

	//attack click
	for (int i = 0; i < NUM_STRINGS; i++)
	{
		processString(i);
	}
	tempSample += tNoise_tick(&noise) * tADSR_tick(&noiseEnvelope);


	int myString = tSimplePoly_getPitch(&poly, 0);
	float theFreq = stringFrequencies[myString];
	float pitchEnv = tADSR_tick(&pitchEnvelope);
	float filterEnv = tADSR_tick(&filterEnvelope);
	for (int i = 0; i < NUM_SAWTOOTHS; i++)
	{
		tSawtooth_setFreq(&mySaw[i], theFreq * detuneAmounts[i]);

		tempSample += tSawtooth_tick(&mySaw[i]) * pitchEnv;
	}
	tCycle_setFreq(&mySine, theFreq);
	tempSample += tCycle_tick(&mySine) * pitchEnv;

	tSVF_setFreq(&myLowpass, LEAF_clip(20.0f, theFreq * ((filterEnv * 60.0f) + 1.0f), 19000.0f));
	tempSample = tSVF_tick(&myLowpass, tempSample);
	tempSample = tLockhartWavefolder_tick(&folder, tempSample);
	tempSample = tanhf(tempSample);

	if (lockOutCountdown > 0)
	{
		lockOutCountdown--;
	}


	return tempSample;

}



//this keeps min and max, but doesn't do the array for averaging - a bit less expensive
void CycleCounterTrackMinAndMax( int whichCount)
{
	if (cycleCountVals[whichCount] > 0) //the [2] spot in the array will be set to 1 if an interrupt happened during the cycle count -- need to set that in any higher-priority interrupts to make that true
	{
		if ((cycleCountVals[whichCount] < cycleCountAverages[whichCount][1]) || (cycleCountAverages[whichCount][1] == 0))
		{
			cycleCountAverages[whichCount][1] = cycleCountVals[whichCount];
		}
		//update max value ([2])
		if (cycleCountVals[whichCount] > cycleCountAverages[whichCount][2])
		{
			cycleCountAverages[whichCount][2] = cycleCountVals[whichCount];
		}
	}
}

float maxAmp[NUM_STRINGS];
int offLockout[NUM_STRINGS];
int prevPolyOn = 0;

float processString(int whichString)
{

	stringTouchLH[whichString] = (SPI_RX[8] >> whichString) & 1;
	stringTouchRH[whichString] = (SPI_RX[8] >> (whichString + 4)) & 1;

	if (stringTouchRH[0])
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET);
	}

	if (stringTouchRH[1])
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
	}


	//noteOffHappened[whichString] = 0;


	stringPositions[whichString] =  ((uint16_t)SPI_RX[whichString * 2] << 8) + ((uint16_t)SPI_RX[(whichString * 2) + 1] & 0xff);




	if (stringPositions[whichString] == 65535)
	{
		stringFrequencies[whichString] = openStringFrequencies[whichString];
		//stringMIDIVersionOfFrequencies[whichString] = LEAF_frequencyToMidi(stringFrequencies[whichString]);
	}
	else
	{
		stringMappedPositions[whichString] = map((float)stringPositions[whichString], fretMeasurements[1][whichString], fretMeasurements[2][whichString], fretScaling[1], fretScaling[2]);
		stringFrequencies[whichString] = ((1.0 / stringMappedPositions[whichString])) * openStringFrequencies[whichString];
		//stringMIDIVersionOfFrequencies[whichString] = LEAF_frequencyToMidi(stringFrequencies[whichString]);
	}

	//check for muting
	if ((stringTouchLH[whichString]) && (stringPositions[whichString] == 65535))
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
	if (((LHmuteCounter[whichString] > LHMUTE_COUNTLIM) || (RHmuteCounter[whichString] > RHMUTE_COUNTLIM)))
	{


		if (offLockout[whichString] == 0)
		{
			tSimplePoly_noteOff(&poly, whichString);
			if (tSimplePoly_getNumActiveVoices(&poly) == 0)
			{
				tADSR_off(&pitchEnvelope);
				tADSR_off(&filterEnvelope);
				tADSR_off(&noiseEnvelope);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			}
			//tADSR_off(&envelope[whichString]);
			//noteOffHappened[whichString] = 1;
		}

	}
	/*
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
	*/
	//if (LHstabilityCounter[whichString] > 8000)
	{
		//tExpSmooth_setDest(&pitchSmoother[whichString], mtof((round(stringMIDIVersionOfFrequencies[whichString]))));
	}
	//float myFreq = tExpSmooth_tick(&pitchSmoother[whichString]) * octave;
	//for (int i = 0; i < NUM_SAWTOOTHS; i++)
	//{
		//tSawtooth_setFreq(&mySaw[whichString][i], myFreq * detuneAmounts[i]);
	//}
	//tCycle_setFreq(&mySine[whichString], myFreq);
	//joy_x = ((SPI_RX[9] << 8) + (SPI_RX[10])) * 0.00025f;
	//joy_y = ((SPI_RX[11] << 8) + (SPI_RX[12])) * 0.00025f;
	//knobParams[0] = faster_mtof(joy_x * 230.0f) ; //brightness
	//tSVF_setFreq(&myLowpass[whichString], LEAF_clip(50.0f, (knobParams[0] + (myFreq * 3.0f)) * tADSR_tick(&envelope[whichString]), 18000.0f));
	if (offLockout[whichString] > 0)
	{
		offLockout[whichString]--;
	}
	//stringFreqs[whichString] = myFreq;


	return 0.0f;

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

volatile int myTempResult = 0;
float invMaxAmp[NUM_STRINGS];
volatile int alreadyPlaying = 0;

void ADC_Frame(int offset)
{
	//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_11);

	//stuff for cycle counting
	volatile uint32_t tempCount5 = 0;
	volatile uint32_t tempCount6 = 0;
	__disable_irq();
	tempCount5 = DWT->CYCCNT;

	//sampRecords[currentSamp] = frameCount;
	//currentSamp++;
	for (int i = offset; i < ADC_FRAME_SIZE + offset; i++)
	{
		for (int j = 0; j < NUM_ADC_CHANNELS; j++)
		{
			int tempInt = ADC_values[(i*NUM_ADC_CHANNELS) + j];
			//float tempSamp = (((float)tempInt - INV_TWO_TO_15) * INV_TWO_TO_15);

			//stringPositions[j] =  ((uint16_t)SPI_RX[j * 2] << 8) + ((uint16_t)SPI_RX[(j * 2) + 1] & 0xff);
			//if (stringPositions[j] == 65535)
			//{
			//	stringPositions[j] = 0;
			//}
			//stringTouchLH[j] = (SPI_RX[8] >> j) & 1;
			//stringTouchRH[j] = (SPI_RX[8] >> (j + 4)) & 1;
			myTempResult = tPluckDetectorInt_tick(&pluck[j], tempInt);
			if (lockOutCountdown == 0)
			{

				if (myTempResult > 0)
				{
					alreadyPlaying = (tSimplePoly_getPitchAndCheckActive(&poly, 0) == j);
					if (alreadyPlaying)
					{
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
					}
					else
					{

						float tempAmplitude = ((float)myTempResult * INV_TWO_TO_16);
						if (tempAmplitude > maxAmp[j])
						{
							maxAmp[j] = tempAmplitude;
							invMaxAmp[j] = 1.0f / maxAmp[j];
						}
						tempAmplitude = tempAmplitude * invMaxAmp[j];
						int intVersionOfAmp = (int)(tempAmplitude * 127.0f);
						tSimplePoly_noteOn(&poly, j, intVersionOfAmp);

						tADSR_on(&pitchEnvelope, tempAmplitude);
						tADSR_on(&filterEnvelope, tempAmplitude);
						tADSR_on(&noiseEnvelope, tempAmplitude);

						lockOutCountdown = 512;
						offLockout[j] == 512;
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
					}
				}
			}


/*
			if (SDReady)
			{
				writeToSD(SDWriteIndex, tempInt, (int)stringPositions[j],(int) stringTouchLH[j], (int)stringTouchRH[j], j);

				if (memoryPointer >= (LARGE_MEM_SIZE - 300))
				{
					finishSD = 1;
					HAL_ADC_Stop(&hadc1);
					HAL_SAI_DMAStop(&hsai_BlockA1);
					HAL_SAI_DMAStop(&hsai_BlockB1);
				}
			}
*/

			//processString(j);

		}

	}

	ADC_Ready = 1;
   	//cycle counting stuff below. At 48k you have at most 10000 cycles per sample (when running at 480MHz). There is also overhead from the frame processing and function calls, so in reality less than that.
	tempCount6 = DWT->CYCCNT;
	cycleCountVals[0] = tempCount6-tempCount5;
	CycleCounterTrackMinAndMax(0);
	if (cycleCountVals[0] > 9900)
	{
		//setLED_D(255);
		//overflow
	}
	__enable_irq();
	//CycleCounterAverage(0);

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{


	ADC_Frame(ADC_FRAME_SIZE);


}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	ADC_Frame(0);
}
void HAL_ADC_Error(ADC_HandleTypeDef *hadc)
{

}
