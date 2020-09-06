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
#include "sai.h"
#include "ui.h"


//the audio buffers are put in the D2 RAM area because that is a memory location that the DMA has access to.
int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t ADC_values[NUM_ADC_CHANNELS] __ATTR_RAM_D2;


void audioFrame(uint16_t buffer_offset);
float audioTick(float* input);

HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

volatile int32_t cycleCountVals[4];
volatile int32_t cycleCountMinMax[4][2];
uint8_t codecReady = 0;

//LEAF instance
LEAF leaf;

//MEMPOOLS
char smallMemory[SMALL_MEM_SIZE];
char mediumMemory[MEDIUM_MEM_SIZE] __ATTR_RAM_D1;
float largeMemory[LARGE_MEM_SIZE_IN_FLOAT] __ATTR_SDRAM;
char largeMemoryScratch[SCRATCH_MEM_SIZE] __ATTR_SDRAM;
uint32_t scratchPosition = 0;

//small memory will be the default LEAF mempool
// need to create custom mempools for the medium and large memory
tMempool mediumPool;
tMempool largePool;


/**********************************************/

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;

int distortionMode = 0;
tExpSmooth adcSmooth[NUM_ADC_CHANNELS];
tBuffer myWaves[MAX_WAV_FILES];
tMBSampler mySamplers[MAX_WAV_FILES];
tExpSmooth sampleGains[MAX_WAV_FILES];

void CycleCounterTrackMinAndMax( int whichCount);

void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn)
{
	// Initialize LEAF.

	LEAF_init(&leaf, SAMPLE_RATE, AUDIO_FRAME_SIZE, smallMemory, SMALL_MEM_SIZE, &randomNumber);

	tMempool_init (&mediumPool, mediumMemory, MEDIUM_MEM_SIZE, &leaf);
	//tMempool_init (&largePool, largeMemory, LARGE_MEM_SIZE, &leaf);

	HAL_Delay(10);
	leaf.clearOnAllocation = 1;


	for (int i = 0; i< MAX_WAV_FILES; i++)
	{
		tBuffer_init(&myWaves[i], 1, &leaf);
		tBuffer_setBuffer(&myWaves[i], &largeMemory[waves[i][0]], waves[i][3]);
		tMBSampler_initToPool(&mySamplers[i], &myWaves[i], &mediumPool);
		tMBSampler_setMode(&mySamplers[i], PlayNormal);
		tExpSmooth_init(&sampleGains[i], 0.0f, 0.01f, &leaf);
	}


	for(int i = 0; i < NUM_ADC_CHANNELS; i++)
	{
		tExpSmooth_init(&adcSmooth[i], 0.0f, 0.1f, &leaf);
	}

	leaf.clearOnAllocation = 0;


	for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		audioOutBuffer[i] = 0;
	}


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

uint intVersion = 0;
uint intVersionPlusOne = 1;
float floatVersion = 0.0f;
int currentSample = 0;
float adcHysteresisThreshold = 0.001f;
float lastFloatADC[NUM_ADC_CHANNELS];
float floatADC[NUM_ADC_CHANNELS];
float smoothedADC[NUM_ADC_CHANNELS];
void audioFrame(uint16_t buffer_offset)
{


	int i;
	buttonCheck();

	for (int i = 0; i < NUM_ADC_CHANNELS; i++)
	{
		floatADC[i] = ((ADC_values[i]>>6) * INV_TWO_TO_10);

		if (fastabsf(floatADC[i] - lastFloatADC[i]) > adcHysteresisThreshold)
		{
			tExpSmooth_setDest(&adcSmooth[i], floatADC[i]);
			lastFloatADC[i] = floatADC[i];
		}

	}


/*

	params[0] = LEAF_clip(0.0f, (smoothedADC[0] + smoothedADC[8]), 1.0f) * 4000.0f;
    params[1] = LEAF_clip(0.0f, ((1.0f - floatADC[1]) + floatADC[9]), 1.0f) * 1.9f;
    params[2] = LEAF_clip(0.0f, (smoothedADC[2] + smoothedADC[10]), 1.0f) * 160.0f;
    params[3] = LEAF_clip(0.0f, (smoothedADC[3] + smoothedADC[11]), 1.0f);
    params[4] = LEAF_clip(0.0f, ((smoothedADC[4]* 1.1f) - 0.1f), 1.0f);
    params[5] = smoothedADC[5];

    tExpSmooth_setDest(&wanderSmoothers[1], params[1]);
	//float truncatedParam1 = (round(params[1] * 32.0f) * 0.03125f);
	float truncatedParam1 = round((LEAF_clip(0.0f, (smoothedADC[1] + smoothedADC[9]), 1.0f) * 1.9f)) * .99f;
	//params[4] is wiggliness (amount of noise allowed through in CV)
	params[1] = LEAF_interpolation_linear(truncatedParam1, tExpSmooth_tick(&wanderSmoothers[1]), params[4]);

	intVersion = (int)params[1];
	intVersionPlusOne = (intVersion + 1) & 3;
	floatVersion = params[1] - intVersion;

	for (int i = 0; i < 16; i++)
	{
		updateFilter();
	}

*/


/*
    tVZFilter_setGain(&shelf1, fastdbtoa(-1.0f * params[1]));
    tVZFilter_setGain(&shelf2, fastdbtoa(params[1]));
    tVZFilter_setFreq(&bell1, params[3]*invOsRatio);
    tVZFilter_setGain(&bell1, fastdbtoa(params[2]));
*/
	if (codecReady)
	{
		float inputSamples[2];
		for (i = 0; i < (HALF_BUFFER_SIZE); i = i + 2)
		{
			inputSamples[0] = (audioInBuffer[buffer_offset + i]) * INV_TWO_TO_31;
			inputSamples[1] = (audioInBuffer[buffer_offset + i + 1]) * INV_TWO_TO_31;
			audioTick(inputSamples);
			audioOutBuffer[buffer_offset + i] = (int32_t)(inputSamples[0] * TWO_TO_31);
			audioOutBuffer[buffer_offset + i + 1] = (int32_t)(inputSamples[1] * TWO_TO_31);;
		}

	}


}


// CV input jacks 1-4 add to knobs 1-4.
uint64_t sampleNum = 0;
float prevInput = 0.0f;
float audioTick(float* samples)
{

	//stuff for cycle counting
	volatile uint32_t tempCount5 = 0;
	volatile uint32_t tempCount6 = 0;
	__disable_irq();
	tempCount5 = DWT->CYCCNT;

	//start of audio code
	float output = 0.0f;
	float input = samples[0];
/*
	if ((input > 0.5f) && (prevInput < 0.5f))
	{
		//we got a trigger
		tMBSampler_play(&mySamplers[currentSample]);
	}

	if (distortionMode > 0)
	{
		if ((input < -0.1f) && (prevInput > -0.1f))
		{
			//we got a trigger
			for (int i = 0; i < MAX_WAV_FILES; i++)
			{
				tMBSampler_stop(&mySamplers[i]);
			}
		}
	}
	for (int i = 0; i < 8; i++)
	{
		smoothedADC[i] = 1.0f - tExpSmooth_tick(&adcSmooth[i]);
	}

	for (int i = 8; i < 12; i++)
	{
		smoothedADC[i] = tExpSmooth_tick(&adcSmooth[i]);
	}

	currentSample = LEAF_clip(0, (int)(LEAF_clip(0.0f, smoothedADC[1]+ smoothedADC[9], 1.0f) * MAX_WAV_FILES * .99f), MAX_WAV_FILES-1);

	tMBSampler_setRate(&mySamplers[currentSample], (LEAF_clip(0.0f, smoothedADC[2]+ smoothedADC[10], 1.0f) * 4.0f));

	tExpSmooth_setDest(&sampleGains[currentSample], (LEAF_clip(0.0f, smoothedADC[0] + smoothedADC[8], 1.0f) * 2.0f));
	prevInput = input;


	for (int i = 0; i < MAX_WAV_FILES; i++)
	{
		output += 	(tMBSampler_tick(&mySamplers[i]) * tExpSmooth_tick(&sampleGains[i]));
	}
	output = tanhf(output);

	*/



	samples[0] = largeMemory[sampleNum];
	sampleNum++;
	if (sampleNum >= memoryPointer)
	{
		sampleNum = 0;
	}
	samples[1] = largeMemory[sampleNum];
	sampleNum++;
	if (sampleNum >= memoryPointer)
	{
		sampleNum = 0;
	}



   	//cycle counting stuff below. At 48k you have at most 10000 cycles per sample (when running at 480MHz). There is also overhead from the frame processing and function calls, so in reality less than that.
	tempCount6 = DWT->CYCCNT;

	cycleCountVals[0] = tempCount6-tempCount5;
	CycleCounterTrackMinAndMax(0);
	if (cycleCountVals[0] > 9900)
	{
		setLED_D(255);
		//overflow
	}
	__enable_irq();
   	return 0.0f;
}


//this keeps min and max, but doesn't do the array for averaging - a bit less expensive
void CycleCounterTrackMinAndMax( int whichCount)
{
	if (cycleCountVals[whichCount] > 0)
	{
		if ((cycleCountVals[whichCount] < cycleCountMinMax[whichCount][0]) || (cycleCountMinMax[whichCount][0] == 0))
		{
			cycleCountMinMax[whichCount][0] = cycleCountVals[whichCount];
		}
		//update max value ([2])
		if (cycleCountVals[whichCount] > cycleCountMinMax[whichCount][1])
		{
			cycleCountMinMax[whichCount][1] = cycleCountVals[whichCount];
		}
	}
}


void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
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
