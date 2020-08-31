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

#define NUM_COUNTER_CYCLES_TO_AVERAGE 1024
volatile int64_t cycleCountVals[4][3];
volatile int64_t cycleCountValsAverager[4][NUM_COUNTER_CYCLES_TO_AVERAGE];
volatile uint16_t cycleCountAveragerCounter[4] = {0,0,0,0};
float cycleCountAverages[4][3];

uint8_t codecReady = 0;



//MEMPOOLS
char smallMemory[SMALL_MEM_SIZE];
char mediumMemory[MEDIUM_MEM_SIZE] __ATTR_RAM_D1;
char largeMemory[LARGE_MEM_SIZE] __ATTR_SDRAM;
//small memory will be the default LEAF mempool
// need to create custom mempools for the medium and large memory
tMempool mediumPool;
tMempool largePool;


/**********************************************/

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;



//10 distortion tanh
int distortionMode = 0;
tVZFilter bell1, shelf1, shelf2;
tVZFilter lp1;

tExpSmooth adcSmooth[NUM_ADC_CHANNELS];

void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn)
{
	// Initialize LEAF.

	LEAF_init(SAMPLE_RATE, AUDIO_FRAME_SIZE, smallMemory, SMALL_MEM_SIZE, &randomNumber);

	tMempool_init (&mediumPool, mediumMemory, MEDIUM_MEM_SIZE);
	tMempool_init (&largePool, largeMemory, LARGE_MEM_SIZE);

	HAL_Delay(10);
	leaf.clearOnAllocation = 1;

	tVZFilter_init(&shelf1, Lowshelf, 80.0f , 6.0f);
	tVZFilter_init(&shelf2, Highshelf, 12000.0f , 6.0f);
	tVZFilter_init(&bell1, Bell, 1000.0f , 1.9f);

	tVZFilter_init(&lp1, Lowpass, 18000.0f, 0.8f);

	for(int i = 0; i < NUM_ADC_CHANNELS; i++)
	{
		tExpSmooth_init(&adcSmooth[i], 0.0f, 0.1f);
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

int frameCount = 0;

float smoothedADC[NUM_ADC_CHANNELS];

float params[8];

void audioFrame(uint16_t buffer_offset)
{


	int i;
	buttonCheck();
	for (int i = 0; i < NUM_ADC_CHANNELS; i++)
	{
		tExpSmooth_setDest(&adcSmooth[i], (ADC_values[i] * INV_TWO_TO_16));
	}

	for (int i = 0; i < 8; i++)
	{
		smoothedADC[i] = 1.0f - tExpSmooth_tick(&adcSmooth[i]);
	}

	for (int i = 8; i < 12; i++)
	{
		smoothedADC[i] = tExpSmooth_tick(&adcSmooth[i]);
	}





	params[0] = LEAF_clip(0.0f,(((smoothedADC[0] + smoothedADC[8]) * 20.0f) + 1.0f), 40.0f);
    params[1] = LEAF_clip(-30.0f, ((smoothedADC[1] + smoothedADC[9]) * 30.0f) - 15.0f, 30.0f);
    params[2] = LEAF_clip(-34.0f, ((smoothedADC[2] + smoothedADC[10]) * 34.0f) - 17.0f, 34.0f);
    params[3] = LEAF_clip(10.0f, faster_mtof((smoothedADC[3] + smoothedADC[11]) * 77.0f + 42.0f), 18000.0f);
    params[4] = smoothedADC[4];



    tVZFilter_setGain(&shelf1, fastdbtoa(-1.0f * params[1]));
    tVZFilter_setGain(&shelf2, fastdbtoa(params[1]));
    tVZFilter_setFreq(&bell1, params[3]);
    tVZFilter_setGain(&bell1, fastdbtoa(params[2]));



	frameCount++;
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


float audioTick(float* samples)
{

	//stuff for cycle counting
	volatile uint32_t tempCount5 = 0;
	volatile uint32_t tempCount6 = 0;
	__disable_irq();
	tempCount5 = DWT->CYCCNT;

	//start of audio code
	float sample = samples[0];
	sample = tVZFilter_tick(&lp1, sample);
    sample = sample * params[0];

    //button B sets distortion mode
   	if (distortionMode > 0)
   	{
   		sample = LEAF_shaper(sample, 1.0f);
   	}
   	else
   	{
   		sample = tanhf(sample);
   	}

   	sample= tVZFilter_tick(&shelf1, sample); //put it through the low shelf
   	sample = tVZFilter_tick(&shelf2, sample); // now put that result through the high shelf
   	sample = tVZFilter_tick(&bell1, sample); // now add a bell (or peaking eq) filter

   	sample = tanhf(sample * 0.9f) * params[4];

   	samples[0] = sample;
   	samples[1] = sample;


   	//cycle counting stuff below. At 192k you have at most 2500 cycles per sample (when running at 480MHz). There is also overhead from the frame processing and function calls, so in reality less than that.
	tempCount6 = DWT->CYCCNT;

	//cycleCountVals[0][2] = 0;

	cycleCountVals[0][1] = tempCount6-tempCount5;
	if (cycleCountVals[0][1] > 2500)
	{
		setLED_D(255);
		//overflow
	}
	__enable_irq();
   	return 0.0f;
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
