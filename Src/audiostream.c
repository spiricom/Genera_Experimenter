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

//the audio buffers are put in the D2 RAM area because that is a memory location that the DMA has access to.
int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;

void audioFrame(uint16_t buffer_offset);
float audioTickL(float audioIn);
float audioTickR(float audioIn);
float map(float value, float istart, float istop, float ostart, float ostop);
float processString(int whichString, float input);

HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;



int whichBoard = 0;


uint8_t codecReady = 0;
float sample = 0.0f;
float rightIn = 0.0f;


tSawtooth mySaw[2];
tHighpass dcBlock[2];
tEnvelopeFollower myFollower[2];

uint16_t stringPositions[2];
float stringMappedPositions[2];
float stringFrequencies[2];
float stringMIDIVersionOfFrequencies[2];
uint32_t currentMIDInotes[2];
uint32_t previousMIDInotes[2];
uint8_t stringTouchLH[2] = {0,0};
uint8_t stringTouchRH[2] = {0,0};
float openStringFrequencies[4] = {41.204f, 55.0f, 73.416f, 97.999f};
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

#define LARGE_MEM_SIZE 33554432 //32 MBytes - size of SDRAM IC
char largeMemory[LARGE_MEM_SIZE] __ATTR_SDRAM;

tMempool smallPool;
tMempool largePool;


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
	tMempool_init (&largePool, largeMemory, LARGE_MEM_SIZE);

	HAL_Delay(10);

	for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		audioOutBuffer[i] = 0;
	}

	tSawtooth_init(&mySaw);
	tEnvelopeFollower_init(&myFollower, 0.01f, 0.9999f);
	tHighpass_init(&dcBlock, 250.0f);
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

void audioFrame(uint16_t buffer_offset)
{
	int i;

	//if the codec isn't ready, keep the buffer as all zeros
	//otherwise, start computing audio!

	if (codecReady)
	{

		for (i = 0; i < (HALF_BUFFER_SIZE); i = i + 2)
		{
			audioOutBuffer[buffer_offset + i] = (int32_t)(audioTickR((float) (audioInBuffer[buffer_offset + i] << 8) * INV_TWO_TO_31) * TWO_TO_23);
			audioOutBuffer[buffer_offset + i + 1] = (int32_t)(audioTickL((float) (audioInBuffer[buffer_offset + i + 1] << 8) * INV_TWO_TO_31) * TWO_TO_23);
		}

	}
}




float audioTickL(float audioIn)
{
	sample = processString(0, audioIn);
	return sample;
}

float audioTickR(float audioIn)
{
	sample = processString(1, audioIn);
	return sample;
}


float processString(int whichString, float input)
{
	whichString = whichString + whichBoard;
	stringPositions[whichString] =  ((uint16_t)SPI_RX[whichString * 2] << 8) + ((uint16_t)SPI_RX[(whichString * 2) + 1] & 0xff);
	if (stringPositions[whichString] == 65535)
	{
		stringFrequencies[whichString] = openStringFrequencies[whichString];
		//stringMIDIVersionOfFrequencies[whichString] = LEAF_frequencyToMidi(stringFrequencies[whichString]);
	}
	else
	{
		stringMappedPositions[whichString] = map((float)stringPositions[whichString], fretMeasurements[whichString][1], fretMeasurements[whichString][2], fretScaling[1], fretScaling[2]); //scale length of measurement from 12th fret to 7th fret to be string length of .5 to .666666
		stringFrequencies[whichString] = (1.0 / (2.0f * stringMappedPositions[whichString])) * openStringFrequencies[whichString] * 2.0f;
		///-stringMIDIVersionOfFrequencies[whichString] = LEAF_frequencyToMidi(stringFrequencies[whichString]);
	}

	tSawtooth_setFreq(&mySaw[whichString], stringFrequencies[whichString]);
	return tSawtooth_tick(&mySaw[whichString]) * tEnvelopeFollower_tick(&myFollower[whichString], tHighpass_tick(&dcBlock[whichString], input));
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
