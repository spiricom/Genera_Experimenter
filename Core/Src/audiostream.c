/*
 * audiostream.c
 *
 *  Created on: Aug 30, 2019
 *      Author: jeffsnyder
 */


/* Includes ------------------------------------------------------------------*/

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
#include "additive.h"
#include "vocal.h"
#include "synth.h"
#include "string3.h"
#include "string4.h"

//the audio buffers are put in the D2 RAM area because that is a memory location that the DMA has access to.
int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2_DMA;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2_DMA;


char small_memory[SMALL_MEM_SIZE];
char medium_memory[MED_MEM_SIZE] __ATTR_RAM_D1;
char large_memory[LARGE_MEM_SIZE] __ATTR_SDRAM;
tMempool mediumPool;
tMempool largePool;

HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

uint32_t codecReady = 0;

uint32_t frameCounter = 0;

volatile int stringPositions[2];
volatile int stringPositionsPrev[2];

volatile uint32_t newPluck = 0 ;
volatile uint32_t newBar = 0 ;



const uint8_t numStrings = 10; //TODO FIX THIS!

float invNumStrings = 1.0f / numStrings;

volatile uint16_t previousStringInputs[12];

//float volumeAmps128[128] = {0.000562, 0.000569, 0.000577, 0.000580, 0.000587, 0.000601, 0.000622, 0.000650, 0.000676, 0.000699, 0.000720, 0.000739, 0.000753, 0.000766, 0.000791, 0.000826, 0.000872, 0.000912, 0.000953, 0.001012, 0.001091, 0.001188, 0.001270, 0.001360, 0.001465, 0.001586, 0.001717, 0.001829, 0.001963, 0.002118, 0.002295, 0.002469, 0.002636, 0.002834, 0.003063, 0.003322, 0.003496, 0.003750, 0.004143, 0.004675, 0.005342, 0.005880, 0.006473, 0.007122, 0.007827, 0.008516, 0.009167, 0.009968, 0.010916, 0.012014, 0.012944, 0.013977, 0.015352, 0.017070, 0.019130, 0.020965, 0.022847, 0.024823, 0.026891, 0.028835, 0.030496, 0.033044, 0.036478, 0.040799, 0.045093, 0.049150, 0.053819, 0.059097, 0.064986, 0.070712, 0.076315, 0.081930, 0.087560, 0.093117, 0.098283, 0.104249, 0.111012, 0.118575, 0.124879, 0.131163, 0.141721, 0.156554, 0.175663, 0.195870, 0.213414, 0.228730, 0.241817, 0.252675, 0.264038, 0.276776, 0.290871, 0.306323, 0.322794, 0.338528, 0.353711, 0.368343, 0.382424, 0.393015, 0.406556, 0.426763, 0.453639, 0.487182, 0.522242, 0.550876, 0.573000, 0.588613, 0.598943, 0.613145, 0.628104, 0.643820, 0.660293, 0.676658, 0.692845, 0.709881, 0.727766, 0.746500, 0.764505, 0.782949, 0.802346, 0.822696, 0.844189, 0.867268, 0.886360, 0.901464, 0.912581, 0.921606, 0.932834, 0.944061};

BOOL bufferCleared = TRUE;

int numBuffersToClearOnLoad = 2;
int numBuffersCleared = 0;

float masterVolFromBrain = 0.5f;
float masterVolFromBrainForSynth = 0.25f;

volatile int firstString = 0;
volatile int lastString = 0;
volatile int lastStringPlusOne = 0;


float mtofTable[MTOF_TABLE_SIZE]__ATTR_RAM_D2;

float atoDbTable[ATODB_TABLE_SIZE]__ATTR_RAM_D2;
float dbtoATable[DBTOA_TABLE_SIZE]__ATTR_RAM_D2;


float barInMIDI[2] ;
float prevBarInMIDI[2] ;


uint8_t numStringsThisBoard = NUM_STRINGS_PER_BOARD;

tExpSmooth stringFreqSmoothers[NUM_STRINGS_PER_BOARD];

tExpSmooth volumeSmoother;
tExpSmooth knobSmoothers[20];
tExpSmooth pedalSmoothers[10];
tExpSmooth barSlideSmoother[NUM_STRINGS_PER_BOARD];
tEnvelopeFollower barNoiseSmoother[NUM_STRINGS_PER_BOARD];


uint8_t lsDecay[NUM_STRINGS_PER_BOARD];

audioFrame_t audioFrameFunction;


tCycle testSine;

//envelope tables
float decayExpBuffer[DECAY_EXP_BUFFER_SIZE];
float decayExpBufferSizeMinusOne;

LEAF leaf;

union breakFloat{
	float f;
	uint8_t b[4];
};



float prevSamp[NUM_STRINGS_PER_BOARD];

tADSRT fenvelopes[NUM_STRINGS_PER_BOARD];


tVZFilter noiseFilt;
tVZFilter noiseFilt2;

tNoise myNoise;



float audioInputs[2];

volatile uint8_t whichBar = 0;



float randomFactors[256];



volatile uint16_t stringInputs[NUM_STRINGS];
volatile uint8_t resetStringInputs = 0;

float octave;

float atodbTableScalar;
float atodbTableOffset;
float dbtoaTableScalar;
float dbtoaTableOffset;

float stringMappedPositions[NUM_STRINGS]  = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
float stringFrequencies[NUM_STRINGS] ;

float volumePedal  = 0.0f;
float knobScaled[20];
volatile uint8_t knobFrozen[20];
float pedalScaled[10];
volatile float stringMIDIPitches[NUM_STRINGS_PER_BOARD] ;


volatile int voice = 0 ;
volatile int prevVoice = 127;
int dualSlider = 0;
int neck = 0;

int currentBarBuffer = 0;
int currentPluckBuffer = 0;

int edit = 0;
int whichTable = 0;
int presetReady = 0;
uint8_t whichStringModelLoaded = NothingLoaded;





//
float stringOctave[NUM_STRINGS_PER_BOARD];



volatile float MIDIerror = 0.0f;

uint8_t interrupted = 0;

uint32_t timeFrame = 0;
float frameLoadPercentage = 0.0f;
float frameLoadMultiplier = 1.0f / (10000.0f * AUDIO_FRAME_SIZE);
uint32_t frameLoadOverCount = 0;

volatile uint16_t sampleClippedCountdown = 0;

volatile uint16_t volumePedalInt;
float octaveRatios[4] = {0.5f, 1.0f, 2.0f, 4.0f};

int octaveIndex = 1;
int stringOctaveIndex[4];

float pluckPos = 0.5f;
volatile int switchStrings = 0;
volatile uint8_t octaveAction = 0;


void __ATTR_ITCMRAM audioFrameTesting(uint16_t buffer_offset);


/**********************************************/

static float FORCE_INLINE aToDbTableLookup(float in)
{
    in = fastabsf(in);
    float floatIndex = LEAF_clip (0, (in * atodbTableScalar) - atodbTableOffset, ATODB_TABLE_SIZE_MINUS_ONE);
    uint32_t inAmpIndex = (uint32_t) floatIndex;
    uint32_t inAmpIndexPlusOne = inAmpIndex + 1;
    if (inAmpIndexPlusOne > ATODB_TABLE_SIZE_MINUS_ONE)
    {
    	inAmpIndexPlusOne = ATODB_TABLE_SIZE_MINUS_ONE;
    }
    float alpha = floatIndex - (float)inAmpIndex;
    return ((atoDbTable[inAmpIndex] * (1.0f - alpha)) + (atoDbTable[inAmpIndexPlusOne] * alpha));
}

static float FORCE_INLINE aToDbTableLookupFast(float in)
{
    in = fastabsf(in);
    uint32_t inAmpIndex = LEAF_clip (0, (in * atodbTableScalar) - atodbTableOffset, ATODB_TABLE_SIZE_MINUS_ONE);
    return atoDbTable[inAmpIndex];
}

float FORCE_INLINE dbToATableLookup(float in)
{
    float floatIndex = LEAF_clip (0, (in * dbtoaTableScalar) - dbtoaTableOffset, DBTOA_TABLE_SIZE_MINUS_ONE);
    uint32_t inDBIndex = (uint32_t) floatIndex;
    uint32_t inDBIndexPlusOne = inDBIndex + 1;
    if (inDBIndexPlusOne > DBTOA_TABLE_SIZE_MINUS_ONE)
    {
    	inDBIndexPlusOne = DBTOA_TABLE_SIZE_MINUS_ONE;
    }
    float alpha = floatIndex - (float)inDBIndex;
    return ((dbtoATable[inDBIndex] * (1.0f - alpha)) + (dbtoATable[inDBIndexPlusOne] * alpha));
}

static float FORCE_INLINE dbToATableLookupFast(float in)
{
    uint32_t inDBIndex = LEAF_clip (0, (in * dbtoaTableScalar) - dbtoaTableOffset, DBTOA_TABLE_SIZE_MINUS_ONE);
    return dbtoATable[inDBIndex];
}



float FORCE_INLINE mtofTableLookup(float tempMIDI)
{
	float tempIndexF = ((LEAF_clip(-163.0f, tempMIDI, 163.0f) * 100.0f) + 16384.0f);
	int tempIndexI = (int)tempIndexF;
	tempIndexF = tempIndexF -tempIndexI;
	float freqToSmooth1 = mtofTable[tempIndexI & 32767];
	float freqToSmooth2 = mtofTable[(tempIndexI + 1) & 32767];
	return ((freqToSmooth1 * (1.0f - tempIndexF)) + (freqToSmooth2 * tempIndexF));
}


void audioInit()
{
	// Initialize LEAF.

	LEAF_init(&leaf, SAMPLE_RATE, small_memory, SMALL_MEM_SIZE, &randomNumber);

	tMempool_init (&mediumPool, medium_memory, MED_MEM_SIZE, &leaf);
	tMempool_init (&largePool, large_memory, LARGE_MEM_SIZE, &leaf);

	leaf.clearOnAllocation = 1;
	LEAF_generate_exp(decayExpBuffer, 0.001f, 0.0f, 1.0f, -0.0008f, DECAY_EXP_BUFFER_SIZE); // exponential decay buffer falling from 1 to 0
	decayExpBufferSizeMinusOne = DECAY_EXP_BUFFER_SIZE - 1;

	for (int i = 0; i < 12; i++)
	{
		previousStringInputs[i]	= 0;
	}



	tCycle_init(&testSine, &leaf);
	tCycle_setFreq(testSine, 440.0f);

	for (int i = 0; i < 256; i++)
	{
		randomFactors[i] = (randomNumber() * 0.4f) + 0.8f;
	}
	LEAF_generate_atodb(atoDbTable, ATODB_TABLE_SIZE, 0.00001f, 1.0f);
	LEAF_generate_dbtoa(dbtoATable, DBTOA_TABLE_SIZE, -90.0f, 50.0f);

	atodbTableScalar = ATODB_TABLE_SIZE_MINUS_ONE/(1.0f-0.00001f);
	atodbTableOffset = 0.00001f * atodbTableScalar;
	dbtoaTableScalar = DBTOA_TABLE_SIZE_MINUS_ONE/(50.0f+90.0f);
	dbtoaTableOffset = -90.0f * dbtoaTableScalar;

	LEAF_generate_mtof(mtofTable, -163.8375f, 163.8375f,  MTOF_TABLE_SIZE); //mtof table for fast calc





	if (numStrings == 6)
	{
		firstString = boardNumber;
		numStringsThisBoard = 1;
	}

	else if (numStrings == 10)
	{
#if 0
		// first two strings are one board each, other 8 are two strings each.
		if (boardNumber == 0)
		{
			firstString = 0;
			numStringsThisBoard = 1;
		}
		else if (boardNumber == 1)
		{
			firstString = 1;
			numStringsThisBoard = 1;
		}
		else
		{
			firstString = (boardNumber - 1) * NUM_STRINGS_PER_BOARD;
			numStringsThisBoard = 2;
		}
#endif
		numStringsThisBoard = 1;
		// first two strings are one board each, other 8 are two strings each.
		firstString = boardNumber;

	}

	else //otherwise 12-string version
	{

		firstString = boardNumber * NUM_STRINGS_PER_BOARD;
		numStringsThisBoard = 2;

	}




	tExpSmooth_init(&volumeSmoother,0.0f, 0.0005f, &leaf);
	for (int i = 0; i < 20; i++)
	{
		tExpSmooth_init(&knobSmoothers[i],0.0f, 0.0005f, &leaf);
	}
	for (int i = 0; i < 10; i++)
	{
		tExpSmooth_init(&pedalSmoothers[i],0.0f, 0.0005f, &leaf);
	}

	for (int i = 0; i < NUM_STRINGS_PER_BOARD; i++)
	{
		tExpSmooth_init(&barSlideSmoother[i],0.000f, 0.01f, &leaf);
		tEnvelopeFollower_init(&barNoiseSmoother[i],0.0001f, 0.9993f, &leaf);
	}

	audioInitAdditive();
	//audioInitString1();
	//audioInitVocal();
	audioInitSynth();
	audioInitString3();
	audioInitString4();
	for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
	{


		tADSRT_init(&fenvelopes[v], 0.0f,  50.0f, 0.0f, 200.0f, decayExpBuffer, DECAY_EXP_BUFFER_SIZE, &leaf);


		tVZFilter_init(&noiseFilt, BandpassPeak, 1500.0f, 1.5f, &leaf);
		tVZFilter_init(&noiseFilt2, Lowpass, 800.0f, 0.9f, &leaf);
		//tVZFilter_setFreq(&noiseFilt2, 3332.0f); //based on testing with knob values


		tVZFilter_setFreq(noiseFilt, faster_mtof(0.9f * 128.0f));
		tVZFilter_setFreq(noiseFilt2,faster_mtof(0.8f * 128.0f));

		tNoise_init(&myNoise, WhiteNoise, &leaf);

	}

	for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{

			audioOutBuffer[ i] = (int32_t)(0.0f * TWO_TO_23);
	}

//messing around
	//audioFrameFunction = audioFrameTesting;
	//diskBusy = 0;
	//presetReady = 1;
		audioFrameFunction = audioFrameWaiting;
	HAL_Delay(1);

}

void audioStart(SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn)
{
	HAL_Delay(1);
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);
}

volatile uint32_t timeSPI = 0;
void __ATTR_ITCMRAM updateStateFromSPIMessage(uint8_t offset)
{
	uint32_t tempCountSPI = DWT->CYCCNT;
	int modeBit = SPI_LEVERS_RX[24 + offset];

	octaveAction = (modeBit >> 6) & 1;
	dualSlider = (modeBit >> 5) & 1;

	edit = (modeBit >> 4) & 1;
	voice = SPI_LEVERS_RX[25 + offset];



	octave = (((int32_t) (modeBit & 15) - 5 ) * 12.0f);
	//if "octave action" is set to 1, then immediately change octave instead of waiting for new note
	if (octaveAction)
	{
		for (int i = 0; i < numStringsThisBoard; i++)
		{
			stringOctave[i] = octave;
		}
	}

	volumePedalInt = ((uint16_t)SPI_LEVERS_RX[26 + offset] << 8) + ((uint16_t)SPI_LEVERS_RX[27 + offset] & 0xff);
	volumePedal = volumePedalInt * 0.0002442002442f;

	stringPositions[whichBar] = ((uint16_t)SPI_LEVERS_RX[28 + offset] << 8) + ((uint16_t)SPI_LEVERS_RX[29 + offset] & 0xff);
	if (stringPositions[whichBar] != stringPositionsPrev[whichBar])
	{
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
		stringPositionsPrev[whichBar] = stringPositions[whichBar];
		barInMIDI[0] = stringPositions[0] * 0.001953125f;
		barInMIDI[1] = stringPositions[1] * 0.001953125f;
	}
	tExpSmooth_setDest(volumeSmoother,volumePedal);
	timeSPI = DWT->CYCCNT - tempCountSPI;
}



void voiceChangeCheck(void)
{
	if (voice != prevVoice)
	{
		if (voice == 63)
		{
			audioFrameFunction = audioFrameWaiting;
			audioSwitchToString1();
			currentActivePreset = voice;
			resetStringInputs = 1;
			diskBusy = 0;
			whichModel = 1;
		}
		else if (voice == 62)
		{
			audioFrameFunction = audioFrameWaiting;
			audioSwitchToString2();
			currentActivePreset = voice;
			resetStringInputs = 1;
			diskBusy = 0;
			whichModel = 2;
		}
		else if (voice == 61)
		{
			audioFrameFunction = audioFrameAdditive;
			audioSwitchToAdditive();
			currentActivePreset = voice;
			diskBusy = 0;
			presetReady = 1;
			resetStringInputs = 1;
			whichModel = 3;
		}
		else if (voice == 60)
		{
			audioFrameFunction = audioFrameVocal;
			audioSwitchToString4();
			currentActivePreset = voice;
			diskBusy = 0;
			presetReady = 1;
			resetStringInputs = 1;
			whichModel = 4;
		}
		else if (voice == 59)
		{
			audioFrameFunction = audioFrameString3;
			audioSwitchToString3();
			currentActivePreset = voice;
			diskBusy = 0;
			presetReady = 1;
			resetStringInputs = 1;
			whichModel = 5;
		}
		else
		{
			audioFrameFunction = audioFrameWaiting;
			presetWaitingToLoad = 1;
			presetNumberToLoad = voice;
			presetReady = 0;
			if (prevVoice > 58)
			{
				resetStringInputs = 1;
			}
			frameLoadOverCount = 0;
			whichModel = 0;
		}
		for (int i = 0; i < AUDIO_BUFFER_SIZE; i+=2)
		{
			audioOutBuffer[i] = 0;
			audioOutBuffer[i + 1] = 0;
		}
	}

	prevVoice = voice;
}


volatile uint32_t timeClean = 0;
void __ATTR_ITCMRAM HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{

	//SCB_CleanInvalidateDCache_by_Addr((uint32_t*)(((uint32_t)audioOutBuffer) & ~(uint32_t)0x1F), AUDIO_BUFFER_SIZE+32);

	if ((!diskBusy)&& (presetReady))
	{
		audioFrameFunction(HALF_BUFFER_SIZE);
	}

	voiceChangeCheck();
	uint32_t tempCountClean = DWT->CYCCNT;
	SCB_CleanInvalidateDCache_by_Addr((uint32_t*)(((uint32_t)audioOutBuffer) & ~(uint32_t)0x1F), AUDIO_BUFFER_SIZE+32);
	timeClean = DWT->CYCCNT - tempCountClean;
}

void __ATTR_ITCMRAM HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	//SCB_CleanInvalidateDCache_by_Addr((uint32_t*)(((uint32_t)audioOutBuffer) & ~(uint32_t)0x1F), AUDIO_BUFFER_SIZE+32);
	if ((!diskBusy)&& (presetReady))
	{
		audioFrameFunction(0);
	}
	voiceChangeCheck();
	SCB_CleanInvalidateDCache_by_Addr((uint32_t*)(((uint32_t)audioOutBuffer) & ~(uint32_t)0x1F), AUDIO_BUFFER_SIZE+32);
}

void __ATTR_ITCMRAM HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
}

void __ATTR_ITCMRAM audioFrameTesting(uint16_t buffer_offset)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	uint32_t tempCountFrame = DWT->CYCCNT;
	//mono operation, no need to compute right channel. Also for loop iterating by 2 instead of 1 to avoid if statement.
	for (int i = 0; i < HALF_BUFFER_SIZE; i+=2)
	{
		int iplusbuffer = buffer_offset + i;
		float leftOut = tCycle_tick(testSine);
		audioOutBuffer[iplusbuffer] = (int32_t)(leftOut * TWO_TO_23);
		audioOutBuffer[iplusbuffer + 1] = (int32_t)(leftOut * -.99999f *  TWO_TO_23);
	}
	timeFrame = DWT->CYCCNT - tempCountFrame;
	frameLoadPercentage = (float)timeFrame * frameLoadMultiplier;
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}


void __ATTR_ITCMRAM audioFrameWaiting(uint16_t buffer_offset)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	uint32_t tempCountFrame = DWT->CYCCNT;
	//mono operation, no need to compute right channel. Also for loop iterating by 2 instead of 1 to avoid if statement.
	for (int i = 0; i < HALF_BUFFER_SIZE; i+=2)
	{
		int iplusbuffer = buffer_offset + i;
		audioOutBuffer[iplusbuffer] = 0;
		audioOutBuffer[iplusbuffer + 1] = 0;
	}
	timeFrame = DWT->CYCCNT - tempCountFrame;
	frameLoadPercentage = (float)timeFrame * frameLoadMultiplier;
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

