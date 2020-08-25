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

#define FILTER_ORDER 12
#define LHMUTE_COUNTLIM 100
#define RHMUTE_COUNTLIM 1

float atodbTable[ATODB_TABLE_SIZE];


//the audio buffers are put in the D2 RAM area because that is a memory location that the DMA has access to.
int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t ADC_values[NUM_ADC_CHANNELS * ADC_BUFFER_SIZE] __ATTR_RAM_D2;


void audioFrame(uint16_t buffer_offset);
float audioTick(void);
float map(float value, float istart, float istop, float ostart, float ostop);
float processString(int whichString, float input);
void attackDetectMedian(int whichString, float input);
void attackDetect2(int whichString, int tempInt);

void attackDetectAH_init(void);


HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

#define NUM_COUNTER_CYCLES_TO_AVERAGE 1024
volatile int64_t cycleCountVals[4][3];
volatile int64_t cycleCountValsAverager[4][NUM_COUNTER_CYCLES_TO_AVERAGE];
volatile uint16_t cycleCountAveragerCounter[4] = {0,0,0,0};
float cycleCountAverages[4][3];

uint8_t codecReady = 0;
float sample = 0.0f;
float rightIn = 0.0f;

#define NUM_SAWTOOTHS 4
tSVF myLowpass[NUM_STRINGS];
tSawtooth mySaw[NUM_STRINGS][NUM_SAWTOOTHS];
tCycle mySine[NUM_STRINGS];
tHighpass dcBlock[NUM_STRINGS];
tEnvelopeFollower myFollower[NUM_STRINGS];
tExpSmooth pitchSmoother[NUM_STRINGS];
tMedianFilter median[NUM_STRINGS];
tNoise noise[NUM_STRINGS];
tThreshold threshold[NUM_STRINGS];
tADSR envelope[NUM_STRINGS];
tSlide fastSlide[NUM_STRINGS];
tSlide slowSlide[NUM_STRINGS];
tSVF lowpass[NUM_STRINGS];
tRampUpDown updownRamp[NUM_STRINGS];

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

float noiseFloor = -40.0f;
float status[NUM_STRINGS] = {0.0f, 0.0f, 0.0f, 0.0f};
int outOfThreshPositiveChange[NUM_STRINGS] = {0,0,0,0};
int delayCounter[NUM_STRINGS] = {0,0,0,0};
float currentMaximum[NUM_STRINGS] = {-120.f,-120.f, -120.f, -120.f};
int sahArmed[NUM_STRINGS] = {0,0,0,0};
int previousOutOfThresh[NUM_STRINGS] = {0,0,0,0};
int attackDelay = 400;
int offLockout[NUM_STRINGS] = {0,0,0,0};
int offLockoutDelay = 1000;
float finalFreqs[NUM_STRINGS];
int noteOnHappened[NUM_STRINGS] = {0,0,0,0};
int noteOffHappened[NUM_STRINGS] = {0,0,0,0};
float noteOnAmplitude[NUM_STRINGS] = {0.0f, 0.0f, 0.0f, 0.0f};
int outOfThresh[NUM_STRINGS] = {0,0,0,0};

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


tHighpass opticalHighpass[NUM_STRINGS * FILTER_ORDER];
tVZFilter opticalLowpass[NUM_STRINGS * FILTER_ORDER];

tExpSmooth gainSmoothed;

int waitTimeOver = 0;

//MEMPOOLS
#define SMALL_MEM_SIZE 10000
char smallMemory[SMALL_MEM_SIZE];

#define MEDIUM_MEM_SIZE 400000
char mediumMemory[MEDIUM_MEM_SIZE] __ATTR_RAM_D1;


char largeMemory[LARGE_MEM_SIZE] __ATTR_SDRAM;
//char largeMemory[LARGE_MEM_SIZE] __ATTR_RAM_D1;
tMempool mediumPool;
tMempool largePool;


/**********************************************/

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;


void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn)
{
	// Initialize LEAF.

	LEAF_init(SAMPLE_RATE, AUDIO_FRAME_SIZE, smallMemory, SMALL_MEM_SIZE, &randomNumber);

	tMempool_init (&mediumPool, mediumMemory, MEDIUM_MEM_SIZE);
	tMempool_init (&largePool, largeMemory, LARGE_MEM_SIZE);

	HAL_Delay(10);

	for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		audioOutBuffer[i] = 0;
	}

	for (int i = 0; i < NUM_ADC_CHANNELS * FILTER_ORDER; i++)
	{
		tHighpass_init(&opticalHighpass[i], 100.0f);
		tVZFilter_init(&opticalLowpass[i], Lowpass, 1000.0f, 0.6f);
	}
	LEAF_generate_atodb(atodbTable, ATODB_TABLE_SIZE);

	tSimplePoly_init(&poly,1);
	tSimplePoly_setNumVoices(&poly,1);
	poly->recover_stolen = 0;
	tExpSmooth_init(&gainSmoothed, 0.0f, 0.01f);
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

/*
	for (int i = 0; i < NUM_STRINGS; i++)
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
		tThreshold_init(&threshold[i],0.5f, 8.0f);
		tADSR_init(&envelope[i], 6.0f, 100.0f, 0.6f, 20.0f);
		tADSR_setLeakFactor(&envelope[i], 0.99998f);
		tSlide_init(&fastSlide[i],1.0f,1110.0f);
		tSlide_init(&slowSlide[i],500.0f,1.0f);
		tSVF_init(&lowpass[i], SVFTypeLowpass, 4000.0f, 1.0f);
		tRampUpDown_init(&updownRamp[i], 0.0f, 104.0f, 1); //5000 samples should be 104 ms
	}
*/

	attackDetectAH_init();

	HAL_Delay(1);

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
			//inputSamples[1] = (audioInBuffer[buffer_offset + i] << 8) * INV_TWO_TO_31;
			//inputSamples[0] = (audioInBuffer[buffer_offset + i + 1] << 8) * INV_TWO_TO_31;

			//audioOutBuffer[buffer_offset + i] = (int32_t)(audioTick() * TWO_TO_23);
			//audioOutBuffer[buffer_offset + i + 1] = 0;
		}

	}
}
float intoThresh1 = 0.0f;

int sampleNumGlobal = 0;


int distanceBetweenReadAndWrite = 0;
float audioTick()
{
	sample = 0.0f;
/*
	if (ADC_Ready)
	{
		//distanceBetweenReadAndWrite = currentADCBufferPos - sampleNumGlobal;
		//if (distanceBetweenReadAndWrite < 0)
		//{
		//	distanceBetweenReadAndWrite += ADC_RING_BUFFER_SIZE;
		//}
		for (int i = 0; i < NUM_STRINGS; i++)
		{
			//float tempSamp = audioADCInputs[i][sampleNumGlobal];
			//attackDetect2(i, tempSamp);
			//processString(i, tempSamp);
			float tempSample = 0.0f;
			//sample += tNoise_tick(&noise[whichString]);// * tADSR_tick(&envelope[whichString]);
			float theEnv = tADSR_tick(&envelope[i]);
			for (int j = 0; j < NUM_SAWTOOTHS; j++)
			{
				tempSample += tSawtooth_tick(&mySaw[i][j]) * theEnv;
			}
			//tempSample += tCycle_tick(&mySine[i]) * theEnv;
			//tempSample = tSVF_tick(&myLowpass[i], tempSample);
			//}
			sample += tempSample;
			//sample +=  SFXRhodesTick(whichString);
		}
		sampleNumGlobal++;
		//do this as a mask instead to protect the data from COVID
		if (sampleNumGlobal >= ADC_RING_BUFFER_SIZE)
		{
			sampleNumGlobal = 0;
		}

	}
*/

	/*
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


		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, outOfThreshPositiveChange[1] & 1);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, sahArmed[1] & 1);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, outOfThresh[1] & 1);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, stringTouchRH[1] & 1);


		//sample = tanhf(sample * 1.6f);
		//sample = LEAF_clip(-1.0f, dbtoa(intoThresh1) * 0.5f, 1.0f);
		//sample = 0.0f;
		//sample = smoothed;

		//sample = tRampUpDown_sample(&updownRamp[0]);
		//sample = powf(sample, 0.5f);
		//sample += outOfThreshPositiveChange[0];
		//sample -= noteOnHappened[0];
	}
	*/
/*
	float tempSample = 0.0f;
	//sample += tNoise_tick(&noise[whichString]);// * tADSR_tick(&envelope[whichString]);

	tExpSmooth_setDest(&gainSmoothed, tSimplePoly_getVelocity(&poly, 0) * 0.2f);
	int myString = tSimplePoly_getPitch(&poly, 0);
	float theFreq = stringFreqs[myString];
	for (int i = 0; i < NUM_SAWTOOTHS; i++)
	{
		tSawtooth_setFreq(&mySaw[0][i], theFreq * detuneAmounts[i]);
	}

	//float theEnv = tExpSmooth_tick(&gainSmoothed);

	float theEnv = tADSR_tick(&envelope[0]);
	for (int j = 0; j < NUM_SAWTOOTHS; j++)
	{
		tempSample += tSawtooth_tick(&mySaw[0][j]) * theEnv;
	}
	sample = tempSample;
	sample = tanhf(sample);
	*/
	return sample;

}


// ALGORITHM PARMETERS
#define MAX_SAMPLES_STILL_SAME_PLUCK 400
#define MAX_VAR_DIFF_WIDTH 10000
#define MAX_WIDTH_IS_RESONATING 1000
#define MAX_RATIO_VALUE_DIFFS 0.2
#define MIN_VALUE_SPREAD 500
#define MIN_SAME_DIR_STEPS 150
#define SMOOTHING_WINDOW 16
#define SUPER_SMOOTHING_WINDOW 128
#define ENVELOPE_WINDOW 400

//INITIALIZE VARIABLES
int current_dir = 1;
int envelope_min = 65535;//maybe could be 32 bit?
int envelope_max = 0;//maybe could be 32 bit?
int prior_smoothed = 0;
int prior_super_smoothed = 0;
int prior_super_smoothed_dir = 1;
int prior_dirs[3] = {1,1,1};
int prior_changepoints_index[5] = {0,0,0,0,0};//   	# empty vector of length 5
int prior_changepoints_value[5] = {0,0,0,0,0};//		# empty vector of length 5
int prior_detect_1_index = 0;
int prior_detect_1_value = 0;
int prior_detect_2_index = 0;
int prior_detect_2_value = 0;
int prior_detect_3_index = 0;
int prior_detect_3_value = 0;
uint midpoint_estimate = 48552;
int delay_since_last_detect = 0;
int dir_count = 0;
int ready_for_pluck = TRUE;
uint64_t Pindex = 1;
int totalNumChangepoints = 0;

// SET UP THE HISTORICAL DATA
// 	I have these rounded to the nearest integer, though you don't have to round them
//	In the beginning it’s fine to pad these with some default value until they fill with actual data samples
uint smoothed = 0;//Mean of the last [SMOOTHING_WINDOW] samples
uint smoothedAccum = 0;
uint super_smoothed = 0;//Mean of the last [SUPER_SMOOTHING_WINDOW] smoothed values
uint super_smoothedAccum = 0;


tRingBufferInt smoothed_array;
tRingBufferInt super_smoothed_array;
tRingBufferInt last400_smoothed; // making this 512 for now to use ringbuffer object


volatile int pluck_strength = 0;

//this keeps min and max, but doesn't do the array for averaging - a bit less expensive
void CycleCounterTrackMinAndMax( int whichCount)
{
	if ((cycleCountVals[whichCount][2] == 0) && (cycleCountVals[whichCount][1] > 0)) //the [2] spot in the array will be set to 1 if an interrupt happened during the cycle count -- need to set that in any higher-priority interrupts to make that true
	{
		if ((cycleCountVals[whichCount][1] < cycleCountAverages[whichCount][1]) || (cycleCountAverages[whichCount][1] == 0))
		{
			cycleCountAverages[whichCount][1] = cycleCountVals[whichCount][1];
		}
		//update max value ([2])
		if (cycleCountVals[whichCount][1] > cycleCountAverages[whichCount][2])
		{
			cycleCountAverages[whichCount][2] = cycleCountVals[whichCount][1];
		}
	}
}

//these are expensive but give an average of several counts
void CycleCounterAddToAverage( int whichCount)
{
	if ((cycleCountVals[whichCount][2] == 0) && (cycleCountVals[whichCount][1] > 0)) //the [2] spot in the array will be set to 1 if an interrupt happened during the cycle count -- need to set that in any higher-priority interrupts to make that true
	{
		cycleCountValsAverager[whichCount][cycleCountAveragerCounter[whichCount]] = cycleCountVals[whichCount][1];
	}
	else
	{
		cycleCountValsAverager[whichCount][cycleCountAveragerCounter[whichCount]] = -1;
	}
	cycleCountAveragerCounter[whichCount]++;
	if (cycleCountAveragerCounter[whichCount] >= NUM_COUNTER_CYCLES_TO_AVERAGE)
	{
		cycleCountAveragerCounter[whichCount]  = 0;
	}
}

//these are expensive but give an average of several counts
void CycleCounterAverage( int whichCount)
{
	float totalCycles = 0.0f;
	float numberOfCountedSamples = 0.0f;
	for (int i = 0; i < NUM_COUNTER_CYCLES_TO_AVERAGE; i++)
	{
		if (cycleCountValsAverager[whichCount][i] >= 0) //check if the count is valid (not interrupted by an interrupt)
		{
			totalCycles += cycleCountValsAverager[whichCount][i];

			//update min value ([1])
			if ((cycleCountValsAverager[whichCount][i] < cycleCountAverages[whichCount][1]) || (cycleCountAverages[whichCount][1] == 0))
			{
				cycleCountAverages[whichCount][1] = cycleCountValsAverager[whichCount][i];
			}
			//update max value ([2])
			if (cycleCountValsAverager[whichCount][i] > cycleCountAverages[whichCount][2])
			{
				cycleCountAverages[whichCount][2] = cycleCountValsAverager[whichCount][i];
			}
			numberOfCountedSamples++;
		}
	}
	if (numberOfCountedSamples > 0.0f)
	{
		cycleCountAverages[whichCount][0] = totalCycles / numberOfCountedSamples;
	}
	else
	{
		cycleCountAverages[whichCount][0] = 0.0f;
	}

}

int sign(int x) {
    return (x > 0) - (x < 0);
}

struct AVLNode *root = NULL;
char AVL_array[12288] __ATTR_RAM_D1;

void attackDetectAH_init(void)
{
	tRingBufferInt_initToPool(&smoothed_array, SMOOTHING_WINDOW, &mediumPool);

	tRingBufferInt_initToPool(&super_smoothed_array, SUPER_SMOOTHING_WINDOW, &mediumPool);

	tRingBufferInt_initToPool(&last400_smoothed, 512, &mediumPool); // making this 512 for now to use ringbuffer object

	AVL_memory_offset = (int)&AVL_array;
}





uint myMin = 65535;
uint myMax = 0;
volatile int oldValue = 0;
void attackDetectAH(int whichString, int input)
{
	//UPDATE THE HISTORICAL SMOOTHED DATA
	volatile uint32_t tempCount5 = 0;
	volatile uint32_t tempCount6 = 0;
	__disable_irq();
	tempCount5 = DWT->CYCCNT;

	//update smoothed for current sample
	tRingBufferInt_push(&smoothed_array, input);

	//get the smoothed mean of that array
	int oldSmoothed = tRingBufferInt_getOldest(&smoothed_array);
	smoothedAccum -= oldSmoothed;
	smoothedAccum += input;
	smoothed = smoothedAccum >> 4; // divide by 16

	//update super_smoothed for current sample
	tRingBufferInt_push(&super_smoothed_array, input);

	//get the smoothed mean of that array
	int oldSuperSmoothed = tRingBufferInt_getOldest(&super_smoothed_array);
	super_smoothedAccum -= oldSuperSmoothed;
	super_smoothedAccum += input;
	super_smoothed = super_smoothedAccum >> 7; // divide by 128


	//update last400_smoothed

	oldValue = tRingBufferInt_getOldest(&last400_smoothed);

	///here's where we do the AVL tree implementation:
	//1. create a tree
	//3. delete and insert to move window
	//2. get maximum and minimum from BST
	tRingBufferInt_push(&last400_smoothed, smoothed);
/*
	root = deleteNode(root, oldValue);
	root = insert(root, smoothed);

	myMin = minValueKey(root);
	myMax = maxValueKey(root);
*/


	if ((oldValue >= myMax) || (oldValue <= myMin))
	{
		tRingBufferInt_push(&last400_smoothed, smoothed);
		myMax = 0;
		myMin = 65535;
		for (int i = 0; i < 512; i++)
		{

			int tempVal = tRingBufferInt_get(&last400_smoothed, i);
			if (tempVal < myMin)
			{
				myMin = tempVal;
			}
			if (tempVal > myMax)
			{
				myMax = tempVal;
			}
		}

	}
	else
	{
		tRingBufferInt_push(&last400_smoothed, smoothed);
		if (smoothed > myMax)
		{
			myMax = smoothed;
		}
		if (smoothed < myMin)
		{
			myMin = smoothed;
		}
	}


	//CHECK IF WE FALL OUTSIDE THE CURRENT "ENVELOPE"
	//TODO: need to get the max and min of that last400 ring buffer. For now I'm doing very slow naive version.



/*
	for (int i = 0; i < 512; i++)
	{
		int tempVal = tRingBufferInt_get(&last400_smoothed, i);
		if (tempVal < myMin)
		{
			myMin = tempVal;
		}
		if (tempVal > myMax)
		{
			myMax = tempVal;
		}
	}
*/
	int outside_envelope = ((myMin < envelope_min) || (myMax > envelope_max));



	//COLLECT THE DIRECTION OF MOVEMENT FOR SUPER-SMOOTHED SEQUENCE (FOR DETECTING IF READY FOR NEXT PLUCK)
	//Here we're basically counting how many times we've taken consecutive steps in the same direction
	//If we move in a different direction (up or down) then it resets
	//This is helpful for detecting that movement up or down right at the start of a pluck signal

	int super_smoothed_dir = sign(super_smoothed - prior_super_smoothed);
	if (super_smoothed_dir != prior_super_smoothed_dir)
	{
		dir_count = 0;
	}
	else
	{
	    dir_count = dir_count + 1;
	}

	//CHECK IF WE SEE THE SIGNS THAT WE ARE READY FOR NEXT PLUCK
	//We are ready for a new pluck if we've both:
	//(1) seen enough steps in same direction, and
	//(2) moved outside our current envelope

	if (ready_for_pluck==FALSE)
	{
		if ((dir_count > MIN_SAME_DIR_STEPS) && (outside_envelope==TRUE))
		{
			ready_for_pluck = TRUE;
		}
	}

	//COLLECT THE DIRECTION OF MOVEMENT FOR SMOOTHED SEQUENCE (FOR CHANGEPOINT DETECTION)
	int direction = sign(smoothed-prior_smoothed);
	//prior_dirs = c(prior_dirs[-1],direction); //Update by removing first element and adding new value to end
	prior_dirs[0] = prior_dirs[1];
	prior_dirs[1] = prior_dirs[2];
	prior_dirs[2] = direction;


	//BEGIN TO CHECK IF WE ARE AT A "CHANGEPOINT"
	//A changepoint is when both of the following are true:
	// 		(1) Several consistent steps all up (or all down) in sequence, and then suddenly a change
	// 		(2) There is enough overall vertical movement in the recent samples

	int tempMin = 1;
	int tempMax = -1;
	for (int i = 0; i < 3; i++)
	{
		if (prior_dirs[i] < tempMin)
		{
			tempMin = prior_dirs[i];
		}
		if (prior_dirs[i] > tempMax)
		{
			tempMax = prior_dirs[i];
		}
	}
	if (((current_dir == 1) && (tempMax == -1)) ||((current_dir == -1) && (tempMin == 1)))
	{
		//UPDATE THE DIRECTION THAT WE'll BE COMPARING AGAINST NEXT TIME
		current_dir = -current_dir;

		if (totalNumChangepoints < 5)
		{
			totalNumChangepoints++;
		}

		//UPDATE VECTORS THAT STORE THE LAST 5 CHANGEPOINTS

		prior_changepoints_index[0] = prior_changepoints_index[1];
		prior_changepoints_index[1] = prior_changepoints_index[2];
		prior_changepoints_index[2] = prior_changepoints_index[3];
		prior_changepoints_index[3] = prior_changepoints_index[4];
		prior_changepoints_index[4] = Pindex;


		prior_changepoints_value[0] = prior_changepoints_value[1];
		prior_changepoints_value[1] = prior_changepoints_value[2];
		prior_changepoints_value[2] = prior_changepoints_value[3];
		prior_changepoints_value[3] = prior_changepoints_value[4];
		prior_changepoints_value[4] = smoothed;


		//ONCE THERE HAVE BEEN AT LEAST THREE CHANGEPOINTS
		//I'm doing this as 5 so I don't need to check any NULL values
	    if (totalNumChangepoints >= 5)
	    {

	    	//COMPUTE NUMBER OF SAMPLES BETWEEN EACH CHANGEPOINT
			//### 	Eg. if prior_changepoints_index = [NULL,NULL,40,60,90] then
			//###		width_differences = [NULL,NULL,20,30]
	    	int width_differences[4];
	    	width_differences[0] = prior_changepoints_index[1] - prior_changepoints_index[0];
	    	width_differences[1] = prior_changepoints_index[2] - prior_changepoints_index[1];
	    	width_differences[2] = prior_changepoints_index[3] - prior_changepoints_index[2];
	    	width_differences[3] = prior_changepoints_index[4] - prior_changepoints_index[3];

	    	//= vector of incremental differences between values in prior_changepoints_index

			//### COMPUTE THE VALUE DEVIATIONS FROM THE MIDPOINT (ONLY IF THE MIDPOINT IS NON-NULL)
			//### 	Eg. if prior_changepoints_value = [NULL,NULL,100,200,300] and midpoint_estimate = 150
			//###     	then dirs_from_midpoint = [NULL,NULL,-1,1,1]
	    	int dirs_from_midpoint[5];
	    	for (int i = 0; i < 5; i++)
	    	{
	    		dirs_from_midpoint[i] = sign(prior_changepoints_value[i] - midpoint_estimate);
	    	}

			//### ASSEMBLE STATISTICS RELATED TO A 3-POINT PATTERN (UP/DOWN/UP or vice versa)
	    	int tempZeroCheck = abs(prior_changepoints_value[4] - prior_changepoints_value[3]);
	    	if (tempZeroCheck == 0)
	    	{
	    		tempZeroCheck = 1;
	    	}
			float ratio_value_diffs_1 = ((float)abs(prior_changepoints_value[4] - prior_changepoints_value[2])) / (float)tempZeroCheck;
			int spread_value_1 = abs(prior_changepoints_value[4] - prior_changepoints_value[3]);
			int falls_about_midpoint_1 = ((dirs_from_midpoint[2] == dirs_from_midpoint[4]) && (dirs_from_midpoint[3] != dirs_from_midpoint[4]));


	    	tempZeroCheck = abs(prior_changepoints_value[4] - prior_changepoints_value[3]);
	    	if (tempZeroCheck == 0)
	    	{
	    		tempZeroCheck = 1;
	    	}

			float ratio_value_diffs_2 = ((float)abs(prior_changepoints_value[4] - prior_changepoints_value[0])) / (float)tempZeroCheck;
			int spread_value_2 = abs(prior_changepoints_value[0] - prior_changepoints_value[1]);
			int falls_about_midpoint_2 = ( (dirs_from_midpoint[0] == dirs_from_midpoint[4]) && (dirs_from_midpoint[0] != dirs_from_midpoint[1]) && (dirs_from_midpoint[0] != dirs_from_midpoint[3]));



			//### CHECK WHETHER WE ARE IN A DETECTION PATTERN
			//### 	All of the following must be satisfied in order to be a detected event:
			//### 		(1) outer two values are much closer to each other than they are to middle value
			//### 		(2) distance between first and second values is far enough apart
			//### 		(3) differences in widths are consistently spaced
			//### 		(4) differences in widths are small enough that it could be a vibration signal
			//### 		(5) points fall on the correct sides of the midpoint estimate for the given pattern


			//### 3-POINT PATTERN
			//### NOTE: var() here means the "sample variance". Tell me if you need help with it.
			//### See link: https://www.mathsisfun.com/data/standard-deviation.html

			//compute var of width differences using just elements [2] and [3]
			//first take the mean
			int tempMean = (width_differences[2] + width_differences[3]) / 2; //divide by 2
			int tempVar1 = width_differences[2] - tempMean;
			int tempVar2 = width_differences[3] - tempMean;
			int tempVariance = ((tempVar1 * tempVar1) + (tempVar2 * tempVar2)) / 2; // divide by 2;

			tempMax = width_differences[2];
			if (width_differences[3] > tempMax)
			{
				tempMax = width_differences[3];
			}

			int firstTest = (ratio_value_diffs_1 < MAX_RATIO_VALUE_DIFFS) && (spread_value_1 > MIN_VALUE_SPREAD) && (tempVariance < MAX_VAR_DIFF_WIDTH) && (tempMax < MAX_WIDTH_IS_RESONATING) && (falls_about_midpoint_1==TRUE);


			//### 5-POINT PATTERN

			//compute var of width differences using all elements
			//first take the mean
			tempMean = (width_differences[0] + width_differences[1] + width_differences[2] + width_differences[3] + width_differences[4]) / 5;
			tempVar1 = width_differences[0] - tempMean;
			tempVar2 = width_differences[1] - tempMean;
			int tempVar3 = width_differences[2] - tempMean;
			int tempVar4 = width_differences[3] - tempMean;
			int tempVar5 = width_differences[4] - tempMean;
			tempVariance = ((tempVar1 * tempVar1) + (tempVar2 * tempVar2) + (tempVar3 * tempVar3) + (tempVar4 * tempVar4) + (tempVar5 * tempVar5)) / 5; // divide by 5;

			tempMax = width_differences[0];
			for (int i = 1; i < 5; +i++)
			if (width_differences[i] > tempMax)
			{
				tempMax = width_differences[i];
			}

			int secondTest = (ratio_value_diffs_2 < MAX_RATIO_VALUE_DIFFS) && (spread_value_2 > MIN_VALUE_SPREAD) && (tempVariance < MAX_VAR_DIFF_WIDTH) && (tempMax < MAX_WIDTH_IS_RESONATING) && (falls_about_midpoint_2==TRUE);

			if (firstTest || secondTest)
			{
				//### UPDATE THE ENVELOPE
				envelope_min = myMin;
				envelope_max = myMax;
				int is_pluck;

				//### CHECK IF THIS IS A NEW PLUCK (NOT JUST FURTHER DETECTION OF RESONANCE ON EXISTING PLUCK)
				//### 	If it is an actual pluck, then also collect its strength
				if (ready_for_pluck==TRUE)
				{
					is_pluck = (delay_since_last_detect > MAX_SAMPLES_STILL_SAME_PLUCK);
					if (is_pluck==TRUE)
					{
						tempCount6 = DWT->CYCCNT;

						cycleCountVals[0][2] = 0;

						cycleCountVals[0][1] = tempCount6-tempCount5;
						pluck_strength = envelope_max - envelope_min;

						//adding this - not sure if it's what Angie meant:
						//ready_for_pluck = FALSE;
						// TODO: notify of a pluck event
					}
				}
				else
				{
					is_pluck = FALSE;
				}

				//### IF WE HAVE HAD AT LEAST THREE DETECTIONS OF RESONANCE WITHIN THE SAME PLUCK'S SIGNAL
				//### THEN WE CAN COMPUTE OR UPDATE THE MIDPOINT ESTIMATE
				if (prior_detect_3_index > 0)
				{
					if ((Pindex - prior_detect_1_index < MAX_SAMPLES_STILL_SAME_PLUCK) && (prior_detect_1_index - prior_detect_2_index < MAX_SAMPLES_STILL_SAME_PLUCK) && (prior_detect_2_index - prior_detect_3_index < MAX_SAMPLES_STILL_SAME_PLUCK))
					{
						//### Note: This can be rounded to the nearest int, but doesn't need to be
						midpoint_estimate = (myMax + myMin) >> 1;
					}
				}

				//### RESET THE DELAY SINCE LAST DETECTION
				delay_since_last_detect = 0;

				//### IF THIS DETECTION WAS A PLUCK, THEN WE ARE NOT READY FOR ANOTHER PLUCK YET
				///?????  that comment doesn't really match this code - am I supposed to set is_first_pluck_signal to be true after the first pluck but false thereafter? or is this supposed to be is_pluck? in which case it could just go above when is_pluck is set to TRUE

				if (is_pluck)
				{
						ready_for_pluck = FALSE;
				}

				//### UPDATE THE INFORMATION FOR THE PRIOR THREE DETECT EVENTS
				prior_detect_3_index = prior_detect_2_index;
				prior_detect_3_value = prior_detect_2_value;
				prior_detect_2_index = prior_detect_1_index;
				prior_detect_2_value = prior_detect_1_value;
				prior_detect_1_index = Pindex;
				prior_detect_1_value = smoothed;

			}
	    }

	}

	//### INCREMENT THE TIME DELAY SINCE THE LAST PLUCK
	delay_since_last_detect = delay_since_last_detect + 1;

	//### INCREMENT INDEX COUNTER THAT TRACKS HOW MANY SAMPLES WE'VE SEEN SO FAR
	Pindex = Pindex + 1;
	if (Pindex == 0)
	{
		Pindex = 1;
	}

	//### STORE CURRENT VALUES TO COMPARE AGAINST IN NEXT ITERATION
	prior_super_smoothed = super_smoothed;
	prior_smoothed = smoothed;


	tempCount6 = DWT->CYCCNT;

	cycleCountVals[0][2] = 0;

	cycleCountVals[0][1] = tempCount6-tempCount5;
	CycleCounterTrackMinAndMax(0);
	__enable_irq();
	CycleCounterAddToAverage(0);
}




float previousRampSmoothed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
int downCounter[4] = {0,0,0,0};
float pastValues[4][512];
int whichVal = 0;
int strikeTime[4];

void attackDetect2(int whichString, int tempInt)
{
	volatile uint32_t tempCount5 = 0;
	volatile uint32_t tempCount6 = 0;
	__disable_irq();
	tempCount5 = DWT->CYCCNT;

	float intoThresh = 0.0f;
	float dbSmoothed = 0.0f;
	float dbSmoothed2 = 0.0f;
	float smoothed = 0.0f;
	float smoothed2 = 0.0f;
	float tempAbs = 0.0f;
	//float increment = 0.005f;
	float increment = 0.000755857898715f;
	//float increment = 0.000455857898715f;
	//float increment = 0.000255857898715f;
	//float rampsmoothed = 0.0f;

	float tempSamp = (((float)tempInt - INV_TWO_TO_15) * INV_TWO_TO_15);
	for (int k = 0; k < FILTER_ORDER; k++)
	{
		//tempSamp = tHighpass_tick(&opticalHighpass[j+ (NUM_STRINGS * k)], tVZFilter_tick(&opticalLowpass[j + (NUM_STRINGS * k)], tempSamp));
		tempSamp = tHighpass_tick(&opticalHighpass[whichString+ (NUM_STRINGS * k)], tempSamp);
		//
	}
	tempSamp = tVZFilter_tick(&opticalLowpass[whichString], tempSamp) * 1.5f;
	float input = tempSamp;
	noteOnHappened[whichString] = 0;
	//input = tHighpass_tick(&dcBlock[whichString], input);
	//input = tSVF_tick(&lowpass[whichString], input);
	tempAbs = fabsf(input);
	pastValues[whichString][whichVal%512] = tempAbs;

	smoothed = tSlide_tick(&fastSlide[whichString], tempAbs);
	smoothed2  = tSlide_tick(&slowSlide[whichString], smoothed);
	dbSmoothed = LEAF_clip(-60.0f, atodbTable[(uint32_t)(smoothed * ATODB_TABLE_SIZE_MINUS_ONE)], 12.0f);
	dbSmoothed2 = LEAF_clip(-60.0f, atodbTable[(uint32_t)(smoothed2 * ATODB_TABLE_SIZE_MINUS_ONE)], 12.0f);
	intoThresh = dbSmoothed - dbSmoothed2;

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
		status[0] = 1.0f;
		status[1] = 1.0f;
		status[2] = 1.0f;
		status[3] = 1.0f;
		currentMaximum[whichString] = 0.0f;
		/*
		for (int i = 0; i < distanceBetweenReadAndWrite; i++)
		{
			float testSample = audioADCInputs[whichString][(i+sampleNumGlobal) % ADC_RING_BUFFER_SIZE];
			if (testSample > currentMaximum[whichString])
			{
				currentMaximum[whichString] = testSample;
			}
		}
		*/
		if (whichString == 0)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
		}
		else if (whichString == 1)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET);
		}

		sahArmed[whichString] = 1;
		//downCounter[whichString] = 0;
		offLockout[whichString] = offLockoutDelay;
		delayCounter[whichString] = 200;
		pastValues[whichString][whichVal%512] += 2.0f;
		//TODO: //may want to make this look into the future values held by the ADC input array instead and set this delay to the time between the read and write pointers
		strikeTime[whichString] = whichVal%512;
		sahArmed[whichString] = 1;
	}
	else if (status[whichString] > 0.0f)
	{
		status[whichString] = status[whichString] - (increment);
	}
	/*
	tRampUpDown_setDest(&updownRamp[whichString], tempAbs);
	rampsmoothed = tRampUpDown_tick(&updownRamp[whichString]);

	*/
	if (tempAbs > currentMaximum[whichString])
	{
		currentMaximum[whichString] = tempAbs;
	}


	if (delayCounter[whichString] > 0)
	{
		delayCounter[whichString]--;
	}

	/*
	if ((intoThresh < (previousRampSmoothed[whichString] - 0.004f)) && (sahArmed[whichString] == 1))
	{
		downCounter[whichString]++;
	}
	else if ((intoThresh >= (previousRampSmoothed[whichString]) - 0.1f) && (sahArmed[whichString] == 1))
	{
		downCounter[whichString] = 0;
	}
*/

	if ((sahArmed[whichString] == 1) && (delayCounter[whichString] == 0))
	//if you waited some number of samples to ride to the peak, then now make a noteOn event
	//if ((delayCounter[whichString] == 0) && (sahArmed[whichString] == 1))
	{
		//float tempAmp = map(currentMaximum, -60.0f, 6.0f, 0.0f, 1.0f);
		//float tempAmp = fasterPowf(currentMaximum[whichString], 0.5f);

		//tADSR_on(&envelope[whichString], currentMaximum[whichString]);
		if (whichString == 0)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
		}
		else if (whichString == 1)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
		}
		if (waitTimeOver)
		{
			tSimplePoly_noteOn(&poly, whichString, LEAF_clip(1, currentMaximum[whichString] * 512.0f, 127));
			tADSR_on(&envelope[0], currentMaximum[whichString]*10.0f);
		}
		//tSimplePoly_noteOn(&poly, whichString, 127);
		pastValues[whichString][whichVal%512] += 3.0f;
		//noteOnAmplitude[whichString] = tempAmp;
		noteOnHappened[whichString] = 1;
		sahArmed[whichString] = 0;
		//downCounter[whichString] = 0;
		offLockout[whichString] = offLockoutDelay;
	}
	//previousRampSmoothed[whichString] = intoThresh;

	if (offLockout[whichString] > 0)
	{
		offLockout[whichString]--;
	}
	tempCount6 = DWT->CYCCNT;

	cycleCountVals[1][2] = 0;

	cycleCountVals[1][1] = tempCount6-tempCount5;
	__enable_irq();
}


int prevPolyOn = 0;

float processString(int whichString, float input)
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


	noteOffHappened[whichString] = 0;


	stringPositions[whichString] =  ((uint16_t)SPI_RX[whichString * 2] << 8) + ((uint16_t)SPI_RX[(whichString * 2) + 1] & 0xff);




	if (stringPositions[whichString] == 65535)
	{
		stringFrequencies[whichString] = openStringFrequencies[whichString];
		stringMIDIVersionOfFrequencies[whichString] = LEAF_frequencyToMidi(stringFrequencies[whichString]);
	}
	else
	{
		stringMappedPositions[whichString] = map((float)stringPositions[whichString], fretMeasurements[1][whichString], fretMeasurements[2][whichString], fretScaling[1], fretScaling[2]);
		stringFrequencies[whichString] = ((1.0 / stringMappedPositions[whichString])) * openStringFrequencies[whichString];
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
	if (((LHmuteCounter[whichString] > LHMUTE_COUNTLIM) && (stringPositions[whichString] == 65535)) || (RHmuteCounter[whichString] > RHMUTE_COUNTLIM))
	{


		if (offLockout[whichString] == 0)
		{
			tSimplePoly_noteOff(&poly, whichString);
			if (tSimplePoly_getNumActiveVoices(&poly) == 0)
			{
				tADSR_off(&envelope[0]);
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
		tExpSmooth_setDest(&pitchSmoother[whichString], mtof((round(stringMIDIVersionOfFrequencies[whichString]))));
	}
	float myFreq = tExpSmooth_tick(&pitchSmoother[whichString]) * octave;
	//for (int i = 0; i < NUM_SAWTOOTHS; i++)
	//{
		//tSawtooth_setFreq(&mySaw[whichString][i], myFreq * detuneAmounts[i]);
	//}
	//tCycle_setFreq(&mySine[whichString], myFreq);
	//joy_x = ((SPI_RX[9] << 8) + (SPI_RX[10])) * 0.00025f;
	//joy_y = ((SPI_RX[11] << 8) + (SPI_RX[12])) * 0.00025f;
	//knobParams[0] = faster_mtof(joy_x * 230.0f) ; //brightness
	//tSVF_setFreq(&myLowpass[whichString], LEAF_clip(50.0f, (knobParams[0] + (myFreq * 3.0f)) * tADSR_tick(&envelope[whichString]), 18000.0f));


	stringFreqs[whichString] = myFreq;

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
int sampRecords[256];
uint8_t currentSamp;

uint64_t SDWriteIndex = 0;

void ADC_Frame(int offset)
{
	//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_11);

	sampRecords[currentSamp] = frameCount;
	currentSamp++;
	for (int i = offset; i < ADC_FRAME_SIZE + offset; i++)
	{
		for (int j = 0; j < NUM_ADC_CHANNELS; j++)
		{
			int tempInt = ADC_values[(i*NUM_ADC_CHANNELS) + j];
			//float tempSamp = (((float)tempInt - INV_TWO_TO_15) * INV_TWO_TO_15);

			stringPositions[j] =  ((uint16_t)SPI_RX[j * 2] << 8) + ((uint16_t)SPI_RX[(j * 2) + 1] & 0xff);
			if (stringPositions[j] == 65535)
			{
				stringPositions[j] = 0;
			}
			stringTouchLH[j] = (SPI_RX[8] >> j) & 1;
			stringTouchRH[j] = (SPI_RX[8] >> (j + 4)) & 1;

			if (j == 0)
			{
				attackDetectAH(0, tempInt);
			}
			//tempSamp = tHighpass_tick(&opticalHighpass[j+NUM_STRINGS], tHighpass_tick(&opticalHighpass[j], tempSamp));

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



			//attackDetect2(j, tempInt);
			//processString(j, tempSamp);

		}
		whichVal++;

	}


	if (whichVal > 3000)
	{
		waitTimeOver = 1;
	}
	ADC_Ready = 1;

	CycleCounterAverage(0);

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
