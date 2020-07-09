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

#define NUM_STRINGS NUM_ADC_CHANNELS

#define ATODB_TABLE_SIZE 512
#define ATODB_TABLE_SIZE_MINUS_ONE 511

#define FILTER_ORDER 12
#define LHMUTE_COUNTLIM 100
#define RHMUTE_COUNTLIM 10

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
void attackDetect2(int whichString, float input);
float SFXRhodesTick(int whichString);
void SFXRhodesAlloc(void);



HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;


uint8_t codecReady = 0;
float sample = 0.0f;
float rightIn = 0.0f;

#define NUM_SAWTOOTHS 1
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

//MEMPOOLS
#define SMALL_MEM_SIZE 50000
char smallMemory[SMALL_MEM_SIZE];

#define MEDIUM_MEM_SIZE 500000
char mediumMemory[MEDIUM_MEM_SIZE] __ATTR_RAM_D1;

//#define LARGE_MEM_SIZE 33554432 //32 MBytes - size of SDRAM IC
//char largeMemory[LARGE_MEM_SIZE] __ATTR_SDRAM;

tMempool mediumPool;
//tMempool largePool;


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
	//tMempool_init (&largePool, largeMemory, LARGE_MEM_SIZE);

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
		tADSR_init(&envelope[i], 0.0f, 100.0f, 0.6f, 20.0f);
		tADSR_setLeakFactor(&envelope[i], 0.999998f);
		tSlide_init(&fastSlide[i],1.0f,1110.0f);
		tSlide_init(&slowSlide[i],500.0f,1.0f);
		tSVF_init(&lowpass[i], SVFTypeLowpass, 4000.0f, 1.0f);
		tRampUpDown_init(&updownRamp[i], 0.0f, 104.0f, 1); //5000 samples should be 104 ms
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

			audioOutBuffer[buffer_offset + i] = (int32_t)(audioTick() * TWO_TO_23);
			audioOutBuffer[buffer_offset + i + 1] = 0;
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

	float tempSample = 0.0f;
	//sample += tNoise_tick(&noise[whichString]);// * tADSR_tick(&envelope[whichString]);

	tExpSmooth_setDest(&gainSmoothed, tSimplePoly_getVelocity(&poly, 0) * 0.2f);
	int myString = tSimplePoly_getPitch(&poly, 0);
	float theFreq = stringFreqs[myString];
	for (int i = 0; i < NUM_SAWTOOTHS; i++)
	{
		tSawtooth_setFreq(&mySaw[0][i], theFreq * detuneAmounts[i]);
	}
	float theEnv = tExpSmooth_tick(&gainSmoothed);

	//float theEnv = tADSR_tick(&envelope[0]);
	for (int j = 0; j < NUM_SAWTOOTHS; j++)
	{
		tempSample += tSawtooth_tick(&mySaw[0][j]) * theEnv;
	}
	sample = tempSample;
	sample = tanhf(sample);
	return sample;

}



float previousRampSmoothed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
int downCounter[4] = {0,0,0,0};
float pastValues[4][512];
int whichVal = 0;
int strikeTime[4];

void attackDetect2(int whichString, float input)
{
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
		status[whichString] = 1.0f;
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
		delayCounter[whichString] = 128;
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
		tSimplePoly_noteOn(&poly, whichString, LEAF_clip(0, currentMaximum[whichString] * 512.0f, 127));
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

}




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
			float tempSamp = (((float)tempInt - INV_TWO_TO_15) * INV_TWO_TO_15);
			//tempSamp = tHighpass_tick(&opticalHighpass[j+NUM_STRINGS], tHighpass_tick(&opticalHighpass[j], tempSamp));


			for (int k = 0; k < FILTER_ORDER; k++)
			{
				//tempSamp = tHighpass_tick(&opticalHighpass[j+ (NUM_STRINGS * k)], tVZFilter_tick(&opticalLowpass[j + (NUM_STRINGS * k)], tempSamp));
				tempSamp = tHighpass_tick(&opticalHighpass[j+ (NUM_STRINGS * k)], tempSamp);
				//
			}
			tempSamp = tVZFilter_tick(&opticalLowpass[j], tempSamp);

			attackDetect2(j, tempSamp);
			processString(j, tempSamp);
		}
		whichVal++;

	}

	ADC_Ready = 1;

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
