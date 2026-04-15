/**
  ******************************************************************************
  * @file    Audio_playback_and_record/inc/waveplayer.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Header for waveplayer.c module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIOSTREAM_H
#define __AUDIOSTREAM_H

/* Includes ------------------------------------------------------------------*/
#include "parameters.h"
#include "leaf.h"
#include "main.h"


#define AUDIO_FRAME_SIZE      64
#define HALF_BUFFER_SIZE      AUDIO_FRAME_SIZE * 2 //number of samples per half of the "double-buffer" (twice the audio frame size because there are interleaved samples for both left and right channels)
#define AUDIO_BUFFER_SIZE     AUDIO_FRAME_SIZE * 4 //number of samples in the whole data structure (four times the audio frame size because of stereo and also double-buffering/ping-ponging)
#define SMALL_MEM_SIZE 60000
#define MED_MEM_SIZE 262144//180000
#define LARGE_MEM_SIZE 31457280//67108864 would be 64 MBytes - size of SDRAM IC, but we are using 2MB each for firmware buffers = 2097152*2 =   4194304, so 67108864-4194304=62914560
#define MTOF_TABLE_SIZE	32768
#define MTOF_TABLE_SIZE_MINUS_ONE 32767
#define MTOF_TABLE_SIZE_DIV_TWO	16384
#define ATODB_TABLE_SIZE 16384
#define DBTOA_TABLE_SIZE 16384
#define ATODB_TABLE_SIZE_MINUS_ONE 16383
#define DBTOA_TABLE_SIZE_MINUS_ONE 16383

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

typedef enum
{
  NothingLoaded = 0,
  String1Loaded,
  String2Loaded,
  String3Loaded,
  String4Loaded,
  SynthLoaded,
}StringModelLoadedTypeDef;

//#define SAMPLERATE96K
#ifdef SAMPLERATE96K
#define SAMPLE_RATE 96000.f
#else
#define SAMPLE_RATE 48000.f
#endif

#define INV_SAMPLE_RATE 1.f/SAMPLE_RATE
#define SAMPLE_RATE_MS (SAMPLE_RATE / 1000.f)
#define INV_SR_MS 1.f/SAMPLE_RATE_MS
#define SAMPLE_RATE_DIV_PARAMS SAMPLE_RATE / 3
#define SAMPLE_RATE_DIV_PARAMS_MS (SAMPLE_RATE_DIV_PARAMS / 1000.f)
#define INV_SR_DIV_PARAMS_MS 1.f/SAMPLE_RATE_DIV_PARAMS_MS


#define EXP_BUFFER_SIZE 2048
#define DECAY_EXP_BUFFER_SIZE 4096

#define CTRL_MIDI_START 9

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;



void audioInit(void);
void audioStart(SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn);

void initFunctionPointers(void);



typedef void (*audioFrame_t)(uint16_t);
extern audioFrame_t audioFrameFunction;

typedef float (*audioTick_t)(void);
extern audioTick_t audioTickFunction;

void audioFrame(uint16_t buffer_offset);
void DMA1_TransferCpltCallback(DMA_HandleTypeDef *hdma);
void DMA1_HalfTransferCpltCallback(DMA_HandleTypeDef *hdma);
void updateStateFromSPIMessage(uint8_t currentByte);

float mtofTableLookup(float tempMIDI);

void  switchStringModel(int which);
float dbToATableLookup(float in);

void  audioFrameWaiting(uint16_t buffer_offset);

extern tMempool largePool;
extern float sourceValues[NUM_SOURCES][NUM_STRINGS_PER_BOARD];
extern uint8_t lfoOn[NUM_LFOS];
extern uint8_t envOn[NUM_ENV];
extern uint8_t oscOn[NUM_OSC];
extern uint8_t noiseOn;
extern float oscAmpMult;
extern float oscAmpMultArray[4];

extern int32_t audioOutBuffer[AUDIO_BUFFER_SIZE];
extern int32_t audioInBuffer[AUDIO_BUFFER_SIZE];
extern uint32_t codecReady;
extern int presetReady;
extern volatile int voice;
extern volatile int prevVoice;
extern uint8_t oscToTick;
extern uint8_t filterToTick;
extern uint32_t overSampled;
extern uint8_t numEffectToTick;
//extern float audioDisplayBuffer[128];
extern uint32_t displayBufferIndex;
extern uint8_t numStringsThisBoard;
extern volatile int firstString;
extern volatile float stringMIDIPitches[NUM_STRINGS_PER_BOARD];
extern tExpSmooth knobSmoothers[20];
extern tExpSmooth pedalSmoothers[10];
extern volatile uint16_t stringInputs[NUM_STRINGS];
extern volatile uint8_t knobFrozen[20];
extern volatile uint8_t whichBar;
extern volatile uint16_t sampleClippedCountdown;
extern float masterVolFromBrain;
extern float masterVolFromBrainForSynth;
extern uint8_t interrupted;
extern volatile uint32_t newPluck;

extern volatile uint8_t resetStringInputs;
extern uint8_t lsDecay[NUM_STRINGS_PER_BOARD];
extern volatile uint16_t previousStringInputs[12];
extern float stringOctave[NUM_STRINGS_PER_BOARD];
extern float octave;
extern tMempool mediumPool;
extern uint8_t whichStringModelLoaded;
extern float randomFactors[256];
extern float pluckPos;
extern float knobScaled[20];
extern volatile int switchStrings;

extern uint32_t timeFrame;
extern float frameLoadPercentage;
extern float frameLoadMultiplier;
extern tExpSmooth volumeSmoother;
extern tExpSmooth pitchSmoother[NUM_OSC][NUM_STRINGS_PER_BOARD];
extern float barInMIDI[2];
extern float prevBarInMIDI[2];
extern tExpSmooth barSlideSmoother[NUM_STRINGS_PER_BOARD];
extern tEnvelopeFollower barNoiseSmoother[NUM_STRINGS_PER_BOARD];
extern tVZFilter noiseFilt;
extern tVZFilter noiseFilt2;

extern tNoise myNoise;

extern float volumeAmps128[128];
extern tADSRT fenvelopes[NUM_STRINGS_PER_BOARD];
extern float decayExpBuffer[DECAY_EXP_BUFFER_SIZE];
extern LEAF leaf;
extern float stringFrequencies[NUM_STRINGS];
extern float atoDbTable[ATODB_TABLE_SIZE];
extern float dbtoATable[DBTOA_TABLE_SIZE];
extern uint32_t frameLoadOverCount;
extern float pedalScaled[10];
extern float volumePedal;
extern float invNumStrings;



#endif /* __AUDIOSTREAM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
