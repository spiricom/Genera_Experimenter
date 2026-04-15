/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "parameters.h"
#include "audiostream.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define __ATTR_RAM_D1	__attribute__ ((section(".sram1_bss"))) __attribute__ ((aligned (32)))
#define __ATTR_RAM_D2_DMA	__attribute__ ((section(".sram2_dma_bss"))) __attribute__ ((aligned (32)))
#define __ATTR_RAM_D2	__attribute__ ((section(".sram2_bss"))) __attribute__ ((aligned (32)))
#define __ATTR_RAM_D3	__attribute__ ((section(".sram3_bss"))) __attribute__ ((aligned (32)))
#define __ATTR_USER_FLASH	__attribute__ ((section(".userflash"))) __attribute__ ((aligned (32)))
#define __ATTR_SDRAM	__attribute__ ((section(".sdram_bss"))) __attribute__ ((aligned (32)))
#define __ATTR_ITCMRAM	__attribute__ ((section(".itcmram"))) __attribute__ ((aligned (32)))

# define FORCE_INLINE __attribute__((always_inline)) inline

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
float randomNumber(void);
void CycleCounterInit( void );
void __ATTR_ITCMRAM handleSPI (uint8_t offset);
float __ATTR_ITCMRAM scaleFilterResonance(float input);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

#define LEVER_BUFFER_SIZE 32
#define LEVER_BUFFER_SIZE_TIMES_TWO LEVER_BUFFER_SIZE*2

#define PLUCK_BUFFER_SIZE 32 //26
#define PLUCK_BUFFER_SIZE_TIMES_TWO PLUCK_BUFFER_SIZE*2

#define BAR_BUFFER_SIZE 8
#define BAR_BUFFER_SIZE_TIMES_TWO BAR_BUFFER_SIZE*2

extern uint8_t SPI_RX[BAR_BUFFER_SIZE_TIMES_TWO];
extern uint8_t SPI_PLUCK_TX[PLUCK_BUFFER_SIZE_TIMES_TWO];
extern uint8_t SPI_PLUCK_RX[PLUCK_BUFFER_SIZE_TIMES_TWO];
extern uint8_t SPI_LEVERS_TX[LEVER_BUFFER_SIZE_TIMES_TWO];
extern uint8_t SPI_LEVERS_RX[LEVER_BUFFER_SIZE_TIMES_TWO];
extern uint8_t levers[2][LEVER_BUFFER_SIZE];
extern int currentLeverBuffer;
#define MAX_NUM_PRESETS 64
extern float midiKeyDivisor;
extern float midiKeySubtractor;
extern uint8_t effectsActive[4];
extern volatile uint8_t currentActivePreset;
#define NUM_PARAMS numParams
#define MAX_NUM_MAPPINGS 32
extern param params[NUM_PARAMS];
extern mapping mappings[MAX_NUM_MAPPINGS];
extern volatile float audioMasterLevel;
extern uint8_t fxPre;
extern uint8_t pedalControlsMaster;
extern uint8_t numMappings;
extern uint8_t diskBusy;
extern uint8_t volatile interruptChecker;
extern float random_values[256];
extern volatile uint32_t presetWaitingToLoad;
extern volatile uint8_t presetNumberToLoad;
extern uint8_t boardNumber;
extern uint8_t currentRandom;
extern uint8_t receivingI2C;
extern volatile uint8_t presetNamesArray[MAX_NUM_PRESETS][14]__ATTR_RAM_D2;
extern volatile uint8_t macroNamesArray[MAX_NUM_PRESETS][20][10]__ATTR_RAM_D2;
extern float loadedKnobParams[20];
extern uint8_t whichModel;
extern uint_fast8_t knobTicked[12];
extern uint_fast8_t pedalTicked[10];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
