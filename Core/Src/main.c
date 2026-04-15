/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rng.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "audiostream.h"
#include "leaf.h"
#include "codec.h"
#include "synth.h"
#include "string1.h"
#include "string2.h"
#include "string3.h"
#include "additive.h"
#include "vocal.h"
#include "string4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint_fast8_t  brainFirmwareUpdateRequested = 0;
uint_fast8_t  pluckFirmwareUpdateRequested = 0;

uint_fast8_t knobTicked[12];
uint_fast8_t pedalTicked[10];
uint8_t SPI_PLUCK_TX[PLUCK_BUFFER_SIZE_TIMES_TWO] __ATTR_RAM_D3;
uint8_t SPI_PLUCK_RX[PLUCK_BUFFER_SIZE_TIMES_TWO] __ATTR_RAM_D3;
uint8_t SPI_LEVERS_RX[LEVER_BUFFER_SIZE_TIMES_TWO] __ATTR_RAM_D2_DMA;
uint8_t SPI_LEVERS_TX[LEVER_BUFFER_SIZE_TIMES_TWO] __ATTR_RAM_D2_DMA;
volatile uint32_t myTester = 0;
int currentLeverBuffer = 0;

uint8_t volatile currentPage = 0;
uint8_t volatile previousPage = 0;

float random_values[256];
uint8_t currentRandom = 0;
uint8_t diskBusy = 0;
FILINFO fno;
DIR dir;
const TCHAR path = 0;

volatile uint8_t numResets = 0;

uint8_t receivingI2C = 0;

uint8_t bootloaderFlag[32] __ATTR_USER_FLASH;
uint8_t buttonPressed = 0;
uint8_t resetFlag = 0;

uint32_t currentPresetSize = 0;
uint8_t brainFirmwareBuffer[2097152] __ATTR_SDRAM; // 2 MB of space for firmware
//uint8_t pluckFirmwareBuffer[2097152] __ATTR_SDRAM; // 2 MB of space for firmware

//uint8_t localPluck[40000] __ATTR_RAM_D2;
uint32_t pluckIndex = 0;

uint32_t copyingPluckToLocalArray = 0;
uint32_t brainFirmwareSize = 0;
uint32_t pluckFirmwareSize = 0;
uint8_t foundBrainFirmware = 0;
uint8_t foundPluckFirmware = 0;
uint32_t brainFirmwareBufferIndex = 0;
uint32_t pluckFirmwareBufferIndex = 0;
uint_fast8_t  brainFirmwareEndSignal = 0;
uint_fast8_t  pluckFirmwareEndSignal = 0;

uint_fast8_t  pluckFirmwareClearSignal = 0;
uint_fast8_t  brainFirmwareSendInProgress = 0;
uint_fast8_t  pluckFirmwareSendInProgress = 0;
uint32_t  pluckFirmwareReadyToSend = 0;
volatile uint8_t writingState = 0;
volatile float 	audioMasterLevel = 1.0f;
uint8_t fxPre = 0;
uint8_t pedalControlsMaster = 0;

FIL fdst;
uint8_t buffer[4096] __ATTR_RAM_D2;
volatile uint16_t bufferPos = 0;
FRESULT res;

volatile uint8_t presetNumberToSave;
volatile uint8_t presetNumberToLoad = 0;
volatile uint8_t tuningNumberToSave;
volatile uint8_t currentActivePreset = 127;//
volatile uint8_t presetName[14];
volatile uint8_t presetNamesArray[MAX_NUM_PRESETS][14]__ATTR_RAM_D2;
volatile uint8_t whichPresetToSendName = 0;
volatile uint32_t presetWaitingToParse = 0;
volatile uint32_t presetWaitingToWrite = 0;
volatile uint32_t presetWaitingToLoad = 0;

volatile uint8_t macroNamesArray[MAX_NUM_PRESETS][20][10]__ATTR_RAM_D2;
uint8_t whichMacroToSendName = 0;

param params[NUM_PARAMS];
mapping mappings[MAX_NUM_MAPPINGS];
uint8_t numMappings = 0;

uint8_t effectsActive[4] = {0,0,0,0};

filterSetter filterSetters[NUM_FILT];
lfoSetter lfoSetters[NUM_LFOS];
effectSetter effectSetters[NUM_EFFECT];
float defaultScaling = 1.0f;

#define SCALE_TABLE_SIZE 2048
float resTable[SCALE_TABLE_SIZE];
float envTimeTable[SCALE_TABLE_SIZE];
float lfoRateTable[SCALE_TABLE_SIZE];

int oscsEnabled[3] = {0,0,0};
float midiKeyDivisor;
float midiKeySubtractor;

int32_t volatile prevKnobByte[20];

uint8_t volatile interruptChecker = 0;

uint8_t volatile memoryTest[700];

uint8_t volatile foundOne = 0;
uint8_t loadFailed = 0;
uint32_t volatile myTestInt = 0;
uint8_t boardNumber = 0;

uint8_t volatile i2cSending = 0;

const char* specialModeNames[5];
const char* specialModeMacroNames[5][20];

float loadedKnobParams[20];

uint8_t whichModel; //0=synth 1=string1 2=string2 3=additive 4=vocal 5=string3

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void MPU_Conf(void);
void SDRAM_init(void);
static int checkForSDCardPreset(uint8_t value);
static void writePresetToSDCard(int fileSize);
void __ATTR_ITCMRAM parsePreset(int size, int presetNumber);
void getPresetNamesFromSDCard(void);
static void checkForBootloadableBrainFile(void);
//static void checkForBootloadablePluckFile(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_8)
  {

	  for (int i = 0; i < 32; i++)
	  {
		  levers[!currentLeverBuffer][i] = SPI_LEVERS_RX[i];
	  }
	  currentLeverBuffer = !currentLeverBuffer;
	  HAL_SPI_Receive_DMA(&hspi1, SPI_LEVERS, 32);
  }
}
*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
  {

  /* USER CODE BEGIN 1 */
	//SCB_CleanInvalidateDCache();
	//SCB_InvalidateICache();
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  __enable_irq();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  //HAL_Delay(500);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FMC_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_SAI1_Init();
  MX_RNG_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */

	int bit0 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	int bit1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	int bit2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	boardNumber = ((bit0 << 1)+(bit1 << 2)+(bit2));


	  //HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

	//GPIO_InitTypeDef GPIO_InitStruct = {0};
	//  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 ;
	//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	// GPIO_InitStruct.Pull = GPIO_NOPULL;
	//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	if (boardNumber == 0)
	{
		  //HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

		//GPIO_InitTypeDef GPIO_InitStruct = {0};
	  	 // GPIO_InitStruct.Pin = GPIO_PIN_12;
	  	 // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  	 //GPIO_InitStruct.Pull = GPIO_NOPULL;
	  	  //HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	}
	HAL_Delay(500);
    if (boardNumber !=0)
    {
    	//HAL_I2C_Slave_Receive_IT(&hi2c1, buffer, 4096);
    }

  /* Enable write access to Backup domain */
   PWR->CR1 |= PWR_CR1_DBP;
   while((PWR->CR1 & PWR_CR1_DBP) == RESET)
   {
	   ;
   }
   /*Enable BKPRAM clock*/
   __HAL_RCC_BKPRAM_CLK_ENABLE();
  // __HAL_RCC_D2SRAM1_CLK_ENABLE();

  // __HAL_RCC_D2SRAM2_CLK_ENABLE();

  // __HAL_RCC_D2SRAM3_CLK_ENABLE();
  //LED off
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  uint32_t tempFPURegisterVal = __get_FPSCR();
  tempFPURegisterVal |= (1<<24); // set the FTZ (flush-to-zero) bit in the FPU control register  // this makes checking for denormals not necessary as they are automatically set to zero by the hardware
  __set_FPSCR(tempFPURegisterVal);


  CycleCounterInit();
/*
  while(true)
  {

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	  HAL_Delay(200);

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	  HAL_Delay(200);
  }
  */

  for (int i = 0; i < PLUCK_BUFFER_SIZE_TIMES_TWO; i++)
  {
	  SPI_PLUCK_TX[i] = 0;
  } //put in some values to make the array valid as a preset


  for (int i = 0; i < 4096; i++)
  {
	  buffer[i] = 0;
  } //put in some values to make the array valid as a preset
  buffer[15+112] = NUM_PARAMS;
  buffer[NUM_PARAMS*2+16+112] = 0xef;
  buffer[NUM_PARAMS*2+17+112] = 0xef;
  buffer[NUM_PARAMS*2+19+112] = 1;
  buffer[NUM_PARAMS*2+25+112] = 0xfe;
  buffer[NUM_PARAMS*2+26+112] = 0xfe;

  LEAF_generate_table_skew_non_sym(resTable, 0.01f, 10.0f, 0.5f, SCALE_TABLE_SIZE);
  LEAF_generate_table_skew_non_sym(envTimeTable, 0.0f, 20000.0f, 4000.0f, SCALE_TABLE_SIZE);
  LEAF_generate_table_skew_non_sym(lfoRateTable, 0.0f, 30.0f, 2.0f, SCALE_TABLE_SIZE);

  for (int i = 0; i < 3; i++)
  {
	  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	  //HAL_Delay(10);
	  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	  //HAL_Delay(10);
  }
  //only board 0 has access to the SD card
  if (boardNumber == 0)
  {
	  getPresetNamesFromSDCard();
	  //foundOne  = checkForSDCardPreset(presetNumberToLoad);
	  //presetWaitingToLoad = 1;
	  diskBusy = 1;
  }
  else
  {
	  diskBusy = 1;
  }


  SDRAM_init();

  //codec_init(&hi2c2);

  audioInit();




  specialModeNames[0] = "PHYS STR1     ";
  specialModeMacroNames[0][0] = "Feedback  ";
  specialModeMacroNames[0][1] = "SlideNois ";
  specialModeMacroNames[0][2] = "PluckWidt ";
  specialModeMacroNames[0][3] = "PickNoise ";
  specialModeMacroNames[0][4] = "PickupSim ";
  specialModeMacroNames[0][5] = "          ";
  specialModeMacroNames[0][6] = "          ";
  specialModeMacroNames[0][7] = "          ";
  specialModeMacroNames[0][8] = "          ";
  specialModeMacroNames[0][9] = "          ";
  specialModeMacroNames[0][10] = "          ";
  specialModeMacroNames[0][11] = "          ";
  specialModeMacroNames[0][12] = "          ";
  specialModeMacroNames[0][13] = "          ";
  specialModeMacroNames[0][14] = "          ";
  specialModeMacroNames[0][15] = "          ";
  specialModeMacroNames[0][16] = "          ";
  specialModeMacroNames[0][17] = "          ";
  specialModeMacroNames[0][18] = "          ";
  specialModeMacroNames[0][19] = "          ";

  specialModeNames[1] = "PHYS STR2     ";
  specialModeMacroNames[1][0] = "DecayTime ";
  specialModeMacroNames[1][1] = "Tone      ";
  specialModeMacroNames[1][2] = "PluckPos  ";
  specialModeMacroNames[1][3] = "PickupPos ";
  specialModeMacroNames[1][4] = "SlideNois ";
  specialModeMacroNames[1][5] = "Stiffness ";
  specialModeMacroNames[1][6] = "FB Amp    ";
  specialModeMacroNames[1][7] = "FB Speed  ";
  specialModeMacroNames[1][8] = "PU Filter ";
  specialModeMacroNames[1][9] = "Harmonic  ";
  specialModeMacroNames[1][10] = "HarmPosX  ";
  specialModeMacroNames[1][11] = "HarmPosY  ";
  specialModeMacroNames[1][12] = "PUModRate ";
  specialModeMacroNames[1][13] = "PUModAmp  ";
  specialModeMacroNames[1][14] = "PhantomH  ";
  specialModeMacroNames[1][15] = "PUFilterQ ";
  specialModeMacroNames[1][16] = "PeakF Frq ";
  specialModeMacroNames[1][17] = "PeakF Q   ";
  specialModeMacroNames[1][18] = "Tension G ";
  specialModeMacroNames[1][19] = "Acoustic  ";


  specialModeNames[2] = "ADDITIVE      ";
  specialModeMacroNames[2][0] = "Stretch   ";
  specialModeMacroNames[2][1] = "Tilt      ";
  specialModeMacroNames[2][2] = "NoiseAmp  ";
  specialModeMacroNames[2][3] = "PickupPos ";
  specialModeMacroNames[2][4] = "PickupAmp ";
  specialModeMacroNames[2][5] = "DiveAmp   ";
  specialModeMacroNames[2][6] = "DiveRate  ";
  specialModeMacroNames[2][7] = "          ";
  specialModeMacroNames[2][8] = "          ";
  specialModeMacroNames[2][9] = "          ";
  specialModeMacroNames[2][10] = "Tone      ";
  specialModeMacroNames[2][11] = "Decay     ";
  specialModeMacroNames[2][12] = "          ";
  specialModeMacroNames[2][13] = "          ";
  specialModeMacroNames[2][14] = "          ";
  specialModeMacroNames[2][15] = "          ";
  specialModeMacroNames[2][16] = "          ";
  specialModeMacroNames[2][17] = "          ";
  specialModeMacroNames[2][18] = "          ";
  specialModeMacroNames[2][19] = "          ";

  specialModeNames[3] = "STRING4      ";
  specialModeMacroNames[3][0] = "PU Width  ";
  specialModeMacroNames[3][1] = "PU Pos    ";
  specialModeMacroNames[3][2] = "PU Filt   ";
  specialModeMacroNames[3][3] = "Nonlin    ";
  specialModeMacroNames[3][4] = "V gain    ";
  specialModeMacroNames[3][5] = "H gain    ";
  specialModeMacroNames[3][6] = "Decay     ";
  specialModeMacroNames[3][7] = "Tone    ";
  specialModeMacroNames[3][8] = "BackPos   ";
  specialModeMacroNames[3][9] = "BackDiam  ";
  specialModeMacroNames[3][10] = "ToungePos  ";
  specialModeMacroNames[3][11] = "ToungeDia  ";
  specialModeMacroNames[3][12] = "          ";
  specialModeMacroNames[3][13] = "          ";
  specialModeMacroNames[3][14] = "          ";
  specialModeMacroNames[3][15] = "          ";
  specialModeMacroNames[3][16] = "          ";
  specialModeMacroNames[3][17] = "          ";
  specialModeMacroNames[3][18] = "          ";
  specialModeMacroNames[3][19] = "          ";

  specialModeNames[4] = "STRING3      ";
  specialModeMacroNames[4][0] = "Stiffness ";
  specialModeMacroNames[4][1] = "PU Prop   ";
  specialModeMacroNames[4][2] = "Plck Prop ";
  specialModeMacroNames[4][3] = "PU Filter ";
  specialModeMacroNames[4][4] = "NoiseFilt ";
  specialModeMacroNames[4][5] = "NoiseGain ";
  specialModeMacroNames[4][6] = "          ";
  specialModeMacroNames[4][7] = "          ";
  specialModeMacroNames[4][8] = "Decay     ";
  specialModeMacroNames[4][9] = "Damping   ";
  specialModeMacroNames[4][10] = "PluckPos ";
  specialModeMacroNames[4][11] = "PU Pos   ";
  specialModeMacroNames[4][12] = "          ";
  specialModeMacroNames[4][13] = "          ";
  specialModeMacroNames[4][14] = "          ";
  specialModeMacroNames[4][15] = "          ";
  specialModeMacroNames[4][16] = "          ";
  specialModeMacroNames[4][17] = "          ";
  specialModeMacroNames[4][18] = "          ";
  specialModeMacroNames[4][19] = "          ";


  for (int i = 0; i < 5; i++)
  {
	  for (int j = 0; j < 14; j++)
	  {
		  presetNamesArray[63-i][j] = specialModeNames[i][j];
	  }
	  for (int k = 0; k < 20; k++)
	  {
		  for (int j = 0; j < 10; j++)
		  {
			  macroNamesArray[63-i][k][j] = specialModeMacroNames[i][k][j];
		  }
	  }
  }

  for (int i = 0; i < 20; i++)
   {
 	  prevKnobByte[i] = 256;
   }
	//now to send all the necessary messages to the codec

  //HAL_Delay(1000);
/*
  	if (foundOne == 0)
	{
	  //parsePreset((NUM_PARAMS*2)+27+(8*14), 0); //default preset binary
  		diskBusy = 1;
	}
	else
	{
	  parsePreset(presetWaitingToParse, presetNumberToLoad);
	}
*/

  HAL_SPI_Receive_DMA(&hspi5, SPI_PLUCK_RX, PLUCK_BUFFER_SIZE_TIMES_TWO);

  HAL_SPI_TransmitReceive_DMA(&hspi1, SPI_LEVERS_TX, SPI_LEVERS_RX, LEVER_BUFFER_SIZE_TIMES_TWO);




    audioStart(&hsai_BlockA1, &hsai_BlockB1);
	AudioCodec_init(&hi2c2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (boardNumber == 0)
	  {
		  if (presetWaitingToLoad > 0)
		  {

			  if (!i2cSending)
			  {

				  checkForSDCardPreset(presetNumberToLoad);
			  }

		  }
		  else if (presetWaitingToWrite > 0)
		  {
			  if (boardNumber == 0)
			  {
				  writePresetToSDCard(presetWaitingToWrite);
			  }
		  }
	  }
	  if (presetWaitingToParse > 0)
	  {
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		  parsePreset(presetWaitingToParse, presetNumberToLoad);
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	  }

	  if ((stringInputs[0] == 0) && (stringInputs[1] == 0))
	  {
	  	  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	  }
	  else
	  {
		  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	  }
	  if (brainFirmwareUpdateRequested)
	  {
		  checkForBootloadableBrainFile();
	  }
#if 0
	  if (pluckFirmwareUpdateRequested)
	  {
		  checkForBootloadablePluckFile();
	  }

	  if (copyingPluckToLocalArray)
	  {
		  localPluck[pluckIndex] = pluckFirmwareBuffer[pluckIndex];
		  pluckIndex++;
		  if (pluckIndex > 40000)
		  {
			  copyingPluckToLocalArray = 0;
			  foundPluckFirmware = 0;
			  pluckIndex = 0;
			  pluckFirmwareReadyToSend = 1;
		  }
	  }
#endif
	  uint32_t rand;
	  HAL_RNG_GenerateRandomNumber(&hrng, &rand);

	  if (rand > TWO_TO_31)
	  {
		  myTestInt++;
		  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
	  }
	  random_values[currentRandom++] = (float)rand * INV_TWO_TO_32 ;
	  //random_values[currentRandom++] = (floatrand * 2.0f) - 1.0f;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 344;
  PeriphClkInitStruct.PLL2.PLL2P = 7;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

float __ATTR_ITCMRAM randomNumber(void) {

	return random_values[currentRandom++];
}



uint8_t BSP_SD_IsDetected(void)
{
  __IO uint8_t status = SD_PRESENT;

  //if (BSP_PlatformIsDetected() == 0x0)
  //{
  //  status = SD_NOT_PRESENT;
  //}

  return status;
}

void getPresetNamesFromSDCard(void)
{
	if(BSP_SD_IsDetected())
	{
		for (int i = 0; i < AUDIO_BUFFER_SIZE; i+=2)
		{
			audioOutBuffer[i] = 0;
			audioOutBuffer[i + 1] = 0;
		}
		diskBusy = 1;

		loadFailed = 0;
		//HAL_Delay(300);

		disk_initialize(0);

	    disk_status(0);

		if(f_mount(&SDFatFS,  SDPath, 1) == FR_OK)
		{

			FRESULT res;
			/* Start to search for preset files */


			//turn the integer value into a 2 digit string
			char charBuf[10];
			char finalString[10];

			for(int i = 0; i < MAX_NUM_PRESETS; i++)
			{
				itoa(i, charBuf, 10);
				int len = ((strlen(charBuf)));
				if (len == 1)
				{
					finalString[2] = charBuf[1];
					finalString[1] = charBuf[0];
					finalString[0] = '0';
					strcat(finalString, "*.ebp");
				}

				else
				{
					strcat(charBuf, "*.ebp");
					strcpy(finalString, charBuf);
				}


				res = f_findfirst(&dir, &fno, SDPath, finalString);
				uint32_t bytesRead;
				if(res == FR_OK)
				{
					if(f_open(&SDFile, fno.fname, FA_OPEN_ALWAYS | FA_READ) == FR_OK)
					{
						f_read(&SDFile, &buffer, f_size(&SDFile), &bytesRead);
						f_close(&SDFile);
						uint16_t bufferIndex = 0;
						//skip the first 4 bytes if there is a version number stored in the preset
						if (buffer[bufferIndex] == 17)
						{
							bufferIndex = 4;
						}
						//14-byte name
						for (int j = 0; j < 14; j++)
						{
							presetNamesArray[i][j] = buffer[bufferIndex];
							bufferIndex++;
						}
						//9-byte macros
						for (int j = 0; j < 8; j++)
						{
							for (int k = 0; k < 9; k++)
							{
								macroNamesArray[i][j][k] = buffer[bufferIndex];
								bufferIndex++;
							}
						}
						//10-byte macros
						for (int j = 0; j < 4; j++)
						{
							for (int k = 0; k < 10; k++)
							{
								macroNamesArray[i][j+8][k] = buffer[bufferIndex];
								bufferIndex++;
							}
						}
					}
				}
			}

		}

	}
	diskBusy = 0;
	return;
}

static int checkForSDCardPreset(uint8_t numberToLoad)
{
	int found = 0;
	prevVoice = numberToLoad;
	voice = numberToLoad;
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	if(BSP_SD_IsDetected())
	{
		for (int i = 0; i < AUDIO_BUFFER_SIZE; i+=2)
		{
			audioOutBuffer[i] = 0;
			audioOutBuffer[i + 1] = 0;
		}
		diskBusy = 1;
		loadFailed = 0;
		//HAL_Delay(300);
		presetWaitingToLoad = 0;
		disk_initialize(0);

	    disk_status(0);

		if(f_mount(&SDFatFS,  SDPath, 1) == FR_OK)
		{

			FRESULT res;
			/* Start to search for preset files */
			char charBuf[10];
			char finalString[10];

			//turn the integer value into a 2 digit string

			itoa(numberToLoad, charBuf, 10);
			int len = ((strlen(charBuf)));
			if (len == 1)
			{
				finalString[2] = charBuf[1];
				finalString[1] = charBuf[0];
				finalString[0] = '0';
				strcat(finalString, "*.ebp");
			}

			else
			{
				strcat(charBuf, "*.ebp");
				strcpy(finalString, charBuf);
			}

			res = f_findfirst(&dir, &fno, SDPath, finalString);
			uint32_t bytesRead;
			if(res == FR_OK)
			{
				if(f_open(&SDFile, fno.fname, FA_OPEN_ALWAYS | FA_READ) == FR_OK)
				{
					f_read(&SDFile, &buffer, f_size(&SDFile), &bytesRead);
					presetWaitingToParse = bytesRead;
					f_close(&SDFile);
					found = 1;
				}
			}
		}
	}
	if (!found)
	{
		loadFailed = 1;
	}
	//if you succeeded, send the data to the other boards
	else
	{


		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	  	 // HAL_Delay(1);
	  	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	  	 // HAL_Delay(1);
	  	  i2cSending = 1;
	  	__disable_irq();
	  	  //HAL_I2C_Master_Transmit(&hi2c1, 34<<1, buffer, 4096, 10000);
	  	__enable_irq();
	  	i2cSending = 0;
	}

	diskBusy = 0;
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	return found;
}




static void checkForBootloadableBrainFile(void)
{
	if (boardNumber == 0)
	{
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		if(BSP_SD_IsDetected())
		{
			for (uint_fast16_t i = 0; i < AUDIO_BUFFER_SIZE; i+=2)
			{
				audioOutBuffer[i] = 0;
				audioOutBuffer[i + 1] = 0;
			}
			diskBusy = 1;
			loadFailed = 0;

			disk_initialize(0);

			disk_status(0);
			uint_fast32_t  bytesRead;
			if(f_mount(&SDFatFS,  SDPath, 1) == FR_OK)
			{

				FRESULT res;
				/* Start to search for firmware files */
				char finalString[10] = "brain.bin";
				//Read Brain Firmware
				//strcat(finalString, "brain.bin");

				res = f_findfirst(&dir, &fno, SDPath, finalString);

				if(res == FR_OK)
				{
					if(f_open(&SDFile, fno.fname, FA_OPEN_ALWAYS | FA_READ) == FR_OK)
					{
						brainFirmwareSize = f_size(&SDFile);

						f_read(&SDFile, &brainFirmwareBuffer, brainFirmwareSize, &bytesRead);
						f_close(&SDFile);

						for (uint_fast16_t i = 0; i< 700; i++)
						{
							memoryTest[i] = brainFirmwareBuffer[i];
						}
						foundBrainFirmware = 1;
						brainFirmwareBufferIndex = 0;
					}
				}
			}
		}
		brainFirmwareUpdateRequested = 0;


		diskBusy = 0;
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	}
}

static void writePresetToSDCard(int fileSize)
{
	__disable_irq();
	 for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	 {
		 audioOutBuffer[i] = 0;
	 }
	if(BSP_SD_IsDetected())
	{
		//if(f_mount(&SDFatFS,  SDPath, 1) == FR_OK)
		{
			//if(res == FR_OK)
			{
				for (int i = 0; i < AUDIO_BUFFER_SIZE; i+=2)
				{
					audioOutBuffer[i] = 0;
					audioOutBuffer[i + 1] = 0;
				}
				diskBusy = 1;
				//make sure the number is not above 2 digits
			    if (presetNumberToSave > 99)
			    {
			        presetNumberToSave = 99;
			    }
			    //first, delete any existing presets that share the same number
			    FRESULT res;
				/* Start to search for preset files */
				char charBufC[10];
				char finalStringC[10];

				itoa(presetNumberToSave, charBufC, 10);
				int len = ((strlen(charBufC)));
				if (len == 1)
				{
					finalStringC[2] = charBufC[1];
					finalStringC[1] = charBufC[0];
					finalStringC[0] = '0';
					strcat(finalStringC, "*.ebp");
				}

				else
				{
					strcat(charBufC, "*.ebp");
					strcpy(finalStringC, charBufC);
				}

				uint32_t keepChecking = 1;
				while(keepChecking)
				{
					res = f_findfirst(&dir, &fno, SDPath, finalStringC);

					//delete if found
					if((res == FR_OK) && (fno.fname[0]))
					{
						f_unlink (fno.fname);
					}
					else
					{
						keepChecking = 0;
					}
				}

			    //turn the integer value into a 2 digit string
				char charBuf[22];
				char finalString[22];
				itoa(presetNumberToSave, charBuf, 10);
				len = ((strlen(charBuf)));
				if (len == 1)
				{
					finalString[21] = 0;
					finalString[20] = 'p';
					finalString[19] = 'b';
					finalString[18] = 'e';
					finalString[17] = '.';
					for (int i = 0; i < 14; i++)
					{
						finalString[i+3] = buffer[i+4];
						//replace spaces with underscores for filename
						if (finalString[i+3] == 32)
						{
							finalString[i+3] = '_';
						}
					}
					finalString[2] = '_';
					finalString[1] = charBuf[0];
					finalString[0] = '0';

				}

				else
				{
					finalString[21] = 0;
					finalString[20] = 'p';
					finalString[19] = 'b';
					finalString[18] = 'e';
					finalString[17] = '.';
					for (int i = 0; i < 14; i++)
					{
						finalString[i+3] = buffer[i+4];
						//replace spaces with underscores for filename
						if (finalString[i+3] == 32)
						{
							finalString[i+3] = '_';
						}
					}
					finalString[2] = '_';
					finalString[1] = charBuf[1];
					finalString[0] = charBuf[0];


				}

				if(f_open(&SDFile, finalString, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
				{
					uint32_t bytesRead;
					f_write(&SDFile, &buffer, fileSize, &bytesRead);
					f_close(&SDFile);
				}

			}
			//f_mount(0, "", 0); //unmount
		}
	}
	presetWaitingToWrite = 0;
	diskBusy = 0;
	__enable_irq();
}


//excellent resource on sdram configuration in cubemx
//https://community.st.com/s/article/How-to-set-up-the-FMC-peripheral-to-interface-with-the-SDRAM-IS42S16800F-6BLI-from-ISSI
//datasheet for memory used on daisy = https://www.issi.com/WW/pdf/42-45SM-RM-VM32160E.pdf

#define SDRAM_MODEREG_BURST_LENGTH_2 ((1 << 0))
#define SDRAM_MODEREG_BURST_LENGTH_4 ((1 << 1))

#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL ((0 << 3))

#define SDRAM_MODEREG_CAS_LATENCY_2 ((1 << 5))

#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE ((1 << 9))
#define SDRAM_MODEREG_WRITEBURST_MODE_PROG_BURST ((0 << 9))

#define SDRAM_MODEREG_OPERATING_MODE_STANDARD ((0 << 13)|(0 << 14))

void SDRAM_init()
{
	    	FMC_SDRAM_CommandTypeDef Command;

	        __IO uint32_t tmpmrd = 0;
	        /* Step 3:  Configure a clock configuration enable command */
	        Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
	        Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	        Command.AutoRefreshNumber      = 1;
	        Command.ModeRegisterDefinition = 0;

	        /* Send the command */
	        HAL_SDRAM_SendCommand(&hsdram1, &Command, 0x1000);

	        /* Step 4: Insert 100 us delay */
	        HAL_Delay(1);


	        /* Step 5: Configure a PALL (precharge all) command */
	        Command.CommandMode            = FMC_SDRAM_CMD_PALL;
	        Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	        Command.AutoRefreshNumber      = 1;
	        Command.ModeRegisterDefinition = 0;

	        /* Send the command */
	        HAL_SDRAM_SendCommand(&hsdram1, &Command, 0x1000);

	        /* Step 6 : Configure a Auto-Refresh command */
	        Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	        Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	        Command.AutoRefreshNumber      = 2;
	        Command.ModeRegisterDefinition = 0;

	        /* Send the command */
	        HAL_SDRAM_SendCommand(&hsdram1, &Command, 0x1000);

	        /* Step 7: Program the external memory mode register */
	        tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_4
	                 | SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL | SDRAM_MODEREG_CAS_LATENCY_2
	                 | SDRAM_MODEREG_WRITEBURST_MODE_SINGLE | SDRAM_MODEREG_OPERATING_MODE_STANDARD;
	        // // Used in example, but can't find reference except for "Test Mode"

	        Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
	        Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
	        Command.AutoRefreshNumber      = 1;
	        Command.ModeRegisterDefinition = tmpmrd;

	        /* Send the command */
	        HAL_SDRAM_SendCommand(&hsdram1, &Command, 0x1000);

	        //HAL_SDRAM_ProgramRefreshRate(hsdram, 0x56A - 20);
	        HAL_SDRAM_ProgramRefreshRate(&hsdram1, 762); // ((64ms / 8192) * 100MHz) - 20
	        //8192 is 2^numberofrows (which is 13 in the case of the sdram)

}

float __ATTR_ITCMRAM scaleDefault(float input)
{
	//input = LEAF_clip(0.f, input, 1.f);
	return input;
}

float __ATTR_ITCMRAM scaleTwo(float input)
{
	//input = LEAF_clip(0.f, input, 1.f);
	return (input * 2.0f);
}

float __ATTR_ITCMRAM scaleOscPitch(float input)
{
	//input = LEAF_clip(0.0f, input, 1.0f);
	return ((input * 2.0f) - 1.0f);
}

float __ATTR_ITCMRAM scaleOscFine(float input)
{
	//input = LEAF_clip(0.0f, input, 1.f);
	return (input * 200.0f) - 100.0f;
}

float __ATTR_ITCMRAM scaleOscFreq(float input)
{
	//input = LEAF_clip(0.f, input, 1.f);
	return (input * 4000.0f) - 2000.0f;
}

float __ATTR_ITCMRAM scaleTranspose(float input)
{
	input = LEAF_clip(0.0f, input, 1.f);
	return (input * 96.0f) - 48.0f;
}

float __ATTR_ITCMRAM scalePitchBend(float input)
{
	input = LEAF_clip(0.f, input, 1.f);
	return (input * 48.0f);
}

float __ATTR_ITCMRAM scaleFilterCutoff(float input)
{
	//input = LEAF_clip(0.f, input, 1.f);
	return (input * 127.0f);
}

float __ATTR_ITCMRAM scaleFilterResonance(float input)
{
	//lookup table for filter res
	//input = LEAF_clip(0.1f, input, 1.0f);
	//scale to lookup range
	input *= 2047.0f;
	int inputInt = (int)input;
	float inputFloat = (float)inputInt - input;
	int nextPos = LEAF_clip(0, inputInt + 1, 2047);
	return LEAF_clip(0.1f, (resTable[inputInt] * (1.0f - inputFloat)) + (resTable[nextPos] * inputFloat), 10.0f);
	//return
}

float __ATTR_ITCMRAM scaleEnvTimes(float input)
{
	//lookup table for env times
	//input = LEAF_clip(0.0f, input, 1.0f);
	//scale to lookup range
	input *= 2047.0f;
	int inputInt = (int)input;
	float inputFloat = (float)inputInt - input;
	int nextPos = LEAF_clip(0, inputInt + 1, 2047);
	return (envTimeTable[inputInt] * (1.0f - inputFloat)) + (envTimeTable[nextPos] * inputFloat);

	//return
}

float __ATTR_ITCMRAM scaleLFORates(float input)
{
	//lookup table for LFO rates
	//input = LEAF_clip(0.0f, input, 1.0f);
	//scale to lookup range
	input *= 2047.0f;
	int inputInt = (int)input;
	float inputFloat = (float)inputInt - input;
	int nextPos = LEAF_clip(0, inputInt + 1, 2047);
	return (lfoRateTable[inputInt] * (1.0f - inputFloat)) + (lfoRateTable[nextPos] * inputFloat);
	//return
}

float __ATTR_ITCMRAM scaleFinalLowpass(float input)
{
	//input = LEAF_clip(0.f, input, 1.f);
	return ((input * 70.0f) + 58.0f);
}


void __ATTR_ITCMRAM blankFunction(float a, int b, int c)
{
	;
}

volatile uint8_t chorusAssignment = 255;
volatile uint8_t delayAssignment = 255;

void setEffectsFunctions(FXType effectType, int i)
{
	effectsActive[i] = 1;
	switch (effectType)
	{
		  case None:
			  effectTick[i] = &blankTick;
			  effectSetters[i].setParam1 = &blankFunction;
			  effectSetters[i].setParam2 = &blankFunction;
			  effectSetters[i].setParam3 = &blankFunction;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;
			  effectsActive[i] = 0;
			  break;
		  case Softclip:
			  effectTick[i] = &softClipTick;
			  effectSetters[i].setParam1 = &clipperGainSet;
			  effectSetters[i].setParam2 = &offsetParam2;
			  effectSetters[i].setParam3 = &param3Soft;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;

			  break;
		  case Hardclip:
			  effectTick[i] = &hardClipTick;
			  effectSetters[i].setParam1 = &clipperGainSet;
			  effectSetters[i].setParam2 = &offsetParam2;
			  effectSetters[i].setParam3 = &param3Hard;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;
			  break;
		  case PolynomialShaper:
			  effectTick[i] = &polynomialShaperTick;
			  effectSetters[i].setParam1 = &clipperGainSet;
			  effectSetters[i].setParam2 = &offsetParam2;
			  effectSetters[i].setParam3 = &param3Poly;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;
			  break;
		  case ABSaturator:
			  effectTick[i] = &satTick;
			  effectSetters[i].setParam1 = &clipperGainSet;
			  effectSetters[i].setParam2 = &offsetParam2;
			  effectSetters[i].setParam3 = &blankFunction;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;
			  break;
		  case Tanh:
			  effectTick[i] = &tanhTick;
			  effectSetters[i].setParam1 = &clipperGainSet;
			  effectSetters[i].setParam2 = &offsetParam2;
			  effectSetters[i].setParam3 = &blankFunction;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;
			  break;
		  case Shaper2:
			  effectTick[i] = &shaperTick;
			  effectSetters[i].setParam1 = &param1Linear;
			  effectSetters[i].setParam2 = &offsetParam2;
			  effectSetters[i].setParam3 = &param3Linear;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;
			  break;
		  case Compressor:
			  effectTick[i] = &compressorTick;
			  effectSetters[i].setParam1 = &compressorParam1;
			  effectSetters[i].setParam2 = &compressorParam2;
			  effectSetters[i].setParam3 = &compressorParam3;
			  effectSetters[i].setParam4 = &compressorParam4;
			  effectSetters[i].setParam5 = &compressorParam5;
			  break;
		  case Chorus:
			  //there isn't enough memory for multiple chorus effects. Disable any other ticks using them
			  if (chorusAssignment != 255)
			  {
				  effectTick[chorusAssignment] = &blankTick;
				  effectSetters[chorusAssignment].setParam1 = &blankFunction;
				  effectSetters[chorusAssignment].setParam2 = &blankFunction;
				  effectSetters[chorusAssignment].setParam3 = &blankFunction;
				  effectSetters[chorusAssignment].setParam4 = &blankFunction;
				  effectSetters[chorusAssignment].setParam5 = &blankFunction;
			  }
			  chorusAssignment = i;
			  effectTick[i] = &chorusTick;
			  effectSetters[i].setParam1 = &chorusParam1;
			  effectSetters[i].setParam2 = &chorusParam2;
			  effectSetters[i].setParam3 = &chorusParam3;
			  effectSetters[i].setParam4 = &chorusParam4;
			  effectSetters[i].setParam5 = &blankFunction;
			  break;
		  case Bitcrush:
			  effectTick[i] = &bcTick;
			  effectSetters[i].setParam1 = &clipperGainSet;
			  effectSetters[i].setParam2 = &param2BC;
			  effectSetters[i].setParam3 = &param3BC;
			  effectSetters[i].setParam4 = &param4BC;
			  effectSetters[i].setParam5 = &param5BC;
			  break;
		  case TiltFilter:
			  effectTick[i] = &tiltFilterTick;
			  effectSetters[i].setParam1 = &tiltParam1;
			  effectSetters[i].setParam2 = &tiltParam2;
			  effectSetters[i].setParam3 = &tiltParam3;
			  effectSetters[i].setParam4 = &tiltParam4;
			  effectSetters[i].setParam5 = &param5Linear;
			  break;
		  case Wavefolder:
			  effectTick[i] = &wavefolderTick;
			  effectSetters[i].setParam1 = &wavefolderParam1;
			  effectSetters[i].setParam2 = &offsetParam2;
			  effectSetters[i].setParam3 = &wavefolderParam3;
			  effectSetters[i].setParam4 = &param4Linear;
			  effectSetters[i].setParam5 = &param5Linear;
			  break;

		  case Delay:
			  //there isn't enough memory for multiple delay effects. Disable any other ticks using them
			  if (delayAssignment != 255)
			  {
				  effectTick[delayAssignment] = &blankTick;
				  effectSetters[delayAssignment].setParam1 = &blankFunction;
				  effectSetters[delayAssignment].setParam2 = &blankFunction;
				  effectSetters[delayAssignment].setParam3 = &blankFunction;
				  effectSetters[delayAssignment].setParam4 = &blankFunction;
				  effectSetters[delayAssignment].setParam5 = &blankFunction;
			  }
			  delayAssignment = i;
			  effectTick[i] = &delayTick;
			  effectSetters[i].setParam1 = &delayParam1;
			  effectSetters[i].setParam2 = &delayParam2;
			  effectSetters[i].setParam3 = &delayParam3;
			  effectSetters[i].setParam4 = &delayParam4;
			  effectSetters[i].setParam5 = &delayParam5;
			  break;

		  case FXLowpass :
			  effectTick[i] = &FXlowpassTick;
			  effectSetters[i].setParam1 = &FXLowpassParam1;
			  effectSetters[i].setParam2 = &blankFunction;
			  effectSetters[i].setParam3 = &FXLowpassParam3;
			  effectSetters[i].setParam4 = &blankFunction;;
			  effectSetters[i].setParam5 = &blankFunction;;
			  break;
		  case FXHighpass :
			  effectTick[i] = &FXhighpassTick;
			  effectSetters[i].setParam1 = &FXHighpassParam1;
			  effectSetters[i].setParam2 = &blankFunction;
			  effectSetters[i].setParam3 = &FXHighpassParam3;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;
			  break;
		  case FXBandpass :
			  effectTick[i] = &FXbandpassTick;
			  effectSetters[i].setParam1 = &FXBandpassParam1;
			  effectSetters[i].setParam2 = &blankFunction;
			  effectSetters[i].setParam3 = &FXBandpassParam3;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;
			  break;
		  case FXDiode :
			  effectTick[i] = &FXdiodeLowpassTick;
			  effectSetters[i].setParam1 = &FXDiodeParam1;
			  effectSetters[i].setParam2 = &blankFunction;
			  effectSetters[i].setParam3 = &FXDiodeParam3;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;
			  break;
		  case FXPeak :
			  effectTick[i] = &FXVZpeakTick;
			  effectSetters[i].setParam1 = &FXPeakParam1;
			  effectSetters[i].setParam2 = &FXPeakParam2;
			  effectSetters[i].setParam3 = &FXPeakParam3;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;
			  break;
		  case FXLowShelf :
			  effectTick[i] = &FXVZlowshelfTick;
			  effectSetters[i].setParam1 = &FXLowShelfParam1;
			  effectSetters[i].setParam2 = &FXLowShelfParam2;
			  effectSetters[i].setParam3 = &FXLowShelfParam3;
			  effectSetters[i].setParam4 = &blankFunction;
			  effectSetters[i].setParam5 = &blankFunction;
			  break;
		  case FXHighShelf :
			  effectTick[i] = FXVZhighshelfTick;
			  effectSetters[i].setParam1 = &FXHighShelfParam1;;
			  effectSetters[i].setParam2 = &FXHighShelfParam2;;
			  effectSetters[i].setParam3 = &FXHighShelfParam3;;
			  effectSetters[i].setParam4 = &blankFunction;;
			  effectSetters[i].setParam5 = &blankFunction;;
			  break;
		  case FXNotch :
			  effectTick[i] = FXVZbandrejectTick;
			  effectSetters[i].setParam1 = &FXNotchParam1;;
			  effectSetters[i].setParam2 = &FXNotchParam2;;
			  effectSetters[i].setParam3 = &FXNotchParam3;;
			  effectSetters[i].setParam4 = &blankFunction;;
			  effectSetters[i].setParam5 = &blankFunction;;
			  break;
		  case FXLadder :
			  effectTick[i] = &FXLadderLowpassTick;
			  effectSetters[i].setParam1 = &FXLadderParam1;;
			  effectSetters[i].setParam2 = &blankFunction;;
			  effectSetters[i].setParam3 = &FXLadderParam3;;
			  effectSetters[i].setParam4 = &blankFunction;;
			  effectSetters[i].setParam5 = &blankFunction;;
			  break;
		  default:
			  break;
	}
}

void setOscilllatorShapes(int oscshape, int i)
{
	switch (oscshape)
	{
		  case 0:
			  shapeTick[i] = &sawSquareTick;
			  break;
		  case 1:
			  shapeTick[i] = &sineTriTick;
			  break;
		  case 2:
			  shapeTick[i] = &sawTick;
			  break;
		  case 3:
			  shapeTick[i] = &pulseTick;
			  break;
		  case 4:
			  shapeTick[i] = &sineTick;
			  break;
		  case 5:
			  shapeTick[i] = &triTick;
			  break;
		  //case 6:
			  //shapeTick[i] = &userTick;
			  //break;
		  default:
			  break;
	}
}
void setFilterTypes(int filterType, int i)
{
	switch (filterType)
		{
			  case 0:
				  filterTick[i] = &lowpassTick;
				  filterSetters[i].setQ = &lowpassSetQ;
				  filterSetters[i].setGain = &lowpassSetGain;
				  break;
			  case 1:
				  filterTick[i] = &highpassTick;
				  filterSetters[i].setQ = &highpassSetQ;
				  filterSetters[i].setGain = &highpassSetGain;
				  break;
			  case 2:
				  filterTick[i] = &bandpassTick;
				  filterSetters[i].setQ = &bandpassSetQ;
				  filterSetters[i].setGain = &bandpassSetGain;
				  break;
			  case 3:
				  filterTick[i] = &diodeLowpassTick;
				  filterSetters[i].setQ = &diodeLowpassSetQ;
				  filterSetters[i].setGain = &diodeLowpassSetGain;
				  break;
			  case 4:
				  filterTick[i] = &VZpeakTick;
				  filterSetters[i].setQ = &VZpeakSetQ;
				  filterSetters[i].setGain = &VZpeakSetGain;
				  break;
			  case 5:
				  filterTick[i] = &VZlowshelfTick;
				  filterSetters[i].setQ = &VZlowshelfSetQ;
				  filterSetters[i].setGain = &VZlowshelfSetGain;
				  break;
			  case 6:
				  filterTick[i] = &VZhighshelfTick;
				  filterSetters[i].setQ = &VZhighshelfSetQ;
				  filterSetters[i].setGain = &VZhighshelfSetGain;
				  break;
			  case 7:
				  filterTick[i] = &VZbandrejectTick;
				  filterSetters[i].setQ = &VZbandrejectSetQ;
				  filterSetters[i].setGain = &VZbandrejectSetGain;
				  break;
			  case 8:
				  filterTick[i] = &LadderLowpassTick;
				  filterSetters[i].setQ = &LadderLowpassSetQ;
				  filterSetters[i].setGain = &LadderLowpassSetGain;
				  break;
			  default:
				  break;
		}
}

void setLFOShapes(int LFOShape, int i)
{
	switch(LFOShape)
	{
		case SineTriLFOShapeSet:
			lfoShapeTick[i] = &lfoSineTriTick;
			lfoSetters[i].setRate = &lfoSineTriSetRate;
			lfoSetters[i].setShape = &lfoSineTriSetShape;
			lfoSetters[i].setPhase = &lfoSineTriSetPhase;
			break;
		case SawPulseLFOShapeSet:
			lfoShapeTick[i] = &lfoSawSquareTick;
			lfoSetters[i].setRate = &lfoSawSquareSetRate;
			lfoSetters[i].setShape = &lfoSawSquareSetShape;
			lfoSetters[i].setPhase = &lfoSawSquareSetPhase;
			break;
		case SineLFOShapeSet:
			lfoShapeTick[i] = &lfoSineTick;
			lfoSetters[i].setRate = &lfoSineSetRate;
			lfoSetters[i].setShape = &lfoSineSetShape;
			lfoSetters[i].setPhase = &lfoSineSetPhase;
			break;
		case TriLFOShapeSet:
			lfoShapeTick[i] = &lfoTriTick;
			lfoSetters[i].setRate = &lfoTriSetRate;
			lfoSetters[i].setShape = &lfoTriSetShape;
			lfoSetters[i].setPhase = &lfoTriSetPhase;
			break;
		case SawLFOShapeSet:
			lfoShapeTick[i] = &lfoSawTick;
			lfoSetters[i].setRate = &lfoSawSetRate;
			lfoSetters[i].setShape = &lfoSawSetShape;
			lfoSetters[i].setPhase = &lfoSawSetPhase;
			break;
		case PulseLFOShapeSet:
			lfoShapeTick[i] = &lfoPulseTick;
			lfoSetters[i].setRate = &lfoPulseSetRate;
			lfoSetters[i].setShape = &lfoPulseSetShape;
			lfoSetters[i].setPhase = &lfoPulseSetPhase;
			break;
	}
}


uint8_t rowNumber = 0;
uint8_t arrayNumber = 0;
uint16_t positionInRowLine = 0;

uint8_t fromHex(char value)
{
	if (('0' <= value) && (value <= '9'))
		return (uint8_t) (value - '0');
	if (('a' <= value) && (value <= 'f'))
		return (uint8_t) (10 + value - 'a');
	if (('A' <= value) && (value <= 'F'))
		return (uint8_t) (10 + value - 'A');
	return 0;
}
uint8_t fromAscii(uint8_t input1, uint8_t input2)
{
	return ((fromHex(input1)<<4) | (fromHex(input2)));
}

volatile uint32_t tested[32];
void  handleSPI (uint8_t offset)
{
	interruptChecker = 1;

	if (foundBrainFirmware)
	{
		if (brainFirmwareEndSignal)
		{
			SPI_LEVERS_TX[offset] = 249; //special byte that says I'm done sending you firmware, you doofus, now use it;
			SPI_LEVERS_TX[offset+1] = brainFirmwareSize >> 24;
			SPI_LEVERS_TX[offset+2] = brainFirmwareSize >> 16;
			SPI_LEVERS_TX[offset+3] = brainFirmwareSize >> 8;
			SPI_LEVERS_TX[offset+4] = brainFirmwareSize && 0xff;
			SPI_LEVERS_TX[offset+31] = 254;
			brainFirmwareEndSignal = 0;
			brainFirmwareSendInProgress = 0;
			foundBrainFirmware = 0;
		}
		else if (brainFirmwareSendInProgress)
		{
			uint8_t rowEnded = 0;
			SPI_LEVERS_TX[offset] = 251; //special byte that says it's a firmware chunk
			for (int i = 0; i < 30; i++)
			{
				//go byte by byte and parse them out of the ascii hex values
				uint8_t val1 = brainFirmwareBuffer[brainFirmwareBufferIndex];
				uint8_t val2 = brainFirmwareBuffer[brainFirmwareBufferIndex+1];
				uint8_t valToSend = fromAscii(val1, val2);

				if (positionInRowLine < 294)
				{
					SPI_LEVERS_TX[offset+i+1] = valToSend;
				}
				else
				{
					rowEnded = 1;
				}
				positionInRowLine++;
				if (rowEnded == 0)
				{
					brainFirmwareBufferIndex += 2;
				}
				else
				{
					positionInRowLine = 0;
					for (uint8_t j = 0; j<10; j++)
					{
						if (brainFirmwareBuffer[brainFirmwareBufferIndex+j] == 0x3a)
						{
							brainFirmwareBufferIndex = brainFirmwareBufferIndex+j+1; // start after the header, so it's the first real byte after the ":"
						}
					}
					rowEnded = 0;
					i-=1;//push i back one because otherwise it increments even though we didn't send, just prepped for next send
				}
			}
			SPI_LEVERS_TX[offset+31] = 254;
			if (brainFirmwareBufferIndex >= brainFirmwareSize)
			{
				brainFirmwareEndSignal = 1;
			}
		}
		else
		{
			SPI_LEVERS_TX[offset] = 252; //special byte that says I'm gonna send you new firmware so reboot into bootloader;
			brainFirmwareSendInProgress = 1;
			for (uint8_t i = 0; i<100; i++)
			{
				if (brainFirmwareBuffer[i] ==  0x3a)
				{
					brainFirmwareBufferIndex = i+1; // start after the header, so it's the first real byte after the ":"
				}
			}
			positionInRowLine = 0;
		}
	}
	else
	{
		// if the first number is a 1 then it's a midi note/ctrl/bend message
		if (SPI_LEVERS_RX[offset] == ReceivingPitches)
		{
			 uint8_t currentByte = offset+1;
			 for (int i = 0; i < numStringsThisBoard; i++)
			 {
				float myPitch = (float)(SPI_LEVERS_RX[((i+firstString) * 2) + currentByte] << 8) + SPI_LEVERS_RX[((i+firstString) * 2) + 1 + currentByte];
				myPitch = myPitch * 0.001953154802777f; //(128 / 65535) scale the 16 bit integer into midinotes.
				if ((myPitch > 0.0f) && (myPitch < 140.0f))
				{
					stringMIDIPitches[i] = myPitch;
				}
			 }
			 //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
			 whichBar = 0;
			 updateStateFromSPIMessage(offset);
			 //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
			 //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		}
		// if the first number is a 2 then it's a preset write
		else if (SPI_LEVERS_RX[offset] == ReceivingPreset)
		{
			//got a new preset to write to memory
			 //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);

			 //if you aren't already writing a preset to memory, start the process
			 if (writingState != ReceivingPreset)
			 {
				 writingState = ReceivingPreset; // set the flag to let the mcu know that a preset write is in progress
					for (int i = 0; i < AUDIO_BUFFER_SIZE; i+=2)
					{
						audioOutBuffer[i] = 0;
						audioOutBuffer[i + 1] = 0;
					}
				 diskBusy = 1;
				 audioMasterLevel = 0.0f;
				 //write the raw data as a preset number on the SD card
				 bufferPos = 0;
			 }
			 presetNumberToSave = SPI_LEVERS_RX[offset + 1];
			 uint8_t currentByte = offset+2; // first number says what it is 2nd number says which number it is

			 for (int i = 0; i < 28; i++)
			 {
				 buffer[bufferPos++] = SPI_LEVERS_RX[currentByte + i];

			 }
			 //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		}

		else if (SPI_LEVERS_RX[offset] == ReceivingKnobs)
		{
			uint8_t currentByte = offset+1;
			for (int i = 0; i < 8; i++)
			{
				int32_t newByte = SPI_LEVERS_RX[i + currentByte];
				if (knobFrozen[i])
				{
					if ((newByte > (prevKnobByte[i] + 3)) || (newByte < (prevKnobByte[i] - 3)))
					{
						knobFrozen[i] = 0;
						prevKnobByte[i] = newByte;
					}
				}
				else
				{
					tExpSmooth_setDest(knobSmoothers[i], (SPI_LEVERS_RX[i + currentByte] * 0.003921568627451f)); //scaled 0.0 to 1.0
					prevKnobByte[i] = newByte;
				}

			}

			for (int i = 8; i < 12; i++)
			{
				int32_t newByte = SPI_LEVERS_RX[i + currentByte];

				if (knobFrozen[i])
				{
					if ((newByte > (prevKnobByte[i] + 3)) || (newByte < (prevKnobByte[i] - 3)))
					{
						knobFrozen[i] = 0;
						prevKnobByte[i] = newByte;
					}

				}
				else
				{
					tExpSmooth_setDest(knobSmoothers[i], (newByte * 0.003921568627451f)); //scaled 0.0 to 1.0
					prevKnobByte[i] = newByte;
				}

			}
			currentByte += 12;
			for (int i = 0; i < 10; i++)
			{
				tExpSmooth_setDest(pedalSmoothers[i], (SPI_LEVERS_RX[i + currentByte ] * 0.003921568627451f)); //scaled 0.0 to 1.0
			}
			//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
			whichBar = 1;
			updateStateFromSPIMessage(offset);
			//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);


		}

		else if (SPI_LEVERS_RX[offset] == ReceivingAdditionalKnobs)
		{
			 uint8_t currentByte = offset+1;

			for (int i = 0; i < 8; i++)
			{

				uint32_t whichKnob = i+12;
				int32_t newByte = SPI_LEVERS_RX[i + currentByte];
				if (knobFrozen[whichKnob])
				{
					if ((newByte > (prevKnobByte[whichKnob] + 3)) || (newByte < (prevKnobByte[whichKnob] - 3)))
					{
						knobFrozen[whichKnob] = 0;
						prevKnobByte[whichKnob] = newByte;
					}

				}
				else
				{
					tExpSmooth_setDest(knobSmoothers[whichKnob], (newByte * 0.003921568627451f)); //scaled 0.0 to 1.0
					prevKnobByte[whichKnob] = newByte;
				}

			}
			for (int i = 8; i < 12; i++)
			{
				int32_t newByte = SPI_LEVERS_RX[i + currentByte];
				uint32_t whichKnob = i;
				if (knobFrozen[whichKnob])
				{
					if ((newByte > (prevKnobByte[whichKnob] + 3)) || (newByte < (prevKnobByte[whichKnob] - 3)))
					{
						knobFrozen[whichKnob] = 0;
						prevKnobByte[whichKnob] = newByte;
					}

				}
				else
				{
					tExpSmooth_setDest(knobSmoothers[whichKnob], (SPI_LEVERS_RX[i + currentByte] * 0.003921568627451f)); //scaled 0.0 to 1.0
					prevKnobByte[whichKnob] = newByte;
				}

			}

			currentByte += 12;
			for (int i = 0; i < 10; i++)
			{
				tExpSmooth_setDest(pedalSmoothers[i], (SPI_LEVERS_RX[i + currentByte ] * 0.003921568627451f)); //scaled 0.0 to 1.0
			}
			//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
			whichBar = 1;
			updateStateFromSPIMessage(offset);
			//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);


		}
		else if (SPI_LEVERS_RX[offset] == ReceivingEnd)
		{
			if(writingState == ReceivingPreset)
			{
				 writingState = 0;
				 presetNumberToLoad = presetNumberToSave;
				 /* Parse into Audio Params */
				 presetWaitingToParse = bufferPos;
				 presetWaitingToWrite = bufferPos;
			}
		}

		else if (SPI_LEVERS_RX[offset] == ReceivingSingleParamChange)
		{

			if (presetReady)
			{

				uint8_t currentByte = offset+1;

				uint16_t whichParam = ((SPI_LEVERS_RX[currentByte]<< 8) + SPI_LEVERS_RX[currentByte+1]);
				currentByte = currentByte + 2;


				for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
				{
					//get the zero-to-one-value
					params[whichParam].zeroToOneVal[v] = INV_TWO_TO_16 * ((SPI_LEVERS_RX[currentByte] << 8) + SPI_LEVERS_RX[currentByte+1]);
				}

				if ((whichParam == Effect1FXType) || (whichParam == Effect2FXType) || (whichParam == Effect3FXType) || (whichParam == Effect4FXType))
				{
					uint8_t whichEffect = (whichParam - Effect1FXType) / EffectParamsNum;
					FXType effectType = roundf(params[whichParam].zeroToOneVal[0] * (NUM_EFFECT_TYPES-1));
					param *FXAlias = &params[whichParam + 1];

					if (effectType > FXLowpass)
					{
						FXAlias[2].scaleFunc = &scaleFilterResonance;
					}
					setEffectsFunctions(effectType, whichEffect);
					FXAlias[0].setParam = effectSetters[whichEffect].setParam1;
					FXAlias[1].setParam = effectSetters[whichEffect].setParam2;
					FXAlias[2].setParam = effectSetters[whichEffect].setParam3;
					FXAlias[3].setParam = effectSetters[whichEffect].setParam4;
					FXAlias[4].setParam = effectSetters[whichEffect].setParam5;
				}

				for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
				{
					//set the real value based on the scale function
					params[whichParam].realVal[v] = params[whichParam].scaleFunc(params[whichParam].zeroToOneVal[v]);
					//set the actual parameter
					params[whichParam].setParam(params[whichParam].realVal[v], params[whichParam].objectNumber, v);
				}
				if ((whichParam == Osc1ShapeSet) || (whichParam == Osc2ShapeSet) || (whichParam == Osc3ShapeSet))
				{
					int whichOsc =(whichParam - Osc1ShapeSet) / OscParamsNum;
					int oscshape = roundf(params[whichParam].realVal[0] * (NUM_OSC_SHAPES-1));
					setOscilllatorShapes(oscshape, whichOsc);
				}
				if ((whichParam == Osc1) || (whichParam == Osc2) ||(whichParam == Osc3))
				{
					int whichOsc = (whichParam - Osc1) / OscParamsNum;
					if (params[whichParam].realVal[0]  > 0.5f)
					{
						oscsEnabled[whichOsc] = 1;
						oscOn[whichOsc] = 1;
					}
					else
					{
						oscsEnabled[whichOsc] = 0;
					}
					int enabledCount = 0;

					for (int j = 0; j < 3; j++)
					{
						enabledCount += oscsEnabled[j];
					}
					oscAmpMult = oscAmpMultArray[enabledCount];
				}
				if ((whichParam == Noise))
				{
					if (params[whichParam].realVal[0]  > 0.5f)
					{
						noiseOn = 1;
					}
				}
				if ((whichParam == Filter1Type) || (whichParam == Filter2Type))
				{
					int whichFilter = (whichParam - Filter1Type) / FilterParamsNum;
					int filterType = roundf(params[whichParam].realVal[0] * (NUM_FILTER_TYPES-1));
					setFilterTypes(filterType, whichFilter);
					int filterResParamNum = Filter1Resonance + (whichFilter * FilterParamsNum);
					int filterGainParamNum = Filter1Gain + (whichFilter * FilterParamsNum);
					params[filterResParamNum].setParam = filterSetters[whichFilter].setQ;
					params[filterGainParamNum].setParam = filterSetters[whichFilter].setGain;

					//set the resonance and gain params of that filter
					for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
					{
						params[filterResParamNum].setParam(params[filterResParamNum].realVal[v], params[filterResParamNum].objectNumber, v);
						params[filterGainParamNum].setParam(params[filterGainParamNum].realVal[v], params[filterGainParamNum].objectNumber, v);
					}
				}
				if ((whichParam == LFO1ShapeSet) || (whichParam == LFO2ShapeSet) || (whichParam == LFO3ShapeSet) || (whichParam == LFO4ShapeSet))
				{
					int whichLFO = (whichParam - LFO1ShapeSet) / LFOParamsNum;
					int LFOShape = roundf(params[whichParam].realVal[0] * (NUM_LFO_SHAPES-1));
					setLFOShapes(LFOShape, whichLFO);
					int rateParamNum = LFO1Rate + (whichLFO * LFOParamsNum);
					int shapeParamNum = LFO1Shape + (whichLFO * LFOParamsNum);
					int phaseParamNum = LFO1Phase + (whichLFO * LFOParamsNum);
					params[rateParamNum].setParam = lfoSetters[whichLFO].setRate;
					params[shapeParamNum].setParam = lfoSetters[whichLFO].setShape;
					params[phaseParamNum].setParam = lfoSetters[whichLFO].setPhase;

					//set the lfo params for that particular new lfo shape
					for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
					{
						params[rateParamNum].setParam(params[rateParamNum].realVal[v], params[rateParamNum].objectNumber, v);
						params[shapeParamNum].setParam(params[shapeParamNum].realVal[v], params[shapeParamNum].objectNumber, v);
						params[phaseParamNum].setParam(params[phaseParamNum].realVal[v], params[phaseParamNum].objectNumber, v);
					}
				}
				if ((whichParam == MIDIKeyMax) || (whichParam == MIDIKeyMin))
				{
					midiKeyDivisor = 1.0f / ((params[MIDIKeyMax].realVal[0]*127.0f) - (params[MIDIKeyMin].realVal[0]*127.0f));
					midiKeySubtractor = (params[MIDIKeyMin].realVal[0] * 127.0f);
				}
				/*
				if (whichParam == Transpose)
				{
					masterTranspose = params[Transpose].realVal[0];
				}
				*/
				if (whichParam == FXOrder)
				{
					fxPre = params[FXOrder].realVal[0] > 0.5f;
				}
				if (whichParam == PedalControlsMaster)
				{
					pedalControlsMaster = params[PedalControlsMaster].realVal[0] > 0.5f;
				}
			}
		}

		else if (SPI_LEVERS_RX[offset] == ReceivingMappingChange)
		{
			if (presetReady)
			{

				uint8_t currentByte = offset+1;

				uint16_t destNumber = ((SPI_LEVERS_RX[currentByte]<< 8) + SPI_LEVERS_RX[currentByte+1]);
				uint8_t whichSlot = (SPI_LEVERS_RX[currentByte+2]);
				uint8_t mappingChangeType = (SPI_LEVERS_RX[currentByte+3]);
				int16_t mappingChangeValue = ((SPI_LEVERS_RX[currentByte+4]<< 8) + SPI_LEVERS_RX[currentByte+5]);
				uint8_t whichMapping = 0;
				uint8_t foundOne = 0;

				// TODO: replace this search with explicit mapping slots instead
					// we need to add sending of mapping slots

				uint8_t lowestEmptyMapping = MAX_NUM_MAPPINGS;
				//search to see if this destination already has other mappings
				for (int j = 0; j < MAX_NUM_MAPPINGS; j++)
				{
					if (mappings[j].destNumber == destNumber)
					{
						//found one, use this mapping
						whichMapping = j;
						foundOne = 1;
					}
					if ((mappings[j].destNumber == 255) && (j < lowestEmptyMapping))
					{
						lowestEmptyMapping = j;
					}
				}
				if (foundOne == 0)
				{
					//didn't find another mapping with this destination, start a new mapping
					whichMapping = lowestEmptyMapping;
					numMappings++;
					mappings[whichMapping].destNumber = destNumber;
					mappings[whichMapping].dest = &params[destNumber];
				}


		//		//if the source is bipolar (oscillators, noise, and LFOs) then double the amount because it comes in as only half the range
		//		if ((source < 4) || ((source >= LFO_SOURCE_OFFSET) && (source < (LFO_SOURCE_OFFSET + NUM_LFOS))))
		//		{
		//			amountFloat *= 2.0f;
		//		}


				if (mappingChangeType == SourceID)
				{
					mappings[whichMapping].sourceSmoothed[whichSlot] = 1;
					int source = mappingChangeValue;

					if (source == 255)
					{
						//delete this hook
						mappings[whichMapping].hookActive[whichSlot] = 0;
						// if all hooks for this destination have source 255, delete this mapping
						int countHooks = 0;
						for (int i = 0; i < 3; i++)
						{
							if (mappings[whichMapping].hookActive[whichSlot] != 0)
							{
								countHooks++;
							}
						}
						//if you just removed the only hook from a mapping, mark the mapping invalid and remove it from the list
						//TODO: I think we are going to have to store a stack that represents which mappings are active and need to be ticked, otherwise it has to iterate all 32, now that we can remove one in the middle of the list.
						//or we keep track of the highest number of mapping we are ticking, and always tick up to that, ignoring elements we pass that have dest set to 255.

						if (countHooks == 0)
						{
							mappings[whichMapping].destNumber = 255;

							//since the mapping tick will no longer update it, it would stick on the last value, so reset it to the unmapped initial value
							for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
							{
								//sources are now summed - let's add the initial value
								float finalVal = mappings[whichMapping].dest->zeroToOneVal[v];


								//now scale the value with the correct scaling function
								mappings[whichMapping].dest->realVal[v] = mappings[whichMapping].dest->scaleFunc(finalVal);

								//and pop that value where it belongs by setting the actual parameter
								mappings[whichMapping].dest->setParam(mappings[whichMapping].dest->realVal[v], mappings[whichMapping].dest->objectNumber, v);
							}
						}
					}
					else
					{
						mappings[whichMapping].hookActive[whichSlot] = 1;

						for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
						{
							mappings[whichMapping].sourceValPtr[whichSlot][v] = &sourceValues[source][v];
							mappings[whichMapping].scalarSourceValPtr[whichSlot][v] = &defaultScaling; //blank out the scalar source, because otherwise it will point to some random function or a null pointer
						}
						if (source < 4) //if it's oscillators or noise (the first 4 elements of the source array), don't smooth to allow FM
						{
							mappings[whichMapping].sourceSmoothed[whichSlot] = 0;
						}
						if ((source >= LFO_SOURCE_OFFSET) && (source < (LFO_SOURCE_OFFSET + NUM_LFOS)))
						{
							lfoOn[source - LFO_SOURCE_OFFSET] = 1;
						}
						if ((source >= ENV_SOURCE_OFFSET) && (source < (ENV_SOURCE_OFFSET + NUM_ENV)))
						{
							envOn[source - ENV_SOURCE_OFFSET] = 1;
						}
						if ((source >= OSC_SOURCE_OFFSET) && (source < (OSC_SOURCE_OFFSET+NUM_OSC)))
						{
							oscOn[source - OSC_SOURCE_OFFSET] = 1;
						}
						if ((source >= NOISE_SOURCE_OFFSET) && (source < (NOISE_SOURCE_OFFSET+1)))
						{
							noiseOn = 1;
						}
						mappings[whichMapping].amount[whichSlot] = 0.0f;
					}


				}
				else if (mappingChangeType == Amount)
				{
					mappings[whichMapping].amount[whichSlot] = (float)mappingChangeValue * INV_TWO_TO_15;
				}
				else if (mappingChangeType == ScalarID)
				{
					int scalar = mappingChangeValue;
					for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
					{
						if (scalar == 0xff)
						{
							mappings[whichMapping].scalarSourceValPtr[whichSlot][v] = &defaultScaling;
						}
						else
						{
							mappings[whichMapping].scalarSourceValPtr[whichSlot][v] = &sourceValues[scalar][v];
							if ((scalar >= LFO_SOURCE_OFFSET) && (scalar < (LFO_SOURCE_OFFSET + NUM_LFOS)))
							{
								lfoOn[scalar - LFO_SOURCE_OFFSET] = 1;
							}
							if ((scalar >= ENV_SOURCE_OFFSET) && (scalar < (ENV_SOURCE_OFFSET + NUM_ENV)))
							{
								envOn[scalar - ENV_SOURCE_OFFSET] = 1;
							}
							if ((scalar >= OSC_SOURCE_OFFSET) && (scalar < (OSC_SOURCE_OFFSET + NUM_OSC)))
							{
								oscOn[scalar - OSC_SOURCE_OFFSET] = 1;
							}
							if ((scalar >= NOISE_SOURCE_OFFSET) && (scalar < (NOISE_SOURCE_OFFSET + 1)))
							{
								noiseOn = 1;
							}
							//TODO: doesn't cleanly remove lfoOn settings during streaming data - after deleting an LFO used as a scalar it will keep computing the LFO. How should we remember what the source of the scalar was when removing it? -JS
						}
					}
				}
			}

		}
		else if (SPI_LEVERS_RX[offset] == ReceivingPresetRequestCommand)
		{
			//send the current synth preset back to the PSOC5LP master microcontroller
			if (boardNumber == 0)
			{
				;
			}
		}
		else if (SPI_LEVERS_RX[offset] == ReceivingVolume)
		{
			uint_fast8_t  currentByte = offset+1;
			masterVolFromBrain =  ((SPI_LEVERS_RX[currentByte]) * 0.01f);
			masterVolFromBrainForSynth = masterVolFromBrain * 0.5f;
		}

		else if (SPI_LEVERS_RX[offset] == ReceivingBrainFirmwareUpdateRequest)
		{
			if (boardNumber == 0)
			{
				brainFirmwareUpdateRequested = 1;
			}
		}

		else if (SPI_LEVERS_RX[offset] == ReceivingPluckFirmwareUpdateRequest)
		{
			if (boardNumber == 0)
			{
				pluckFirmwareUpdateRequested = 1;
			}
		}
		else if (SPI_LEVERS_RX[offset] == RecievingCommandToStoreCurrentPreset)
		{
			if (boardNumber == 0)
			{
				uint_fast8_t  currentByte = offset+1;



				presetNumberToSave = SPI_LEVERS_RX[currentByte];
				currentByte++;
				bufferPos = 0;

				for (int i = 0; i < 18; i++)
				{
					buffer[bufferPos++] = SPI_LEVERS_RX[currentByte++];
				}
				/*
				for (int i = 0; i < 28; i++)
				{
					buffer[bufferPos++] = SPI_LEVERS_RX[currentByte + i];
					currentByte++;
				}
*/
				if (whichModel != 0)
				{
					buffer[1] = 19; // instead of the 18 that was sent by the brain, to signal that this is an internal model, not synth

					//skip over the name, we need to keep it from the brain

					bufferPos = 20;//first byte after name
					buffer[bufferPos] = whichModel; // not a synth preset, maybe string or additive or something
					bufferPos++;

					//9-byte macros
					for (int j = 0; j < 8; j++)
					{
						for (int k = 0; k < 9; k++)
						{
							buffer[bufferPos] = macroNamesArray[currentActivePreset][j][k];
							bufferPos++;
						}
					}
					//10-byte macros
					for (int j = 0; j < 4; j++)
					{
						for (int k = 0; k < 10; k++)
						{
							buffer[bufferPos] = macroNamesArray[currentActivePreset][j+8][k];
							bufferPos++;
						}
					}

					//remaining 9-byte macros
					for (int j = 0; j < 8; j++)
					{
						for (int k = 0; k < 9; k++)
						{
							buffer[bufferPos] = macroNamesArray[currentActivePreset][j+12][k];
							bufferPos++;
						}
					}

					for (int i = 0; i < 20; i++)
					{
						//copy the parameters into the default KnobParams Buffer
						uint16_t integerVersion = knobScaled[i] * TWO_TO_16;
						buffer[bufferPos] = integerVersion >> 8;
						buffer[bufferPos+1] = integerVersion & 255;
						bufferPos = bufferPos + 2;
					}

					 presetNumberToLoad = presetNumberToSave;
					 /* Parse into Audio Params */
					 presetWaitingToParse = bufferPos;
					 presetWaitingToWrite = bufferPos;

				}
				else //otherwise, it's a synth preset, keep the current buffer but change the name and the knob/joystick settings
				{
					//need to copy the buffer after the new name into the buffer
					bufferPos = 138;//first byte after name

					for (int i = 0; i < 12; i++)
					{
						//copy the parameters into the default KnobParams Buffer
						uint16_t integerVersion = knobScaled[i] * TWO_TO_16;
						buffer[bufferPos] = integerVersion >> 8;
						buffer[bufferPos+1] = integerVersion & 255;
						bufferPos = bufferPos + 2;
					}

					 presetNumberToLoad = presetNumberToSave;
					 /* Parse into Audio Params */
					 presetWaitingToParse = currentPresetSize; //use current stored preset size because that's how long the whole remaining buffer we didn't alter is
					 presetWaitingToWrite = currentPresetSize;

				}

			}
		}


	/*
		else if (SPI_LEVERS_RX[offset] == LoadingPreset)
		{
			uint8_t loadNumber = SPI_LEVERS_RX[offset+1];
			if (loadNumber < MAX_NUM_PRESETS)
			{
				presetNumberToLoad = loadNumber;
				whichPresetToSendName = loadNumber;
				presetWaitingToLoad = 1;
			}
		}
		if (SPI_LEVERS_RX[offset] == WaitingForLoadAck)
		{
			SPI_LEVERS_RX[offset] = 252;
			if(!loadFailed)
			{
				SPI_LEVERS_RX[offset+1] = currentActivePreset;//this will change to the loaded preset number when parsing is finished
			}
			else
			{
				SPI_LEVERS_RX[offset+1] = 254; //load failed
				SPI_LEVERS_RX[offset+2] = currentActivePreset; //tell the PSOC that it needs to show the old currently active preset, since the new load failed.
			}
		}
		*/
		//else
		{
			if (boardNumber == 0)
			{
				SPI_LEVERS_TX[offset] = 253; //special byte that says this is a preset name;
				SPI_LEVERS_TX[offset+1] = whichPresetToSendName;
				SPI_LEVERS_TX[offset+2] = presetNamesArray[whichPresetToSendName][0];
				SPI_LEVERS_TX[offset+3] = presetNamesArray[whichPresetToSendName][1];
				SPI_LEVERS_TX[offset+4] = presetNamesArray[whichPresetToSendName][2];
				SPI_LEVERS_TX[offset+5] = presetNamesArray[whichPresetToSendName][3];
				SPI_LEVERS_TX[offset+6] = presetNamesArray[whichPresetToSendName][4];
				SPI_LEVERS_TX[offset+7] = presetNamesArray[whichPresetToSendName][5];
				SPI_LEVERS_TX[offset+8] = presetNamesArray[whichPresetToSendName][6];
				SPI_LEVERS_TX[offset+9] = presetNamesArray[whichPresetToSendName][7];
				SPI_LEVERS_TX[offset+10] = presetNamesArray[whichPresetToSendName][8];
				SPI_LEVERS_TX[offset+11] = presetNamesArray[whichPresetToSendName][9];
				SPI_LEVERS_TX[offset+12] = presetNamesArray[whichPresetToSendName][10];
				SPI_LEVERS_TX[offset+13] = presetNamesArray[whichPresetToSendName][11];
				SPI_LEVERS_TX[offset+14] = presetNamesArray[whichPresetToSendName][12];
				SPI_LEVERS_TX[offset+15] = presetNamesArray[whichPresetToSendName][13];
				SPI_LEVERS_TX[offset+16] = whichMacroToSendName;
				SPI_LEVERS_TX[offset+17] = macroNamesArray[whichPresetToSendName][whichMacroToSendName][0];
				SPI_LEVERS_TX[offset+18] = macroNamesArray[whichPresetToSendName][whichMacroToSendName][1];
				SPI_LEVERS_TX[offset+19] = macroNamesArray[whichPresetToSendName][whichMacroToSendName][2];
				SPI_LEVERS_TX[offset+20] = macroNamesArray[whichPresetToSendName][whichMacroToSendName][3];
				SPI_LEVERS_TX[offset+21] = macroNamesArray[whichPresetToSendName][whichMacroToSendName][4];
				SPI_LEVERS_TX[offset+22] = macroNamesArray[whichPresetToSendName][whichMacroToSendName][5];
				SPI_LEVERS_TX[offset+23] = macroNamesArray[whichPresetToSendName][whichMacroToSendName][6];
				SPI_LEVERS_TX[offset+24] = macroNamesArray[whichPresetToSendName][whichMacroToSendName][7];
				SPI_LEVERS_TX[offset+25] = macroNamesArray[whichPresetToSendName][whichMacroToSendName][8];
				SPI_LEVERS_TX[offset+26] = macroNamesArray[whichPresetToSendName][whichMacroToSendName][9];
				SPI_LEVERS_TX[offset+27] = 13;
				SPI_LEVERS_TX[offset+28] = 13;
				SPI_LEVERS_TX[offset+29] = 13;
				SPI_LEVERS_TX[offset+30] = (sampleClippedCountdown > 0); //report whether there was a clip on the first board in the last 65535 samples
				SPI_LEVERS_TX[offset+31] = 254;
				whichMacroToSendName = (whichMacroToSendName + 1);
				if (whichMacroToSendName >= 20)
				{
					whichMacroToSendName = 0;
					whichPresetToSendName = (whichPresetToSendName + 1) % MAX_NUM_PRESETS;
				}
			}
		}
	}
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
}


/*
 * 				if (foundPluckFirmware)
				{
					SPI_LEVERS_TX[offset] = 252; //special byte that says this is a preset name;
					for (int i = 0; i < 30; i++)
					{

						SPI_LEVERS_TX[offset+i+1] = pluckFirmwareBuffer[pluckFirmwareBufferIndex + i];

					}
					SPI_LEVERS_TX[offset+31] = 254;
					pluckFirmwareBufferIndex += 30;
				}

			}
		}
		*/

void __ATTR_ITCMRAM parsePreset(int size, int presetNumber)
{
	//turn off the volume while changing parameters
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	uint16_t presetVersionNumber = 0;
	currentPresetSize = size;
	 __disable_irq();
	 presetReady = 0;
	 for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	 {
		 audioOutBuffer[i] = 0;
	 }
	audioMasterLevel = 0.0f;
	//osc params


	chorusAssignment = 255;
	delayAssignment = 255;

	uint16_t bufferIndex = 0;

	//should add an indicator at the beginning of the version number.
	// maybe 4 bytes:
		// [0] = 17   marker1 letting the parser know that a preset version number will follow
	    // [1] = 18   marker2 letting the parser know that a preset version number will follow // or 19 to say it's storing an internal preset
	    // [2] = version number major (i.e. the 01 in 1.04)
	    // [3] = version number minor (i.e. the 04 in 1.04)

	//if a version number is indicated, then set the presetVersionNumber value for later use, otherwise use default versionNumber of 0
	if (buffer[bufferIndex] == 17)
	{
		if (buffer[bufferIndex + 1] == 18)
		{
			presetVersionNumber = ((buffer[bufferIndex + 2] << 8) + buffer[bufferIndex + 3]);
			whichModel = 0; //this is a synth preset
			bufferIndex = 4;
		}


		if (buffer[bufferIndex + 1] == 19) //this means its an internal model, not the subtractive synth
		{
			presetVersionNumber = ((buffer[bufferIndex + 2] << 8) + buffer[bufferIndex + 3]);



			bufferIndex = 4;


			//read first 14 items in buffer as the 14 character string that is the name of the preset
			for (int i = 0; i < 14; i++)
			{
				presetName[i] = buffer[bufferIndex];
				presetNamesArray[presetNumber][i] = buffer[bufferIndex];
				bufferIndex++;
			}

			bufferIndex = 20;
			whichModel = buffer[bufferIndex]; // not a synth preset, maybe string or additive or something
			bufferIndex++;


			//9-byte macros
			for (int j = 0; j < 8; j++)
			{
				for (int k = 0; k < 9; k++)
				{
					macroNamesArray[presetNumber][j][k] = buffer[bufferIndex];
					bufferIndex++;
				}
			}
			//10-byte macros
			for (int j = 0; j < 4; j++)
			{
				for (int k = 0; k < 10; k++)
				{
					macroNamesArray[presetNumber][j+8][k] = buffer[bufferIndex];
					bufferIndex++;
				}
			}

			//remaining 9-byte macros
			for (int j = 0; j < 8; j++)
			{
				for (int k = 0; k < 9; k++)
				{
					macroNamesArray[presetNumber][j+12][k] = buffer[bufferIndex];
					bufferIndex++;
				}
			}

			for (int i = 0; i < 20; i++)
			{
				//copy the parameters into the default KnobParams Buffer
				loadedKnobParams[i] = INV_TWO_TO_16 * ((buffer[bufferIndex] << 8) + buffer[bufferIndex+1]);
				bufferIndex = bufferIndex + 2;
			}
			presetWaitingToParse = 0;
			currentActivePreset = presetNumber;

			if (whichModel == 1)
			{
				switchStrings = 1;
			}
			else if (whichModel == 2)
			{
				switchStrings = 2;
			}
			else if (whichModel == 3)
			{
				audioFrameFunction = audioFrameAdditive;
				audioSwitchToAdditive();
			}
			else if (whichModel == 4)
			{
				audioFrameFunction = audioFrameVocal;
				audioSwitchToVocal();
			}
			else if (whichModel == 5)
			{
				audioFrameFunction = audioFrameString3;
				audioSwitchToString3();
			}
			audioMasterLevel = 1.0f;

			__enable_irq();
			presetReady = 1;
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			diskBusy = 0;
			receivingI2C = 0;
			return;
		}



	}

	//read first 14 items in buffer as the 14 character string that is the name of the preset
	for (int i = 0; i < 14; i++)
	{
		presetName[i] = buffer[bufferIndex];
		presetNamesArray[presetNumber][i] = buffer[bufferIndex];
		bufferIndex++;
	}
	//9-byte macros
	for (int j = 0; j < 8; j++)
	{
		for (int k = 0; k < 9; k++)
		{
			macroNamesArray[presetNumber][j][k] = buffer[bufferIndex];
			bufferIndex++;
		}
	}
	//10-byte macros
	for (int j = 0; j < 4; j++)
	{
		for (int k = 0; k < 10; k++)
		{
			macroNamesArray[presetNumber][j+8][k] = buffer[bufferIndex];
			bufferIndex++;
		}
	}

	// blank out 9-byte macros after
	for (int j = 0; j < 8; j++)
	{
		for (int k = 0; k < 9; k++)
		{
			macroNamesArray[presetNumber][j+12][k] = 32;
		}
	}

	//read first element in buffer (after the 14 character name) as a count of how many parameters
	uint16_t paramCount = (buffer[bufferIndex] << 8) + buffer[bufferIndex+1];
	if (paramCount > size)
	{
		//error in transmission - give up and don't parse!
		audioMasterLevel = 1.0f;
		presetWaitingToParse = 0;
		 presetReady = 0;

		__enable_irq();
		return;
	}

	//check the validity of the transfer by verifying that the param array and mapping arrays both end with the required 0xefef values
	//should make this a real checksum
	uint16_t paramEndCheck = (buffer[paramCount*2+bufferIndex+2] << 8) + buffer[paramCount*2+bufferIndex+3];
	if (paramEndCheck != 0xefef)
	{
		//error in transmission - give up and don't parse!
		audioMasterLevel = 1.0f;
		presetWaitingToParse = 0;
		 presetReady = 0;
		__enable_irq();

		return;
	}
	uint16_t mappingCount = (buffer[paramCount*2+bufferIndex+4] << 8) + buffer[paramCount*2+bufferIndex+5];


	//20 is the 6 bytes plus the 14 characters
	uint16_t mappingEndLocation = 0;
	if (presetVersionNumber == 0)
	{
		mappingEndLocation = (paramCount * 2) + (mappingCount * 5) + bufferIndex+6;
	}
	else
	//paramCount is * 2 because they are 2 bytes per param, mappingCount * 6 because they are 6 bytes per mapping (changed from 5 to allow for sending of slot locations)
	{
		mappingEndLocation = (paramCount * 2) + (mappingCount * 6) + bufferIndex+6;
	}

	if (mappingEndLocation > size)
	{
		//error in transmission - give up and don't parse!
		audioMasterLevel = 1.0f;
		presetWaitingToParse = 0;
		 presetReady = 0;
		__enable_irq();

		return;
	}

	uint16_t mappingEndCheck = (buffer[mappingEndLocation] << 8) + buffer[mappingEndLocation+1];
	if (mappingEndCheck != 0xfefe) //this check value is 0xfefe
	{
		//error in transmission - give up and don't parse!
		audioMasterLevel = 1.0f;
		presetWaitingToParse = 0;
		 presetReady = 0;
		__enable_irq();

		return;
	}





	 //move past the name characters (4byte version, 14 bytes name, 9-byte * 8, 10-byte * 4) and paramcount position (2 bytes) in the buffer to start parsing the parameter data
	bufferIndex = bufferIndex + 2;

	//now read the parameters

	for (int i = 0; i < paramCount; i++)
	{
		for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
		{
			params[i].zeroToOneVal[v] = INV_TWO_TO_16 * ((buffer[bufferIndex] << 8) + buffer[bufferIndex+1]);
		}
		//need to map all of the params to their scaled parameters and set them to the realVals
		params[i].scaleFunc = &scaleDefault;

		//blank function means that it doesn't actually set a final value, we will read directly from the realVals when we need it
		params[i].setParam = &blankFunction;

		bufferIndex += 2;
	}


	//if loading old presets that don't have as many params, blank out the empty slots
	for (int i = paramCount; i < NUM_PARAMS; i++)
	{
		for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
		{
			params[i].zeroToOneVal[v] = 0.0f;
		}
		params[i].scaleFunc = &scaleDefault;
		params[i].setParam = &blankFunction;
	}


	//params[Master].scaleFunc = &scaleTwo;
	params[Transpose].scaleFunc = &scaleTranspose;
	params[PitchBendRange].scaleFunc = &scalePitchBend;
	//params[NoiseAmp].scaleFunc = &scaleTwo;
	params[Osc1Pitch].scaleFunc = &scaleOscPitch;
	params[Osc1Fine].scaleFunc = &scaleOscFine;
	params[Osc1Freq].scaleFunc = &scaleOscFreq;
	//params[Osc1Amp].scaleFunc = &scaleTwo;
	params[Osc1Harmonics].scaleFunc = &scaleOscPitch;
	params[Osc2Pitch].scaleFunc = &scaleOscPitch;
	params[Osc2Fine].scaleFunc = &scaleOscFine;
	params[Osc2Freq].scaleFunc = &scaleOscFreq;
	//params[Osc2Amp].scaleFunc = &scaleTwo;
	params[Osc2Harmonics].scaleFunc = &scaleOscPitch;
	params[Osc3Pitch].scaleFunc = &scaleOscPitch;
	params[Osc3Fine].scaleFunc = &scaleOscFine;
	params[Osc3Freq].scaleFunc = &scaleOscFreq;
	//params[Osc3Amp].scaleFunc = &scaleTwo;
	params[Osc3Harmonics].scaleFunc = &scaleOscPitch;
	params[Filter1Cutoff].scaleFunc = &scaleFilterCutoff;
	params[Filter1Resonance].scaleFunc = &scaleFilterResonance;
	params[Filter2Cutoff].scaleFunc = &scaleFilterCutoff;
	params[Filter2Resonance].scaleFunc = &scaleFilterResonance;
	params[Envelope1Attack].scaleFunc = &scaleEnvTimes;
	params[Envelope1Decay].scaleFunc = &scaleEnvTimes;
	params[Envelope1Release].scaleFunc = &scaleEnvTimes;
	params[Envelope2Attack].scaleFunc = &scaleEnvTimes;
	params[Envelope2Decay].scaleFunc = &scaleEnvTimes;
	params[Envelope2Release].scaleFunc = &scaleEnvTimes;
	params[Envelope3Attack].scaleFunc = &scaleEnvTimes;
	params[Envelope3Decay].scaleFunc = &scaleEnvTimes;
	params[Envelope3Release].scaleFunc = &scaleEnvTimes;
	params[Envelope4Attack].scaleFunc = &scaleEnvTimes;
	params[Envelope4Decay].scaleFunc = &scaleEnvTimes;
	params[Envelope4Release].scaleFunc = &scaleEnvTimes;
	params[LFO1Rate].scaleFunc = &scaleLFORates;
	params[LFO2Rate].scaleFunc = &scaleLFORates;
	params[LFO3Rate].scaleFunc = &scaleLFORates;
	params[LFO4Rate].scaleFunc = &scaleLFORates;
	//params[OutputAmp].scaleFunc = &scaleTwo;
	params[OutputTone].scaleFunc  = &scaleFinalLowpass;
	for (int i = 0; i < NUM_EFFECT; i++)
	{
		for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
		{
			FXType effectType = roundf(params[Effect1FXType + (EffectParamsNum * i)].zeroToOneVal[v] * (NUM_EFFECT_TYPES-1));
			param *FXAlias = &params[Effect1Param1 + (EffectParamsNum*i)];


			if (effectType > FXLowpass) // this assumes filters are the last effects
			{
				FXAlias[2].scaleFunc = &scaleFilterResonance;
			}
			setEffectsFunctions(effectType, i);
		}
	}
	for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
	{
		for (int i = 0; i < NUM_PARAMS; i++)
		{
			params[i].realVal[v] = params[i].scaleFunc(params[i].zeroToOneVal[v]);
		}
	}
	uint8_t enabledCount = 0;


	for (int i = 0; i < NUM_OSC; i++)
	{
		int oscshape = roundf(params[Osc1ShapeSet + (OscParamsNum * i)].realVal[0] * (NUM_OSC_SHAPES-1));
		setOscilllatorShapes(oscshape, i);
		if (params[Osc1 + (OscParamsNum * i)].realVal[0]  > 0.5f)
		{
			enabledCount++;
			oscsEnabled[i] = 1;
			oscOn[i] = 1;
		}
		else
		{
			oscsEnabled[i] = 0;
			oscOn[i] = 0;
		}
	}
	//set amplitude of oscillators based on how many are enabled
	oscAmpMult = oscAmpMultArray[enabledCount];

	if (params[Noise].realVal[0]  > 0.5f)
	{
		noiseOn = 1;
	}
	else
	{
		noiseOn = 0;
	}

	for (int i = 0; i < NUM_FILT; i++)
	{
		int filterType = roundf(params[Filter1Type + (i * FilterParamsNum)].realVal[0] * (NUM_FILTER_TYPES-1));
		setFilterTypes(filterType, i);
	}

	for (int i = 0; i < NUM_LFOS; i++)
	{
		int LFOShape = roundf(params[LFO1ShapeSet + (i * LFOParamsNum)].realVal[0] * (NUM_LFO_SHAPES-1));
		setLFOShapes(LFOShape, i);
	}


	//noiseparams
	params[NoiseTilt].setParam = &noiseSetTilt;
	params[NoisePeakFreq].setParam = &noiseSetFreq;
	params[NoisePeakGain].setParam  = &noiseSetGain;
	///////Setters for paramMapping
	params[Master].setParam = &setMaster;
	params[Transpose].setParam = &setTranspose;
	params[PitchBendRange].setParam = &setPitchBendRange;
	params[OutputTone].setParam = &setFinalLowpass;

	params[Osc1Pitch].setParam = &setFreqMultPitch;
	params[Osc2Pitch].setParam = &setFreqMultPitch;
	params[Osc3Pitch].setParam = &setFreqMultPitch;

	params[Osc1Harmonics].setParam = &setFreqMultHarm;
	params[Osc2Harmonics].setParam = &setFreqMultHarm;
	params[Osc3Harmonics].setParam = &setFreqMultHarm;

	params[Effect1Param1].setParam = effectSetters[0].setParam1;
	params[Effect1Param2].setParam = effectSetters[0].setParam2;
	params[Effect1Param3].setParam = effectSetters[0].setParam3;
	params[Effect1Param4].setParam = effectSetters[0].setParam4;
	params[Effect1Param5].setParam = effectSetters[0].setParam5;
	params[Effect1Mix].setParam = &fxMixSet;
	params[Effect1PostGain].setParam = &fxPostGainSet;
	params[Effect2Param1].setParam = effectSetters[1].setParam1;
	params[Effect2Param2].setParam = effectSetters[1].setParam2;
	params[Effect2Param3].setParam = effectSetters[1].setParam3;
	params[Effect2Param4].setParam = effectSetters[1].setParam4;
	params[Effect2Param5].setParam = effectSetters[1].setParam5;
	params[Effect2Mix].setParam = &fxMixSet;
	params[Effect2PostGain].setParam = &fxPostGainSet;
	params[Effect3Param1].setParam = effectSetters[2].setParam1;
	params[Effect3Param2].setParam = effectSetters[2].setParam2;
	params[Effect3Param3].setParam = effectSetters[2].setParam3;
	params[Effect3Param4].setParam = effectSetters[2].setParam4;
	params[Effect3Param5].setParam = effectSetters[2].setParam5;
	params[Effect3Mix].setParam = &fxMixSet;
	params[Effect3PostGain].setParam = &fxPostGainSet;
	params[Effect4Param1].setParam = effectSetters[3].setParam1;
	params[Effect4Param2].setParam = effectSetters[3].setParam2;
	params[Effect4Param3].setParam = effectSetters[3].setParam3;
	params[Effect4Param4].setParam = effectSetters[3].setParam4;
	params[Effect4Param5].setParam = effectSetters[3].setParam5;
	params[Effect4Mix].setParam = &fxMixSet;
	params[Effect4PostGain].setParam = &fxPostGainSet;
	params[Filter1Resonance].setParam = filterSetters[0].setQ;
	params[Filter1Gain].setParam = filterSetters[0].setGain;
	params[Filter2Resonance].setParam = filterSetters[1].setQ;
	params[Filter2Gain].setParam = filterSetters[1].setGain;
	params[Envelope1Attack].setParam = &setEnvelopeAttack;
	params[Envelope1Decay].setParam = &setEnvelopeDecay;
	params[Envelope1Sustain].setParam = &setEnvelopeSustain;
	params[Envelope1Release].setParam = &setEnvelopeRelease;
	params[Envelope1Leak].setParam = &setEnvelopeLeak;
	params[Envelope2Attack].setParam = &setEnvelopeAttack;
	params[Envelope2Decay].setParam = &setEnvelopeDecay;
	params[Envelope2Sustain].setParam = &setEnvelopeSustain;
	params[Envelope2Release].setParam = &setEnvelopeRelease;
	params[Envelope2Leak].setParam = &setEnvelopeLeak;
	params[Envelope3Attack].setParam = &setEnvelopeAttack;
	params[Envelope3Decay].setParam = &setEnvelopeDecay;
	params[Envelope3Sustain].setParam = &setEnvelopeSustain;
	params[Envelope3Release].setParam = &setEnvelopeRelease;
	params[Envelope3Leak].setParam = &setEnvelopeLeak;
	params[Envelope4Attack].setParam = &setEnvelopeAttack;
	params[Envelope4Decay].setParam = &setEnvelopeDecay;
	params[Envelope4Sustain].setParam = &setEnvelopeSustain;
	params[Envelope4Release].setParam = &setEnvelopeRelease;
	params[Envelope4Leak].setParam = &setEnvelopeLeak;
	params[LFO1Rate].setParam = lfoSetters[0].setRate;
	params[LFO2Rate].setParam = lfoSetters[1].setRate;
	params[LFO3Rate].setParam = lfoSetters[2].setRate;
	params[LFO4Rate].setParam = lfoSetters[3].setRate;
	params[LFO1Shape].setParam = lfoSetters[0].setShape;
	params[LFO2Shape].setParam = lfoSetters[1].setShape;
	params[LFO3Shape].setParam = lfoSetters[2].setShape;
	params[LFO4Shape].setParam = lfoSetters[3].setShape;
	params[LFO1Phase].setParam = lfoSetters[0].setPhase;
	params[LFO2Phase].setParam = lfoSetters[1].setPhase;
	params[LFO3Phase].setParam = lfoSetters[2].setPhase;
	params[LFO4Phase].setParam = lfoSetters[3].setPhase;
	params[OutputAmp].setParam = &setAmp;



	for (int i = 0; i < NUM_PARAMS; i++)
	{
		params[i].objectNumber = 0;
		//oscillators
		if ((i >= Osc1) && (i < Osc2))
		{
			params[i].objectNumber = 0;
		}
		else if ((i >= Osc2) && (i < Osc3))
		{
			params[i].objectNumber = 1;
		}
		else if ((i >= Osc3) && (i < Effect1FXType))
		{
			params[i].objectNumber = 2;
		}
		//effects
		//filters
		else if ((i >= Filter1) && (i < Filter2))
		{
			params[i].objectNumber = 0;
		}
		else if ((i >= Filter2) && (i < Envelope1Attack))
		{
			params[i].objectNumber = 1;
		}
		//envelopes
		else if ((i >= Envelope1Attack) && (i < Envelope2Attack))
		{
			params[i].objectNumber = 0;
		}
		else if ((i >= Envelope2Attack) && (i < Envelope3Attack))
		{
			params[i].objectNumber = 1;
		}
		else if ((i >= Envelope3Attack) && (i < Envelope4Attack))
		{
			params[i].objectNumber = 2;
		}
		else if ((i >= Envelope4Attack) && (i < LFO1Rate))
		{
			params[i].objectNumber = 3;
		}
		//lfos
		else if ((i >= LFO1Rate) && (i < LFO2Rate))
		{
			params[i].objectNumber = 0;
		}
		else if ((i >= LFO2Rate) && (i < LFO3Rate))
		{
			params[i].objectNumber = 1;
		}
		else if ((i >= LFO3Rate) && (i < LFO4Rate))
		{
			params[i].objectNumber = 2;
		}
		else if ((i >= LFO4Rate) && (i < OutputAmp))
		{
			params[i].objectNumber = 3;
		}
		//effects
		else if ((i >= Effect1FXType) && (i < Effect2FXType))
		{
			params[i].objectNumber = 0;
		}
		else if ((i >= Effect2FXType) && (i < Effect3FXType))
		{
			params[i].objectNumber = 1;
		}
		else if ((i >= Effect3FXType) && (i < Effect4FXType))
		{
			params[i].objectNumber = 2;
		}
		else if ((i >= Effect4FXType) && (i < Filter1))
		{
			params[i].objectNumber = 3;
		}

		for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
		{
			params[i].setParam(params[i].realVal[v], params[i].objectNumber, v);
		}

	}
	midiKeyDivisor = 1.0f / ((params[MIDIKeyMax].realVal[0]*127.0f) - (params[MIDIKeyMin].realVal[0]*127.0f));
	midiKeySubtractor = (params[MIDIKeyMin].realVal[0] * 127.0f);
	fxPre = params[FXOrder].realVal[0] > 0.5f;
	if (presetVersionNumber > 0)
	{
		pedalControlsMaster = params[PedalControlsMaster].realVal[0] > 0.5f;
	}
	else
	{
		pedalControlsMaster = 1;
	}
	//mappings parsing

	//move past the countcheck elements (already checked earlier)
	bufferIndex += 2;

	//move past the mappingCount elements (already stored that value earlier)
	bufferIndex += 2;

	numMappings = 0;
	for (int i = 0; i < NUM_LFOS; i++)
	{
		lfoOn[i] = 0;
	}
	for (int i = 0; i < NUM_ENV; i++)
	{
		envOn[i] = 0;
	}

	for (int i = 0; i < 12; i++)
	{
		knobFrozen[i] = 0;
		knobTicked[i] = 0;
	}
	for (int i = 0; i < 10; i++)
	{
		pedalTicked[i] = 0;
	}
	//blank out all current mappings
	for (int i = 0; i < MAX_NUM_MAPPINGS; i++)
	{
		mappings[i].destNumber = 255;
		mappings[i].hookActive[0] = 0;
		mappings[i].hookActive[1] = 0;
		mappings[i].hookActive[2] = 0;
		mappings[i].numHooks = 0;
	}



	for (int i = 0; i < mappingCount; i++)
	{
		uint8_t destNumber = buffer[bufferIndex+1];
		uint8_t whichMapping = 0;
		uint8_t whichHook = 0;
		uint8_t foundOne = 0;


		// TODO: replace this search with explicit mapping slots instead
			// we need to add sending of mapping slots

		if (presetVersionNumber > 0)
		{
			whichHook = buffer[bufferIndex+5]; //slotID sent as last bit of data in new preset sending versions
		}
		//search to see if this destination already has other mappings
		for (int j = 0; j < MAX_NUM_MAPPINGS; j++)
		{
			if (mappings[j].destNumber == destNumber)
			{
				//found one, use this mapping and add another hook to it
				whichMapping = j;
				if (presetVersionNumber == 0)
				{
					whichHook = mappings[j].numHooks;
				}
				foundOne = 1;
			}
		}
		if (foundOne == 0)
		{
			//didn't find another mapping with this destination, start a new mapping
			whichMapping = numMappings;

			numMappings++;
			if (presetVersionNumber == 0)
			{
				whichHook = 0;
			}
			mappings[whichMapping].destNumber = destNumber;
			mappings[whichMapping].dest = &params[destNumber];

		}
		mappings[whichMapping].sourceSmoothed[whichHook] = 1;

		int source = buffer[bufferIndex];

		for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
		{
			mappings[whichMapping].sourceValPtr[whichHook][v] = &sourceValues[source][v];
		}

		if (source < 4) //if it's oscillators or noise (the first 4 elements of the source array), don't smooth to allow FM
		{
			mappings[whichMapping].sourceSmoothed[whichHook] = 0;

		}
		if ((source >= LFO_SOURCE_OFFSET) && (source < (LFO_SOURCE_OFFSET + NUM_LFOS)))
		{
			lfoOn[source - LFO_SOURCE_OFFSET] = 1;
		}
		if ((source >= ENV_SOURCE_OFFSET) && (source < (ENV_SOURCE_OFFSET + NUM_ENV)))
		{
			envOn[source - ENV_SOURCE_OFFSET] = 1;
		}
		if ((source >= OSC_SOURCE_OFFSET) && (source < (OSC_SOURCE_OFFSET+NUM_OSC)))
		{
			oscOn[source - OSC_SOURCE_OFFSET] = 1;
		}
		if ((source >= NOISE_SOURCE_OFFSET) && (source < (NOISE_SOURCE_OFFSET+1)))
		{
			noiseOn = 1;
		}
		if ((source >= MACRO_SOURCE_OFFSET) && (source < (MACRO_SOURCE_OFFSET + NUM_MACROS + NUM_CONTROL)))
		{
			//if it's a macro, also set its value and set the knob to frozen state so it'll hold until the knob is moved.

			uint8_t whichMacro = source - MACRO_SOURCE_OFFSET;
			for (int v = 0; v < numStringsThisBoard; v++)
			{
				sourceValues[source][v] = params[whichMacro + MACRO_PARAMS_OFFSET].realVal[v];
			}
			//set starting point for the knob smoothers to smooth from
			tExpSmooth_setValAndDest(knobSmoothers[whichMacro], params[whichMacro + MACRO_PARAMS_OFFSET].realVal[0]);
			knobFrozen[whichMacro] = 1;
			knobTicked[whichMacro] = 1;
		}
		if ((source >= PEDAL_SOURCE_OFFSET) && (source < (PEDAL_SOURCE_OFFSET + 10)))
		{
			pedalTicked[source - PEDAL_SOURCE_OFFSET] = 1;
		}
		int scalar = buffer[bufferIndex+2];
		for (int v = 0; v < NUM_STRINGS_PER_BOARD; v++)
		{
			if (scalar == 0xff)
			{
				mappings[whichMapping].scalarSourceValPtr[whichHook][v] = &defaultScaling;
			}
			else
			{
				mappings[whichMapping].scalarSourceValPtr[whichHook][v] = &sourceValues[scalar][v];
				if ((scalar >= LFO_SOURCE_OFFSET) && (scalar < (LFO_SOURCE_OFFSET + NUM_LFOS)))
				{
					lfoOn[scalar - LFO_SOURCE_OFFSET] = 1;
				}
				if ((scalar >= ENV_SOURCE_OFFSET) && (scalar < (ENV_SOURCE_OFFSET + NUM_ENV)))
				{
					envOn[scalar - ENV_SOURCE_OFFSET] = 1;
				}
				if ((scalar >= OSC_SOURCE_OFFSET) && (scalar < (OSC_SOURCE_OFFSET + NUM_OSC)))
				{
					oscOn[scalar - OSC_SOURCE_OFFSET] = 1;
				}
				if ((scalar >= NOISE_SOURCE_OFFSET) && (scalar < (NOISE_SOURCE_OFFSET + 1)))
				{
					noiseOn = 1;
				}
				if ((scalar >= MACRO_SOURCE_OFFSET) && (scalar < (MACRO_SOURCE_OFFSET + NUM_MACROS + NUM_CONTROL)))
				{
					//if it's a macro, also set its value and set the knob to frozen state so it'll hold until the knob is moved.
					uint8_t whichMacro = scalar - MACRO_SOURCE_OFFSET;
					for (int v = 0; v < numStringsThisBoard; v++)
					{
						sourceValues[scalar][v] = params[whichMacro + MACRO_PARAMS_OFFSET].realVal[v];
					}
					//set starting point for the knob smoothers to smooth from
					tExpSmooth_setValAndDest(knobSmoothers[whichMacro], params[whichMacro + MACRO_PARAMS_OFFSET].realVal[0]);
					knobFrozen[whichMacro] = 1;
					knobTicked[whichMacro] = 1;
				}
				if ((scalar >= PEDAL_SOURCE_OFFSET) && (scalar < (PEDAL_SOURCE_OFFSET + 10)))
				{
					pedalTicked[scalar - PEDAL_SOURCE_OFFSET] = 1;
				}
			}
		}
		int16_t amountInt = (buffer[bufferIndex+3] << 8) + buffer[bufferIndex+4];
		float amountFloat = (float)amountInt * INV_TWO_TO_15;
//		//if the source is bipolar (oscillators, noise, and LFOs) then double the amount because it comes in as only half the range
//		if ((source < 4) || ((source >= LFO_SOURCE_OFFSET) && (source < (LFO_SOURCE_OFFSET + NUM_LFOS))))
//		{
//			amountFloat *= 2.0f;
//		}
		mappings[whichMapping].amount[whichHook] = amountFloat;
		mappings[whichMapping].hookActive[whichHook] = 1;
		mappings[whichMapping].numHooks++;

		if (presetVersionNumber > 0)
		{
			bufferIndex += 6;
		}
		else
		{
			bufferIndex += 5;
		}

	}

	uint8_t totalFilters = 0;
	if (params[Filter1].zeroToOneVal[0])
	{
		totalFilters++;
	}
	if (params[Filter2].zeroToOneVal[0])
	{
		totalFilters++;
	}

	//tick mappings once with no smoothing to set initial values
	for (int i = 0; i < numMappings; i++)
	{
		if (mappings[i].destNumber != 255)
		{
			for (int v = 0; v < numStringsThisBoard; v++)
			{
				float unsmoothedValue = 0.0f;

				for (int j = 0; j < 3; j++)
				{
					if (mappings[i].hookActive[j])
					{
						float sum = *mappings[i].sourceValPtr[j][v] * mappings[i].amount[j] * *mappings[i].scalarSourceValPtr[j][v];

						unsmoothedValue += sum;

					}
				}
				float finalVal = unsmoothedValue + mappings[i].dest->zeroToOneVal[v];

				//now scale the value with the correct scaling function
				mappings[i].dest->realVal[v] = mappings[i].dest->scaleFunc(finalVal);

				//and pop that value where it belongs by setting the actual parameter
				mappings[i].dest->setParam(mappings[i].dest->realVal[v], mappings[i].dest->objectNumber, v);
			}
		}
	}
	audioSwitchToSynth();
	presetWaitingToParse = 0;
	currentActivePreset = presetNumber;
	audioMasterLevel = 1.0f;
	oscToTick = NUM_OSC;
	overSampled = 1;
	changeOversampling(overSampled);
	__enable_irq();
	presetReady = 1;
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	diskBusy = 0;
	receivingI2C = 0;

}





// helper function to initialize measuring unit (cycle counter) */
void CycleCounterInit( void )
{
  /* Enable TRC */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Unlock DWT registers */
  if ((*(uint32_t*)0xE0001FB4) & 1)
    *(uint32_t*)0xE0001FB0 = 0xC5ACCE55;

  /* clear the cycle counter */
  DWT->CYCCNT = 0;

  /* start the cycle counter */
  DWT->CTRL = 0x40000001;

}


void FlushECC(void *ptr, int bytes)
{

	uint32_t addr = (uint32_t)ptr;
	/* Check if accessing AXI SRAM => 64-bit words*/
	if(addr >= 0x24000000 && addr < 0x24080000){
		volatile uint64_t temp;
		volatile uint64_t* flush_ptr = (uint64_t*) (addr & 0xFFFFFFF8);
		uint64_t *end_ptr = (uint64_t*) ((addr+bytes) & 0xFFFFFFF8);

		do{
			temp = *flush_ptr;
			*flush_ptr = temp;
			flush_ptr++;
		}while(flush_ptr != end_ptr);
	}
	/* Otherwise 32-bit words */
	else {
		volatile uint32_t temp;
		volatile uint32_t* flush_ptr = (uint32_t*) (addr & 0xFFFFFFFC);
		uint32_t *end_ptr = (uint32_t*) ((addr+bytes) & 0xFFFFFFFC);

		do{
			temp = *flush_ptr;
			*flush_ptr = temp;
			flush_ptr++;
		}while(flush_ptr != end_ptr);
	}
}


uint8_t volatile I2CErrors = 0;
void __ATTR_ITCMRAM HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

	presetWaitingToParse = 4096;
}


void __ATTR_ITCMRAM HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2cSending = 0;
}

void __ATTR_ITCMRAM HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	i2cSending = 0;
	receivingI2C = 0;
	I2CErrors++;
}

/* USER CODE BEGIN 4 */

void __ATTR_ITCMRAM HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	interrupted = 1;

	SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(((uint32_t )SPI_LEVERS_RX) & ~(uint32_t )0x1F), LEVER_BUFFER_SIZE_TIMES_TWO+32);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	if ((SPI_LEVERS_RX[62] == 254) && (SPI_LEVERS_RX[63] == 253))
	{
		handleSPI(LEVER_BUFFER_SIZE);
	}
	else
	{
		HAL_SPI_Abort(&hspi1);
		__HAL_RCC_SPI1_FORCE_RESET();
		__HAL_RCC_SPI1_RELEASE_RESET();
		MX_SPI1_Init();
		__HAL_SPI_ENABLE(&hspi1);
		HAL_SPI_TransmitReceive_DMA(&hspi1, SPI_LEVERS_TX, SPI_LEVERS_RX, LEVER_BUFFER_SIZE_TIMES_TWO);
		numResets++;
	}
	SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(((uint32_t )SPI_LEVERS_TX) & ~(uint32_t )0x1F), LEVER_BUFFER_SIZE_TIMES_TWO+32);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}

void __ATTR_ITCMRAM HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	interrupted = 1;

	SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(((uint32_t )SPI_LEVERS_RX) & ~(uint32_t )0x1F), LEVER_BUFFER_SIZE_TIMES_TWO+32);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	if ((SPI_LEVERS_RX[30] == 254) && (SPI_LEVERS_RX[31] == 253))
	{
		handleSPI(0);
	}
	else
	{
		HAL_SPI_Abort(&hspi1);
		__HAL_RCC_SPI1_FORCE_RESET();
		__HAL_RCC_SPI1_RELEASE_RESET();
		MX_SPI1_Init();
		__HAL_SPI_ENABLE(&hspi1);
		HAL_SPI_TransmitReceive_DMA(&hspi1, SPI_LEVERS_TX, SPI_LEVERS_RX, LEVER_BUFFER_SIZE_TIMES_TWO);
		numResets++;
	}
	SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(((uint32_t )SPI_LEVERS_TX) & ~(uint32_t )0x1F), LEVER_BUFFER_SIZE_TIMES_TWO+32);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

}

void __ATTR_ITCMRAM HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	interrupted = 1;
	if (hspi == &hspi5)
	{
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		//SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(((uint32_t )SPI_PLUCK_RX) & ~(uint32_t )0x1F), PLUCK_BUFFER_SIZE_TIMES_TWO+32);

		if ((SPI_PLUCK_RX[32] == 254) && (SPI_PLUCK_RX[63] == 253))
		{
			for (uint_fast8_t i = 0; i < numStringsThisBoard; i++)
			{
				stringInputs[i] = (SPI_PLUCK_RX[((i+firstString)*2)+ 33] << 8) + SPI_PLUCK_RX[((i+firstString)*2)+ 34];
			}
			newPluck = 1;
			HAL_SPI_Receive_DMA(&hspi5, SPI_PLUCK_RX, PLUCK_BUFFER_SIZE_TIMES_TWO);
		}
		else
		{
			HAL_SPI_Abort(&hspi5);
			__HAL_RCC_SPI5_FORCE_RESET();
			__HAL_RCC_SPI5_RELEASE_RESET();
			MX_SPI5_Init();
			__HAL_SPI_ENABLE(&hspi5);
			HAL_SPI_Receive_DMA(&hspi5,  SPI_PLUCK_RX, PLUCK_BUFFER_SIZE_TIMES_TWO);
			numResets++;
		}
		SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(((uint32_t )SPI_PLUCK_RX) & ~(uint32_t )0x1F), PLUCK_BUFFER_SIZE_TIMES_TWO+32);


	}


}

void __ATTR_ITCMRAM HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	interrupted = 1;
	if (hspi == &hspi5)
	{
		//SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(((uint32_t )SPI_PLUCK_RX) & ~(uint32_t )0x1F), PLUCK_BUFFER_SIZE_TIMES_TWO+32);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		if ((SPI_PLUCK_RX[0] == 254) && (SPI_PLUCK_RX[31] == 253))
		{
			for (uint_fast8_t i = 0; i < numStringsThisBoard; i++)
			{
				stringInputs[i] = (SPI_PLUCK_RX[((i+firstString)*2)+ 1] << 8) + SPI_PLUCK_RX[((i+firstString)*2)+ 2];
			}
			newPluck = 1;

			HAL_SPI_Receive_DMA(&hspi5, SPI_PLUCK_RX, PLUCK_BUFFER_SIZE_TIMES_TWO);
		}
		else
		{
			HAL_SPI_Abort(&hspi5);
			__HAL_RCC_SPI5_FORCE_RESET();
			__HAL_RCC_SPI5_RELEASE_RESET();
			MX_SPI5_Init();
			__HAL_SPI_ENABLE(&hspi5);
			HAL_SPI_Receive_DMA(&hspi5,  SPI_PLUCK_RX, PLUCK_BUFFER_SIZE_TIMES_TWO);
			numResets++;
		}
		SCB_CleanInvalidateDCache_by_Addr((uint32_t *)(((uint32_t )SPI_PLUCK_RX) & ~(uint32_t )0x1F), PLUCK_BUFFER_SIZE_TIMES_TWO+32);


	}


}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_6) {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0)
    {
    	//firmwareUpdateRequested = 1;
    }
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0xc0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x60000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
