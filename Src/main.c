/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rng.h"
#include "sai.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ui.h"
#include "leaf.h"
#include "audiostream.h"
#include "eeprom.h"
#include "MIDI_application.h"
#include "synth.h"
#include "oled.h"
#include "mapping.h"
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
uint16_t count;

//FLASH storage EEPROM emulation variables
FLASH_OBProgramInitTypeDef OBInit;
uint16_t VarDataTab = 0;
uint16_t VarValue = 0;

#define SYSEX_BUFFER_SIZE 8192

tFullPresetStateWithNameAndNumberUnion presetBuffer;

volatile uint32_t receivingSysex = 0;
volatile uint32_t parsingSysex = 0;

volatile uint32_t uartTest = 0;
uint32_t sysexPointerMask = SYSEX_BUFFER_SIZE - 1;
uint32_t sysexWritePointer = 0;
uint32_t sysexReadPointer = 0;
uint32_t lastBufferStuff = 0;
uint32_t lastEndReceive = 0;
uint32_t masterTimer = 0;
uint32_t newSysexStart = 0;
uint32_t prevLastParseCall = 0;
uint32_t lastParseCall = 0;

uint32_t sysexReset = 1;
uint32_t sysexMessageStartPointsWritePosition = 0;
uint32_t sysexMessageStartPointsReadPosition = 0;
uint32_t sysexMessageStartPoints[256];
uint32_t skipSysexHeader = 0;
uint32_t sysexHeaderCount = 2;
uint32_t lastBufferBegin[2];
uint32_t prevLastBufferBegin[2];
uint32_t sysexParseInProgress = 0;
uint32_t currentFloat = 0;
uint32_t presetNumberToWrite = 0;
presetArraySectionState presetArraySection = presetNameSection;
float myTestVal = 0.0f;
float valCheck = 0.0f;
uint32_t valsCount = 0;
uint32_t mapCount = 0;
uint32_t sysexParseError = 0;
uint32_t mapCountExpectation = 0;
uint32_t readyToWritePreset = 0;
uint32_t parseEBPPreset = 0;
union breakFloat {
 float f;
 uint8_t b[4];
 uint32_t u32;
};
#define PRESET_NAME_LENGTH_IN_BYTES 14
#define MACRO_NAME_LENGTH_IN_BYTES 9
#define CONTROL_NAME_LENGTH_IN_BYTES 10
#define NUM_MACROS 8
#define NUM_CONTROLS 4



#define NUM_COUNTER_CYCLES_TO_AVERAGE 32
volatile int64_t cycleCountVals[4][3];
volatile int64_t cycleCountValsAverager[4][NUM_COUNTER_CYCLES_TO_AVERAGE];
volatile uint16_t cycleCountAveragerCounter[4] = {0,0,0,0};
float cycleCountAverages[4][3];

volatile uint32_t loadingPreset = 0;
volatile uint32_t currentPreset = 0;
volatile uint32_t currentPresetSize = 0;
volatile uint32_t presetReady = 0;
volatile float audioMasterLevel = 0.0f;
volatile uint32_t prevPreset = 0;
volatile uint32_t diskBusy = 1;

volatile uint32_t loadFailed = 0;
int32_t volatile prevKnobByte[20];
FILINFO fno;
FIL fdst;
DIR dir;


uint8_t sysexBuffer[SYSEX_BUFFER_SIZE];
uint32_t sysexPointer = 0;
//uint8_t buffer[SYSEX_BUFFER_SIZE] __ATTR_RAM_D2;
volatile uint16_t bufferPos = 0;
FRESULT res;
const TCHAR path = 0;

volatile uint8_t presetName[14];
volatile uint8_t presetNamesArray[MAX_NUM_PRESETS][14]__ATTR_RAM_D2;
volatile uint8_t presetNumberToLoad = 0;
volatile uint32_t presetWaitingToParse = 0;
volatile uint32_t presetWaitingToWrite = 0;
volatile uint32_t presetWaitingToLoad = 0;

volatile uint32_t waitingToParseSingleParameterChange = 0;

volatile uint32_t waitingToParseSingleMappingChange = 0;

volatile uint8_t macroNamesArray[MAX_NUM_PRESETS][20][10]__ATTR_RAM_D2;

float loadedKnobParams[20];

uint_fast8_t knobTicked[12];

param params[NUM_PARAMS];
//mapping mappings[MAX_NUM_MAPPINGS];
uint8_t numMappings = 0;

uint8_t effectsActive[4] = {0,0,0,0};

filterSetter filterSetters[NUM_FILT];
lfoSetter lfoSetters[NUM_LFOS];
effectSetter effectSetters[NUM_EFFECT];
float defaultScaling = 1.0f;

uint8_t fxPre = 0;
uint8_t pedalControlsMaster = 0;

#define SCALE_TABLE_SIZE 2048
float resTable[SCALE_TABLE_SIZE];
float envTimeTable[SCALE_TABLE_SIZE];
float lfoRateTable[SCALE_TABLE_SIZE];
int oscsEnabled[3] = {0,0,0};
float midiKeyDivisor;
float midiKeySubtractor;
uint_fast8_t pedalTicked[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void MPU_Conf(void);
void SDRAM_Initialization_sequence(void);
static void CycleCounterInit( void );
void getPresetNamesFromSDCard(void);
static int checkForSDCardPreset(uint8_t numberToLoad);
void parsePreset(int, int);
void parseSysexPreset();
void parseSingleParameterChange();
void parseSingleMappingChange();
static void writePresetToSDCard(int fileSize);
#define testDataSize 32
volatile uint8_t testData[testDataSize];

volatile uint8_t errorTime1 = 0;

volatile uint8_t errorTime2 = 0;
volatile uint8_t errorTime3 = 0;
volatile uint8_t errorTime4 = 0;
volatile uint8_t testInt = 0;

volatile uint16_t ADC_values[6] __ATTR_RAM_D2_DMA;

volatile uint32_t OLED_changed = 0;

#define UART_BUFFER_SIZE 2
#define UART_HALF_BUFFER_SIZE 1
#define UART_BUFFER_MASK 1
volatile uint8_t UART_buffer[UART_BUFFER_SIZE] __ATTR_RAM_D2_DMA;

volatile uint32_t sysexReadyToParse = 0;


uint32_t parseThatMF = 0;

void errorFunction(int i)
{
	errorTime1 = large_memory[i];

	errorTime2 = testData[i];

	errorTime3 = testData[i];
	errorTime4 = large_memory[i + 32];
}

void SDRAM_test()
{
	uint32_t startingPoint = 0;
	SCB_CleanDCache();
	while (startingPoint < (LARGE_MEM_SIZE-testDataSize))
	{
		for (uint32_t i = 0; i < testDataSize; i++)
		{
			testData[i] = i;
			large_memory[i+startingPoint] = testData[i];
		}
		SCB_CleanDCache();
		for (uint32_t i = 0; i < testDataSize; i++)
		{
			testData[i] = i;
			testInt = large_memory[i+startingPoint];
			if (testInt != testData[i])
			{
				errorFunction(i);
			}
		}
		startingPoint += testDataSize;
	}
}





/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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

  //disabling I and D cache because they cause issues with the USB initialization when -o3 optimization is on
  /* Enable I-Cache---------------------------------------------------------*/
  SCB_DisableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_DisableDCache();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FMC_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_SAI1_Init();
  MX_RNG_Init();
  MX_USB_HOST_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /// it seems we need to enable caching after setting up the USB Host Controller -
  // otherwise turning on -o3 optimization causes unreliable behavior where it's not set up correctly and never reaches the USB interrupt for connection
  /* Enable I-Cache---------------------------------------------------------*/
 SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();


  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  //HAL_Delay(1);
  // Emulated EEPROM Init
  HAL_FLASH_Unlock();
  if( EE_Init() != EE_OK)
  {
    Error_Handler();
  }
  if((EE_ReadVariable(VirtAddVarTab[0],  &VarDataTab)) != HAL_OK) // read what the preset was before last power-off
  {
	//if it can't read something, it's probably because this brain has never been programmed, so write a value in there to start with
	  if((EE_WriteVariable(VirtAddVarTab[0],  0)) != HAL_OK)
	{
		Error_Handler();
	}
  }

  CycleCounterInit();

  //pull reset pin on audio codec low to make sure it's stable
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  uint32_t tempFPURegisterVal = __get_FPSCR();
  tempFPURegisterVal |= (1<<24); // set the FTZ (flush-to-zero) bit in the FPU control register  // this makes checking for denormals not necessary as they are automatically set to zero by the hardware
  __set_FPSCR(tempFPURegisterVal);


  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  if (HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_values, NUM_ADC_CHANNELS) != HAL_OK)
  {
      Error_Handler();
  }

  HAL_Delay(10);
  //HAL_Delay(10);

  SDRAM_Initialization_sequence();

  for (int i = 0; i < 20; i++)
  {
	  prevKnobByte[i] = 256;//to give a clearly wrong value for the knobs to see when they haven't been initialized with real 0-255 numbers yet
  }
  /*
  for (int i = 0; i < 4096; i++)
    {
  	  //buffer[i] = 0;
    }
*/

    //LEAF_generate_table_skew_non_sym(resTable, 0.01f, 10.0f, 0.5f, SCALE_TABLE_SIZE);
    //LEAF_generate_table_skew_non_sym(envTimeTable, 0.0f, 20000.0f, 4000.0f, SCALE_TABLE_SIZE);
    //LEAF_generate_table_skew_non_sym(lfoRateTable, 0.0f, 30.0f, 2.0f, SCALE_TABLE_SIZE);
    HAL_Delay(10);


  if (VarDataTab < MAX_NUM_PRESETS) //make sure the stored data is a number not past the number of available presets
  {
  	  currentPreset = VarDataTab; //if it's good, start at that remembered preset number
  }
  else
  {
  	  currentPreset = 0; //if the data is messed up for some reason, just initialize at the first preset (preset 0)
  }
  getPresetNamesFromSDCard();
  checkForSDCardPreset(currentPreset);
  presetNumberToLoad = currentPreset;
  presetWaitingToLoad = 1;

  OLED_init(&hi2c2, 0); // 1 for plug on bottom, 0 for plug on top orientation

  OLED_writePreset();

  audioInit(&hi2c2, &hsai_BlockA1, &hsai_BlockB1);

  for (int i = 0; i < UART_BUFFER_SIZE; i++)
  {
	  UART_buffer[i] = 0;
  }
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart6,UART_buffer,UART_BUFFER_SIZE);
  HAL_UART_Receive_DMA(&huart6,UART_buffer,UART_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//HAL_Delay(10);

	MIDI_Application();

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    if (presetWaitingToLoad > 0)
    {
    	checkForSDCardPreset(presetNumberToLoad);
    }

    if (parseThatMF > 0)
    {
    	//parseSysexPreset();
    }
    if (presetWaitingToWrite > 0)
    {
    	writePresetToSDCard(presetWaitingToWrite);
    }
    if (presetWaitingToParse > 0)
    {
    	//parsePreset(presetWaitingToParse, presetNumberToLoad);
    }
    if (waitingToParseSingleParameterChange > 0)
    {
    	//parseSingleParameterChange();
    }
    if (waitingToParseSingleMappingChange > 0)
    {
    	//parseSingleMappingChange();
    }


    OLED_process();
	if ((hi2c2.State == HAL_I2C_STATE_READY) && (OLED_changed))
	{
	  OLED_draw();
	}

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
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 344;
  PeriphClkInitStruct.PLL2.PLL2P = 7;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 1;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


#define SDRAM_TIMEOUT ((uint32_t)0xFFFF)



#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)

#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)

#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)

#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0003)

#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)

#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)

#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)

#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)

#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)

#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)

#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)



//#define SDRAM_REFRESH_COUNT                   	 ((uint32_t)956)// 7.9us in cycles of 8.333333ns + 20 cycles as recommended by datasheet page 866/3289 for STM32H743

#define SDRAM_REFRESH_COUNT                   	 ((uint32_t)0x0569)// 7.9us in cycles of 8.333333ns + 20 cycles as recommended by datasheet page 866/3289 for STM32H743

void SDRAM_Initialization_sequence(void)

{

    __IO uint32_t tmpmrd = 0;

    FMC_SDRAM_CommandTypeDef Command;

    /* Step 1: Configure a clock configuration enable command */

    Command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;

    Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;

    Command.AutoRefreshNumber = 1;

    Command.ModeRegisterDefinition = 0;



    /* Send the command */

    HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);



    /* Step 2: Insert 100 us minimum delay */

    /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */

    HAL_Delay(1);



    /* Step 3: Configure a PALL (precharge all) command */

    Command.CommandMode = FMC_SDRAM_CMD_PALL;

    Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;

    Command.AutoRefreshNumber = 1;

    Command.ModeRegisterDefinition = 0;



    /* Send the command */

    HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);



    /* Step 5: Program the external memory mode register */

    tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_4 | SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL

        | SDRAM_MODEREG_CAS_LATENCY_2 | SDRAM_MODEREG_OPERATING_MODE_STANDARD

        | SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;



    Command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;

    Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;

    Command.AutoRefreshNumber = 1;

    Command.ModeRegisterDefinition = tmpmrd;



    /* Send the command */

    HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);



    /* Step 4: Configure the 1st Auto Refresh command */

    Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;

    Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;

    Command.AutoRefreshNumber = 8;

    Command.ModeRegisterDefinition = 0;



    /* Send the command */

    HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);



    /* Step 2: Insert 100 us minimum delay */

    /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */

    HAL_Delay(1);



    /* Step 5: Configure the 2nd Auto Refresh command */

    Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;

    Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;

    Command.AutoRefreshNumber = 8;

    Command.ModeRegisterDefinition = 0;



    /* Send the command */

    HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);



    /* Step 6: Set the refresh rate counter */

    /* Set the device refresh rate */

    HAL_SDRAM_ProgramRefreshRate(&hsdram1, SDRAM_REFRESH_COUNT);

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

volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr; // Link register.
volatile uint32_t pc; // Program counter.
volatile uint32_t psr;// Program status register.

/*
	static void HardFault_Handler(void)
	{
	    __asm volatile
	    (
	        " tst lr, #4                                                n"
	        " ite eq                                                    n"
	        " mrseq r0, msp                                             n"
	        " mrsne r0, psp                                             n"
	        " ldr r1, [r0, #24]                                         n"
	        " ldr r2, handler2_address_const                            n"
	        " bx r2                                                     n"
	        " handler2_address_const: .word prvGetRegistersFromStack    n"
	    );
	}

*/
void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
//These are volatile to try and prevent the compiler/linker optimising them
//away as the variables never actually get used.  If the debugger won't show the
//values of the variables, make them global by moving their declaration outside
//of this function.



	r0 = pulFaultStackAddress[ 0 ];
	r1 = pulFaultStackAddress[ 1 ];
	r2 = pulFaultStackAddress[ 2 ];
	r3 = pulFaultStackAddress[ 3 ];

	r12 = pulFaultStackAddress[ 4 ];
	lr = pulFaultStackAddress[ 5 ];
	pc = pulFaultStackAddress[ 6 ];
	psr = pulFaultStackAddress[ 7 ];

	// When the following line is hit, the variables contain the register values.
	for( ;; );
}



/*
uint8_t LEAF_error(uint8_t errorCode)
{
	setLED_C(1);
	while(1)
		{
			;
		}

	return 0;
}
*/

// helper function to initialize measuring unit (cycle counter) */
static void CycleCounterInit( void )
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

  for (int i = 0; i < 4; i++)
  {
	  cycleCountAverages[i][0] = 0.0f;
	  cycleCountAverages[i][1] = 0.0f;
	  cycleCountAverages[i][2] = 0.0f;
  }
}





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
				unsigned int bytesRead;
				if(res == FR_OK)
				{
					if(f_open(&SDFile, fno.fname, FA_OPEN_ALWAYS | FA_READ) == FR_OK)
					{
						f_read(&SDFile, &presetBuffer.bytes, f_size(&SDFile), &bytesRead);
						f_close(&SDFile);
						/*
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
						*/
					}
				}
			}

		}

	}
	diskBusy = 0;
	return;
}
volatile uint32_t checkCount = 0;
volatile uint32_t parseCount = 0;
static int checkForSDCardPreset(uint8_t numberToLoad)
{

	int found = 0;
	volatile uint32_t tempCountCheck = DWT->CYCCNT;

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	if(BSP_SD_IsDetected())
	{
		for (int i = 0; i < AUDIO_BUFFER_SIZE; i+=2)
		{
			audioOutBuffer[i] = 0;
			audioOutBuffer[i + 1] = 0;
		}
		__disable_irq();
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
				strcat(finalString, "*.esp");
			}

			else
			{
				strcat(charBuf, "*.esp");
				strcpy(finalString, charBuf);
			}

			res = f_findfirst(&dir, &fno, SDPath, finalString);
			unsigned int  bytesRead;
			if(res == FR_OK)
			{
				if(f_open(&SDFile, fno.fname, FA_OPEN_ALWAYS | FA_READ) == FR_OK)
				{
					f_read(&SDFile, &presetBuffer.bytes, f_size(&SDFile), &bytesRead);
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
	else
	{
		prevPreset = numberToLoad;
		currentPreset = numberToLoad;
	}
	OLED_writePreset();
	diskBusy = 0;
	checkCount = DWT->CYCCNT - tempCountCheck;
	__enable_irq();
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	return found;
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
			    if (presetNumberToWrite > 99)
			    {
			    	presetNumberToWrite = 99;
			    }


			    //first, delete any existing presets that share the same number
			    FRESULT res;
				/* Start to search for preset files */
				char charBufC[10];
				char finalStringC[10];

				//turn the integer value into a 2 digit string

				itoa(presetNumberToWrite, charBufC, 10);
				int len = ((strlen(charBufC)));
				if (len == 1)
				{
					finalStringC[2] = charBufC[1];
					finalStringC[1] = charBufC[0];
					finalStringC[0] = '0';
					strcat(finalStringC, "*.esp");
				}

				else
				{
					strcat(charBufC, "*.esp");
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
				itoa(presetNumberToWrite, charBuf, 10);
				len = ((strlen(charBuf)));
				if (len == 1)
				{
					finalString[21] = 0;
					finalString[20] = 'p';
					finalString[19] = 's';
					finalString[18] = 'e';
					finalString[17] = '.';
					for (int i = 0; i < 14; i++)
					{
						finalString[i+3] = presetBuffer.data.name[i];
						//replace spaces with underscores for filename
						if (finalString[i+3] <= 32)
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
					finalString[19] = 's';
					finalString[18] = 'e';
					finalString[17] = '.';
					for (int i = 0; i < 14; i++)
					{
						finalString[i+3] = presetBuffer.data.name[i];
						//replace spaces with underscores for filename
						if (finalString[i+3] <= 32)
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
					unsigned int bytesRead;
					f_write(&SDFile, &presetBuffer.bytes, fileSize, &bytesRead);
					f_close(&SDFile);
				}

			}
			//f_mount(0, "", 0); //unmount
		}
	}
	presetWaitingToWrite = 0;
	currentPreset = presetNumberToWrite;
	presetWaitingToParse = 1;
	diskBusy = 0;
	__enable_irq();
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


void  parsePreset(int size, int presetNumber)
{
	//turn off the volume while changing parameters
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

#if 0

	volatile uint32_t tempCountParse = DWT->CYCCNT;
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


//do stuff

	presetWaitingToParse = 0;
	audioMasterLevel = 1.0f;
	oscToTick = NUM_OSC;
	overSampled = 1;
	changeOversampling(overSampled);
	//currentPreset = presetNumberToWrite;
	OLED_writePreset();

	parseCount = DWT->CYCCNT - tempCountParse;
	__enable_irq();
	presetReady = 1;
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	diskBusy = 0;
	//receivingI2C = 0;
#endif

}





//translate that preset! this is to take the complete sysex midi message that has been received and stored
// and format it in the binary ESP (electrosynth preset) format. It comes in as 7bit words, turn it back into 8-bit

void parseSysexPreset()
{
	uint32_t presetSize7bit = (sysexWritePointer - sysexReadPointer - 2) & sysexPointerMask; // should check that this works when wrapping around

	uint32_t eightBitSize = roundf(presetSize7bit * (7.0f/8.0f) + 0.5f);



	uint32_t eightBitByteCount = 0;
	uint32_t eightBitBitCount = 0;

	sysexReadPointer += 2; //skip past the type tag and test byte for now

	uint8_t tempByte = 0;
	//split the 7-bit bytes up and pack them in 8-bit bytes
	for (uint32_t inputByte = 0; inputByte < presetSize7bit; inputByte++)
	{
		for (uint8_t bit = 0; bit < 7; bit++)
		{

			uint8_t valueOfCurrentBit = (sysexBuffer[(inputByte + sysexReadPointer) & sysexPointerMask] >> bit) & 1; //get the value of the bit we're reading from the 7-bit byte
			tempByte |= (valueOfCurrentBit << eightBitBitCount); //plop it in the 8-bit byte in the position where it needs to be.

			//count which bit you're on in the 8-bit word (different from "bit" in the 8-bit word)
			eightBitBitCount++;
			//roll over the 8-bit byte if you reached the end
			if (eightBitBitCount >= 8)
			{
				presetBuffer.bytes[eightBitByteCount] = tempByte;
				eightBitBitCount = 0;
				tempByte = 0;
				eightBitByteCount++;
			}
		}
	}
	sysexReadPointer = (presetSize7bit + sysexReadPointer) & sysexPointerMask;
	presetNumberToWrite = presetBuffer.data.presetNumber;
	for (int i = 0; i < 14; i++)
	{
		if (presetBuffer.data.name[i] != 0)
		{
			presetNamesArray[presetNumberToWrite][i] = presetBuffer.data.name[i];
		}
		else
		{
			presetNamesArray[presetNumberToWrite][i] = 32;
		}
	}
	currentPreset = presetNumberToWrite;
	OLED_writePreset();
	presetWaitingToWrite = eightBitSize;
	//presetWaitingToParse = 1;
	//parseFullPreset();

#if 0
	parsingSysex = 1;
	presetReady = 0;
	uint32_t messageStart = sysexMessageStartPoints[sysexMessageStartPointsReadPosition];
	uint32_t messageEnd = sysexMessageStartPoints[(sysexMessageStartPointsReadPosition + 1) & 255];
	sysexMessageStartPointsReadPosition = (sysexMessageStartPointsReadPosition + 1) & 255;
	sysexReadPointer = messageStart;

    if (sysexBuffer[sysexReadPointer & sysexPointerMask] == 0)
    {
    	sysexParseInProgress = 1; // set a flag that we've started a sysex preset transfer. May take multiple sysex parse calls on the chunks to complete
        currentFloat = 0;
        presetArraySection = presetNameSection;
        presetNumberToWrite = sysexBuffer[(sysexReadPointer+1) & sysexPointerMask];
        buffer[0] = sysexBuffer[(sysexReadPointer+2) & sysexPointerMask];
        buffer[1] = sysexBuffer[(sysexReadPointer+3) & sysexPointerMask];
        buffer[2] = sysexBuffer[(sysexReadPointer+4) & sysexPointerMask];
        buffer[3] = sysexBuffer[(sysexReadPointer+5) & sysexPointerMask];

        union breakFloat theVal;
        uint32_t i = 6;
        sysexReadPointer = i + sysexReadPointer;
        uint8_t stoppingPoint = PRESET_NAME_LENGTH_IN_BYTES+i;
        for (; i < stoppingPoint; i++)
        {
        	buffer[i-2] = sysexBuffer[sysexReadPointer & sysexPointerMask] & 127; // pass on the first 14 elements as 8-bit bytes (they are the chars for the name string)
            presetNamesArray[presetNumberToWrite][i-6] = sysexBuffer[sysexReadPointer & sysexPointerMask] & 127;
            sysexReadPointer++;
        }

        presetArraySection = macroNamesSection;


        for (int j = 0; j < (NUM_MACROS); j++)
        {
            for (int k = 0; k < MACRO_NAME_LENGTH_IN_BYTES; k++)
            {
            	buffer[i-2] = sysexBuffer[sysexReadPointer & sysexPointerMask] & 127; // pass on the first 14 elements as 8-bit bytes (they are the chars for the name string)
                macroNamesArray[presetNumberToWrite][j][k] = sysexBuffer[sysexReadPointer & sysexPointerMask] & 127; // pass on the first 14 elements as 8-bit bytes (they are the chars for the name string)
                i++;
                sysexReadPointer++;
            }
        }
        for (int j = 0; j < NUM_CONTROLS; j++)
        {
            for (int k = 0; k < CONTROL_NAME_LENGTH_IN_BYTES; k++)
            {
            	buffer[i-2] = sysexBuffer[sysexReadPointer & sysexPointerMask] & 127; // pass on the first 14 elements as 8-bit bytes (they are the chars for the name string)
            	macroNamesArray[presetNumberToWrite][j+8][k] = sysexBuffer[sysexReadPointer & sysexPointerMask] & 127; // pass on the first 14 elements as 8-bit bytes (they are the chars for the name string)
                 i++;
                sysexReadPointer++;
            }
        }

        uint16_t valsStart = 4 + PRESET_NAME_LENGTH_IN_BYTES + (MACRO_NAME_LENGTH_IN_BYTES * NUM_MACROS) + (CONTROL_NAME_LENGTH_IN_BYTES * NUM_CONTROLS);

        presetArraySection = initialValsSection;

        for (; sysexReadPointer < (messageEnd); sysexReadPointer = (sysexReadPointer+5))
        {
            theVal.u32 = 0;
            theVal.u32 |= ((sysexBuffer[sysexReadPointer & sysexPointerMask ] &15) << 28);
            theVal.u32 |= (sysexBuffer[(sysexReadPointer+1) & sysexPointerMask] << 21);
            theVal.u32 |= (sysexBuffer[(sysexReadPointer+2) & sysexPointerMask] << 14);
            theVal.u32 |= (sysexBuffer[(sysexReadPointer+3) & sysexPointerMask] << 7);
            theVal.u32 |= (sysexBuffer[(sysexReadPointer+4) & sysexPointerMask] & 127);
            myTestVal = theVal.f;
            if (presetArraySection == initialValsSection)
            {

                if (currentFloat == 0)
                {
                    valsCount = (uint16_t) theVal.f;
                    buffer[valsStart + currentFloat++] = valsCount >> 8;
                    buffer[valsStart + currentFloat++] = valsCount & 0xff;
                }
                else if (currentFloat < ((valsCount+1)*2))
                {
                    uint16_t intVal = (uint16_t)(theVal.f * 65535.0f);
                    buffer[valsStart + currentFloat++] = intVal >> 8;
                    buffer[valsStart + currentFloat++] = intVal & 0xff;
                }
                else if (currentFloat == ((valsCount+1)*2))
                {
                    valCheck = theVal.f;
                    if ((valCheck < -1.5f) && (valCheck > -2.5f))
                    {
                    	buffer[valsStart + currentFloat++] = 0xef;
                    	buffer[valsStart + currentFloat++] = 0xef;
                        presetArraySection = mapCountNextSection;
                        mapCount = 0;
                    }
                    else
                    {
                        //error state
                    	sysexParseError++;
                    	parsingSysex = 0;
                    }
                }
            }
            else if (presetArraySection == mapCountNextSection)
            {
                mapCountExpectation = (uint16_t)theVal.f;
                buffer[valsStart + currentFloat++] = mapCountExpectation >> 8;
                buffer[valsStart + currentFloat++] = mapCountExpectation & 0xff;
                presetArraySection = mappingSection;
                numMappings = 0;
            }
            else if (presetArraySection == mappingSection)
            {
                // this is the order
                // source (int), target (int), scalarSource (arrives as -1.0f if no scalar, send as 255 if no scalar)(int), range (float -1.0 to 1.0), slot# (in uint8_t)
                if (numMappings < mapCountExpectation)
                {
                    if ((mapCount % 5) == 0)
                    {
                    	buffer[valsStart + currentFloat++] = (uint8_t)theVal.f;
                    }
                    else if  (mapCount % 5 == 1)
                    {
                    	buffer[valsStart + currentFloat++] = (uint8_t)theVal.f;
                    }
                    else if (mapCount % 5 == 2) //check if the scalar source is -1 (if so send 255 instead of a valid source number)
                    {
                        if (theVal.f < 0.0f)
                        {
                        	buffer[valsStart + currentFloat++] = 0xff;
                        }
                        else
                        {
                        	buffer[valsStart + currentFloat++] = (uint8_t)theVal.f;
                        }
                    }
                    else if (mapCount % 5 == 3)
                    {
                        int16_t intVal = (int16_t)(theVal.f * 32767.0f); //keep it signed to allow negative numbers
                        buffer[valsStart + currentFloat++] = intVal >> 8;
                        buffer[valsStart + currentFloat++] = intVal & 0xff;

                    }
                    else
                    {
                    	buffer[valsStart + currentFloat++] = (uint8_t)theVal.f;
                        numMappings++;
                    }
                    mapCount++;
                }


                else
                {
                    //mapcount ended
                    if ((theVal.f < -2.5f) && (theVal.f > -3.5f))
                    {
                    	buffer[valsStart + currentFloat++] = 0xfe;
                    	buffer[valsStart + currentFloat++] = 0xfe;
                        presetArraySection = presetEndSection;
                        parsingSysex = 0;
                        presetWaitingToWrite = valsStart + currentFloat;
                        presetWaitingToParse = valsStart + currentFloat;
                        parseThatMF = 0;
                        //prese = presetNumberToWrite;
                        //messageArraySize = valsStart + currentFloat;
                    }
                    else
                    {
                        //error state
                    	sysexParseError++;
                       // sysexPointer = 0;
                        parsingSysex = 0;
                        parseThatMF = 0;

                    }
                }
            }

        }
    }
#endif

}

void parseSingleParameterChange()
{

	if (presetReady)
	{

		//sysexMessageInProgress = 1; // set a flag that we've started a sysex preset transfer. May take multiple sysex parse calls on the chunks to complete
		union breakFloat theVal;
		uint32_t i = (2 + sysexMessageStartPoints[sysexMessageStartPointsReadPosition]);
		sysexMessageStartPointsReadPosition = (sysexMessageStartPointsReadPosition + 1) & 255;

		//get the destination number
		theVal.u32 = 0;
		theVal.u32 |= ((sysexBuffer[i & sysexPointerMask] &15) << 28);
		theVal.u32 |= (sysexBuffer[(i+1) & sysexPointerMask] << 21);
		theVal.u32 |= (sysexBuffer[(i+2) & sysexPointerMask] << 14);
		theVal.u32 |= (sysexBuffer[(i+3) & sysexPointerMask] << 7);
		theVal.u32 |= (sysexBuffer[(i+4) & sysexPointerMask] & 127);
		uint16_t whichParam  = (uint16_t)roundf(theVal.f);

		 i = (i+5);

		 //get the parameter value
		 theVal.u32 = 0;
		 theVal.u32 |= ((sysexBuffer[i & sysexPointerMask] &15) << 28);
		 theVal.u32 |= (sysexBuffer[(i+1)& sysexPointerMask] << 21);
		 theVal.u32 |= (sysexBuffer[(i+2)& sysexPointerMask] << 14);
		 theVal.u32 |= (sysexBuffer[(i+3)& sysexPointerMask] << 7);
		 theVal.u32 |= (sysexBuffer[(i+4)& sysexPointerMask] & 127);

		 //uint16_t intVal = (uint16_t)(theVal.f * 65535.0f);

	}


	waitingToParseSingleParameterChange = 0;
}

uint32_t sendMappingChangeUpdate = 0;

void parseSingleMappingChange()
{
	if (presetReady)
	{
		//sysexMessageInProgress = 1; // set a flag that we've started a sysex preset transfer. May take multiple sysex parse calls on the chunks to complete
		union breakFloat theVal;
		uint32_t i = (2 + sysexMessageStartPoints[sysexMessageStartPointsReadPosition]);
		sysexMessageStartPointsReadPosition = (sysexMessageStartPointsReadPosition + 1) & 255;
		//get the destination number
		theVal.u32 = 0;
		theVal.u32 |= ((sysexBuffer[i & sysexPointerMask] &15) << 28);
		theVal.u32 |= (sysexBuffer[(i+1) & sysexPointerMask] << 21);
		theVal.u32 |= (sysexBuffer[(i+2) & sysexPointerMask] << 14);
		theVal.u32 |= (sysexBuffer[(i+3) & sysexPointerMask] << 7);
		theVal.u32 |= (sysexBuffer[(i+4) & sysexPointerMask] & 127);
		uint16_t destNumber  = (uint16_t)roundf(theVal.f);



		uint8_t whichSlot = sysexBuffer[(i+5) & sysexPointerMask]; //slot id
		uint8_t mappingChangeType = sysexBuffer[(i+6) & sysexPointerMask]; //mapping change type

		i = (i+7);

		//get the parameter value
		theVal.u32 = 0;
		theVal.u32 |= ((sysexBuffer[i & sysexPointerMask] &15) << 28);
		theVal.u32 |= (sysexBuffer[(i+1) & sysexPointerMask] << 21);
		theVal.u32 |= (sysexBuffer[(i+2) & sysexPointerMask] << 14);
		theVal.u32 |= (sysexBuffer[(i+3) & sysexPointerMask] << 7);
		theVal.u32 |= (sysexBuffer[(i+4) & sysexPointerMask] & 127);
	}
	waitingToParseSingleMappingChange = 0;
}

uint32_t currentUARTBufferPos = 0;
uint32_t uartParseCount = 0;

void UART_buffer_parse(uint32_t offset)
{
#if 0
	for (int i = 0; i < size; i++)
	{

		sysexBuffer[(sysexWritePointer++) & sysexPointerMask] = UART_buffer[currentUARTBufferPos++ & UART_BUFFER_MASK];

		uint32_t offset = currentUARTBufferPos++ & UART_BUFFER_MASK;
#endif
#if 0
		if (receivingSysex)
		{

			if (UART_buffer[offset] < 128)
			{
				sysexBuffer[(sysexWritePointer++) & sysexPointerMask] = UART_buffer[offset];
				lastBufferStuff = masterTimer;
				sysexReset = 0;
			}
			else
			{
				if (UART_buffer[offset] == 0xf7)
				{
					receivingSysex = 0;
					lastEndReceive = masterTimer;

					//parseSysex();
					return;
				 }
			}
		}

		else if (newSysexStart)
		{
			if (!parsingSysex)
			{
				if (UART_buffer[offset] == 126) // special message saying that sysex multi-chunk transmission is finished. Parse it!
				{

					switch (sysexBuffer[sysexMessageStartPoints[sysexMessageStartPointsWritePosition] & sysexPointerMask])
					{
						case 0://new preset
							parseThatMF = 1;
							parsingSysex = 1;
							break;
						case 3: //real-time parameter change
							waitingToParseSingleParameterChange = 1;
							break;
						case 4: //real-time mapping change
							waitingToParseSingleMappingChange = 1;
							break;
						default:
							break;
					}


					prevLastParseCall = lastParseCall;
					lastParseCall = masterTimer;
					sysexReset = 1;

					sysexMessageStartPointsWritePosition = (sysexMessageStartPointsWritePosition + 1) & 255;
					sysexMessageStartPoints[sysexMessageStartPointsWritePosition] = sysexWritePointer;
					newSysexStart = 0;
					//sysexPointer = 0;
				}
				else
				{
					if (sysexHeaderCount == 2) //first byte after start byte
					{

						if (UART_buffer[offset] == 0 || UART_buffer[offset] == 1 || UART_buffer[offset] == 2 || UART_buffer[offset] == 3 || UART_buffer[offset] == 4)
						{
							// if this is the first chunk, put in the first and second elements (following chunks need this data stripped until the final message gets sent)
							prevLastBufferBegin[0] = lastBufferBegin[0];
							prevLastBufferBegin[1] = lastBufferBegin[1];
							lastBufferBegin[0] = masterTimer;
							lastBufferBegin[1] = UART_buffer[offset];
						}
					}
					//only store the two header bytes in the first chonk of a multi-chonk transmission
					// if this is the first sysex chunk of a multi-chonk transmission
					if ((sysexReset == 1) && (sysexHeaderCount > 0))
					{
						sysexBuffer[sysexWritePointer++ & sysexPointerMask] = UART_buffer[offset];
					}
					sysexHeaderCount--;
					//if you got past the first two bytes after the start byte, start just filling the buffer
					if (sysexHeaderCount == 0)
					{
						receivingSysex = 1;
						newSysexStart = 0;
					}
				}


			}
		}

		else if (UART_buffer[offset] == 0xf0) //got a start byte
		{
			newSysexStart = 1;
			sysexHeaderCount = 2; //how many header bytes will follow (save them in first chonk and drop the ones in following chonks)
		}
#endif
	if (receivingSysex)
	{

		if (UART_buffer[offset] < 128)
		{
			sysexBuffer[(sysexWritePointer++) & sysexPointerMask] = UART_buffer[offset];
			lastBufferStuff = masterTimer;
			sysexReset = 0;
		}
		else
		{
			if (UART_buffer[offset] == 0xf7)
			{
				receivingSysex = 0;
				lastEndReceive = masterTimer;

				parseSysexPreset();
				return;
			 }
		}
	}
	else if (UART_buffer[offset] == 0xf0) //got a start byte
	{
		receivingSysex = 1;
	}


	masterTimer++;

}

volatile uint32_t sizeComingIn = 0;
volatile uint32_t sizeMinusPosition = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
#if 0
	HAL_UART_RxEventTypeTypeDef eventTypeUART = HAL_UARTEx_GetRxEventType(huart);
	sizeComingIn = Size;
	sizeMinusPosition = (Size - currentUARTBufferPos) & UART_BUFFER_MASK;
	if (eventTypeUART == HAL_UART_RXEVENT_IDLE)
	{
		UART_buffer_parse(sizeMinusPosition);
	}
	else if (eventTypeUART == HAL_UART_RXEVENT_TC)
	{

		UART_buffer_parse(sizeMinusPosition);
	}

	else if (eventTypeUART == HAL_UART_RXEVENT_HT)
	{
		UART_buffer_parse(sizeMinusPosition);
	}
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UART_buffer_parse(1);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	UART_buffer_parse(0);
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
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x60000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

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
while(1)
{
;
}
/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
