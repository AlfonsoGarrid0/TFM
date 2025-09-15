/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> //para sprintf
#include <string.h> // para strlen
#include "usbd_cdc_if.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_FREQ 256UL //frecuencia de muestreo del ADC disparado por el TIMER2 (APB1/PSC_TIM2/ARR)

#define ADC_BUFFER_SIZE 6UL //tamaño del buffer de ADC-DMA, modificándolo cambio la frecuencia de procesamiento

#define PSD_TIME 1UL //Tiempo máximo durante el que puedo almacenar muestras (configurable) para hacer la PSD

#define BUFFER_SIZE (ADC_FREQ*PSD_TIME)

#define FFT_SIZE 256UL

#define SLIDING_WINDOW_SIZE 4

///////////////////////////////////////////////////////////////////////////////////////////

#define	STIM_TYPE_CH_DEF	0x10
#define	STIM_AMPLIT_DEF		0x80
#define	STIM_DURATION_DEF	0xC0

#define	POSITIVE			0x20
#define	NEGATIVE			0x40
#define	BIPHASIC			0x60

#define STIM_AMP_4mA 		8 //Consultar valores en tabla en Stimulator.c (StimEAST) o en doc EAST UART Comm protocol
#define STIM_AMP_4_5mA 		9
#define STIM_AMP_5mA 		10
#define STIM_AMP_5_5mA 		11
#define STIM_AMP_6mA 		12
#define STIM_AMP_6_5mA 		13
#define STIM_AMP_7mA 		14
#define STIM_AMP_7_5mA 		15
#define STIM_AMP_8mA 		16
#define STIM_AMP_8_5mA 		17
#define STIM_AMP_9mA 		18
#define STIM_AMP_9_5mA 		19
#define STIM_AMP_10mA 		20
#define STIM_AMP_10_5mA 	21
#define STIM_AMP_11mA 		22
#define STIM_AMP_11_5mA 	23
#define STIM_AMP_12mA 		24
#define STIM_AMP_12_5mA 	25
#define STIM_AMP_13mA 		26

#define STIM_DURATION_400US 13 //Consultar valores en tabla en Stimulator.c (StimEAST) o en doc EAST UART Comm protocol

#define	RESET_STIM 	HAL_GPIO_WritePin(RESET_STIM_GPIO_Port, RESET_STIM_Pin, 0)
#define	NO_RST_STIM HAL_GPIO_WritePin(RESET_STIM_GPIO_Port, RESET_STIM_Pin, 1)

#define CRG_STAT		HAL_GPIO_ReadPin(CHRG_STAT_GPIO_Port, CHRG_STAT_Pin)

#define	LEDB_ON		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1)
#define	LEDB_OFF	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 0)

#define	LEDW_ON		HAL_GPIO_WritePin(LED_W_GPIO_Port, LED_W_Pin, 1)
#define	LEDW_OFF	HAL_GPIO_WritePin(LED_W_GPIO_Port, LED_W_Pin, 0)

#define	LEDR_ON		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1)
#define	LEDR_OFF	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0)
#define LED_RED_IS_ON	HAL_GPIO_ReadPin(LED_R_GPIO_Port, LED_R_Pin)

#define    SET_TRIG_OUT    HAL_GPIO_WritePin(TRIG_OUT_GPIO_Port, TRIG_OUT_Pin, 1)
#define    RES_TRIG_OUT    HAL_GPIO_WritePin(TRIG_OUT_GPIO_Port, TRIG_OUT_Pin, 0)

#define SWITCH			HAL_GPIO_ReadPin(SWITCH_GPIO_Port, SWITCH_Pin)

#define	READ_INPUTS_INTERVAL	100			// Interval between reads of the inputs in ms
#define	BATT_CRG_BLK_INTERVAL	500			// Blinking time when the battery is charging in ms
#define BATT_LEV_READ_INTERVAL	1000		// Interval between reads of the battery level in ms

#define	LOW_BATTERY			800

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t accel_adc_buffer[ADC_BUFFER_SIZE];


volatile uint8_t ADC_DMA_transfer_cmplt = 0;
volatile uint8_t ADC_DMA_transfer_half = 0;


float x_accel[BUFFER_SIZE];
float y_accel[BUFFER_SIZE];
float z_accel[BUFFER_SIZE];
uint16_t idx = 0;


const float vref = 3.3; // (accel Vref = 3.3 V)
const uint16_t adc_res = 4095; // (2^12 - 1)
const float zero_ref = 1.65; //(1.65 V)
const float adxl335_precission = 0.33; // (Volts/g)


//char uart_tx_buffer[160];


float32_t fft_output[FFT_SIZE];
const float freq_res = ((float)ADC_FREQ) / FFT_SIZE; // resolución en Hz
const uint16_t k_low = (uint16_t)(4.0f / freq_res + 0.5f);
const uint16_t k_high = (uint16_t)(12.0f / freq_res + 0.5f);

arm_rfft_fast_instance_f32 S_rfft; // instancia reutilizable
float32_t rfft_in[FFT_SIZE];
float32_t rfft_out[FFT_SIZE];
const float32_t hann_U = 0.375f; // Σw²/N de una ventana Hann (coeficiente de tabla)
float32_t thPSD = 0.15;   // umbral → calibrar con datos reales

float32_t energy = 0.0f;

uint8_t tremor = 0;


volatile uint8_t user_button_flag = 0;
uint32_t last_exti_time = 0;
const uint8_t debounce_delay = 50; // 50 ms de antirrebote


typedef enum { FLAT = 0, POS = 1, NEG = 2 } slope_t;
slope_t slope = FLAT, prev_slope = FLAT;

const float TH0_POS = 0.095;
const float TH0_NEG = -0.095;
float der = 0;

uint8_t stim_flag = 0;
uint8_t stim_array_info[BUFFER_SIZE] = {0}; //array para enviar por USB y comprobar que estimulo como quiero

///////////////////////////////////////////////////////////////////////////////////////////////


const uint8_t StimType = BIPHASIC;		// Stimulation type (positive, negative or biphasic)

const uint8_t StimAmp_aux[2] = {STIM_AMP_6_5mA, STIM_AMP_5_5mA}; //0 - 20 mA
uint8_t StimAmp[2] = {0, 0}; //StimAmp[0] = EXT StimAmp[1] = FLEX

const uint8_t StimDur = STIM_DURATION_400US;

uint8_t		Time2ReadInputs = 0;		// Flag, is it time to read the inputs?
uint8_t		Time2ReadBatLev	= 0;		// Flag, is it time to read the battery level?
uint8_t 	BatteryCharging = 0;		// Flag, the battery is recharging
uint8_t		LowBattery = 0;				// Flag, the battery is low

uint32_t	ADConv;		// Result of ADC1 conversion (battery)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void GetAccel(void);
void CheckStim(void);
void ComputePSD(void);
void SendUSB(void);
void ReadInputs(void);
void ReadBatteryLev(void);
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //__HAL_DMA_DISABLE_IT(hdma_adc2, DMA_IT_TC); //Disable Transfer Complete interrupt for adc2
  //__HAL_DMA_DISABLE_IT(hdma_adc2, DMA_IT_HC); //Disable Half Transfer Complete interrupt for adc2


  LEDW_ON;


  NO_RST_STIM; //PB5 RESET DE LA STIM (LÓGICA NEGADA)


  HAL_TIM_Base_Start(&htim2);

  //HAL_ADC_Start(&hadc1);
  //HAL_ADC_Start_DMA(&hadc2, (uint32_t*)accel_adc_buffer, ADC_BUFFER_SIZE); //accel_adc_buffer works as a pointer to the address of the first position of the accel_adc_buffer[] array


  arm_rfft_fast_init_f32(&S_rfft, FFT_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(user_button_flag == 1){

		  if(ADC_DMA_transfer_half == 1){

			  GetAccel();

			  if(tremor == 1){
				  CheckStim();
			  }

			  stim_array_info[idx] = stim_flag;

			  idx ++;

			  SendUSB();

			  if(idx >= BUFFER_SIZE){
				  idx = 0;
				  ComputePSD();
			  }
		  }

		  if(ADC_DMA_transfer_cmplt == 1){

			  GetAccel();

			  if(tremor == 1){
				  CheckStim();
			  }

			  stim_array_info[idx] = stim_flag;

			  idx ++;

			  SendUSB();

			  if(idx >= BUFFER_SIZE){
				  idx = 0;
				  ComputePSD();
			  }

		  }
	  }

			if(Time2ReadInputs){
				ReadInputs();
				Time2ReadInputs = 0;
			}

			if(Time2ReadBatLev){
				ReadBatteryLev();
				Time2ReadBatLev = 0;
			}

/*
			if(TrigInSet){
				// Do something when the TRIG_IN is set
				TrigInSet = 0;
			}
*/

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 3;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 28124;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1439;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 249;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS0_Pin|CS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, START_ADC_Pin|RESET_ADC_Pin|BOOT_STIM_Pin|RESET_STIM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_W_Pin|LED_B_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_OUT_GPIO_Port, TRIG_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS0_Pin CS1_Pin */
  GPIO_InitStruct.Pin = CS0_Pin|CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : START_ADC_Pin RESET_ADC_Pin BOOT_STIM_Pin RESET_STIM_Pin */
  GPIO_InitStruct.Pin = START_ADC_Pin|RESET_ADC_Pin|BOOT_STIM_Pin|RESET_STIM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : P1_2_Pin P1_3_Pin */
  GPIO_InitStruct.Pin = P1_2_Pin|P1_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : P1_7_Pin P1_6_Pin P0_2_Pin P2_7_Pin
                           SWITCH_Pin CHRG_STAT_Pin */
  GPIO_InitStruct.Pin = P1_7_Pin|P1_6_Pin|P0_2_Pin|P2_7_Pin
                          |SWITCH_Pin|CHRG_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CARD_PR_Pin */
  GPIO_InitStruct.Pin = CARD_PR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CARD_PR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_W_Pin LED_B_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = LED_W_Pin|LED_B_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_OUT_Pin */
  GPIO_InitStruct.Pin = TRIG_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_IN_Pin */
  GPIO_InitStruct.Pin = TRIG_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TRIG_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void) //needed to add HAL_SYSTICK_IRQHandler(); in "stm32f4xx_it.c"" for using the callback
{
	static uint32_t	ReadInputsTimer = 0;
	static uint32_t	BattChargeTimer = 0;
	static uint32_t	ReadBattLevTimer = 0;

	// Check if it is time to read the digital inputs
	ReadInputsTimer++;
	if(ReadInputsTimer >= READ_INPUTS_INTERVAL)
	{
		ReadInputsTimer = 0;
		Time2ReadInputs = 1;
	}

	// Check if it is time to blink the red LED for indicating the battery is in charge
	if(BatteryCharging)
	{
		BattChargeTimer++;
		if(BattChargeTimer >= BATT_CRG_BLK_INTERVAL)
		{
			BattChargeTimer = 0;
			if(LED_RED_IS_ON)
				LEDR_OFF;
			else
				LEDR_ON;
		}
	}

	// Check if it is time for reading the battery level
	ReadBattLevTimer++;
	if(ReadBattLevTimer >= BATT_LEV_READ_INTERVAL)
	{
		ReadBattLevTimer = 0;
		Time2ReadBatLev = 1;
	}
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){

	if(hadc->Instance == ADC2){ //Verificar que la interrupción viene del ADC2

		ADC_DMA_transfer_half = 1;

/*************************************************************
		uint32_t current_tick = HAL_GetTick();
		time_between_interruptions = current_tick - last_tick;
		last_tick = current_tick;
**************************************************************/

	}

}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	if(hadc->Instance == ADC2){ //Verificar que la interrupción viene del ADC2

		ADC_DMA_transfer_cmplt = 1;

/***************************************************************
		uint32_t current_tick = HAL_GetTick();
		time_between_interruptions = current_tick - last_tick;
		last_tick = current_tick;
****************************************************************/

	}

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

    if(htim->Instance == TIM7){ //Verificar que la interrupción viene del TIM7
    	static uint8_t ChanInd = 0;
    	uint8_t	UARTCommand[3];

    	// Sent the UART command to the stimulator microcontroller
    	// | es el operador OR a nivel de bit, combina los bits de todos los parámetros en el mismo comando
    	UARTCommand[0] = STIM_TYPE_CH_DEF | ChanInd | StimType;
    	UARTCommand[1] = STIM_AMPLIT_DEF | StimAmp[ChanInd];
    	UARTCommand[2] = STIM_DURATION_DEF | StimDur;
    	HAL_UART_Transmit(&huart1, UARTCommand, 3, 100);

    	ChanInd++;
    	if(ChanInd >= 2) {
    		ChanInd = 0;
    	}
    }
}


void GetAccel(void){
	float x_aux, y_aux, z_aux;
	float x_volts, y_volts, z_volts;

	if(ADC_DMA_transfer_half == 1){
		ADC_DMA_transfer_half = 0;

		x_aux = ((float)accel_adc_buffer[0]);
		y_aux = ((float)accel_adc_buffer[1]);
		z_aux = ((float)accel_adc_buffer[2]);

		x_volts = (x_aux / adc_res) * vref;
		y_volts = (y_aux / adc_res) * vref;
		z_volts = (z_aux / adc_res) * vref;

		x_accel[idx] = (x_volts - zero_ref)/adxl335_precission;
		y_accel[idx] = (y_volts - zero_ref)/adxl335_precission;
		z_accel[idx] = (z_volts - zero_ref)/adxl335_precission;
	}

	if(ADC_DMA_transfer_cmplt == 1){
		ADC_DMA_transfer_cmplt = 0;

		x_aux = ((float)accel_adc_buffer[(ADC_BUFFER_SIZE)/2 + 0]);
		y_aux = ((float)accel_adc_buffer[(ADC_BUFFER_SIZE)/2 + 1]);
		z_aux = ((float)accel_adc_buffer[(ADC_BUFFER_SIZE)/2 + 2]);

		x_volts = (x_aux / adc_res) * vref;
		y_volts = (y_aux / adc_res) * vref;
		z_volts = (z_aux / adc_res) * vref;

		x_accel[idx] = (x_volts - zero_ref)/adxl335_precission;
		y_accel[idx] = (y_volts - zero_ref)/adxl335_precission;
		z_accel[idx] = (z_volts - zero_ref)/adxl335_precission;
	}
}


void ComputePSD(void){

	/* Copia + ventana Hann (entrada real) */
    for (uint16_t n = 0; n < FFT_SIZE; n++) {
        float32_t w = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * n / (FFT_SIZE - 1)));
        rfft_in[n] = z_accel[n] * w;
    }

    /* FFT rápida real → espectro empaquetado */
    arm_rfft_fast_f32(&S_rfft, rfft_in, rfft_out, 0);   // 0 = forward

    /* |X[k]|² y normalización a PSD */
    const float32_t scale = 1.0f / (ADC_FREQ * FFT_SIZE * hann_U);

    /* DC y Nyquist */
    fft_output[0]           = (rfft_out[0] * rfft_out[0]) * scale;
    fft_output[FFT_SIZE/2]  = (rfft_out[1] * rfft_out[1]) * scale;

    /* bins 1 … N/2-1 (Re,Im intercalados en rfft_out) */
    for (uint16_t k = 1; k < FFT_SIZE/2; k++) {
        float32_t re = rfft_out[2*k];
        float32_t im = rfft_out[2*k + 1];
        fft_output[k] = (re*re + im*im) * scale;
    }

    /*Energía*/
    //float32_t energy = 0.0f;
    energy = 0.0f;
    for (uint16_t k = k_low; k <= k_high; k++) {
        energy += fft_output[k] * freq_res;   // g²
    }

    /* Decisión */
    if(energy > thPSD){
    	tremor = 1;
    	HAL_TIM_Base_Start_IT(&htim7);
    	LEDB_ON;
    }
    else{
    	tremor = 0;
    	HAL_TIM_Base_Stop_IT(&htim7);
    	LEDB_OFF;
    	stim_flag = 0;
    }
}


void CheckStim(void){

	//if(idx==0) return; --> no es necesario ya que como mínimo la 1ª vez que se llame a esta función se tendrán 256 muestras y lo hago modo circular

	float aux_der = 0;
	for(uint8_t i = 0; i<SLIDING_WINDOW_SIZE; i++){
		uint8_t curr_idx = (idx - i + BUFFER_SIZE) % BUFFER_SIZE;
		uint8_t prev_idx = (idx - i - 1 + BUFFER_SIZE) % BUFFER_SIZE;
		aux_der += z_accel[curr_idx] - z_accel[prev_idx];
	}

	der = aux_der/SLIDING_WINDOW_SIZE;

	if(der>TH0_POS){
		slope = POS;
	}
	else if(der<TH0_NEG){
		slope = NEG;
	}
	else{
		slope = FLAT;
	}

	if (slope == FLAT && prev_slope == POS){
		//START STIM IN EXT AND STOP IT IN FLEX (OOP) (bc ext was on and now it stopped and flex it´s going to be on)
		StimAmp[0] = StimAmp_aux[0]; //EXT STIM ON
		StimAmp[1] = 0; //FLEX STIM OFF
		stim_flag = 1;
	}
	else if(slope == FLAT && prev_slope == NEG){
		//START STIM IN FLEX AND STOP IT IN EXT (OOP) (bc flex was on and now it stopped and ext it´s going to be on)
		StimAmp[0] = 0; //EXT STIM ON
		StimAmp[1] = StimAmp_aux[1]; //FLEX STIM OFF
		stim_flag = 2;
	}

	prev_slope = slope;
}


void SendUSB(void){ //cuando hay 16 nuevas muestras las envía [0-15], [16-31]... [240-255]... circular

	static uint8_t usb_tx_buffer[80]; //16*4B + 16*1B
	if (idx >= 16 && (idx % 16 == 0)){ // 16 nuevas muestras llenas

	    memcpy(usb_tx_buffer, &z_accel[idx - 16], 64);   //copia en tx los valores de z_accel desde [idx-16] hasta que haya 64 bytes (o sea 16 valores, 16 * 4B)

	    memcpy(&usb_tx_buffer[64], &stim_array_info[idx - 16], 16);

	    if (CDC_Transmit_FS(usb_tx_buffer, 80) != USBD_OK){   //espera a que quede libre
	    	//Nop();
	    	//while(1);
	    }

	}



/*****Si quiero enviar por USB con los valores ya formateados a ASCII*******

	if (idx >= 16 && (idx % 16 == 0)) {
		char ascii_tx[192];
	    int len = 0;

		for (int i = idx - 16; i < idx; i++){
	    	len += snprintf(ascii_tx + len, sizeof(ascii_tx) - len, "% .3f\r\n", z_accel[i]);
	   	}

		if (CDC_Transmit_FS((uint8_t*)ascii_tx, len) != USBD_OK) {
			//Nop();
	    	//while(1);
	    }
	}

*******************************************************************************/

}


void ReadInputs(void){
	if(CRG_STAT == 0){
		BatteryCharging = 1;
	}
	else{
		BatteryCharging = 0;
		if(!LowBattery)
			LEDR_OFF;
	}

	static uint8_t prev_button_state = 0;
	if(SWITCH && !prev_button_state){

	    uint32_t current_time = HAL_GetTick();
	    if ((current_time - last_exti_time) > debounce_delay) {
	        last_exti_time = current_time;

	        user_button_flag = !user_button_flag;  // alterna ON/OFF

	        if(user_button_flag == 1){
	            // Activar DMA
	            idx = 0; // reinicia para empezar desde cero
	            HAL_ADC_Start_DMA(&hadc2, (uint32_t*)accel_adc_buffer, ADC_BUFFER_SIZE);
	            //LEDB_ON; // opcional: indicador visual
	            SET_TRIG_OUT; //para sincronizar con quatroccento
	        }
	        else{
	            // Desactivar DMA
	            HAL_ADC_Stop_DMA(&hadc2);
	            //LEDB_OFF; // opcional: indicador visual
	            RES_TRIG_OUT;
	            HAL_TIM_Base_Stop_IT(&htim7);
	        }
	    }
	}
	prev_button_state = SWITCH;
}


void ReadBatteryLev(void)
{

	HAL_ADC_Start(&hadc1);  // Inicia una sola conversión
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		ADConv = HAL_ADC_GetValue(&hadc1); // Lee el resultado
	}

	if((ADConv < LOW_BATTERY) && (CRG_STAT == 1))
	{
		LEDR_ON;
		LowBattery = 1;
	}
	else
	{
		LowBattery = 0;
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

