/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FREQ_MAX 16384		// 16.3kHz (potentiometer step of 4Hz)

#define POT_MIN 160
#define POT_MAX 3940
#define POT_INTER (POT_MAX - POT_MIN)

#define LED_INTERVAL 250

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t led_last_time;

uint16_t adc_val[2];
uint16_t pot1 = 0;
uint16_t pot2 = 0;

uint32_t freq = 1;
uint16_t ducy = 50;

uint16_t psc = 72;
uint32_t ccr = 0;
uint32_t arr = 0;
uint32_t clk = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DMA_ADC_init(void);
void crop_values(void);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);
  DMA_ADC_init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_Delay(1);	// shift signals
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_Delay(1); // shift signals
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

	  pot1 = adc_val[0];
	  pot2 = adc_val[1];
	  crop_values();

	  freq = ((float)pot1 / POT_INTER) * FREQ_MAX; // (adc_val[0] / 4096.0) * FREQ_MAX;
	  ducy = ((float)pot2 / POT_INTER) * 100;	// (adc_val[1] / 4096.0) * 100;
	  if (freq == 0)
		  freq = 1;
	  if (ducy > 100)
		  ducy = 100;

	  // Uncomment to set constant values
//	  freq = 10000;
//	  ducy = 30;

	  // TIM_CLK = APB_TIM_CLOCK / PRESCALAR
	  // FREQ = TIM_CLOCK / ARR
	  // DUTY (%) = (CCRx / ARR) * 100
	  while (1) {
		  clk = 72000000 / psc;
		  arr = clk / freq;
		  if (arr < 100) {
			  psc--;
			  continue;
		  } else if (arr > 65000) {
			  psc++;
			  continue;
		  }
		  ccr = ducy * arr / 100;
		  break;
	  }

	  TIM1->CCR1 = ccr & 0xFFFF;
	  TIM1->CCR2 = ccr & 0xFFFF;
	  TIM1->CCR3 = ccr & 0xFFFF;
	  TIM1->CCR4 = ccr & 0xFFFF;
	  TIM1->ARR = (arr-1) & 0xFFFF;
	  TIM1->PSC = psc-1;

//	  TIM2->CCR1 = ccr & 0xFFFF;
//	  TIM2->CCR2 = ccr & 0xFFFF;
	  TIM2->CCR3 = ccr & 0xFFFF;
	  TIM2->CCR4 = ccr & 0xFFFF;
	  TIM2->ARR = (arr-1) & 0xFFFF;
	  TIM2->PSC = psc-1;

	  TIM3->CCR1 = ccr & 0xFFFF;
	  TIM3->CCR2 = ccr & 0xFFFF;
//	  TIM3->CCR3 = ccr & 0xFFFF;
//	  TIM3->CCR4 = ccr & 0xFFFF;
	  TIM3->ARR = (arr-1) & 0xFFFF;
	  TIM3->PSC = psc-1;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (HAL_GetTick() - led_last_time > LED_INTERVAL) {
		  led_last_time = HAL_GetTick();
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void DMA_ADC_init(void) {
	// Start DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// Reset DMA1 Channel 1 configuration
	DMA1_Channel1->CCR = 0x00000000;

	// Configure DMA1 Channel 1
	// Peripheral -> Memory
	// Peripheral is 16-bit, no increment
	// Memory is 16-bit, increment
	// Circular mode
	DMA1_Channel1->CCR |= (0x01 <<DMA_CCR_PSIZE_Pos) | (0x01 <<DMA_CCR_MSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_CIRC;

	// Peripheral is ADC1 DR
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;

	// Memory is adc_val
	DMA1_Channel1->CMAR = (uint32_t)adc_val;

	// Set Memory Buffer size
	DMA1_Channel1->CNDTR = 2;

	// Enable DMA1 Channel 1
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	// Enable ADC DMA Request
	ADC1->CR2 |= ADC_CR2_DMA;

	// Enable ADC
	ADC1->CR2 |= ADC_CR2_ADON;

	// Start conversion
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

void crop_values(void) {
	if (pot1 < POT_MIN)
		pot1 = POT_MIN;
	if (pot2 < POT_MIN)
		pot2 = POT_MIN;
	if (pot1 > POT_MAX)
		pot1 = POT_MAX;
	if (pot2 > POT_MAX)
		pot2 = POT_MAX;
	pot1 -= POT_MIN;
	pot2 -= POT_MIN;
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

