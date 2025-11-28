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
#include "cmsis_os.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>   // for abs()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ===== SERVO PULSE LIMITS =====
#define SERVO_MIN_PULSE 1000   // 1.0 ms
#define SERVO_MAX_PULSE 2000   // 2.0 ms
#define SERVO_STOP      1500   // 1.5 ms (center)

#define MY_I2C_ADDR  (0x68 << 1)   // 0x68 is 7-bit address

// ===== CALIBRATION DATA =====
// X Axis (Restricted Range)
#define X_MIN     976
#define X_CENTER  1987
#define X_MAX     3019

// Y Axis (Full Range)
#define Y_MIN     0
#define Y_CENTER  2005
#define Y_MAX     4032

// Deadzone to prevent drift / cross-talk between axes
#define DEAD_BAND 150

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
 #include "i2c.h"


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

// Call this from your FreeRTOS task loop
void Process_Joystick(void);

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Start PWM for both servos (TIM2_CH3, TIM3_CH2, plus TIM3_CH3 for buzzer)
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) {
      Error_Handler();
  }


  // Set initial center position for both servos
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, SERVO_STOP);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_STOP);

  // LED blink to confirm init (optional)
  for (int i = 0; i < 3; i++) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

      HAL_Delay(200);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

      HAL_Delay(200);
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //uint8_t data[6];
	 // I2C_ReadBytes(0x68, 0x3B, data, 6);   // Read accelerometer 6 bytes
	  char msg[90];
	  sprintf(msg,"PHOTO: %d",LightSensor_ReadLux());
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    // DO NOT put Process_Joystick() here once FreeRTOS is running.
    // Call Process_Joystick() from a FreeRTOS task instead (e.g. defaultTask in freertos.c).
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Read joystick (ADC) and update servo PWMs.
  * @note   CALL THIS FROM A FREERTOS TASK (e.g. defaultTask in freertos.c),
  *         NOT from main's while(1) once the scheduler is running.
  */
void Process_Joystick(void)
{
    uint32_t rawX = 0;
    uint32_t rawY = 0;

    // Example using polling for 2 channels in sequence.
    // Make sure MX_ADC1_Init() is configured to scan both channels in sequence.
    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        rawX = HAL_ADC_GetValue(&hadc1);  // first conversion: X
    }

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        rawY = HAL_ADC_GetValue(&hadc1);  // second conversion: Y
    }

    HAL_ADC_Stop(&hadc1);

    // Map to servo pulses using your calibrated asymmetric mapping
    uint32_t pwmX = Map_Joystick_Axis(rawX, X_MIN, X_CENTER, X_MAX);
    uint32_t pwmY = Map_Joystick_Axis(rawY, Y_MIN, Y_CENTER, Y_MAX);

    // Write to servo timers
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwmX); // X-axis servo
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmY); // Y-axis servo
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
