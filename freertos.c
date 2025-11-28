/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name         : freertos.c
  * Description       : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h" // Assumed for huart2
#include "tim.h"   // Assumed for htim2 and htim3
#include "adc.h"   // Assumed for hadc1
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For abs()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// This structure holds all data that needs to be passed
// between tasks.
typedef struct {
    uint32_t joystick_x;
    uint32_t joystick_y;
    uint16_t servo_x_pulse;
    uint16_t servo_y_pulse;
    uint8_t  buzzer_active;
} SharedData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Joystick/Servo Constants ---
// X-Axis (Good Analog)
const uint16_t JOY_X_MIN = 0;
const uint16_t JOY_X_CENTER = 1987;
const uint16_t JOY_X_MAX = 4095;
const uint16_t X_DEADZONE = 50;

// Y-Axis (Broken, hard-code to center to stop jitter)
#define BROKEN_Y_AXIS_FIXED_PULSE 1500

// Servo Output
const uint16_t SERVO_MIN = 1000;
const uint16_t SERVO_CENTER = 1500;
const uint16_t SERVO_MAX = 2000;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// --- Global Shared Data ---
// The single instance of our shared data, protected by the mutex
volatile SharedData_t g_sharedData;

// --- Mutex Handle ---
osMutexId_t g_dataMutexHandle;

// --- Task Handles ---
osThreadId_t g_joystickTaskHandle;
osThreadId_t g_servoTaskHandle;
osThreadId_t g_buzzerTaskHandle;
osThreadId_t g_uartPrinterTaskHandle;


// --- Volatile ADC Buffer ---
// 'volatile' is critical because DMA writes to this memory,
// and the JoystickTask reads from it.

// --- UART Print Buffer ---
char g_txBuffer[100]; // Buffer for sprintf
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* --- Task Function Prototypes --- */
void vJoystickTask(void *argument);
void vServoTask(void *argument);
void vBuzzerTask(void *argument);
void vUARTPrinterTask(void *argument);

/* --- Helper Function Prototypes --- */
uint16_t map_asymmetric(uint32_t value);
void joystickTask(void *argument);
void servoTask(void *argument);
void buzzerTask(void *argument);
void printerTask(void *argument);
uint32_t Read_ADC_Channel(uint32_t channel);


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */


	  // Start the Servo PWM Timers
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);




  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	  /* add mutexes, ... */
	  // (We create it in the RTOS_THREADS section, per your example)
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	  /* add threads, ... */

	  // --- Create Mutex ---
	  const osMutexAttr_t g_dataMutex_attributes = {
	  	      .name = "dataMutex"
	  };

	  g_dataMutexHandle = osMutexNew(&g_dataMutex_attributes);
	  	  g_sharedData.joystick_x = 2048;
	    g_sharedData.joystick_y = 2048;
	    g_sharedData.servo_x_pulse = 1500;
	    g_sharedData.servo_y_pulse = 1500;
	    g_sharedData.buzzer_active = 0;
	  const osThreadAttr_t g_joystickTask_attributes = {
	      .name = "JoystickTask",
	      .priority = (osPriority_t) osPriorityHigh,
	      .stack_size = 500 * 4
	    };
	    g_joystickTaskHandle = osThreadNew(joystickTask, NULL, &g_joystickTask_attributes);

	    // --- Create ServoTask ---
	    const osThreadAttr_t g_servoTask_attributes = {
	      .name = "ServoTask",
	      .priority = (osPriority_t) osPriorityAboveNormal, // Servos are important
	      .stack_size = 256 * 4
	    };
	    g_servoTaskHandle = osThreadNew(servoTask, NULL, &g_servoTask_attributes);

	    // --- Create BuzzerTask ---
	    const osThreadAttr_t g_buzzerTask_attributes = {
	      .name = "BuzzerTask",
	      .priority = (osPriority_t) osPriorityLow,
	      .stack_size = 256 * 4
	    };
	    g_buzzerTaskHandle = osThreadNew(buzzerTask, NULL, &g_buzzerTask_attributes);

	    // --- Create UARTPrinterTask ---
	    const osThreadAttr_t g_uartPrinterTask_attributes = {
	      .name = "UARTPrinterTask",
	      .priority = (osPriority_t) osPriorityBelowNormal, // Printing is slow
	      .stack_size = 256 * 4 // sprintf needs more stack!
	    };
	    g_uartPrinterTaskHandle = osThreadNew(printerTask, NULL, &g_uartPrinterTask_attributes);


  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void play_tone(uint32_t freq, uint32_t duration_ms) {
	uint32_t timer_clk = 1000000; // 1 MHz based on 79 prescalar
	uint32_t period = timer_clk / freq;
	__HAL_TIM_SET_AUTORELOAD(&htim3, period - 1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, period / 4); // 25% duty cycle
	osDelay(duration_ms);
	// Silence between notes
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	osDelay(30);
}

/* --- Helper Function: Asymmetric Mapping (for X-Axis) --- */
uint16_t map_asymmetric(uint32_t value)
{
    // Check for deadzone
    if (abs(value - JOY_X_CENTER) < X_DEADZONE)
    {
        return SERVO_CENTER;
    }

    uint32_t out_val;
    if (value < JOY_X_CENTER) {
        // Map the "lower" half (e.g., 0-1987)
        out_val = (uint32_t)(value - JOY_X_MIN) * (SERVO_CENTER - SERVO_MIN) / (JOY_X_CENTER - JOY_X_MIN) + SERVO_MIN;
    } else {
        // Map the "upper" half (e.g., 1987-4095)
        out_val = (uint32_t)(value - JOY_X_CENTER) * (SERVO_MAX - SERVO_CENTER) / (JOY_X_MAX - JOY_X_CENTER) + SERVO_CENTER;
    }

    if (out_val < SERVO_MIN) return SERVO_MIN;
    if (out_val > SERVO_MAX) return SERVO_MAX;
    return (uint16_t)out_val;
}

void joystickTask(void *argument)
{
    const uint32_t task_delay = 20; // 20ms = 50Hz
    uint32_t raw_x, raw_y;
    uint16_t mapped_x, mapped_y;

    for(;;)
    {
        // 1. Read ADC values from DMA buffer
        raw_x = Read_ADC_Channel(ADC_CHANNEL_5);
        raw_y = Read_ADC_Channel(ADC_CHANNEL_6);

        // 2. Map the X-axis value
        mapped_x = map_asymmetric(raw_x);
        mapped_y = map_asymmetric(raw_y);


        // 3. Update shared data (with mutex protection)
        if(osMutexAcquire(g_dataMutexHandle, 10) == osOK)
        {
            g_sharedData.joystick_x     = raw_x;
            g_sharedData.joystick_y     = raw_y;
            g_sharedData.servo_x_pulse  = mapped_x;
            g_sharedData.servo_y_pulse  = mapped_y;

            osMutexRelease(g_dataMutexHandle);
        }

        // 4. Wait for next cycle
        osDelay(task_delay);
    }
}

void servoTask(void *argument)
{
    const uint32_t task_delay = 20; // 20ms = 50Hz
    uint16_t local_x_pulse, local_y_pulse;

    for(;;)
    {
        // 1. Read shared data (with mutex protection)
        if(osMutexAcquire(g_dataMutexHandle, 10) == osOK)
        {
            local_x_pulse = g_sharedData.servo_x_pulse;
            local_y_pulse = g_sharedData.servo_y_pulse;
            osMutexRelease(g_dataMutexHandle);
        }
        else
        {
            // Mutex timeout - use safe center position
            local_x_pulse = SERVO_CENTER;
            local_y_pulse = SERVO_CENTER;
        }

        // 2. Update servo positions (NO mutex needed for hardware access)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, local_x_pulse);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, local_y_pulse);

        // 3. Wait for next cycle
        osDelay(task_delay);
    }
}

void buzzerTask(void *argument)
{
    const uint32_t task_delay = 50; // 50ms = 20Hz
    uint8_t local_buzzer_flag;

    for(;;)
    {
        // 1. Lock the Mutex to read the flag
        if(osMutexAcquire(g_dataMutexHandle, 10) == osOK)
        {
            local_buzzer_flag = g_sharedData.buzzer_active;
            osMutexRelease(g_dataMutexHandle);
        }

        // 2. Control the hardware
        if(local_buzzer_flag)
        {
        	play_tone(440, 300); // A4
        	play_tone(523, 300); // C5
        	play_tone(659, 300); // E5
        }
        else
        {
        	play_tone(440, 300); // A4
        	play_tone(523, 300); // C5
        	play_tone(659, 300); // E5
        }

        // 3. Wait for the next cycle
        osDelay(task_delay);
    }
}

void printerTask(void *argument)
{
    const uint32_t task_delay = 250; // Print slowly: 4 times/sec
    uint32_t local_x, local_y;

    for(;;)
    {
        // 1. Lock the Mutex to read data
        if(osMutexAcquire(g_dataMutexHandle, 10) == osOK)
        {
            local_x = g_sharedData.joystick_x;
            local_y = g_sharedData.joystick_y;
            osMutexRelease(g_dataMutexHandle);
        }
        else
        {
            local_x = 9999; // Indicate error
            local_y = 9999;
        }

        // 2. Format the string (mimicking your example)
        int len = sprintf(g_txBuffer, "X: %4lu, Y: %4lu\r\n",
                          local_x,
                          local_y);

        // 3. Transmit (This is blocking, but OK for a low-priority task)
        //    (Assumes huart2 is defined in usart.h)
        HAL_UART_Transmit(&huart2, (uint8_t*)g_txBuffer, len, HAL_MAX_DELAY);

        // 4. Wait for the next cycle
        osDelay(task_delay);
    }
}
uint32_t Read_ADC_Channel(uint32_t channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(&hadc1);
}

/* USER CODE END Application */

