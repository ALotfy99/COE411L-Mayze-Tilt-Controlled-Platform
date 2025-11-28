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
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h" // for huart2
#include "tim.h"   // for htim2, htim3
#include "adc.h"   // for hadc1
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // abs()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Shared data between tasks
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

// ===== Joystick Calibration =====
// X-axis: [418, 3100], center = almost 2000
const uint16_t JOY_X_MIN    = 418;
const uint16_t JOY_X_CENTER = 2000;
const uint16_t JOY_X_MAX    = 3100;

// Y-axis: [0, 4095], center = almost 2000
const uint16_t JOY_Y_MIN    = 0;
const uint16_t JOY_Y_CENTER = 2000;
const uint16_t JOY_Y_MAX    = 3218;

// Servo pulse range (µs / timer ticks)
const uint16_t SERVO_MIN    = 1000;
const uint16_t SERVO_CENTER = 1500;
const uint16_t SERVO_MAX    = 2000;
// Right = positive side, Left = negative side (smaller pulse)
#define SERVO_Y_MAX_OFFSET   150   // range of movement on servo y
#define SERVO_Y_SLEW_STEP    30     // lower = slower, higher = faster
#define SERVO_X_SLEW_STEP   30    // lower = slower, higher = faster

// Shift Y center: negative = more to the left, positive = more to the right
#define SERVO_Y_CENTER_TRIM  -50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// Shared data instance
volatile SharedData_t g_sharedData;

// Mutex
osMutexId_t g_dataMutexHandle;

// Task handles
osThreadId_t g_joystickTaskHandle;
osThreadId_t g_servoTaskHandle;
osThreadId_t g_buzzerTaskHandle;
osThreadId_t g_uartPrinterTaskHandle;

// UART TX buffer
char g_txBuffer[100];

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
void debugPrint(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void StartDefaultTask(void *argument);

/* Task functions */
void joystickTask(void *argument);
void servoTask(void *argument);
void buzzerTask(void *argument);
void printerTask(void *argument);
// Helper function: read 5 samples from ADC_CHANNEL_15, average them,
// and trigger the buzzer if the average is above 1200.

/* Helpers */
uint32_t Read_ADC_Channel(uint32_t channel);

static int32_t MapRawAxisToPercent(uint32_t raw,
                                   uint32_t min,
                                   uint32_t neutral,
                                   uint32_t max);

static void Apply_CrossAxis_Compensation(int32_t *x, int32_t *y);

void Read_Calibrated_Joystick(int32_t *x_out, int32_t *y_out);

uint16_t Map_JoyPercent_To_Pulse(int32_t pct);

void play_tone(uint32_t duration_ms);
void debugPrint(char *msg);
void Check_Adc15_And_Buzz(void)
{
    uint32_t sum = 0;
    uint32_t avg = 0;

    for (int i = 0; i < 5; i++)
    {
        sum += Read_ADC_Channel(ADC_CHANNEL_15);  // your existing ADC helper
    }

    avg = sum / 5;

    if (avg > 1200)
    {
        // Call your existing tone function
        play_tone(300);
    }
}

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

// Center servos on startup
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, SERVO_CENTER); // X servo
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_CENTER + SERVO_Y_CENTER_TRIM); // Y servo

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */

  const osMutexAttr_t g_dataMutex_attributes = {
      .name = "dataMutex"
  };
  g_dataMutexHandle = osMutexNew(&g_dataMutex_attributes);
  if (g_dataMutexHandle == NULL) {
      debugPrint("ERROR: Failed to create dataMutex!\r\n");
  }

  // Initialize shared data to center
  g_sharedData.joystick_x    = JOY_X_CENTER;
  g_sharedData.joystick_y    = JOY_Y_CENTER;
  g_sharedData.servo_x_pulse = SERVO_CENTER;
  g_sharedData.servo_y_pulse = SERVO_CENTER + SERVO_Y_CENTER_TRIM;
  g_sharedData.buzzer_active = 1;

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

  // --- Create Joystick Task ---
  const osThreadAttr_t g_joystickTask_attributes = {
      .name       = "JoystickTask",
      .priority   = (osPriority_t) osPriorityHigh,
      .stack_size = 500 * 4
  };
  g_joystickTaskHandle = osThreadNew(joystickTask, NULL, &g_joystickTask_attributes);
  if (g_joystickTaskHandle == NULL) {
      debugPrint("ERROR: Failed to create JoystickTask!\r\n");
  }

  // --- Create Servo Task ---
  const osThreadAttr_t g_servoTask_attributes = {
      .name       = "ServoTask",
      .priority   = (osPriority_t) osPriorityAboveNormal,
      .stack_size = 256 * 4
  };
  g_servoTaskHandle = osThreadNew(servoTask, NULL, &g_servoTask_attributes);
  if (g_servoTaskHandle == NULL) {
      debugPrint("ERROR: Failed to create ServoTask!\r\n");
  }

  // --- Create Buzzer Task ---
  const osThreadAttr_t g_buzzerTask_attributes = {
      .name       = "BuzzerTask",
      .priority   = (osPriority_t) osPriorityNormal,
      .stack_size = 256 * 4
  };
  g_buzzerTaskHandle = osThreadNew(buzzerTask, NULL, &g_buzzerTask_attributes);
  if (g_buzzerTaskHandle == NULL) {
      debugPrint("ERROR: Failed to create BuzzerTask!\r\n");
  }

  // --- Create UART Printer Task ---
  const osThreadAttr_t g_uartPrinterTask_attributes = {
      .name       = "UARTPrinterTask",
      .priority   = (osPriority_t) osPriorityNormal,
      .stack_size = 256 * 4
  };
  g_uartPrinterTaskHandle = osThreadNew(printerTask, NULL, &g_uartPrinterTask_attributes);
  if (g_uartPrinterTaskHandle == NULL) {
      debugPrint("ERROR: Failed to create UARTPrinterTask!\r\n");
  }

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

#define PCF_MODULE_ADDRESS  (0x48 << 1)  // 7-bit base address shifted once for HAL
#define PCF_READ_AIN1       (0x40 | 0x01)  // 0x40 = enable ADC, 0x01 = select AIN1

//reading value from photoresistor (0 to 250)
int LightSensor_ReadLux(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t cmd = PCF_READ_AIN1;
    uint8_t dummy;
    uint8_t value;

    // Step 1: Send control byte (select AIN1 and turn on ADC)
    status = HAL_I2C_Master_Transmit(hi2c, PCF_MODULE_ADDRESS, &cmd, 1, 100);
    if (status != HAL_OK)
        return -1;

    // Step 2: Dummy read (previous conversion)
    status = HAL_I2C_Master_Receive(hi2c, PCF_MODULE_ADDRESS, &dummy, 1, 100);
    if (status != HAL_OK)
        return -1;

    // Step 3: Actual conversion result
    status = HAL_I2C_Master_Receive(hi2c, PCF_MODULE_ADDRESS, &value, 1, 100);
    if (status != HAL_OK)
        return -1;

    // Return 0–255 light level (higher = darker)
    return value;
}


void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void play_tone(uint32_t duration_ms) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); // buzzer ON
	osDelay(duration_ms);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
}

/* ================================================================
   JOYSTICK PROCESSING PIPELINE (CLEAN VERSION — NO ROTATION MATH)
   ================================================================

   This module converts raw joystick ADC readings into stable,
   clean, servo-ready control signals with correct X/Y axis
   separation and minimal noise.

   The processing happens in 4 stages:

   ---------------------------------------------------------------
   1. RAW ADC READ
   ---------------------------------------------------------------
   We read the hardware joystick directly using:
       raw_x = Read_ADC_Channel(ADC_CHANNEL_5);
       raw_y = Read_ADC_Channel(ADC_CHANNEL_6);

   These values are typically in the range 0–4095.

   ---------------------------------------------------------------
   2. NORMALIZATION TO PERCENT (-100 .. +100)
   ---------------------------------------------------------------
   Using joystick calibration ranges (min, neutral, max),
   each raw value is converted into a percentage:

        -100 = fully left / down
         0   = center
        +100 = fully right / up

   The function:
       MapRawAxisToPercent()

   handles asymmetric ranges (neutral not centered),
   clamps values correctly, and produces clean percent outputs.

   ---------------------------------------------------------------
   3. CROSS-AXIS COMPENSATION
   ---------------------------------------------------------------
   Real joysticks never move perfectly straight. When the stick
   is pushed up, the X axis moves slightly; when pushed left,
   Y moves slightly. This produces diagonal drift.

   The function:
       Apply_CrossAxis_Compensation()

   fixes this by:
     • When movement is strongly horizontal → zero out Y
     • When movement is strongly vertical   → zero out X
     • When diagonal movement is detected   → suppress the
       weaker axis so movement feels natural

   This gives you clean, stable axes without using any rotation
   matrices or trigonometry.

   ---------------------------------------------------------------
   4. MAPPING TO SERVO PULSE WIDTH
   ---------------------------------------------------------------
   After X/Y are corrected and percent-based, they are mapped
   into the servo’s pulse range:

       SERVO_MIN  → -100%
       SERVO_CENTER →   0%
       SERVO_MAX  → +100%

   via:
       Map_JoyPercent_To_Pulse()

   This keeps your joystick mapping independent of hardware
   ADC ranges — only percent values flow into the servo layer.

   ---------------------------------------------------------------
   5. SLEW, OFFSET, AND TRIM (servoTask)
   ---------------------------------------------------------------
   The servoTask applies "physical movement shaping":

     • Slew-rate limiting
         Prevents servos from jumping too fast (protects load)
     • Center offset
         Fixes mechanical misalignment
     • Y-axis max deviation limit
         Prevents flipping or dangerous tilt

   These adjustments are NOT done in joystick space — they are
   done after percent-to-pulse conversion to keep the logic clean.

   ---------------------------------------------------------------
   Summary:
     joystickTask:
         - Reads ADC → percent → cross-axis fix → pulse targets
     servoTask:
         - Applies slew, trimming, mechanical limits
         - Updates hardware PWM channels

   This architecture keeps your joystick logic clean and robust,
   and your servo movement safe, smooth, and predictable.
   ================================================================ */

//turn raw voltage value into a calibrated percentage
static int32_t MapRawAxisToPercent(uint32_t raw,
                                   uint32_t min,
                                   uint32_t neutral,
                                   uint32_t max)
{
    int32_t range_pos = (int32_t)max - (int32_t)neutral;
    int32_t range_neg = (int32_t)neutral - (int32_t)min;
    int32_t out;

    if ((int32_t)raw >= (int32_t)neutral) {
        out = (int32_t)(( (int32_t)raw - (int32_t)neutral ) * 100) / range_pos;
    } else {
        out = (int32_t)(( (int32_t)raw - (int32_t)neutral ) * 100) / range_neg;
    }

    // Clamp to [-100, 100]
    if (out > 100)  out = 100;
    if (out < -100) out = -100;

    return out;
}

//create a dominant axis to deal with cross-talk between axis x and y
static void Apply_CrossAxis_Compensation(int32_t *x, int32_t *y)
{
    int32_t original_x = *x;
    int32_t original_y = *y;

    // Snap mostly-horizontal
    if (abs(original_x) > 70 && abs(original_y) < 30) {
        *y = 0;
    }
    // Snap mostly-vertical
    else if (abs(original_y) > 70 && abs(original_x) < 30) {
        *x = 0;
    }
    // Strong diagonal → reduce the weaker axis
    else if (abs(original_x) > 40 && abs(original_y) > 40) {
        if (abs(original_x) > abs(original_y)) {
            *y = (original_y * 30) / 100;  // shrink Y
        } else {
            *x = (original_x * 30) / 100;  // shrink X
        }
    }
}

//calibrating joystick in percentage form
void Read_Calibrated_Joystick(int32_t *x_out, int32_t *y_out)
{
    uint32_t raw_x = Read_ADC_Channel(ADC_CHANNEL_5);
    uint32_t raw_y = Read_ADC_Channel(ADC_CHANNEL_6);

    int32_t x_pct = MapRawAxisToPercent(raw_x,
                                        JOY_X_MIN,
                                        JOY_X_CENTER,
                                        JOY_X_MAX);

    int32_t y_pct = MapRawAxisToPercent(raw_y,
                                        JOY_Y_MIN,
                                        JOY_Y_CENTER,
                                        JOY_Y_MAX);

    *x_out = x_pct;
    *y_out = y_pct;

    // Cross-axis cleanup (no cos/sin, just logic)
    Apply_CrossAxis_Compensation(x_out, y_out);
}

//converting percentage to servo pulse value
uint16_t Map_JoyPercent_To_Pulse(int32_t pct)
{
    // pct in [-100 .. 100], 0 is center
    // DEAD_BAND is your existing deadzone in percent (e.g. 5 or 10)
    const int32_t DEAD_BAND_PERCENT = 5;

    if (abs(pct) < DEAD_BAND_PERCENT) {
        return SERVO_CENTER;  // center pulse
    }

    int32_t out;

    if (pct < 0) {
        // [-100 .. 0) -> [SERVO_MIN_PULSE .. SERVO_STOP)
        out = (pct + 100) * (SERVO_CENTER - SERVO_MIN) / 100 + SERVO_MIN;
    } else {
        // (0 .. 100] -> (SERVO_STOP .. SERVO_MAX_PULSE]
        out = pct * (SERVO_MAX - SERVO_CENTER) / 100 + SERVO_CENTER;
    }

    if (out < SERVO_MIN) out = SERVO_MIN;
    if (out > SERVO_MAX) out = SERVO_MAX;

    return (uint16_t)out;
}

//joystick task will use functions explained above to create the final value which will be passed to the
//servos
void joystickTask(void *argument)
{
	const uint32_t task_delay = 20; // 50 Hz
	    int32_t joy_x_pct, joy_y_pct;
	    uint16_t mapped_x, mapped_y;

	    for(;;)
	    {
	        // 1. Read calibrated joystick in [-100..100] with cross-axis compensation
	        Read_Calibrated_Joystick(&joy_x_pct, &joy_y_pct);

	        // 2. Map to servo pulses
	        mapped_x = Map_JoyPercent_To_Pulse(joy_x_pct);
	        mapped_y = Map_JoyPercent_To_Pulse(joy_y_pct);

	        // 3. Update shared data (servoTask still does slew + trim)
	        if(osMutexAcquire(g_dataMutexHandle, 10) == osOK)
	        {
	            g_sharedData.joystick_x    = joy_x_pct; // or keep for debugging
	            g_sharedData.joystick_y    = joy_y_pct;
	            g_sharedData.servo_x_pulse = mapped_x;
	            g_sharedData.servo_y_pulse = mapped_y;
	            osMutexRelease(g_dataMutexHandle);
	        }

	        osDelay(task_delay);
	    }
}

//servo task where we move servo but we restrict movement on y so that board does not tilt too much
//and we control speed of movment. We also recenter due to mechanical error (propeller misplacement)
void servoTask(void *argument)
{
    const uint32_t task_delay = 20; // 50 Hz
    uint16_t local_x_pulse, local_y_pulse;

    // Start filtered Y at shifted center
    static uint16_t filtered_y_pulse = SERVO_CENTER + SERVO_Y_CENTER_TRIM;
    static uint16_t filtered_x_pulse = SERVO_CENTER;

    for(;;)
    {
        if(osMutexAcquire(g_dataMutexHandle, 10) == osOK)
        {
            local_x_pulse = g_sharedData.servo_x_pulse;
            local_y_pulse = g_sharedData.servo_y_pulse;
            osMutexRelease(g_dataMutexHandle);
        }
        else
        {
            local_x_pulse = SERVO_CENTER;
            local_y_pulse = SERVO_CENTER + SERVO_Y_CENTER_TRIM;
        }

        // ---------- X smoothing (slow movement) ----------
        int16_t diffX = (int16_t)local_x_pulse - (int16_t)filtered_x_pulse;

        if (diffX > SERVO_X_SLEW_STEP)
            diffX = SERVO_X_SLEW_STEP;
        else if (diffX < -SERVO_X_SLEW_STEP)
            diffX = -SERVO_X_SLEW_STEP;

        filtered_x_pulse = (uint16_t)((int16_t)filtered_x_pulse + diffX);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, filtered_x_pulse);


        // ---------- Y: symmetric range around NEW center ----------

        // Treat SERVO_CENTER as logical center coming from map_axis
        int16_t offset = (int16_t)local_y_pulse - (int16_t)SERVO_CENTER;

        // Clamp offset symmetrically
        if (offset > SERVO_Y_MAX_OFFSET)
            offset = SERVO_Y_MAX_OFFSET;
        else if (offset < -SERVO_Y_MAX_OFFSET)
            offset = -SERVO_Y_MAX_OFFSET;

        // NEW center = SERVO_CENTER + SERVO_Y_CENTER_TRIM
        uint16_t target_y_pulse = (uint16_t)(
            (int16_t)SERVO_CENTER + SERVO_Y_CENTER_TRIM + offset
        );

        // Slew toward target
        int16_t diff = (int16_t)target_y_pulse - (int16_t)filtered_y_pulse;

        if (diff > SERVO_Y_SLEW_STEP)
            diff = SERVO_Y_SLEW_STEP;
        else if (diff < -SERVO_Y_SLEW_STEP)
            diff = -SERVO_Y_SLEW_STEP;

        filtered_y_pulse = (uint16_t)((int16_t)filtered_y_pulse + diff);

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, filtered_y_pulse);

        osDelay(task_delay);
    }
}

//buzzer, buzzes at the start of the game and win you finish the maze
void buzzerTask(void *argument)
{
    const uint32_t task_delay = 50;
    uint8_t local_buzzer_flag;

    for(;;)
    {
        if(osMutexAcquire(g_dataMutexHandle, 10) == osOK)
        {
            local_buzzer_flag = g_sharedData.buzzer_active;
            osMutexRelease(g_dataMutexHandle);
        }



        if(local_buzzer_flag)
        {
            //Short activation to indicate event happening
        	play_tone(300);
        	osMutexAcquire(g_dataMutexHandle, 10);
            g_sharedData.buzzer_active = 0;
            osMutexRelease(g_dataMutexHandle);

        }
        else
        {
            // Silent when inactive
            osDelay(300);
        }

        osDelay(task_delay);
    }
}

void printerTask(void *argument)
{
    const uint32_t task_delay = 2000;

    for (;;)
    {
        int32_t ldr = LightSensor_ReadLux(&hi2c1);

        //if value bigger than 150, ball is covering photoresistor and you win
        if (ldr >= 150) {
            char buf[32];
            sprintf(buf, "YOU WIN!\r\n");
            HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);
            osMutexAcquire(g_dataMutexHandle, 10);
			g_sharedData.buzzer_active = 1;
			osMutexRelease(g_dataMutexHandle);

        } else {

        }

        osDelay(task_delay);
    }
}

//readiing joystick voltages from channel 5 & 6
uint32_t Read_ADC_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

    return HAL_ADC_GetValue(&hadc1);
}

/* USER CODE END Application */

