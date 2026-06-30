/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include "fsm.h"
#include "hc06.h"
#include "lcd.h"
#include "mpu6050.h"
#include "soft_i2c.h"
#include "kalman.h"
#include "line_sensor.h"
#include "paramstore.h"
#include "pid.h"
#include "vl53l1x.h"

#include <math.h>
#include <stdio.h>

#define STAT_GYRO_THRESH  1.0f
#define STAT_WINDOW       200U
#define SPEED_PERIOD_MS   10U
#define TOF_PERIOD_MS     50U
#define STREAM_PERIOD_MS  50U
#define LCD_PERIOD_MS     200U
#define MOTOR_TEST_DUTY   2000
#define MOTOR_TEST_MS     1000U
#define MOTOR_TEST_GAP_MS 500U
/* PLACEHOLDER speed-PID gains; USER must tune to hardware. */
#define SPEED_KP          5.0f
#define SPEED_KI          0.0f
#define SPEED_KD          0.0f

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
volatile uint8_t g_param_save_request = 0U;
static PID pid_speed[MOTOR_COUNT];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t Hall_ReadLevel(MotorId wheel);
static void MotorTest_Run(void);
static const char *MotorTest_Name(MotorId wheel);

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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  uint32_t last_tick;
  uint32_t last_speed_tick;
  uint32_t last_tof_tick;
  uint32_t last_stream_tick;
  uint32_t last_lcd_tick;
  uint32_t stat_count;
  float stat_sum;
  float stat_sumsq;
  float stored_r;
  float stored_bias;
  float yaw;
  float line_error;
  uint8_t line_ok;
  uint8_t mpu_ok;
  uint8_t tof_ok;

  Motor_Init();
  Encoder_Init();
  FSM_Init();
  HC06_Init();
  SoftI2C_Init();
  tof_ok = (TOF_Init_All() == 0U) ? 1U : 0U;
  if (tof_ok != 0U)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)"TOF OK\r\n", 8U, HAL_MAX_DELAY);
  }
  else
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)"TOF FAIL\r\n", 10U, HAL_MAX_DELAY);
  }
  line_ok = (LineSensor_Init((const LineSensorConfig *)0) == HAL_OK) ? 1U : 0U;

  if (MPU6050_Init() == 0U)
  {
    mpu_ok = 1U;
    HAL_UART_Transmit(&huart2, (uint8_t *)"MPU OK\r\n", 8U, HAL_MAX_DELAY);
  }
  else
  {
    mpu_ok = 0U;
    HAL_UART_Transmit(&huart2, (uint8_t *)"MPU FAIL\r\n", 10U, HAL_MAX_DELAY);
  }

  Kalman_Init();
  yaw = 0.0f;
  line_error = 0.0f;
  if (ParamStore_Load(&stored_r, &stored_bias) != 0U)
  {
    Kalman_SetR(stored_r);
    Kalman_SetBias(stored_bias);
    HAL_UART_Transmit(&huart2, (uint8_t *)"PARAM LOADED\r\n", 14U, HAL_MAX_DELAY);
  }
  else
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)"PARAM DEFAULT\r\n", 15U, HAL_MAX_DELAY);
  }

  for (MotorId w = MOTOR_LF; w < MOTOR_COUNT; w++)
  {
    PID_Init(&pid_speed[w], SPEED_KP, SPEED_KI, SPEED_KD, -3599.0f, 3599.0f);
  }

  stat_count = 0U;
  stat_sum = 0.0f;
  stat_sumsq = 0.0f;
  LCD_Init();
  LCD_Update(FSM_GetStateName(), yaw, mpu_ok);
  last_tick = HAL_GetTick();
  last_speed_tick = last_tick;
  last_tof_tick = last_tick;
  last_stream_tick = last_tick;
  last_lcd_tick = last_tick;
  MotorTest_Run();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now;
    float dt;
    float omega_dps = 0.0f;
    float accel_angle;
    uint8_t sample_ok;

    HC06_Process();

    if (g_param_save_request != 0U)
    {
      uint8_t save_ok;

      g_param_save_request = 0U;
      save_ok = ParamStore_Save(Kalman_GetR(), Kalman_GetBias());
      if (save_ok != 0U)
      {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SAVED\r\n", 7U, HAL_MAX_DELAY);
      }
      else
      {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SAVE_FAIL\r\n", 11U, HAL_MAX_DELAY);
      }
    }

    now = HAL_GetTick();
    dt = (float)(now - last_tick) / 1000.0f;
    if (dt <= 0.0f)
    {
      continue;
    }
    last_tick = now;

    if ((now - last_speed_tick) >= SPEED_PERIOD_MS)
    {
      float speed_dt;
      MotorId w;

      speed_dt = (float)(now - last_speed_tick) / 1000.0f;
      Encoder_Sample(speed_dt);
      for (w = MOTOR_LF; w < MOTOR_COUNT; w++)
      {
        float measured;
        float out;
        float setpoint;
        uint32_t primask;
        uint8_t output_valid;

        setpoint = 0.0f;
        output_valid = 0U;
        if (FSM_IsMotionEnabled() != 0U)
        {
          setpoint = FSM_GetSpeedSetpoint(w);
          measured = Encoder_GetSpeed(w);
          if (setpoint < 0.0f)
          {
            measured = -measured;
          }
          out = PID_Update(&pid_speed[w], setpoint, measured, speed_dt);
          output_valid = 1U;
        }
        else
        {
          PID_Reset(&pid_speed[w]);
          out = 0.0f;
        }

        primask = __get_PRIMASK();
        __disable_irq();
        if (FSM_IsEmergency() != 0U)
        {
          PID_Reset(&pid_speed[w]);
          Motor_StopAll();
        }
        else if ((output_valid == 0U) || (FSM_IsMotionEnabled() == 0U)
                 || (FSM_GetSpeedSetpoint(w) != setpoint))
        {
          PID_Reset(&pid_speed[w]);
          Motor_SetDuty(w, 0);
        }
        else
        {
          Motor_SetDuty(w, (int16_t)out);
        }
        if (primask == 0U)
        {
          __enable_irq();
        }
      }
      last_speed_tick = now;
    }

    sample_ok = (MPU6050_Read(&omega_dps, &accel_angle) == 0U) ? 1U : 0U;
    if (sample_ok != 0U)
    {
      mpu_ok = 1U;
      yaw = Kalman_Update(accel_angle, omega_dps, dt);

      if (fabsf(omega_dps) < STAT_GYRO_THRESH)
      {
        stat_count++;
        stat_sum += accel_angle;
        stat_sumsq += accel_angle * accel_angle;

        if (stat_count >= STAT_WINDOW)
        {
          float variance;

          /* R is Var(accel_angle) in deg^2 when the gyro indicates the robot is at rest. */
          variance = (stat_sumsq - ((stat_sum * stat_sum) / (float)STAT_WINDOW))
                     / ((float)STAT_WINDOW - 1.0f);
          if (variance < 1e-6f)
          {
            variance = 1e-6f;
          }
          Kalman_SetR(variance);

          stat_count = 0U;
          stat_sum = 0.0f;
          stat_sumsq = 0.0f;
        }
      }
      else
      {
        stat_count = 0U;
        stat_sum = 0.0f;
        stat_sumsq = 0.0f;
      }
    }
    else
    {
      mpu_ok = 0U;
      stat_count = 0U;
      stat_sum = 0.0f;
      stat_sumsq = 0.0f;
    }

    {
      LineSensorSample line_sample;

      line_sample = LineSensor_Read();
      line_error = line_sample.error;
      line_ok = line_sample.active;
    }

    if ((tof_ok != 0U) && ((now - last_tof_tick) >= TOF_PERIOD_MS))
    {
      if (TOF_UpdateAll() == 0U)
      {
        FSM_SetObstacle(TOF_IsObstacle());
      }
      last_tof_tick = now;
    }
    FSM_Dispatch(yaw, omega_dps, dt, line_error, line_ok);

    if ((now - last_stream_tick) >= STREAM_PERIOD_MS)
    {
      char buffer[32];
      char spdbuf[160];
      char tofbuf[64];
      int length;
      int speed_length;
      int tof_length;
      int32_t yaw_centi;
      long setpoint_drpm[MOTOR_COUNT];
      long measured_drpm[MOTOR_COUNT];
      unsigned long counts[MOTOR_COUNT];
      MotorId wheel;

      yaw_centi = (int32_t)(yaw * 100.0f);
      length = snprintf(buffer, sizeof(buffer), "YAW=%ld\r\n", (long)yaw_centi);
      if (length > 0)
      {
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
      }
      for (wheel = MOTOR_LF; wheel < MOTOR_COUNT; wheel++)
      {
        setpoint_drpm[wheel] = (long)(FSM_GetSpeedSetpoint(wheel) * 10.0f);
        measured_drpm[wheel] = (long)(Encoder_GetSpeed(wheel) * 10.0f);
        counts[wheel] = (unsigned long)Encoder_GetCount(wheel);
      }
      speed_length = snprintf(spdbuf, sizeof(spdbuf),
                              "SPD=%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld;CNT=%lu,%lu,%lu,%lu;HALL=%u,%u,%u,%u\r\n",
                              setpoint_drpm[MOTOR_LF], measured_drpm[MOTOR_LF],
                              setpoint_drpm[MOTOR_RF], measured_drpm[MOTOR_RF],
                              setpoint_drpm[MOTOR_LR], measured_drpm[MOTOR_LR],
                              setpoint_drpm[MOTOR_RR], measured_drpm[MOTOR_RR],
                              counts[MOTOR_LF], counts[MOTOR_RF],
                              counts[MOTOR_LR], counts[MOTOR_RR],
                              (unsigned int)Hall_ReadLevel(MOTOR_LF),
                              (unsigned int)Hall_ReadLevel(MOTOR_RF),
                              (unsigned int)Hall_ReadLevel(MOTOR_LR),
                              (unsigned int)Hall_ReadLevel(MOTOR_RR));
      if (speed_length > 0)
      {
        HAL_UART_Transmit(&huart2, (uint8_t *)spdbuf, (uint16_t)speed_length, HAL_MAX_DELAY);
      }
      if (tof_ok != 0U)
      {
        TOF_Sample tof_front;
        TOF_Sample tof_left;
        TOF_Sample tof_right;

        tof_front = TOF_GetSample(TOF_FRONT);
        tof_left = TOF_GetSample(TOF_LEFT);
        tof_right = TOF_GetSample(TOF_RIGHT);
        tof_length = snprintf(tofbuf, sizeof(tofbuf),
                              "TOF=%u,%u,%u;OBS=%u\r\n",
                              (unsigned int)tof_front.distance_mm,
                              (unsigned int)tof_left.distance_mm,
                              (unsigned int)tof_right.distance_mm,
                              (unsigned int)TOF_IsObstacle());
        if (tof_length > 0)
        {
          HAL_UART_Transmit(&huart2, (uint8_t *)tofbuf, (uint16_t)tof_length, HAL_MAX_DELAY);
        }
      }
      last_stream_tick = now;
    }

    if ((now - last_lcd_tick) >= LCD_PERIOD_MS)
    {
      LCD_Update(FSM_GetStateName(), yaw, mpu_ok);
      last_lcd_tick = now;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, VL53_XSHUT_FRONT_Pin|VL53_XSHUT_LEFT_Pin|VL53_XSHUT_RIGHT_Pin);

  /**/
  GPIO_InitStruct.Pin = VL53_XSHUT_FRONT_Pin|VL53_XSHUT_LEFT_Pin|VL53_XSHUT_RIGHT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE4);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE6);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE0);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE1);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE7);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_6;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(HALL_RR_GPIO_Port, HALL_RR_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SHOCK_GPIO_Port, SHOCK_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(HALL_FL_GPIO_Port, HALL_FL_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(HALL_FR_GPIO_Port, HALL_FR_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(HALL_RL_GPIO_Port, HALL_RL_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(HALL_RR_GPIO_Port, HALL_RR_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SHOCK_GPIO_Port, SHOCK_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(HALL_FL_GPIO_Port, HALL_FL_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(HALL_FR_GPIO_Port, HALL_FR_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(HALL_RL_GPIO_Port, HALL_RL_Pin, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI1_IRQn);
  /* EXTI2_IRQn removed: HALL_RL moved from PB2 to PB7 (EXTI9_5 below) */
  NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static const char *MotorTest_Name(MotorId wheel)
{
  switch (wheel)
  {
    case MOTOR_LF:
      return "LF";

    case MOTOR_RF:
      return "RF";

    case MOTOR_LR:
      return "LR";

    case MOTOR_RR:
      return "RR";

    case MOTOR_COUNT:
    default:
      return "INVALID";
  }
}

static void MotorTest_Print(const char *phase, MotorId wheel)
{
  char buffer[128];
  int length;

  length = snprintf(buffer, sizeof(buffer),
                    "TEST=%s,%s;CNT=%lu,%lu,%lu,%lu;HALL=%u,%u,%u,%u\r\n",
                    phase, MotorTest_Name(wheel),
                    (unsigned long)Encoder_GetCount(MOTOR_LF),
                    (unsigned long)Encoder_GetCount(MOTOR_RF),
                    (unsigned long)Encoder_GetCount(MOTOR_LR),
                    (unsigned long)Encoder_GetCount(MOTOR_RR),
                    (unsigned int)Hall_ReadLevel(MOTOR_LF),
                    (unsigned int)Hall_ReadLevel(MOTOR_RF),
                    (unsigned int)Hall_ReadLevel(MOTOR_LR),
                    (unsigned int)Hall_ReadLevel(MOTOR_RR));
  if (length > 0)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
  }
}

static void MotorTest_Run(void)
{
  MotorId wheel;

  HAL_UART_Transmit(&huart2, (uint8_t *)"MOTOR TEST BEGIN\r\n", 18U, HAL_MAX_DELAY);
  Motor_StopAll();
  HAL_Delay(MOTOR_TEST_GAP_MS);

  for (wheel = MOTOR_LF; wheel < MOTOR_COUNT; wheel++)
  {
    MotorTest_Print("FWD_START", wheel);
    Motor_SetDuty(wheel, MOTOR_TEST_DUTY);
    HAL_Delay(MOTOR_TEST_MS);
    Motor_StopAll();
    MotorTest_Print("FWD_STOP", wheel);
    HAL_Delay(MOTOR_TEST_GAP_MS);

    MotorTest_Print("REV_START", wheel);
    Motor_SetDuty(wheel, -MOTOR_TEST_DUTY);
    HAL_Delay(MOTOR_TEST_MS);
    Motor_StopAll();
    MotorTest_Print("REV_STOP", wheel);
    HAL_Delay(MOTOR_TEST_GAP_MS);
  }

  HAL_UART_Transmit(&huart2, (uint8_t *)"MOTOR TEST END\r\n", 16U, HAL_MAX_DELAY);
}

static uint8_t Hall_ReadLevel(MotorId wheel)
{
  switch (wheel)
  {
    case MOTOR_LF:
      return LL_GPIO_IsInputPinSet(HALL_FL_GPIO_Port, HALL_FL_Pin) ? 1U : 0U;

    case MOTOR_RF:
      return LL_GPIO_IsInputPinSet(HALL_FR_GPIO_Port, HALL_FR_Pin) ? 1U : 0U;

    case MOTOR_LR:
      return LL_GPIO_IsInputPinSet(HALL_RL_GPIO_Port, HALL_RL_Pin) ? 1U : 0U;

    case MOTOR_RR:
      return LL_GPIO_IsInputPinSet(HALL_RR_GPIO_Port, HALL_RR_Pin) ? 1U : 0U;

    case MOTOR_COUNT:
    default:
      return 0U;
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
