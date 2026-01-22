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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "VL53L0X.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern I2C_HandleTypeDef VL53L0X_I2C_Handler;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/*=======================================RAZER=======================================*/
statInfo_t_VL53L0X distanceL_stats;
statInfo_t_VL53L0X distanceR_stats;
uint16_t distanceL = 0; // volatile 추가
uint16_t distanceR = 0; // volatile 추가


#define FILTER_BUFFER_SIZE 10
volatile uint16_t distanceL_buffer[FILTER_BUFFER_SIZE] = {0};
volatile uint16_t distanceR_buffer[FILTER_BUFFER_SIZE] = {0};
int buffer_index_L = 0;
int buffer_index_R = 0;
/*=======================================RAZER=======================================*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	if (ch == '\n')
		HAL_UART_Transmit(&huart2, (uint8_t*) "\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}


/*=============================DT=============================*/
extern volatile uint32_t msTicks;   // SysTick 1ms 카운터
#define DT_S          0.039f      // 39 ms 고정 주기
/*=============================DT=============================*/

/*=========================Encoder value=========================*/
volatile int32_t left_encoder_value = 0;    //left
volatile int32_t right_encoder_value  = 0;  //right

volatile int32_t prev_count_L = 0;
volatile int32_t prev_count_R = 0;
/*=========================Encoder value=========================*/

/*============================motor============================*/
#define WHEEL_RADIUS    0.0625f    // 바퀴 반지름 (단위: m)
#define WHEEL_BASE      0.33f    // 바퀴 간 거리 (단위: m)
#define ENCODER_RES     3172       // 엔코더 분해능 (4체배 적용된 값)
volatile int32_t tow = 0;
volatile int32_t tow_applied = 0;
/*============================motor============================*/

/*=============================PID=============================*/
float Kp = 4, Kd = 1.6, Ki = 8;
float P_term = 0, D_term = 0, I_term = 0;
float pid = 0;

float Kp_a = 5, Kd_a = 1.3, Ki_a = 6.25;
float P_term_a = 0, D_term_a = 0, I_term_a = 0;
float pid_a = 0;

float error = 0;
float previous_error = 0;
float old_error = 0;

float error_a = 0;
float previous_error_a = 0;
float old_error_a = 0;

float speed = 0;
float speed_r = 0;

double angle = 0.0019808276504349264;   // will fix (rad/count)

/*=============================MID=============================*/
volatile int32_t delta_cnt_L = 0;
volatile int32_t delta_cnt_R = 0;
float   dtheta_L    = 0.0f;    // [rad]
float   dtheta_R    = 0.0f;    // [rad]
float   dist_L      = 0.0f;    // [m]
float   dist_R      = 0.0f;    // [m]
float   s_step      = 0.0f;    // [m/period]
float   theta_step  = 0.0f;    // [rad/period]
float   v_meas      = 0.0f;    // [m/s]
float   w_meas      = 0.0f;    // [rad/s]
/*=============================MID=============================*/

/*=============================PID=============================*/


/*=============================USART=============================*/
char rx_buffer[256];
char rx_data;
int rx_index = 0;

int stop_flag = 0;
int sent_flag = 0;  // 전송 여부 상태 저장

// 1. ROS에서 받은 선속도/각속도 값 (단위: m/s, rad/s)
float cmd_vel_x = 0.0;  // v
float cmd_vel_z = 0.0;  // w

/*=============================USART=============================*/

/*=============================DIR=============================*/
void check_move_state();
void check_move_state_r();
void motorControl(int in1, int in2);
void motorControl_r(int in1, int in2);

int control_1 = 0, control_2 = 1;
int control_1_r = 1, control_2_r = 0;

int move_state = 0;  // front:0 back:1
int move_state_r = 0;
/*=============================DIR=============================*/

/*=============================PWM=============================*/
void Pwm_Left(int pwm_input);
void Pwm_Right(int pwm_input);
void check_pwm();
void check_pwm_r();
/*=============================PWM=============================*/


/*=============================RK2=============================*/
double odom_x = 0.0;     // [m]
double odom_y = 0.0;     // [m]
double odom_th = 0.0;    // [rad]

double previous_odom_x = 0.0;
double previous_odom_y = 0.0;
double previous_odom_th = 0.0;

double value = 0.0;
double dtt = 0.0;

/*=============================RK2=============================*/

/*=============================RAZER=============================*/
void apply_sensor_config(void);
uint16_t get_filtered_distance(volatile uint16_t* buffer, int* index, uint16_t new_distance);
/*=============================RAZER=============================*/
/*=============================SWITCH=============================*/
void limited_switch();
int switch_flag = 0;
int switch_count = 0;
/*=============================SWITCH=============================*/

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
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /*----------------------------------------------------*/
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);


  /*----------------------------------------------------*/

  /*----------------------------------------------------*/
  motorControl(control_1,  control_2);
  motorControl_r(control_1_r, control_2_r);

  /*----------------------------------------------------*/
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_data, 1);
  HAL_TIM_Base_Start_IT(&htim2);
  /*----------------------------------------------------*/
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);
  /*----------------------------------------------------*/

	//  // --- 센서 초기화 ---
	HAL_GPIO_WritePin(RAZER_POWER1_GPIO_Port, RAZER_POWER1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RAZER_POWER2_GPIO_Port, RAZER_POWER2_Pin, GPIO_PIN_SET);

	VL53L0X_I2C_Handler = hi2c1;
	if (initVL53L0X(1, &hi2c1)) {
		apply_sensor_config(); // 안정적인 330ms 설정 적용
	}

	VL53L0X_I2C_Handler = hi2c2;
	if (initVL53L0X(1, &hi2c2)) {
		apply_sensor_config();
	}

	// --- 첫 측정 시작 ---
	VL53L0X_I2C_Handler = hi2c1;
	startSingleMeasurement_nonBlocking();
	VL53L0X_I2C_Handler = hi2c2;
	startSingleMeasurement_nonBlocking();
  /*----------------------------------------------------*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {



		limited_switch();

		//방향 제어
		motorControl(control_1, control_2);
		motorControl_r(control_1_r, control_2_r);
		//PWM 제어
		Pwm_Left(fabsf(speed));
		Pwm_Right(fabsf(speed_r));

		/******** PID only + v,w print (전역 변수만 사용) ********/
		if (msTicks >= 39) {
			dtt = msTicks;

			/*************************** TOW ******************************/
			if (tow == 1 && tow_applied == 0)
			{
				//KP,KI,KD 값 변경
			    Kp   = Kp   * 1.25f;   Ki   = Ki   * 1.20f;   Kd   = Kd   * 0.90f;
			    Kp_a = Kp_a * 1.30f;   Ki_a = Ki_a * 1.20f;   Kd_a = Kd_a * 0.90f;

			    previous_error = old_error = 0.0f;
			    previous_error_a = old_error_a = 0.0f;

			    tow_applied = 1;
			}
			/*************************** TOW ******************************/

			/*************************** RAZER ******************************/
			// --- 센서 상태 확인 및 데이터 수신 (논블로킹) ---
			VL53L0X_I2C_Handler = hi2c1;
			if (isMeasurementReady()) {
				uint16_t raw_distanceL = readMeasurementResult(
						&distanceL_stats);
				// 필터링 및 범위 제한 적용
				if (distanceL_stats.rangeStatus == 0
						|| distanceL_stats.rangeStatus == 11) {
					distanceL = get_filtered_distance(distanceL_buffer,
							&buffer_index_L, raw_distanceL);
				}
				startSingleMeasurement_nonBlocking(); // 다음 측정 시작
			}

			VL53L0X_I2C_Handler = hi2c2;
			if (isMeasurementReady()) {
				uint16_t raw_distanceR = readMeasurementResult(
						&distanceR_stats);
				if (distanceR_stats.rangeStatus == 0
						|| distanceR_stats.rangeStatus == 11) {
					distanceR = get_filtered_distance(distanceR_buffer,
							&buffer_index_R, raw_distanceR);
				}
				startSingleMeasurement_nonBlocking(); // 다음 측정 시작
			}
			/*************************** RAZER ******************************/
			// 1) 이번 주기 엔코더 증가량
			delta_cnt_L = left_encoder_value - prev_count_L;
			delta_cnt_R = right_encoder_value - prev_count_R;
			prev_count_L = left_encoder_value;
			prev_count_R = right_encoder_value;

			// 2) 바퀴 각도[rad] → 선형 이동[m]
			dtheta_L = (float) delta_cnt_L * (float) angle;  // angle: rad/count
			dtheta_R = (float) delta_cnt_R * (float) angle;
			dist_L = dtheta_L * WHEEL_RADIUS;
			dist_R = dtheta_R * WHEEL_RADIUS;

			// 3) 이번 주기 전진/회전 변위
			s_step = 0.5f * (dist_R + dist_L);            // [m/period]
			theta_step = (dist_R - dist_L) / WHEEL_BASE;      // [rad/period]

			/******************** RK2 ***********************/
			value = previous_odom_th + (theta_step * 0.5f);
			odom_x = previous_odom_x + (s_step * cosf(value));
			odom_y = previous_odom_y + (s_step * sinf(value));
			odom_th = previous_odom_th + theta_step;

			previous_odom_x = odom_x;
			previous_odom_y = odom_y;
			previous_odom_th = odom_th;

			// 4) 측정 속도 환산
			v_meas = s_step / (dtt * 0.001f);                       // [m/s]
			w_meas = theta_step / (dtt * 0.001f);                     // [rad/s]

			/******************** PID ***********************/
			// 목표속도 - 측정속도
			error = (cmd_vel_x - v_meas);
			error_a = (cmd_vel_z - w_meas);

			// 증분형(Delta) 형태
			P_term = (error - previous_error) * Kp;
			D_term = (error - 2.0f * previous_error + old_error) * Kd;
			I_term = error * Ki;
			pid = P_term + D_term + I_term;

			P_term_a = (error_a - previous_error_a) * Kp_a;
			D_term_a = (error_a - 2.0f * previous_error_a + old_error_a) * Kd_a;
			I_term_a = error_a * Ki_a;
			pid_a = P_term_a + D_term_a + I_term_a;

			if (stop_flag == 0) {
				// 차동 결합: 선형(pid) ± 회전(pid_a)
				speed = speed + pid - pid_a;
				speed_r = speed_r + pid + pid_a;
			} else {
				speed = speed - (speed) / 5.0f;
				speed_r = speed_r - (speed_r) / 5.0f;
			}

			// 상태 갱신
			old_error = previous_error;
			old_error_a = previous_error_a;
			previous_error = error;
			previous_error_a = error_a;

			// PWM/상태 체크
			check_pwm();
			check_pwm_r();
			check_move_state();
			check_move_state_r();

			if (cmd_vel_x == 0 && cmd_vel_z == 0) {
				stop_flag = 1;
			} else {
				stop_flag = 0;
			}

			printf("# %f %f %f %f %f %f | %f %f %d\n", odom_x, odom_y, odom_th,
					dtt, v_meas, w_meas, (float)distanceL, (float)distanceR, switch_flag);


		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 41;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 255;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Actuator1_HOOK_Pin|Actuator2_HOOK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, left_IN2_Pin|Right_IN1_Pin|Right_IN2_Pin|Actuator1_Lift_Pin
                          |Actuator2_Lift_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Left_IN1_Pin|RAZER_POWER1_Pin|RAZER_POWER2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Actuator1_HOOK_Pin Actuator2_HOOK_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Actuator1_HOOK_Pin|Actuator2_HOOK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : left_IN2_Pin Right_IN1_Pin Right_IN2_Pin Actuator1_Lift_Pin
                           Actuator2_Lift_Pin */
  GPIO_InitStruct.Pin = left_IN2_Pin|Right_IN1_Pin|Right_IN2_Pin|Actuator1_Lift_Pin
                          |Actuator2_Lift_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_IN1_Pin RAZER_POWER1_Pin RAZER_POWER2_Pin */
  GPIO_InitStruct.Pin = Left_IN1_Pin|RAZER_POWER1_Pin|RAZER_POWER2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_pulse_B_Pin Left_pulse_A_Pin */
  GPIO_InitStruct.Pin = Left_pulse_B_Pin|Left_pulse_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HOOK_Input_Pin */
  GPIO_InitStruct.Pin = HOOK_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HOOK_Input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Right_pulse_B_Pin Right_pulse_A_Pin */
  GPIO_InitStruct.Pin = Right_pulse_B_Pin|Right_pulse_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*=========================Encoder value=========================*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if (GPIO_Pin == GPIO_PIN_15) // Left A
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
        {
            left_encoder_value--;
        }
        else
        {
            left_encoder_value++;
        }
    }
    else if (GPIO_Pin == GPIO_PIN_14) // Left B
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
        {
            left_encoder_value++;
        }
        else
        {
            left_encoder_value--;
        }
    }

    else if (GPIO_Pin == GPIO_PIN_12) // Right A
    {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11))
        {
            right_encoder_value++;
        }
        else
        {
            right_encoder_value--;
        }
    }
    else if (GPIO_Pin == GPIO_PIN_11) // Right B
    {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11))
        {
            right_encoder_value--;
        }
        else
        {
            right_encoder_value++;
        }
    }


    UNUSED(GPIO_Pin); // Prevent unused warning
}
/*=========================Encoder value=========================*/


/*=============================USART=============================*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		if (rx_data != '\n') {
			rx_buffer[rx_index++] = rx_data;
		}
		else {
			rx_buffer[rx_index] = '\0';

			if (rx_buffer[0] == '1') {
				// 선속도 (cmd_vel_x) 추출
				char first_number[10];
				strncpy(first_number, &rx_buffer[2], 9);
				first_number[9] = '\0';
				cmd_vel_x = atof(first_number);
				if (rx_buffer[1] == '0') {
					cmd_vel_x = -cmd_vel_x;
				}

				// 각속도 (cmd_vel_z) 추출
				char second_number[10];
				strncpy(second_number, &rx_buffer[12], 9);
				second_number[9] = '\0';
				cmd_vel_z = atof(second_number);
				if (rx_buffer[11] == '0') {
					cmd_vel_z = -cmd_vel_z;
				}
			}
			else if (rx_buffer[0] == '3')
			{
                // 00
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			else if (rx_buffer[0] == '4')
			{
				//10
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			else if (rx_buffer[0] == '5')
			{

				//01
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			}
			else if (rx_buffer[0] == '6')
			{
				NVIC_SystemReset();
			}
			// 수신 버퍼 초기화
			rx_index = 0;
			memset(rx_buffer, 0, sizeof(rx_buffer));
		}
	}

	HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_data, 1);
}
/*=============================USART=============================*/

/*=============================TIM=============================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		if (msTicks > 39) {
			msTicks = 0;
		}

	}
	if (htim->Instance == TIM3) {

		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 0) {
			if (switch_flag == 0) {
				switch_count++;
			}

			if (switch_count >= 50000) {

			    tow = 1;
				switch_count = 0;
				switch_flag = 1;
			}

		} else {
			switch_count = 0;
			switch_flag = 0;
		}

	}
}
/*=============================TIM=============================*/

/*=============================DIR=============================*/
void check_move_state() {
  if (speed >= 0) {
    move_state = 0;
    control_1 = 0; control_2 = 1;  // Forward
  } else {
    move_state = 1;
    control_1 = 1; control_2 = 0;  // Reverse
  }
}

void check_move_state_r() {
  if (speed_r >= 0) {
    move_state_r = 0;
    control_1_r = 1; control_2_r = 0;  // Forward
  } else {
    move_state_r = 1;
    control_1_r = 0; control_2_r = 1;  // Reverse
  }
}


void motorControl(int in1, int in2) {
	GPIO_PinState state1=GPIO_PIN_RESET,state2=GPIO_PIN_RESET;
	if(in1==1){
		state1=GPIO_PIN_SET;
	}
	else{
		state1=GPIO_PIN_RESET;
	}
	if(in2==1){
		state2=GPIO_PIN_SET;
	}
	else{
		state2=GPIO_PIN_RESET;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, state1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, state2);

}

void motorControl_r(int in1, int in2) {
	GPIO_PinState state1=GPIO_PIN_RESET,state2=GPIO_PIN_RESET;
	if(in1==1){
		state1=GPIO_PIN_SET;
	}
	else{
		state1=GPIO_PIN_RESET;
	}
	if(in2==1){
		state2=GPIO_PIN_SET;
	}
	else{
		state2=GPIO_PIN_RESET;
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, state1);
 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, state2);
}
/*=============================DIR=============================*/


/*=============================PWM=============================*/
void check_pwm() {
	if (speed >= 180) {
		speed = 180;
	} else if (speed <= -180) {
		speed = -180;
	}
}

void check_pwm_r() {
	if (speed_r >= 180) {
		speed_r = 180;
	} else if (speed_r <= -180) {
		speed_r = -180;
	}
}

// 왼쪽 모터 PWM 제어 (pwm_value: 0~PWM_MAX)
void Pwm_Left(int pwm_input)
{

    htim5.Instance->CCR1 = pwm_input;
}

// 오른쪽 모터 PWM 제어 (pwm_value: 0~PWM_MAX)
void Pwm_Right(int pwm_input)
{

    htim5.Instance->CCR2 = pwm_input;
}
/*=============================PWM=============================*/

/*=======================================RAZER=======================================*/
void apply_sensor_config(void)
{
    setSignalRateLimit(200);
    setVcselPulsePeriod(VcselPeriodPreRange, 14);
    setVcselPulsePeriod(VcselPeriodFinalRange, 10);
    setMeasurementTimingBudget(330000UL); // 330ms
}

// 이동 평균 필터 및 범위 제한을 적용하는 함수
uint16_t get_filtered_distance(volatile uint16_t* buffer, int* index, uint16_t new_distance)
{
    // 1. 새 값을 버퍼에 저장 (오래된 값 덮어쓰기)
    buffer[*index] = new_distance;
    *index = (*index + 1) % FILTER_BUFFER_SIZE;

    // 2. 버퍼의 평균 계산
    long sum = 0;
    int valid_samples = 0;
    for (int i = 0; i < FILTER_BUFFER_SIZE; i++) {
        // 유효한 초기값이 들어왔을 때만 평균에 포함
        if (buffer[i] > 0 && buffer[i] < 8000) {
            sum += buffer[i];
            valid_samples++;
        }
    }

    uint16_t averaged_distance = 0;
    if (valid_samples > 0) {
        averaged_distance = sum / valid_samples;
    } else {
        return 8191; // 버퍼에 유효한 값이 하나도 없으면 에러 값 반환
    }

    // 3. 거리 제한 (0 ~ 1000mm)
    if (averaged_distance > 1000) {
        averaged_distance = 1000;
    }

    return averaged_distance;
}
/*=======================================RAZER=======================================*/

/*=======================================SWITCH=======================================*/
void limited_switch()
{
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 0)
	{
		HAL_TIM_Base_Start_IT(&htim3);
	}
}
/*=======================================SWITCH=======================================*/
/*----------------------------------------------------*/
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
