/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}

int mapChannel(int speed, int minLimit, int maxLimit){
	return MAP(speed, -20, 20, minLimit, maxLimit);
}

typedef struct{
	int16_t velocity;
	int64_t position;
	int64_t rpm;
	uint32_t last_counter_value;
}encoder_instance;

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim){
	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(&htim2);
	static uint8_t first_time = 0;
	if(!first_time){
		encoder_value ->velocity = 0;
		first_time = 1;
	}
	else{
		if(temp_counter == encoder_value ->last_counter_value){
			encoder_value ->velocity = 0;
		}
		else if(temp_counter > encoder_value ->last_counter_value){
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
				encoder_value ->velocity = -encoder_value ->last_counter_value - (__HAL_TIM_GET_AUTORELOAD(&htim2)-temp_counter);
			}
			else{
				encoder_value ->velocity = temp_counter - encoder_value ->last_counter_value;
			}
		}
		else{
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
				encoder_value ->velocity = temp_counter - encoder_value ->last_counter_value;
			}
			else{
				encoder_value ->velocity = temp_counter + (__HAL_TIM_GET_AUTORELOAD(&htim2) - encoder_value ->last_counter_value);
			}
		}
	}
	encoder_value ->position += encoder_value ->velocity;
	encoder_value ->last_counter_value = temp_counter;
}



uint32_t counter1 = 0;
int16_t count1 = 0;
uint32_t counter2 = 0;
int16_t count2 = 0;
uint32_t counter3 = 0;
int16_t count3 = 0;

int16_t encoder_velocity;
int64_t encoder_position;
int64_t encoder_rpm = 0;

encoder_instance enc_instance_mot1 = {0,0,0,0};
encoder_instance enc_instance_mot2 = {0,0,0,0};
encoder_instance enc_instance_mot3 = {0,0,0,0};


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	counter1 = __HAL_TIM_GET_COUNTER(&htim2);
	count1 = (int16_t) counter1;

	counter2 = __HAL_TIM_GET_COUNTER(&htim3);
	count2 = (int16_t) counter2;

	counter3 = __HAL_TIM_GET_COUNTER(&htim4);
	count3 = (int16_t) counter3;

	update_encoder(&enc_instance_mot1, &htim2);

	encoder_position = enc_instance_mot1.position;
	encoder_velocity = enc_instance_mot1.velocity;
}

void measureRPM(){
	encoder_rpm = 0;
	int16_t tempCount1 = count1;
	int16_t tempCount2 = count2;
	int16_t tempCount3 = count3;
	HAL_Delay(1000);



	int64_t tempDiff1 = abs(count1 - tempCount1) / 28;
	int64_t tempDiff2 = abs(count2 - tempCount2) / 28;
	int64_t tempDiff3 = abs(count3 - tempCount3) / 28;

	enc_instance_mot1.rpm = tempDiff1;
	encoder_rpm = enc_instance_mot1.rpm;

	enc_instance_mot2.rpm = tempDiff2;
	encoder_rpm = enc_instance_mot2.rpm;

	enc_instance_mot3.rpm = tempDiff3;
	encoder_rpm = enc_instance_mot3.rpm;
}

int M1_Speed = -8;
int M2_Speed = -7;
int M3_Speed = -8;

// Fast Forward = 20 for all motors ~ 28 RPM unloaded
// Fast Reverse = -20 for all motors ~ 28 RPM unloaded
// Neutral = 0 for all
// Medium Forward = 13, 12, 12 ~ 14RPM unloaded
// Slow Forward = 10, 7, 7 ~ 7 RPM unloaded
// Medium Reverse = -13, -12, -12 ~ 14RPM unloaded
// Slow Reverse = -8, -7, -8 ~ 7RPM unloaded


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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Setting up PWM Timer
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Setting up PWM Timer
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Setting up PWM Timer

	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL); // Setting up Encoders
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// Setting up USB message and sending through serial.
//	uint8_t buffer[] = "Hello World!!!!!\r\n";
//	CDC_Transmit_FS(buffer, sizeof(buffer));
//	HAL_Delay(1000);
//
	// Mapping Range of -10 to 10 to the duty cycle of the motors.

	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1300, 1684); // current, minimum, maximum, default
	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1320, 1681);
	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1323, 1712);
//	HAL_Delay(10000);

//	M1_Speed += 1;
//	M2_Speed += 1;
//	M3_Speed += 1;

//	if (M1_Speed > 10) {
//		M1_Speed = 0;
//		HAL_Delay(10000);
//		M1_Speed = -10;
//		M2_Speed = -10;
//		M3_Speed = -10;
//	}

	measureRPM();

//	HAL_TIM_PeriodElapsedCallback(&htim2);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Limit Switch Interrupt Functionality
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if((GPIO_Pin == GPIO_PIN_9)) {
		if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)){
			 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			  }
	}
	else{
		__NOP();
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
