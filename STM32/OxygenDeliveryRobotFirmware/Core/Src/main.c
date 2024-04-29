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




int M1_Speed = 0;
int M2_Speed = 0;
int M3_Speed = 0;

void SPIConfig (void)
{
  RCC->APB2ENR |= (1<<12);  // Enable SPI1 CLock

  SPI1->CR1 |= (1<<0)|(1<<1);   // CPOL=1, CPHA=1

  SPI1->CR1 |= (1<<2);  // Master Mode

  SPI1->CR1 |= (3<<3);  // BR[2:0] = 011: fPCLK/16, PCLK2 = 80MHz, SPI clk = 5MHz

  SPI1->CR1 &= ~(1<<7);  // LSBFIRST = 0, MSB first

  SPI1->CR1 |= (1<<8) | (1<<9);  // SSM=1, SSi=1 -> Software Slave Management

  SPI1->CR1 &= ~(1<<10);  // RXONLY = 0, full-duplex

  SPI1->CR1 &= ~(1<<11);  // DFF=0, 8 bit data

  SPI1->CR2 = 0;
}

void SPI_Transmit (uint8_t *data, int size)
{

	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/

	int i=0;
	while (i<size)
	{
	   while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	   SPI1->DR = data[i];  // load the data into the Data Register
	   i++;
	}


/*During discontinuous communications, there is a 2 APB clock period delay between the
write operation to the SPI_DR register and BSY bit setting. As a consequence it is
mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
data.
*/
	while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication

	//  Clear the Overrun flag by reading DR and SR
	uint8_t temp = SPI1->DR;
					temp = SPI1->SR;

}

void SPI_Receive (uint8_t *data, int size)
{
	/************** STEPS TO FOLLOW *****************
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
	************************************************/

	while (size)
	{
		while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		SPI1->DR = 0;  // send dummy data
		while (!((SPI1->SR) &(1<<0))){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	  *data++ = (SPI1->DR);
		size--;
	}
}

void GPIOConfig (void)
{
	RCC->APB2ENR |=  (1<<2);  // Enable GPIOA clock

	GPIOA->CRL = 0;
	GPIOA->CRL |= (11U<<20);   // PA5 (SCK) AF output Push Pull
	GPIOA->CRL |= (11U<<28);   // PA7 (MOSI) AF output Push Pull
	GPIOA->CRL |= (1<<26);    // PA6 (MISO) Input mode (floating)
	GPIOA->CRL |= (3<<16);    // PA4 used for CS, GPIO Output

}

void SPI_Enable (void)
{
	SPI1->CR1 |= (1<<6);   // SPE=1, Peripheral enabled
}

void SPI_Disable (void)
{
	SPI1->CR1 &= ~(1<<6);   // SPE=0, Peripheral Disabled
}

void CS_Enable (void)
{
	GPIOA->BSRR |= (1<<9)<<16;
}

void CS_Disable (void)
{
	GPIOA->BSRR |= (1<<9);
}

uint8_t RxData[3]; // receive SPI data bytes
uint8_t RxDataTest[1]; // receive SPI data bytes

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


	SPI_Enable();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// Setting up USB message and sending through serial.
//	uint8_t buffer[] = "Hello World!\r\n";
//	CDC_Transmit_FS(buffer, sizeof(buffer));
//	HAL_Delay(1000);
//
	// Mapping Range of -10 to 10 to the duty cycle of the motors.

//	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1300, 1684); // current, minimum, maximum, default
//	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1320, 1681);
//	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1323, 1712);


//	uint8_t data[1];
//	uint8_t address = 0x67;
//	data[0] = address;
//
//	CS_Enable ();  // pull the pin low
//	SPI_Transmit (data, 1);  // send test
//	SPI_Receive (RxDataTest, 1);  // receive 1 byte of  test data
//	CS_Disable ();  // pull the pin high
//	HAL_Delay(500);

//	uint8_t buffer[] = "SPI Test!\r\n";
//	CDC_Transmit_FS(buffer, sizeof(buffer));
//	HAL_Delay(1000);

//	if (RxDataTest[0] == 0x67){
//		uint8_t buffer[] = "Received: 0x67\r\n";
//		CDC_Transmit_FS(buffer, sizeof(buffer));
//		HAL_Delay(500);
//	}
//	else {
//		uint8_t buffer[] = "Error\r\n";
//		CDC_Transmit_FS(buffer, sizeof(buffer));
//		HAL_Delay(500);
//	}


////	 Working SPI Transmit
//	CS_Enable ();  // pull the cs pin low
//	SPI_Transmit (data, 1);  // write data to register
//	CS_Disable ();  // pull the cs pin high
//	HAL_Delay(500);

//	uint8_t activateCode[1];
//	activateCode[0] = address;
//
//	CS_Enable ();  // pull the pin low
//	SPI_Transmit (activateCode, 1);  // send address (just need to send something random
//	SPI_Receive (RxData, 3);  // receive 6 bytes data
//	CS_Disable ();  // pull the pin high
//	HAL_Delay(1000);

	  //proper
	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1100, 1884); // current, minimum, maximum, default
	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1100, 1884);
	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1100, 1884);

	HAL_Delay(3000);


	// Go Forward
	M1_Speed = 0;
	M2_Speed = -10;
	M3_Speed = 10;

	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1100, 1884); // current, minimum, maximum, default
	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1100, 1884);
	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1100, 1884);

	HAL_Delay(3000);


	// Pause
	M1_Speed = 0;
	M2_Speed = 0;
	M3_Speed = 0;

	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1100, 1884); // current, minimum, maximum, default
	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1100, 1884);
	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1100, 1884);

	HAL_Delay(3000);

	// Go Backwards
	M1_Speed = 0;
	M2_Speed = 10;
	M3_Speed = -10;

	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1100, 1884); // current, minimum, maximum, default
	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1100, 1884);
	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1100, 1884);

	HAL_Delay(3000);

	// Pause
	M1_Speed = 0;
	M2_Speed = 0;
	M3_Speed = 0;

	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1100, 1884); // current, minimum, maximum, default
	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1100, 1884);
	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1100, 1884);

	HAL_Delay(3000);

	// Go Left
	M1_Speed = 10;
	M2_Speed = -5;
	M3_Speed = -5;

	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1100, 1884); // current, minimum, maximum, default
	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1100, 1884);
	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1100, 1884);

	HAL_Delay(3000);

	// Pause
	M1_Speed = 0;
	M2_Speed = 0;
	M3_Speed = 0;

	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1100, 1884); // current, minimum, maximum, default
	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1100, 1884);
	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1100, 1884);

	HAL_Delay(3000);

	// Go Right
	M1_Speed = -10;
	M2_Speed = 5;
	M3_Speed = 5;

	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1100, 1884); // current, minimum, maximum, default
	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1100, 1884);
	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1100, 1884);

	HAL_Delay(3000);

	// Pause
	M1_Speed = 0;
	M2_Speed = 0;
	M3_Speed = 0;

	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1100, 1884); // current, minimum, maximum, default
	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1100, 1884);
	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1100, 1884);

	HAL_Delay(3000);

	//end proper

//	M1_Speed = -10;
//	M2_Speed = -10;
//	M3_Speed = 10;
//
//	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1100, 1884);
//	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1100, 1884);
//
//	HAL_Delay(4000);
//
//	M1_Speed = 0;
//	M2_Speed = 0;
//	M3_Speed = 0;
//
//	htim1.Instance->CCR1 = mapChannel(M1_Speed, 1100, 1884); // current, minimum, maximum, default
//	htim1.Instance->CCR2 = mapChannel(M2_Speed, 1100, 1884);
//	htim1.Instance->CCR3 = mapChannel(M3_Speed, 1100, 1884);
//
//	HAL_Delay(5000);
//
//	measureRPM();



//	M1_Speed += 1;
//	M2_Speed += 1;
//	M3_Speed += 1;
//
//	if (M1_Speed > 20) {
//		M1_Speed = 0;
//		M2_Speed = 0;
//		M3_Speed = 0;
//		htim1.Instance->CCR1 = mapChannel(M1_Speed, 1300, 1684); // current, minimum, maximum, default
//		htim1.Instance->CCR2 = mapChannel(M2_Speed, 1300, 1684);
//		htim1.Instance->CCR3 = mapChannel(M3_Speed, 1300, 1684);
//		HAL_Delay(10000);
//		M1_Speed = -20;
//		M2_Speed = -20;
//		M3_Speed = -20;
//	}


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
