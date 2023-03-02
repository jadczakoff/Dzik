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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// VARIABLES FROM RASPBERRY

#define MAX_FORWARD 224
#define MIN_FORWARD 255

#define MAX_BACKWARD 32
#define MIN_BACKWARD 0

#define MAX_RIGHT 157
#define MIN_RIGHT 0

#define MAX_LEFT 99
#define MIN_LEFT 255

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RxBuff;
uint16_t i = 0;
int8_t FinalBuffer[16];
uint8_t frameready = 0;
uint8_t framefirst = 0;

uint8_t ForwardDriver = 0;
uint8_t BackwardDriver = 0;
uint8_t RightDriver = 0;
uint8_t LeftDriver = 0;

uint8_t finalframeready = 0;

uint8_t Values[12];

uint8_t Flag_Direction = 0; // FLAG = 0 < Neutral | FLAG = 2 < Right | Flag = 1 < Left

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t computeProportionForwardDrive(int8_t in){
	return (((float)in*(0.5)) + 150.0);
}

uint8_t computeProportionBackwardDrive(int8_t in){
	return (((float)in*(0.5)) + 150.0);
}

uint8_t computeLeftServo(int8_t in){
	if(in == 0){
		return 150;
	}
	return (((float)in*(-0.7)) + 150);
}

uint8_t computeRightServo(int8_t in){

	if(in == 0){
		return 150;
	}

	return (((float)in*(-0.65)) + 150);
}

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

/*

USART1_TX | PB6
USART1_RX | PB7
Baudrate 115200

Dla napedu kol:
Ustawienie predkosci do przodu jest w zakresie decy: 32: START zaczyna sie od 255 (0xFF) do 224 (0xE0)
Ustawienie predkosci do tylu jest w zakresie decy: 32: START zaczyna sie od 1 (0x01) do 32 (0x20)

Do skretu serwa:

Ustawienie kata skretu w prawo jest w zakresie decy: 157; START zaczyna sie od MIN: 0x01 (1) do 0x9D (157)
Ustawienie kata skretu w lewo jest w zakresie decy: 157; START zaczyna sie od MIN: 0xFF (255) do 0x63 (99)
zakres: 157
PRAWO: MAX: 0x9D -> 0x01 min
lewo; max 0x63 -. 0xff min

Glowne taktowanie zegara 48MHz
Taktowanie glowne taktowanie timer'a 48MHz

Dla pololu
Prescaler 479
ARR: 1999

ARR: 100 MAX W LEWO
ARR: 200 MAX W PRAWO
ARR: 150 NEUTRAL

Serwo:
Prescaler 479
ARR: 1999

ARR: 215 MAX W LEWO
ARR: 80 MAX W PRAWO
ARR: 150 NEUTRAL (POZYCJA POCZATKOWA)

 ##################### FRAME #########################
        return std::vector<uint8_t>{0x9B, // Wartosc rozpoczecia ramki
                                    getWheelCommand(commands.frontLeftWheel), // Numer elementu w tablicy FinalBuffer - 0
                                    getSteerCommand(commands.frontLeftSteer), 1
                                    getWheelCommand(commands.frontRightWheel), 2
                                    getSteerCommand(commands.frontRightSteer), 3
                                    getWheelCommand(commands.midLeftWheel), 4
                                    getSteerCommand(commands.midLeftSteer), 5
                                    getWheelCommand(commands.midRightWheel), 6
                                    getSteerCommand(commands.midRightSteer), 7
                                    getWheelCommand(commands.rearLeftWheel), 8
                                    getSteerCommand(commands.rearLeftSteer), 9
                                    getWheelCommand(commands.rearRightWheel), 10
                                    getSteerCommand(commands.rearRightSteer), 11
                                    0x65}; // Wartosc konca ramki
    }



*/

  HAL_UART_Receive_IT(&huart1, &RxBuff, 1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // SERWO P1 | PA8
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // SERWO L1 | PA9
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // SERWO P2 | PA10
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // SERWO L2 | PA11

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // SERWO P3 | PB4
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // SERWO L3 | PA7
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // POLOLU | PB1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // POLOLU | PB0

 // Ustawienie serw oraz kol w poczatkowa pozycje
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 150);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 150);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 150);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 150);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 150);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 150);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 150);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 150);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(finalframeready == 1){

		  // ###### FRONT LEFT STEER ######
		  if(FinalBuffer[1] > 0 && FinalBuffer[1] <= 100){ // RIGHT
			  Values[0] = computeRightServo(FinalBuffer[1]);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Values[0]);
		  } else if(FinalBuffer[1] >= -100 && FinalBuffer[1] < 0){ // LEFT
			  Values[0] = computeLeftServo(FinalBuffer[1]);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Values[0]);
		  } else { // NEUTRAL
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 150);
		  }


		  // ###### FRONT RIGHT STEER ######
		  if(FinalBuffer[3] > 0 && FinalBuffer[3] <= 100){ // RIGHT
			  Values[1] = computeRightServo(FinalBuffer[3]);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Values[1]);
		  } else if(FinalBuffer[3] >= -100 && FinalBuffer[3] < 0){ // LEFT
			  Values[1] = computeLeftServo(FinalBuffer[3]);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Values[1]);
		  } else { // NEUTRAL
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 150);
		  }

		  // ###### MID LEFT STEER ######
		  if(FinalBuffer[5] > 0 && FinalBuffer[5] <= 100){ // RIGHT
			  Values[2] = computeRightServo(FinalBuffer[5]);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Values[2]);
		  } else if(FinalBuffer[5] >= -100 && FinalBuffer[5] < 0){ // LEFT
			  Values[2] = computeLeftServo(FinalBuffer[5]);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Values[2]);
		  } else { // NEUTRAL
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 150);
		  }

		  // ###### MID RIGHT STEER ######
		  if(FinalBuffer[7] > 0 && FinalBuffer[7] <= 100){ // RIGHT
			  Values[3] = computeRightServo(FinalBuffer[7]);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Values[3]);
		  } else if(FinalBuffer[7] >= -100 && FinalBuffer[7] < 0){ // LEFT
			  Values[3] = computeLeftServo(FinalBuffer[7]);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Values[3]);
		  } else { // NEUTRAL
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 150);
		  }

		  // ###### REAR LEFT STEER ######
		  if(FinalBuffer[9] > 0 && FinalBuffer[9] <= 100){ // RIGHT
			  Values[4] = computeRightServo(FinalBuffer[9]);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Values[4]);
		  } else if(FinalBuffer[9] >= -100 && FinalBuffer[9] < 0){ // LEFT
			  Values[4] = computeLeftServo(FinalBuffer[9]);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Values[4]);
		  } else { // NEUTRAL
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 150);
		  }

		  // ###### REAR RIGHT STEER ######
		  if(FinalBuffer[11] > 0 && FinalBuffer[11] <= 100){ // RIGHT
			  Values[5] = computeRightServo(FinalBuffer[11]);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Values[5]);
		  } else if(FinalBuffer[11] >= -100 && FinalBuffer[11] < 0){ // LEFT
			  Values[5] = computeLeftServo(FinalBuffer[11]);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Values[5]);
		  } else { // NEUTRAL
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 150);
		  }

		  if(FinalBuffer[0] > 0 && FinalBuffer[0] <= 100){
			  Values[6] = computeProportionForwardDrive(FinalBuffer[0]);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Values[6]);
		  } else if(FinalBuffer[0] >= -100 && FinalBuffer[0] < 0){
			  Values[7] = computeProportionBackwardDrive(FinalBuffer[0]);
			 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Values[7]);
		  } else if(FinalBuffer[0] == 0){
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 150);
		  }

		  if(FinalBuffer[2] > 0 && FinalBuffer[2] <= 100){
			  Values[8] = computeProportionForwardDrive(FinalBuffer[0]);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Values[8]);
		  } else if(FinalBuffer[2] >= -100 && FinalBuffer[2] < 0){
			  Values[9] = computeProportionBackwardDrive(FinalBuffer[0]);
			 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Values[9]);
		  } else if(FinalBuffer[2] == 0){
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 150);
		  }

		  finalframeready = 0;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//if(huart2.Instance == huart){

		HAL_UART_Receive_IT(&huart1, &RxBuff, 1);

		if(RxBuff == 0x9B){
			frameready = 1;
			framefirst = 1;
		} else {
			framefirst = 0;
		}

		if(RxBuff == 0x65){
			i = 0;
			frameready = 0;
			//memset(FinalBuffer, 0, 64);
			finalframeready = 1;
		}

		if(frameready == 1 && framefirst == 0){
			FinalBuffer[i] = RxBuff;
			i++;

		}

	//}
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
