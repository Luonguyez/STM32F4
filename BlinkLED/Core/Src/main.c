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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile uint32_t led1_counter = 0;
volatile uint32_t led2_counter = 0;
volatile uint32_t button_pressed;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile __UINT32_TYPE__ last_button_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

void LED3_Update(void);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
{
  LED3_Update();
}

  /* USER CODE BEGIN 3 */
  
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  This function is executed toggle led 1 cycles 100 ms.
  * @retval None
  */
void LED1_Update(void)
{
  led1_counter++;
  if (led1_counter >= 100)  // 500ms
  {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    led1_counter = 0;
  }
}

/**
  * @brief  This function is executed toggle led 2 cycles 2000 ms.
  * @retval None
  */
void LED2_Update(void)
{
  led2_counter++;
  if (led2_counter >= 2000)  // 1000ms
  {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    led2_counter = 0;
  }
}
/**
  * @brief  This function is executed toggle led 3 when button is pressed.
  * @retval None
  */
void LED3_Update(void)
{
    // static uint32_t last_sample_tick = 0;
    // static uint8_t sample_count = 0;
    // static uint8_t button_state = 0;
    // static uint8_t last_button_state = 0;

    // if (HAL_GetTick() - last_sample_tick >= 10) // mỗi 10ms kiểm tra 1 lần
    // {
    //     last_sample_tick = HAL_GetTick();

    //     if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
    //     {
    //         if (sample_count < 3)
    //             sample_count++;
    //     }
    //     else
    //     {
    //         sample_count = 0;
    //     }

    //     if (sample_count >= 3)
    //         button_state = 1;
    //     else
    //         button_state = 0;

    //     // Phát hiện cạnh lên (nhấn mới)
    //     if (button_state && !last_button_state)
    //     {
    //         HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);  // toggle LED
    //     }
    //     last_button_state = button_state;
    // }

  // static __UINT32_TYPE__ last_tick = 0;
  // static __UINT8_TYPE__  but_cnt   = 0;
  // static __UINT8_TYPE__  last_state = 0;
  // if ((HAL_GetTick() - last_tick) >= 10)
  // {
  //   last_tick = HAL_GetTick();
  //   if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) &&   
  //   {
  //     if (but_cnt < 3)
  //     {
  //       but_cnt ++;
  //     }
  //     else if (but_cnt >= 3)
  //     {
  //       HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
  //     }
  //     else 
  //     {
  //       but_cnt = 0;
  //     }
  //   }
  // }
	static uint8_t last_button_state = 0;
	static uint32_t last_tick = 0;
  static uint8_t count = 0;

	if ((HAL_GetTick() - last_tick) >= 10)  
  {
    last_tick = HAL_GetTick();

    uint8_t current_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    if (current_state == GPIO_PIN_RESET && last_button_state == GPIO_PIN_SET)
    {
      if (count >=3 )
      {
        //check pulse down to up
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
      }
      else if (count < 3)
      {
        count ++;
      }
      else 
      {
        count = 0;
      }
    }

    last_button_state = current_state;
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

/**
  * @brief  This function is executed toggled if button is used, func use EXTI.
  * @retval None
  */


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
