
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
/* vim: set ai et ts=4 sw=4: */
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void UART_Printf(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff),
                      HAL_MAX_DELAY);
    va_end(args);
}

void keyboard_prepare() {
    HAL_GPIO_WritePin(Col1_GPIO_Port, Col1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Col2_GPIO_Port, Col2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Col3_GPIO_Port, Col3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Col4_GPIO_Port, Col4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Col5_GPIO_Port, Col5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Col6_GPIO_Port, Col6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Col7_GPIO_Port, Col7_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Col8_GPIO_Port, Col8_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Col9_GPIO_Port, Col9_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Col10_GPIO_Port, Col10_Pin, GPIO_PIN_RESET);
}

void change_column(uint8_t column) {
    switch(column) {
    case 0:
        HAL_GPIO_WritePin(Col10_GPIO_Port, Col10_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Col1_GPIO_Port, Col1_Pin, GPIO_PIN_SET);
        break;
    case 1:
        HAL_GPIO_WritePin(Col1_GPIO_Port, Col1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Col2_GPIO_Port, Col2_Pin, GPIO_PIN_SET);
        break;
    case 2:
        HAL_GPIO_WritePin(Col2_GPIO_Port, Col2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Col3_GPIO_Port, Col3_Pin, GPIO_PIN_SET);
        break;
    case 3:
        HAL_GPIO_WritePin(Col3_GPIO_Port, Col3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Col4_GPIO_Port, Col4_Pin, GPIO_PIN_SET);
        break;
    case 4:
        HAL_GPIO_WritePin(Col4_GPIO_Port, Col4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Col5_GPIO_Port, Col5_Pin, GPIO_PIN_SET);
        break;
    case 5:
        HAL_GPIO_WritePin(Col5_GPIO_Port, Col5_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Col6_GPIO_Port, Col6_Pin, GPIO_PIN_SET);
        break;
    case 6:
        HAL_GPIO_WritePin(Col6_GPIO_Port, Col6_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Col7_GPIO_Port, Col7_Pin, GPIO_PIN_SET);
        break;
    case 7:
        HAL_GPIO_WritePin(Col7_GPIO_Port, Col7_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Col8_GPIO_Port, Col8_Pin, GPIO_PIN_SET);
        break;
    case 8:
        HAL_GPIO_WritePin(Col8_GPIO_Port, Col8_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Col9_GPIO_Port, Col9_Pin, GPIO_PIN_SET);
        break;
    default: // 9
        HAL_GPIO_WritePin(Col9_GPIO_Port, Col9_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Col10_GPIO_Port, Col10_Pin, GPIO_PIN_SET);
    }
}

bool keyboard_read(uint8_t* out_row, uint8_t* out_col) {
    keyboard_prepare();
    for(uint8_t col = 0; col < 10; col++) {
        change_column(col);
        if(HAL_GPIO_ReadPin(Row1_GPIO_Port, Row1_Pin) == GPIO_PIN_SET) {
            *out_col = col;
            *out_row = 0;
            return true;
        } else if(HAL_GPIO_ReadPin(Row2_GPIO_Port, Row2_Pin) == GPIO_PIN_SET) {
            *out_col = col;
            *out_row = 1;
            return true;
        } else if(HAL_GPIO_ReadPin(Row3_GPIO_Port, Row3_Pin) == GPIO_PIN_SET) {
            *out_col = col;
            *out_row = 2;
            return true;
        } else if(HAL_GPIO_ReadPin(Row4_GPIO_Port, Row4_Pin) == GPIO_PIN_SET) {
            *out_col = col;
            *out_row = 3;
            return true;
        }
   }
   return false;
}

void init() {
    UART_Printf("Ready!\r\n");
    HAL_Delay(1);
}

static uint32_t total_clicks = 0;

void loop() {
    uint8_t row, col;
    if(keyboard_read(&row, &col)) {
        // discard impossible reads which sometimes
        // can happen because of noise RF signals
        // or because of accidental touch of button contacts
        if((row <= 2) || ((row == 3) && (col <= 4))) {
            total_clicks++;
            UART_Printf("row = %d, col = %d\r\n", row, col);
            UART_Printf("clks = %ld\r\n", total_clicks);
        }
    }

    HAL_Delay(10);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  init();
  while (1)
  {
    loop();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA5   ------> SPI1_SCK
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Col10_Pin|Col1_Pin|Col2_Pin|GPIO_PIN_6 
                          |GPIO_PIN_7|Col5_Pin|Col6_Pin|Col7_Pin 
                          |Col8_Pin|Col9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Col3_Pin|Col4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Col10_Pin Col1_Pin Col2_Pin PC6 
                           PC7 Col5_Pin Col6_Pin Col7_Pin 
                           Col8_Pin Col9_Pin */
  GPIO_InitStruct.Pin = Col10_Pin|Col1_Pin|Col2_Pin|GPIO_PIN_6 
                          |GPIO_PIN_7|Col5_Pin|Col6_Pin|Col7_Pin 
                          |Col8_Pin|Col9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Row1_Pin Row2_Pin Row3_Pin Row4_Pin */
  GPIO_InitStruct.Pin = Row1_Pin|Row2_Pin|Row3_Pin|Row4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Col3_Pin Col4_Pin */
  GPIO_InitStruct.Pin = Col3_Pin|Col4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
