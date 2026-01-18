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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define LCD_ADDR (0x27 << 1)
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
int last_state = -1; // -1: ????, 0: NORMAL, 1: BRAKE
uint32_t adc_value = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

float Get_Distance(void) {
    uint32_t start_time = 0, end_time = 0;
    
    // 1. Trig ??? 10us ?? ?? ??
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    
    // 2. Echo ?? High? ? ??? ?? (?? ?? ??)
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    
    // 3. Echo ?? Low? ? ??? ?? (?? ?? ??)
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET);
    end_time = __HAL_TIM_GET_COUNTER(&htim2);
    
    // 4. ?? ?? (?? 340m/s ??, ????? / 2)
    return (float)end_time * 0.034 / 2;
}

void lcd_write_i2c(uint8_t data) {
    // Backlight ??(0x08)? ?? ???? ??
    uint8_t i2c_data = data | 0x08; 
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &i2c_data, 1, 10);
}

// ?????? ?? ??? I2C ???? ???
void pulse_en_i2c(uint8_t data) {
    lcd_write_i2c(data | 0x04); // EN High (P2 ??)
    delay_us(20);
    lcd_write_i2c(data & ~0x04); // EN Low
    delay_us(20);
}

// ?? lcd_send_4bit? I2C ???? ??
void lcd_send_4bit(char data, uint8_t rs) {
    // ?? 4??? ???? ?? RS ??? ??
    uint8_t res = (data << 4) | rs;
    pulse_en_i2c(res);
}

void lcd_send_8bit(char data, uint8_t rs) {
    lcd_send_4bit((data >> 4) & 0x0F, rs); // ?? 4??
    lcd_send_4bit(data & 0x0F, rs);        // ?? 4??
}

// ?? ??? ?? (RS ? ?? ?? ????? ??)
void lcd_set_cur(int row, int col) {
    uint8_t addr = (row == 0) ? (0x80 | col) : (0xC0 | col);
    lcd_send_8bit(addr, 0); // RS=0 (?? ??)
}

void lcd_clear() {
    lcd_send_8bit(0x01, 0); // RS=0
    HAL_Delay(2);
}

void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_8bit(*str++, 1); // RS=1 (?? ?? ??)
    }
}

void lcd_init(void) {
    HAL_Delay(50);
    // 4bit ?? ?? ?? ??? (RS=0 ??)
    lcd_send_4bit(0x03, 0); HAL_Delay(5);
    lcd_send_4bit(0x03, 0); HAL_Delay(1);
    lcd_send_4bit(0x03, 0); HAL_Delay(1);
    lcd_send_4bit(0x02, 0); HAL_Delay(1); // 4?? ?? ??

    // ????? ?? ??
    lcd_send_8bit(0x28, 0); // 4??, 2?, 5x8
    lcd_send_8bit(0x08, 0); // Display OFF
    lcd_send_8bit(0x01, 0); // Clear
    HAL_Delay(2);
    lcd_send_8bit(0x06, 0); // Entry mode
    lcd_send_8bit(0x0C, 0); // Display ON, Cursor OFF
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC_Init();
  MX_TIM6_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // TIM2? ?? 1 PWM ??
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim6);
	HAL_ADC_Start(&hadc);
	HAL_Delay(500);
	lcd_init();
	HAL_Delay(100);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 244); //Initial value
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 66);
  while (1)
  {
		adc_value = HAL_ADC_GetValue(&hadc);
		float distance = Get_Distance();
		
		uint8_t arduino_signal = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    
    if (distance <= 25.0 || arduino_signal == GPIO_PIN_SET) { // 10cm
			
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 250);
			HAL_Delay(32); 
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 72);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			
			if (last_state != 1) {
            lcd_clear();
            lcd_set_cur(0, 0);
            lcd_send_string("EMERGENCY BRAKE");
            last_state = 1; 
        } 
    }
		
    else {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 244); 
			HAL_Delay(32); 
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 66);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			
			if (last_state != 0) {
            lcd_clear();
            lcd_set_cur(0, 0);
            lcd_send_string("Status: NORMAL");
            last_state = 0;
        }
			
			
			
    }
    
    HAL_Delay(100); // 0.1??? ?? ??
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
