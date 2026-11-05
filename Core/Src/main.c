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

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "bme280.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define QUEUE_SIZE                     64
#define COMMAND_MAX_LENGTH             32
#define ADC_MAX_COUNT                  4095
#define ADC_REFERENCE_MILLIVOLTS       3300
#define ADC_TIMEOUT_MS                 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile uint8_t button_pressed =   0;
uint32_t last_press_ms =            0;
uint8_t incoming_byte;
volatile uint8_t received_bytes[QUEUE_SIZE];
volatile uint8_t next_read_index =  0;
volatile uint8_t next_write_index = 0;
char command_text[COMMAND_MAX_LENGTH];
uint8_t command_length =            0;
uint32_t press_count;
char press_message[32];
bool bme280_ready = false;
volatile uint8_t report_timer_elapsed = 0;
bool auto_report_enabled = false;
uint8_t led_brightness_percent = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t adc_count_to_millivolts(uint16_t raw_count)
{
    return ((uint32_t)raw_count * ADC_REFERENCE_MILLIVOLTS) / ADC_MAX_COUNT;
}

static bool read_a0_raw_count(uint16_t *a0_raw_count)
{
    if(HAL_ADC_Start(&hadc1) != HAL_OK){ return false; }
    if(HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) != HAL_OK)
    {
    	HAL_ADC_Stop(&hadc1);
    	return false;
    }

    *a0_raw_count = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return true;
}

static void set_led_brightness(uint8_t brightness_percent)
{
	if(brightness_percent > 100)
	{
		brightness_percent = 100;
	}
	led_brightness_percent = brightness_percent;

	uint16_t timer_ticks = (brightness_percent * 10);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, timer_ticks);
}

static int32_t celsius_to_fahrenheit_hundredths(int32_t celsius_hundredths)
{
	return ((celsius_hundredths * 9) / 5) + 3200;
}

static void send_message(const char *text)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

static void send_reading(const char *label, int32_t value_hundredths, const char *unit)
{
	const char *sign = "";

	if (value_hundredths < 0)
	{
		sign = "-";
		value_hundredths = -value_hundredths;
	}

	char reading_message[48];

	snprintf(reading_message, sizeof(reading_message), "%s %s%ld.%02ld %s\r\n",
			label, sign, (long)value_hundredths / 100, (long)value_hundredths % 100, unit);

	send_message(reading_message);
}

static bool queue_add_byte(uint8_t byte)
{
	uint8_t index_after_write = (next_write_index + 1) & (QUEUE_SIZE -1);
	if (index_after_write == next_read_index)
	{
		return false;
	}
	received_bytes[next_write_index] = byte;
	next_write_index = index_after_write;
	return true;
}

static bool queue_is_empty(void)
{
	return next_write_index == next_read_index;
}

static uint8_t queue_take_byte(void)
{
	uint8_t byte_taken = received_bytes[next_read_index];
	next_read_index = (next_read_index + 1) & (QUEUE_SIZE - 1);
	return byte_taken;
}

static bool get_store_readings(bme_measurements *reading)
{
	if(!bme280_ready || !bme280_read(reading))
	{
		send_message("Sensor not responding\r\n");
		return false;
	}
	return true;
}

static void send_all_readings(void)
{
	bme_measurements readings;
	if(!get_store_readings(&readings)){ return; }
	send_reading("Temperature", readings.temperature_hundredths, "C");
	int32_t F = celsius_to_fahrenheit_hundredths(readings.temperature_hundredths);

	send_reading("Temperature", F, "F");
	send_reading("Humidity", (int32_t)readings.humidity_hundredths, "%");
	send_reading("Pressure", (int32_t)readings.pressure_pascals, "hPa");
}

static void run_command(void)
{
	if(command_length == 0)
	{
		return;
	}

	else if(strcmp(command_text, "led_on") == 0)
	{
		set_led_brightness(100);
		send_message("OK\r\n");
	}
	else if(strcmp(command_text, "led_off") == 0)
	{
		set_led_brightness(0);
		send_message("OK\r\n");
	}
	else if(strcmp(command_text, "status") == 0)
	{
		char status_message[48];
		snprintf(status_message, sizeof(status_message), "LED brightness %d%% Presses %lu\r\n", (int)led_brightness_percent, (unsigned long)press_count);
		send_message(status_message);
	}
	else if(strcmp(command_text, "sensor_id") == 0)
	{
		uint8_t sensor_id_value = 0;

		if (!bme280_read_id(&sensor_id_value))
		{
			send_message("Sensor not responding\r\n");
		}
		else
		{
			char id_message[32];
			snprintf(id_message, sizeof(id_message), "BME280 ID 0x%02X\r\n", sensor_id_value);
			send_message(id_message);
		}
	}
	else if (strcmp(command_text, "temp") == 0)
	{
		bme_measurements values;
		if(get_store_readings(&values))
		{
			send_reading("Temperature", values.temperature_hundredths, "C");
			send_reading("Temperature", celsius_to_fahrenheit_hundredths(values.temperature_hundredths), "F");
		}
	}
	else if (strcmp(command_text, "humidity") == 0)
	{
		bme_measurements values;
		if(get_store_readings(&values))
		{
			send_reading("Humidity", (int32_t)values.humidity_hundredths, "%");
		}
	}
	else if (strcmp(command_text, "pressure") == 0)
	{
		bme_measurements values;
		if(get_store_readings(&values))
		{
			send_reading("Pressure", (int32_t)values.pressure_pascals, "hPa");
		}
	}
	else if (strcmp(command_text, "read") == 0)
	{
		send_all_readings();
	}
	else if (strcmp(command_text, "auto_on") == 0)
	{
		auto_report_enabled = true;
		send_message("OK\r\n");
	}
	else if (strcmp(command_text, "auto_off") == 0)
	{
		auto_report_enabled = false;
		send_message("OK\r\n");
	}
	else if (strcmp(command_text, "a0_voltage") == 0)
	{
		uint16_t a0_raw_count;
		char voltage_message[48];
		if(!read_a0_raw_count(&a0_raw_count))
		{
			send_message("ADC not responding\r\n");
		}
		else
		{
			uint32_t a0_millivolts = adc_count_to_millivolts(a0_raw_count);
			snprintf(voltage_message, sizeof(voltage_message), "A0 raw %u = %lu mV\r\n", (unsigned int)a0_raw_count, (unsigned long)a0_millivolts);
			send_message(voltage_message);
		}
	}
	else
	{
		send_message("Unknown command\r\n");
	}
}

static uint8_t new_brightness(uint8_t current)
{
	uint8_t new = current +25;
	if (new > 100)
	{
		return 0;
	}
	else
	{
		return new;
	}
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  bme280_ready = bme280_init(&hi2c1);

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  if(!bme280_ready)
  {
	  send_message("bme 280 is not ready\r\n");
  }

  HAL_UART_Receive_IT(&huart2, &incoming_byte, 1);
  send_message("\r\nReady. Commands: led_on, led_off, status, sensor_id, "
		  "temp, humidity, pressure, read, auto_on, auto_off, a0_voltage\r\n> ");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(report_timer_elapsed == 1)
	{
		report_timer_elapsed = 0;
		if(auto_report_enabled == true)
		{
			send_all_readings();
		}
	}

	if(button_pressed)
	{
		press_count++;
		button_pressed = 0;
		int len = snprintf(press_message, sizeof(press_message), "Pressed %lu\r\n", (unsigned long)press_count);
		HAL_UART_Transmit(&huart2, (uint8_t *)press_message, len, HAL_MAX_DELAY);
		set_led_brightness(new_brightness(led_brightness_percent));
	}
	// while loop that transmits until queue is empty
	while(!queue_is_empty())
	{
		uint8_t current_byte = queue_take_byte();

		// enter
		if(current_byte == '\r')
		{
			send_message("\r\n");
			command_text[command_length] = '\0';
			run_command();
			command_length = 0;
			send_message("> ");
		}
		// backspace
		else if(current_byte == 0x7F || current_byte == '\b')
		{
			if(command_length > 0)
			{
				send_message("\b \b");
				command_length--;
			}

		}
		// normal character
		else if(command_length < (COMMAND_MAX_LENGTH - 1))
		{
			command_text[command_length] = current_byte;
			command_length++;
			HAL_UART_Transmit(&huart2, &current_byte, 1, HAL_MAX_DELAY);
		}

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
		report_timer_elapsed = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == B1_Pin)
	{
		uint32_t now = HAL_GetTick();
		if(now - last_press_ms >= 50)
		{
			last_press_ms = now;
			button_pressed = 1;
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		queue_add_byte(incoming_byte);
		HAL_UART_Receive_IT(&huart2, &incoming_byte, 1);
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
