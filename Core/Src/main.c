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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "printing.h"
#include "math_ops.h"
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
#define M_ZERO					0
void update_encoder(Controller *controller) {
	// read AS5048 encoder:
	// HAL_StatusTypeDef spi_status;
	// uint16_t enc_pos;
	// uint8_t result1, result2;  // Changed to uint8_t to match SPI transfer size
	// uint8_t dummy = 0xFF;      // Dummy byte for transmit

	// // Lower the CS pin to start the transmission
	// HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET);

	// // Transmit and receive the first byte
	// spi_status = HAL_SPI_TransmitReceive(&hspi2, &dummy, &result1, 1, HAL_MAX_DELAY);
	// while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY);

	// // Mask the unnecessary bits and shift left to prepare for concatenation
	// result1 &= 0b00111111;

	// // Transmit and receive the second byte
	// spi_status = HAL_SPI_TransmitReceive(&hspi2, &dummy, &result2, 1, HAL_MAX_DELAY);
	// while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY);

	// // Concatenate the two results
	// enc_pos = (result1 << 8) | result2;
	// enc_pos &= 0x3FFF;
	// // Raise the CS pin to end the transmission
	// HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);
// Read AS5048 encoder:
    HAL_StatusTypeDef spi_status;
    uint16_t enc_pos;
    uint8_t tx_data[2] = {0xFF, 0xFF}; // Dummy bytes for transmit
    uint8_t rx_data[2] = {0};         // Buffer for received data

    // Lower the CS pin to start the transmission
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET);

    // Transmit and receive 16 bits (2 bytes) in one transaction
    spi_status = HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, 2, HAL_MAX_DELAY);
    while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY);

    // Raise the CS pin to end the transmission
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);

    // Combine the two received bytes into a 16-bit value
    enc_pos = (rx_data[0] << 8) | rx_data[1];

    // Check for errors (optional)
    if (enc_pos & 0x4000) { // Check error flag (bit 14)
        // Handle error (e.g., magnet misalignment)
        // You can log the error or take corrective action
    }

    // Extract the 14-bit angle (bits [13:0])
    enc_pos &= 0x3FFF; // Mask out bits [15:14]

    // Now `enc_pos` contains the 14-bit angular position (0 to 16383)

    controller->raw_encoder = enc_pos;
    controller->encoder_status = 0;
    controller->q_prev = controller->q_single;
    controller->q_single = (enc_pos - M_ZERO)/16384.0 * 2.0 * PI_F;
    controller->q_single = controller->q_single<-PI_F ? controller->q_single + TWO_PI_F : controller->q_single;
    controller->q_single = controller->q_single>PI_F  ? controller->q_single - TWO_PI_F : controller->q_single;

	/* Rollover */
	int rollover = 0;
	float angle_diff = controller->q_single - controller->q_prev;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	if(!controller->first_sample){
		controller->turns = 0;
		controller->first_sample = 1;
	} else {
		controller->turns += rollover;
	}

	/* Multi-turn position */
    controller->q = controller->q_single + TWO_PI_F*(float)controller->turns;


    float qd_raw = (controller->q_single - controller->q_prev)/DT;

	float sum = qd_raw;
	for (int i = 1; i < N_POS_SAMPLES; i++){
		controller->qd_vec[N_POS_SAMPLES - i] = controller->qd_vec[N_POS_SAMPLES-i-1];
		sum += controller->qd_vec[N_POS_SAMPLES-i];
		}
	controller->qd_vec[0] = qd_raw;
	controller->qd =  sum/((float)N_POS_SAMPLES);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  Controller controller;
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    update_encoder(&controller);
    printf(">Encoder: %d\r\n", controller.raw_encoder);
    HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
