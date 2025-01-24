/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#define N_POS_SAMPLES 40		// Number of position samples to store.  should put this somewhere else...
#define DT						0.0001 	// running at 10kHz
#define I_SCALE					0.50
#define MAX_DUTY 				500 	// set PWM frequency to 20kHz

typedef struct {
    int raw_encoder;
    float q;
    float q_single;
    int turns;
    int first_sample;
    float q_prev;
    float qd;
    float qd_vec[N_POS_SAMPLES];

    float tau;
    int encoder_status;
    float current_a;
    float current_b;
    float current_c;

    float current_a_offset;
    float current_b_offset;
    float current_c_offset;

    float motor_current;
    float commanded_voltage;
    float duty_a;
    float duty_b;
	uint16_t DUTY_A;
	uint16_t DUTY_B;

	float bus_voltage;
    float temperature;
    float thermistor_resistance;
    
    float Kp;
    float Kd;
    float q_des;
    float qd_des;
    float tau_des;

    float current_command;
    float torque_command;
    float error;
    float error_int;

    int timeout;    //CAN timeout

} Controller;

// void zero_current(Controller *controller);
void update_current_sense(Controller *controller);
// void update_encoder(Controller *controller);
// void control(Controller *controller);
// void init_controller(Controller *controller);
// void disable_controller(Controller *controller);

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ENC_CS_Pin GPIO_PIN_15
#define ENC_CS_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
