/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
typedef enum{
	SHUT = 0,
	OPEN
}Flag;
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
#define BUTTON_TRIG_Pin GPIO_PIN_7
#define BUTTON_TRIG_GPIO_Port GPIOI
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define ADC_BAT_Pin GPIO_PIN_10
#define ADC_BAT_GPIO_Port GPIOF
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/******* 通用部分 开始 ***********/

#define ROBOT_ID 4

/******* 通用部分 结束 ***********/


/*******************************/
////云台有关
//#define PITCH_MOTOR_ID 0x205
//#define YAW_MOTOR_ID 0x207
//#define FR1_MOTOR_ID 0x201	//friction motor 1 ID
//#define FR2_MOTOR_ID 0x202	//friction motor 2 ID
//#define SHOOT_MOTOR_ID 0x203	//shoot motor ID
////#define CHASSIS_TO_GIMBAL_ID 0x303  //chassis ID

#define ECD_MIN 0
#define ECD_MAX 8191


////#define CHANNEL_SCALE_FACTOR 2000.0f
//#define CHANNEL_SCALE_FACTOR 3000.0f
//#define MOUSE_SCALE_FACTOR 500.0f
//#define KEY_SCALE_FACTOR 3.5f
//#define ROTATE_KEY_SCALE_FACTOR (KEY_SCALE_FACTOR / 2)
/**********************************/

//typedef enum
//{
//	PITCH,
//	YAW,
//	ROLL
//}Axis_e;

//typedef struct
//{
//	int16_t ecd_fdb;
//    int16_t speed_rpm_fdb;
//	int16_t current_fdb;
//	int8_t temperate_fdb;
//}Motor_Feedback_t;

//typedef struct
//{
//	int16_t speed_rpm_set;
//	int16_t current_set;
//}Motor_Set_t;

typedef union{
	int16_t data_raw;
	uint8_t data[2];
}Half_Word_Transform_u;

typedef union{
	float data_raw;
	uint8_t data[4];
}Float_Transform_u;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
