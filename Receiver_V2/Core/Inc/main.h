/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32wlxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "platform.h"
#include "lsm6dso_reg.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// FATFS MicroSD Card Variables
extern bool isPairingConfirmed;
extern bool sd_initialized;
extern bool record_enabled;
extern bool recording;

// EEPROM Variables to save pairing IDS
extern uint16_t * EEPROM_PairingID1;
extern uint16_t * EEPROM_PairingID2;
extern uint16_t EEPROM_Start_Address;
extern uint16_t id1;
extern uint16_t id2;

// Pairing Algo Variables
extern uint32_t lastButtonPressTime; // used to track the last timestamp of a button press
extern uint8_t pair_payload[2];

typedef struct
{

	uint16_t deviceID;
	uint16_t otherDeviceID;
	uint16_t combinedDeviceID[2];
	uint16_t swappedCombinedDeviceID[2];

} DeviceIDS;

extern DeviceIDS deviceIds;
extern uint16_t deviceId;
extern uint16_t otherDeviceID;
extern uint16_t combinedDeviceID[2];
extern uint16_t swappedCombinedDeviceID[2];

typedef enum
{

	Unknown,
	GenerateDeviceID,
	EncodeDeviceID,
	BroadcastDeviceID,
	SendAck,
	PairingConfirmation,
	Paired

} PairingStates_t;

extern PairingStates_t currentPairingState;

extern stmdev_ctx_t dev_ctx;

extern uint8_t AccelReg;

extern int16_t data_raw_acceleration[3];

extern int16_t acceleration_mg[3];

extern float x;
extern float y;
extern float z;

extern float sendIMUData;

extern bool sendIMUData_FLAG;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define DEBOUNCE_DELAY 50
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

float READ_IMU_DATA(void);
void MX_I2C2_Init(void);
void broadcastDeviceId(uint8_t* pair_payload);
void sendAck(void);
void shutdown_Fnctn(void *argument);
void radio_send_wrapper(uint8_t payloadSize);
int32_t my_write_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t my_read_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
bool pairingConfirmation(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define Tx_Rx_Ctl_Pin GPIO_PIN_8
#define Tx_Rx_Ctl_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_9
#define SD_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

#define BUTTON_PIN				GPIO_PIN_15
#define BUTTON_PORT				GPIOA 				// Also used for blue led as well as pairing button
#define DEBOUNCE_DELAY_MS		200
#define RED_PIN					GPIO_PIN_4
#define GREEN_PIN				GPIO_PIN_13
#define BLUE_PIN				GPIO_PIN_6
#define PAIR_LIGHT_PORT_1		GPIOB
#define PAIR_LIGHT_PORT_2		GPIOC
#define SD_SPI_HANDLE hspi2
#define SD_CS_GPIO_Port			GPIOA
#define SD_CS_Pin				GPIO_PIN_9

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
