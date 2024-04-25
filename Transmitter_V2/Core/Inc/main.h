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
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern uint32_t cabChargeLevel;

extern bool isRepaired;	// debounce if this device successfully pairs, but other device fails

extern uint32_t lastButtonPressTime; // used to track the last timestamp of a button press

extern bool isPairingButtonPressed;

extern uint8_t payload[2]; // This loads the Device IDs into Buffer Tx

extern uint16_t * EEPROM_PairingID1;
extern uint16_t * EEPROM_PairingID2;
extern uint16_t EEPROM_Start_Address;

extern bool isPairingConfirmed;
//bool isPairingButtonPressed = false;

extern bool pairingFlag;

extern bool sd_initialized;
extern bool record_enabled;
extern bool recording;

extern uint16_t id1;
extern uint16_t id2;

typedef struct
{
	uint16_t deviceID;
	uint16_t otherDeviceID;
	uint16_t combinedDeviceID[2];
	uint16_t swappedCombinedDeviceID[2];
} DeviceIDS;

extern DeviceIDS deviceIds;

extern uint16_t deviceId; // This is the local device ID
extern uint16_t otherDeviceID; // This is the remote device ID
extern uint16_t combinedDeviceID[2]; // This is the local combination (unswapped)
extern uint16_t swappedCombinedDeviceID[2]; // This is the local combination that we swap before sending so that it matches the remote device's otherCombinedDeviceID

extern I2C_HandleTypeDef hi2c1;

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

extern bool CenterCheck;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define BUTTON_PIN					GPIO_PIN_15  // This is the pairing button
#define BUTTON_PORT					GPIOA
#define DEBOUNCE_DELAY_MS			200  // adjust the debounce delay as needed -- troubleshooting APP_LOG function only

#define RED_PIN						GPIO_PIN_0
#define GREEN_PIN					GPIO_PIN_8
#define BLUE_PIN					GPIO_PIN_1
#define PAIR_LIGHT_PORT				GPIOA
#define DEBOUNCE_DELAY 100

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

int32_t MX_I2C2_Init(void);
int32_t my_write_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t my_read_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

void encodeDeviceId(uint16_t deviceId, uint8_t *payload);
void broadcastDeviceId(uint8_t* payload);
void sendAck(void);
bool pairingConfirmation(void);
void pairingFunction(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
