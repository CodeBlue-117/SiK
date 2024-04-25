/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.h
  * @author  MCD Application Team
  * @brief   Header of application of the SubGHz_Phy Middleware
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
#ifndef __SUBGHZ_PHY_APP_H__
#define __SUBGHZ_PHY_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "i2c.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t *RxData;

extern uint8_t payload[2];

extern uint8_t Ack1[1];
extern uint8_t Ack2[1];
extern bool isAck1Received;
extern bool isAck2Received;
extern uint8_t receivedAck;

extern uint16_t combinedBytes1;
extern uint16_t combinedBytes2;

extern bool isOtherDeviceIDReceived;
extern bool isRepaired;

struct dataBitField
{
	bool LRM_Select_Bit;
	bool Heartbeat_Bit;
	bool Plow_Angle_State_Bit;
	bool Plow_Box_Battery_SOC_Bit;

};

void ProcessData(void);
void encodeDeviceId(uint16_t deviceId, uint8_t* payload);
void broadcastDeviceId(uint8_t* pair_payload);
void sendAck(void);
bool pairingConfirmation(void);
void shutdown_Fnctn(void *argument);
void saveIDsToEEPROM(I2C_HandleTypeDef *hi2c1, uint16_t id1, uint16_t id2, uint16_t startAddress);
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* MODEM type: one shall be 1 the other shall be 0 */
#define USE_MODEM_LORA  0
#define USE_MODEM_FSK   1

#define RF_FREQUENCY                                910000000 /* Hz */

#ifndef TX_OUTPUT_POWER   /* please, to change this value, redefine it in USER CODE SECTION */
#define TX_OUTPUT_POWER                             14        /* dBm */
#endif /* TX_OUTPUT_POWER */

#if (( USE_MODEM_LORA == 1 ) && ( USE_MODEM_FSK == 0 ))
#define LORA_BANDWIDTH                              0         /* [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved] */
#define LORA_SPREADING_FACTOR                       7         /* [SF7..SF12] */
#define LORA_CODINGRATE                             1         /* [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] */
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif (( USE_MODEM_LORA == 0 ) && ( USE_MODEM_FSK == 1 ))

#define FSK_FDEV                                    10000     /* Hz */
#define FSK_DATARATE                                20000     /* bps */
#define FSK_BANDWIDTH                               40000     /* Hz */
#define FSK_PREAMBLE_LENGTH                         5         /* Same for Tx and Rx */
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
#error "Please define a modem in the compiler subghz_phy_app.h."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

#define PAYLOAD_LEN                                 64

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Init Subghz Application
  */
void SubghzApp_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*__SUBGHZ_PHY_APP_H__*/
