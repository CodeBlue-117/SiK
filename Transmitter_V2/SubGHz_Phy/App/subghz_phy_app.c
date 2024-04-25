/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
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

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "app_version.h"
#include "subghz_phy_version.h"
#include <stdint.h>
#include "main.h"
#include <stdio.h>
#include "radio_driver.h"
#include "i2c.h"
#include "usart.h"
#include "usart_if.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

// Pairing Variables
uint8_t receivedAck 	   		= 0; // flag to check if we have received an acknowledgment during pairing process
uint8_t Ack1[1] 	       		= {0x0F}; // acknowledgment 1 arbitrary value
uint8_t Ack2[1] 	       		= {0xF0}; // acknowledgment 2 arbitrary value
bool isAck1Received        		= false;
bool isAck2Received        		= false;
bool edgeTrigger2          		= false; // false means GPIOB2 is not being pressed
bool edgeTrigger12         		= false; // false means GPIOB12 is not being pressed

bool isOtherDeviceIDReceived;

// Plow Angle State Variables
int filteredAngle 				= 0;
uint8_t receivedData[8]			= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint16_t combinedBytes1			= 0;
uint16_t combinedBytes2			= 0;
int angle 						= 0;

// #define DEBOUNCE_DELAY 50

// FSK config
#define USE_MODEM_LORA  		  0
#define USE_MODEM_FSK   		  1
#define RF_FREQUENCY        	  910000000 /* Hz */
#define EEPROM_START_ADDR		  0x00

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
} States_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Configurations */
/*Timeout*/
#define RX_TIMEOUT_VALUE            1000
#define TX_TIMEOUT_VALUE            1000
/* PING string*/

#define PING 						"NI"	// NI = Not recording, pIng
#define PINGR 						"RI"    // RI = Recording, pIng
#define RIGHT 	 					0x0F	// NR = Not recording, Right
#define RIGHTR 						"RR"	// RR = Recording, Right
#define LEFT 						"NL"	// NL = Not recording, Left
#define LEFTR 						"RL"	// RL = Recording, Left

/* PONG string*/
#define PONG 						"NO"	// NO = Not recording, pOng
#define PONGR 						"RO"	// RO = Not recording, pOng
/*Size of the payload to be sent*/
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE          255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */
/* wait for remote to be in Rx, before sending a Tx frame*/
#define RX_TIME_MARGIN                0
/* Afc bandwidth in Hz */
#define FSK_AFC_BANDWIDTH             83333
/* LED blink Period*/
#define LED_PERIOD_MS                 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */

/*Ping Pong FSM states */
static States_t State = RX;
/* App Rx Buffer*/
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
/* App Tx Buffer*/
uint8_t BufferTx[MAX_APP_BUFFER_SIZE] 			= {0};
/* Last  Received Buffer Size*/
uint16_t RxBufferSize 							= 0;
/* Last  Received packer Rssi*/
int8_t RssiValue 								= 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue 								= 0;
/* Led Timers objects*/
//static UTIL_TIMER_Object_t timerLed;
/* device state. Master: true, Slave: false*/
bool isMaster 									= true;
bool waitForTransmit 							= false;
bool record_enabled 							= false;
bool recording 									= false;
/* random delay to make sure 2 devices will sync*/
/* the closest the random delays are, the longer it will
   take for the devices to sync when started simultaneously*/
static int32_t random_delay;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */

bool PingPongWrapperReceive(const uint8_t *RxData, size_t RxDataLength, const uint16_t *prefix, size_t prefixLength, const char *substring);

void StormEq_WCS(void);

void StormEq_Pairing(void);

void saveIDsToEEPROM(I2C_HandleTypeDef *hi2c3, uint16_t id1, uint16_t id2, uint16_t startAddress);

bool readIDsFromEEPROM(I2C_HandleTypeDef *hi2c3, uint16_t *id1, uint16_t *id2, uint16_t startAddress);

void clearEEPROMData(I2C_HandleTypeDef *hi2c3, uint16_t clearValue, uint16_t startAddress);

void pairing_Function(void);

void broadcastDeviceId(uint8_t* payload);

void heartBeat(void);

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

	printf( "\n\rPING PONG\n\r");
	/* Get SubGHY_Phy APP version*/
	printf( "APPLICATION_VERSION: V%X.%X.%X\r\n",
		  (uint8_t)(APP_VERSION_MAIN),
		  (uint8_t)(APP_VERSION_SUB1),
		  (uint8_t)(APP_VERSION_SUB2));

	/* Get MW SubGhz_Phy info */
	printf( "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
		  (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
		  (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
		  (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */
  /*calculate random delay for synchronization*/
     random_delay = (Radio.Random()) >> 22; /*10bits random e.g. from 0 to 1023 ms*/
     /* Radio Set frequency */
     Radio.SetChannel(RF_FREQUENCY);

     /* Radio configuration */
     #if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
      printf( "---------------\n\r");
      printf( "LORA_MODULATION\n\r");
      printf( "LORA_BW=%d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
      printf( "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);

      Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                        LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                        true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

      Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                        LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                        LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                        0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

      Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

    #elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
      printf( "---------------\n\r");
      printf( "FSK_MODULATION\n\r");
      printf( "FSK_BW=%d Hz\n\r", FSK_BANDWIDTH);
      printf( "FSK_DR=%d bits/s\n\r", FSK_DATARATE);

      Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                        FSK_DATARATE, 0,
                        FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                        true, 0, 0, 0, TX_TIMEOUT_VALUE);

      Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                        0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                        0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                        0, 0, false, true);

      Radio.SetMaxPayloadLength(MODEM_FSK, MAX_APP_BUFFER_SIZE);

    #else
    #error "Please define a modulation in the subghz_phy_app.h file."
    #endif /* USE_MODEM_LORA | USE_MODEM_FSK */

      /*fills tx buffer*/
      memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

      APP_LOG(TS_ON, VLEVEL_L, "rand=%d\n\r", random_delay);
      /*starts reception*/
      Radio.Rx(RX_TIMEOUT_VALUE + random_delay);

      /*register task to to be run in while(1) after Radio IT*/
      UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, StormEq_WCS);

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

// wrapper to pull PB8 high before all transmissions and pull back low after tranmissions
void radio_send_wrapper(size_t payloadSize)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	Radio.Send(BufferTx, payloadSize);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
}

// Function to encode the deviceId into the payload buffer as binary (Hex) number
// Convert the 32-bit deviceId to a 4-byte representation
// Note: This implementation assumes little-endian byte order
// payload does not need to be returned as it is updated globally
// If deviceId is larger than 32-bits, we may need to modify the encoding accordingly

void encodeDeviceId(uint16_t deviceId, uint8_t *payload)
{
  printf("Entered encodeDeviceId function.\n\r");

  HAL_GPIO_WritePin(PAIR_LIGHT_PORT, RED_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PAIR_LIGHT_PORT, GREEN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PAIR_LIGHT_PORT, BLUE_PIN, GPIO_PIN_SET);

  payload[0] = (uint8_t)(deviceId & 0xFF);
  payload[1] = (uint8_t)((deviceId >> 8) & 0xFF);

  deviceIds.deviceID = ((uint16_t)(payload[0] | payload[1] << 8));

  printf("This Device ID is (struct): 0x%04X\n\r", deviceIds.deviceID);
}

void broadcastDeviceId(uint8_t* payload)
{
	printf("Entered BroadcastDeviceID function.\n\r");

	HAL_GPIO_WritePin(PAIR_LIGHT_PORT, RED_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PAIR_LIGHT_PORT, GREEN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PAIR_LIGHT_PORT, BLUE_PIN, GPIO_PIN_SET);

	while(!isAck1Received)
	{

		// Broadcast local Device ID
		printf("Cab PCB transmitting this DeviceID, then listening for Ack\n\r");
		memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);  // Clear the BufferTx array
		size_t payloadSize = strlen((char*)payload);
		memcpy(BufferTx, payload, payloadSize);  // Copy the payload to BufferTx
		radio_send_wrapper(payloadSize);
		HAL_Delay(100);

		HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);

		printf("Listen for Ack1\n\r");
		memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
		// start reception
		Radio.Rx(RX_TIMEOUT_VALUE + random_delay);
		HAL_Delay(100);

		// check to see if a message is received
		if(RxBufferSize >= sizeof(uint8_t))
		{
			// Process the received message
			memcpy(&receivedAck, BufferRx, sizeof(uint8_t));
			// Check to see if the received message is 0x0F (Ack1)
			if(receivedAck == 0x0F)
			{
				printf("\n\r\n\rAck1 message received!!\n\r");
				isAck1Received = true;
				break;
			}
			else
			{
				//printf("Ack1 message NOT received. Continuing with Broadcast Function.\n\r");
			}
		}
	}

    printf("CAB PCB listening for otherDeviceID.\n\r");
    // flags to check if the otherDeviceID was received
	bool isOtherDeviceIDReceived = false;

	while(!isOtherDeviceIDReceived)
	{
		// Clear BufferRx
		memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
		// start reception
		Radio.Rx(RX_TIMEOUT_VALUE + random_delay);
		HAL_Delay(100);

		// check to see if message is received
		if(RxBufferSize >= sizeof(uint16_t))
		{
			// Process the received message (Save remote DeviceId to memory location &otherDeviceID)
			memcpy(&otherDeviceID, BufferRx, sizeof(uint16_t));
			printf("Received Payload is: 0x%04X\n\r", otherDeviceID);

			if((otherDeviceID != 0x0F) && (otherDeviceID != 0xF0) && (otherDeviceID != 0x0000))
			{
				deviceIds.otherDeviceID              = otherDeviceID; // assign the other deviceId to the struct (should be the sent encoded value)
				deviceIds.combinedDeviceID[0]        = deviceIds.deviceID;
				deviceIds.combinedDeviceID[1]        = deviceIds.otherDeviceID;
				deviceIds.swappedCombinedDeviceID[0] = deviceIds.combinedDeviceID[1];
				deviceIds.swappedCombinedDeviceID[1] = deviceIds.combinedDeviceID[0];

				swappedCombinedDeviceID[0] 			 = deviceIds.swappedCombinedDeviceID[0];
				swappedCombinedDeviceID[1] 			 = deviceIds.swappedCombinedDeviceID[1];

				printf("combinedDeviceID[0] = 0x%04X\n\r", deviceIds.combinedDeviceID[0]);
				printf("combinedDeviceID[1] = 0x%04X\n\r", deviceIds.combinedDeviceID[1]);
				printf("swappedCombinedDeviceID[0] = 0x%04X\n\r", deviceIds.swappedCombinedDeviceID[0]);
				printf("swappedCombinedDeviceID[1] = 0x%04X\n\r", deviceIds.swappedCombinedDeviceID[1]);

				uint16_t pairingId1 = deviceIds.deviceID;
				uint16_t pairingId2 = deviceIds.otherDeviceID;

				saveIDsToEEPROM(&hi2c3, pairingId1, pairingId2, EEPROM_Start_Address);

				printf("\n\r\n\rOther Device ID is (Struct): 0x%04X\n\r\n\r", deviceIds.otherDeviceID);
				isOtherDeviceIDReceived = true;

				HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);

				for(int i = 0; i < 5; i++)
				{
					printf("Sending Ack1 x5\n\r");
					memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);  // Clear the BufferTx array
					size_t payloadSize = sizeof(Ack1);
					memcpy(BufferTx, Ack1, payloadSize);  // Copy the payload to BufferTx
					radio_send_wrapper(payloadSize);
					HAL_Delay(100);
				}

				currentPairingState = Paired;
				break;
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_StatusTypeDef UART_Check = HAL_OK;

	uint8_t dataToSend = 0x00;

	// BSP_LED_Toggle(LED1);
	if (GPIO_Pin == GPIO_PIN_2) // Activate Right Wing
	{
		if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET) && (!edgeTrigger12)) // GPIOB2 is pushed down and GPIOB12 is not pushed down
		{
			if(CenterCheck == true)
			{
				// Send UART message to BMD-350 to alert iPad that manual button 1 has been pressed
				dataToSend = 0x0F;
				UART_Check = HAL_UART_Transmit(&huart1, &dataToSend, sizeof(dataToSend), HAL_MAX_DELAY);

				if(UART_Check == HAL_OK)
				{
					printf("UART message sent successfully\n\r");
				}

				else
				{
					printf("UART message failed, error %d\n\r", UART_Check);
				}

				edgeTrigger2 = true;

				uint8_t data = 15; // 0x0F for right wing
				memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);
				size_t payloadSize = sizeof(swappedCombinedDeviceID) + sizeof(uint8_t);
				uint8_t combinedBuffer[sizeof(swappedCombinedDeviceID) + sizeof(uint8_t)];
				memcpy(combinedBuffer, swappedCombinedDeviceID, strlen((char*)(swappedCombinedDeviceID)));
				memcpy(combinedBuffer + sizeof(swappedCombinedDeviceID), &data, sizeof(data));
				memcpy(BufferTx, combinedBuffer, payloadSize);

				// for (size_t i = 0; i < 20; ++i)
				// {
				// 		printf("BufferTx[%d] = %02x\n\r", i, BufferTx[i]);
				// }

				radio_send_wrapper(payloadSize);
			}
		}

		else if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) && (!edgeTrigger12))// GPIOB2 is released
		{
			if(CenterCheck == true)
			{
				// Send UART message to BMD-350 to alert iPad that manual button 1 has been de-pressed
				dataToSend = 0xFF;
				UART_Check = HAL_UART_Transmit(&huart1, &dataToSend, sizeof(dataToSend), HAL_MAX_DELAY);

				if(UART_Check == HAL_OK)
				{
					printf("UART message sent successfully\n\r");
				}

				else
				{
					printf("UART message failed, error %d\n\r", UART_Check);
				}

				edgeTrigger2 = false;

				uint8_t data = 255; // 0xFF for center moldboard
				memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);
				size_t payloadSize = sizeof(swappedCombinedDeviceID) + sizeof(uint8_t);
				uint8_t combinedBuffer[sizeof(swappedCombinedDeviceID) + sizeof(uint8_t)];
				memcpy(combinedBuffer, swappedCombinedDeviceID, strlen((char*)(swappedCombinedDeviceID)));
				memcpy(combinedBuffer + sizeof(swappedCombinedDeviceID), &data, sizeof(data));
				memcpy(BufferTx, combinedBuffer, payloadSize);

				// for (size_t i = 0; i < 20; ++i)
				// {
				//   	printf("BufferTx[%d] = %02x\n\r", i, BufferTx[i]);
				// }

				radio_send_wrapper(payloadSize);
			}
		}
	}

	else if (GPIO_Pin == GPIO_PIN_12) // Activate Left Wing
	{

		if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) && !edgeTrigger2) // GPIOB12 is pushed down and GPIOB2 is not pushed down
		{
			if(CenterCheck == true)
			{

				// Send UART message to BMD-350 to alert iPad that manual button 1 has been pressed
				dataToSend = 0xF0;
				UART_Check = HAL_UART_Transmit(&huart1, &dataToSend, sizeof(dataToSend), HAL_MAX_DELAY);

				if(UART_Check == HAL_OK)
				{
					printf("UART message sent successfully\n\r");
				}

				else
				{
					printf("UART message failed, error %d\n\r", UART_Check);
				}

				edgeTrigger12 = true;

				uint8_t data = 240; // 0xF0 for left wing
				memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);
				size_t payloadSize = sizeof(swappedCombinedDeviceID) + sizeof(uint8_t);
				uint8_t combinedBuffer[sizeof(swappedCombinedDeviceID) + sizeof(uint8_t)];
				memcpy(combinedBuffer, swappedCombinedDeviceID, strlen((char*)(swappedCombinedDeviceID)));
				memcpy(combinedBuffer + sizeof(swappedCombinedDeviceID), &data, sizeof(data));
				memcpy(BufferTx, combinedBuffer, payloadSize);
				// for (size_t i = 0; i < 20; ++i)
				// {
				//		printf("BufferTx[%d] = %02x\n\r", i, BufferTx[i]);
				//	}

				radio_send_wrapper(payloadSize);
			}
		}

		else if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) && !edgeTrigger2)
		{
			if(CenterCheck == true)
			{

				// Send UART message to BMD-350 to alert iPad that manual button 1 has been pressed
				dataToSend = 0xFF;
				UART_Check = HAL_UART_Transmit(&huart1, &dataToSend, sizeof(dataToSend), HAL_MAX_DELAY);

				if(UART_Check == HAL_OK)
				{
					printf("UART message sent successfully\n\r");
				}

				else
				{
					printf("UART message failed, error %d\n\r", UART_Check);
				}

				edgeTrigger12 = false;
				//transmitLeft = false;
				//transmitRight = true;

				uint8_t data = 255; // 0xFF
				memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);
				size_t payloadSize = sizeof(swappedCombinedDeviceID) + sizeof(uint8_t);
				uint8_t combinedBuffer[sizeof(swappedCombinedDeviceID) + sizeof(uint8_t)];
				memcpy(combinedBuffer, swappedCombinedDeviceID, strlen((char*)(swappedCombinedDeviceID)));
				memcpy(combinedBuffer + sizeof(swappedCombinedDeviceID), &data, sizeof(data));
				memcpy(BufferTx, combinedBuffer, payloadSize);
//				for (size_t i = 0; i < 20; ++i) {
//					printf("BufferTx[%d] = %02x\n\r", i, BufferTx[i]);
//				}
				radio_send_wrapper(payloadSize);
			}
		}
	}


	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}


void heartBeat(void)
{

	if(currentPairingState == Paired)
	{

		uint8_t data = 0x11; // 0x11 for heart beat
		memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);
		size_t payloadSize = sizeof(swappedCombinedDeviceID) + sizeof(uint8_t);
		uint8_t combinedBuffer[sizeof(swappedCombinedDeviceID) + sizeof(uint8_t)];
		memcpy(combinedBuffer, swappedCombinedDeviceID, strlen((char*)(swappedCombinedDeviceID)));
		memcpy(combinedBuffer + sizeof(swappedCombinedDeviceID), &data, sizeof(data));
		memcpy(BufferTx, combinedBuffer, payloadSize);

		printf("\n\r");

		printf("HEARTBEAT: \n\r");

		for (size_t i = 0; i < 5; ++i)
		{
			printf("In HeartBeat: BufferTx[%d] = %02x\n\r", i, BufferTx[i]);
		}

		printf("\n\r");

		radio_send_wrapper(payloadSize);

		 UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);

	}

}

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */

	 printf("\n\r");

     for(int i = 0; i < 5; i++)
	 {
		  printf("In OnTxDone: BufferTx[%d] = %04X\n\r", i, BufferTx[i]);
	 }

     printf("\n\r");

	 // APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");
     /* Update the State of the FSM*/
	 State = TX;


	 /* Run PingPong process in background*/
	  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);

  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */

	// uint32_t receivedIMUDataRaw = 0;
	// float receivedIMUDataParsed = 0.0;



	// APP_LOG(TS_ON, VLEVEL_L, "OnRxDone\n\r");
	#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
	  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
	  /* Record payload Signal to noise ratio in Lora*/
	  SnrValue = LoraSnr_FskCfo;
	#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
	#if ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
	  // APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, Cfo=%dkHz\n\r", rssi, LoraSnr_FskCfo);
	  SnrValue = 0; /*not applicable in GFSK*/
	#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
	  /* Update the State of the FSM*/
	  State = RX;
	  /* Clear BufferRx*/
	  memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
	  /* Record payload size*/
	  RxBufferSize = size;

	  memcpy(BufferRx, payload, RxBufferSize);

//	  printf("\n\r");
//
//	  for(int i = 0; i < RxBufferSize; i++)
//	  {
//		  printf("BufferRx[%d]: %04X\n\r", i, BufferRx[i]);
//	  }
//
//
//		  printf("\n\r");
//
//		  for(int i = 0; i < RxBufferSize; i++)
//		  {
//			  printf("In OnRxDone: payload[%d]: %04X\n\r", i, payload[i]);
//		  }
//
//		  printf("\n\r");
//
//		  for(int i = 0; i < RxBufferSize; i++)
//		  {
//			  printf("In OnRxDone (BEFORE): BufferRx[%d]: %04X\n\r", i, BufferRx[i]);
//		  }
//
//		  printf("\n\r");
//
//		  for(int i = 0; i < RxBufferSize; i++)
//		  {
//			  BufferRx[i] = payload[i];
//		  }
//
//		  for(int i = 0; i < RxBufferSize; i++)
//		  {
//			  printf("In OnRxDone (AFTER): BufferRx[%d]: %04X\n\r", i, BufferRx[i]);
//		  }
//
//		  printf("\n\r");

	  // Need to parse the combined device ID here

	  // Radio.Rx(RX_TIMEOUT_VALUE + random_delay);

	  /* Record Received Signal Strength*/
	  RssiValue = rssi;
	  //printf("RSSI Value: %d dBm\n\r", RssiValue);
	  /* Run PingPong process in background*/
	  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);

  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */

	// APP_LOG(TS_ON, VLEVEL_L, "OnTxTimeout\n\r");
	/* Update the State of the FSM*/
	State = TX_TIMEOUT;
	/* Run PingPong process in background*/
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);

  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */

	// APP_LOG(TS_ON, VLEVEL_L, "OnRxTimeout\n\r");
	/* Update the State of the FSM*/
	State = RX_TIMEOUT;
	/* Run PingPong process in background*/
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);

  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */

	 // APP_LOG(TS_ON, VLEVEL_L, "OnRxError\n\r");
	 /* Update the State of the FSM*/
	 State = RX_ERROR;

 	  printf("\n\r");

 	  for(int i = 0; i < RxBufferSize; i++)
 	  {
 		  printf("BufferRx[%d]: %04X\n\r", i, BufferRx[i]);
 	  }



	 /* Run PingPong process in background*/
	 UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);

  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */

void StormEq_WCS(void) // PingPong_Process
{
	  Radio.Sleep();
	  switch (State)
	  {
	    case RX:


	      // APP_LOG(TS_OFF, VLEVEL_L, "Rx State:\n\r");
	      // Clear BufferRx
	      // memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
	      // start reception

	      HAL_StatusTypeDef UART_Check = HAL_OK;

	      if(currentPairingState == Paired)
		  {
			  uint8_t dataToSend[] = {0x00, 0x00, 0x00};

			  combinedBytes1 = (BufferRx[1] << 8) | BufferRx[0];
			  combinedBytes2 = (BufferRx[3] << 8) | BufferRx[2];

			  printf("deviceIds.combinedDeviceID[0] = %04X\n\r", deviceIds.combinedDeviceID[0]);
			  printf("deviceIds.combinedDeviceID[1] = %04X\n\r", deviceIds.combinedDeviceID[1]);

			  printf("combinedBytes1 = %04X\n\r", combinedBytes1);
			  printf("combinedBytes2 = %04X\n\r", combinedBytes2);

			  if((combinedBytes1 == deviceIds.combinedDeviceID[0]) && (combinedBytes2 == deviceIds.combinedDeviceID[1]))
			  {
				  printf("The first Two Bytes Match!!!!\n\r");


				  uint8_t encodedPayload = BufferRx[4];
			      printf("The encodedPayload is: %02X\n\r", encodedPayload);

			      uint8_t encodedPlowBatteryCharge = (encodedPayload >> 4) & 0xFF;

			      uint8_t encodedPlowAngle = encodedPayload & 0x0F;

			      printf("encodedPlowBatteryCharge = %02X\n\r", encodedPlowBatteryCharge); // 0 -> 0%, 9 -> 90%, 10 -> 100%

			      printf("encodedPlowAngle = %02X\n\r", encodedPlowAngle); // See encoding algorithm sheet


			      dataToSend[0] = 0x03;

			      switch(encodedPlowAngle)
			      {
			          case 0x00:
			              dataToSend[1] = 0x00;
			              break;

			          case 0x01:
			              dataToSend[1] = 0x10;
			              break;

			          case 0x02:
			              dataToSend[1] = 0x20;
			              break;

			          case 0x03:
						  dataToSend[1] = 0x30;
						  break;

			          case 0x04:
						  dataToSend[1] = 0x40;
						  break;

			          case 0x05:
						  dataToSend[1] = 0x50;
						  break;

			          case 0x06:
						  dataToSend[1] = 0x60;
						  break;

			          case 0x07:
						  dataToSend[1] = 0x70;
						  break;

			          case 0x08: // First positive case 2-4 degrees
						  dataToSend[1] = 0x01;
						  break;

			          case 0x09:
						  dataToSend[1] = 0x02;
						  break;

			          case 0xA:
						  dataToSend[1] = 0x03;
						  break;

			          case 0xB:
						  dataToSend[1] = 0x04;
						  break;

			          case 0xC:
						  dataToSend[1] = 0x05;
						  break;

			          case 0xD:
						  dataToSend[1] = 0x06;
						  break;

			          case 0xE:
						  dataToSend[1] = 0x07;
						  break;

			          case 0xF:
						  dataToSend[1] = 0x08;
						  break;

			          default:
			        	  printf("Entered the defualt case!!! ERROR\n\r");
			        	  break;
			      }


			       switch(encodedPlowBatteryCharge)
			       {
			       	   case 0:
			       		   dataToSend[2] = 0xE;
			       		   break;

			       	   case 1:
			       		   dataToSend[2] = 0x26;
			       		   break;

			       	   case 2:
			       		   dataToSend[2] = 0x3E;
			       		   break;

			       	   case 3:
			       		   dataToSend[2] = 0x56;
			       		   break;

			       	   case 4:
			       		   dataToSend[2] = 0x6E;
			       		   break;

			       	   case 5:
			       		   dataToSend[2] = 0x86;
			       		   break;

			       	   case 6:
			       		   dataToSend[2] = 0x9E;
			       		   break;

			       	   case 7:
			       		   dataToSend[2] = 0xB6;
			       		   break;

			       	   case 8:
			       		   dataToSend[2] = 0xCE;
			       		   break;

			       	   case 9:
			       		   dataToSend[2] = 0xE6;
			       		   break;

			       	   case 10:
			       		   dataToSend[2] = 0xFE;
			       		   break;

			       	   default:
			       		   printf("Entered default case!!!! ERROR\n\r");
			       		   break;

			       }

			       for(int i = 0; i < sizeof(dataToSend); i++)
			       {
			    	    printf("dataToSend[%d] = %02X\n\r", i, dataToSend[i]);
			       }



			       UART_Check = HAL_UART_Transmit(&huart1, dataToSend, sizeof(dataToSend), HAL_MAX_DELAY);
				   if(UART_Check == HAL_OK)
				   {
					   printf("UART message sent successfully\n\r");
				   }

				   else
				   {
					   printf("UART message failed, error %d\n\r", UART_Check);
				   }

			  }

			  else
			  {
				  printf("First two bytes DO NOT Match!!!\n\r");
			  }
		  }

		  else
		  {
			  if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
			  {
					memcpy(BufferRx, payload, RxBufferSize);
			  }
		  }


	      Radio.Rx(RX_TIMEOUT_VALUE + random_delay);
	      break;

	    case TX:
	     // APP_LOG(TS_OFF, VLEVEL_L, "Tx State:\n\r");
	      Radio.Rx(RX_TIMEOUT_VALUE);
	      break;

	    case RX_TIMEOUT:
	      //	      APP_LOG(TS_OFF, VLEVEL_L, "Rx_TIMEOUT State:\n\r");
	      //	      APP_LOG(TS_OFF, VLEVEL_L, "Rx start.\n\r");
	      Radio.Rx(RX_TIMEOUT_VALUE);
	      break;

	    case RX_ERROR:
	      //	      APP_LOG(TS_OFF, VLEVEL_L, "Rx_ERROR State:\n\r");
	      HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN + random_delay);
	      Radio.Rx(RX_TIMEOUT_VALUE);
	      break;

	    case TX_TIMEOUT:
	     //  APP_LOG(TS_OFF, VLEVEL_L, "TX_TIMEOUT State:\n\r");
	      HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN + random_delay);
	      Radio.Rx(RX_TIMEOUT_VALUE);
	      break;

	    default:
	      break;
	  }
}

/* USER CODE END PrFD */
