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
#include "main.h"
#include "radio_driver.h"
#include <stdlib.h>


/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

// Pairing Variables
uint8_t receivedAck 				= 0;
uint8_t Ack1[1] 					= {0x0F};
uint8_t Ack2[1] 					= {0xF0};
bool isAck1Received 				= false;
bool isAck2Received 				= false;
uint16_t combinedBytes1 			= 0;
uint16_t combinedBytes2 			= 0;

bool isOtherDeviceIDReceived;

// FSK Tx/Rx Variables
uint8_t *RxData 					= NULL;

// EEPROM variables to save pairing IDS
#define DEBOUNCE_DELAY 				50 // Define a debounce delay (in milliseconds)
#define EEPROM_START_ADDR			0x00

// Define a variable to store the last button state
static GPIO_PinState lastButtonState = GPIO_PIN_RESET;

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
#define PING 						"NI"
#define PINGR 						"RI"
#define RIGHT 						"NR"
#define RIGHTR						"RR"
#define LEFT 						"NL"
#define LEFTR 						"RL"

/* PONG string*/
#define PONG 						"NO"
#define PONGR 						"RO"
/*Size of the payload to be sent*/
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE         255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */
/* wait for remote to be in Rx, before sending a Tx frame*/
#define RX_TIME_MARGIN               0
/* Afc bandwidth in Hz */
#define FSK_AFC_BANDWIDTH            83333
/* LED blink Period*/
#define LED_PERIOD_MS                200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
/*Ping Pong FSM states */
static States_t State 				= RX;
/* App Rx Buffer*/
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize 				= 0;
/* Last  Received packer Rssi*/
int8_t RssiValue 					= 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue 					= 0;
/* Led Timers objects*/
static UTIL_TIMER_Object_t timerLed;
/* device state. Master: true, Slave: false*/
bool isMaster 						= true;
/* random delay to make sure 2 devices will sync*/
/* the closest the random delays are, the longer it will
   take for the devices to sync when started simultaneously*/
static int32_t random_delay;
uint8_t received5ByteData[5] 		= {0x00, 0x00, 0x00, 0x00, 0x00}; // Array to reorganize the Big Endian data back to little endian

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

/**
  * @brief  Function executed on when led timer elapses
  * @param  context ptr of LED context
  */
static void OnledEvent(void *context);

/**
  * @brief PingPong state machine implementation
  */
static void PingPong_Process(void);

void radio_send_wrapper(uint8_t payloadSize);

uint16_t MAX17261_ReadCharge(void);


/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

  APP_LOG(TS_OFF, VLEVEL_M, "\n\rPING PONG\n\r");
  /* Get SubGHY_Phy APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
		  (uint8_t)(APP_VERSION_MAIN),
		  (uint8_t)(APP_VERSION_SUB1),
		  (uint8_t)(APP_VERSION_SUB2));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
		  (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
		  (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
		  (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

  /* Led Timers*/
  UTIL_TIMER_Create(&timerLed, LED_PERIOD_MS, UTIL_TIMER_ONESHOT, OnledEvent, NULL);
  UTIL_TIMER_Start(&timerLed);

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
     APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
     APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
     APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW=%d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
     APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);

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
     APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
     APP_LOG(TS_OFF, VLEVEL_M, "FSK_MODULATION\n\r");
     APP_LOG(TS_OFF, VLEVEL_M, "FSK_BW=%d Hz\n\r", FSK_BANDWIDTH);
     APP_LOG(TS_OFF, VLEVEL_M, "FSK_DR=%d bits/s\n\r", FSK_DATARATE);

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
   UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, PingPong_Process);

   // Register task for processing data
   UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_Process_Data), UTIL_SEQ_RFU, ProcessData);

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

void radio_send_wrapper(uint8_t payloadSize)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	Radio.Send(BufferTx, payloadSize);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

}

//deviceIds.deviceID = deviceId; // assign this deviceID to the struct
// Function to encode the deviceId into the payload buffer as binary number
// Convert the 32-bit deviceId to a 4-byte representation
// Note: This implementation assumes little-endian byte order
// payload does not need to be returned as it is updated globally
// If deviceId is larger than 32-bits, we may need to modify the encoding accordingly

void encodeDeviceId(uint16_t deviceId, uint8_t *payload)
{
	HAL_GPIO_WritePin(PAIR_LIGHT_PORT_2, GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PAIR_LIGHT_PORT_1, RED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUTTON_PORT, BLUE_PIN, GPIO_PIN_SET);

    APP_LOG(TS_OFF, VLEVEL_M,"Entered encodeDeviceId function.\n\r");

    payload[0] = (uint8_t)(deviceId & 0xFF);
    payload[1] = (uint8_t)((deviceId >> 8) & 0xFF);

    deviceIds.deviceID = (uint16_t)(payload[0] | (payload[1] << 8)); // assign this deviceID to the struct (use the encoded version)

    APP_LOG(TS_OFF, VLEVEL_M,"This device ID is (struct): 0x%04X\n\r", deviceIds.deviceID);

    return;
}

void broadcastDeviceId(uint8_t* payload)
{
	printf("Entered BroadcastDeviceID function.\n\r");

	HAL_GPIO_WritePin(PAIR_LIGHT_PORT_2, GREEN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PAIR_LIGHT_PORT_1, RED_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUTTON_PORT, BLUE_PIN, GPIO_PIN_SET);

	while(!isOtherDeviceIDReceived)
	{
		//Clear BufferRx
		printf("Plow PCB Listening.\n\r");
		memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
		// start reception
		Radio.Rx(RX_TIMEOUT_VALUE + random_delay);
		HAL_Delay(100);

		// check to see if message is received
		if(RxBufferSize >= sizeof(uint16_t))
		{

			// for(int i = 0; i < RxBufferSize; i++)
			// {
			//	 if(BufferRx[i] != 0)
			//	 {
			//		printf("Raw Data[%d]: 0x%02X\n\r", i, BufferRx[i]);
			//	 }
			// }
			// int8_t rssiValue = SUBGRF_GetRssiInst();
			// printf("RSSI Value: %d dBm.\n\r", rssiValue);

			// Process the received message
			memcpy(&otherDeviceID, BufferRx, sizeof(uint16_t));
			printf("Received Payload is: 0x%04X\n\r", otherDeviceID);

			if(otherDeviceID != 0x0000)
			{

				deviceIds.otherDeviceID 				= otherDeviceID;
				deviceIds.combinedDeviceID[0] 			= deviceIds.deviceID;
				deviceIds.combinedDeviceID[1] 			= deviceIds.otherDeviceID;
				deviceIds.swappedCombinedDeviceID[0] 	= deviceIds.combinedDeviceID[1];
				deviceIds.swappedCombinedDeviceID[1] 	= deviceIds.combinedDeviceID[0];
				uint16_t pairingId1 					= deviceIds.deviceID;
				uint16_t pairingId2 					= deviceIds.otherDeviceID;

				saveIDsToEEPROM(&hi2c1, pairingId1, pairingId2, EEPROM_Start_Address);

				printf("\n\r\n\rotherDeviceID is (struct): 0x%04X\n\r\n\r", deviceIds.otherDeviceID);
				isOtherDeviceIDReceived = true;

				HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);

				for(int i = 0; i < 5; i++)
				{
					printf("Sending Ack1 x5\n\r");
					memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);  // Clear the BufferTx array
					size_t payloadSize = sizeof(Ack1);
					memcpy(BufferTx, (uint8_t *)Ack1, payloadSize);  // Copy the payload to BufferTx
					// Radio.Send(BufferTx, payloadSize); // Broadcast deviceId x infinity times before listening
					radio_send_wrapper(payloadSize);
				}
			}

			else
			{
				printf("received a packet with payload 0x0000, discarding it.\n\r");
			}
		}

		GPIO_PinState buttonState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
		uint32_t currentTime = HAL_GetTick(); // Get current time in milliseconds

		if(buttonState == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET && (currentTime - lastButtonPressTime) >= DEBOUNCE_DELAY) // button is pressed (low voltage)  changed to high voltage
		{
			APP_LOG(TS_OFF, VLEVEL_M,"1. Entered buttonState reset to Unknown!!!!\n\r");
			isOtherDeviceIDReceived 	= false;
			isAck1Received 				= false;
			currentPairingState 		= Unknown;
			lastButtonPressTime 		= currentTime;
			return;
		}

		lastButtonState = buttonState;
	}

	// Transmit this deviceID second
	while(!isAck1Received)
	{
		// Broadcast Local device ID
		printf("PLOW PCB transmitting.\n\r");
		memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);
		size_t payloadSize = strlen((char *)payload);
		memcpy(BufferTx, payload, payloadSize);
		radio_send_wrapper(payloadSize);

		HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);

		// printf("Listen for Ack1.\n\r");
		memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
		Radio.Rx(RX_TIMEOUT_VALUE + random_delay);
		HAL_Delay(100);

		// check to see if Ack1 is received
		if(RxBufferSize >= sizeof(uint8_t))
		{

			printf("Message received.\n\r");
			memcpy(&receivedAck, BufferRx, sizeof(uint8_t));
			printf("Received message is: 0x%02X\n\r", receivedAck);

			// int8_t rssiValue = SUBGRF_GetRssiInst();
			// printf("RSSI: %d dBm\n\r", rssiValue);

			if(receivedAck == 0x0F)
			{
				printf("\n\r\n\rAck1 message received!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\r\n\r");
				isAck1Received = true;
				currentPairingState = Paired;
				break;
			}

			else
			{
				// printf("Ack1 message NOT received. Continuing with Broadcast Function.\n\r");
			}
		}

		GPIO_PinState buttonState 		= HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
		uint32_t currentTime 			= HAL_GetTick(); // Get current time in milliseconds

		if(buttonState == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET && (currentTime - lastButtonPressTime) >= DEBOUNCE_DELAY) // button is pressed (low voltage)  changed to high voltage
		{
			APP_LOG(TS_OFF, VLEVEL_M,"1. Entered buttonState reset to Unknown!!!!\n\r");
			isOtherDeviceIDReceived 	= false;
			isAck1Received 				= false;
			currentPairingState 		= Unknown;
			lastButtonPressTime 		= currentTime;
			return;
		}

		lastButtonState = buttonState;
	}
}

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */

	 // APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");
	 /* Update the State of the FSM*/

	 //	for(int i = 0; i < 8; i++)
	 //	{
	 //		printf("In OnTxDone: BufferTx[%d] = %04X\n\r", i, BufferTx[i]);
	 //	}

	 State = TX;
	 /* Run PingPong process in background*/
	 UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);

  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */

	  //	for(int i = 0; i < size; i++)
	  //	{
	  //		printf("In OnRxDone: payload[%d] = %04X\n\r", i, payload[i]);
	  //	}

	  //	printf("\n\r");

	  //APP_LOG(TS_ON, VLEVEL_L, "OnRxDone\n\r");
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

	  /* Clear BufferRx*/
	  memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
	  /* Record payload size*/
	  RxBufferSize = size;
	  // printf("printf RxBufferSize is : %02X\n\r", RxBufferSize);

	  // for(int i = 0; i < RxBufferSize; i++)
	  // {
	  //     printf("BufferRx[%d]: %02X\n\r", i, BufferRx[i]);
	  // }

	  if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
	  {
		  memcpy(BufferRx, payload, RxBufferSize);
		  //   printf("OnRxDone BufferRx: 0x%04X\n\r", BufferRx);
	  }
	  /* Record Received Signal Strength*/
	  RssiValue = rssi;
	  //     printf("RSSI OnRxDone Value: %d\n\r", RssiValue);

	  /* Record payload content*/
	   APP_LOG(TS_ON, VLEVEL_H, "payload. size=%d \n\r", size);
	   for (int i = 0; i < PAYLOAD_LEN; i++)
	   {
		   APP_LOG(TS_OFF, VLEVEL_H, "%02X", BufferRx[i]);
		   if (i % 16 == 15)
		   {
			   APP_LOG(TS_OFF, VLEVEL_H, "\n\r");
		   }
	   }
	   // APP_LOG(TS_OFF, VLEVEL_H, "\n\r");

	   if(currentPairingState == Paired)
	   {
		   if (BufferRx[0] != 0)
		   {
				// APP_LOG(TS_OFF, VLEVEL_M, "APP_LOG RxBufferSize: %04X\n\r", RxBufferSize);

				// for (int i = 0; i < RxBufferSize; i++)
				// {
				//     APP_LOG(TS_OFF, VLEVEL_M, "RX State: BufferRx[%d]: %02X\n\r", i, BufferRx[i]);
				// }

				received5ByteData[0] = BufferRx[1];
				received5ByteData[1] = BufferRx[0];
				received5ByteData[2] = BufferRx[3];
				received5ByteData[3] = BufferRx[2];
				received5ByteData[4] = BufferRx[4];

				// for(int i = 0; i < 5; i++)
				// {
				//    	printf("received5ByteData[%d] = 0x%02X\n\r", i, received5ByteData[i]);
				// }

				combinedBytes1 = (received5ByteData[0] << 8) | received5ByteData[1];
				combinedBytes2 = (received5ByteData[2] << 8) | received5ByteData[3];

				if((combinedBytes1 == deviceIds.combinedDeviceID[0]) && (combinedBytes2 == deviceIds.combinedDeviceID[1]))
				{
					printf("The first two bytes match!!\n\r");

					if(received5ByteData[4] == 0xF0)
					{
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
					}

					else if(received5ByteData[4] == 0x0F)
					{
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
					}

					else if(received5ByteData[4] == 0xFF)
					{
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
					}

					else if(received5ByteData[4] == 0x11)
					{
						sendIMUData_FLAG = true;
					    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_Process_Data), CFG_SEQ_Prio_3);

					}
				}

				else
				{
					printf("The first two bytes DO NOT match\n\r");
				}
		   }
	  }

	   State = RX;

	  /* Run PingPong process in background*/
	  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);

  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */

	//APP_LOG(TS_ON, VLEVEL_L, "OnTxTimeout\n\r");
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
	/* Run PingPong process in background*/
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);

  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */


// Send and Receive Data
void ProcessData(void)
{
    // Process reception
    // Example:
    // if (RxBufferSize > 0) {
    //     // Process received data
    // }

	if(currentPairingState == Paired)
	{
		if (sendIMUData_FLAG)
		{

			uint8_t EncodedPayload = 0x00;

			uint16_t chargeLevel = MAX17261_ReadCharge();
			// printf("ChargeLevel = %04X\n\r", chargeLevel);
			uint8_t chargeHexInteger = (chargeLevel >> 8) & 0xFF;
			// printf("Charge Hex Integer = %02X\n\r", chargeHexInteger);
			/////////////////////////////////////////////////////////////////////////////////////////
			// Convert the integer to a string representation
			char hexBatteryString[3];
			sprintf(hexBatteryString, "%02X", chargeHexInteger); // Cast chargeHexInteger as a string and save it in hexString

			// Convert hexadecimal string to decimal using strtol
			long decimalBatteryValue = strtol(hexBatteryString, NULL, 16);
			// Print the decimal value
			printf("Charge Decimal Value = %ld\n\r", decimalBatteryValue);

			// Get the 10s place value
			uint8_t batteryChargeTensPlace = (uint8_t)decimalBatteryValue / 10;

			// Print the 10s place value
			// printf("Charge Tens place value = %ld\n\r", batteryChargeTensPlace);
			/////////////////////////////////////////////////////////////////////////////////////
			sendIMUData = READ_IMU_DATA();
			uint8_t imuData8Bit = (uint8_t)sendIMUData;///////////////////// May want to subtract 90 here
			printf("IMU Data to Send (uint8_t): %d\n\r", imuData8Bit);

			char hexAngleString[3];
			sprintf(hexAngleString, "%02X", imuData8Bit);

			// Convert hexadecimal string to decimal using strtol
			long decimalAngleValue = strtol(hexAngleString, NULL, 16);

			printf("Angle Decimal Value = %ld\n\r", decimalAngleValue);

			if((decimalAngleValue >= 89) && (decimalAngleValue <= 91))
			{
				EncodedPayload = 0x00;
			}

			else if((decimalAngleValue >= 86) && (decimalAngleValue <= 88))
			{
				EncodedPayload = 0x01;
			}

			else if((decimalAngleValue >= 83) && (decimalAngleValue <= 85))
			{
				EncodedPayload = 0x02;
			}

			else if((decimalAngleValue >= 80) && (decimalAngleValue <= 82))
			{
				EncodedPayload = 0x03;
			}

			else if((decimalAngleValue >= 77) && (decimalAngleValue <= 79))
			{
				EncodedPayload = 0x04;
			}

			else if((decimalAngleValue >= 74) && (decimalAngleValue <= 76))
			{
				EncodedPayload = 0x05;
			}

			else if((decimalAngleValue >= 71) && (decimalAngleValue <= 73))
			{
				EncodedPayload = 0x06;
			}

			else if((decimalAngleValue >= 68) && (decimalAngleValue <= 70))
			{
				EncodedPayload = 0x07;
			}

			else if(decimalAngleValue < 68)
			{
				EncodedPayload = 0x07;
			}
			/////////////////////////////////////////////////////////////////////////

			else if((decimalAngleValue >= 92) && (decimalAngleValue <= 94))
			{
				EncodedPayload = 0x08;
			}

			else if((decimalAngleValue >= 95) && (decimalAngleValue <= 97))
			{
				EncodedPayload = 0x09;
			}

			else if((decimalAngleValue >= 98) && (decimalAngleValue <= 100))
			{
				EncodedPayload = 0xA;
			}

			else if((decimalAngleValue >= 101) && (decimalAngleValue <= 103))
			{
				EncodedPayload = 0xB;
			}

			else if((decimalAngleValue >= 104) && (decimalAngleValue <= 106))
			{
				EncodedPayload = 0xC;
			}

			else if((decimalAngleValue >= 107) && (decimalAngleValue <= 109))
			{
				EncodedPayload = 0xD;
			}

			else if((decimalAngleValue >= 110) && (decimalAngleValue <= 112))
			{
				EncodedPayload = 0xE;
			}

			else if((decimalAngleValue >= 113) && (decimalAngleValue <= 115))
			{
				EncodedPayload = 0xF;
			}

			else if(decimalAngleValue > 115)
			{
				EncodedPayload = 0xF;
			}

			// printf("(Before Battery Charge Added) EncodedPayload = %02X\n\r", EncodedPayload);

			///////////////////////////////////////////////////////////////////////

			EncodedPayload = (batteryChargeTensPlace << 4) | EncodedPayload;

			// printf("(After Battery Charge Added) EncodedPayload = %02X\n\r", EncodedPayload);

			////////////////////////////////////////////////////////////////////////

			HAL_Delay(10);

			size_t payloadSize = 5;  //////////////////// -- Change back to 5!!!!!!!!!!!!1
			uint8_t combinedPayload[payloadSize];

			combinedPayload[1] = (deviceIds.swappedCombinedDeviceID[0] >> 8) & 0xFF;
			combinedPayload[0] = deviceIds.swappedCombinedDeviceID[0] & 0xFF;
			combinedPayload[3] = (deviceIds.swappedCombinedDeviceID[1] >> 8) & 0xFF; // Shift right by 8 bits
			combinedPayload[2] = deviceIds.swappedCombinedDeviceID[1] & 0xFF;

			// Load imuData8Bit into combinedPayload
			// combinedPayload[4] = imuData8Bit;

			combinedPayload[4] = EncodedPayload;

			// Load chargeHexInteger into combinedPayload
			// combinedPayload[5] = (uint8_t)chargeHexInteger;  /// RE COMMENT OUT !!!!!!!!!1

			// Clear BufferTx before copying data
			memset(BufferTx, 0, MAX_APP_BUFFER_SIZE);

			// Copy combinedPayload into BufferTx
			memcpy(BufferTx, combinedPayload, payloadSize);

			for(int i = 0; i < payloadSize; i++) // Iterate up to payloadSize
			{
				printf("BufferTx[%d] = %02X\n\r", i, BufferTx[i]);
			}

			printf("\n\r");

			radio_send_wrapper(payloadSize);

			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
		}
		sendIMUData_FLAG = false;
	}
}

void PingPong_Process(void)
{
  Radio.Sleep();

  switch (State)
  {
    case RX:
    	// APP_LOG(TS_OFF, VLEVEL_M, "Rx State (Transmit):\n\r");
    	// if (RxBufferSize > 0)

    	// memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);

    	Radio.Rx(RX_TIMEOUT_VALUE);
    	break;

    case TX:
         // APP_LOG(TS_OFF, VLEVEL_M, "Tx State (Receive):\n\r");
         // APP_LOG(TS_ON, VLEVEL_L, "Rx start\n\r");

         Radio.Rx(RX_TIMEOUT_VALUE);
         break;

    case RX_TIMEOUT:
    	 // APP_LOG(TS_OFF, VLEVEL_M, "RX_TIMEOUT State:\n\r");
         //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
         //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    	 break;

    case RX_ERROR:
         // APP_LOG(TS_OFF, VLEVEL_M, "RX_ERROR State:\n\r");
    	 // if (RxBufferSize > 0)
    	 // {
    	 //    APP_LOG(TS_OFF, VLEVEL_M, "RxBufferSize: %04X\n\r", RxBufferSize);
    	 //
    	 //    for(int i = 0; i < RxBufferSize; i++)
    	 //    {
    	 //         APP_LOG(TS_OFF, VLEVEL_M, "BufferRx[%d]: %02X\n\r", i, BufferRx[i]);
    	 //    }
    	 // }

         if (isMaster == true)
         {
        	 /* Send the next PING frame */
        	 /* Add delay between RX and TX*/
        	 /* add random_delay to force sync between boards after some trials*/
        	 HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN + random_delay);
        	 // APP_LOG(TS_ON, VLEVEL_L, "Master Tx start\n\r");
        	 /* master sends PING*/
        	 memcpy(BufferTx, PING, sizeof(PING) - 1);
        	 radio_send_wrapper(PAYLOAD_LEN);
        	 //Radio.Send(BufferTx, PAYLOAD_LEN);
         }
         else
         {
        	 // APP_LOG(TS_ON, VLEVEL_L, "Slave Rx start\n\r");
        	 Radio.Rx(RX_TIMEOUT_VALUE);
         }

         Radio.Rx(RX_TIMEOUT_VALUE);

         break;

    case TX_TIMEOUT:
         // APP_LOG(TS_OFF, VLEVEL_M, "TX_TIMEOUT State:\n\r");
    	 // APP_LOG(TS_ON, VLEVEL_L, "Slave Rx start\n\r");
    	 Radio.Rx(RX_TIMEOUT_VALUE);
    	 break;

    default:
    	 break;

  }

}

static void OnledEvent(void *context)
{
  UTIL_TIMER_Start(&timerLed);
}

/* USER CODE END PrFD */
