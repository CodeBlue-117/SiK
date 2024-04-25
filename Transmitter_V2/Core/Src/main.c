/* USER CODE BEGIN Header */
// NGRF WCS for StormEq
// Jake Price
// 11/13/23

/**
 * @brief Wireless Control Service (WCS)
 * @defgroup bt_wcs Wireless Control Service (WCS)
 * @ingroup FSK
 * @{
 *
 * [Experimental] Users should note that the APIs can change
 * as a part of ongoing development.
 */

/*
 * Copyright (c) 2023 NGRF
 *
 *
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "app_subghz_phy.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "subghz_phy_app.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "radio.h"
#include "sys_app.h"
#include "stm32wlxx_hal_i2c.h"
#include "radio_driver.h"
#include "usart_if.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// LTC4015 Config Variables
#define LTC4015_I2C_ADDRESS 				 	 0xD0
#define LTC4015_I2C_WRITE_ADDRESS				 0xD0
#define LTC4015_I2C_READ_ADDRESS				 0xD1
// #define LTC4015_I2C_WRITE_ADDRESS		     0x68
// #define LTC4015_I2C_READ_ADDRESS				 0x69
// #define LTC4015_I2C_ADDRESS      			 ((uint16_t)0xD0)
// #define  LTC4015_I2C_ADDRESS 				 0x68
// uint16_t LTC4015_I2C_ADDRESS 			     = (uint16_t)0xD0;

#define LTC4015_CONTROL_REGISTER				0x14
#define EN_QCOUNT_BIT							(1 << 2)

// Sub-address for Coulomb Counter Prescale Factor
#define QCOUNT_PRESCALE_FACTOR_REGISTER_ADDR 	0x12

// Sub-address for Coulomb Counter Accumulator Value
#define QCOUNT_REGISTER_ADDR					0x13

// EEPROM Read and Write Addresses
#define EEPROM_ADDR 							0xA0
// #define EEPROM_START_ADDR					0x0000
#define CLEAR_DATA_VALUE 						0xFFFF

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Pairing Variables
PairingStates_t currentPairingState;

uint32_t lastButtonPressTime 		= 0;
uint8_t payload[2] 					= {0}; // This loads the Device IDs into Buffer Tx
uint16_t deviceId 					= 0;   // This is the local device ID
uint16_t otherDeviceID 				= 0;   // This is the remote device ID
uint16_t combinedDeviceID[2] 		= {0}; // This is the local combination (unswapped)
uint16_t swappedCombinedDeviceID[2] = {0}; // This is the local combination that we swap before sending so that it matches the remote device's otherCombinedDeviceID
uint16_t *EEPROM_PairingID1 		= NULL;
uint16_t *EEPROM_PairingID2			= NULL;
uint16_t EEPROM_Start_Address		= 0x0000;
bool isPairingConfirmed		 		= false;
bool isRepaired 			 		= false;	// debounce if this device successfully pairs, but other device fails
bool isPairingButtonPressed  		= false;
bool pairingFlag             		= false;
DeviceIDS deviceIds = {.deviceID 	= 0, .otherDeviceID = 0, .combinedDeviceID = {0, 0}, .swappedCombinedDeviceID = {0, 0}};

// UART Edge Trigger Flag
bool CenterCheck 					= true;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t cabChargeLevel 			= 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void UARTCallback(UART_HandleTypeDef *huart);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void LTC4015_WriteRegister(I2C_HandleTypeDef *hi2c1, uint16_t registerAddr, uint16_t data)
{

	//	printf( "\n\rEntered LTC4015_WriteRegister()!!!!!!!!!!!!!!!!!!!!!\n\r");
	uint16_t writeAddress = (LTC4015_I2C_ADDRESS);
	HAL_StatusTypeDef status;

	uint8_t sendData[2] = {0};
	sendData[1] = (data >> 8) & 0xFF;
	sendData[0] = data & 0xFF;

    // printf("\n\rPre-Write Data: %u, %u\n\r", registerAddr, sendData[0], sendData[1]);
	// status = HAL_I2C_Mem_Write(hi2c1, writeAddress, registerAddr, I2C_MEMADD_SIZE_16BIT, &sendData, 1, HAL_MAX_DELAY);

	//	if(status == HAL_OK) {
	//	    printf("HAL_I2C_Mem_Write successful\n\r");
	//	} else if(status == HAL_ERROR) {
	//	    printf("HAL_I2C_Mem_Write error: HAL_ERROR\n\r");
	//	} else if(status == HAL_BUSY) {
	//	    printf("HAL_I2C_Mem_Write error: HAL_BUSY\n\r");
	//	} else if(status == HAL_TIMEOUT) {
	//	    printf("HAL_I2C_Mem_Write error: HAL_TIMEOUT\n\r");
	//	} else {
	//	    printf("Unknown error occurred during HAL_I2C_Mem_Write\n\r");
	//	}

}

uint16_t LTC4015_ReadRegister(I2C_HandleTypeDef *hi2c1, uint16_t registerAddr)
{
	//	printf( "\n\rEntered LTC4015_ReadRegister()!!!!!!!!!!!!!!!!!!!!!!\n\r");
	uint16_t readAddress = (LTC4015_I2C_ADDRESS);
	HAL_StatusTypeDef status;
	uint8_t *data = malloc(2 * sizeof(uint8_t));
	//uint8_t data[2] = {0};

	//	status = HAL_I2C_Mem_Read(hi2c1, readAddress, registerAddr, I2C_MEMADD_SIZE_16BIT, &data, 2, HAL_MAX_DELAY);
	//    if (status != HAL_OK)
	//    {
	//        printf("\n\rHAL_I2C_Mem_Read Failed\n\r");
	//        return 0;
	//    }
	//    else
	//    {
	//        printf("\n\rSUCCESS Read from register (HEX) 0x%04X: Data (HEX, HEX) = %02X, %02X\n\r", registerAddr, data[0], data[1]);
	//    }

    uint16_t registerValue = (data[0] << 8) | data[1];
    // Print the register value
    printf("\n\rRead from register (HEX) 0x%04X:  (HEX) 0x%04X\n\r", registerAddr, registerValue);

    return registerValue;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////// Added this
//    status = HAL_SMBUS_Master_Transmit_IT(&hsmbus1, combinedAddress, (uint8_t *)&registerAddr, 1, 0);
//    if (status != HAL_OK)
//	{
//    	printf("\n\r1) HAL_SMBUS_Master_Transmit_IT Failed\n\r");
//	}
//	/////////////////////////
//
//	status = HAL_SMBUS_Master_Transmit_IT(&hsmbus1, combinedAddress, (uint8_t *)&data, 1, 0);
//    if (status != HAL_OK)
//    {
//    	printf("\n\r2) HAL_SMBUS_Master_Transmit_IT Failed\n\r");
//    }

//    //  --- Added this ---/////////////////////////////////////
//    status = HAL_SMBUS_Master_Transmit_IT(&hsmbus1, combinedAddress, (uint8_t *)&registerAddr, 1, 0);
//    if (status != HAL_OK)
//	{
//    	printf("\n\rHAL_SMBUS_Master_Transmit_IT Failed\n\r");
//	}
//    /////////////////////////////////////////////////////////
//
//    status = HAL_SMBUS_Master_Receive_IT(&hsmbus1, combinedAddress, (uint8_t *)&data, 1, 0);
//    if (status != HAL_OK)
//    {
//    	 printf("\n\rHAL_SMBUS_Mem_Read: Read from register 0x%04X failed. HAL Status: %u\n\r", registerAddr, status);
//    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to set the 'en_qcount' bit in the LTC4015
void EnableCoulombCounter(void)
{

	//printf( "\n\rEntered EnableCoulombCounter()!!!!!!!!!!!!!!!!!!!!!\n\r");

	/////////////////////////////////// INITIAL VALUES ////////////////////////////

	printf( "\n\r\n\r---INITIAL VALUES---\n\r\n\r");
	// Read the current value of the control register
    uint16_t ControlRegisterValue = LTC4015_ReadRegister(&hi2c1, LTC4015_CONTROL_REGISTER); // 0x0A
    printf( "\n\rInitial REGISTER (HEX) %02X (CONTROL REGISTER) contents: (HEX) %02X\n\r", LTC4015_CONTROL_REGISTER, ControlRegisterValue);
    //
    //    uint16_t preValue = LTC4015_ReadRegister(&hsmbus1, QCOUNT_PRESCALE_FACTOR_REGISTER_ADDR);
    //    printf( "\n\rInitial REGISTER 0x12 (PRESCALER) contents: %02X\n\r", preValue);
    //
    //    uint16_t preValue1 = LTC4015_ReadRegister(&hsmbus1, QCOUNT_REGISTER_ADDR);
    //    printf( "\n\rInitial REGISTER QCOUNT %02X\n\r", preValue1);

	printf( "\n\r\n\r---INITIAL VALUES---\n\r\n\r");

    // Set the 'en_qcount' bit while keeping other bits unchanged
	// uint16_t newControlValue = EN_QCOUNT_BIT;
    uint16_t newControlValue = ControlRegisterValue | EN_QCOUNT_BIT;
    printf( "\n\rnew control Value after setting EN_QCOUNT_BIT: (HEX) %02X\n\r", newControlValue);

    // Write the modified value back to the control register
    LTC4015_WriteRegister(&hi2c1, LTC4015_CONTROL_REGISTER, newControlValue);

    // Debug statement to check the new value saved to the register
    uint16_t newRegisterValue = LTC4015_ReadRegister(&hi2c1, LTC4015_CONTROL_REGISTER);
    printf( "\n\r\n\rnew Control Register Value after write: (HEX) %04X.\n\r\n\r", newRegisterValue);

}

// Function to set Coulomb Counter Prescale Factor
void SetCoulombCounterPrescaleFactor(uint8_t value)
{

	// printf("Entered  SetCoulombCounterPrescaleFactor function!!!!!!!!!!!!!!!!!!!\n\r");
    // Write the value to the specified sub-address

    LTC4015_WriteRegister(&hi2c1, QCOUNT_PRESCALE_FACTOR_REGISTER_ADDR, value);

    uint8_t newRegisterValue = LTC4015_ReadRegister(&hi2c1, QCOUNT_PRESCALE_FACTOR_REGISTER_ADDR);

    printf( "\n\r\n\rnew Control Register Value after write: (HEX) %02X\n\r\n\r", newRegisterValue);
}

// Function to set Coulomb Counter Accumulator Value
void SetCoulombCounterAccumulatorValue(uint16_t value)
{
	// printf("Entered SetCoulombCounterAccumulatorValue function!!!!!!!!!!!!!!!!!!\n\r");
    /////////////////////////////// Double check this!!!!!!!!!!!!!!!!!
    // Write the value to the specified sub-address (16-bit write)
    LTC4015_WriteRegister(&hi2c1, QCOUNT_REGISTER_ADDR, (uint8_t)(value & 0xFF));         		// Low byte
    LTC4015_WriteRegister(&hi2c1, QCOUNT_REGISTER_ADDR + 1, (uint8_t)((value >> 8) & 0xFF));      // High byte

    uint8_t newRegisterValue = LTC4015_ReadRegister(&hi2c1, QCOUNT_REGISTER_ADDR);
    printf( "\n\rnew Control Register Value after write: (HEX) %02X\n\r", newRegisterValue);

}

// Function to calculate State of Charge (SoC) without using floats
uint32_t CalculateSOC(void)
{
	printf("Entered CalculateSOC function!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\r");

    // Constants from the formula
    uint16_t offset  = 16384;
    uint16_t divisor = 32768;
    uint8_t scale    = 1; // set to 1 for 100%

    // Read QCOUNT from register 0x13
    uint16_t qcount        = LTC4015_ReadRegister(&hi2c1, QCOUNT_REGISTER_ADDR);
    printf( "\n\rQCOUNT1 %02X\n\r", qcount);
    // Perform integer-based SoC calculation
    uint32_t soc           = scale * (qcount - offset);
    soc /= divisor;

    // Ensure SoC is within the valid range (0 to 100)
    soc = (soc < 0) ? 0 : soc;
    soc = (soc > 100) ? 100 : soc;

    printf("SOC = %u\n\r", soc);

    uint8_t preValue = LTC4015_ReadRegister(&hi2c1, 0x12);
    printf( "\n\rPRESCALE FACTOR %02X\n\r", preValue);

    uint8_t preValue1 = LTC4015_ReadRegister(&hi2c1, 0x13);
    printf( "\n\rQCOUNT2 %02X\n\r", preValue1);

    return soc;
}

// EEPROM Read and Write Functions
void saveIDsToEEPROM(I2C_HandleTypeDef *hi2c3, uint16_t id1, uint16_t id2, uint16_t startAddress)
{

	// Construct the high and low bytes of the pairingID 1
	uint8_t data_high_byte_1  = (id1 >> 8) & 0xFF;
	uint8_t data_low_byte_1   = id1 & 0xFF;

	// Construct the high and low bytes of pairing ID 2
	uint8_t data_high_byte_2  = (id2 >> 8) & 0xFF;
	uint8_t data_low_byte_2   = id2 & 0xFF;

	// Perform the I2C write operations

    printf("Writing to address: 0x%02X, id1 Value: 0x%02X, id2 Value: 0x%02X\n\r", startAddress, id1, id2);
    printf("data_high_byte_1: 0x%02X, data_low_byte_1: 0x%02X\n\r", data_high_byte_1, data_low_byte_1);
    printf("data_high_byte_2: 0x%02X, data_low_byte_2: 0x%02X\n\r", data_high_byte_2, data_low_byte_2);

	// Send high and low bytes of pairingId1
    HAL_StatusTypeDef receiveStatus = HAL_I2C_Mem_Write(hi2c3, EEPROM_ADDR, startAddress + 1, 1, &data_high_byte_1, 1, HAL_MAX_DELAY);

	if (receiveStatus == HAL_OK)
	{
		printf("HAL_I2C_Mem_Write to EEPROM Successful!\n\r");
	}
	else
	{
		printf("HAL_I2C_Mem_Write to EEPROM Failed.\n\r");
	}

	// Send high and low bytes of pairingId2
	receiveStatus = HAL_I2C_Mem_Write(hi2c3, EEPROM_ADDR, startAddress + 2, 1, &data_low_byte_1, 1, HAL_MAX_DELAY);

	if (receiveStatus == HAL_OK)
	{
		printf("HAL_I2C_Mem_Write to EEPROM Successful!\n\r");
	}
	else
	{
		printf("HAL_I2C_Mem_Write to EEPROM Failed.\n\r");
	}

	// Send high and low bytes of pairingId2
	receiveStatus = HAL_I2C_Mem_Write(hi2c3, EEPROM_ADDR, startAddress + 3, 1, &data_high_byte_2, 1, HAL_MAX_DELAY);

	if (receiveStatus == HAL_OK)
	{
		printf("HAL_I2C_Mem_Write to EEPROM Successful!\n\r");
	}
	else
	{
		printf("HAL_I2C_Mem_Write to EEPROM Failed.\n\r");
	}

	 // Send high and low bytes of pairingId2
	receiveStatus = HAL_I2C_Mem_Write(hi2c3, EEPROM_ADDR, startAddress + 4, 1, &data_low_byte_2, 1, HAL_MAX_DELAY);

	if (receiveStatus == HAL_OK)
	{
		printf("HAL_I2C_Mem_Write to EEPROM Successful!\n\r");
	}
	else
	{
		printf("HAL_I2C_Mem_Write to EEPROM Failed.\n\r");
	}

}

bool readIDsFromEEPROM(I2C_HandleTypeDef *hi2c3, uint16_t *id1, uint16_t *id2, uint16_t startAddress)
{

	printf("Read Device IDS from EEPROM\n\r");

	// construct the read buffer and temp variables
	uint8_t * read_buffer 	= NULL;
	uint16_t temp1 			= 0;
	uint16_t temp2 			= 0;

	read_buffer = (uint8_t *)malloc(4 * sizeof(uint8_t));

    HAL_StatusTypeDef receiveStatus = HAL_I2C_Mem_Read(hi2c3, EEPROM_ADDR, startAddress + 1, 1, &read_buffer[0], 1, HAL_MAX_DELAY);
	if (receiveStatus != HAL_OK)
	{
		printf("Read Failed (ID1) - HAL Status: %u\n\r", receiveStatus);
		return false;
	}

	receiveStatus = HAL_I2C_Mem_Read(hi2c3, EEPROM_ADDR, startAddress + 2, 1, &read_buffer[1], 1, HAL_MAX_DELAY);
	if (receiveStatus != HAL_OK)
	{
		printf("Read Failed (ID2) - HAL Status: %u\n\r", receiveStatus);
		return false;
	}

	*id1 = (uint16_t)(read_buffer[0] << 8) | read_buffer[1];

	temp1 = *id1;

	if((temp1 != 0x0000) && (temp1 != 0xFFFF))
	{
		deviceIds.deviceID = temp1;
		printf("4. deviceIds.deviceID: 0x%04X\n\r", deviceIds.deviceID);
		deviceIds.combinedDeviceID[0]        = deviceIds.deviceID;
	}

	else
	{
		printf("4. No deviceID saved to EEPROM, waiting for pairing...\n\r");
	}

    receiveStatus = HAL_I2C_Mem_Read(hi2c3, EEPROM_ADDR, startAddress + 3, 1, &read_buffer[2], 1, HAL_MAX_DELAY);
	if (receiveStatus != HAL_OK)
	{
		printf("Read Failed (ID1) - HAL Status: %u\n\r", receiveStatus);
		return false;
	}

	receiveStatus = HAL_I2C_Mem_Read(hi2c3, EEPROM_ADDR, startAddress + 4, 1, &read_buffer[3], 1, HAL_MAX_DELAY);
	if (receiveStatus != HAL_OK)
	{
		printf("Read Failed (ID2) - HAL Status: %u\n\r", receiveStatus);
		return false;
	}

	*id2 = (uint16_t)(read_buffer[2] << 8) | read_buffer[3];
	temp2 = *id2;

	if((temp2 != 0x0000) && (temp2 != 0xFFFF))
	{
		deviceIds.otherDeviceID = temp2;
		printf("6. deviceIds.otherDeviceID: 0x%04X\n\r", deviceIds.otherDeviceID);

		deviceIds.combinedDeviceID[1]        = deviceIds.otherDeviceID;
		deviceIds.swappedCombinedDeviceID[0] = deviceIds.combinedDeviceID[1];
		deviceIds.swappedCombinedDeviceID[1] = deviceIds.combinedDeviceID[0];

		swappedCombinedDeviceID[0] 			 = deviceIds.swappedCombinedDeviceID[0];
	    swappedCombinedDeviceID[1] 			 = deviceIds.swappedCombinedDeviceID[1];

	}

	else
	{
		printf("6. No deviceID saved to EEPROM, waiting for pairing...\n\r");
	}

	printf("7. deviceIds.deviceID = 0x%04X\n\r", deviceIds.deviceID);
	printf("8. deviceIds.otherDeviceID = 0x%04X\n\r", deviceIds.otherDeviceID);

	free(read_buffer);

	if((temp1 != 0) && (temp1 != 0xFFFF) && (temp2 != 0) && (temp2 != 0xFFFF))
	{
		return true;
	}

	else
	{
		return false;
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

  // HAL_InitTick should be called after SystemClock_Config
  HAL_InitTick(TICK_INT_PRIORITY);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SubGHz_Phy_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  SUBGRF_SetPaConfig(0x2, 0x3, 0x0, 0x1);

  HAL_GPIO_WritePin(PAIR_LIGHT_PORT, RED_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PAIR_LIGHT_PORT, GREEN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PAIR_LIGHT_PORT, BLUE_PIN, GPIO_PIN_RESET);

  if(readIDsFromEEPROM(&hi2c3, EEPROM_PairingID1, EEPROM_PairingID2, EEPROM_Start_Address))
  {
	  printf("readIDSFromEEPROM() successful!\n\r");
	  isPairingButtonPressed = false;
	  currentPairingState = Paired;
  }

  // Configure NVIC interrupts
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);   HAL_UART_MspInit(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 3, 0);
    HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);

    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    __enable_irq();

   HAL_UART_Receive_IT(&huart1, dataReceived, sizeof(dataReceived));

   HAL_StatusTypeDef status = HAL_OK;

   status = HAL_I2C_IsDeviceReady(&hi2c1, LTC4015_I2C_ADDRESS, 2, HAL_MAX_DELAY);//FINDME!

   if(status != HAL_OK)
   {
	   printf("SMBUS configuration failed, status = %d\n\r", status);
   }

   else
   {
	   printf("SMBUS configuration succeeded!\n\r");
   }

   //////////////////////////////////////////////////////////////////////

   if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
   {
     /* Starting Error */
     Error_Handler();
   }
   ///////////////////////////////////////////////////////////////////////////
   HAL_Delay(300); // Need a 300 msec delay here for some reason

  // Enable the Coulomb Counter
  // EnableCoulombCounter();
  // Set Coulomb prescale factor
  //SetCoulombCounterPrescaleFactor(0x65);
  // Set Coulomb Counter Accumulator Value
  //SetCoulombCounterAccumulatorValue(0x0C00);

  // Example usage to calculate SoC  -------- Make interrupts for these or place in blocking while loop for testing
  //uint8_t soc = CalculateSOC();

  // Print the calculated SoC
  //printf( "\n\rState of Charge (SoC): %u%%\n\r", soc);

  readIDsFromEEPROM(&hi2c3, &EEPROM_PairingID1, &EEPROM_PairingID2, EEPROM_Start_Address);

  SUBGRF_SetPaConfig(2, 2, 0, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // HAL_Delay(3); // Need a 3 msec delay here for some reason

	  if(currentPairingState == Paired)
	  {
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, RED_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, GREEN_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, BLUE_PIN, GPIO_PIN_RESET);
			//printf("Entering MX_SubGHz_Phy_Process\n\r");
			MX_SubGHz_Phy_Process();
	   }

	   else if((currentPairingState == GenerateDeviceID) || (currentPairingState == EncodeDeviceID) || (currentPairingState == BroadcastDeviceID))
	   {
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, RED_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, GREEN_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, BLUE_PIN, GPIO_PIN_SET);
	   }

	   else if((currentPairingState == Unknown))
	   {
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, RED_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, GREEN_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, BLUE_PIN, GPIO_PIN_RESET);
	   }

	   else
	   {
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, RED_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, GREEN_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PAIR_LIGHT_PORT, BLUE_PIN, GPIO_PIN_SET);
	   }

	  GPIO_PinState buttonState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
	  if(buttonState == GPIO_PIN_RESET)
	  {

		  uint32_t startTime = HAL_GetTick();

		  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)
		  {
			  HAL_Delay(10);

			  if((HAL_GetTick() - startTime) >= 5000)
			  {
				  printf("Button held down for 3 seconds\n\r");

				  deviceIds.deviceID 					= 0;
				  deviceIds.otherDeviceID 				= 0;
				  deviceIds.combinedDeviceID[0]		    = 0;
				  deviceIds.combinedDeviceID[1]         = 0;
				  deviceIds.swappedCombinedDeviceID[0]  = 0;
				  deviceIds.swappedCombinedDeviceID[1]  = 0;

				  isAck1Received						= false;
				  isOtherDeviceIDReceived               = false;
				  currentPairingState                   = Unknown;
				  isPairingButtonPressed                = true;

				  while(isPairingButtonPressed)
				  {
					  switch(currentPairingState)
					  {

					  case Unknown:
						// 0. Enter Pairing Mode
						HAL_GPIO_WritePin(PAIR_LIGHT_PORT, RED_PIN, GPIO_PIN_SET);
						HAL_GPIO_WritePin(PAIR_LIGHT_PORT, GREEN_PIN, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(PAIR_LIGHT_PORT, BLUE_PIN, GPIO_PIN_RESET);

						isAck1Received                       = false;
						isOtherDeviceIDReceived              = false;

						deviceIds.deviceID                   = 0;
						deviceIds.otherDeviceID              = 0;
						deviceIds.combinedDeviceID[0]        = 0;
						deviceIds.combinedDeviceID[1]        = 0;
						deviceIds.swappedCombinedDeviceID[0] = 0;
						deviceIds.swappedCombinedDeviceID[1] = 0;

						printf("Current state is: %d\n\r", currentPairingState);
						currentPairingState = GenerateDeviceID;
						break;

					  case GenerateDeviceID:
						// 1. Generate a unique device identifier or key for pairing
						printf("Current state is: %d\n\r", currentPairingState);
						deviceId = Radio.Random();
						printf("randomly generated device ID is: %u\n\r", deviceId);
						currentPairingState = EncodeDeviceID;
						break;

					  case EncodeDeviceID:
						// 2. Encode the device ID
						printf("Current state is: %d\n\r", currentPairingState);
						encodeDeviceId(deviceId, payload);
						printf( "Encoded device ID is: 0x%02X%02X\n\r", payload[1], payload[0]);
						currentPairingState = BroadcastDeviceID;
						break;

					  case BroadcastDeviceID:
						// 3. Transmitter starts by transmitting deviceId x10, then listens for other deviceId x10
						printf("Current state is: %d\n\r", currentPairingState);
						broadcastDeviceId(payload);
						break;

					  case Paired:
						// 5. Device is paired, move on to PingPong_Process
						// printf("Device is Paired\n\r");
						isPairingButtonPressed = false;
						HAL_GPIO_WritePin(PAIR_LIGHT_PORT, RED_PIN, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(PAIR_LIGHT_PORT, GREEN_PIN, GPIO_PIN_SET);
						HAL_GPIO_WritePin(PAIR_LIGHT_PORT, BLUE_PIN, GPIO_PIN_RESET);
						break;

					  default:
						// error condition
						// printf("Entered the default (error) state!\n\r");
						//currentPairingState = Unknown;
						break;
					}
				 }

				 startTime = HAL_GetTick();

			  }
		  }
	  }
    /* USER CODE END WHILE */
    MX_SubGHz_Phy_Process();

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
