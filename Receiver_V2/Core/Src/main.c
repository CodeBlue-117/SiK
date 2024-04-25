/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c         ----------------------------------------------------- PLOW BOX PCB
  *
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "app_fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "app_subghz_phy.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "sys_app.h"
#include "lsm6dso_reg.h"
#include "lsm6dso.h"
#include "stm32wlxx_hal.h"
#include "stm32wlxx_hal_gpio.h"
#include "stm32_timer.h"
#include "stm32wlxx_hal_i2c.h"
#include "iks01a3_env_sensors.h"
#include "iks01a3_motion_sensors.h"
#include "iks01a3_conf.h"
#include "radio.h"
#include "subghz_phy_app.h"
#include "stm32_seq.h"
#include "usart.h"
#include "stm32wlxx_hal_pwr.h"
#include "stm32wlxx_hal_pwr_ex.h"
#include "stm32wlxx_hal_def.h"
#include "radio_driver.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BOOT_TIME         				10
#define TICKS_BETWEEN_POLLING 			500  // Changed from 300!!!!
#define TICKS_BETWEEN_POLLING_STATE     1000
#define TICKS_BETWEEN_SYNCING 			10000
#define Shutdown_Period 				900000	// If plow is idle for 15 min (15 * 60 * 1000 = 900000 ms), shutdown
#define EEPROM_ADDR 					0xA0

#define MAX17261_WRITE_ADDR				0x6C
#define MAX17261_READ_ADDR				0x6D
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t my_write_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t my_read_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Pairing Algo Variables
PairingStates_t currentPairingState;
uint8_t pair_payload[2] 			= {0};
uint16_t deviceId 					= 0;
uint16_t otherDeviceID 				= 0;
uint16_t combinedDeviceID[2] 		= {0};
uint16_t swappedCombinedDeviceID[2] = {0};
uint32_t lastButtonPressTime 		= 0;
bool isPairingConfirmed 			= false;
bool isRepaired 					= false; // debounce for re-pairing, if unsuccessful on other device
bool isPairingButtonPressed  		= false;
GPIO_PinState buttonState;

DeviceIDS deviceIds = {.deviceID = 0, .otherDeviceID = 0, .combinedDeviceID = {0, 0}, .swappedCombinedDeviceID = {0, 0}};

// EEPROM Variables to Save Pairing IDS
uint16_t *EEPROM_PairingID1 		= NULL;
uint16_t *EEPROM_PairingID2 		= NULL;
uint16_t EEPROM_Start_Address 		= 0x0000;

// FATFS MicroSD Card Variables
static FATFS FatFs; 	//Fatfs handle
static FIL fil; 		//File handle
// static uint32_t nowSync 			= 0;
uint16_t fileNum			 		= 0;
bool sd_initialized;
bool record_enabled;
bool recording;
extern Disk_drvTypeDef  disk;

// IMU Variables
volatile bool imuDataReadingEnabled = true;
uint8_t AccelReg;
uint8_t GyroReg;
int32_t timestamp;
int16_t data_raw_acceleration[3];
int16_t data_raw_gyro[3];
int16_t acceleration_mg[3];
int16_t gyro_mdps[3];
int16_t acceleration_g[3];
int16_t gyro_dps[3];
int acceleration_g_int;
int acceleration_g_decimal;
int angular_accel_g_int;
int angular_accel_g_decimal;
static uint8_t whoamI, rst;
static bool IMU_initialized;
// static uint32_t now 				= 0;
float accelMagnitude 		 		= 0;
float gyroMagnitude 		 		= 0;
float x 					 		= 0.0;
float y						 		= 0.0;
float z						 		= 0.0;
float pitch					 		= 0.0;
float roll					 		= 0.0;
float yaw					 		= 0.0;
int32_t xThreshold			 		= 1050;
int32_t pitchThreshold		 		= 20;
uint8_t accel_timestamp_data[4] 	= {0};
uint8_t gyro_timestamp_data[4] 		= {0};
LSM6DSO_Object_t lsm6dso; 	// Initialize the LSM6DSO sensor
stmdev_ctx_t dev_ctx; 		// Initialize instance of stmdev_ctx
float sendIMUData					= 0.0;
bool sendIMUData_FLAG				= false;
float theta							= 0.0;
float theta_degrees					= 0.0;
uint32_t IMU_COUNT					= 0;

// Auto-Shutdown Variables
static UTIL_TIMER_Object_t shutdownTimer;
static UTIL_TIMER_Object_t timeSpentUP;
static UTIL_TIMER_Object_t timeSpentDOWN;
static UTIL_TIMER_Object_t timeSpentUNKNOWN;
bool timerRunning 					= false; // Define a flag to keep track of whether the timer is running
const uint32_t TimerDuration 		= 15 * 60; // 15 minutes in seconds
int stateTimeElapsedUP		 		= 0;
int stateTimeElapsedDOWN	 		= 0;
int stateTimeElapsedUNKNOWN  		= 0;
uint32_t State_Time			 		= 1000; // Use 1000ms and increment counter
uint32_t elapsedTime 				= 0; // Define a variable to keep track of the elapsed time

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void broadcastDeviceId(uint8_t* pair_payload);
void sendAck(void);
bool pairingConfirmation(void);
void shutdown_Fnctn(void *argument);
bool readIDsFromEEPROM(I2C_HandleTypeDef *hi2c1, uint16_t *id1, uint16_t *id2, uint16_t startAddress);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to allow us to use printf() statements
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

typedef enum {
    STATE_UNKNOWN = 0,
    STATE_UP      = 1,
    STATE_DOWN    = 2
  } DeviceState;

DeviceState currentState = STATE_UNKNOWN;

DeviceState getState()
{
    return currentState;
}

void setState(DeviceState *currentState, int32_t x, int32_t xThreshold) // Only use the z-axis accel to determine state of plow (as per Adam's experiment)
{

	if(x > xThreshold)
	{
		*currentState = STATE_DOWN;
		APP_LOG(TS_OFF, VLEVEL_M,"Device state set to: STATE_DOWN\n\r");
	}

	else
	{
		*currentState = STATE_UP;
		APP_LOG(TS_OFF, VLEVEL_M,"Device state set to: STATE_UP\n\r");
	}

	//  if((*currentState == STATE_DOWN || *currentState == STATE_UNKNOWN) && (x > xThreshold))
	//  {
	//    *currentState = STATE_UP;
	//    APP_LOG(TS_OFF, VLEVEL_M,"Device state set to: STATE_UP\n\r");
	//
	//  }
	//
	//  else if((*currentState == STATE_UP || *currentState == STATE_UNKNOWN) && (x < -xThreshold))
	//  {
	//    *currentState = STATE_DOWN;
	//    APP_LOG(TS_OFF, VLEVEL_M,"Device state set to: STATE_DOWN\n\r");
	//  }

	// According to Adam's research, we only need one axis to determine if the plow is up or down
	// If the plow is down, there will be more vibration and the x accelerometer values should exceed the xThreshold
	// We need to determine an appropriate threshold for the x-values to determine up or down

	//  else // error/unknown case
	//  {
	//    *currentState = STATE_UNKNOWN;
	//    APP_LOG(TS_OFF, VLEVEL_M,"Device state set to: STATE_UNKNOWN\n\r");
	//  }
}

void INIT_SD(void) // Unused
{
  APP_LOG(TS_OFF, VLEVEL_M,"Init SD...\r\n");
  FRESULT fres; //Result after operations
  fres = f_mount(&FatFs, "", 1); //1=mount now

  if ((fres == FR_NOT_READY )||(fres == FR_DISK_ERR ))
  {
	  disk.is_initialized[0] = 0;
	  APP_LOG(TS_OFF, VLEVEL_M,"f_mount error (%i)\r\n", fres);
	  return;
  }

  else if (fres != FR_OK)
  {
	  APP_LOG(TS_OFF, VLEVEL_M,"f_mount error (%i)\r\n", fres);
	  return;
  }

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;
  FATFS* getFreeFs;
  fres = f_getfree("", &free_clusters, &getFreeFs);

  if (fres != FR_OK)
  {
	  APP_LOG(TS_OFF, VLEVEL_M,"f_getfree error (%i)\r\n", fres);
	  return;
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;
  APP_LOG(TS_OFF, VLEVEL_M,"SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);
  fres = f_open(&fil, "control.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS);

  if(fres == FR_OK)
  {
    BYTE readBuf[16];
    TCHAR* rres = f_gets((TCHAR*)readBuf, 16, &fil);

    if(rres != 0)
    {
    	APP_LOG(TS_OFF, VLEVEL_M,"Read string from 'control.txt' contents: %s\r\n", readBuf);
    	sscanf((const char*)readBuf, "%hu", &fileNum);
    }

    else
    {
    	APP_LOG(TS_OFF, VLEVEL_M,"f_gets error (%i)\r\n", rres);
    }
    f_close(&fil);
  }

  else
  {
	  APP_LOG(TS_OFF, VLEVEL_M,"control file f_open error (%i)\r\n", fres);
	  return;
  }

  sd_initialized = true;
  APP_LOG(TS_OFF, VLEVEL_M,"...SD Ready\r\n");
}

void INIT_IMU(void)
{
  APP_LOG(TS_OFF, VLEVEL_M,"Init IMU...\r\n");
  /* Initialize mems driver interface */

  if(LSM6DSO_Init(&lsm6dso) != LSM6DSO_OK)
  {
	  APP_LOG(TS_OFF, VLEVEL_M,"Error!! LSM6DSO did not initialize correctly!\r\n");
  }

  else
  {
	  APP_LOG(TS_OFF, VLEVEL_M,"LSM6DSO initialized correctly!\r\n");
  }

  HAL_Delay(500);

  uint32_t retVal = 0;

  retVal = lsm6dso_device_id_get(&dev_ctx, &whoamI);
  APP_LOG(TS_OFF, VLEVEL_M,"WhoamI hexadecimal is: 0x%X\r\n", whoamI);

  if (whoamI != LSM6DSO_ID || retVal != 0)
  {
	  APP_LOG(TS_OFF, VLEVEL_M,"Error!! WHOAMI != LSM6DSO_ID OR retVal != 0!\r\n");
	  APP_LOG(TS_OFF, VLEVEL_M,"whoamI:%d, retVal= %d\r\n", whoamI, retVal);
	  return;
  }

  else
  {
	  IMU_initialized = true;
  }

  /* Restore default configuration */
  lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do{
	  lsm6dso_reset_get(&dev_ctx, &rst);
	  APP_LOG(TS_OFF, VLEVEL_M,"Waiting On IMU Reset!\r\n");
  	} while (rst);

  /* Enable Block Data Update */
  retVal = lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  if(retVal != 0)
  {
	  APP_LOG(TS_OFF, VLEVEL_M,"Error setting lsm6dso block data update set");
  }

  /* Set Output Data Rate */
  retVal = lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_1667Hz);

  if(retVal != 0)
  {
	  APP_LOG(TS_OFF, VLEVEL_M,"Error setting lsm6dso xl_data_rate set");
  }

  retVal = lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_1667Hz);

  if(retVal != 0)
  {
	  APP_LOG(TS_OFF, VLEVEL_M,"Error setting lsm6dso gy_data_rate set");
  }

  /* Set full scale */
  retVal = lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_4g);

  if(retVal != 0)
  {
	  APP_LOG(TS_OFF, VLEVEL_M,"Error setting lsm6dso xl_full_scale set");
  }

  retVal = lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_500dps);

  if(retVal != 0)
  {
	  APP_LOG(TS_OFF, VLEVEL_M,"Error setting lsm6dso gy_full_scale set");
  }

  APP_LOG(TS_OFF, VLEVEL_M,"...IMU Ready\r\n");

}

float READ_IMU_DATA(void) //////////////////////////////////////////////////////////////////////////////
{
	// record_enabled = true;
	// recording = true;

	/* Read output only if new xl value is available */
	lsm6dso_xl_flag_data_ready_get(&dev_ctx, &AccelReg);

	// APP_LOG(TS_OFF, VLEVEL_M,"lsm6dso_xl_flag_data_ready_get ACCELretVal= %d, reg=%d\r\n", ACCELretVal, AccelReg);
	if (AccelReg)
	{
	  /* Read acceleration field data */
	  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	  lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

	  acceleration_mg[0] = lsm6dso_from_fs4_to_mg(data_raw_acceleration[0]);

	  acceleration_mg[1] = lsm6dso_from_fs4_to_mg(data_raw_acceleration[1]);

	  acceleration_mg[2] = lsm6dso_from_fs4_to_mg(data_raw_acceleration[2]); // Offset first by just subtracting, may need to use offset registers to calibrate later though

	  x = acceleration_mg[0] / 1000.0f;
	  y = acceleration_mg[1] / 1000.0f;
	  z = acceleration_mg[2] / 1000.0f;

	  // Compute angle using arctangent function
	  theta = atan2(y, z);

	  // Convert angle from radians to degrees
	  theta_degrees = theta * (180.0 / M_PI);

	  return theta_degrees;
	}

	return 0.0;

}


//		      if (record_enabled && sd_initialized && IMU_initialized)
//		      {
//		        if (!recording)
//		        {
//		          char fileName [16];
//		          snprintf(fileName, 16, "out%d.csv", fileNum++);
//		          fres = f_open(&fil, fileName, FA_OPEN_APPEND | FA_WRITE | FA_OPEN_ALWAYS);
//
//		          if(fres == FR_OK)
//		          {
//		            APP_LOG(TS_OFF, VLEVEL_M,"I was able to open '%s' for writing\r\n", fileName);
//		          }
//
//		          else
//		          {
//		            APP_LOG(TS_OFF, VLEVEL_M,"f_open error (%i)\r\n", fres);
//		            sd_initialized = false;
//		            return;
//		          }
//		          recording = true;
//		        }
//
//		        //Copy in a string
//		        snprintf((char*)readBuf, 31, "ACCEL,%06.3f,%06.3f,%06.3f,\n", acceleration_mg[0]/1000.0f, acceleration_mg[1]/1000.0f, acceleration_mg[2]/1000.0f);
//		        printf("\n\rreadBuf: %s\n\r", readBuf);
//		        // APP_LOG(TS_OFF, VLEVEL_M,"\n\rreadBuf: %s\n\r", readBuf);
//
//		        UINT bytesWrote;
//		        fres = f_write(&fil, readBuf, 31, &bytesWrote);
//
//		        if(fres == FR_OK)
//		        {
//		          // APP_LOG(TS_OFF, VLEVEL_M,"Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
//		        }
//
//		        else
//		        {
//		          APP_LOG(TS_OFF, VLEVEL_M,"f_write error ACCEL (%i)\r\n");
//		          sd_initialized = false;
//		          f_close(&fil);
//		        }
//		        // f_close(&fil);
//		      }
//		      else
//		      {
//		        //APP_LOG(TS_OFF, VLEVEL_M,"NOT RECORDING! record_enabled = %d sd_initialized = %d IMU_initialized = %d\r\n", record_enabled, sd_initialized, IMU_initialized);
//		        if (recording)
//		        {
//		          recording = false;
//		          fres = f_close(&fil);
//		          if(fres == FR_OK)
//		          {
//		            fres = f_open(&fil, "control.txt", FA_WRITE | FA_OPEN_ALWAYS);
//		            if(fres == FR_OK)
//		            {
//		              UINT bytesWrote;
//		              snprintf((char*)readBuf, 3, "%d", fileNum);
//		              fres = f_write(&fil, readBuf, sizeof(fileNum), &bytesWrote);
//
//		              if(fres  == FR_OK)
//		              {
//		                APP_LOG(TS_OFF, VLEVEL_M,"Wrote %i bytes to 'control.txt'!\r\n", bytesWrote);
//		                f_close(&fil);
//		              }
//
//		              else
//		              {
//		                APP_LOG(TS_OFF, VLEVEL_M,"f_gets error (%i)\r\n", fres);
//		              }
//		            }
//		            else
//		            {
//		              APP_LOG(TS_OFF, VLEVEL_M,"control file f_open error (%i)\r\n", fres);
//		              return;
//		            }
//		          }
//		        }
//		      }
//		    }
//		    else
//		    {
//		      IMU_initialized = false;
//		      APP_LOG(TS_OFF, VLEVEL_M,"Accel Data isn't ready!\r\n");
//		    }
//
//		    ///////////////////////////////////////////////////////////////////
//		    // Read Gyro Data
//		    /* Read output only if new gyro value is available */
//		    lsm6dso_gy_flag_data_ready_get(&dev_ctx, &GyroReg);
//
//		    // APP_LOG(TS_OFF, VLEVEL_M,"lsm6dso_xl_flag_data_ready_get GYROretVal= %d, reg=%d\r\n", GYROretVal, GyroReg);
//		    if (GyroReg)
//		    {
//		      /* Read acceleration field data */
//		      memset(data_raw_gyro, 0x00, 3 * sizeof(int16_t));
//		      lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_gyro);
//
//		      // lsm6dso_read_reg(&dev_ctx, LSM6DSO_TIMESTAMP0, &gyro_timestamp_data[0], 1);
//		      // lsm6dso_read_reg(&dev_ctx, LSM6DSO_TIMESTAMP1, &gyro_timestamp_data[1], 1);
//		      // lsm6dso_read_reg(&dev_ctx, LSM6DSO_TIMESTAMP2, &gyro_timestamp_data[2], 1);
//		      // lsm6dso_read_reg(&dev_ctx, LSM6DSO_TIMESTAMP3, &gyro_timestamp_data[3], 1);
//		      // Combine timestamp registers to obtain the 32-bit timestamp value
//		      // uint32_t gyro_timestamp_value = ((uint32_t)gyro_timestamp_data[3] << 24) |
//		      // ((uint32_t)gyro_timestamp_data[2] << 16) |
//		      // ((uint32_t)gyro_timestamp_data[1] << 8) |
//		      // (uint32_t)gyro_timestamp_data[0];
//		      // HAL_Delay(1);
//			  // printf("gyro timestamp: %lu\r\n", gyro_timestamp_value);
//
//		      //////////////////////////////////////////////////////////////
//		      // X AXIS ANGULAR
//		      // APP_LOG(TS_OFF, VLEVEL_M,"RAW Gyro[0]: %d\n\r", data_raw_gyro[0]);
//		      gyro_mdps[0] = lsm6dso_from_fs500_to_mdps(data_raw_gyro[0]);
//		      gyro_mdps[0] = gyro_mdps[0] - 122; // OFFSET
//		      // APP_LOG(TS_OFF, VLEVEL_M,"Gyro[0]: %d mdps\n\r", gyro_mdps[0]);
//
//		      ///////////////////////////////////////////////////////////////
//		      // Y AXIS ANGULAR
//		      // APP_LOG(TS_OFF, VLEVEL_M,"RAW Gyro[1]: %d\n\r", data_raw_gyro[1]);
//		      gyro_mdps[1] = lsm6dso_from_fs500_to_mdps(data_raw_gyro[1]);
//		      // gyro_mdps[1] = gyro_mdps[1];
//		      // APP_LOG(TS_OFF, VLEVEL_M,"Gyro[1]: %d mdps\n\r", gyro_mdps[1]);
//
//		      ///////////////////////////////////////////////////////////////////
//		      // Z AXIS ANGULAR
//		      // APP_LOG(TS_OFF, VLEVEL_M,"RAW Gyro[2]: %d\n\r", data_raw_gyro[2]);
//		      gyro_mdps[2] = lsm6dso_from_fs500_to_mdps(data_raw_gyro[2]);
//		      gyro_mdps[2] = gyro_mdps[2] + 17; // OFFSET
//		      // APP_LOG(TS_OFF, VLEVEL_M,"Gyro[2]: %d mdps\n\r", gyro_mdps[2]);
//
//			  ////////////////////////////////////////////////////////////////////
//
//		      if (record_enabled && sd_initialized && IMU_initialized)
//		      {
//		        if (!recording)
//		        {
//		          char fileName [16];
//		          snprintf(fileName, 16, "out%d.csv", fileNum++);
//		          fres = f_open(&fil, fileName, FA_OPEN_APPEND | FA_WRITE | FA_OPEN_ALWAYS);
//		          if(fres == FR_OK)
//		          {
//		            APP_LOG(TS_OFF, VLEVEL_M,"I was able to open '%s' for writing\r\n", fileName);
//		          }
//		          else
//		          {
//		            APP_LOG(TS_OFF, VLEVEL_M,"f_open error (%i)\r\n", fres);
//		            sd_initialized = false;
//		            return;
//		          }
//		          recording = true;
//		        }
//
//		        //Copy in a string
//		        snprintf((char*)readBuf, 31, "GYRO,%06.3f,%06.3f,%06.3f,\n", gyro_mdps[0]/1000.0f, gyro_mdps[1]/1000.0f, gyro_mdps[2]/1000.0f);
//		        printf("readBuf: %s\n\r", readBuf);
//		        // APP_LOG(TS_OFF, VLEVEL_M,"\n\rreadBuf: %s\n\r", readBuf);
//
//		        UINT bytesWrote;
//		        fres = f_write(&fil, readBuf, 31, &bytesWrote);
//		        if(fres == FR_OK) {
//		          // APP_LOG(TS_OFF, VLEVEL_M,"Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
//		        } else {
//		          APP_LOG(TS_OFF, VLEVEL_M,"f_write error GYRO (%i)\r\n");
//		          sd_initialized = false;
//		          f_close(&fil);
//		        }
//		        // f_close(&fil);
//		      }
//		      else
//		      {
//		        //APP_LOG(TS_OFF, VLEVEL_M,"NOT RECORDING! record_enabled = %d sd_initialized = %d gyro_initialized = %d\r\n", record_enabled, sd_initialized, IMU_initialized);
//		        if (recording)
//		        {
//		          recording = false;
//		          fres = f_close(&fil);
//		          if(fres == FR_OK)
//		          {
//		            fres = f_open(&fil, "control.txt", FA_WRITE | FA_OPEN_ALWAYS);
//		            if(fres == FR_OK)
//		            {
//		              UINT bytesWrote;
//		              snprintf((char*)readBuf, sizeof(readBuf), "%d", (int)fileNum);
//		              fres = f_write(&fil, readBuf, sizeof(fileNum), &bytesWrote);
//		              if(fres  == FR_OK) {
//		                APP_LOG(TS_OFF, VLEVEL_M,"Wrote %i bytes to 'control.txt'!\r\n", bytesWrote);
//		                f_close(&fil);
//		              }
//		              else {
//		                APP_LOG(TS_OFF, VLEVEL_M,"f_gets error (%i)\r\n", fres);
//		              }
//		            }
//		            else
//		            {
//		              APP_LOG(TS_OFF, VLEVEL_M,"control file f_open error (%i)\r\n", fres);
//		              return;
//		            }
//		          }
//		        }
//		      }
//		   }
//		   else
//		   {
//		      IMU_initialized = false;
//		      APP_LOG(TS_OFF, VLEVEL_M,"Gyro Data isn't ready!\r\n");
//		   }

		   /////////////////////////////////////////////////////////////////////// Is this an appropriate place for accel and gyro data calculations? May need to implement getter functions for this data in main?

		   // printf("\n\rX: %f\n\r", x);
		   // printf("\n\rY: %f\n\r", y);
		   // printf("\n\rZ: %f\n\r", z);

		   // pitch = gyro_mdps[0] / 1000.0f;
		   // roll = gyro_mdps[1] / 1000.0f;
		   // yaw = gyro_mdps[2] / 1000.0f;

		   // printf("\n\rPITCH: %f\n\r", pitch);
		   // printf("\n\rROLL: %f\n\r", roll);
		   // printf("\n\rYAW: %f\n\r", yaw);

		   // accelMagnitude = sqrt(x*x + y*y + z*z);
		   // gyroMagnitude = sqrt(pitch*pitch + roll*roll + yaw*yaw);

		   // printf("accelMagnitude: %f\n\r", accelMagnitude);
		   // printf("gyroMagnitude: %f\n\r", gyroMagnitude);

		   //return;
		  //}
	 //}
//}

void shutdown_Check()
{
	// check if system has been idle for a specified amount of time and then shut down
   if(accelMagnitude <= 1020)
   {
	   if(!timerRunning)
	   {
		   // If the timer is not already running, start it
		   UTIL_TIMER_Create(&shutdownTimer, Shutdown_Period, UTIL_TIMER_ONESHOT, shutdown_Fnctn, NULL);
		   UTIL_TIMER_Start(&shutdownTimer);
		   APP_LOG(TS_OFF, VLEVEL_M," Shutdown Timer is Running!!\n\r");
		   timerRunning 		= true;
	   }
   }
   else
   {
	     // Reset the timer if accelMagnitude is above 0.5
		 UTIL_TIMER_Stop(&shutdownTimer);
		 timerRunning 		= false;
		 elapsedTime 		= 0;
		 APP_LOG(TS_OFF, VLEVEL_M," Shutdown Timer is NOT Running!!\n\r");
   }
}

// Shutdown MCU and peripherals -- Uses Standby mode, which is lowest power consumption
// Based on FPU deepsleep mode. Voltage regulator disabled, 1.2V domain powered off, PLLs and HSI Oscillator are off.
// RTC remains enabled for wakeup interrupt. Wakeup sequence is the same as system reset
void shutdown_Fnctn(void *argument)
{

	 imuDataReadingEnabled = false;

	 // Clear any pending EXTI interrupts
	 HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
	 HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);
	 HAL_NVIC_ClearPendingIRQ(EXTI2_IRQn);

	 /* disable the WAKEUP PIN */
	 HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	 HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
	 HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN3);

	 PWR->CR1 |= PWR_SR1_WUF1;
	 PWR->CR1 |= PWR_SR1_WUF2;
	 PWR->CR1 |= PWR_SR1_WUF3;

	 PWR->CR1 |= (0b100 << PWR_CR1_LPMS_Pos);

	 RTC_HandleTypeDef hrtc;

	 HAL_RTCEx_DeactivateWakeUpTimer(&hrtc); // Disable RTC Wake-Up
	 //HAL_RTCEx_DeactivateWakeUpTimer_IT(&hrtc); // Disable RTC Wake-Up Interrupt

	 // Disable WKUP1
	 PWR->CR3 &= ~PWR_CR3_EWUP1;

	 // Disable WKUP2
	 PWR->CR3 &= ~PWR_CR3_EWUP2;

	 // Disable WKUP3
	 PWR->CR3 &= ~PWR_CR3_EWUP3;

	 // Clear WUF1 flag
	 PWR->SR1 |= PWR_SR1_WUF1;

	 // Clear WUF2 flag
	 PWR->SR1 |= PWR_SR1_WUF2;

	 // Clear WUF3 flag
	 PWR->SR1 |= PWR_SR1_WUF3;

	 PWR->CR1 |= (0b100 << PWR_CR1_LPMS_Pos);

	 HAL_Delay(100);

	 /* Finally enter the shutdown mode */
	 HAL_PWREx_EnterSHUTDOWNMode();


	 if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
	 {
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);  // clear the flag

		/** Disable the WWAKEUP PIN **/
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);  // disable PA0

		/** Deactivate the RTC wakeup  **/
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	 }

	 /** Now enter the standby mode **/
	 /* Clear the WU FLAG */
	 __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	 /* clear the RTC Wake UP (WU) flag */
	 //__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

	 /* Enable the WAKEUP PIN */
	 HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

	 /* Finally enter the standby mode */
	 HAL_PWR_EnterSTANDBYMode();

	 // Reset the timer and flag
	 UTIL_TIMER_Stop(&shutdownTimer);
	 timerRunning = false;
	 elapsedTime = 0;

	 //	// Reset the timer and flag //////////////////////// Should place these at the top of the code so they run upon restart
	 //	UTIL_TIMER_Stop(&shutdownTimer);
	 //	timerRunning = false;
	 //	elapsedTime = 0;
}

void setStateTime()
{

	if(currentState == STATE_UP)
	{
		stateTimeElapsedUP += 1;  // each time counter expires we check the state were in and increment one of the counters
	}

	if(currentState == STATE_DOWN)
	{
		stateTimeElapsedDOWN += 1; // Each increment represents one second
	}

	if(currentState == STATE_UNKNOWN)
	{
		stateTimeElapsedUNKNOWN += 1;
	}
}

int getStateTimeUp()
{
	return stateTimeElapsedUP;
}

int getStateTimeDOWN()
{
	return stateTimeElapsedDOWN;
}

int getStateTimeUnknown()
{
	return stateTimeElapsedUNKNOWN;
}

// Timer function to determine how long plow is in UP, DOWN or UNKNOWN states
void Init_Timer(void)
{
	UTIL_TIMER_Create(&timeSpentUP, State_Time, UTIL_TIMER_PERIODIC, setStateTime, NULL);
	UTIL_TIMER_Create(&timeSpentDOWN, State_Time, UTIL_TIMER_PERIODIC, setStateTime, NULL);
	UTIL_TIMER_Create(&timeSpentUNKNOWN, State_Time, UTIL_TIMER_PERIODIC, setStateTime, NULL);
}

int32_t MX_I2C2_DeInit(void)  // Optional DeInit function
{
  HAL_I2C_DeInit(&hi2c2);
  return HAL_OK;
}

/* I2C Write Function for the IMU */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	//printf("handle = 0x%p, hi2c2 content: 0x%p\n", (void*)handle, (void*)&hi2c2);
	if (handle == NULL)
	{
		APP_LOG(TS_OFF, VLEVEL_M, "Error: Invalid handle pointer!\n\r");
		return -1; // Return an error code indicating invalid handle
	}

	// APP_LOG(TS_OFF, VLEVEL_M, "handle = %p, hi2c2 content: %p\n\r", (void*)lsm6dso.Ctx.handle, (void*)&hi2c2);
	//printf("handle = 0x%p, hi2c2 content: 0x%p\n\r", (void*)lsm6dso.Ctx.handle, (void*)hi2c2);

	if (handle == (void*)lsm6dso.Ctx.handle)
	{
		uint8_t dev_addr = 0xD4; // Shift left by 1 for some reason? --- Change back to 0xD4
		HAL_StatusTypeDef status = HAL_I2C_Mem_Write(handle, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 500);

		if (status == HAL_OK)
		{
			APP_LOG(TS_OFF, VLEVEL_M, "I2C Write Operation Successful!\n\r");
			return 0; // Return 0 for successful read
		}

		else
		{
			APP_LOG(TS_OFF, VLEVEL_M, "I2C Write Operation Failed!!\n\r");
			HAL_NVIC_SystemReset();
			return 1; // Return non-zero for read error
		}
	}

  APP_LOG(TS_OFF, VLEVEL_M,"platform_write FAILED!\r\n");
  return 1;
}

/* I2C Read Function for the IMU*/
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	//printf("handle = 0x%p, hi2c2 content: 0x%p\n", (void*)handle, (void*)&hi2c2);
	if (handle == NULL)
	{
		APP_LOG(TS_OFF, VLEVEL_M, "Error: Invalid handle pointer!\n\r");
		return -1; // Return an error code indicating invalid handle
	}

	// APP_LOG(TS_OFF, VLEVEL_M, "handle = %p, hi2c2 content: %p\n\r", (void*)lsm6dso.Ctx.handle, (void*)&hi2c2);
	//printf("handle = %p, lsm6dso.Ctx.handle = %p\n", (void*)handle, (void*)lsm6dso.Ctx.handle);

	//printf("handle = 0x%p, hi2c2 content: 0x%p\n\r", (void*)lsm6dso.Ctx.handle, (void*)hi2c2);
	if (handle == lsm6dso.Ctx.handle)
	{

		uint8_t dev_addr = 0xD5; // Device ID is shifted left by one? ---- change back to 0xD4
		HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 500);

		if (status == HAL_OK)
		{
			//APP_LOG(TS_OFF, VLEVEL_M, "I2C Read Operation Successful!!\n\r");
			return 0; // Return 0 for successful read
		}

		else
		{
			// Handle the error, if needed
			APP_LOG(TS_OFF, VLEVEL_M, "I2C Read Operation Failed!!\n\r");
			HAL_NVIC_SystemReset();
			return 1; // Return non-zero for read error
		}
	}

  APP_LOG(TS_OFF, VLEVEL_M,"platform_read FAILED!\r\n");
  return 1;
}

int32_t BSP_GetTick(void)
{
    return HAL_GetTick();
}

void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    for (uint8_t address = 0; address < 128; address++)
    {
        HAL_StatusTypeDef status;
        status = HAL_I2C_IsDeviceReady(hi2c, address << 1, 1, HAL_MAX_DELAY);
        if (status == HAL_OK)
        {
            APP_LOG(TS_OFF, VLEVEL_M, "I2C device found at address: 0x%02X\n", address);
        }
    }
}

// Define the wrapper function my_write_reg
int32_t my_write_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    // Cast handle to the correct type (e.g., I2C_HandleTypeDef*) and call platform_write
    return platform_write(handle, reg, bufp, len);
}

// Define the wrapper function my_read_reg
int32_t my_read_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    // Cast handle to the correct type (e.g., I2C_HandleTypeDef*) and call platform_read
    return platform_read(handle, reg, bufp, len);
}

void saveIDsToEEPROM(I2C_HandleTypeDef *hi2c1, uint16_t id1, uint16_t id2, uint16_t startAddress)
{

	printf("Save device IDS to EEPROM\n\r");

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
    HAL_StatusTypeDef receiveStatus = HAL_I2C_Mem_Write(hi2c1, EEPROM_ADDR, startAddress + 1, 1, &data_high_byte_1, 1, HAL_MAX_DELAY);

	if (receiveStatus == HAL_OK)
	{
		printf("HAL_I2C_Mem_Write to EEPROM Successful!\n\r");
	}
	else
	{
		printf("HAL_I2C_Mem_Write to EEPROM Failed.\n\r");
	}

	// Send high and low bytes of pairingId2
	receiveStatus = HAL_I2C_Mem_Write(hi2c1, EEPROM_ADDR, startAddress + 2, 1, &data_low_byte_1, 1, HAL_MAX_DELAY);

	if (receiveStatus == HAL_OK)
	{
		printf("HAL_I2C_Mem_Write to EEPROM Successful!\n\r");
	}
	else
	{
		printf("HAL_I2C_Mem_Write to EEPROM Failed.\n\r");
	}

	// Send high and low bytes of pairingId2
	receiveStatus = HAL_I2C_Mem_Write(hi2c1, EEPROM_ADDR, startAddress + 3, 1, &data_high_byte_2, 1, HAL_MAX_DELAY);

	if (receiveStatus == HAL_OK)
	{
		printf("HAL_I2C_Mem_Write to EEPROM Successful!\n\r");
	}
	else
	{
		printf("HAL_I2C_Mem_Write to EEPROM Failed.\n\r");
	}

	// Send high and low bytes of pairingId2
	receiveStatus = HAL_I2C_Mem_Write(hi2c1, EEPROM_ADDR, startAddress + 4, 1, &data_low_byte_2, 1, HAL_MAX_DELAY);

	if (receiveStatus == HAL_OK)
	{
		printf("HAL_I2C_Mem_Write to EEPROM Successful!\n\r");
	}
	else
	{
		printf("HAL_I2C_Mem_Write to EEPROM Failed.\n\r");
	}
}

bool readIDsFromEEPROM(I2C_HandleTypeDef *hi2c1, uint16_t *id1, uint16_t *id2, uint16_t startAddress)
{

	printf("Read Device IDS from EEPROM\n\r");

	// construct the read buffer and temp variables
	uint8_t * read_buffer 	= NULL;
	uint16_t temp1 			= 0;
	uint16_t temp2 			= 0;

	read_buffer = (uint8_t *)malloc(4 * sizeof(uint8_t));

    HAL_StatusTypeDef receiveStatus = HAL_I2C_Mem_Read(hi2c1, EEPROM_ADDR, startAddress + 1, 1, &read_buffer[0], 1, HAL_MAX_DELAY);
	if (receiveStatus != HAL_OK)
	{
		printf("Read Failed (ID1) - HAL Status: %u\n\r", receiveStatus);
		return false;
	}

	// printf("1. Read_Buffer[0] = 0x%02X\n\r", read_buffer[0]);

	receiveStatus = HAL_I2C_Mem_Read(hi2c1, EEPROM_ADDR, startAddress + 2, 1, &read_buffer[1], 1, HAL_MAX_DELAY);
	if (receiveStatus != HAL_OK)
	{
		printf("Read Failed (ID2) - HAL Status: %u\n\r", receiveStatus);
		return false;
	}

	// printf("2. Read_Buffer[1] = 0x%02X\n\r", read_buffer[1]);

	*id1 = (uint16_t)(read_buffer[0] << 8) | read_buffer[1];

	temp1 = *id1;

	// printf("3. Read Successful: id1: 0x%04X\n\r", *id1);
	// printf("3. Read Successful: temp1: 0x%04X\n\r", temp1);

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

    receiveStatus = HAL_I2C_Mem_Read(hi2c1, EEPROM_ADDR, startAddress + 3, 1, &read_buffer[2], 1, HAL_MAX_DELAY);
	if (receiveStatus != HAL_OK)
	{
		printf("Read Failed (ID1) - HAL Status: %u\n\r", receiveStatus);
		return false;
	}

	receiveStatus = HAL_I2C_Mem_Read(hi2c1, EEPROM_ADDR, startAddress + 4, 1, &read_buffer[3], 1, HAL_MAX_DELAY);
	if (receiveStatus != HAL_OK)
	{
		printf("Read Failed (ID2) - HAL Status: %u\n\r", receiveStatus);
		return false;
	}

	*id2 = (uint16_t)(read_buffer[2] << 8) | read_buffer[3];

	temp2 = *id2;

	// printf("5. Read Successful: id1: 0x%04X, id2: 0x%04X\n\r", *id1, *id2);
	// printf("5. Read Successful: temp2: 0x%04X\n\r", temp2);

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

// Function to read a 16-bit register
uint16_t ReadRegister(uint8_t regAddress)
{
    uint8_t regData[2]; // Buffer to store the read data
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MAX17261_WRITE_ADDR, regAddress, 1, regData, 2, HAL_MAX_DELAY);
    if (status == HAL_OK)
    {
    	printf("Value of register %02X: 0x%02X%02X\n\r", regAddress, regData[0], regData[1]);
        // Combine the two bytes into a 16-bit value
        return (regData[1] << 8) | regData[0];  //////////// swap 0 and 1 indexes!!!!!!
    }
    else
    {
    	printf("Failed to read value of register %02X, return value = %d\n\r", regAddress, status);
        return 0xFFFF; // Placeholder value to indicate error
    }
}

// Function to write a 16-bit value to a register
void WriteRegister(uint8_t regAddress, uint16_t data)
{
    uint8_t regData[2]; // Buffer to store the data to be written
    regData[0] = data & 0xFF; // Lower byte
    regData[1] = (data >> 8) & 0xFF; // Upper byte

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, MAX17261_WRITE_ADDR, regAddress, 1, regData, 2, HAL_MAX_DELAY);
    if (status == HAL_OK)
   {
	printf("Write data: %04X to register %02X Success!!!\n\r", data, regAddress);
   }
   else
   {
	   printf("Write data: %04X to register %02X FAILED\n\r", data, regAddress);
   }
}

// Function to wait for a certain amount of time (example implementation)
void Wait(uint16_t milliseconds)
{
	HAL_Delay(milliseconds);
}

// Function to write a 16-bit value to a register and verify
void WriteAndVerifyRegister(uint8_t regAddress, uint16_t data)
{
    // Write the data to the register
    WriteRegister(regAddress, data);

    // Read back the data from the register
    uint16_t readData = ReadRegister(regAddress);

    // Compare the written data with the read data
    if (readData == data)
    {
        printf("Write and verify data: %04X to register %02X successful!\n\r", data, regAddress);
    }
    else
    {
        printf("Write and verify data: %04X to register %02X failed! Read data: %04X\n\r", data, regAddress, readData);
    }
}

void MAX17261_Init(void)
{
    printf("Entered the MAX17261_Init function!\n\r");

    uint8_t reg3a_data[2] = {0x82, 0x4B}; // Data to be written to register 3ah for configuration
	uint8_t reg18_data[2] = {0xC3, 0x50};
	uint8_t reg1E_data[2] = {0x06, 0x40};

	/////////////////////////////////////////////////////////////////////////////////

	// Step 0:
	uint16_t StatusPOR = ReadRegister(0x00) & 0x0002;
	if(StatusPOR==0)
	{
		goto Step3_2;
	}

	// Step 1: Delay until FSTAT.DNR bit == 0
	while (ReadRegister(0x3D) & 1)
	{
		Wait(10);
	}

	// Step 2: Initialize Configuration

	uint16_t HibCFG = ReadRegister(0xBA); // Store original value // Write this value back into register 0xBA after 2nd while loop
	printf("Original HibCFG value: 0x%04X\n\r", HibCFG); // Print the original HibCFG value

	WriteRegister(0x60, 0x90); // Exit Hibernate Mode step 1
	WriteRegister(0xBA, 0x00); // Exit Hibernate Mode step 2
	WriteRegister(0x60, 0x00); // Exit Hibernate Mode step 3

	// Step 2.1: OPTION 1 EZ Config (No INI file is needed)
	WriteRegister(0x18, (reg18_data[1] << 8) | reg18_data[0]); // Write DesignCap
	WriteRegister(0x1E, (reg1E_data[1] << 8) | reg1E_data[0]); // Write IchgTerm
	WriteRegister(0x3A, (reg3a_data[1] << 8) | reg3a_data[0]); // Write VEmpty

	WriteRegister(0xDB, 0x8064); // Write 0x8064 to register 0xDB -- Write ModelCFG

	// Poll ModelCFG.Refresh(highest bit), proceed to Step 3 when ModelCFG.Refresh=0
	while (ReadRegister(0xDB) & 0x8000)
	{
		Wait(10);
	}

	WriteRegister(0xBA, HibCFG);

	// Step 3: Initialization Complete
	// Read the current status from register 0x00
	uint16_t status = ReadRegister(0x00);

	// Clear the POR bit (bit 1) by ANDing with 0xFFFD
	status &= 0xFFFD;

	// Write back the modified status to register 0x00
	WriteAndVerifyRegister(0x00, status);

	goto Step3_2;


Step3_2:
		uint16_t RepCap = ReadRegister(0x05); // Read RepCap
		uint16_t RepSOC = ReadRegister(0x06); // Read RepSOC

		// RepSOC has a LSB of 1/256%. Round the RepSOC to the nearest integer value
		float RepSOC_percentage = (float)RepSOC / 256.0; // Calculate RepSOC percentage
		int RepSOC_rounded = (int)(RepSOC_percentage + 0.5); // Round RepSOC to the nearest integer

		printf("RepCap: %u mAh\n\r", RepCap);
		printf("RepSOC: %d%%\n\r", RepSOC_rounded);


	printf("MAX17261_Init COMPLETE!\n\r");


}


uint16_t MAX17261_ReadCharge(void)
{
	uint8_t charge_data[2]; // data buffer to store the read charge value

	// Read SOC data from register 0x06
	HAL_StatusTypeDef MAX17261_Status = HAL_I2C_Mem_Read(&hi2c1, MAX17261_READ_ADDR, 0x06, 1, charge_data, 2, HAL_MAX_DELAY);
	if(MAX17261_Status == HAL_OK)
	{
		// Combine the two bytes into a 16-bit unsigned integer
		uint16_t charge = (charge_data[1] << 8) | charge_data[0];
		printf("MAX17261_ReadCharge: %04X\n\r", charge);
		return charge;
	}
	else
	{
		// Handle error
		printf("MAX17261 Read SOC FAILED, return value = %d\n\r", MAX17261_Status);
		return 0; // Return 0 as a default value or for error indication
	}

	//	uint16_t RepCap = ReadRegister(0x05); // Read RepCap
	//	uint16_t RepSOC = ReadRegister(0x06); // Read RepSOC
	//
	//	// RepSOC has a LSB of 1/256%. Round the RepSOC to the nearest integer value
	//	float RepSOC_percentage = (float)RepSOC / 256.0; // Calculate RepSOC percentage
	//	int RepSOC_rounded = (int)(RepSOC_percentage + 0.5); // Round RepSOC to the nearest integer
	//
	//	printf("RepCap: %u mAh\n\r", RepCap);
	//	printf("RepSOC: %d%%\n\r", RepSOC_rounded);



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

   lsm6dso.IO.ReadReg 		= my_read_reg;
   lsm6dso.IO.WriteReg 		= my_write_reg;
   lsm6dso.handle 			= &hi2c2;
   lsm6dso.Ctx.my_read_reg 	= my_read_reg;
   lsm6dso.Ctx.my_write_reg = my_write_reg;
   lsm6dso.Ctx.handle 		= &hi2c2;
   lsm6dso.Ctx.mdelay 		= (stmdev_mdelay_ptr)HAL_Delay;
   LSM6DSO_RegisterBusIO(&lsm6dso, &lsm6dso.IO);

   // Initialize dev_ctx params for I2C2
   dev_ctx.my_write_reg 	= my_write_reg;					// Assign custom write function
   dev_ctx.my_read_reg 		= my_read_reg; 					// Assign custom read function
   dev_ctx.mdelay 			= (stmdev_mdelay_ptr)HAL_Delay; // Set mdelay to point to HAL_Delay
   dev_ctx.handle 			= &hi2c2;

   lsm6dso.Ctx 				= dev_ctx;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_SubGHz_Phy_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(2000); //a short delay is important to let the SD card settle
  // INIT_SD();
  INIT_IMU();
  Init_Timer();

  SUBGRF_SetPaConfig(0x2, 0x3, 0x0, 0x1);

  // Need TIM16 to use HAL_GetTick()?
  HAL_NVIC_SetPriority(TIM16_IRQn, 3, 0);  // Timer16 should be configured to trigger 2-20 ktimes per second to record READ_IMU datta to SD card (right now it is triggering in main while loop)
  HAL_NVIC_EnableIRQ(TIM16_IRQn);

  // HAL_Delay(100);

  if(HAL_TIM_Base_Start_IT(&htim16) != HAL_OK)
  {
  	  APP_LOG(TS_OFF, VLEVEL_M, "Error Configuring TIM16 Interrupt!!!\n\r\n\r");/////////////////////////////////////////////////////////////////////////////////
	  Error_Handler();
  }

  //  HAL_Delay(100);

  uint32_t retVal;

  if ((retVal = IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO | MOTION_GYRO)) != BSP_ERROR_NONE)
  {
	  printf("the error was: %lu \r\n",retVal);
  	  APP_LOG(TS_OFF, VLEVEL_M, "Error: IKS01A3/LSM6DSO Initialization failed!\n");/////////////////////////////////////////////////////////////////////////////////
      Error_Handler(); // Handle the error if sensor initialization fails
  }

  else
  {
  	  APP_LOG(TS_OFF, VLEVEL_M, "IKS01A3_MOTION_SENSOR_Init Successful!\n");
  }

  // HAL_Delay(100);

  // Enable the LSM6DSO accelerometer and gyroscope
  if (IKS01A3_MOTION_SENSOR_Enable(IKS01A3_LSM6DSO_0, MOTION_ACCELERO | MOTION_GYRO) != BSP_ERROR_NONE)
  {
  	  APP_LOG(TS_OFF, VLEVEL_M, "Error: LSM6DSO Enable failed!\n");
      Error_Handler(); // Handle the error if enabling the sensors fails
  }


  //  // Accel and Gyro Timestamp Config
  //  lsm6dso_timestamp_set(&dev_ctx, PROPERTY_ENABLE);
  //  uint8_t timestamp_enabled = 1;
  //  int32_t timestamp;
  //  timestamp = lsm6dso_timestamp_get(&dev_ctx, &timestamp_enabled);

  if(readIDsFromEEPROM(&hi2c1, EEPROM_PairingID1, EEPROM_PairingID2, EEPROM_Start_Address))
  {
	  printf("ReadIDSFromEEPROM() successful!\n\r");
	  isPairingButtonPressed = false;
	  currentPairingState = Paired;
  }

  HAL_GPIO_WritePin(PAIR_LIGHT_PORT_2, GREEN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PAIR_LIGHT_PORT_1, RED_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BUTTON_PORT, BLUE_PIN, GPIO_PIN_RESET);

	//  struct dataBitField myDataBitField;
	//
	//  myDataBitField.LRM_Select_Bit 				= false;
	//  myDataBitField.Heartbeat_Bit 				= false;
	//  myDataBitField.Plow_Angle_State_Bit 		= false;
	//  myDataBitField.Plow_Box_Battery_SOC_Bit 	= false;


  ////////////////////////////////////////////////////////////// Function to configure the fuel gauge
  HAL_Delay(100);
  MAX17261_Init();
  HAL_Delay(100);
  //////////////////////////////////////////////////////////////
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // HAL_Delay(3);

	  if(currentPairingState == Paired)
	  {

		 HAL_GPIO_WritePin(PAIR_LIGHT_PORT_2, GREEN_PIN, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(PAIR_LIGHT_PORT_1, RED_PIN, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(BUTTON_PORT, BLUE_PIN, GPIO_PIN_RESET);
		 MX_SubGHz_Phy_Process();

		//		 uint16_t chargeLevel = MAX17261_ReadCharge();
		//		 printf("ChargeLevel = %04X\n\r", chargeLevel);
	  }

	  else if((currentPairingState == GenerateDeviceID) || (currentPairingState == EncodeDeviceID) ||(currentPairingState == BroadcastDeviceID))
	  {
		  HAL_GPIO_WritePin(PAIR_LIGHT_PORT_2, GREEN_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(PAIR_LIGHT_PORT_1, RED_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(BUTTON_PORT, BLUE_PIN, GPIO_PIN_SET);
	  }

	  else if(currentPairingState == Unknown)
	  {
		  HAL_GPIO_WritePin(PAIR_LIGHT_PORT_2, GREEN_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(PAIR_LIGHT_PORT_1, RED_PIN, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(BUTTON_PORT, BLUE_PIN, GPIO_PIN_RESET);
	  }

	  else
	  {
		  HAL_GPIO_WritePin(PAIR_LIGHT_PORT_2, GREEN_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(PAIR_LIGHT_PORT_1, RED_PIN, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(BUTTON_PORT, BLUE_PIN, GPIO_PIN_RESET);
	  }

	  // Read the state of the pairing button
	  buttonState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
	  if(buttonState == GPIO_PIN_RESET) // button is pressed (low voltage)
	  {
		  deviceIds.deviceID = 0;
		  deviceIds.otherDeviceID = 0;
		  deviceIds.combinedDeviceID[0] = 0;
		  deviceIds.combinedDeviceID[1] = 0;
		  deviceIds.swappedCombinedDeviceID[0] = 0;
		  deviceIds.swappedCombinedDeviceID[1] = 0;

		  isOtherDeviceIDReceived = false;
		  isAck1Received = false;

		  currentPairingState = Unknown;

	  	  isPairingButtonPressed = true;

	  	  while(isPairingButtonPressed)
	  	  {

			  switch(currentPairingState)
			  {

				 case Unknown:
					 // 0. Enter Pairing Mode
					 isOtherDeviceIDReceived = false;
					 isAck1Received = false;
					 deviceIds.deviceID = 0;
					 deviceIds.otherDeviceID = 0;
					 deviceIds.combinedDeviceID[0] = 0;
					 deviceIds.combinedDeviceID[1] = 0;
					 deviceIds.swappedCombinedDeviceID[0] = 0;
					 deviceIds.swappedCombinedDeviceID[1] = 0;
					 HAL_GPIO_WritePin(PAIR_LIGHT_PORT_2, GREEN_PIN, GPIO_PIN_RESET);
				     HAL_GPIO_WritePin(PAIR_LIGHT_PORT_1, RED_PIN, GPIO_PIN_SET);
				     HAL_GPIO_WritePin(BUTTON_PORT, BLUE_PIN, GPIO_PIN_RESET);
					 APP_LOG(TS_OFF, VLEVEL_M, "Current state is: %d\n\r", currentPairingState);
					 currentPairingState = GenerateDeviceID;
					 break;

				 case GenerateDeviceID:
					 // 1. Generate a unique device identifier or key for pairing
					 APP_LOG(TS_OFF, VLEVEL_M, "Current state is: %d\n\r", currentPairingState);
					 deviceId = Radio.Random();
					 APP_LOG(TS_OFF, VLEVEL_M, "Randomly generated device ID is: %u\n\r", deviceId);
					 currentPairingState = EncodeDeviceID;
					 break;

				 case EncodeDeviceID:
					 // 2. Encode the device ID
					 APP_LOG(TS_OFF, VLEVEL_M, "Current state is: %d\n\r", currentPairingState);
					 encodeDeviceId(deviceId, pair_payload);
					 APP_LOG(TS_OFF, VLEVEL_M, "Encoded Device ID is: 0x%02X%02X\n\r", pair_payload[1], pair_payload[0]);
					 currentPairingState = BroadcastDeviceID;
					 break;

				  case BroadcastDeviceID:
					 // 3. Receiver starts by listening for otherDeviceId x10, then transmits it's deviceId x10
					 APP_LOG(TS_OFF, VLEVEL_M, "Current state is: %d\n\r", currentPairingState);
					 APP_LOG(TS_OFF, VLEVEL_M,"This device ID is (struct): 0x%04X\n\r", deviceIds.deviceID);
					 APP_LOG(TS_OFF, VLEVEL_M,"Other Device ID is (Struct): 0x%04X\n\r", deviceIds.otherDeviceID);
					 broadcastDeviceId(pair_payload);
					 // HAL_Delay(100);
					 break;

				  case Paired:
					 // 5.
					 APP_LOG(TS_OFF, VLEVEL_M, "Device is Paired\n\r");
					 isPairingButtonPressed = false;
					 break;

				  default:
					// error condition
					APP_LOG(TS_OFF, VLEVEL_M, "Entered the error state!\n\r");
					//currentPairingState = Unknown;
					break;
				}
		   }
	  }




//	  if(sd_initialized)
//	  {
//		  if ((HAL_GetTick() - nowSync) > TICKS_BETWEEN_SYNCING)
//		  {
//			  f_sync(&fil);
//			  nowSync = HAL_GetTick();
//		  }
//	   }

	 // MX_SubGHz_Phy_Process();
  	 }

    /* USER CODE END WHILE */
    // MX_SubGHz_Phy_Process();

    /* USER CODE BEGIN 3 */

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
