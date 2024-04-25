/**
  ******************************************************************************
  * @file    iks01a3_conf.h
  * @author  MEMS Application Team
  * @brief   This file contains definitions for the MEMS components bus interfaces
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

#include "stm32wlxx_hal.h"
#include "stm32wlxx_nucleo_bus.h"
#include "stm32wlxx_nucleo_errno.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IKS01A3_CONF_H__
#define __IKS01A3_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN 1 */
#define USE_IKS01A3_ENV_SENSOR_HTS221_0                0U
#define USE_IKS01A3_ENV_SENSOR_LPS22HH_0               0U
#define USE_IKS01A3_ENV_SENSOR_STTS751_0               0U

#define USE_IKS01A3_MOTION_SENSOR_LSM6DSO_0            1U
#define USE_IKS01A3_MOTION_SENSOR_LIS2DW12_0           0U
#define USE_IKS01A3_MOTION_SENSOR_LIS2MDL_0            0U


void MX_I2C2_Init(void);
int32_t MX_I2C2_DeInit(void);
extern int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
extern int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t BSP_GetTick(void);
int32_t my_write_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t my_read_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

static int32_t Custom_I2C_Init(void)
{
    MX_I2C2_Init(); // Call the generated initialization function
    return 0;       // Return an appropriate status code
}


/* USER CODE END 1 */

#define IKS01A3_I2C_Init Custom_I2C_Init
#define IKS01A3_I2C_DeInit MX_I2C2_DeInit
#define IKS01A3_I2C_ReadReg my_read_reg
#define IKS01A3_I2C_WriteReg my_write_reg
#define IKS01A3_GetTick BSP_GetTick
#define IKS01A3_Delay HAL_Delay

#define BUS_IKS01A3_INSTANCE BUS_I2C1_INSTANCE
#define BUS_IKS01A3_CLK_DISABLE() __HAL_RCC_I2C1_CLK_DISABLE()
#define BUS_IKS01A3_CLK_ENABLE() __HAL_RCC_I2C1_CLK_ENABLE()
#define BUS_IKS01A3_SCL_GPIO_PORT BUS_I2C1_SCL_GPIO_PORT
#define BUS_IKS01A3_SCL_GPIO_AF BUS_I2C1_SCL_GPIO_AF
#define BUS_IKS01A3_SCL_GPIO_CLK_ENABLE() BUS_I2C1_SCL_GPIO_CLK_ENABLE()
#define BUS_IKS01A3_SCL_GPIO_CLK_DISABLE() BUS_I2C1_SCL_GPIO_CLK_DISABLE()
#define BUS_IKS01A3_SCL_GPIO_PIN BUS_I2C1_SCL_GPIO_PIN
#define BUS_IKS01A3_SDA_GPIO_PIN BUS_I2C1_SDA_GPIO_PIN
#define BUS_IKS01A3_SDA_GPIO_CLK_DISABLE() BUS_I2C1_SDA_GPIO_CLK_DISABLE()
#define BUS_IKS01A3_SDA_GPIO_PORT BUS_I2C1_SDA_GPIO_PORT
#define BUS_IKS01A3_SDA_GPIO_AF BUS_I2C1_SDA_GPIO_AF
#define BUS_IKS01A3_SDA_GPIO_CLK_ENABLE() BUS_I2C1_SDA_GPIO_CLK_ENABLE()

#ifdef __cplusplus
}
#endif

#endif /* __IKS01A3_CONF_H__*/

