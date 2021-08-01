/**
  ******************************************************************************
  * @file    sensors_manager.h
  * @author  SRA
  * @version v1.0.0
  * @date    12-Sep-19
  * @brief   Header for sensors_manager.c module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_COMM_MANAGER_H
#define __BLE_COMM_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h" 
#include "cmsis_os.h"


#define BLE_CM_SPI_x                                SPI2
#define BLE_CM_SPIx_CLK_ENABLE()                    __HAL_RCC_SPI2_CLK_ENABLE()

#define BLE_CM_SPI_CLK_PIN                          GPIO_PIN_1
#define BLE_CM_SPI_CLK_GPIO_PORT                    GPIOD
#define BLE_CM_SPI_CLK_PIN_CLK_ENABLE()             __HAL_RCC_GPIOD_CLK_ENABLE()
#define BLE_CM_SPI_CLK_AF                           GPIO_AF5_SPI2

#define BLE_CM_SPI_MISO_PIN                         GPIO_PIN_3
#define BLE_CM_SPI_MISO_GPIO_PORT                   GPIOD
#define BLE_CM_SPI_MISO_PIN_CLK_ENABLE()            __HAL_RCC_GPIOD_CLK_ENABLE()
#define BLE_CM_SPI_MISO_AF                          GPIO_AF5_SPI2

#define BLE_CM_SPI_MOSI_PIN                         GPIO_PIN_3
#define BLE_CM_SPI_MOSI_GPIO_PORT                   GPIOC
#define BLE_CM_SPI_MOSI_PIN_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define BLE_CM_SPI_MOSI_AF                          GPIO_AF5_SPI2

#define BLE_CM_SPI_EXTI_PORT                        GPIOG
#define BLE_CM_SPI_EXTI_PIN                         GPIO_PIN_1
#define BLE_CM_SPI_EXTI_IRQn                        EXTI1_IRQn

#define BLE_CM_SPI_CS_PORT                          GPIOG
#define BLE_CM_SPI_CS_PIN                           GPIO_PIN_5

#define BLE_CM_RST_PORT                             GPIOA
#define BLE_CM_RST_PIN                              GPIO_PIN_8




int32_t BLE_CM_SPI_SendReceive(void * handle, uint8_t reg, uint8_t * data, uint16_t len);
void BLE_CM_SPI_Init(void);
void BLE_CM_SPI_DeInit(void);
void BLE_CM_SPI_Reset(void);
void BLE_CM_SPI_Enable_IRQ(void);
void BLE_CM_SPI_Disable_IRQ(void);
void BLE_CM_OS_Init(void);

/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BLE_CM_SPI_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_COMM_MANAGER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
