/**
  ******************************************************************************
  * @file    imp34dt05_app.h
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief   Header for imp34dt05_app.c module.
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
#ifndef __IMP34DT05_APP_H
#define __IMP34DT05_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h" 
#include "cmsis_os.h"
#include "sensors_manager.h"
  

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define IMP34DT05_SAMPLING_FREQUENCY            (uint32_t)(48000)
#define AMIC_MS                                 (uint32_t)(1)
#define IMP34DT05_MS                            (uint32_t)(1)

#define IMP34DT05_DFSDM_FILTER                         DFSDM1_Filter0
#define IMP34DT05_DFSDM_CHANNEL                         DFSDM1_Channel5
#define IMP34DT05_DFSDM_CLK_ENABLE()                  __HAL_RCC_DFSDM1_CLK_ENABLE()
  
#define IMP34DT05_DFSDM_CLK_PIN                         GPIO_PIN_9
#define IMP34DT05_DFSDM_CLK_GPIO_PORT                   GPIOE
#define IMP34DT05_DFSDM_CLK_PIN_CLK_ENABLE()            __HAL_RCC_GPIOE_CLK_ENABLE()
#define IMP34DT05_DFSDM_CLK_AF                          GPIO_AF6_DFSDM1
  
#define IMP34DT05_DFSDM_PDM_PIN                         GPIO_PIN_6
#define IMP34DT05_DFSDM_PDM_GPIO_PORT                   GPIOB
#define IMP34DT05_DFSDM_PDM_PIN_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define IMP34DT05_DFSDM_PDM_AF                          GPIO_AF6_DFSDM1 

#define IMP34DT05_DFSDM_RX_DMA_CHANNEL                   DMA1_Channel5
#define IMP34DT05_DFSDM_RX_DMA_REQUEST                   DMA_REQUEST_DFSDM1_FLT0
#define IMP34DT05_DFSDM_RX_DMA_IRQn                      DMA1_Channel5_IRQn
#define IMP34DT05_DFSDM_DMA_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

/* Exported macro ------------------------------------------------------------*/  
/* Exported variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_dfsdm1_flt0;
extern osThreadId IMP34DT05_Thread_Id;
extern SM_Init_Param_t IMP34DT05_Init_Param;

/* Exported functions ------------------------------------------------------- */
void IMP34DT05_Peripheral_Init(void);
void IMP34DT05_OS_Init(void);
void IMP34DT05_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp);
void IMP34DT05_Set_ODR(float newODR);
void IMP34DT05_Set_FS(float newFS1, float newFS2);
void IMP34DT05_Start(void);
void IMP34DT05_Stop(void);


#ifdef __cplusplus
}
#endif

#endif /* __IMP34DT05_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
