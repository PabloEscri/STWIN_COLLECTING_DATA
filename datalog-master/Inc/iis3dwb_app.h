/**
  ******************************************************************************
  * @file    iis3dwb_app.h
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief   Header for iis3dwb_app.c module.
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
#ifndef __IIS3DWB_APP_H
#define __IIS3DWB_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h" 
#include "cmsis_os.h"
#include "sensors_manager.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/  
#define IIS3DWB_MAX_DRDY_PERIOD           (1.0)
#define IIS3DWB_MAX_WTM_LEVEL             (256)
#define IIS3DWB_MIN_WTM_LEVEL             (16)
#define IIS3DWB_MAX_SAMPLES_PER_IT        (IIS3DWB_MAX_WTM_LEVEL)
  
#define IIS3DWB_SPI_CS_Pin GPIO_PIN_5
#define IIS3DWB_SPI_CS_GPIO_Port GPIOF
#define IIS3DWB_INT1_Pin GPIO_PIN_14
#define IIS3DWB_INT1_GPIO_Port GPIOE
#define IIS3DWB_INT1_EXTI_IRQn EXTI15_10_IRQn
  
#define FFT_LEN_AXL             (uint32_t)(256)
#define OVLP_AXL                (float)(0.25)  
#define N_AVERAGE_AXL           (int)(4)

/* Exported macro ------------------------------------------------------------*/  
/* Exported variables --------------------------------------------------------*/
extern osThreadId IIS3DWB_Thread_Id;
extern EXTI_HandleTypeDef iis3dwb_exti;
extern SM_Init_Param_t IIS3DWB_Init_Param;

/* Exported functions ------------------------------------------------------- */
void IIS3DWB_Peripheral_Init(void);
void IIS3DWB_OS_Init(void);
void IIS3DWB_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp);
void IIS3DWB_Set_ODR(float newODR);
void IIS3DWB_Set_FS(float newFS1, float newFS2);
void IIS3DWB_Start(void);
void IIS3DWB_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __IIS3DWB_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
