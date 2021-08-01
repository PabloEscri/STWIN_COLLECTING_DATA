/**
  ******************************************************************************
  * @file    hts221_app.h
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief   Header for hts221_app.c module.
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
#ifndef __HTS221_APP_H
#define __HTS221_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h" 
#include "cmsis_os.h"
#include "sensors_manager.h"
#include "hts221.h"
#include "hts221_reg.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define HTS221_INT_Pin                  GPIO_PIN_6
#define HTS221_INT_GPIO_Port            GPIOG
#define HTS221_INT_EXTI_IRQn            EXTI9_5_IRQn
#define HTS221_INT_EXTI_LINE            EXTI_LINE_6
#define HTS221_INT_GPIO_ADDITIONAL()    HAL_PWREx_EnableVddIO2()
#define HTS221_INT_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()

/* Exported macro ------------------------------------------------------------*/  
/* Exported variables --------------------------------------------------------*/
extern EXTI_HandleTypeDef hts221_exti;
extern osThreadId HTS221_Thread_Id;
extern EXTI_HandleTypeDef hts221_exti;
extern SM_Init_Param_t HTS221_Init_Param;

/* Exported functions ------------------------------------------------------- */
void HTS221_Peripheral_Init(void);
void HTS221_OS_Init(void);
void HTS221_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp);
void HTS221_Set_ODR(float newODR);
void HTS221_Set_FS(float newFS1, float newFS2);
void HTS221_Start(void);
void HTS221_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __HTS221_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
