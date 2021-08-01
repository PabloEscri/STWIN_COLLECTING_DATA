/**
  ******************************************************************************
  * @file    lps22hh_app.h
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief   Header for lps22hh_app.c module.
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
#ifndef __LPS22HH_APP_H
#define __LPS22HH_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h" 
#include "cmsis_os.h"
#include "sensors_manager.h"
#include "lps22hh_reg.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define LPS22HH_SAMPLES_PER_TS (256)  
  
/* Exported macro ------------------------------------------------------------*/  
/* Exported variables --------------------------------------------------------*/
extern SM_Init_Param_t LPS22HH_Init_Param;
  
/* Exported functions ------------------------------------------------------- */
void LPS22HH_Peripheral_Init(void);
void LPS22HH_OS_Init(void);
void LPS22HH_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp);
void LPS22HH_Set_ODR(float newODR);
void LPS22HH_Set_FS(float newFS1, float newFS2);
void LPS22HH_Start(void);
void LPS22HH_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __LPS22HH_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
