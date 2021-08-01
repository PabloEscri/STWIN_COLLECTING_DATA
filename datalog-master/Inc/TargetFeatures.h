/**
  ******************************************************************************
  * @file    TargetFeatures.h 
  * @author  SRA
  * @version v1.0.0
  * @date    29-Jul-19
  * @brief   Specification of the HW Features for each target platform
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
#ifndef _TARGET_FEATURES_H_
#define _TARGET_FEATURES_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_exti.h"
#include "STWIN.h"
#include "STWIN_bc.h"
#include "stm32l4xx_hal_conf.h"
#include "stm32l4xx_hal_def.h"
#include "STWIN_env_sensors.h"

/* Exported defines ------------------------------------------------------- */
#define MAX_TEMP_SENSORS 2

/* BlueNRG Board Type */
#define IDB04A1 0
#define IDB05A1 1

/* Exported macros ------------------------------------------------------- */

/* Exported types ------------------------------------------------------- */
/**
 * @brief  Target type data definition
 */
typedef enum
{
  TARGET_NUCLEO,
  TARGET_BLUECOIN,
  TARGET_SENSORTILE,
  TARGET_STWIN,
  TARGETS_NUMBER
} TargetType_t;

/**
 * @brief  Target's Features data structure definition
 */
typedef struct
{
  TargetType_t BoardType;
  uint32_t SensorEmulation;
  int32_t NumTempSensors;
  int32_t HandleTempSensors[MAX_TEMP_SENSORS];
  int32_t HandlePressSensor;
  int32_t HandleHumSensor;

  int32_t HWAdvanceFeatures;
  int32_t HandleAccSensor;
  int32_t HandleGyroSensor;
  int32_t HandleMagSensor;

  uint8_t LedStatus;
  uint8_t bnrg_expansion_board;

  void *HandleGGComponent;

} TargetFeatures_t;

/* Exported variables ------------------------------------------------------- */
extern TargetFeatures_t TargetBoardFeatures;

/* Exported functions ------------------------------------------------------- */
extern void InitTargetPlatform(TargetType_t BoardType);
extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);

extern uint32_t GetPage(uint32_t Address);
extern uint32_t GetBank(uint32_t Address);

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/

