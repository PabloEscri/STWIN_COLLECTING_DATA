/**
  ******************************************************************************
  * @file    sdcard_manager.h
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief	 Header for sdcard_manager.c module
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
#ifndef __SDCARD_MANAGER_H
#define __SDCARD_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cube_hal.h"
#include "cmsis_os.h"
  

#define SDM_CMD_MASK                (0x00008000)
#define SDM_DATA_READY_MASK         (0x00004000)
#define SDM_DATA_FIRST_HALF_MASK    (0x00002000)
#define SDM_DATA_SECOND_HALF_MASK   (0x00000000)
#define SDM_SENSOR_ID_MASK          (0x000000FF)
#define SDM_SUBSENSOR_ID_MASK       (0x00000700)

#define SDM_START_STOP          (0x00000001|SDM_CMD_MASK)
  
#define HSD_SD_LOGGING_MODE_CONTINUOUS          (1U)
#define HSD_SD_LOGGING_MODE_INTERMITTENT        (0U)

/* Define the duty cycle of the data logging */
#define HSD_LOGGING_TIME_SECONDS_IDLE     (5)
#define HSD_LOGGING_TIME_SECONDS_ACTIVE   (15*60 - HSD_LOGGING_TIME_SECONDS_IDLE)


#define SDM_DEFAULT_CONFIG      (uint8_t)(0x00)
#define SDM_MLC_CONFIG          (uint8_t)(0x01)  
#define SDM_JSON_CONFIG         (uint8_t)(0x10)
  

extern osMessageQId sdThreadQueue_id;

void SDM_OS_Init(void);
void SDM_Peripheral_Init(void);
void SDM_SD_Init(void);
void SDM_SD_DeInit(void);
uint8_t SDM_Memory_Init(void);

uint8_t SDM_InitFiles(void);
uint8_t SDM_CloseFiles(void);
uint8_t SDM_UpdateDeviceConfig(void);
uint8_t SDM_OpenLogErrorFile(const char *name);
uint8_t SDM_OpenDatFile(uint8_t sID, uint8_t ssID, const char *sensorName);
uint8_t SDM_CloseFile(uint8_t sID, uint8_t ssID);
uint8_t SDM_WriteBuffer(uint8_t sID, uint8_t ssID, uint8_t *buffer, uint32_t size);
uint8_t SDM_WriteConfigBuffer(uint8_t *buffer, uint32_t size);
uint8_t SDM_Flush_Buffer(uint8_t sID, uint8_t ssID);
uint8_t SDM_Fill_Buffer(uint8_t sID, uint8_t ssID, uint8_t *src, uint16_t srcSize);
void SDM_AutosaveFile(void);

uint32_t SDM_CreateJSON(char **serialized_string);
uint32_t SDM_ReadJSON(char *serialized_string);
uint32_t SDM_CreateAcquisitionJSON(char **serialized_string);

void SDM_WriteUCF(char* ucfData, uint32_t ucfSize);


#ifdef __cplusplus
}
#endif

#endif /* __SDCARD_MANAGER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
