/**
  ******************************************************************************
  * @file    device_description.c
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief   This file provides a set of functions to handle the COM structure
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

/* Includes ------------------------------------------------------------------*/
#include "device_description.h"
#include "main.h"

#include "mp23abs1_app.h"
#include "imp34dt05_app.h"
#include "ism330dhcx_app.h"
#include "iis3dwb_app.h"
#include "iis2mdc_app.h"
#include "iis2dh_app.h"
#include "hts221_app.h"
#include "lps22hh_app.h"
#include "stts751_app.h"

#include "sensors_manager.h"
#include "ble_config_service.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* 
 * Default value for SD Buffer Size for each SubSensor
 * The allocated amount is the following value multiplied by 2 (double buffer)
 */
#define WRITE_BUFFER_SIZE_IIS3DWB        (uint32_t)(37000)
#define WRITE_BUFFER_SIZE_ISM330DHCX_A   (uint32_t)(12000)
#define WRITE_BUFFER_SIZE_ISM330DHCX_G   (uint32_t)(13000)
#define WRITE_BUFFER_SIZE_ISM330DHCX_MLC (uint32_t)(100)
#define WRITE_BUFFER_SIZE_IIS2DH         (uint32_t)(8000)
#define WRITE_BUFFER_SIZE_IIS2MDC        (uint32_t)(2000)
 
#define WRITE_BUFFER_SIZE_LPS22HH_P      (uint32_t)(4000)
#define WRITE_BUFFER_SIZE_LPS22HH_T      (uint32_t)(4000)
#define WRITE_BUFFER_SIZE_HTS221_H       (uint32_t)(250)
#define WRITE_BUFFER_SIZE_HTS221_T       (uint32_t)(250)
#define WRITE_BUFFER_SIZE_STTS751        (uint32_t)(100)

#define WRITE_BUFFER_SIZE_IMP34DT05      (uint32_t)(30000)
#define WRITE_BUFFER_SIZE_MP23ABS1       (uint32_t)(104000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Unique ID is directly derived from STM32 UID and converted to string
   string needs to be 25bytes 24+\0  */
static void get_unique_id(char *id)
{ 
  uint32_t stm32_UID[3];

  stm32_UID[0] = HAL_GetUIDw0();
  stm32_UID[1] = HAL_GetUIDw1();
  stm32_UID[2] = HAL_GetUIDw2();
  
  sprintf(id, "%08lX%08lX%08lX", stm32_UID[0], stm32_UID[1], stm32_UID[2]);
}

/**
* @brief Set default device description
* @param None
* @retval None
*/
void set_default_description(void)
{  
  COM_Device_t *pDevice;
  COM_DeviceDescriptor_t * pDeviceDescriptor;
  COM_Sensor_t * pSensor;

  pDevice = COM_GetDevice();
  char tmp[6] =  {HSD_JSON_VERSION_MAJOR,'.',HSD_JSON_VERSION_MINOR,'.',HSD_JSON_VERSION_PATCH,'\0'};
  strcpy(pDevice->JSONVersion, tmp);
  
  pDeviceDescriptor = COM_GetDeviceDescriptor();
  get_unique_id(pDeviceDescriptor->serialNumber);
  strcpy(pDeviceDescriptor->alias, "STWIN_001");
  strcpy(pDeviceDescriptor->partNumber, "STEVAL-STWINKT1"); 
  strcpy(pDeviceDescriptor->URL, "www.st.com/stwin"); 
  strcpy(pDeviceDescriptor->fwName, "HSDatalog");
  char tmp1[6] = {HSD_VERSION_MAJOR,'.',HSD_VERSION_MINOR,'.',HSD_VERSION_PATCH,'\0'};
  strcpy(pDeviceDescriptor->fwVersion, tmp1);
  strcpy(pDeviceDescriptor->dataFileExt, HSD_DATA_FILE_EXTENSION);
  strcpy(pDeviceDescriptor->dataFileFormat, HSD_DATA_FILE_FORMAT);

  /***** IIS3DWB *****/
  iis3dwb_com_id = COM_AddSensor();
  
  pSensor = COM_GetSensor(iis3dwb_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "IIS3DWB");
  pSensor->sensorDescriptor.nSubSensors = 1;
  
 
  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 26667.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = COM_END_OF_LIST_FLOAT;  /* Terminate list */
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[2], "z");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "mg");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 2.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = 4.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[2] = 8.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[3] = 16.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[4] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  pSensor->sensorStatus.subSensorStatus[0].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[0].FS = 16.0f;
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.0305f *  pSensor->sensorStatus.subSensorStatus[0].FS;
  pSensor->sensorStatus.subSensorStatus[0].ODR = 26667.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 3000;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_IIS3DWB;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  
  IIS3DWB_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS3DWB_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS3DWB_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  
  /*****                                                                 *****/ 
  
  /***** HTS221 *****/
  hts221_com_id = COM_AddSensor();
  pSensor = COM_GetSensor(hts221_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "HTS221");
  pSensor->sensorDescriptor.nSubSensors = 2;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_TEMP;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "tem");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 7.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 12.5f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Celsius");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 120.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  pSensor->sensorStatus.subSensorStatus[0].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[0].FS = 120.0f;
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[0].ODR = 12.5f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 50;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 16;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_HTS221_T;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  
    /* SUBSENSOR 1 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[1].id = 1;
  pSensor->sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_HUM;
  pSensor->sensorDescriptor.subSensorDescriptor[1].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[0], "hum");
  pSensor->sensorDescriptor.subSensorDescriptor[1].dataType = DATA_TYPE_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[1] = 7.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[2] = 12.5f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[3] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].unit, "%");
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[0] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 1 STATUS */
  pSensor->sensorStatus.subSensorStatus[1].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[1].FS = 100.0f;
  pSensor->sensorStatus.subSensorStatus[1].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[1].ODR = 12.5f;
  pSensor->sensorStatus.subSensorStatus[1].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].samplesPerTimestamp = 50;
  pSensor->sensorStatus.subSensorStatus[1].usbDataPacketSize = 16;
  pSensor->sensorStatus.subSensorStatus[1].sdWriteBufferSize = WRITE_BUFFER_SIZE_HTS221_H;
  pSensor->sensorStatus.subSensorStatus[1].comChannelNumber = -1;
  
  HTS221_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  HTS221_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  HTS221_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  HTS221_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  HTS221_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;
  
  /*****                                                                  *****/ 
    
  /***** IIS2DH *****/
  iis2dh_com_id = COM_AddSensor();
  pSensor = COM_GetSensor(iis2dh_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "IIS2DH");
  pSensor->sensorDescriptor.nSubSensors = 1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[2], "z");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 10.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 25.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = 50.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[4] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[5] = 200.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[6] = 400.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[7] = 1344.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[8] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "mg");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 2.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = 4.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[2] = 8.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[3] = 16.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[4] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  pSensor->sensorStatus.subSensorStatus[0].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[0].FS = 16.0f;
  if (pSensor->sensorStatus.subSensorStatus[0].FS == 16.0f)
  {
    pSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.75f;
  }
  else
  {
    pSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.03125f *  pSensor->sensorStatus.subSensorStatus[0].FS;
  }
  pSensor->sensorStatus.subSensorStatus[0].ODR = 1344.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 2400;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_IIS2DH;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  
  IIS2DH_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS2DH_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2DH_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  
  /**********/ 
  
  /* IIS2MDC */
  iis2mdc_com_id = COM_AddSensor();  
  pSensor = COM_GetSensor(iis2mdc_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "IIS2MDC");
  pSensor->sensorDescriptor.nSubSensors = 1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_MAG;
  //tempSensor->sensorDescriptor.subSensorDescriptor[0].dataPerSample = 3;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[2], "z");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 10.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 20.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 50.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[4] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "gauss");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 50.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  pSensor->sensorStatus.subSensorStatus[0].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[0].FS = 50.0f;
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.5;
  pSensor->sensorStatus.subSensorStatus[0].ODR = 100.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 100;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 600;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_IIS2MDC;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  
  IIS2MDC_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS2MDC_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2MDC_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  
  /**********/ 
  
  /* IMP34DT05 */
  imp34dt05_com_id = COM_AddSensor();
   
  pSensor = COM_GetSensor(imp34dt05_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "IMP34DT05");
  pSensor->sensorDescriptor.nSubSensors = 1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_MIC;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "aud");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 48000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Waveform");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 122.5f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  pSensor->sensorStatus.subSensorStatus[0].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[0].FS = 122.5f;
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0;
  pSensor->sensorStatus.subSensorStatus[0].ODR = 48000.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 4096;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_IMP34DT05;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  
  IMP34DT05_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IMP34DT05_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IMP34DT05_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  
/**********/ 
  
  /* ISM330DHCX */
  ism330dhcx_com_id = COM_AddSensor();
  
  pSensor = COM_GetSensor(ism330dhcx_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "ISM330DHCX");
  pSensor->sensorDescriptor.nSubSensors = 3;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[2], "z");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 12.5f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 26.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 52.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = 104.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[4] = 208.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[5] = 417.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[6] = 833.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[7] = 1667.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[8] = 3333.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[9] = 6667.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[10] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "mg");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 2.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = 4.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[2] = 8.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[3] = 16.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[4] = COM_LIST_SEPARATOR_FLOAT;
  
  /* SUBSENSOR 0 STATUS */ 
  pSensor->sensorStatus.subSensorStatus[0].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[0].FS = 16.0f;
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.0305f * pSensor->sensorStatus.subSensorStatus[0].FS;
  pSensor->sensorStatus.subSensorStatus[0].ODR = 6667.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 2048;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_ISM330DHCX_A;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  
    /* SUBSENSOR 1 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[1].id = 1;
  pSensor->sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_GYRO;
  pSensor->sensorDescriptor.subSensorDescriptor[1].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[2], "z");
  pSensor->sensorDescriptor.subSensorDescriptor[1].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[0] = 12.5f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[1] = 26.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[2] = 52.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[3] = 104.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[4] = 208.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[5] = 417.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[6] = 833.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[7] = 1667.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[8] = 3333.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[9] = 6667.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[10] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].unit, "mdps");
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[0] = 125.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[1] = 250.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[2] = 500.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[3] = 1000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[4] = 2000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[5] = 4000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[6] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 1 STATUS */
  pSensor->sensorStatus.subSensorStatus[1].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[1].FS = 4000.0f;
  pSensor->sensorStatus.subSensorStatus[1].sensitivity = 0.035f * pSensor->sensorStatus.subSensorStatus[1].FS;
  pSensor->sensorStatus.subSensorStatus[1].ODR = 6667.0f;
  pSensor->sensorStatus.subSensorStatus[1].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[1].usbDataPacketSize = 2048;
  pSensor->sensorStatus.subSensorStatus[1].sdWriteBufferSize = WRITE_BUFFER_SIZE_ISM330DHCX_G;
  pSensor->sensorStatus.subSensorStatus[1].comChannelNumber = -1;
  
    /* SUBSENSOR 2 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[2].id = 2;
  pSensor->sensorDescriptor.subSensorDescriptor[2].sensorType = COM_TYPE_MLC; 
  pSensor->sensorDescriptor.subSensorDescriptor[2].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[0], "MLC");
  pSensor->sensorDescriptor.subSensorDescriptor[2].dataType = DATA_TYPE_INT8;
  pSensor->sensorDescriptor.subSensorDescriptor[2].ODR[0] = 12.5f;
  pSensor->sensorDescriptor.subSensorDescriptor[2].ODR[1] = 26.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[2].ODR[2] = 52.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[2].ODR[3] = 104.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[2].ODR[4] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[2].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[2].samplesPerTimestamp[1] = 1000; 
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[2].unit, "mdps");
  
  /* SUBSENSOR 2 STATUS */
  pSensor->sensorStatus.subSensorStatus[2].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[2].FS = 0.0f;
  pSensor->sensorStatus.subSensorStatus[2].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[2].ODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[2].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[2].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[2].samplesPerTimestamp = 1;
  pSensor->sensorStatus.subSensorStatus[2].usbDataPacketSize = 9;
  pSensor->sensorStatus.subSensorStatus[2].sdWriteBufferSize = WRITE_BUFFER_SIZE_ISM330DHCX_MLC;
  pSensor->sensorStatus.subSensorStatus[2].comChannelNumber = -1;  
  
  ISM330DHCX_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  ISM330DHCX_Init_Param.ODR[1] = pSensor->sensorStatus.subSensorStatus[1].ODR;
  ISM330DHCX_Init_Param.ODR[2] = pSensor->sensorStatus.subSensorStatus[2].ODR;
  ISM330DHCX_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  ISM330DHCX_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  ISM330DHCX_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  ISM330DHCX_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;
  ISM330DHCX_Init_Param.subSensorActive[2] = pSensor->sensorStatus.subSensorStatus[2].isActive;
  
  /**********/ 
  
  /* LPS22HH */
  lps22hh_com_id = COM_AddSensor();
  
  pSensor = COM_GetSensor(lps22hh_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "LPS22HH");
  pSensor->sensorDescriptor.nSubSensors = 2;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_PRESS;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "prs");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 10.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 25.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = 50.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[4] = 75.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[5] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[6] = 200.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[7] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "hPa");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 1260.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  pSensor->sensorStatus.subSensorStatus[0].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[0].FS = 1260.0f;
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[0].ODR = 200.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 200;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 1600;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_LPS22HH_P;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  
    /* SUBSENSOR 1 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[1].id = 1;
  pSensor->sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_TEMP;
  pSensor->sensorDescriptor.subSensorDescriptor[1].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[0], "tem");
  pSensor->sensorDescriptor.subSensorDescriptor[1].dataType = DATA_TYPE_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[1] = 10.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[2] = 25.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[3] = 50.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[4] = 75.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[5] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[6] = 200.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[7] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].unit, "Celsius");
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[0] = 85.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 1 STATUS */
  pSensor->sensorStatus.subSensorStatus[1].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[1].FS = 85.0f;
  pSensor->sensorStatus.subSensorStatus[1].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[1].ODR = 200.0f;
  pSensor->sensorStatus.subSensorStatus[1].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].samplesPerTimestamp = 200;
  pSensor->sensorStatus.subSensorStatus[1].usbDataPacketSize = 1600;
  pSensor->sensorStatus.subSensorStatus[1].sdWriteBufferSize = WRITE_BUFFER_SIZE_LPS22HH_T;
  pSensor->sensorStatus.subSensorStatus[1].comChannelNumber = -1;
  
  LPS22HH_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  LPS22HH_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  LPS22HH_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  LPS22HH_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  LPS22HH_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;
  
  /**********/ 
  
  /* MP23ABS1 */
  mp23abs1_com_id = COM_AddSensor();
  
   pSensor = COM_GetSensor(mp23abs1_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "MP23ABS1");
  pSensor->sensorDescriptor.nSubSensors = 1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_MIC;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "aud");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 192000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Waveform");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 130.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  pSensor->sensorStatus.subSensorStatus[0].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[0].FS = 130.0f;
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[0].ODR = 192000.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 4096;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_MP23ABS1;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  
  MP23ABS1_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  MP23ABS1_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  MP23ABS1_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
    
  /**********/ 
  
  /* STTS751 */
  stts751_com_id = COM_AddSensor();
  
  pSensor = COM_GetSensor(stts751_com_id);
  
  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "STTS751");
  pSensor->sensorDescriptor.nSubSensors = 1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_TEMP;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "tem");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 2.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 4.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Celsius");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  pSensor->sensorStatus.subSensorStatus[0].isActive = 0;
  pSensor->sensorStatus.subSensorStatus[0].FS = 100.0f;
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[0].ODR = 4.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 20;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 16;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_STTS751;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  
  STTS751_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  STTS751_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  STTS751_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

}

void update_sensorStatus(COM_SensorStatus_t * oldSensorStatus, COM_SensorStatus_t * newSensorStatus, uint8_t sID)
{
  /* Check differences between oldSensorStatus and newSensorStatus, act properly*/

  /* subsensor: FS, ODR, is Active, channel number, samplePerTimestamp*/
  for (int i = 0; i < COM_GetSubSensorNumber(sID); i++)
  {
    if(oldSensorStatus->subSensorStatus[i].FS != newSensorStatus->subSensorStatus[i].FS)
    {
      oldSensorStatus->subSensorStatus[i].FS = newSensorStatus->subSensorStatus[i].FS; /* Todo Setters and getters */
    }
    
    if(oldSensorStatus->subSensorStatus[i].ODR != newSensorStatus->subSensorStatus[i].ODR)
    {
      oldSensorStatus->subSensorStatus[i].ODR = newSensorStatus->subSensorStatus[i].ODR; /* Todo Setters and getters */
    }
    
    if(oldSensorStatus->subSensorStatus[i].isActive != newSensorStatus->subSensorStatus[i].isActive)
    {
      oldSensorStatus->subSensorStatus[i].isActive = newSensorStatus->subSensorStatus[i].isActive; /* Todo Setters and getters */
    }

    if(oldSensorStatus->subSensorStatus[i].comChannelNumber != newSensorStatus->subSensorStatus[i].comChannelNumber)
    {
      oldSensorStatus->subSensorStatus[i].comChannelNumber = newSensorStatus->subSensorStatus[i].comChannelNumber; /* Todo Setters and getters */         
    }  
    
    if(oldSensorStatus->subSensorStatus[i].samplesPerTimestamp != newSensorStatus->subSensorStatus[i].samplesPerTimestamp)
    {
      oldSensorStatus->subSensorStatus[i].samplesPerTimestamp = newSensorStatus->subSensorStatus[i].samplesPerTimestamp; /* Todo Setters and getters */         
    }

    if(oldSensorStatus->subSensorStatus[i].usbDataPacketSize != newSensorStatus->subSensorStatus[i].usbDataPacketSize)
    {
      oldSensorStatus->subSensorStatus[i].usbDataPacketSize = newSensorStatus->subSensorStatus[i].usbDataPacketSize;
    }

    if(oldSensorStatus->subSensorStatus[i].sdWriteBufferSize != newSensorStatus->subSensorStatus[i].sdWriteBufferSize)
    {
      oldSensorStatus->subSensorStatus[i].sdWriteBufferSize = newSensorStatus->subSensorStatus[i].sdWriteBufferSize;
    }
#if (HSD_USE_FAKE_DATA == 1)
  oldSensorStatus->subSensorStatus[i].sensitivity = 1.0f;
#endif
  }

/* Specific cases for Sensitivity */  
#if (HSD_USE_FAKE_DATA != 1)
  if (sID == iis3dwb_com_id)
  {
    oldSensorStatus->subSensorStatus[0].sensitivity = 0.0305f *  oldSensorStatus->subSensorStatus[0].FS;
  }
  else if (sID == iis2dh_com_id)
  {    
    if (oldSensorStatus->subSensorStatus[0].FS == 16.0f)
    {
      oldSensorStatus->subSensorStatus[0].sensitivity = 0.75f;
    }
    else
    {
      oldSensorStatus->subSensorStatus[0].sensitivity = 0.03125f * oldSensorStatus->subSensorStatus[0].FS;
    }
  }
  else if (sID == ism330dhcx_com_id)
  {        
    oldSensorStatus->subSensorStatus[0].sensitivity = 0.0305f *  oldSensorStatus->subSensorStatus[0].FS;
    oldSensorStatus->subSensorStatus[1].sensitivity = 0.035f * oldSensorStatus->subSensorStatus[1].FS;
  }
#endif
}

void update_sensors_config(void)
{  
  COM_Sensor_t * pSensor;  
  
  pSensor = COM_GetSensor(iis3dwb_com_id);  
  IIS3DWB_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS3DWB_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS3DWB_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  
  pSensor = COM_GetSensor(hts221_com_id);
  HTS221_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  HTS221_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  HTS221_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  HTS221_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  HTS221_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;
  
  pSensor = COM_GetSensor(iis2dh_com_id);
  IIS2DH_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS2DH_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2DH_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  
  pSensor = COM_GetSensor(iis2mdc_com_id);
  IIS2MDC_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS2MDC_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2MDC_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;  
  
  pSensor = COM_GetSensor(imp34dt05_com_id);
  IMP34DT05_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IMP34DT05_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IMP34DT05_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;  
  
  pSensor = COM_GetSensor(ism330dhcx_com_id);
  ISM330DHCX_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  ISM330DHCX_Init_Param.ODR[1] = pSensor->sensorStatus.subSensorStatus[1].ODR;
  ISM330DHCX_Init_Param.ODR[2] = pSensor->sensorStatus.subSensorStatus[2].ODR;
  ISM330DHCX_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  ISM330DHCX_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  ISM330DHCX_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  ISM330DHCX_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;
  ISM330DHCX_Init_Param.subSensorActive[2] = pSensor->sensorStatus.subSensorStatus[2].isActive;
  
  pSensor = COM_GetSensor(lps22hh_com_id);
  LPS22HH_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  LPS22HH_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  LPS22HH_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  LPS22HH_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  LPS22HH_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive; 
  
  pSensor = COM_GetSensor(mp23abs1_com_id);
  MP23ABS1_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  MP23ABS1_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  MP23ABS1_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;  
  
  pSensor = COM_GetSensor(stts751_com_id);
  STTS751_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  STTS751_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  STTS751_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive; 
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
