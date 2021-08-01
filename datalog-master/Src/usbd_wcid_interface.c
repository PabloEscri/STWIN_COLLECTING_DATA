/**
******************************************************************************
* @file    usbd_wcid_interface.c
* @author  SRA
* @version v3.0.0
* @date    19-Jun-2020
* @brief   Source file for USBD WCID interface
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
#include "usbd_wcid_interface.h"
#include "main.h"
#include "com_manager.h"
#include "HSD_json.h"

#include "mp23abs1_app.h"
#include "imp34dt05_app.h"
#include "ism330dhcx_app.h"
#include "iis3dwb_app.h"
#include "iis2mdc_app.h"
#include "iis2dh_app.h"
#include "hts221_app.h"
#include "lps22hh_app.h"
#include "stts751_app.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t *USB_RxBuffer = NULL;
uint8_t *TxBuffer[N_CHANNELS_MAX];
extern USBD_HandleTypeDef USBD_Device;
extern volatile uint8_t HSD_ResetUSB;

/* Private function prototypes -----------------------------------------------*/
static int8_t WCID_STREAMING_Itf_Init     (void);
static int8_t WCID_STREAMING_Itf_DeInit   (void);
static int8_t WCID_STREAMING_Itf_Control (uint8_t isHostToDevice, uint8_t cmd, uint16_t wValue, uint16_t wIndex, uint8_t* pbuf, uint16_t length);
static int8_t WCID_STREAMING_Itf_Receive  (uint8_t* pbuf, uint32_t Len);

static void _Error_Handler( void );


USBD_WCID_STREAMING_ItfTypeDef USBD_WCID_STREAMING_fops = 
{
  WCID_STREAMING_Itf_Init,
  WCID_STREAMING_Itf_DeInit,
  WCID_STREAMING_Itf_Control,
  WCID_STREAMING_Itf_Receive
};

/* Private functions ---------------------------------------------------------*/

/**
* @brief  WCID_STREAMING_Itf_Init
*         Initializes the WCID media low layer
* @param  None
* @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t WCID_STREAMING_Itf_Init(void)
{
  /*ToDo : add state variable, check for allocation success */
  USB_RxBuffer = HSD_calloc(512, sizeof(uint8_t));
  if(USB_RxBuffer == NULL)
  {
    /* Error */
  }    
  
  USBD_WCID_STREAMING_SetRxDataBuffer(&USBD_Device, (uint8_t *)USB_RxBuffer);  
  return (USBD_OK);
}

/**
* @brief  WCID_STREAMING_Itf_DeInit
*         DeInitializes the WCID media low layer
* @param  None
* @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t WCID_STREAMING_Itf_DeInit(void)
{
  /*ToDo : add state variable, check for allocation success */  
  if( USB_RxBuffer != NULL)
  {
    HSD_free(USB_RxBuffer);
    USB_RxBuffer = NULL;
  }
  
  return (USBD_OK);
}


/**
* @brief  WCID_STREAMING_Itf_Control
*         Manage the WCID class requests
* @param  Cmd: Command code            
* @param  Buf: Buffer containing command data (request parameters)
* @param  Len: Number of data to be sent (in bytes)
* @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t WCID_STREAMING_Itf_Control (uint8_t isHostToDevice, uint8_t cmd, uint16_t wValue, uint16_t wIndex, uint8_t* pbuf, uint16_t length)
{ 
  if (com_status != HS_DATALOG_IDLE && com_status != HS_DATALOG_USB_STARTED )
    return USBD_FAIL;
  
  uint32_t i = 0;
  static uint16_t USB_packet_size = 0;
  static uint16_t counter = 0;
  static char * serialized = 0;
  static char* p = 0;
  static COM_Command_t outCommand;
  static COM_Sensor_t tempSensor;
  COM_Device_t *pDevice;
  COM_DeviceDescriptor_t *pDeviceDescriptor;
  COM_SensorDescriptor_t *pSensorDescriptor;
  COM_SubSensorDescriptor_t *pSubSensorDescriptor;
  COM_SensorStatus_t *pSensorStatus;
  COM_SubSensorStatus_t *pSubSensorStatus;
  COM_AcquisitionDescriptor_t *pAcquisitionDescriptor;
  uint32_t sID=0, ssID=0;

  static uint8_t state = STATE_WAITING;

  if(isHostToDevice)
  {
    switch(state)
    {
    case STATE_WAITING:

      if(cmd != CMD_SIZE_SET)
        return -1; /* error */

      USB_packet_size = *(uint16_t *)pbuf;
      serialized = HSD_malloc(USB_packet_size);
      p = serialized;
      state = STATE_SIZE_RCVD;
      counter = USB_packet_size;

      break;
    case STATE_SIZE_RCVD:
      if(cmd != CMD_DATA_SET)
        return -1; /* error */

      for (i = 0; i < length; i ++)
      {
        *p++ = pbuf[i];
        counter--;
      }

      if (counter == 0)
      {
        HSD_JSON_parse_Command((char *)serialized, &outCommand);
        state = STATE_REQUEST_SET;

        if(outCommand.command == COM_COMMAND_SET)
        {
          switch(outCommand.request)
          {
          case COM_REQUEST_ACQ_INFO:
          {
            //SET Acquisition Name and Description
            char name[HSD_ACQ_NAME_LENGTH];
            char notes[HSD_ACQ_NOTES_LENGTH];
            HSD_JSON_parse_AcqInfoCommand((char*)serialized, name, HSD_ACQ_NAME_LENGTH, notes, HSD_ACQ_NOTES_LENGTH);
            COM_SetAcquisitionDescriptor(name,notes);
            break;
          }              
          case COM_REQUEST_SW_TAG_LABEL:
          {
            //SET SW Tag Label
            uint8_t id;
            char label[HSD_TAGS_LABEL_LENGTH];
            COM_Device_t *device = COM_GetDevice();
            HSD_JSON_parse_UpdateTagLabelCommand((char*)serialized, &id, label, HSD_TAGS_LABEL_LENGTH);
            HSD_TAGS_set_tag_label(device, HSD_TAGS_Type_Sw, id, label);
            break;
          }            
          case COM_REQUEST_HW_TAG_LABEL:
          {
            //SET HW Tag Label
            uint8_t id;
            char label[HSD_TAGS_LABEL_LENGTH];
            COM_Device_t *device = COM_GetDevice();
            HSD_JSON_parse_UpdateTagLabelCommand((char*)serialized, &id, label, HSD_TAGS_LABEL_LENGTH);
            HSD_TAGS_set_tag_label(device, HSD_TAGS_Type_Hw, id, label);
            break;
          }
          case COM_REQUEST_HW_TAG:
          {
            //SET HW Tag (Enable/Disable)
            uint8_t id;
            HSD_Tags_Enable_t enable;
            COM_Device_t *device = COM_GetDevice();
            HSD_JSON_parse_EnableTagCommand((char*)serialized, &id, &enable);
            HSD_free(serialized);
            HSD_TAGS_set_tag_enabled(device, id, enable);
            break;
          }
          case COM_REQUEST_SW_TAG:
          {
            //SET SW Tag (Enable/Disable)
            uint8_t id;
            HSD_Tags_Enable_t enable;
            double timestamp = SM_GetTimeStamp_fromISR();
            HSD_JSON_parse_EnableTagCommand((char*)serialized, &id, &enable);
            HSD_TAGS_add_tag(HSD_TAGS_Type_Sw, id, enable, timestamp);
            break;
          }            
          case COM_REQUEST_MLC_CONFIG:
          {
            //Extract loaded ucf size and data
            uint32_t mlcConfigSize;
            char *mlcConfigData;
            mlcConfigData = (char*)USBD_malloc(USB_packet_size);
            HSD_JSON_parse_MlcConfigCommand((char*)serialized, &mlcConfigSize, mlcConfigData, USB_packet_size);
            HSD_free(serialized);
            ISM330DHCX_SetUCF(mlcConfigSize, mlcConfigData);
            break;
          }            
          default:
          {
            pSensorStatus = COM_GetSensorStatus(outCommand.sensorId);
            memcpy(&tempSensor.sensorStatus, pSensorStatus, sizeof(COM_SensorStatus_t));
            HSD_JSON_parse_Status((char *)serialized, &tempSensor.sensorStatus);
            HSD_free(serialized);            
            update_sensorStatus(pSensorStatus, &tempSensor.sensorStatus, outCommand.sensorId);
            
            /* Update the sensor-specific config structure */
            update_sensors_config();
          }
          }
          
          state = STATE_WAITING;
        }
        else if(outCommand.command == COM_COMMAND_START)
        {
          com_status = HS_DATALOG_USB_STARTED;
          pDeviceDescriptor = COM_GetDeviceDescriptor();
          COM_GenerateAcquisitionUUID();
          SM_TIM_Start();

          uint8_t sensorIsActive;
          for(sID=0;sID<pDeviceDescriptor->nSensor;sID++)
          {
            sensorIsActive = 0;
            pSensorDescriptor = COM_GetSensorDescriptor(sID);

            for(ssID=0;ssID<pSensorDescriptor->nSubSensors;ssID++)
            {
              pSubSensorStatus = COM_GetSubSensorStatus(sID, ssID);

              if(pSubSensorStatus->comChannelNumber != -1 && pSubSensorStatus->isActive)
              {
                sensorIsActive = 1;
                TxBuffer[pSubSensorStatus->comChannelNumber] = NULL;
                TxBuffer[pSubSensorStatus->comChannelNumber] = HSD_calloc((pSubSensorStatus->usbDataPacketSize *2 +2), sizeof(uint8_t));
                if(TxBuffer[pSubSensorStatus->comChannelNumber] == NULL)
                {
                  /* Error */
                  _Error_Handler();
                }

                USBD_WCID_STREAMING_SetTxDataBuffer(&USBD_Device, pSubSensorStatus->comChannelNumber, TxBuffer[pSubSensorStatus->comChannelNumber], pSubSensorStatus->usbDataPacketSize);
                USBD_WCID_STREAMING_CleanTxDataBuffer(&USBD_Device, pSubSensorStatus->comChannelNumber);

                COM_GetSubSensorContext(sID, ssID)->first_dataReady = 1;
              }
            }
            if(sensorIsActive)
            {
              SM_StartSensorThread(sID);
            }
          }
          USBD_WCID_STREAMING_StartStreaming(&USBD_Device);
          HSD_TAGS_timer_start();
          state = STATE_WAITING;
        }
        else if(outCommand.command == COM_COMMAND_STOP)
        {
          USBD_WCID_STREAMING_StopStreaming(&USBD_Device);
          com_status = HS_DATALOG_IDLE;
          HSD_TAGS_timer_stop();

          for (int i = 0; i < N_CHANNELS_MAX; i++)
          {
            if( TxBuffer[i] != NULL)
            {
              HSD_free(TxBuffer[i]);
              TxBuffer[i] = NULL;
            }
          }

          pDeviceDescriptor = COM_GetDeviceDescriptor();

          for(sID=0;sID<pDeviceDescriptor->nSensor;sID++)
          {
            pSensorDescriptor = COM_GetSensorDescriptor(sID);

            for(ssID=0;ssID<pSensorDescriptor->nSubSensors;ssID++)
            {
              pSubSensorStatus = COM_GetSubSensorStatus(sID, ssID);

              if(pSubSensorStatus->comChannelNumber != -1)
              {
                SM_StopSensorThread(sID);
                COM_GetSubSensorContext(sID, ssID)->first_dataReady = 0;
              }
            }
          }
          SM_TIM_Stop();
          state = STATE_WAITING;
          
          HSD_ResetUSB = 1;
        }
      }
      break;
    }
  }
  else /* Device to host */
  {
    switch(state)
    {
    case STATE_REQUEST_SET: /* Host needs size */

      if(cmd != CMD_SIZE_GET)
        return -1; /* error*/

      HSD_JSON_free(serialized);

      switch(outCommand.request)
      {

      case COM_REQUEST_DEVICE:
        {
        pDevice = COM_GetDevice();
        USB_packet_size = HSD_JSON_serialize_Device(pDevice, &serialized, SHORT_JSON);
        break;
        }
      case COM_REQUEST_DEVICE_INFO:
        {
        pDeviceDescriptor = COM_GetDeviceDescriptor();
        USB_packet_size = HSD_JSON_serialize_DeviceInfo(pDeviceDescriptor, &serialized);
        break;
        }
      case COM_REQUEST_ACQ_INFO:
        {
        pAcquisitionDescriptor = COM_GetAcquisitionDescriptor();
        USB_packet_size = HSD_JSON_serialize_Acquisition(pAcquisitionDescriptor, &serialized, PRETTY_JSON);
        break;
        }
      case COM_REQUEST_DESCRIPTOR:
        {
        if (outCommand.subSensorId < 0) /* Request is for Sensor, since subSensor was not present in the Json */
        {
          pSensorDescriptor =  COM_GetSensorDescriptor(outCommand.sensorId);
          USB_packet_size = HSD_JSON_serialize_SensorDescriptor(pSensorDescriptor, &serialized);
        }
        else
        {
          pSubSensorDescriptor = COM_GetSubSensorDescriptor(outCommand.sensorId, outCommand.subSensorId);
          USB_packet_size = HSD_JSON_serialize_SubSensorDescriptor(pSubSensorDescriptor, &serialized);
        }
        break;
        }
      case COM_REQUEST_STATUS:
        {
          if (outCommand.subSensorId < 0) /* Request is for Sensor, since subSensor was not present in the Json */
        {
          pSensorStatus =  COM_GetSensorStatus(outCommand.sensorId);
          USB_packet_size = HSD_JSON_serialize_SensorStatus(pSensorStatus, &serialized);
        }
        else
        {
          pSubSensorStatus = COM_GetSubSensorStatus(outCommand.sensorId, outCommand.subSensorId);
          USB_packet_size = HSD_JSON_serialize_SubSensorStatus(pSubSensorStatus, &serialized);
        }
        break;   
        }
      case COM_REQUEST_TAG_CONFIG:
        {
          COM_TagList_t *tagConfig = COM_GetTagList();
          tagConfig->HwTag[1].enabled=1;
          USB_packet_size = HSD_JSON_serialize_TagList(tagConfig, &serialized, PRETTY_JSON);
          break;
        }
      } 
     

      *(uint16_t *)pbuf = USB_packet_size;
      p = serialized;

      state = STATE_SIZE_SENT;
      counter = USB_packet_size;
      break;
    case STATE_SIZE_SENT:

      if(cmd != CMD_DATA_GET)
        return -1; /* error*/

      for (i = 0; i < length; i++)
      {
        pbuf[i] = *p++;
        counter--;
      }
      if(counter == 0)
      {
        HSD_JSON_free(serialized);
        serialized = NULL;
        state = STATE_WAITING;
      }
      break;
    }
  }
  
  return (USBD_OK);
}

/**
* @brief  SS_WCID_Itf_DataRx
*         Data received over USB OUT endpoint are sent over WCID interface 
*         through this function.
* @param  Buf: Buffer of data to be transmitted
* @param  Len: Number of data received (in bytes)
* @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t WCID_STREAMING_Itf_Receive(uint8_t* Buf, uint32_t Len)
{  
  return (USBD_OK);
}


/**
* @brief  This function is executed in case of error occurrence
* @param  None
* @retval None
*/
static void _Error_Handler( void )
{
  while (1)
  {}
}





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
