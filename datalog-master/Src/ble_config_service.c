/**
  ******************************************************************************
  * @file    config_service.c
  * @author  SRA
  * @version v1.0.0
  * @date    18-Oct-19
  * @brief   Add bluetooth services using vendor specific profiles.
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
  
#include <stdio.h>
#include "main.h"
#include "bluenrg1_l2cap_aci.h"
#include "bluenrg1_gatt_aci.h"
#include "bluenrg1_gap_aci.h"
#include "hci.h"
#include "bluenrg1_hci_le.h"
#include "bluenrg1_hal_aci.h" 
#include "ble_config_service.h"
#include "ble_comm_transfer_protocol.h"
#include "sensors_manager.h"
#include "com_manager.h"
#include "HSD_tags.h"
#include "HSD_json.h"
#include "ism330dhcx_app.h"
#include "sdcard_manager.h"
#include "main.h"

/* Defines -------------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
int connected = FALSE;
uint8_t set_connectable = TRUE;

/* Imported Variables -------------------------------------------------------------*/
extern osSemaphoreId bleSendThreadSem_id;
extern osSemaphoreId bleInitThreadSem_id;
#if (HSD_BLE_STATUS_TIMER_ENABLE == 1)
extern osTimerId bleSendPerformanceStatusTim_id;
#endif
extern uint8_t SD_Logging_Active;

uint32_t ConnectionBleStatus;

uint8_t bdaddr[6];

/* Private variables ------------------------------------------------------------*/
uint16_t configServiceHandle;
uint16_t configTxCharHandle;
static uint16_t configRxCharHandle;
static uint16_t connection_handle = 0;

Service_UUID_t service_config_uuid;
Char_UUID_t char_tx_uuid;
Char_UUID_t char_rx_uuid;

//uint32_t commandFullLength = 0;
COM_Command_t outCommand;
COM_Sensor_t tempSensor;
COM_SensorStatus_t * myStatus;
uint8_t commandMask = 0;

/* STWINConfig Service UUID */
static const uint8_t config_service_uuid[16] = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xb4,0x9a,0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x00};
static const uint8_t config_tx_char_uuid[16] = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x02,0x00,0x11,0x00,0x00,0x00};
static const uint8_t config_rx_char_uuid[16] = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x02,0x00,0x12,0x00,0x00,0x00}; // UNUSED to be removed

////////////////****************************AÑADIDO*/////////////////


Service_UUID_t service_uuid;
Char_UUID_t char_uuid;
uint16_t ConfigServW2STHandle;
uint16_t ConfigCharHandle;
uint16_t char_mia;
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
{\
  uuid_struct[0 ] = uuid_0 ; uuid_struct[1 ] = uuid_1 ; uuid_struct[2 ] = uuid_2 ; uuid_struct[3 ] = uuid_3 ; \
  uuid_struct[4 ] = uuid_4 ; uuid_struct[5 ] = uuid_5 ; uuid_struct[6 ] = uuid_6 ; uuid_struct[7 ] = uuid_7 ; \
  uuid_struct[8 ] = uuid_8 ; uuid_struct[9 ] = uuid_9 ; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
  uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}
#define COPY_CONFIG_W2ST_CHAR3_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55)

#define COPY_CONFIG_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0F,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#define COPY_CONFIG_W2ST_CHAR2_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33)
////////////////****************************FIN AÑADIDO****************************/////////////////


uint8_t *hs_command_buffer;//[800];
char *mlcConfigData;//[800];

uint8_t ble_init = 0;


/* Private functions ------------------------------------------------------------*/

void APP_UserEvtRx(void *pData);
tBleStatus Init_BlueNRG_Stack(void);
tBleStatus Add_Config_Service(void);
void setConnectable(void);

/* Private define ------------------------------------------------------------*/


void ble_interface_init(void)
{
  if(ble_init != 1)
  {
    BLE_CM_SPI_Reset();  
  
    Init_BlueNRG_Stack();
  
    Add_Config_Service();
    
    ble_init = 1;
  }
  setConnectable();
}

/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
tBleStatus Init_BlueNRG_Stack(void)
{
  const char BoardName[8] = {NAME_HSD,0};
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint32_t stm32_UID[3];

  /* Initialize the BlueNRG HCI */
  hci_init(APP_UserEvtRx, NULL);
  
  HAL_Delay(100);
  
  /* Create a Unique BLE MAC */
  stm32_UID[0] = HAL_GetUIDw0();
  stm32_UID[1] = HAL_GetUIDw1();
  stm32_UID[2] = HAL_GetUIDw2();

  bdaddr[0] = (stm32_UID[1]>>24)&0xFF;
  bdaddr[1] = (stm32_UID[0]    )&0xFF;
  bdaddr[2] = (stm32_UID[2] >>8)&0xFF;
  bdaddr[3] = (stm32_UID[0]>>16)&0xFF;
  bdaddr[4] = (((HSD_VERSION_MAJOR-48)*10) + (HSD_VERSION_MINOR-48)+100)&0xFF;
  bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
  
  ret = aci_gatt_init();
  ////////////////****************************AÑADIDO*/////////////////
  uint8_t uuid[16];

      COPY_CONFIG_SERVICE_UUID(uuid);
      BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
      ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 8,&ConfigServW2STHandle);


      COPY_CONFIG_W2ST_CHAR2_UUID(uuid);
      BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
      ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, &char_uuid, 20 ,
                               CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                               ATTR_PERMISSION_NONE,
  							 GATT_DONT_NOTIFY_EVENTS,
                               16, 1, &ConfigCharHandle);


      COPY_CONFIG_W2ST_CHAR3_UUID(uuid);
      BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
      ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, &char_uuid, 20 /* Max Dimension */,
    		  CHAR_PROP_READ|CHAR_PROP_NOTIFY,
                               ATTR_PERMISSION_NONE,
  							 GATT_DONT_NOTIFY_EVENTS,
  							 16, 1, &char_mia);
      char vector[6] = {'H','O','W','L','A','B'};
      aci_gatt_update_char_value(ConfigServW2STHandle, char_mia, 0, 6,vector);
  ////////////////****************************FIN AÑADIDO****************************/////////////////


  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }
  
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  ret = hci_le_set_random_address(bdaddr);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }
  
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  ret = aci_gap_set_authentication_requirement(BONDING,
                                     MITM_PROTECTION_REQUIRED,
                                     SC_IS_SUPPORTED,
                                     KEYPRESS_IS_NOT_SUPPORTED,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     0x00);
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);  

  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Add the Config service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_Config_Service(void)
{
  tBleStatus ret;
  
  BLUENRG_memcpy(&service_config_uuid.Service_UUID_128, config_service_uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_config_uuid, PRIMARY_SERVICE, 1+6, &configServiceHandle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }
    
  BLUENRG_memcpy(&char_tx_uuid.Char_UUID_128, config_tx_char_uuid, 16);  
  ret =  aci_gatt_add_char(configServiceHandle, UUID_TYPE_128, &char_tx_uuid, 20,
                           CHAR_PROP_NOTIFY | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE, 16, 1, &configTxCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }
  
  BLUENRG_memcpy(&char_rx_uuid.Char_UUID_128, config_rx_char_uuid, 16);  
  ret =  aci_gatt_add_char(configServiceHandle, UUID_TYPE_128, &char_rx_uuid, 20,
                           CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE, 16, 1, &configRxCharHandle);

  if (ret != BLE_STATUS_SUCCESS) 
  {
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

tBleStatus status_ble;
/**
 * @brief  Puts the device in connectable mode.
 * @param  None 
 * @retval None
 */
void setConnectable(void)
{  
  char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NAME_HSD};
  uint8_t manuf_data[26] = {
    2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
    8,0x09,NAME_HSD, // Complete Name
    13,0xFF,0x01/*SKD version */,
    0x02,
    0x00, /* */
    0xE0, /* ACC+Gyro+Mag*/
    0x00, /*  */
    0x00, /*  */
    0x00, /* BLE MAC start */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
  };

  /* BLE MAC */
  manuf_data[20] = bdaddr[5];
  manuf_data[21] = bdaddr[4];
  manuf_data[22] = bdaddr[3];
  manuf_data[23] = bdaddr[2];
  manuf_data[24] = bdaddr[1];
  manuf_data[25] = bdaddr[0];


  /* disable scan response */
  status_ble = hci_le_set_scan_response_data(0,NULL);
  status_ble = aci_gap_set_discoverable(ADV_IND, 0, 0,
                           STATIC_RANDOM_ADDR,
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), (uint8_t*)local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
  status_ble = aci_gap_update_adv_data(26, manuf_data);
}


/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application 
 * is subscribed or not to the one service
 * @param uint16_t att_handle Handle of the attribute
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{
  if(attr_handle == configTxCharHandle + 1)
  {
    uint32_t HSCommandBufLen = BLECommand_TP_Parse(&hs_command_buffer, att_data, data_length);
    if (HSCommandBufLen > 0) 
    {
      HSD_JSON_parse_Command((char*)hs_command_buffer, &outCommand);
      if(outCommand.command == COM_COMMAND_SET)
      {
        switch (outCommand.request)
        {
          case COM_REQUEST_DEVICE_INFO:
          {
            //SET device alias
            char alias[HSD_DEVICE_ALIAS_LENGTH];
            HSD_JSON_parse_SetDeviceAliasCommand((char*)hs_command_buffer, alias, HSD_DEVICE_ALIAS_LENGTH);
            HSD_free(hs_command_buffer);
            COM_SetDeviceAlias(alias);
            commandMask = COM_REQUEST_DEVICE_INFO;
            osSemaphoreRelease(bleSendThreadSem_id);
            break;
          }
          case COM_REQUEST_SW_TAG:
          {
            //SET SW Tag (Enable/Disable)
            uint8_t id;
            HSD_Tags_Enable_t enable;
            double timestamp = SM_GetTimeStamp();
            HSD_JSON_parse_EnableTagCommand((char*)hs_command_buffer, &id, &enable);
            HSD_free(hs_command_buffer);
            HSD_TAGS_add_tag(HSD_TAGS_Type_Sw, id, enable, timestamp);
            break;
          }
          case COM_REQUEST_SW_TAG_LABEL:
          {
            //SET SW Tag Label
            uint8_t id;
            char label[HSD_TAGS_LABEL_LENGTH];
            COM_Device_t *device = COM_GetDevice();
            HSD_JSON_parse_UpdateTagLabelCommand((char*)hs_command_buffer, &id, label, HSD_TAGS_LABEL_LENGTH);
            HSD_free(hs_command_buffer);
            HSD_TAGS_set_tag_label(device, HSD_TAGS_Type_Sw, id, label);
            break;
          }
          case COM_REQUEST_HW_TAG:
          {
            //SET HW Tag (Enable/Disable)
            uint8_t id;
            HSD_Tags_Enable_t enable;
            COM_Device_t *device = COM_GetDevice();
            HSD_JSON_parse_EnableTagCommand((char*)hs_command_buffer, &id, &enable);
            HSD_free(hs_command_buffer);
            HSD_TAGS_set_tag_enabled(device, id-HSD_TAGS_MAX_SW_CLASSES, enable);
            break;
          }
          case COM_REQUEST_HW_TAG_LABEL:
          {
            //SET HW Tag Label
            uint8_t id;
            char label[HSD_TAGS_LABEL_LENGTH];
            COM_Device_t *device = COM_GetDevice();
            HSD_JSON_parse_UpdateTagLabelCommand((char*)hs_command_buffer, &id, label, HSD_TAGS_LABEL_LENGTH);
            HSD_free(hs_command_buffer);
            HSD_TAGS_set_tag_label(device, HSD_TAGS_Type_Hw, id-HSD_TAGS_MAX_SW_CLASSES, label);
            break;
          }
          case COM_REQUEST_STATUS_NETWORK:
          {
            //TODO set network credentials command parse
            HSD_free(hs_command_buffer);
            break;
          }
          case COM_REQUEST_ACQ_INFO:
          {
            //SET Acquisition Name and Description
            char name[HSD_ACQ_NAME_LENGTH];
            char notes[HSD_ACQ_NOTES_LENGTH];
            HSD_JSON_parse_AcqInfoCommand((char*)hs_command_buffer, name, HSD_ACQ_NAME_LENGTH, notes, HSD_ACQ_NOTES_LENGTH);
            HSD_free(hs_command_buffer);
            COM_SetAcquisitionDescriptor(name,notes);
            break;
          }
          case COM_REQUEST_MLC_CONFIG:
          {
            //Extract loaded ucf size and data
            uint32_t mlcConfigSize;
            mlcConfigData = (char*)HSD_malloc(HSCommandBufLen);
            //*buffer_out = (uint8_t*)malloc((message_length) * sizeof(uint8_t));
            HSD_JSON_parse_MlcConfigCommand((char*)hs_command_buffer, &mlcConfigSize, mlcConfigData, HSCommandBufLen);
            HSD_free(hs_command_buffer);
            ISM330DHCX_SetUCF(mlcConfigSize, mlcConfigData);
            break;
          }
          default:
          {
            myStatus = COM_GetSensorStatus(outCommand.sensorId);
            memcpy(&tempSensor.sensorStatus, myStatus, sizeof(COM_SensorStatus_t));
            HSD_JSON_parse_Status((char *)hs_command_buffer, &tempSensor.sensorStatus);
            HSD_free(hs_command_buffer);
            update_sensorStatus(myStatus, &tempSensor.sensorStatus, outCommand.sensorId);
            
            /* Update the sensor-specific config structure */
            update_sensors_config();
          }
        }
      } 
      else if (outCommand.command == COM_COMMAND_GET)
      {
        commandMask = outCommand.request;
        HSD_free(hs_command_buffer);
        osSemaphoreRelease(bleSendThreadSem_id);
      }
      else if ((outCommand.command == COM_COMMAND_START) && (SD_Logging_Active == 0))
      {
        HSD_free(hs_command_buffer);
        if(BSP_SD_IsDetected() == SD_PRESENT){
          if(osMessagePut(sdThreadQueue_id, SDM_START_STOP, 0) != osOK)
          {
            SM_Error_Handler();
          }
        } else {
          commandMask = COM_REQUEST_STATUS_LOGGING;
          osSemaphoreRelease(bleSendThreadSem_id);
        }
      }
      else if ((outCommand.command == COM_COMMAND_STOP) && (SD_Logging_Active == 1))
      {
        HSD_free(hs_command_buffer);
        if(osMessagePut(sdThreadQueue_id, SDM_START_STOP, 0) != osOK)
        {
          SM_Error_Handler();
        }
      }
      else if (outCommand.command == BLE_COMMAND_SAVE)
      {
        HSD_free(hs_command_buffer);
        SDM_UpdateDeviceConfig();
      }
    }
  }
}

void APP_UserEvtRx(void *pData)
{
  uint32_t i;

  hci_spi_pckt *hci_pckt = (hci_spi_pckt *)pData;

  if(hci_pckt->type == HCI_EVENT_PKT)
  {
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

    if(event_pckt->evt == EVT_LE_META_EVENT)
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      for (i = 0; i < (sizeof(hci_le_meta_events_table)/sizeof(hci_le_meta_events_table_type)); i++)
      {
        if (evt->subevent == hci_le_meta_events_table[i].evt_code)
        {
          hci_le_meta_events_table[i].process((void *)evt->data);
        }
      }
    }
    else if(event_pckt->evt == EVT_VENDOR)
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;        

      for (i = 0; i < (sizeof(hci_vendor_specific_events_table)/sizeof(hci_vendor_specific_events_table_type)); i++)
      {
        if (blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code)
        {
          hci_vendor_specific_events_table[i].process((void *)blue_evt->data);
        }
      }
    }
    else
    {
      for (i = 0; i < (sizeof(hci_events_table)/sizeof(hci_events_table_type)); i++)
      {
        if (event_pckt->evt == hci_events_table[i].evt_code)
        {
          hci_events_table[i].process((void *)event_pckt->data);
        }
      }
    }
  }
}

//uint8_t HCI_Version;
//uint16_t HCI_Revision;
//uint8_t LMP_PAL_Version;
//uint16_t Manufacturer_Name;
//uint16_t LMP_PAL_Subversion;
/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{ 
  connected = TRUE;
  connection_handle = Connection_Handle;

  ConnectionBleStatus=0;
  
  aci_l2cap_connection_parameter_update_req(connection_handle,8,17,0,400);

#if (HSD_BLE_STATUS_TIMER_ENABLE == 1)
  osTimerStart(bleSendPerformanceStatusTim_id,2000);
#endif
  
  //hci_read_local_version_information(&HCI_Version,&HCI_Revision,&LMP_PAL_Version,&Manufacturer_Name,&LMP_PAL_Subversion);
  
}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  connected = FALSE;

#if (HSD_BLE_STATUS_TIMER_ENABLE == 1)
  osTimerStop(bleSendPerformanceStatusTim_id);
#endif
  
  /* Make the device connectable again. */
  osSemaphoreRelease(bleInitThreadSem_id);
  
  set_connectable = TRUE;

  ConnectionBleStatus=0;
}/* end hci_disconnection_complete_event() */




void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  Attribute_Modified_CB(Attr_Handle, Attr_Data, Attr_Data_Length); 
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
