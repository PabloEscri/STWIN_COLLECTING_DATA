/**
******************************************************************************
* @file    HSD_json.c
* @author  SRA
* @version v3.0.0
* @date    19-Jun-2020
* @brief   High Speed DataLog Json Interpreter
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
#include "HSD_json.h"
#include "parson.h"
#include "cpu_utils.h"
#include "STWIN_bc.h"
#include "STWIN_sd.h"

/* Private variables ---------------------------------------------------------*/


static void (*JSON_free_function)(void *);
extern uint8_t SD_Logging_Active;

/* Private function prototypes -----------------------------------------------*/
static int32_t get_JSON_from_Device(COM_Device_t *device, char **serialized_string, uint8_t pretty);
static int32_t get_JSON_from_DeviceInfo(COM_DeviceDescriptor_t *device_descriptor, char **serialized_string);
static int32_t get_JSON_from_TagList(COM_TagList_t *tagList, char **serialized_string, uint8_t pretty);
static int32_t get_JSON_from_Sensor(COM_Sensor_t *sensor, char **serialized_string);
static int32_t get_JSON_from_SensorDescriptor(COM_SensorDescriptor_t *sensor_descriptor, char **serialized_string);
static int32_t get_JSON_from_SensorStatus(COM_SensorStatus_t *sensor_status, char **serialized_string);
static int32_t get_JSON_from_SubSensorDescriptor(COM_SubSensorDescriptor_t *sub_sensor_descriptor, char **serialized_string);
static int32_t get_JSON_from_SubSensorStatus(COM_SubSensorStatus_t *sub_sensor_status, char **serialized_string);
static int32_t get_JSON_FWStatus(char **serialized_string, uint8_t cmd_type);
static int32_t get_JSON_from_AcquisitionDescriptor(COM_AcquisitionDescriptor_t *acquisition_descriptor, char **serialized_string, uint8_t pretty);

static int32_t parse_Device_from_JSON(char *SerializedJSON, COM_Device_t * Device);
static int32_t parse_Command_from_JSON(char *SerializedJSON, COM_Command_t * outCommand);
static int32_t parse_Status_from_JSON(char *SerializedJSON, COM_SensorStatus_t * sensorStatus);
static int32_t parse_SetDeviceAliasCommand_from_JSON(char *SerializedJSON, char *alias, uint8_t aliasSize);
static int32_t parse_EnableTagCommand_from_JSON(char *SerializedJSON, uint8_t *class_id, HSD_Tags_Enable_t *enable);
static int32_t parse_UpdateTagLabelCommand_from_JSON(char *SerializedJSON, uint8_t *class_id, char *label, uint8_t labelSize);
static int32_t parse_AcqInfoCommand_from_JSON(char *SerializedJSON, char *name, uint8_t nameSize, char *notes, uint8_t notesSize);
static int32_t parse_MlcConfigCommand_from_JSON(char *SerializedJSON, uint32_t *mlcConfigSize, char *mlcConfigData, uint32_t mlcConfigDataSize);  

static void create_JSON_DeviceInfo(COM_DeviceDescriptor_t *device_descriptor, JSON_Value *tempJSON);
static void create_JSON_TagList(COM_TagList_t *tagList, JSON_Value *tempJSON);
static void create_JSON_Sensor(COM_Sensor_t *sensor, JSON_Value *tempJSON);
static void create_JSON_SensorDescriptor(COM_SensorDescriptor_t *sensor_descriptor, JSON_Value *tempJSON);
static void create_JSON_SensorStatus(COM_SensorStatus_t *sensor_status, JSON_Value *tempJSON);
static void create_JSON_SubSensorDescriptor(COM_SubSensorDescriptor_t *sub_sensor_descriptor, JSON_Value *tempJSON);
static void create_JSON_SubSensorStatus(COM_SubSensorStatus_t *sub_sensor_status, JSON_Value *tempJSON);
static void create_JSON_PerformanceStatus(JSON_Value *tempJSON);
static void create_JSON_IsLogging(JSON_Value *tempJSON);
static void create_JSON_NetworkStatus(JSON_Value *tempJSON);
static void create_JSON_DeviceHWTag(uint8_t id, const COM_HwTag_t *hwTag, JSON_Value *tempJSON);
static void create_JSON_DeviceSWTag(uint8_t id, char *label, JSON_Value *tempJSON);
static void create_JSON_AcquisitionDescriptor(COM_AcquisitionDescriptor_t *acquisition_descriptor, JSON_Value *tempJSON);

/* Public function -----------------------------------------------------------*/

/**
* @brief  Set malloc() and free() Callbacks for
* @param  malloc_function: malloc() implementation
* @param  free_function: free() implementation
* @retval 0: no error
*/
int32_t HSD_JSON_set_allocation_functions(void * (*malloc_function)(size_t), void (*free_function)(void *))
{
  json_set_allocation_functions(malloc_function, free_function);
  JSON_free_function = free_function;
  return 0;
}

int32_t HSD_JSON_free(void * mem)
{
  JSON_free_function(mem);
  return 0;
}

/**
* @brief  Set malloc() and free() Callbacks for
* @param  Device: COM_Device_t struct instance to be serialized 
* @param  SerializedJSON: free() implementation
* @retval 0: no error
*/
int32_t HSD_JSON_serialize_Device(COM_Device_t *Device, char **SerializedJSON, uint8_t pretty)
{
  return get_JSON_from_Device(Device, SerializedJSON, pretty);
}

int32_t HSD_JSON_serialize_DeviceInfo(COM_DeviceDescriptor_t *DeviceInfo, char **SerializedJSON)
{
  return get_JSON_from_DeviceInfo(DeviceInfo, SerializedJSON);
}

int32_t HSD_JSON_serialize_TagList(COM_TagList_t *TagList, char **SerializedJSON, uint8_t pretty)
{
  return get_JSON_from_TagList(TagList, SerializedJSON, pretty);
}

int32_t HSD_JSON_serialize_Sensor(COM_Sensor_t *Sensor, char **SerializedJSON)
{
  return get_JSON_from_Sensor(Sensor, SerializedJSON);
}

int32_t HSD_JSON_serialize_SensorDescriptor(COM_SensorDescriptor_t *SensorDescriptor, char **SerializedJSON)
{
  return get_JSON_from_SensorDescriptor(SensorDescriptor, SerializedJSON);
}

int32_t HSD_JSON_serialize_SensorStatus(COM_SensorStatus_t *SensorStatus, char **SerializedJSON)
{
  return get_JSON_from_SensorStatus(SensorStatus, SerializedJSON);
}

int32_t HSD_JSON_serialize_SubSensorDescriptor(COM_SubSensorDescriptor_t *SubSensorDescriptor, char **SerializedJSON)
{
  return get_JSON_from_SubSensorDescriptor(SubSensorDescriptor, SerializedJSON);
}

int32_t HSD_JSON_serialize_SubSensorStatus(COM_SubSensorStatus_t *SubSensorStatus, char **SerializedJSON)
{
  return get_JSON_from_SubSensorStatus(SubSensorStatus, SerializedJSON);
}

int32_t HSD_JSON_serialize_Acquisition(COM_AcquisitionDescriptor_t *AcquisitionDescriptor, char **SerializedJSON, uint8_t pretty)
{
  return get_JSON_from_AcquisitionDescriptor(AcquisitionDescriptor, SerializedJSON, pretty);
}

int32_t HSD_JSON_parse_Device(char *SerializedJSON, COM_Device_t *Device)
{
  return parse_Device_from_JSON(SerializedJSON, Device);
}

int32_t HSD_JSON_parse_Command(char *SerializedJSON, COM_Command_t *Command)
{ 
  return parse_Command_from_JSON(SerializedJSON, Command);
}

int32_t HSD_JSON_parse_Status(char *SerializedJSON, COM_SensorStatus_t *SensorStatus)
{
  return parse_Status_from_JSON(SerializedJSON, SensorStatus);
}

int32_t HSD_JSON_parse_SetDeviceAliasCommand(char *SerializedJSON, char *alias, uint8_t aliasSize)
{
  return parse_SetDeviceAliasCommand_from_JSON(SerializedJSON,alias,aliasSize);
}

int32_t HSD_JSON_parse_EnableTagCommand(char *SerializedJSON, uint8_t *class_id, HSD_Tags_Enable_t *enable)
{
  return parse_EnableTagCommand_from_JSON(SerializedJSON,class_id,enable);
}

int32_t HSD_JSON_parse_UpdateTagLabelCommand(char *SerializedJSON, uint8_t *class_id, char *label, uint8_t labelSize)
{
  return parse_UpdateTagLabelCommand_from_JSON(SerializedJSON,class_id,label,labelSize);
}

int32_t HSD_JSON_parse_AcqInfoCommand(char *SerializedJSON, char *name, uint8_t nameSize, char *notes, uint8_t notesSize)
{
  return parse_AcqInfoCommand_from_JSON(SerializedJSON,name,nameSize,notes,notesSize);
}

int32_t HSD_JSON_parse_MlcConfigCommand(char *SerializedJSON, uint32_t *mlcConfigSize, char *mlcConfigData, uint32_t mlcConfigDataSize)
{
  return parse_MlcConfigCommand_from_JSON(SerializedJSON,mlcConfigSize,mlcConfigData,mlcConfigDataSize);  
}

int32_t HSD_JSON_serialize_FWStatus(char **SerializedJSON, uint8_t cmd_type)
{
  return get_JSON_FWStatus(SerializedJSON, cmd_type);
}

/* Private function ----------------------------------------------------------*/
static int32_t get_JSON_from_Device(COM_Device_t *device, char **serialized_string, uint8_t pretty)
{
  JSON_Value *tempJSON;
  JSON_Value *tempJSON1;
  JSON_Object *JSON_Device;
  JSON_Array *JSON_SensorArray;
  JSON_Array *JSON_SWTagsArray;
  JSON_Array *JSON_HWTagsArray;
  uint32_t size;
  uint32_t i;

  tempJSON = json_value_init_object();
  JSON_Device = json_value_get_object(tempJSON);
  
  json_object_dotset_string(JSON_Device, "UUIDAcquisition", device->UUIDAcquisition);
  json_object_dotset_string(JSON_Device, "JSONVersion", device->JSONVersion);

  json_object_dotset_string(JSON_Device, "device.deviceInfo.serialNumber", device->deviceDescriptor.serialNumber);
  json_object_dotset_string(JSON_Device, "device.deviceInfo.alias", device->deviceDescriptor.alias);
  json_object_dotset_string(JSON_Device, "device.deviceInfo.partNumber", device->deviceDescriptor.partNumber);
  json_object_dotset_string(JSON_Device, "device.deviceInfo.URL", device->deviceDescriptor.URL);
  json_object_dotset_string(JSON_Device, "device.deviceInfo.fwName", device->deviceDescriptor.fwName);
  json_object_dotset_string(JSON_Device, "device.deviceInfo.fwVersion", device->deviceDescriptor.fwVersion);
  json_object_dotset_string(JSON_Device, "device.deviceInfo.dataFileExt", device->deviceDescriptor.dataFileExt);
  json_object_dotset_string(JSON_Device, "device.deviceInfo.dataFileFormat", device->deviceDescriptor.dataFileFormat);
  json_object_dotset_number(JSON_Device, "device.deviceInfo.nSensor", device->deviceDescriptor.nSensor);
  
  json_object_dotset_value(JSON_Device, "device.sensor", json_value_init_array());
  JSON_SensorArray = json_object_dotget_array(JSON_Device, "device.sensor");
  
  for(i = 0; i < device->deviceDescriptor.nSensor; i++)
  {        
    tempJSON1 = json_value_init_object();  
    create_JSON_Sensor(device->sensors[i], tempJSON1);    
    json_array_append_value(JSON_SensorArray,tempJSON1);
  }

  json_object_dotset_number(JSON_Device, "device.tagConfig.maxTagsPerAcq", HSD_TAGS_MAX_PER_ACQUISITION);
  
  json_object_dotset_value(JSON_Device, "device.tagConfig.swTags", json_value_init_array());
  JSON_SWTagsArray = json_object_dotget_array(JSON_Device, "device.tagConfig.swTags");
  
  for(i = 0; i < HSD_TAGS_MAX_SW_CLASSES; i++)
  {        
    tempJSON1 = json_value_init_object();  
    create_JSON_DeviceSWTag(i,device->tagList.HSD_SwTagClasses[i], tempJSON1);    
    json_array_append_value(JSON_SWTagsArray,tempJSON1);
  } 
  
  json_object_dotset_value(JSON_Device, "device.tagConfig.hwTags", json_value_init_array());
  JSON_HWTagsArray = json_object_dotget_array(JSON_Device, "device.tagConfig.hwTags");
  
  for(i = HSD_TAGS_MAX_SW_CLASSES; i < HSD_TAGS_MAX_SW_CLASSES + HSD_TAGS_MAX_HW_CLASSES; i++)
  {        
    tempJSON1 = json_value_init_object();
    create_JSON_DeviceHWTag(i, &device->tagList.HwTag[i - HSD_TAGS_MAX_SW_CLASSES], tempJSON1);
    json_array_append_value(JSON_HWTagsArray,tempJSON1);
  }
  
  /* convert to a json string and write to file */
  if (pretty == 1)
  {
    *serialized_string = json_serialize_to_string_pretty(tempJSON);
    size = json_serialization_size_pretty(tempJSON);    
  }
  else
  {
    *serialized_string = json_serialize_to_string(tempJSON);
    size = json_serialization_size(tempJSON);    
  }
  
  json_value_free(tempJSON);
  
  return size;    
}


static int32_t get_JSON_from_DeviceInfo(COM_DeviceDescriptor_t *device_descriptor, char **serialized_string)
{
  int32_t size = 0;
  
  JSON_Value *tempJSON = json_value_init_object();
  create_JSON_DeviceInfo(device_descriptor, tempJSON);
  
  /* convert to a json string and write as string */
  *serialized_string = json_serialize_to_string(tempJSON);
  size = json_serialization_size(tempJSON); 
  
  json_value_free(tempJSON); 
  
  return size;    
}

static int32_t get_JSON_from_TagList(COM_TagList_t *tagList, char **serialized_string, uint8_t pretty)
{
  int32_t size = 0;
  
  JSON_Value *tempJSON = json_value_init_object();
  create_JSON_TagList(tagList, tempJSON);
  
  /* convert to a json string and write to file */
  if (pretty == 1)
  {
    *serialized_string = json_serialize_to_string_pretty(tempJSON);
    size = json_serialization_size_pretty(tempJSON);
  }
  else
  {
    *serialized_string = json_serialize_to_string(tempJSON);
    size = json_serialization_size(tempJSON);
  }
  
  json_value_free(tempJSON); 
  return size;
}


static int32_t get_JSON_from_Sensor(COM_Sensor_t *sensor, char **serialized_string)
{
  int32_t size = 0;
  
  JSON_Value *tempJSON = json_value_init_object();
  create_JSON_Sensor(sensor, tempJSON);
  
  /* convert to a json string and write as string */
  *serialized_string = json_serialize_to_string(tempJSON);
  size = json_serialization_size(tempJSON); 
  
  json_value_free(tempJSON); 
  
  return size;     
}


static int32_t get_JSON_from_SensorDescriptor(COM_SensorDescriptor_t *sensor_descriptor, char **serialized_string)
{
  int32_t size = 0;
  
  JSON_Value *tempJSON = json_value_init_object();
  create_JSON_SensorDescriptor(sensor_descriptor, tempJSON);
  
  /* convert to a json string and write as string */
  *serialized_string = json_serialize_to_string(tempJSON);
  size = json_serialization_size(tempJSON); 
  
  json_value_free(tempJSON); 
  
  return size;    
}


static int32_t get_JSON_from_SensorStatus(COM_SensorStatus_t *sensor_status, char **serialized_string)
{
  int32_t size = 0;
  
  JSON_Value *tempJSON = json_value_init_object();
  create_JSON_SensorStatus(sensor_status, tempJSON);
  
  /* convert to a json string and write as string */
  *serialized_string = json_serialize_to_string(tempJSON);
  size = json_serialization_size(tempJSON); 
  
  json_value_free(tempJSON); 
  
  return size;  
}


static int32_t get_JSON_from_SubSensorDescriptor(COM_SubSensorDescriptor_t *sub_sensor_descriptor, char **serialized_string)
{
  int32_t size = 0;
  
  JSON_Value *tempJSON = json_value_init_object();
  create_JSON_SubSensorDescriptor(sub_sensor_descriptor, tempJSON);
  
  /* convert to a json string and write as string */
  *serialized_string = json_serialize_to_string(tempJSON);
  size = json_serialization_size(tempJSON); 
  
  json_value_free(tempJSON); 
  
  return size;  
}

static int32_t get_JSON_from_SubSensorStatus(COM_SubSensorStatus_t *sub_sensor_status, char **serialized_string)
{
  int32_t size = 0;
  
  JSON_Value *tempJSON = json_value_init_object();
  create_JSON_SubSensorStatus(sub_sensor_status, tempJSON);
  
  /* convert to a json string and write as string */
  *serialized_string = json_serialize_to_string(tempJSON);
  size = json_serialization_size(tempJSON); 
  
  json_value_free(tempJSON); 
  
  return size;
}

static int32_t get_JSON_FWStatus(char **serialized_string, uint8_t cmd_type)
{
  int32_t size = 0;
  
  JSON_Value *tempJSON = json_value_init_object();
  
  switch(cmd_type){
    case CMD_TYPE_NETWORK:
      create_JSON_NetworkStatus(tempJSON);
      break;
    case CMD_TYPE_PERFORMANCE:
      create_JSON_PerformanceStatus(tempJSON);
      break;
    case CMD_TYPE_LOGSTATUS:
      create_JSON_IsLogging(tempJSON);
      break;
  }
  
  /* convert to a json string and write as string */
  *serialized_string = json_serialize_to_string(tempJSON);
  size = json_serialization_size(tempJSON); 
  
  json_value_free(tempJSON); 
  
  return size;
}

static int32_t get_JSON_from_AcquisitionDescriptor(COM_AcquisitionDescriptor_t *acquisition_descriptor, char **serialized_string, uint8_t pretty)
{
  int32_t size = 0;

  JSON_Value *tempJSON = json_value_init_object();

  create_JSON_AcquisitionDescriptor(acquisition_descriptor, tempJSON);

  /* convert to a json string and write to file */
  if (pretty == 1)
  {
    *serialized_string = json_serialize_to_string_pretty(tempJSON);
    size = json_serialization_size_pretty(tempJSON);
  }
  else
  {
    *serialized_string = json_serialize_to_string(tempJSON);
    size = json_serialization_size(tempJSON);
  }
  /* convert to a json string and write as string */
//  *serialized_string = json_serialize_to_string(tempJSON);
//  *serialized_string = json_serialize_to_string_pretty(tempJSON);

//  size = json_serialization_size(tempJSON);

  json_value_free(tempJSON);

  return size;
}


static int32_t parse_Device_from_JSON(char *SerializedJSON, COM_Device_t* Device)
{  
  JSON_Object *JSON_subSensorObj;
  JSON_Array *JSON_subSensorParser;
  uint32_t ii, size;
  char *StatusText;
  
  JSON_Value *tempJSON = json_parse_string(SerializedJSON);
  JSON_Object *JSON_ParseHandler = json_value_get_object(tempJSON);
  
  if (json_object_dothas_value(JSON_ParseHandler,"device.sensor"))
  {
    JSON_subSensorParser = json_object_dotget_array(JSON_ParseHandler, "device.sensor");    
    size = (uint32_t)json_array_get_count(JSON_subSensorParser);
    
    for (ii = 0; ii < size; ii++)
    {
      JSON_subSensorObj = json_array_get_object(JSON_subSensorParser,ii);
      if (json_object_dothas_value(JSON_subSensorObj,"sensorStatus"))
      {
        StatusText = json_serialize_to_string(json_object_get_value(JSON_subSensorObj,"sensorStatus"));
        parse_Status_from_JSON(StatusText, &Device->sensors[ii]->sensorStatus);
      }
    }
  }
  
  json_value_free(tempJSON);
  
  return 0;
}

static int32_t parse_Command_from_JSON(char *SerializedJSON, COM_Command_t * outCommand)
{  
  JSON_Value *tempJSON = json_parse_string(SerializedJSON);
  JSON_Object *JSON_ParseHandler = json_value_get_object(tempJSON);
  
  if (json_object_dothas_value(JSON_ParseHandler,"command"))
  {
    if (strcmp(json_object_dotget_string(JSON_ParseHandler,"command"),"GET") == 0)
    {
      outCommand->command = COM_COMMAND_GET;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"command"),"SET") == 0)
    {
      outCommand->command = COM_COMMAND_SET;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"command"),"START") == 0)
    {
      outCommand->command = COM_COMMAND_START;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"command"),"STOP") == 0)
    {
      outCommand->command = COM_COMMAND_STOP;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"command"),"SAVE") == 0)
    {
      outCommand->command = BLE_COMMAND_SAVE;
    }
    else
    {
      outCommand->command = COM_COMMAND_ERROR;
    }
  }
  else
  {
    return COM_COMMAND_ERROR;
  }
  
  if (json_object_dothas_value(JSON_ParseHandler,"request"))
  {
    if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"device") == 0)
    {
      outCommand->request = COM_REQUEST_DEVICE;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"deviceInfo") == 0)
    {
      outCommand->request = COM_REQUEST_DEVICE_INFO;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"descriptor") == 0)
    {
      outCommand->request = COM_REQUEST_DESCRIPTOR;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"status") == 0)
    {
      outCommand->request = COM_REQUEST_STATUS;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"register") == 0)
    {
      outCommand->request = COM_REQUEST_REGISTER;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"network") == 0)
    {
      outCommand->request = COM_REQUEST_STATUS_NETWORK;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"sw_tag") == 0)
    {
      outCommand->request = COM_REQUEST_SW_TAG;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"hw_tag") == 0)
    {
      outCommand->request = COM_REQUEST_HW_TAG;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"sw_tag_label") == 0)
    {
      outCommand->request = COM_REQUEST_SW_TAG_LABEL;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"hw_tag_label") == 0)
    {
      outCommand->request = COM_REQUEST_HW_TAG_LABEL;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"acq_info") == 0)
    {
      outCommand->request = COM_REQUEST_ACQ_INFO;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"tag_config") == 0)
    {
      outCommand->request = COM_REQUEST_TAG_CONFIG;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"log_status") == 0)
    {
      outCommand->request = COM_REQUEST_STATUS_LOGGING;
    }
    else if (strcmp(json_object_dotget_string(JSON_ParseHandler,"request"),"mlc_config") == 0)
    {
      outCommand->request = COM_REQUEST_MLC_CONFIG;
    }
    else
    {
      outCommand->request = COM_COMMAND_ERROR;
    }
  }
  else
  {
    outCommand->request = COM_COMMAND_ERROR;
  }  
  
  if (json_object_dothas_value(JSON_ParseHandler,"sensorId"))
  {
    outCommand->sensorId = (int8_t)json_object_dotget_number(JSON_ParseHandler, "sensorId");
  }
  else
  {
    outCommand->sensorId = COM_COMMAND_ERROR;
  }    
  
  if (json_object_dothas_value(JSON_ParseHandler,"subSensorStatus.id"))
  {
    outCommand->subSensorId = (int8_t)json_object_dotget_number(JSON_ParseHandler, "subSensorStatus.id");
  }
  else
  {
    outCommand->subSensorId = COM_COMMAND_ERROR;
  }   
  
  json_value_free(tempJSON);
  
  return 0;  
}
volatile  int8_t subid = -1;
static int32_t parse_Status_from_JSON(char *SerializedJSON, COM_SensorStatus_t * sensorStatus)
{  
  JSON_Object *JSON_subSensorObj;
  JSON_Array *JSON_subSensorParser;
  uint32_t ii, size;
  
  JSON_Value *tempJSON = json_parse_string(SerializedJSON);
  JSON_Object *JSON_ParseHandler = json_value_get_object(tempJSON);
  
//  if (json_object_dothas_value(JSON_ParseHandler,"isActive"))
//  {
//    if (json_object_dotget_boolean(JSON_ParseHandler,"isActive") == 1)
//    {
//      sensorStatus->isActive = 1;
//    }
//    else 
//    {
//      sensorStatus->isActive = 0;
//    }
//  }
  if (json_object_dothas_value(JSON_ParseHandler,"subSensorStatus"))
  {
    JSON_subSensorParser = json_object_dotget_array(JSON_ParseHandler, "subSensorStatus");    
    size = (uint32_t)json_array_get_count(JSON_subSensorParser);
    
    for (ii = 0; ii < size; ii++)
    {
      JSON_subSensorObj = json_array_get_object(JSON_subSensorParser,ii);      
      
      if (json_object_dothas_value(JSON_subSensorObj,"id"))  
      {
        subid = (int8_t)json_object_dotget_number(JSON_subSensorObj, "id");
      }
      else
      {
        subid = ii;
      }
      
      if (json_object_dothas_value(JSON_subSensorObj,"isActive"))
      {
        if (json_object_dotget_boolean(JSON_subSensorObj,"isActive") == 1)
        {
          sensorStatus->subSensorStatus[subid].isActive = 1;
        }
        else 
        {
          sensorStatus->subSensorStatus[subid].isActive = 0;
        }
      }
      
      if (json_object_dothas_value(JSON_subSensorObj,"ODR"))
      {
        sensorStatus->subSensorStatus[subid].ODR = json_object_dotget_number(JSON_subSensorObj, "ODR");
      }  
      if (json_object_dothas_value(JSON_subSensorObj,"usbDataPacketSize"))
      {
        sensorStatus->subSensorStatus[subid].usbDataPacketSize = (uint16_t) json_object_dotget_number(JSON_subSensorObj, "usbDataPacketSize");
      }  
      /* [DG] TODO: user cannot modify it ?? */
//      if (json_object_dothas_value(JSON_subSensorObj,"sdWriteBufferSize"))
//      {
//        sensorStatus->subSensorStatus[subid].sdWriteBufferSize = (uint32_t) json_object_dotget_number(JSON_subSensorObj, "sdWriteBufferSize");
//      }
      if (json_object_dothas_value(JSON_subSensorObj,"wifiDataPacketSize"))
      {
        sensorStatus->subSensorStatus[subid].wifiDataPacketSize = (uint32_t) json_object_dotget_number(JSON_subSensorObj, "wifiDataPacketSize");
      }  
      if (json_object_dothas_value(JSON_subSensorObj,"comChannelNumber"))
      {
        sensorStatus->subSensorStatus[subid].comChannelNumber = (int16_t) json_object_dotget_number(JSON_subSensorObj, "comChannelNumber");
      }  
      if (json_object_dothas_value(JSON_subSensorObj,"samplesPerTs"))
      {
        sensorStatus->subSensorStatus[subid].samplesPerTimestamp = (uint16_t)json_object_dotget_number(JSON_subSensorObj, "samplesPerTs");
      }  
      if (json_object_dothas_value(JSON_subSensorObj,"FS"))
      {
        sensorStatus->subSensorStatus[subid].FS = json_object_dotget_number(JSON_subSensorObj, "FS");
      }
      if (json_object_dothas_value(JSON_subSensorObj,"sensitivity"))
      {
        sensorStatus->subSensorStatus[subid].sensitivity = json_object_dotget_number(JSON_subSensorObj, "sensitivity");
      }
      
    }
  }  
  json_value_free(tempJSON);
  
  return 0;
}

static int32_t parse_SetDeviceAliasCommand_from_JSON(char *SerializedJSON, char *alias, uint8_t aliasSize)
{
  JSON_Value *tempJSON = json_parse_string(SerializedJSON);
  JSON_Object *JSON_ParseHandler = json_value_get_object(tempJSON);
  
  if (json_object_dothas_value(JSON_ParseHandler,"alias"))
  {
    const char* tempLabel = json_object_dotget_string(JSON_ParseHandler,"alias");
    uint8_t tempSize = strlen(tempLabel);
    if(tempSize < aliasSize)
    {
      strcpy(alias,tempLabel);
    }
    else
    {
      memcpy(alias,tempLabel,aliasSize-1);
      alias[aliasSize]='\0';
    }
  }
  else
  {
    return COM_COMMAND_ERROR;
  }
  
  json_value_free(tempJSON);
  return 0;
}

//SerializedJSON --> in
//class_id, enable -->out
static int32_t parse_EnableTagCommand_from_JSON(char *SerializedJSON, uint8_t *class_id, HSD_Tags_Enable_t *enable)
{
  JSON_Value *tempJSON = json_parse_string(SerializedJSON);
  JSON_Object *JSON_ParseHandler = json_value_get_object(tempJSON);
  
  if (json_object_dothas_value(JSON_ParseHandler,"ID"))
  {
    uint8_t temp = (uint8_t)json_object_dotget_number(JSON_ParseHandler, "ID");
    *class_id = temp;
  }
  else
  {
    return COM_COMMAND_ERROR;
  }
  
  if (json_object_dothas_value(JSON_ParseHandler, "enable"))
  {
    HSD_Tags_Enable_t tag_enable = (HSD_Tags_Enable_t)json_object_dotget_boolean(JSON_ParseHandler, "enable");
    if (tag_enable == 1)
    {
      tag_enable = HSD_TAGS_Enable;
    }
    else 
    {
      tag_enable = HSD_TAGS_Disable;
    }
    *enable = tag_enable;
  }
  else
  {
    return COM_COMMAND_ERROR;
  }
  
  json_value_free(tempJSON);
  return 0;
}

//SerializedJSON --> in
//class_id, label -->out
static int32_t parse_UpdateTagLabelCommand_from_JSON(char *SerializedJSON, uint8_t *class_id, char *label, uint8_t labelSize)
{ 
  JSON_Value *tempJSON = json_parse_string(SerializedJSON);
  JSON_Object *JSON_ParseHandler = json_value_get_object(tempJSON);
  
  if (json_object_dothas_value(JSON_ParseHandler,"ID"))
  {
    uint8_t temp = (uint8_t)json_object_dotget_number(JSON_ParseHandler, "ID");
    *class_id = temp;
  }
  else
  {
    return COM_COMMAND_ERROR;
  }
  
  if (json_object_dothas_value(JSON_ParseHandler,"label"))
  {
    const char* tempLabel = json_object_dotget_string(JSON_ParseHandler,"label");
//    *label = (char*)tempLabel;
    uint8_t tempSize = strlen(tempLabel);
    if(tempSize < labelSize)
    {
      strcpy(label,tempLabel);
    }
    else
    {
      memcpy(label,tempLabel,labelSize-1);
      label[labelSize]='\0';
    }
  }
  else
  {
    return COM_COMMAND_ERROR;
  }
  
  json_value_free(tempJSON);
  return 0;
}

//SerializedJSON --> in
//name, notes -->out
static int32_t parse_AcqInfoCommand_from_JSON(char *SerializedJSON, char *name, uint8_t nameSize, char *notes, uint8_t notesSize)
{ 
  JSON_Value *tempJSON = json_parse_string(SerializedJSON);
  JSON_Object *JSON_ParseHandler = json_value_get_object(tempJSON);
  
  if (json_object_dothas_value(JSON_ParseHandler,"name"))
  {
    const char* tempName = json_object_dotget_string(JSON_ParseHandler,"name");
    uint8_t tempSize = strlen(tempName);
    if(tempSize < nameSize)
    {
      strcpy(name,tempName);
    }
    else
    {
      memcpy(name,tempName,nameSize-2);
      name[nameSize-1]='\0';
    }
  }
  else
  {
    return COM_COMMAND_ERROR;
  }
  
  if (json_object_dothas_value(JSON_ParseHandler,"notes"))
  {
    const char* tempNotes = json_object_dotget_string(JSON_ParseHandler,"notes");
    uint8_t tempSize = strlen(tempNotes);
    if(tempSize < notesSize)
    {
      strcpy(notes,tempNotes);
    }
    else
    {
      memcpy(notes,tempNotes,notesSize-2);
      notes[notesSize-1]='\0';
    }
  }
  else
  {
    return COM_COMMAND_ERROR;
  }
  
  json_value_free(tempJSON);
  return 0;
}

static int32_t parse_MlcConfigCommand_from_JSON(char *SerializedJSON, uint32_t *mlcConfigSize, char *mlcConfigData, uint32_t mlcConfigDataSize)
{ 
  
  JSON_Object *JSON_subSensorObj;
  JSON_Array *JSON_subSensorParser;
  uint32_t ii, size;
  
  JSON_Value *tempJSON = json_parse_string(SerializedJSON);
  JSON_Object *JSON_ParseHandler = json_value_get_object(tempJSON);
  
  if (json_object_dothas_value(JSON_ParseHandler,"subSensorStatus"))
  {
    JSON_subSensorParser = json_object_dotget_array(JSON_ParseHandler, "subSensorStatus");    
    size = (uint32_t)json_array_get_count(JSON_subSensorParser);
    
    for (ii = 0; ii < size; ii++)
    {
      JSON_subSensorObj = json_array_get_object(JSON_subSensorParser,ii);      
      
//      if (json_object_dothas_value(JSON_subSensorObj,"id"))  
//      {
//        subid = (int8_t)json_object_dotget_number(JSON_subSensorObj, "id");
//      }
//      else
//      {
//        subid = ii;
//      }
      
      if (json_object_dothas_value(JSON_subSensorObj,"mlcConfigSize"))
      {
        uint32_t temp = (uint32_t)json_object_dotget_number(JSON_subSensorObj,"mlcConfigSize");
        *mlcConfigSize = temp;
      }
      else
      {
        return COM_COMMAND_ERROR;
      }
      if (json_object_dothas_value(JSON_subSensorObj,"mlcConfigData"))
      {
        const char* tempData = json_object_dotget_string(JSON_subSensorObj,"mlcConfigData");
        uint32_t tempSize = strlen(tempData);
        if(tempSize < mlcConfigDataSize)
        {
          strcpy(mlcConfigData,tempData);
        }
        else
        {
          memcpy(mlcConfigData,tempData,mlcConfigDataSize-2);
          mlcConfigData[mlcConfigDataSize-1]='\0';
        }
      }
      else
      {
        return COM_COMMAND_ERROR;
      }
    }
  }
  
  json_value_free(tempJSON);
  return 0;
}

static void create_JSON_DeviceInfo(COM_DeviceDescriptor_t *device_descriptor, JSON_Value *tempJSON)
{
  JSON_Object *JSON_DeviceHandler = json_value_get_object(tempJSON);
  
  json_object_dotset_string(JSON_DeviceHandler, "deviceInfo.serialNumber", device_descriptor->serialNumber);
  json_object_dotset_string(JSON_DeviceHandler, "deviceInfo.alias", device_descriptor->alias);  
  json_object_dotset_string(JSON_DeviceHandler, "deviceInfo.partNumber", device_descriptor->partNumber);//"STEVAL-STWINKT1"
  json_object_dotset_string(JSON_DeviceHandler, "deviceInfo.URL", device_descriptor->URL);//"www.st.com/stwin"
  json_object_dotset_string(JSON_DeviceHandler, "deviceInfo.fwName", device_descriptor->fwName);//"HSDatalog"
  json_object_dotset_string(JSON_DeviceHandler, "deviceInfo.fwName", device_descriptor->fwVersion);//"2.1.0"
  json_object_dotset_string(JSON_DeviceHandler, "deviceInfo.dataFileExt", device_descriptor->dataFileExt);
  json_object_dotset_string(JSON_DeviceHandler, "deviceInfo.dataFileFormat", device_descriptor->dataFileFormat);
  //json_object_dotset_number(JSON_DeviceHandler, "deviceInfo.nSensor", device_descriptor->nSensor);
}

static void create_JSON_TagList(COM_TagList_t *tagList, JSON_Value *tempJSON)
{
  JSON_Object *JSON_TagList = json_value_get_object(tempJSON);
  JSON_Array *JSON_SWTagsArray;
  JSON_Array *JSON_HWTagsArray;
  JSON_Value *tempJSON1;

  json_object_dotset_number(JSON_TagList, "tagConfig.maxTagsPerAcq", HSD_TAGS_MAX_PER_ACQUISITION);
  
  json_object_dotset_value(JSON_TagList, "tagConfig.swTags", json_value_init_array());
  JSON_SWTagsArray = json_object_dotget_array(JSON_TagList, "tagConfig.swTags");
  
  int i;
  for(i = 0; i < HSD_TAGS_MAX_SW_CLASSES; i++)
  {        
    tempJSON1 = json_value_init_object();  
    create_JSON_DeviceSWTag(i,tagList->HSD_SwTagClasses[i], tempJSON1);    
    json_array_append_value(JSON_SWTagsArray,tempJSON1);
  } 
  
  json_object_dotset_value(JSON_TagList, "tagConfig.hwTags", json_value_init_array());
  JSON_HWTagsArray = json_object_dotget_array(JSON_TagList, "tagConfig.hwTags");
  
  for(i = HSD_TAGS_MAX_SW_CLASSES; i < HSD_TAGS_MAX_SW_CLASSES + HSD_TAGS_MAX_HW_CLASSES; i++)
  {
    tempJSON1 = json_value_init_object();
    create_JSON_DeviceHWTag(i, &tagList->HwTag[i - HSD_TAGS_MAX_SW_CLASSES], tempJSON1);
    json_array_append_value(JSON_HWTagsArray,tempJSON1);
  }
}

static void create_JSON_Sensor(COM_Sensor_t *sensor, JSON_Value *tempJSON)
{
  JSON_Value *tempJSON1;
  JSON_Value *tempJSON2;
  JSON_Array *JSON_SensorArray1;
  JSON_Array *JSON_SensorArray2;
  uint32_t ii = 0;
  uint8_t nSubSensors = sensor->sensorDescriptor.nSubSensors; 
  uint8_t nSensor = sensor->sensorDescriptor.id;
  
  JSON_Object *JSON_Sensor = json_value_get_object(tempJSON);
  
  json_object_dotset_number(JSON_Sensor, "id", nSensor);
  json_object_dotset_string(JSON_Sensor, "name", sensor->sensorDescriptor.name);  
  
  JSON_Value *DescriptorJSON = json_value_init_object();
  json_object_set_value(JSON_Sensor, "sensorDescriptor", DescriptorJSON);
  create_JSON_SensorDescriptor(&sensor->sensorDescriptor, DescriptorJSON); 
  
  json_object_dotset_value(JSON_Sensor, "sensorDescriptor.subSensorDescriptor", json_value_init_array());
  
  JSON_SensorArray1 = json_object_dotget_array(JSON_Sensor, "sensorDescriptor.subSensorDescriptor"); 
  for (ii = 0; ii < nSubSensors; ii++)
  {
    tempJSON1 = json_value_init_object();  
    create_JSON_SubSensorDescriptor(&sensor->sensorDescriptor.subSensorDescriptor[ii], tempJSON1);  
    json_array_append_value(JSON_SensorArray1,tempJSON1);
  }
  
  JSON_Value *statusJSON = json_value_init_object();
  json_object_set_value(JSON_Sensor, "sensorStatus", statusJSON);
  create_JSON_SensorStatus(&sensor->sensorStatus, statusJSON); 
  
  json_object_dotset_value(JSON_Sensor, "sensorStatus.subSensorStatus", json_value_init_array());
  JSON_SensorArray2= json_object_dotget_array(JSON_Sensor, "sensorStatus.subSensorStatus"); 
  for (ii = 0; ii < nSubSensors; ii++)
  {
    tempJSON2 = json_value_init_object();  
    create_JSON_SubSensorStatus(&sensor->sensorStatus.subSensorStatus[ii], tempJSON2);    
    json_array_append_value(JSON_SensorArray2,tempJSON2);
  }
}


static void create_JSON_SensorDescriptor(COM_SensorDescriptor_t *sensor_descriptor, JSON_Value *tempJSON)
{
  uint32_t ii = 0;
  
  JSON_Object *JSON_SensorDescriptor = json_value_get_object(tempJSON);
  JSON_Array *JSON_SensorArray1;
  JSON_Value *tempJSON1;
  
//  json_object_dotset_string(JSON_SensorDescriptor, "name", sensor_descriptor->name);  
  
//  switch (sensor_descriptor->dataType)
//  {
//  case DATA_TYPE_UINT8:
//    json_object_dotset_string(JSON_SensorDescriptor, "dataType", "uint8_t");
//    break;
//  case DATA_TYPE_INT8:
//    json_object_dotset_string(JSON_SensorDescriptor, "dataType", "int8_t");
//    break;
//  case DATA_TYPE_UINT16:
//    json_object_dotset_string(JSON_SensorDescriptor, "dataType", "uint16_t");
//    break;
//  case DATA_TYPE_INT16:
//    json_object_dotset_string(JSON_SensorDescriptor, "dataType", "int16_t");
//    break;
//  case DATA_TYPE_UINT32:
//    json_object_dotset_string(JSON_SensorDescriptor, "dataType", "uint32_t");
//    break;
//  case DATA_TYPE_INT32:
//    json_object_dotset_string(JSON_SensorDescriptor, "dataType", "int32_t");
//    break;
//  case DATA_TYPE_FLOAT:
//    json_object_dotset_string(JSON_SensorDescriptor, "dataType", "float");
//    break;
//  default:
//    json_object_dotset_string(JSON_SensorDescriptor, "dataType", "NA");
//    break;
//  }  
//  
//  json_object_dotset_number(JSON_SensorDescriptor, "samplesPerTs.min", sensor_descriptor->samplesPerTimestamp[0]);  
//  json_object_dotset_number(JSON_SensorDescriptor, "samplesPerTs.max", sensor_descriptor->samplesPerTimestamp[1]);  
//  json_object_dotset_string(JSON_SensorDescriptor, "samplesPerTs.dataType", "int16_t");  
//  
//  json_object_dotset_value(JSON_SensorDescriptor, "ODR.values", json_value_init_array());
//  JSON_Array *JSON_SensorArray = json_object_dotget_array(JSON_SensorDescriptor, "ODR.values");
//  
//  while (sensor_descriptor->ODR[ii] > 0)
//  {
//    json_array_append_number(JSON_SensorArray, sensor_descriptor->ODR[ii]);
//    ii++;
//  }
  
  json_object_dotset_value(JSON_SensorDescriptor, "subSensorDescriptor", json_value_init_array());  
  JSON_SensorArray1 = json_object_dotget_array(JSON_SensorDescriptor, "subSensorDescriptor"); 
  for (ii = 0; ii < sensor_descriptor->nSubSensors; ii++)
  {
    tempJSON1 = json_value_init_object();  
    create_JSON_SubSensorDescriptor(&sensor_descriptor->subSensorDescriptor[ii], tempJSON1);  
    json_array_append_value(JSON_SensorArray1,tempJSON1);
  }  
}


static void create_JSON_SensorStatus(COM_SensorStatus_t *sensor_status, JSON_Value *tempJSON)
{
  //EMPTY @ the moment
  
//  JSON_Object *JSON_SensorStatus= json_value_get_object(tempJSON);
  
//  json_object_dotset_number(JSON_SensorStatus, "ODR", sensor_status->ODR);
//  json_object_dotset_number(JSON_SensorStatus, "ODRMeasured", sensor_status->measuredODR);
//  json_object_dotset_number(JSON_SensorStatus, "initialOffset", sensor_status->initialOffset);
//  json_object_dotset_boolean(JSON_SensorStatus, "isActive", sensor_status->isActive);
//  json_object_dotset_number(JSON_SensorStatus, "samplesPerTs", sensor_status->samplesPerTimestamp);
//  json_object_dotset_number(JSON_SensorStatus, "usbDataPacketSize", sensor_status->usbDataPacketSize);
//  json_object_dotset_number(JSON_SensorStatus, "sdWriteBufferSize", sensor_status->sdWriteBufferSize);
//  json_object_dotset_number(JSON_SensorStatus, "comChannelNumber", sensor_status->comChannelNumber);
}


static void create_JSON_SubSensorDescriptor(COM_SubSensorDescriptor_t *sub_sensor_descriptor, JSON_Value *tempJSON)
{
  uint32_t ii = 0;
  
  JSON_Value *tempJSONarray = json_value_init_object();
  JSON_Array *JSON_SensorArray = json_value_get_array(tempJSONarray);
  JSON_Object *JSON_SubSensorDescriptor= json_value_get_object(tempJSON);
  
  //id
  json_object_dotset_number(JSON_SubSensorDescriptor, "id", sub_sensor_descriptor->id);
  //sensorType
  switch (sub_sensor_descriptor->sensorType)
  {
  case COM_TYPE_ACC:
    json_object_dotset_string(JSON_SubSensorDescriptor, "sensorType", "ACC");
    break;
  case COM_TYPE_MAG:
    json_object_dotset_string(JSON_SubSensorDescriptor, "sensorType", "MAG");
    break;
  case COM_TYPE_GYRO:
    json_object_dotset_string(JSON_SubSensorDescriptor, "sensorType", "GYRO");
    break;
  case COM_TYPE_TEMP:
    json_object_dotset_string(JSON_SubSensorDescriptor, "sensorType", "TEMP");
    break;
  case COM_TYPE_PRESS:
    json_object_dotset_string(JSON_SubSensorDescriptor, "sensorType", "PRESS");
    break;
  case COM_TYPE_HUM:
    json_object_dotset_string(JSON_SubSensorDescriptor, "sensorType", "HUM");
    break;
  case COM_TYPE_MIC:
    json_object_dotset_string(JSON_SubSensorDescriptor, "sensorType", "MIC");
    break;
  case COM_TYPE_MLC:
    json_object_dotset_string(JSON_SubSensorDescriptor, "sensorType", "MLC");
    break;
  default:
    json_object_dotset_string(JSON_SubSensorDescriptor, "sensorType", "NA");
    break;      
  } 
  //dimensions
  json_object_dotset_number(JSON_SubSensorDescriptor, "dimensions", sub_sensor_descriptor->dimensions);
  //dimensionsLabel
  json_object_dotset_value(JSON_SubSensorDescriptor, "dimensionsLabel", json_value_init_array());
  JSON_SensorArray = json_object_dotget_array(JSON_SubSensorDescriptor, "dimensionsLabel");
  for (ii=0; ii < sub_sensor_descriptor->dimensions; ii++)
  {
    json_array_append_string(JSON_SensorArray, sub_sensor_descriptor->dimensionsLabel[ii]);
  }
  //unit
  json_object_dotset_string(JSON_SubSensorDescriptor, "unit", sub_sensor_descriptor->unit);
  //dataType
  switch (sub_sensor_descriptor->dataType)
  {
  case DATA_TYPE_UINT8:
    json_object_dotset_string(JSON_SubSensorDescriptor, "dataType", "uint8_t");
    break;
  case DATA_TYPE_INT8:
    json_object_dotset_string(JSON_SubSensorDescriptor, "dataType", "int8_t");
    break;
  case DATA_TYPE_UINT16:
    json_object_dotset_string(JSON_SubSensorDescriptor, "dataType", "uint16_t");
    break;
  case DATA_TYPE_INT16:
    json_object_dotset_string(JSON_SubSensorDescriptor, "dataType", "int16_t");
    break;
  case DATA_TYPE_UINT32:
    json_object_dotset_string(JSON_SubSensorDescriptor, "dataType", "uint32_t");
    break;
  case DATA_TYPE_INT32:
    json_object_dotset_string(JSON_SubSensorDescriptor, "dataType", "int32_t");
    break;
  case DATA_TYPE_FLOAT:
    json_object_dotset_string(JSON_SubSensorDescriptor, "dataType", "float");
    break;
  default:
    json_object_dotset_string(JSON_SubSensorDescriptor, "dataType", "NA");
    break;
  }
  ii=0;
  //FS
  json_object_dotset_value(JSON_SubSensorDescriptor, "FS", json_value_init_array());
  JSON_SensorArray = json_object_dotget_array(JSON_SubSensorDescriptor, "FS");
  while (sub_sensor_descriptor->FS[ii] > 0)
  {
    json_array_append_number(JSON_SensorArray, sub_sensor_descriptor->FS[ii]);
    ii++;
  }
  
  ii=0;
  //ODR
  json_object_dotset_value(JSON_SubSensorDescriptor, "ODR", json_value_init_array());
  JSON_SensorArray = json_object_dotget_array(JSON_SubSensorDescriptor, "ODR");
  while (sub_sensor_descriptor->ODR[ii] > 0)
  {
    json_array_append_number(JSON_SensorArray, sub_sensor_descriptor->ODR[ii]);
    ii++;
  }
  //samplePerTs
  json_object_dotset_number(JSON_SubSensorDescriptor, "samplesPerTs.min", sub_sensor_descriptor->samplesPerTimestamp[0]);  
  json_object_dotset_number(JSON_SubSensorDescriptor, "samplesPerTs.max", sub_sensor_descriptor->samplesPerTimestamp[1]);  
  json_object_dotset_string(JSON_SubSensorDescriptor, "samplesPerTs.dataType", "int16_t");
    
  json_value_free(tempJSONarray);
}


static void create_JSON_SubSensorStatus(COM_SubSensorStatus_t *sub_sensor_status, JSON_Value *tempJSON)
{
  JSON_Object *JSON_SubSensorStatus = json_value_get_object(tempJSON);
  
  json_object_dotset_number(JSON_SubSensorStatus, "ODR", sub_sensor_status->ODR);
  json_object_dotset_number(JSON_SubSensorStatus, "ODRMeasured", sub_sensor_status->measuredODR);
  json_object_dotset_number(JSON_SubSensorStatus, "initialOffset", PRECISION6(sub_sensor_status->initialOffset));
  json_object_dotset_number(JSON_SubSensorStatus, "FS", sub_sensor_status->FS);
  json_object_dotset_number(JSON_SubSensorStatus, "sensitivity", PRECISION6(sub_sensor_status->sensitivity));
  json_object_dotset_boolean(JSON_SubSensorStatus, "isActive", sub_sensor_status->isActive);
  json_object_dotset_number(JSON_SubSensorStatus, "samplesPerTs", sub_sensor_status->samplesPerTimestamp);
  json_object_dotset_number(JSON_SubSensorStatus, "usbDataPacketSize", sub_sensor_status->usbDataPacketSize);
  json_object_dotset_number(JSON_SubSensorStatus, "sdWriteBufferSize", sub_sensor_status->sdWriteBufferSize);
  json_object_dotset_number(JSON_SubSensorStatus, "wifiDataPacketSize", sub_sensor_status->wifiDataPacketSize);
  json_object_dotset_number(JSON_SubSensorStatus, "comChannelNumber", sub_sensor_status->comChannelNumber);
}

static void create_JSON_PerformanceStatus(JSON_Value *tempJSON)
{
  JSON_Object *JSON_PerfStatus = json_value_get_object(tempJSON);
  
  uint32_t mV = 0;
  uint32_t level = 0;
  uint16_t cpu_usage;
  
  BSP_BC_GetVoltageAndLevel(&mV, &level);
  
  cpu_usage = osGetCPUUsage();
  
  json_object_dotset_string(JSON_PerfStatus, "command", "STATUS");
  json_object_dotset_string(JSON_PerfStatus, "type", "performance");
  json_object_dotset_number(JSON_PerfStatus, "cpuUsage", (double)cpu_usage);
  json_object_dotset_number(JSON_PerfStatus, "batteryVoltage", mV);
  json_object_dotset_number(JSON_PerfStatus, "batteryLevel", level);
}

static void create_JSON_IsLogging(JSON_Value *tempJSON)
{
  JSON_Object *JSON_IsLogging = json_value_get_object(tempJSON);
  
  json_object_dotset_string(JSON_IsLogging, "command", "STATUS");
  json_object_dotset_string(JSON_IsLogging, "type", "logstatus");
  json_object_dotset_boolean(JSON_IsLogging, "isLogging", SD_Logging_Active);
  json_object_dotset_boolean(JSON_IsLogging, "isSDInserted", BSP_SD_IsDetected());
}

static void create_JSON_NetworkStatus(JSON_Value *tempJSON)
{
  JSON_Object *JSON_NetworkStatus = json_value_get_object(tempJSON);

  json_object_dotset_string(JSON_NetworkStatus, "command", "STATUS");
  json_object_dotset_string(JSON_NetworkStatus, "type", "network");
  json_object_dotset_string(JSON_NetworkStatus, "ssid", "net_ssid");
  json_object_dotset_string(JSON_NetworkStatus, "password", "net_password");
  json_object_dotset_string(JSON_NetworkStatus, "ip", "unavailable");
}

static void create_JSON_DeviceHWTag(uint8_t id, const COM_HwTag_t *hwTag, JSON_Value *tempJSON)
{
  JSON_Object *JSON_DeviceHWTag = json_value_get_object(tempJSON);
  
  json_object_dotset_number(JSON_DeviceHWTag, "id", id);
  json_object_dotset_string(JSON_DeviceHWTag, "pinDesc", hwTag->pinDesc);
  json_object_dotset_string(JSON_DeviceHWTag, "label", hwTag->label);
  json_object_dotset_boolean(JSON_DeviceHWTag, "enabled", hwTag->enabled);
}

uint8_t idTest;
char *labelTest;
static void create_JSON_DeviceSWTag(uint8_t id,char *label, JSON_Value *tempJSON)
{
  JSON_Object *JSON_DeviceSWTag = json_value_get_object(tempJSON);
  idTest = id;
  labelTest = label;
  
  json_object_dotset_number(JSON_DeviceSWTag, "id", id);
  json_object_dotset_string(JSON_DeviceSWTag, "label", label);
}

static void create_JSON_AcquisitionDescriptor(COM_AcquisitionDescriptor_t *acquisition_descriptor, JSON_Value *tempJSON)
{
  JSON_Object *JSON_AcquisitionDescriptor = json_value_get_object(tempJSON);
  JSON_Array *tmp_array;
  JSON_Value *tmp_value;
  JSON_Object *tmp_obj;

  HSD_Tags_Type_t type;
  uint8_t class_id;
  HSD_Tags_Enable_t enable;
  double time_stamp;

  json_object_dotset_string(JSON_AcquisitionDescriptor, "Name", acquisition_descriptor->name);
  json_object_dotset_string(JSON_AcquisitionDescriptor, "Description", acquisition_descriptor->description);
  json_object_dotset_string(JSON_AcquisitionDescriptor, "UUIDAcquisition", acquisition_descriptor->UUIDAcquisition);

  /* init and get an array for Tags */
  json_object_dotset_value(JSON_AcquisitionDescriptor, "Tags", json_value_init_array());
  tmp_array = json_object_dotget_array(JSON_AcquisitionDescriptor, "Tags");

//  char ts[30];
  /* Create an array item for each tag */
  while(HSD_TAGS_get_tag(&type, &class_id, &enable, &time_stamp) == 0)
  {
    tmp_value = json_value_init_object();
    tmp_obj = json_value_get_object(tmp_value);

    json_object_dotset_number(tmp_obj, "t", time_stamp);
    //	sprintf(ts,"%.2f",time_stamp);
    //	json_object_dotset_string(tmp_obj, "t", ts);
    json_object_dotset_string(tmp_obj, "Label", HSD_TAGS_get_tag_label(COM_GetDevice(), type, class_id));
    json_object_dotset_boolean(tmp_obj, "Enable", (int)enable);

    json_array_append_value(tmp_array, tmp_value);
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
