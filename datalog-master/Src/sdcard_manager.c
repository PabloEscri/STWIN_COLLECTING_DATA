/**
******************************************************************************
* @file    sdcard_manager.c
* @author  SRA
* @version v3.0.0
* @date    19-Jun-2020
* @brief   This file provides a set of functions to handle SDcard communication
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
#include "sdcard_manager.h"
#include "main.h"
#include "com_manager.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdio.h"

#include "sensors_manager.h"

#include "mp23abs1_app.h"
#include "imp34dt05_app.h"
#include "ism330dhcx_app.h"
#include "iis3dwb_app.h"
#include "iis2mdc_app.h"
#include "iis2dh_app.h"
#include "hts221_app.h"
#include "lps22hh_app.h"
#include "stts751_app.h"

#include "HSD_json.h"

#include "bluenrg1_gatt_aci.h"
#include "stm32l4xx_hal_rtc.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define LOG_DIR_PREFIX    "STWIN_"

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL FileConfigHandler;
FIL FileLogError;
char SDPath[4]; /* SD card logical drive path */

SD_HandleTypeDef hsd1;

uint8_t SD_Logging_Active = 0;
uint8_t SD_present = 0;
uint8_t init_SD_peripheral = 0;
uint8_t ConfigFromSD = 0;

uint32_t t_start = 0;

extern RTC_HandleTypeDef  hrtc;

volatile uint8_t BatteryLow = 0;

#if (HSD_SD_LOGGING_MODE == HSD_SD_LOGGING_MODE_INTERMITTENT)
uint32_t SD_Logging_Time_Start=0;
uint32_t SD_Logging_Enabled = 0;
#endif

osSemaphoreId sdioSem_id;
osSemaphoreDef(sdioSem);

osMessageQId sdThreadQueue_id;
osMessageQDef(sdThreadQueue, 100, int);

extern osSemaphoreId bleSendThreadSem_id;
extern uint8_t commandMask;

volatile uint8_t MLC_loaded = 0;
static stmdev_ctx_t MLC_ctx_instance = {SM_SPI_Write_Os, SM_SPI_Read_Os, &ism330dhcx_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId SDM_Thread_Id;
static void SDM_Thread(void const *argument);

static void _Error_Handler( void );

static void Enable_Sensors(void);
static void Activate_Sensor(uint32_t id);

static void SDM_Boot(void);
static void SDM_DataReady(osEvent evt);
static void SDM_StartStopAcquisition(void);
static void SDM_StartAcquisition(void);
static void SDM_StopAcquisition(void);
static uint8_t checkRootFolder(void);
static uint8_t checkConfigJson(FILINFO fno);
static uint8_t checkConfigUcf(FILINFO fno);
static uint32_t readUCFfromSD(char *MLC_string);

uint8_t SDM_Memory_Init(void);
uint8_t SDM_Memory_Deinit(void);
uint32_t SDM_GetLastDirNumber(void);
static uint32_t SDM_SaveData(void);
static uint32_t SDM_SaveDeviceConfig(char *dir_name);
static uint32_t SDM_SaveAcquisitionInfo(char *dir_name);
static uint32_t SDM_SaveUCF(char *dir_name);

static void switchOff_LowBattery(void);


/*----------------------------------------------------------------------------*/
/**
* @brief  Default enabled sensors
* @param  None 
* @retval None
*/
void Enable_Sensors(void)
{  
  /* Comment or uncomment each of the following lines
  * to chose which sensor you want to log.         */
  
  Activate_Sensor(hts221_com_id);
  Activate_Sensor(iis2dh_com_id);
  Activate_Sensor(iis2mdc_com_id);
  Activate_Sensor(iis3dwb_com_id);
  Activate_Sensor(imp34dt05_com_id);
  Activate_Sensor(ism330dhcx_com_id);
  Activate_Sensor(lps22hh_com_id);
  Activate_Sensor(mp23abs1_com_id);
  Activate_Sensor(stts751_com_id);
}


/**
* @brief  Activate the sensor
* @param  id: sensor id 
* @retval None
*/
void Activate_Sensor(uint32_t id)
{
  COM_SensorStatus_t *pSensorStatus = COM_GetSensorStatus(id);
  COM_SensorDescriptor_t *pSensorDescriptor = COM_GetSensorDescriptor(id);
  uint8_t i = 0;
  
  if (id == ism330dhcx_com_id)  /* MLC subsensor should never start by default */
  {
    for(i = 0; i<2; i++)
    {
      pSensorStatus->subSensorStatus[i].isActive = 1;
    }    
  }
  else
  {
    for(i = 0; i< pSensorDescriptor->nSubSensors; i++)
    {
      pSensorStatus->subSensorStatus[i].isActive = 1;
    }
  }
}


/*----------------------------------------------------------------------------*/
/**
* @brief  SD card main thread
* @retval None
*/
static void SDM_Thread(void const *argument)
{
  (void)argument;
  osEvent evt;
  
  SDM_Boot();
  
#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_SDM_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_SDM_DEBUG_PIN );
#endif
  for (;;)
  {
    BSP_LED_Off(LED1); 
    
    /* If the battery is too low close the file and turn off the system */
    if(BatteryLow == 1)
    {
      switchOff_LowBattery();
    }    
    evt = osMessageGet(sdThreadQueue_id, osWaitForever);  /* wait for message */
    
    if (com_status == HS_DATALOG_IDLE || com_status == HS_DATALOG_SD_STARTED)
    {      
      BSP_LED_On(LED1);
      
      if (evt.status == osEventMessage)
      {
        if(evt.value.v == SDM_START_STOP)
        {
          SDM_StartStopAcquisition();
        }
        else if(evt.value.v & SDM_DATA_READY_MASK)
        {
          SDM_DataReady(evt);     
        }
        else
        {
          
        }
      }
    }
  }
}


/**
* @brief  Check if SD Card is inserted and seach for possible config files
* @param  None 
* @retval None
*/
static void SDM_Boot(void)
{
  if (BSP_SD_IsDetected())
  {        
    if (init_SD_peripheral != 1)
    {
      SDM_SD_Init();
      init_SD_peripheral = 1;
    }    
    
    ConfigFromSD = checkRootFolder();
    
    if (ConfigFromSD < SDM_JSON_CONFIG)
    {
      Enable_Sensors();   
      update_sensors_config();
    }    
    if (init_SD_peripheral != 0)
    {
      SDM_SD_DeInit();
      init_SD_peripheral = 0;
    }   
  }  
}

/**
* @brief  Handle SDM_START_STOP task message
* @param  None 
* @retval None
*/
static void SDM_StartStopAcquisition(void)
{
  if(SD_Logging_Active == 0)
  {
    SDM_StartAcquisition();
  }
  else if (SD_Logging_Active == 1)
  {
    SDM_StopAcquisition();
  }  
}

static void SDM_StartAcquisition(void)
{
  com_status = HS_DATALOG_SD_STARTED;            
  SM_TIM_Start();

  COM_GenerateAcquisitionUUID();
  
  if (BSP_SD_IsDetected())
  {              
    if (init_SD_peripheral != 1)
    {
      SDM_SD_Init();
      init_SD_peripheral = 1;
    }
    SD_present = 1;
    if(SDM_InitFiles() == 0)
    {
      SD_Logging_Active = 1;
      BSP_LED_Off(LED_ORANGE);
    }
    HSD_TAGS_timer_start();
  }
  else
  {
    SD_present = 0;
  }  
}

static void SDM_StopAcquisition(void)
{
  SM_TIM_Stop();
  HSD_TAGS_timer_stop();
  
  if(SDM_CloseFiles() == 0)
  {
    SD_Logging_Active = 0;
    commandMask = COM_REQUEST_DEVICEREFRESH;
    
#if (HSD_BLE_ENABLE == 1)
    osSemaphoreRelease(bleSendThreadSem_id);
#endif
  }
  
  if (init_SD_peripheral != 0)
  {
    SDM_SD_DeInit();
    init_SD_peripheral = 0;
  }
  com_status = HS_DATALOG_IDLE;  
}

/**
* @brief  Handle SDM_DATA_READY_MASK task message
* @param  None 
* @retval None
*/
static void SDM_DataReady(osEvent evt)
{
  COM_SubSensorStatus_t *pSubSensorStatus;
  COM_SubSensorContext_t *pSubSensorContext;
  uint32_t buf_size;
  uint8_t sID = (uint8_t)(evt.value.v & SDM_SENSOR_ID_MASK);
  uint8_t ssID = (uint8_t)((evt.value.v & SDM_SUBSENSOR_ID_MASK)>>8);
  
  pSubSensorStatus = COM_GetSubSensorStatus(sID, ssID);
  pSubSensorContext = COM_GetSubSensorContext(sID, ssID);
  
  buf_size = pSubSensorStatus->sdWriteBufferSize;
  
  if(evt.value.v & SDM_DATA_FIRST_HALF_MASK) /* Data available on first half of the circular buffer */
  {
    SDM_WriteBuffer(sID, ssID, pSubSensorContext->sd_write_buffer, buf_size);
  }
  else /* Data available on second half of the circular buffer */
  {
    SDM_WriteBuffer(sID, ssID, (uint8_t *)(pSubSensorContext->sd_write_buffer+buf_size), buf_size);
  } 
}

/**
* @brief  Check if a custom configuration JSON or UCF is available in the root folder of the SD Card
* @param  None 
* @retval 1 if there is a Config JSON, else 0
*/
uint8_t checkRootFolder(void)
{
  DIR dir;
  static FILINFO fno;
  uint8_t isJSON = 7;
  uint8_t isMLC = 7;
  uint8_t ret = 0;
  
  (void)f_opendir(&dir, "/");                   /* Open the root directory */
  
  for (;;) 
  {
    (void)f_readdir(&dir, &fno);                /* Read files in root folder */
    if (fno.fname[0] == 0) break;
    
    if (fno.fattrib & AM_ARC)                   /* It is a file. */
    { 
      char * pch = NULL;
      char local_name[50];      
      strncpy(local_name, &fno.fname[0], 50);   /* Copy file name */
      pch = strtok(local_name," .-,_\r\n");     /* Exclude separators from the search */
      
      while (pch != NULL)                       /* Check files until the end of file list */
      {
        isJSON = strncmp(pch, "json", 4); 
        if (isJSON == 0)                        /* file name has a 'json' extension */
        {
          ret += checkConfigJson(fno);
        }
        
        isMLC = strncmp(pch, "ucf", 3);
        if (isMLC == 0)                         /* file name has a 'ucf' extension*/
        {
          ret += checkConfigUcf(fno);
        }
        pch = strtok(NULL," .-,_\r\n");          
      }
    }
  }
  f_closedir(&dir);
  return ret;
}


/**
* @brief  Open and parse the configuration JSON and update the device model
* @param  FILINFO fno 
* @retval SDM_JSON_CONFIG if JSON is opened and parsed, else SDM_DEFAULT_CONFIG
*/
uint8_t checkConfigJson(FILINFO fno)
{
  uint8_t ret = SDM_DEFAULT_CONFIG;
  FIL FileConfigJSON;
  int checkDeviceConfig = strncmp(fno.fname, "DeviceConfig.json", 17);  /* Check if JSON name is DeviceConfig.json */
  
  if (checkDeviceConfig == 0) /* 0 -> Valid file name -> Load JSON */
  {
    if(f_open(&FileConfigJSON, fno.fname, FA_OPEN_EXISTING | FA_READ) == FR_OK) /* Open JSON file */
    {  
      char* config_JSON_string = NULL;
      int sizeFile;
      UINT br;
      sizeFile = f_size(&FileConfigJSON)+1;
      config_JSON_string = HSD_malloc(sizeFile);
      f_read(&FileConfigJSON, config_JSON_string, sizeFile, &br); /* Read the file */
      SDM_ReadJSON(config_JSON_string);                           /* Parse and update */
      HSD_JSON_free(config_JSON_string);
      config_JSON_string = NULL;
      f_close(&FileConfigJSON);
      ret = SDM_JSON_CONFIG;
    }
  }
  return ret;
}


/**
* @brief  Open and read the UCF file and load the MLC configuration
* @param  FILINFO fno 
* @retval SDM_MLC_CONFIG if MLC id configurated, else SDM_DEFAULT_CONFIG
*/
uint8_t checkConfigUcf(FILINFO fno)
{
  uint8_t ret = SDM_DEFAULT_CONFIG;
  FIL FileConfigMLC;
  if(f_open(&FileConfigMLC, fno.fname, FA_OPEN_EXISTING | FA_READ) == FR_OK) /* Open UCF file */
  {
    char* config_MLC_string = NULL;
    int sizeFile;
    UINT br;
    sizeFile = f_size(&FileConfigMLC)+1;
    config_MLC_string = HSD_malloc(sizeFile);
    f_read(&FileConfigMLC, config_MLC_string, sizeFile, &br);   /* Read the file */
    readUCFfromSD(config_MLC_string);                         /* Load MLC configuration */
    HSD_free(config_MLC_string);
    config_MLC_string = NULL;
    f_close(&FileConfigMLC);
    ret = SDM_MLC_CONFIG;
  }
  return ret;
}


/**
* @brief  When battery is low, save the last available data and switch off the board
* @param  None 
* @retval None
*/
void switchOff_LowBattery(void)
{     
  SM_TIM_Stop();
  if(SDM_CloseFiles() == 0)
  {
    SD_Logging_Active = 0;
  }
  
  if (init_SD_peripheral != 0)
  {
    SDM_SD_DeInit();
    init_SD_peripheral = 0;
  }
  BSP_BC_CmdSend(SHIPPING_MODE_ON);
}


/**
* @brief  PWR PVD interrupt callback
* @param  None 
* @retval None
*/
void HAL_PWR_PVDCallback(void)
{
  BatteryLow = 1;
}


/**
* @brief  SD Card Manager memory initialization. Performs the dinamic allocation for
*         the sd_write_buffer associated to each active subsensor.
* @param  
* @retval 1: no error
*/
uint8_t SDM_Memory_Init(void)
{
  COM_DeviceDescriptor_t *pDeviceDescriptor = COM_GetDeviceDescriptor();
  COM_SensorDescriptor_t *pSensorDescriptor;
  COM_SubSensorStatus_t *pSubSensorStatus;
  COM_SubSensorContext_t *pSubSensorContext;
  uint32_t sID, ssID;
  
  for(sID=0;sID<pDeviceDescriptor->nSensor;sID++)
  {
    pSensorDescriptor = COM_GetSensorDescriptor(sID);

    for(ssID=0;ssID<pSensorDescriptor->nSubSensors;ssID++)
    {
      pSubSensorStatus = COM_GetSubSensorStatus(sID, ssID);
      pSubSensorContext = COM_GetSubSensorContext(sID, ssID);
      if(pSubSensorStatus->isActive)
      {
        pSubSensorContext->sd_write_buffer = HSD_malloc(pSubSensorStatus->sdWriteBufferSize*2);
        if(pSubSensorContext->sd_write_buffer == 0)
        {
          _Error_Handler();
        }
      }
      else
      {
        pSubSensorContext->sd_write_buffer = 0;
      }
    }
  }
  
  return 1;
}


/**
* @brief  SD Card Manager memory De-initialization.
* @param  
* @retval 1: no error
*/
uint8_t SDM_Memory_Deinit(void)
{
  COM_DeviceDescriptor_t *pDeviceDescriptor = COM_GetDeviceDescriptor();
  COM_SensorDescriptor_t *pSensorDescriptor;
  COM_SubSensorStatus_t *pSubSensorStatus;
  COM_SubSensorContext_t *pSubSensorContext;
  uint32_t sID, ssID;
  
  for(sID=0;sID<pDeviceDescriptor->nSensor;sID++)
  {
    pSensorDescriptor = COM_GetSensorDescriptor(sID);

    for(ssID=0;ssID<pSensorDescriptor->nSubSensors;ssID++)
    {
      pSubSensorStatus = COM_GetSubSensorStatus(sID, ssID);
      pSubSensorContext = COM_GetSubSensorContext(sID, ssID);
      if(pSubSensorStatus->isActive && pSubSensorContext->sd_write_buffer!=0)
      {
        HSD_free(pSubSensorContext->sd_write_buffer);
        pSubSensorContext->sd_write_buffer = NULL;
      }
    }
  }
  
  return 1;
}


/**
* @brief  SDM_Peripheral_Init
* @param  None 
* @retval None
*/
void SDM_Peripheral_Init(void)
{
  BSP_SD_Detect_Init();   
}


/**
* @brief  Initialize SD Card Manager thread and queue
* @param  None
* @retval None
*/
void SDM_OS_Init(void)
{
  sdioSem_id = osSemaphoreCreate(osSemaphore(sdioSem), 1);
  osSemaphoreWait(sdioSem_id, osWaitForever);
  
  sdThreadQueue_id = osMessageCreate(osMessageQ(sdThreadQueue), NULL);  
  vQueueAddToRegistry( sdThreadQueue_id, "sdThreadQueue_id" );
  
  /* Thread definition: read data */
//  osThreadDef(SDManager_Thread, SDM_Thread, SD_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE*8);
  osThreadDef(SDManager_Thread, SDM_Thread, SD_THREAD_PRIO, 1, 3800/4);
  
  /* Start thread 1 */
  SDM_Thread_Id = osThreadCreate(osThread(SDManager_Thread), NULL);
}


/**
* @brief  Initialize SD Card and file system
* @param  None
* @retval None
*/
void SDM_SD_Init(void)
{
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      while(1)
      {
        BSP_LED_On(LED1);
        HAL_Delay(500);
        BSP_LED_Off(LED1);
        HAL_Delay(100);
      }
    }
  }
}


/**
* @brief  Deinitialize SD Card and file system
* @param  None
* @retval None
*/
void SDM_SD_DeInit(void)
{
  if(FATFS_UnLinkDriver(SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if(f_mount(NULL, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      while(1)
      {
        BSP_LED_On(LED1);
        HAL_Delay(500);
        BSP_LED_Off(LED1);
        HAL_Delay(100);
      }
    }
  }
}


/**
* @brief  Open error file
* @param  name: name of the log file
* @retval 1 for f_write error, else 0
*/
uint8_t SDM_OpenLogErrorFile(const char *name)
{
  uint32_t byteswritten;
  
  if(f_open(&FileLogError, (char const*)name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 1;
  }   
  if(f_write(&FileLogError, "WARNING: possible data loss at samples [seconds]: ", 50, (void *)&byteswritten) != FR_OK)
  {
    return 1;
  }
  return 0;
}


/**
* @brief  PWR PVD interrupt callback
* @param  None 
* @retval 1 for f_write error, else 0
*/
uint8_t SDM_OpenDatFile(uint8_t sID, uint8_t ssID, const char *sensorName)
{
  char file_name[50];

  FIL *p = &(COM_GetSubSensorContext(sID, ssID)->file_handler);
  sprintf(file_name, "%s%s", sensorName, ".dat");
  
  if(f_open(p, (const char*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 1;
  }  
  return 0;
}


uint8_t SDM_CloseFile(uint8_t sID, uint8_t ssID)
{
  FIL *p = &(COM_GetSubSensorContext(sID, ssID)->file_handler);
  return f_close(p);
}


/**
* @brief  Scan SD Card file system to find the latest directory number that includes to the LOG_DIR_PREFIX
* @param  None
* @retval 
*/
uint32_t SDM_GetLastDirNumber(void)
{
  FRESULT fr;     /* Return value */
  DIR dj;         /* Directory search object */
  FILINFO fno;    /* File information */
  int dir_n = 0, tmp;
  char dir_name[sizeof(LOG_DIR_PREFIX)+1] = LOG_DIR_PREFIX;
  
  dir_name[sizeof(LOG_DIR_PREFIX)-1] = '*';  /* wildcard */
  dir_name[sizeof(LOG_DIR_PREFIX)] = 0;
  
  fr = f_findfirst(&dj, &fno, "", dir_name);  /* Start to search for matching directories */
  if(fno.fname[0])
  {
    tmp = strtol(&fno.fname[sizeof(LOG_DIR_PREFIX)],NULL,10);
    if(dir_n<tmp)
    {
      dir_n = tmp;
    }
  }
  
  /* Repeat while an item is found */
  while (fr == FR_OK && fno.fname[0])
  {
    fr = f_findnext(&dj, &fno);   /* Search for next item */
    if(fno.fname[0])
    {
      tmp = strtol(&fno.fname[sizeof(LOG_DIR_PREFIX)],NULL,10);
      if(tmp > dir_n)
      {
        dir_n = tmp;
      }
    }
  }
  
  f_closedir(&dj);
  return (uint32_t)dir_n;
}


/**
* @brief  Open one file for each subsensor to store raw data and a JSON file with the device configuration
* @param  None
* @retval 1 for f_write error, else 0
*/
uint8_t SDM_InitFiles(void)
{
  COM_DeviceDescriptor_t *pDeviceDescriptor;
  COM_SensorDescriptor_t *pSensorDescriptor;

  uint8_t sensorIsActive;
  uint32_t sID=0, ssID=0, dir_n=0;
  char dir_name[sizeof(LOG_DIR_PREFIX)+4];
  char file_name[50];
  
  pDeviceDescriptor = COM_GetDeviceDescriptor();
  dir_n = SDM_GetLastDirNumber();
  dir_n++;
  
  sprintf(dir_name, "%s%03ld", LOG_DIR_PREFIX, dir_n);
  
  FRESULT test = f_mkdir(dir_name);
  if(test != FR_OK)
  {
    return 1;
  }
  
  for(sID=0;sID<pDeviceDescriptor->nSensor;sID++)
  {
    pSensorDescriptor = COM_GetSensorDescriptor(sID);
    
    for(ssID=0;ssID<pSensorDescriptor->nSubSensors;ssID++)
    {
      if(COM_GetSubSensorStatus(sID, ssID)->isActive)
      {
        char subSensorName[6];
        switch (pSensorDescriptor->subSensorDescriptor[ssID].sensorType)
        {
        case COM_TYPE_ACC:
          sprintf(subSensorName,"ACC");
          break;
        case COM_TYPE_MAG:
          sprintf(subSensorName,"MAG");
          break;
        case COM_TYPE_GYRO:
          sprintf(subSensorName,"GYRO");
          break;
        case COM_TYPE_TEMP:
          sprintf(subSensorName,"TEMP");
          break;
        case COM_TYPE_PRESS:
          sprintf(subSensorName,"PRESS");
          break;
        case COM_TYPE_HUM:
          sprintf(subSensorName,"HUM");
          break;
        case COM_TYPE_MIC:
          sprintf(subSensorName,"MIC");
          break;
        case COM_TYPE_MLC:
          sprintf(subSensorName,"MLC");
          break;
        default:
          sprintf(subSensorName,"NA");
          break;      
        } 
        sprintf(file_name, "%s/%s_%s", dir_name, pSensorDescriptor->name, subSensorName);
        if(SDM_OpenDatFile(sID, ssID, file_name)!=0)
        {
          return 1;
        }
      }
    }
  }  
  SDM_Memory_Init();
  
  for(sID=0;sID<pDeviceDescriptor->nSensor;sID++)
  {
    sensorIsActive = 0;    
    pSensorDescriptor = COM_GetSensorDescriptor(sID);

    for(ssID=0;ssID<pSensorDescriptor->nSubSensors;ssID++)
    {
      COM_ResetSubSensorContext(sID, ssID);      
      sensorIsActive |= COM_GetSubSensorStatus(sID, ssID)->isActive;
    }
    /* Sensor is Active if at least one of the SubSensor is ative */
    if(sensorIsActive)
    {
      SM_StartSensorThread(sID);
    } 
  }
  return 0;
}

uint8_t SDM_UpdateDeviceConfig(void)
{
  FIL fil;        /* File object */
  FRESULT fr;     /* FatFs return code */
  char* JSON_string = NULL;
  uint32_t byteswritten, size;
  
  SDM_SD_Init();

  fr = f_open(&fil, "DeviceConfig.json", FA_OPEN_ALWAYS | FA_WRITE);
  if(fr != FR_OK)
  {
    return 1;
  }

  size = SDM_CreateJSON(&JSON_string);
  fr = f_write(&fil, (uint8_t*)JSON_string, size-1, (void *)&byteswritten);
  if(fr != FR_OK)
  {
    return 1;
  }
  
  fr = f_close(&fil);
  if (fr != FR_OK)
  {
    return 1;
  }
  
  SDM_SD_DeInit();
  return fr;
}


/**
* @brief  Close all files
* @param  None 
* @retval 1 for f_write error, else 0
*/
uint8_t SDM_CloseFiles(void)
{
  char dir_name[sizeof(LOG_DIR_PREFIX)+4];
  uint32_t dir_n = 0;
  
  /* Put all the sensors in "SUSPENDED" mode, write and close all data files */
  if (SDM_SaveData())
  {
    return 1;
  }  
  
  dir_n = SDM_GetLastDirNumber();
  sprintf(dir_name, "%s%03ld", LOG_DIR_PREFIX, dir_n);
  
  /* Write DeviceConfig.json */
  if (SDM_SaveDeviceConfig(dir_name))
  {
    return 1;
  }  
  /* Write Data Tags */
  if (SDM_SaveAcquisitionInfo(dir_name))
  {
    return 1;
  }  
  /* Copy UCF file in the Acquisition folder */
  if (SDM_SaveUCF(dir_name))
  {
    return 1;
  }
  
  return 0;
}

static uint32_t SDM_SaveData(void)
{
  COM_DeviceDescriptor_t *pDeviceDescriptor = COM_GetDeviceDescriptor();
  COM_SensorDescriptor_t *pSensorDescriptor;
  uint32_t i, n = 0;
  
  for(i=0;i<pDeviceDescriptor->nSensor;i++)
  {
    pSensorDescriptor = COM_GetSensorDescriptor(i);

    for(n=0;n<pSensorDescriptor->nSubSensors;n++)
    {
      if(COM_GetSubSensorStatus(i, n)->isActive)
      {
        SM_StopSensorThread(i);
      }
    }
  }

    /* Put all the sensors in "SUSPENDED" mode */
  for(i=0;i<pDeviceDescriptor->nSensor;i++)  
  {
    pSensorDescriptor = COM_GetSensorDescriptor(i);

    for(n=0;n<pSensorDescriptor->nSubSensors;n++)
    {
      if(COM_GetSubSensorStatus(i, n)->isActive)
      {
        SDM_Flush_Buffer(i, n);
        if(SDM_CloseFile(i, n)!=0)
        {
          return 1;
        }
      }
    }
  }
  
  /* Deallocate here SD buffers to have enough memory for next section */
  SDM_Memory_Deinit();
  return 0;  
}

static uint32_t SDM_SaveDeviceConfig(char *dir_name)
{
  char* JSON_string = NULL;
  uint32_t byteswritten, size;
  char file_name[50]; 
  
  sprintf(file_name, "%s/DeviceConfig.json", dir_name);
  
  if(f_open(&FileConfigHandler, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 1;
  }  
  size = SDM_CreateJSON(&JSON_string);
  if(f_write(&FileConfigHandler, (uint8_t*)JSON_string, size-1, (void *)&byteswritten) != FR_OK)
  {
    return 1;
  }  
  if (f_close(&FileConfigHandler)!= FR_OK)
  {
    return 1;
  }
  
  HSD_JSON_free(JSON_string);
  JSON_string = NULL;    
  return 0;
}

static uint32_t SDM_SaveAcquisitionInfo(char *dir_name) 
{  
  char* JSON_string = NULL;
  uint32_t byteswritten, size;
  char file_name[50]; 
  
  sprintf(file_name, "%s/AcquisitionInfo.json", dir_name);
  
  if(f_open(&FileConfigHandler, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 1;
  }  
  size = SDM_CreateAcquisitionJSON(&JSON_string);  
  if(f_write(&FileConfigHandler, (uint8_t*)JSON_string, size-1, (void *)&byteswritten) != FR_OK)
  {
    return 1;
  }  
  if (f_close(&FileConfigHandler)!= FR_OK)
  {
    return 1;
  }
  
  HSD_JSON_free(JSON_string);
  JSON_string = NULL;
  return 0;
}

static uint32_t SDM_SaveUCF(char *dir_name)
{  
  char file_name[50]; 
  FILINFO fno;    /* File information */
  DIR dir;
  uint8_t isMLC = 7; 
  
  (void)f_opendir(&dir, "/");                   /* Open the root directory */
  
  for (;;) 
  {
    (void)f_readdir(&dir, &fno);                /* Read files in root folder */
    if (fno.fname[0] == 0) break;
    
    if (fno.fattrib & AM_ARC)                   /* It is a file. */
    { 
      char * pch = NULL;
      char local_name[50];
      strncpy(local_name, &fno.fname[0], 50);  /* Copy file name */
      pch = strtok(local_name," .-,_\r\n");     /* Exclude separators from the search */
      
      while (pch != NULL)                       /* Check files until the end of file list */
      {        
        isMLC = strncmp(pch, "ucf", 3);
        if (isMLC == 0)                         /* file name has a 'ucf' extension*/
        {
          FIL FileSourceConfigMLC;
          FIL FileDestConfigMLC;
          BYTE buffer[50];                      /* File copy buffer */
          UINT br, bw;                          /* File read/write count */
          
          sprintf(file_name, "%s/%s", dir_name, fno.fname);
          if(f_open(&FileSourceConfigMLC, fno.fname, FA_OPEN_EXISTING | FA_READ) == FR_OK) /* Open UCF file */
          {
            f_open(&FileDestConfigMLC, file_name, FA_WRITE | FA_CREATE_ALWAYS);
            /* Copy source to destination */
            for (;;) 
            {
              f_read(&FileSourceConfigMLC, buffer, sizeof buffer, &br);  /* Read a chunk of data from the source file */
              if (br == 0) break; /* error or eof */
              f_write(&FileDestConfigMLC, buffer, br, &bw);            /* Write it to the destination file */
              if (bw < br) break; /* error or disk full */
            }
            f_close(&FileSourceConfigMLC);
            f_close(&FileDestConfigMLC);
          }          
        }
        pch = strtok(NULL," .-,_\r\n");          
      }
    }
  }
  f_closedir(&dir);  
  return 0;
}

/**
* @brief  
* @param  
* @retval 
*/
uint8_t SDM_WriteConfigBuffer(uint8_t *buffer, uint32_t size)
{
  uint32_t byteswritten;
  FRESULT returnWrite;
  
  returnWrite = f_write(&FileConfigHandler, buffer, size, (void *)&byteswritten);
  if(returnWrite != FR_OK)
  {
    return 0;
  }  
  return 1;
}


/**
* @brief  Write data buffer to SD card
* @param  None 
* @retval 1 for f_write error, else 0
*/
extern Char_UUID_t char_uuid;
extern uint16_t ConfigServW2STHandle;
extern uint16_t ConfigCharHandle;
extern uint16_t char_mia;

uint8_t SDM_WriteBuffer(uint8_t sID, uint8_t ssID, uint8_t *buffer, uint32_t size)
{
  uint32_t byteswritten;
  FIL *p = &(COM_GetSubSensorContext(sID, ssID)->file_handler);

  if(f_write(p, buffer, size, (void *)&byteswritten) != FR_OK)
  {

    return 1;
 }
/*  RTC_TimeTypeDef sTime;
  HAL_RTC_GetTime(&hrtc, &sTime, 0);
  uint8_t  header[55];
uint8_t size2 = sprintf(header,"%d:%d:%d",sTime.Hours,sTime.Minutes, sTime.Seconds);*/
  uint8_t vector[10];
 // vector[0] = buffer[0];
  //vector[1] = buffer[1];
 //uint8_t sizeBuffer = sprintf(vector, "%s", *buffer);
  static uint16_t contador_aux=0;
  double timestan =  SM_GetTimeStamp();
  uint8_t size2 = sprintf(vector, "%lf", timestan);
 //aci_gatt_update_char_value(ConfigServW2STHandle, char_mia, 0, 2,vector);
  //	if(contador_aux==50){
  		aci_gatt_update_char_value(ConfigServW2STHandle, char_mia, 0, size2,vector);

  		//aci_gatt_update_char_value(ConfigServW2STHandle, char_mia, 0, 1,vector);
  		//aci_gatt_update_char_value(ConfigServW2STHandle, char_mia, 0, size2,header);
  	//	contador_aux=0;
  //	}

  return 0;
}


/**
* @brief  Write down all the data available
* @param  id: sensor id 
* @retval 1 for f_write error, else 0
*/
/* Write remaining data to file */
uint8_t SDM_Flush_Buffer(uint8_t sID, uint8_t ssID)
{
  uint8_t ret = 1;
  uint32_t bufSize = COM_GetSubSensorStatus(sID, ssID)->sdWriteBufferSize;
  COM_SubSensorContext_t *pSubSensorContext = COM_GetSubSensorContext(sID, ssID);
  
  if(pSubSensorContext->sd_write_buffer_idx>0 && pSubSensorContext->sd_write_buffer_idx<(bufSize-1))
  {
    /* flush from the beginning */
    ret = SDM_WriteBuffer(sID, ssID, pSubSensorContext->sd_write_buffer, pSubSensorContext->sd_write_buffer_idx+1);
  }
  else if (pSubSensorContext->sd_write_buffer_idx>(bufSize-1) && pSubSensorContext->sd_write_buffer_idx<(bufSize*2-1))
  {
    /* flush from half buffer */
    ret =  SDM_WriteBuffer(sID, ssID, (uint8_t *)(pSubSensorContext->sd_write_buffer+bufSize), pSubSensorContext->sd_write_buffer_idx+1-bufSize);
  }
  
  pSubSensorContext->sd_write_buffer_idx = 0;
  return ret;
}


/**
* @brief  Fill SD buffer with new data
* @param  id: sensor id 
* @param  src: pointer to data buffer 
* @param  srcSize: buffer size
* @retval 0: ok
*/
uint8_t SDM_Fill_Buffer(uint8_t sID, uint8_t ssID, uint8_t *src, uint16_t srcSize)
{
  uint8_t *dst;
  uint32_t dstP, srcP=0;
  uint32_t dstSize, bufSize;
  COM_SubSensorContext_t *pSubSensorContext = COM_GetSubSensorContext(sID, ssID);

  bufSize = COM_GetSubSensorStatus(sID, ssID)->sdWriteBufferSize;
  dstSize = bufSize*2;

  dst = pSubSensorContext->sd_write_buffer;
  dstP = pSubSensorContext->sd_write_buffer_idx;
  
  /* byte per byte copy to SD buffer, automatic wrap */
  while(srcP < srcSize)
  {
    dst[dstP] = src[srcP];
    dstP++;
    srcP++;
    if(dstP>=dstSize)
    {
      dstP=0;
    }
  }
  if(pSubSensorContext->sd_write_buffer_idx < (dstSize/2) && dstP>=(dstSize/2)) /* first half full */
  {           
    /* unlock write task */
    if(osMessagePut(sdThreadQueue_id, sID|ssID<<8|SDM_DATA_READY_MASK|SDM_DATA_FIRST_HALF_MASK, 0) != osOK)
    {
      _Error_Handler();
    }
    /* check for buffer consistency */
  }
  else if(dstP < pSubSensorContext->sd_write_buffer_idx)  /* second half full */
  {
    if(osMessagePut(sdThreadQueue_id, sID|ssID<<8|SDM_DATA_READY_MASK|SDM_DATA_SECOND_HALF_MASK, 0) != osOK)
    {
      _Error_Handler();
    }
  }

  pSubSensorContext->sd_write_buffer_idx = dstP;
  return 0;
}


/**
* @brief  Read and parse Json string and update device model
* @param  serialized_string: pointer to Json string 
* @retval 0: ok
*/
uint32_t SDM_ReadJSON(char *serialized_string)
{  
  static COM_Device_t JSON_device; 
  COM_Device_t *local_device; 
  uint8_t ii;
  uint32_t size;
  
  local_device = COM_GetDevice();  
  size = sizeof(COM_Device_t);
  
  memcpy(&JSON_device, local_device, size);       
  HSD_JSON_parse_Device(serialized_string, &JSON_device);      
  
  for (ii = 0; ii < JSON_device.deviceDescriptor.nSensor; ii++)
  {
    update_sensorStatus(&local_device->sensors[ii]->sensorStatus, &JSON_device.sensors[ii]->sensorStatus, ii);
  }
  
  update_sensors_config();    
  return 0;  
}


/**
* @brief  Serialize Json string from device model
* @param  serialized_string: double pointer to json string 
* @retval size of the string
*/
uint32_t SDM_CreateJSON(char **serialized_string)
{  
  COM_Device_t *device; 
  uint32_t size;
  
  device = COM_GetDevice();  
  size = HSD_JSON_serialize_Device(device, serialized_string, PRETTY_JSON);
  
  return size;
}


/**
* @brief  
* @param  
* @retval size of the string
*/
uint32_t SDM_CreateAcquisitionJSON(char **serialized_string)
{
  COM_AcquisitionDescriptor_t *acquisition;
  uint32_t size;
  
  acquisition = COM_GetAcquisitionDescriptor();
  size = HSD_JSON_serialize_Acquisition(acquisition, serialized_string, PRETTY_JSON);
  
  return size;
}



uint32_t readUCFfromSD(char *MLC_string)
{
  int ucf_reg;
  int ucf_data;
  char * pch = NULL;
  
  pch = strtok(MLC_string," -,_\r\n");
  while (pch != NULL)
  {
    if(strncmp(pch, "Ac", 2) == 0)
    {
      pch = strtok(NULL, " -,_\r\n");
      ucf_reg = strtol(pch, NULL, 16);
      pch = strtok(NULL, " -,_\r\n");
      ucf_data = strtol(pch, NULL, 16);
      ism330dhcx_write_reg(&MLC_ctx_instance, (uint8_t)ucf_reg, (uint8_t*)&ucf_data, 1);
    }
    pch = strtok(NULL, " -,_\r\n");
  }
  MLC_loaded = 1;
  
  return 0;
}


void SDM_WriteUCF(char* ucfData, uint32_t ucfSize)
{
  uint32_t byteswritten;
  FIL FileConfigMLC;
  
  if (BSP_SD_IsDetected())
  {        
    if (init_SD_peripheral != 1)
    {
      SDM_SD_Init();
      init_SD_peripheral = 1;
    }    
    
    f_open(&FileConfigMLC, "ISM330DHCX_MLC.ucf", FA_CREATE_ALWAYS | FA_WRITE);
    f_write(&FileConfigMLC, (uint8_t*)ucfData, 9*(ucfSize/4), (void *)&byteswritten);
    f_close(&FileConfigMLC);
  }   
  
  if (init_SD_peripheral != 0)
  {
    SDM_SD_DeInit();
    init_SD_peripheral = 0;
  } 
}


#if (HSD_SD_LOGGING_MODE == HSD_SD_LOGGING_MODE_INTERMITTENT)

/**
* @brief  Close and open file automatically every HSD_LOGGING_TIME_SECONDS
* @param  None 
* @retval None
*/
void SDM_AutosaveFile(void)
{
  if (SD_Logging_Active)
  {
    if( (HAL_GetTick() - SD_Logging_Time_Start) > HSD_LOGGING_TIME_SECONDS_ACTIVE*1000 )
    {
      // Cannot wait since we are in an ISR
      if(osMessagePut(sdThreadQueue_id, SDM_START_STOP, 0) != osOK)
      {
        _Error_Handler();
      }      
      t_start = SD_Logging_Time_Start = HAL_GetTick();
    }
  }
  else
  {
    if( (HAL_GetTick() - SD_Logging_Time_Start) > HSD_LOGGING_TIME_SECONDS_IDLE*1000 )
    {
      // Cannot wait since we are in an ISR
      if(osMessagePut(sdThreadQueue_id, SDM_START_STOP, 0) != osOK)
      {
        _Error_Handler();
      }      
      t_start = SD_Logging_Time_Start = HAL_GetTick();
    }
  }
}
#endif    


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


/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
