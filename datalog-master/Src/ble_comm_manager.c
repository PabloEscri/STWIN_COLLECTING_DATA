/**
  ******************************************************************************
  * @file    ble_comm_manager.c
  * @author  SRA
  * @version v1.1.0
  * @date    12-Sep-19
  * @brief   This file provides a set of functions to handle ble communication
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
#include "ble_comm_manager.h"
#include "main.h"
#include "hci_tl_interface.h"
#include "hci_tl.h"
#include "hci.h"
#include "ble_status.h"
#include "bluenrg1_types.h"
#include "sensors_manager.h"
#include "com_manager.h"
#include "HSD_json.h"
#include "ble_comm_transfer_protocol.h"
#include "bluenrg1_gatt_aci.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

#ifndef MIN
#define MIN(a,b)            ((a) < (b) )? (a) : (b)
#endif

/* Private variables ---------------------------------------------------------*/

extern uint16_t configServiceHandle;
extern uint16_t configTxCharHandle;

extern uint8_t ble_init;
SPI_HandleTypeDef hble_cm_spi;

extern uint8_t SD_Logging_Active;
extern uint8_t commandMask;
/* Private function prototypes -----------------------------------------------*/

static void BLE_CM_SPI_MspInit(SPI_HandleTypeDef *hspi);
void ble_interface_init(void);

static void ble_user_evt_proc_Thread(void const *argument);
osThreadId bleUserEvtProcThreadId;

static void ble_send_Thread(void const *argument);
osThreadId bleSendThreadId;

static void ble_init_Thread(void const *argument);
osThreadId bleInitThreadId;

void bleSendPerformanceStatusCallback(void const * argument);

#if (HSD_BLE_STATUS_TIMER_ENABLE == 1)
osTimerId bleSendPerformanceStatusTim_id;
osTimerDef(bleSendPerformanceStatusTim,bleSendPerformanceStatusCallback);
#endif

osSemaphoreId bleInitThreadSem_id;
osSemaphoreDef(bleInitThreadSem);

osSemaphoreId bleUserEvtProcThreadSem_id;
osSemaphoreDef(bleUserEvtProcThreadSem);

osSemaphoreId bleSendThreadSem_id;
osSemaphoreDef(bleSendThreadSem);

osSemaphoreId bleCongestionSem_id;
osSemaphoreDef(bleCongestionSem);

/**
* @brief Sensor manager SPI Initialization Function
* @param None
* @retval None
* @note callbacks to the MSP
*/
void BLE_CM_SPI_Init(void)
{  
  /* SPI3 parameter configuration*/
  hble_cm_spi.Instance = BLE_CM_SPI_x;
  hble_cm_spi.Init.Mode = SPI_MODE_MASTER;
  hble_cm_spi.Init.Direction = SPI_DIRECTION_2LINES;
  hble_cm_spi.Init.DataSize = SPI_DATASIZE_8BIT;
  hble_cm_spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hble_cm_spi.Init.CLKPhase = SPI_PHASE_2EDGE;
  hble_cm_spi.Init.NSS = SPI_NSS_SOFT;
  hble_cm_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; 
  hble_cm_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hble_cm_spi.Init.TIMode = SPI_TIMODE_DISABLE;
  hble_cm_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hble_cm_spi.Init.CRCPolynomial = 7;
  hble_cm_spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hble_cm_spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  
  /* Register MSP Callback */
  HAL_SPI_RegisterCallback(&hble_cm_spi, HAL_SPI_MSPINIT_CB_ID, BLE_CM_SPI_MspInit);
  
  if (HAL_SPI_Init(&hble_cm_spi) != HAL_OK)
  {
    SM_Error_Handler();
  }
}

void BLE_CM_SPI_DeInit(void)
{
  HAL_GPIO_DeInit(BLE_CM_SPI_EXTI_PORT, BLE_CM_SPI_EXTI_PIN); 
  HAL_GPIO_DeInit(BLE_CM_SPI_CS_PORT, BLE_CM_SPI_CS_PIN); 
  HAL_GPIO_DeInit(BLE_CM_RST_PORT, BLE_CM_RST_PIN); 
}

void BLE_CM_SPI_Reset(void)
{
  /* Deselect CS PIN for BlueNRG to avoid spurious commands */
  HAL_GPIO_WritePin(BLE_CM_SPI_CS_PORT, BLE_CM_SPI_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BLE_CM_RST_PORT, BLE_CM_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(BLE_CM_RST_PORT, BLE_CM_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(5);    
} 

void BLE_CM_Error_Handler( void )
{
  while (1)
  {}
}

/**
* @brief Sensor manager OS functionalities initialization: for each BUS (I2C and SPI) it
*        initializes a queue to collect read request, a thread (blocking on the queue) to handle
*        read requests and a semaphore used to wait for DMA transfer complete
* @param None
* @retval None
*/
void BLE_CM_OS_Init(void)
{
  /* Bus read semaphores */
  bleInitThreadSem_id = osSemaphoreCreate(osSemaphore(bleInitThreadSem), 1);
  osSemaphoreWait(bleInitThreadSem_id,osWaitForever);
  
  bleUserEvtProcThreadSem_id = osSemaphoreCreate(osSemaphore(bleUserEvtProcThreadSem), 1);
  osSemaphoreWait(bleUserEvtProcThreadSem_id,osWaitForever);
  
  bleSendThreadSem_id = osSemaphoreCreate(osSemaphore(bleSendThreadSem), 1);
  osSemaphoreWait(bleSendThreadSem_id,osWaitForever);
  
  bleCongestionSem_id = osSemaphoreCreate(osSemaphore(bleCongestionSem), 1);
  osSemaphoreWait(bleCongestionSem_id,osWaitForever);

#if (HSD_BLE_STATUS_TIMER_ENABLE == 1)
  /* BLE Performance Status send Timer*/
  bleSendPerformanceStatusTim_id = osTimerCreate(osTimer(bleSendPerformanceStatusTim), osTimerPeriodic, NULL);
#endif
  
  /* BLE init Thread*/
  osThreadDef(BLE_INIT_THREAD, ble_init_Thread, BLE_INIT_THREAD_PRIO, 1, 1024/4);
  
  /* BLE init read Thread */
  bleInitThreadId = osThreadCreate(osThread(BLE_INIT_THREAD), NULL);
  
  /* BLE send Thread*/
  osThreadDef(BLE_SEND_THREAD, ble_send_Thread, BLE_SEND_THREAD_PRIO, 1, 2048/4);
  
  /* BLE send Thread */
  bleSendThreadId = osThreadCreate(osThread(BLE_SEND_THREAD), NULL);
  
  /* Task to process user events*/
  osThreadDef(BLE_USER_EVT_PROC_THREAD, ble_user_evt_proc_Thread, BLE_USER_EVT_PROC_THREAD_PRIO, 1, 1024/4);
  
  /* Start User Events process Thread */
  bleUserEvtProcThreadId = osThreadCreate(osThread(BLE_USER_EVT_PROC_THREAD), NULL);
}


/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BLE_CM_SPI_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t len)
{
  int32_t ret = -1;
  
  if(HAL_SPI_TransmitReceive(&hble_cm_spi, pTxData, pRxData, len, 1000) == HAL_OK)
  {
      ret = (int32_t)len;
  }
  return ret;
}


int32_t BLE_CM_SPI_Write(void * handle, uint8_t reg, uint8_t * data, uint16_t len)
{
  taskENTER_CRITICAL();

  taskEXIT_CRITICAL();
  return 0;
}

/**
* @brief Sensor Manager Blocking SPI read function
* @param None
* @retval None
* @note This function can be liked to the sensor PID if freeRTOS is not used
*/
int32_t BLE_CM_SPI_Read(void * handle, uint8_t reg, uint8_t * data, uint16_t len)
{
  return 0;
}

/**
* @brief  SPI read function: it adds a request on the SPI read queue (which will be handled by the SPI read thread)
* @param  argument not used
* @note when the function is used and linked to the sensor context, all the calls made by the PID driver will result in a
*       call to this function. If this is the case, be sure to make all the calls to the PID driver functions from a freeRTOS thread
* @retval None
*/
// int32_t SM_SPI_Read_Os(void * handle, uint8_t reg, uint8_t * data, uint16_t len)
// {

// }

// int32_t SM_SPI_Write_Os(void * handle, uint8_t reg, uint8_t * data, uint16_t len)
// {

// }


/* Performance Status Callback function */
void bleSendPerformanceStatusCallback(void const * argument)
{
  commandMask = COM_REQUEST_STATUS_PERFORMANCE;
  osSemaphoreRelease(bleSendThreadSem_id);
}

/**
* @brief  BLE receiver thread
* @param  argument not used
* @retval None
*/
static void ble_init_Thread(void const *argument)
{
  (void)argument;
 
  #if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_BLE_INIT_DEBUG_PIN))
    vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_BLE_INIT_DEBUG_PIN );
  #endif
  
  for(;;)
  {
    if (ble_init == 1)
    {
      osSemaphoreWait(bleInitThreadSem_id,osWaitForever);
    }
    ble_interface_init();
  }
}

/**
* @brief  BLE User Events process thread
* @param  argument not used
* @retval None
*/
static void ble_user_evt_proc_Thread(void const *argument)
{ 
  (void)argument;
 
  #if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_BLE_USER_EVT_PROC_DEBUG_PIN))
    vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_BLE_USER_EVT_PROC_DEBUG_PIN );
  #endif

  for (;;)
  {    
    osSemaphoreWait(bleUserEvtProcThreadSem_id, osWaitForever);
    
    if(ble_init)
      {
        hci_user_evt_proc();
      }
  }
}

void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle, uint16_t Available_Buffers)
{
  osSemaphoreRelease(bleCongestionSem_id);
}

/**
* @brief  BLE send thread
* @param  argument not used
* @retval None
*/
static void ble_send_Thread(void const *argument)
{
  (void)argument;
  
  #if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_BLE_SEND_DEBUG_PIN))
    vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_BLE_SEND_DEBUG_PIN );
  #endif
  
  for (;;)
  {
    osSemaphoreWait(bleSendThreadSem_id,osWaitForever);
    
    uint8_t ret = 0;
    uint32_t j = 0, len = 0, tot_len = 0;
    int32_t JSON_string_command_BufLen = 0;
    uint32_t length_wTP = 0;
    char* JSON_string_command = NULL;

    //json stringifyed length + current device configuration (json file serialized)
    switch(commandMask)
    {
      case COM_REQUEST_DEVICE:
      {
        if (SD_Logging_Active == 0)
        {
          COM_Device_t *device = COM_GetDevice();
          JSON_string_command_BufLen = HSD_JSON_serialize_Device(device, &JSON_string_command, SHORT_JSON);
        }
        else 
        {
          JSON_string_command_BufLen = HSD_JSON_serialize_FWStatus(&JSON_string_command,CMD_TYPE_LOGSTATUS);
        }
        break;
      }
      case COM_REQUEST_DEVICEREFRESH:
      {
        COM_Device_t *device = COM_GetDevice();
        JSON_string_command_BufLen = HSD_JSON_serialize_Device(device, &JSON_string_command, SHORT_JSON);
        break;
      }
      case COM_REQUEST_DEVICE_INFO:
      {
        COM_DeviceDescriptor_t *deviceInfo = COM_GetDeviceDescriptor();
        JSON_string_command_BufLen = HSD_JSON_serialize_DeviceInfo(deviceInfo, &JSON_string_command);
        break;
      }
      case COM_REQUEST_DESCRIPTOR:
      {
        break;
      }
      case COM_REQUEST_STATUS:
      {
        break;
      }
      case COM_REQUEST_REGISTER:
      {
        break;
      }
      case COM_REQUEST_STATUS_PERFORMANCE:
      {
        JSON_string_command_BufLen = HSD_JSON_serialize_FWStatus(&JSON_string_command,CMD_TYPE_PERFORMANCE);
        break;
      }
      case COM_REQUEST_STATUS_LOGGING:
      {
        JSON_string_command_BufLen = HSD_JSON_serialize_FWStatus(&JSON_string_command,CMD_TYPE_LOGSTATUS);
        break;
      }
      case COM_REQUEST_STATUS_NETWORK:
      {
        JSON_string_command_BufLen = HSD_JSON_serialize_FWStatus(&JSON_string_command,CMD_TYPE_NETWORK);
        break;
      }
      case COM_REQUEST_TAG_CONFIG:
      {
        COM_TagList_t *tagConfig = COM_GetTagList();
        JSON_string_command_BufLen = HSD_JSON_serialize_TagList(tagConfig, &JSON_string_command, SHORT_JSON);
        break;
      }

    }    
    if (JSON_string_command_BufLen % 19 == 0)
      length_wTP = (JSON_string_command_BufLen/19)+JSON_string_command_BufLen;
    else{
      length_wTP = (JSON_string_command_BufLen/19)+1+JSON_string_command_BufLen;
    }
    
    uint8_t *JSON_string_command_wTP = HSD_malloc(sizeof(uint8_t) * length_wTP);
    
    tot_len = BLECommand_TP_Encapsulate(JSON_string_command_wTP, (uint8_t*)JSON_string_command, JSON_string_command_BufLen); 
    
    /* Data are sent as notifications*/
    while (j < tot_len) 
    {
      len = MIN(20, tot_len - j);
      ret = aci_gatt_update_char_value(configServiceHandle, configTxCharHandle, 0,len,(uint8_t*) &JSON_string_command_wTP[j]);
      if(ret == BLE_STATUS_INSUFFICIENT_RESOURCES)
      {
        osSemaphoreWait(bleCongestionSem_id,osWaitForever);
      }
      else if (ret == BLE_STATUS_SUCCESS)
      {
        j += len;
      }
    }
    free(JSON_string_command);
    free(JSON_string_command_wTP);
  }
}

static void BLE_CM_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  BLE_CM_SPI_CLK_PIN_CLK_ENABLE(); 
  BLE_CM_SPI_MISO_PIN_CLK_ENABLE(); 
  BLE_CM_SPI_MOSI_PIN_CLK_ENABLE(); 
  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  
  GPIO_InitStruct.Alternate = BLE_CM_SPI_CLK_AF;
  GPIO_InitStruct.Pin = BLE_CM_SPI_CLK_PIN;
  HAL_GPIO_Init(BLE_CM_SPI_CLK_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Alternate = BLE_CM_SPI_MISO_AF;
  GPIO_InitStruct.Pin = BLE_CM_SPI_MISO_PIN;
  HAL_GPIO_Init(BLE_CM_SPI_MISO_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Alternate = BLE_CM_SPI_MOSI_AF;
  GPIO_InitStruct.Pin = BLE_CM_SPI_MOSI_PIN;
  HAL_GPIO_Init(BLE_CM_SPI_MOSI_GPIO_PORT, &GPIO_InitStruct);
  
  BLE_CM_SPIx_CLK_ENABLE();
  
  /* Enable GPIO Ports Clock */  
  __GPIOA_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  
  /* Enable SPI clock */
  __SPI2_CLK_ENABLE();  
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddUSB();  
  HAL_PWREx_EnableVddIO2();
  
  /*Configure EXTI Line */
  GPIO_InitStruct.Pin = BLE_CM_SPI_EXTI_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_CM_SPI_EXTI_PORT, &GPIO_InitStruct);
  
  /* Register event irq handler */
  HAL_NVIC_SetPriority(BLE_CM_SPI_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(BLE_CM_SPI_EXTI_IRQn);
   
  /*Configure CS & RESET Line */
  GPIO_InitStruct.Pin = BLE_CM_RST_PIN ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_CM_RST_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = BLE_CM_SPI_CS_PIN ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_CM_SPI_CS_PORT, &GPIO_InitStruct);
}

/**
 * @brief  Enable SPI IRQ.
 * @param  None
 * @retval None
 */
void BLE_CM_SPI_Enable_IRQ(void)
{
  HAL_NVIC_EnableIRQ(BLE_CM_SPI_EXTI_IRQn);  
}

/**
 * @brief  Disable SPI IRQ.
 * @param  None
 * @retval None
 */
void BLE_CM_SPI_Disable_IRQ(void)
{ 
  HAL_NVIC_DisableIRQ(BLE_CM_SPI_EXTI_IRQn);
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
