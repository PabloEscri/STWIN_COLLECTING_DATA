/**
  ******************************************************************************
  * @file    stts751_app.c
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief   This file provides a set of functions to handle stts751 sensor
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
#include "stts751_app.h"
#include "main.h"
#include "cmsis_os.h"
#include "stts751_reg.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t temperature;
static uint16_t taskDelay = 1000;
static volatile double TimeStamp_stts751;

SM_Init_Param_t STTS751_Init_Param;
SM_Sensor_State_t STTS751_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
/* Semaphore used to wait on BUS data read complete, managed by lower layer */
osSemaphoreId STTS751_ReadSem_id;
osSemaphoreDef(STTS751_ReadSem);

static sensor_handle_t stts751_hdl_instance = {0, STTS751_0xxxx_ADD_7K5, NULL, 0, &STTS751_ReadSem_id};
static stmdev_ctx_t stts751_ctx_instance= {SM_I2C_Write_Os, SM_I2C_Read_Os, &stts751_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId STTS751_Thread_Id;
static void STTS751_Thread(void const *argument);
static void STTS751_Sensor_Init(void);


/**
* @brief STTS751 GPIO Initialization Function
* @param None
* @retval None
*/
void STTS751_Peripheral_Init(void)
{
  
}

/**
* @brief STTS751 Threads Creation
* @param None
* @retval None
*/
void STTS751_OS_Init(void)
{  
  STTS751_ReadSem_id = osSemaphoreCreate(osSemaphore(STTS751_ReadSem), 1);
  osSemaphoreWait(STTS751_ReadSem_id,osWaitForever);
  
  /* Thread 1 definition */  
  osThreadDef(STTS751_RD_USR_THREAD, STTS751_Thread, STTS751_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);  
  /* Start thread 1 */
  STTS751_Thread_Id = osThreadCreate(osThread(STTS751_RD_USR_THREAD), NULL);  
  /* Suspend thread */
  osThreadSuspend(STTS751_Thread_Id); 
}


/**
* @brief  Get data raw from sensors to queue
* @param  thread not used
* @retval None
*/ 
static void STTS751_Thread(void const *argument)
{
  (void) argument;
  
#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_STTS751_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_STTS751_DEBUG_PIN );
#endif
  
#if (HSD_USE_FAKE_DATA == 1)
  static uint16_t fakeDataCounter = 0;
#endif
  
  for (;;)
  {
    if (STTS751_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {  
      STTS751_Sensor_Init();      
      STTS751_Sensor_State = SM_SENSOR_STATE_RUNNING;      
    }
    else if(STTS751_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {  
      vTaskDelay(taskDelay);  
      
      if(STTS751_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {    
        float temperature_celsius;
        
        TimeStamp_stts751 = SM_GetTimeStamp();
        
        stts751_temperature_raw_get(&stts751_ctx_instance,  (int16_t *)&temperature);  
        temperature_celsius = (float)temperature / 256.0f;
        
#if (HSD_USE_FAKE_DATA == 1)
        temperature_celsius = (float)fakeDataCounter++;
#endif
        
        STTS751_Data_Ready(0, (uint8_t *)&temperature_celsius, 4, TimeStamp_stts751);
      }
    } 
    else if ( STTS751_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {       
#if (HSD_USE_FAKE_DATA == 1)
      fakeDataCounter = 0;
#endif 
      stts751_temp_data_rate_set(&stts751_ctx_instance,  STTS751_TEMP_ODR_OFF);
      STTS751_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(STTS751_Thread_Id);
    }      
  }
}


static void STTS751_Sensor_Init(void)
{
  stts751_id_t STTS751_Id; 
  
  stts751_device_id_get(&stts751_ctx_instance,  (stts751_id_t *)&STTS751_Id);
  /* ToDo: check Id */
  stts751_temp_data_rate_set(&stts751_ctx_instance,  STTS751_TEMP_ODR_8Hz);
  stts751_resolution_set(&stts751_ctx_instance,  STTS751_12bit);
  
  if(STTS751_Init_Param.ODR[0] < 2.0f)
  {       
    taskDelay = 1000;
  }
  else if(STTS751_Init_Param.ODR[0] < 3.0f)
  {
    taskDelay = 500;
  }
  else if(STTS751_Init_Param.ODR[0] < 5.0f)
  {
    taskDelay = 250;
  }      
}

void STTS751_Set_State(SM_Sensor_State_t state)
{
  STTS751_Sensor_State = state;
}

void STTS751_Start(void)
{
  STTS751_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(STTS751_Thread_Id);
}

void STTS751_Stop(void)
{
  STTS751_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void STTS751_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  
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
