/**
  ******************************************************************************
  * @file    iis2dh_app.c
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief   This file provides a set of functions to handle iis2dh
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
#include "iis2dh_app.h"
#include "main.h"
#include "sensors_manager.h"
#include "iis2dh_reg.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define HTONS(A)  ((((uint16_t)(A) & 0xff00) >> 8) | \
                   (((uint16_t)(A) & 0x00ff) << 8))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* taskDelay is set so that the number of samples in FIFO shouldn't be more than 16
   so the buffer is 16*2 to have some margin */
static uint8_t iis2dh_mem[32 * 7];

static uint16_t taskDelay = 1000;
static volatile double TimeStamp_iis2dh;
volatile uint8_t testWTM = 0;

SM_Init_Param_t IIS2DH_Init_Param;
SM_Sensor_State_t IIS2DH_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
osSemaphoreId  iis2dh_DreadySem_id;
osSemaphoreDef( iis2dh_DreadySem);

/* Semaphore used to wait on BUS data read complete, managed by lower layer */
osSemaphoreId  iis2dhReadSem_id;
osSemaphoreDef( iis2dhReadSem);

static sensor_handle_t  iis2dh_hdl_instance = {IIS2DH_ID, 0, IIS2DH_SPI_CS_GPIO_Port, IIS2DH_SPI_CS_Pin, &iis2dhReadSem_id};
static  stmdev_ctx_t  iis2dh_ctx_instance = {SM_SPI_Write_Os, SM_SPI_Read_Os, & iis2dh_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId IIS2DH_Thread_Id;
static void IIS2DH_Thread(void const *argument);

EXTI_HandleTypeDef iis2dh_exti;
static void IIS2DH_Int_Callback(void);
static void IIS2DH_Sensor_Init(void);



/**
* @brief IIS3DWB GPIO Initialization Function
* @param None
* @retval None
*/
void IIS2DH_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IIS2DH_SPI_CS_GPIO_Port, IIS2DH_SPI_CS_Pin, GPIO_PIN_SET);
  
  /*Configure GPIO pin : IIS3DWB_SPI_CS_Pin */
  GPIO_InitStruct.Pin = IIS2DH_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IIS2DH_SPI_CS_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STTS751_INT_Pin IIS3DWB_INT1_Pin */
  GPIO_InitStruct.Pin =  IIS2DH_INT2_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IIS2DH_INT2_GPIO_Port, &GPIO_InitStruct);  
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  
  HAL_EXTI_GetHandle(& iis2dh_exti, EXTI_LINE_2);  
  HAL_EXTI_RegisterCallback(& iis2dh_exti,  HAL_EXTI_COMMON_CB_ID, IIS2DH_Int_Callback);
  
}


void IIS2DH_OS_Init(void)
{  
  /* Data read complete semaphore initialization */  
  iis2dhReadSem_id = osSemaphoreCreate(osSemaphore( iis2dhReadSem), 1);
  osSemaphoreWait( iis2dhReadSem_id,osWaitForever);
  
  /* Data ready interrupt semaphore initialization */  
  iis2dh_DreadySem_id = osSemaphoreCreate(osSemaphore( iis2dh_DreadySem), 1);
  osSemaphoreWait( iis2dh_DreadySem_id,  osWaitForever);
  
  /* Thread 1 definition */  
  osThreadDef(IIS2DH_RD_USR_THREAD, IIS2DH_Thread, IIS2DH_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread 1 */
  IIS2DH_Thread_Id = osThreadCreate(osThread(IIS2DH_RD_USR_THREAD), NULL);   
  /* Suspend thread */
  osThreadSuspend(IIS2DH_Thread_Id);
  
}



static void IIS2DH_Thread(void const *argument)
{ 
  (void) argument;
  
#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_IIS2DH_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_IIS2DH_DEBUG_PIN );
#endif

#if (HSD_USE_FAKE_DATA == 1)
  static uint16_t fakeDataCounter = 0;
#endif
  
  for (;;)
  {
    if (IIS2DH_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      IIS2DH_Sensor_Init();
      IIS2DH_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if(IIS2DH_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      vTaskDelay(taskDelay);
      
      if(IIS2DH_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        iis2dh_fifo_data_level_get(&iis2dh_ctx_instance, (uint8_t*)&testWTM);
        
        /* This needs to be after iis2dh_fifo_data_level_get to be 
            closer to the real acquisition of the last item in FIFO */
        TimeStamp_iis2dh = SM_GetTimeStamp();
        
        iis2dh_read_reg(& iis2dh_ctx_instance, IIS2DH_OUT_X_L, (uint8_t *) iis2dh_mem, testWTM*6); 
        
#if (HSD_USE_FAKE_DATA == 1)
        uint16_t i = 0;
        int16_t * p16 = (int16_t *)iis2dh_mem;
        
        for (i = 0; i < testWTM*3 ; i++)    
        {          
          *p16++ = fakeDataCounter++;        
        }
#endif      
        
        IIS2DH_Data_Ready(0, (uint8_t *)iis2dh_mem, testWTM*6, TimeStamp_iis2dh);
      }
    }
    else if ( IIS2DH_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {   
#if (HSD_USE_FAKE_DATA == 1)
      fakeDataCounter = 0;
#endif
      iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_POWER_DOWN);
      IIS2DH_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(IIS2DH_Thread_Id);
    }
  }    
}


static void IIS2DH_Sensor_Init(void)
{      
  uint8_t reg0;
  
  iis2dh_boot_set(&iis2dh_ctx_instance, PROPERTY_ENABLE);      
  
  iis2dh_device_id_get(&iis2dh_ctx_instance, &reg0);
  iis2dh_spi_mode_set(&iis2dh_ctx_instance, IIS2DH_SPI_4_WIRE);
  iis2dh_ctrl_reg1_t ctrl_reg1;
  iis2dh_read_reg(&iis2dh_ctx_instance, IIS2DH_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  
  ctrl_reg1.xen = 0;
  ctrl_reg1.yen = 0;
  ctrl_reg1.zen = 0;
  iis2dh_write_reg(&iis2dh_ctx_instance, IIS2DH_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  
  /* Output data rate selection - power down. */
  iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_POWER_DOWN);  
  /* ODisable Temperature Measurement */
  iis2dh_temperature_meas_set(&iis2dh_ctx_instance, IIS2DH_TEMP_DISABLE);      
  /* Enable BDU */
  iis2dh_block_data_update_set(&iis2dh_ctx_instance, PROPERTY_ENABLE);
  
  /* Full scale selection. */
  if(IIS2DH_Init_Param.FS[0] < 3.0f)
    iis2dh_full_scale_set(&iis2dh_ctx_instance, IIS2DH_2g);
  else if(IIS2DH_Init_Param.FS[0] < 5.0f)
    iis2dh_full_scale_set(&iis2dh_ctx_instance, IIS2DH_4g);
  else if(IIS2DH_Init_Param.FS[0] < 9.0f)
    iis2dh_full_scale_set(&iis2dh_ctx_instance, IIS2DH_8g);
  else if(IIS2DH_Init_Param.FS[0] < 17.0f)
    iis2dh_full_scale_set(&iis2dh_ctx_instance, IIS2DH_16g);     
  
  /* Power mode selection */
  iis2dh_fifo_set(& iis2dh_ctx_instance, 1);  
  
  iis2dh_operating_mode_set(&iis2dh_ctx_instance, IIS2DH_HR_12bit);
  
  /* Big/Little Endian data selection configuration */
  iis2dh_data_format_set(&iis2dh_ctx_instance, IIS2DH_MSB_AT_LOW_ADD);
  iis2dh_read_reg(&iis2dh_ctx_instance, IIS2DH_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);      
  ctrl_reg1.xen = 1;
  ctrl_reg1.yen = 1;
  ctrl_reg1.zen = 1;
  iis2dh_write_reg(&iis2dh_ctx_instance, IIS2DH_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);      
  iis2dh_fifo_mode_set(&iis2dh_ctx_instance, IIS2DH_DYNAMIC_STREAM_MODE);      
  
  /* taskDelay is set so that the number of samples in FIFO shouldn't be more than 16 */
  if(IIS2DH_Init_Param.ODR[0] < 2.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_1Hz);
    taskDelay = 1000;
  }
  else if(IIS2DH_Init_Param.ODR[0] < 11.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_10Hz);
    taskDelay = 1000;
  }
  else if(IIS2DH_Init_Param.ODR[0] < 26.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_25Hz);
    taskDelay = 640;
  }
  else if(IIS2DH_Init_Param.ODR[0] < 51.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_50Hz);
    taskDelay = 320;
  }
  else if(IIS2DH_Init_Param.ODR[0] < 101.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_100Hz);
    taskDelay = 160;
  }
  else if(IIS2DH_Init_Param.ODR[0] < 201.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_200Hz);
    taskDelay = 80;
  }
  else if(IIS2DH_Init_Param.ODR[0] < 401.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_400Hz);
    taskDelay = 40;
  }
  else if(IIS2DH_Init_Param.ODR[0] < 1345.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_5kHz376_LP_1kHz344_NM_HP); 
    taskDelay = 12;
  }
  
  HAL_NVIC_EnableIRQ(IIS2DH_INT2_EXTI_IRQn);          
}

static void IIS2DH_Int_Callback(void)
{
  osSemaphoreRelease( iis2dh_DreadySem_id);    
}

void IIS2DH_Set_State(SM_Sensor_State_t state)
{
  IIS2DH_Sensor_State = state;
}

void IIS2DH_Start(void)
{
  IIS2DH_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(IIS2DH_Thread_Id);
}

void IIS2DH_Stop(void)
{
  IIS2DH_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void IIS2DH_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
