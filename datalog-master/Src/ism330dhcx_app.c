/**
******************************************************************************
* @file    ism330dhcx_app.c
* @author  SRA
* @version v3.0.0
* @date    19-Jun-2020
* @brief   This file provides a set of functions to handle ism330dhcx sensor
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
#include "ism330dhcx_app.h"
#include "main.h"
#include "cmsis_os.h"
#include "sensors_manager.h"
#include "com_manager.h"
#include "sdcard_manager.h"
#include "device_description.h"
#include "ism330dhcx_reg.h"
#include "ism330dhcx_reg.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define ISM330DHCX_SAMPLE_SIZE  (7)
#define ISM330DHCX_TAG_ACC      (0x02)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile double TimeStamp_ism330dhcx;
static uint8_t ism330dhcx_mem[ISM330DHCX_MAX_SAMPLES_PER_IT * 7];
static uint8_t ism330dhcx_mem_app[ISM330DHCX_MAX_SAMPLES_PER_IT/2 * 6]; //without Tag
uint16_t ism330dhcx_samples_per_it;

static volatile double TimeStamp_mlc;
static uint8_t mlc_mem = 7;

extern volatile uint8_t MLC_loaded;

static volatile uint32_t UCFSize;
static volatile char *UCFData;

SM_Init_Param_t ISM330DHCX_Init_Param;
SM_Sensor_State_t ISM330DHCX_Sensor_State = SM_SENSOR_STATE_INITIALIZING;
volatile static uint32_t MLC_Data_Ready = 0;

/* Semaphore used to wait on component interrupt */
osSemaphoreId ism330dhcx_DreadySem_id;
osSemaphoreDef(ism330dhcx_DreadySem);

/* Semaphore used to wait on BUS data read complete, managed by lower layer */
osSemaphoreId ism330dhcxReadSem_id;
osSemaphoreDef(ism330dhcxReadSem);

sensor_handle_t ism330dhcx_hdl_instance = {0, 0, ISM330DHCX_SPI_CS_GPIO_Port, ISM330DHCX_SPI_CS_Pin, &ism330dhcxReadSem_id};
stmdev_ctx_t ism330dhcx_ctx_instance = {SM_SPI_Write_Os, SM_SPI_Read_Os, &ism330dhcx_hdl_instance};

EXTI_HandleTypeDef mlc_exti;
EXTI_HandleTypeDef ism330dhcx_exti;

/* Private function prototypes -----------------------------------------------*/

osThreadId ISM330DHCX_Thread_Id;
static void ISM330DHCX_Thread(void const *argument);
static void HSD_MLC_Int_Config(void);

static void ISM330DHCX_Int_Callback(void);
static void ISM330DHCX_Sensor_Init(void);
static void ISM330DHCX_MLC_Int_Callback(void);


/**
* @brief IIS3DWB GPIO Initialization Function
* @param None
* @retval None
*/
void ISM330DHCX_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  ISM330DHCX_SPI_CS_GPIO_CLK_ENABLE();
  ISM330DHCX_INT1_GPIO_CLK_ENABLE();
  ISM330DHCX_INT2_GPIO_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM330DHCX_SPI_CS_GPIO_Port, ISM330DHCX_SPI_CS_Pin, GPIO_PIN_SET);
  
  /*Configure GPIO pin : IIS3DWB_SPI_CS_Pin */
  GPIO_InitStruct.Pin = ISM330DHCX_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ISM330DHCX_SPI_CS_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STTS751_INT_Pin IIS3DWB_INT1_Pin */
  GPIO_InitStruct.Pin =  ISM330DHCX_INT1_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ISM330DHCX_INT1_GPIO_Port, &GPIO_InitStruct);  
  
  /*Configure GPIO pins : STTS751_INT_Pin IIS3DWB_INT1_Pin */
  GPIO_InitStruct.Pin =  ISM330DHCX_INT2_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ISM330DHCX_INT2_GPIO_Port, &GPIO_InitStruct);
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ISM330DHCX_INT1_EXTI_IRQn, 5, 0);
  
  HAL_EXTI_GetHandle(&ism330dhcx_exti, ISM330DHCX_INT1_EXTI_LINE);
  HAL_EXTI_RegisterCallback(&ism330dhcx_exti,  HAL_EXTI_COMMON_CB_ID, ISM330DHCX_Int_Callback);
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ISM330DHCX_INT2_EXTI_IRQn, 5, 0);
  
  HAL_EXTI_GetHandle(&mlc_exti, ISM330DHCX_INT2_EXTI_LINE);
  HAL_EXTI_RegisterCallback(&mlc_exti,  HAL_EXTI_COMMON_CB_ID, ISM330DHCX_MLC_Int_Callback);
}


void ISM330DHCX_OS_Init(void)
{  
  /* Data read complete semaphore initialization */  
  ism330dhcxReadSem_id = osSemaphoreCreate(osSemaphore(ism330dhcxReadSem), 1);
  osSemaphoreWait(ism330dhcxReadSem_id,osWaitForever);
  
  /* Data ready interrupt semaphore initialization */  
  ism330dhcx_DreadySem_id = osSemaphoreCreate(osSemaphore(ism330dhcx_DreadySem), 1);
  osSemaphoreWait(ism330dhcx_DreadySem_id,  osWaitForever);
  
  /* Thread 1 definition */  
  osThreadDef(ISM330_RD_USR_THREAD, ISM330DHCX_Thread, ISM330DHCX_THREAD_PRIO, 1, 2000);  
  /* Start thread 1 */
  ISM330DHCX_Thread_Id = osThreadCreate(osThread(ISM330_RD_USR_THREAD), NULL);   
  /* Suspend thread */
  osThreadSuspend(ISM330DHCX_Thread_Id);
  
}


//ism330dhcx_pin_int1_route_t int1_route;
static void ISM330DHCX_Thread(void const *argument)
{     
  (void) argument;
  
#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_ISM330DHCX_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_ISM330DHCX_DEBUG_PIN );
#endif
  
#if (HSD_USE_FAKE_DATA == 1)
  static int16_t fakeDataCounter_acc = 0;
  static int16_t fakeDataCounter_gyro = 0;
#endif
  
  uint8_t reg0;
  uint8_t reg1;
  uint16_t i = 0;
  
  for (;;)
  {
    if (ISM330DHCX_Sensor_State == SM_SENSOR_STATE_MLC_CONFIG)
    {  
      ISM330DHCX_Program_MLC((uint32_t)UCFSize, (char *)UCFData);
      ISM330DHCX_Sensor_State = SM_SENSOR_STATE_INITIALIZING;
      osThreadSuspend(ISM330DHCX_Thread_Id);
    }
    else if (ISM330DHCX_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {  
      if (MLC_loaded == 0 && ISM330DHCX_Init_Param.subSensorActive[2])      /* If you enter here, MLC isActive is true but no UCF are avalable */
      {                                                                     /* So, MLC can't be active --> subSensorStatus is forced not Active */
        
        COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(ism330dhcx_com_id, 2);
        pSubSensorStatus->isActive = 0;
        /* Update the sensor-specific config structure */
        update_sensors_config();        
      }
      ISM330DHCX_Sensor_Init();
      ISM330DHCX_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if(ISM330DHCX_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {  
      osSemaphoreWait(ism330dhcx_DreadySem_id,  osWaitForever); 
      
      if(ISM330DHCX_Sensor_State == SM_SENSOR_STATE_RUNNING)
      {  
        /* Check FIFO_WTM_IA anf fifo level. We do not use PID in order to avoid reading one register twice */
        ism330dhcx_read_reg(&ism330dhcx_ctx_instance, ISM330DHCX_FIFO_STATUS1, &reg0, 1);
        ism330dhcx_read_reg(&ism330dhcx_ctx_instance, ISM330DHCX_FIFO_STATUS2, &reg1, 1);     
        
        uint16_t fifo_level = ((reg1 & 0x03) << 8) + reg0; 
        
        if((reg1) & 0x80  && (fifo_level>=ism330dhcx_samples_per_it) )
        {        
          /* Read sensor data from FIFO */
          ism330dhcx_read_reg(&ism330dhcx_ctx_instance, ISM330DHCX_FIFO_DATA_OUT_TAG, (uint8_t *)ism330dhcx_mem, ism330dhcx_samples_per_it * 7);
#if (HSD_USE_FAKE_DATA == 1)

          int16_t *p16 = (int16_t *)ism330dhcx_mem;

          for (i = 0; i < ism330dhcx_samples_per_it; i++)
          {
            p16 = (int16_t *)(&ism330dhcx_mem[i*7]+1);
            if((ism330dhcx_mem[i*7]>>3) == ISM330DHCX_TAG_ACC)
            {
              *p16++ = fakeDataCounter_acc++;
              *p16++ = fakeDataCounter_acc++;
              *p16++ = fakeDataCounter_acc++;
            }
            else
            {
              *p16++ = fakeDataCounter_gyro++;
              *p16++ = fakeDataCounter_gyro++;
              *p16++ = fakeDataCounter_gyro++;
            }
          }

#endif
          
          if(ISM330DHCX_Init_Param.subSensorActive[0] && ISM330DHCX_Init_Param.subSensorActive[1])
          {
            uint32_t ODR_Acc  = (uint32_t)COM_GetSubSensorStatus(ism330dhcx_com_id, 0)->ODR;
            uint32_t ODR_Gyro = (uint32_t)COM_GetSubSensorStatus(ism330dhcx_com_id, 1)->ODR;
            uint32_t gyroSamplesCount = 0, accSamplesCount = 0;

            int16_t *p16src = (int16_t *)ism330dhcx_mem;
            int16_t *pAcc, *pGyro;

            if(ODR_Acc > ODR_Gyro) /* Acc is faster than Gyro */
            {
              pAcc  = (int16_t *)ism330dhcx_mem;
              pGyro = (int16_t *)ism330dhcx_mem_app;
            }
            else
            {
              pAcc  = (int16_t *)ism330dhcx_mem_app;
              pGyro = (int16_t *)ism330dhcx_mem;
            }

            uint8_t *pTag = (uint8_t *)p16src;

            for (i = 0; i < ism330dhcx_samples_per_it; i++)
            {
              if(((*pTag)>>3) == ISM330DHCX_TAG_ACC)
              {
                p16src = (int16_t *)(pTag+1);
                *pAcc++ = *p16src++;
                *pAcc++ = *p16src++;
                *pAcc++ = *p16src++;
                accSamplesCount++;
              }
              else
              {
                p16src = (int16_t *)(pTag+1);
                *pGyro++ = *p16src++;
                *pGyro++ = *p16src++;
                *pGyro++ = *p16src++;
                gyroSamplesCount++;
              }
              pTag += 7;
            }
            if(ODR_Acc > ODR_Gyro) /* Acc is faster than Gyro */
            {

              ISM330DHCX_Data_Ready(0, (uint8_t *)ism330dhcx_mem, accSamplesCount * 6, TimeStamp_ism330dhcx);
              ISM330DHCX_Data_Ready(1, (uint8_t *)ism330dhcx_mem_app, gyroSamplesCount * 6, TimeStamp_ism330dhcx);
            }
            else
            {
              ISM330DHCX_Data_Ready(0, (uint8_t *)ism330dhcx_mem_app, accSamplesCount * 6, TimeStamp_ism330dhcx);
              ISM330DHCX_Data_Ready(1, (uint8_t *)ism330dhcx_mem, gyroSamplesCount * 6, TimeStamp_ism330dhcx);
            }
          }
          else /* 1 subsensor active only --> simply drop TAGS */
          {

            int16_t * p16src = (int16_t *)ism330dhcx_mem;
            int16_t * p16dest = (int16_t *)ism330dhcx_mem;
            for (i = 0; i < ism330dhcx_samples_per_it; i++)
            {
              p16src = (int16_t *)&((uint8_t *)(p16src))[1];
              *p16dest++ = *p16src++;
              *p16dest++ = *p16src++;
              *p16dest++ = *p16src++;
            }

            if(ISM330DHCX_Init_Param.subSensorActive[0]) // Acc only
            {
              ISM330DHCX_Data_Ready(0, (uint8_t *)ism330dhcx_mem, ism330dhcx_samples_per_it * 6, TimeStamp_ism330dhcx);
            }
            else if(ISM330DHCX_Init_Param.subSensorActive[1]) // Gyro only
            {
              ISM330DHCX_Data_Ready(1, (uint8_t *)ism330dhcx_mem, ism330dhcx_samples_per_it * 6, TimeStamp_ism330dhcx);
            }
          }
          
          if(ISM330DHCX_Init_Param.subSensorActive[2] && MLC_Data_Ready == 1)
          {
            ism330dhcx_mem_bank_set(&ism330dhcx_ctx_instance, ISM330DHCX_EMBEDDED_FUNC_BANK);  
            ism330dhcx_read_reg(&ism330dhcx_ctx_instance, ISM330DHCX_MLC0_SRC, (uint8_t *)&mlc_mem, 1);
            ism330dhcx_mem_bank_set(&ism330dhcx_ctx_instance, ISM330DHCX_USER_BANK);
            
            ISM330DHCX_Data_Ready(2, &mlc_mem, 1, TimeStamp_mlc);
            MLC_Data_Ready = 0;
          }        
        }
      }      
    } 
    else if ( ISM330DHCX_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_FAKE_DATA == 1)
      fakeDataCounter_acc = 0;
      fakeDataCounter_gyro = 0;
#endif      
      ism330dhcx_fifo_gy_batch_set(&ism330dhcx_ctx_instance, ISM330DHCX_GY_NOT_BATCHED);  /* ToDo power down */
      ISM330DHCX_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(ISM330DHCX_Thread_Id);
    }  
  }    
}



static void ISM330DHCX_Sensor_Init(void)
{
  uint8_t reg0;
  uint16_t ism330dhcx_wtm_level;
  
  ism330dhcx_device_id_get(&ism330dhcx_ctx_instance, (uint8_t *)&reg0);
  
  ism330dhcx_reset_set(&ism330dhcx_ctx_instance, 1);
  ism330dhcx_i2c_interface_set(&ism330dhcx_ctx_instance, ISM330DHCX_I2C_DISABLE);       
  
  /* AXL FS */ 
  if(ISM330DHCX_Init_Param.FS[0] < 3.0f)
    ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_2g); 
  else if(ISM330DHCX_Init_Param.FS[0] < 5.0f)
    ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_4g);  
  else if(ISM330DHCX_Init_Param.FS[0] < 9.0f)
    ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_8g);  
  else if(ISM330DHCX_Init_Param.FS[0] < 17.0f)
    ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_16g);  
  
  /* GYRO FS */ 
  if(ISM330DHCX_Init_Param.FS[1] < 126.0f)
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_125dps); 
  else if(ISM330DHCX_Init_Param.FS[1] < 251.0f)
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_250dps); 
  else if(ISM330DHCX_Init_Param.FS[1] < 501.0f)
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_500dps); 
  else if(ISM330DHCX_Init_Param.FS[1] < 1001.0f)
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_1000dps); 
  else if(ISM330DHCX_Init_Param.FS[1] < 2001.0f)
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_2000dps); 
  else if(ISM330DHCX_Init_Param.FS[1] < 4001.0f)
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_4000dps); 
  
  ism330dhcx_odr_xl_t ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_OFF;
  ism330dhcx_bdr_xl_t ism330dhcx_bdr_xl = ISM330DHCX_XL_NOT_BATCHED;        
  
  if(ISM330DHCX_Init_Param.ODR[0] < 13.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_12Hz5;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_12Hz5;
  }
  else if(ISM330DHCX_Init_Param.ODR[0] < 27.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_26Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_26Hz;     
  }
  else if(ISM330DHCX_Init_Param.ODR[0] < 53.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_52Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_52Hz;     
  }
  else if(ISM330DHCX_Init_Param.ODR[0] < 105.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_104Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_104Hz;     
  }
  else if(ISM330DHCX_Init_Param.ODR[0] < 209.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_208Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_208Hz;     
  }
  else if(ISM330DHCX_Init_Param.ODR[0] < 418.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_417Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_417Hz;     
  }
  else if(ISM330DHCX_Init_Param.ODR[0] < 834.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_833Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_833Hz;     
  }
  else if(ISM330DHCX_Init_Param.ODR[0] < 1668.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_1667Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_1667Hz;     
  }
  else if(ISM330DHCX_Init_Param.ODR[0] < 3334.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_3333Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_3333Hz;     
  }
  else if(ISM330DHCX_Init_Param.ODR[0] < 6668.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_6667Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_6667Hz;     
  }

  ism330dhcx_odr_g_t ism330dhcx_odr_g = ISM330DHCX_GY_ODR_OFF;
  ism330dhcx_bdr_gy_t ism330dhcx_bdr_gy = ISM330DHCX_GY_NOT_BATCHED;

  if(ISM330DHCX_Init_Param.ODR[1] < 13.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_12Hz5;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_12Hz5;
  }
  else if(ISM330DHCX_Init_Param.ODR[1] < 27.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_26Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_26Hz;
  }
  else if(ISM330DHCX_Init_Param.ODR[1] < 53.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_52Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_52Hz;
  }
  else if(ISM330DHCX_Init_Param.ODR[1] < 105.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_104Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_104Hz;
  }
  else if(ISM330DHCX_Init_Param.ODR[1] < 209.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_208Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_208Hz;
  }
  else if(ISM330DHCX_Init_Param.ODR[1] < 418.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_417Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_417Hz;
  }
  else if(ISM330DHCX_Init_Param.ODR[1] < 834.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_833Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_833Hz;
  }
  else if(ISM330DHCX_Init_Param.ODR[1] < 1668.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_1667Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_1667Hz;
  }
  else if(ISM330DHCX_Init_Param.ODR[1] < 3334.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_3333Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_3333Hz;
  }
  else if(ISM330DHCX_Init_Param.ODR[1] < 6668.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_6667Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_6667Hz;
  }
  
  if(ISM330DHCX_Init_Param.subSensorActive[0])
  {
    ism330dhcx_xl_data_rate_set(&ism330dhcx_ctx_instance, ism330dhcx_odr_xl);
    ism330dhcx_fifo_xl_batch_set(&ism330dhcx_ctx_instance, ism330dhcx_bdr_xl);          
  }
  if(ISM330DHCX_Init_Param.subSensorActive[1])
  {
    ism330dhcx_gy_data_rate_set(&ism330dhcx_ctx_instance, ism330dhcx_odr_g);
    ism330dhcx_fifo_gy_batch_set(&ism330dhcx_ctx_instance, ism330dhcx_bdr_gy);
  } 
  
  if(ISM330DHCX_Init_Param.subSensorActive[2]) // MLC isActive
  {
    /* Route int pins for MLC */
    HSD_MLC_Int_Config();       
  }

  /* Calculation of watermark and samples per int*/
  ism330dhcx_wtm_level = ((uint16_t)ISM330DHCX_Init_Param.ODR[0] * (uint16_t)ISM330DHCX_MAX_DRDY_PERIOD);
  if (ism330dhcx_wtm_level > ISM330DHCX_MAX_WTM_LEVEL)
  {
    ism330dhcx_wtm_level = ISM330DHCX_MAX_WTM_LEVEL;
  }
  else if (ism330dhcx_wtm_level < ISM330DHCX_MIN_WTM_LEVEL)
  {
    ism330dhcx_wtm_level = ISM330DHCX_MIN_WTM_LEVEL;
  }
  
  ism330dhcx_samples_per_it = ism330dhcx_wtm_level;
  
  ism330dhcx_pin_int1_route_t int1_route= {0};

  /* Setup int for FIFO */
  ism330dhcx_fifo_watermark_set(&ism330dhcx_ctx_instance, ism330dhcx_wtm_level);   
  int1_route.int1_ctrl.int1_fifo_th = 1;
  ism330dhcx_pin_int1_route_set(&ism330dhcx_ctx_instance, &int1_route);      
  ism330dhcx_fifo_mode_set(&ism330dhcx_ctx_instance, ISM330DHCX_STREAM_MODE);  
  
  HAL_NVIC_EnableIRQ(ISM330DHCX_INT1_EXTI_IRQn);  
}

static void HSD_MLC_Int_Config(void)
{  
  ism330dhcx_pin_int1_route_t pin_int1_route;
  ism330dhcx_pin_int2_route_t pin_int2_route;
  
  ism330dhcx_pin_int1_route_get(&ism330dhcx_ctx_instance, &pin_int1_route);
  ism330dhcx_pin_int2_route_get(&ism330dhcx_ctx_instance, &pin_int2_route);
  
  if (pin_int1_route.mlc_int1.int1_mlc1 == 1 || pin_int1_route.md1_cfg.int1_emb_func == 1)
  {
    pin_int1_route.mlc_int1.int1_mlc1 = 0;
    pin_int1_route.md1_cfg.int1_emb_func = 0;
    ism330dhcx_pin_int1_route_set(&ism330dhcx_ctx_instance, &pin_int1_route);
  }
  
  if (pin_int2_route.mlc_int2.int2_mlc1 == 0 || pin_int2_route.md2_cfg.int2_emb_func == 0)
  {
    pin_int2_route.mlc_int2.int2_mlc1 = 1;
    pin_int2_route.md2_cfg.int2_emb_func = 1;
    ism330dhcx_pin_int2_route_set(&ism330dhcx_ctx_instance, &pin_int2_route);
    
    HAL_NVIC_EnableIRQ(ISM330DHCX_INT2_EXTI_IRQn);  
  }    
}


void ISM330DHCX_Program_MLC(uint32_t size, char *buffer)
{
  char ucf_reg[3];
  char ucf_data[3];
  long reg;
  long data;
  uint32_t ii;
  char *dataRows = NULL;
  uint8_t *p;
  
  BSP_LED_On(LED1);
  dataRows = HSD_malloc(9*(size/4));
  p = (uint8_t*)dataRows;
  
  for (ii=0; ii<size/4; ii++)
  {
    ucf_reg[0] = buffer[4*ii];
    ucf_reg[1] = buffer[4*ii+1];
    ucf_reg[2] = '\0';
    ucf_data[0] = buffer[4*ii+2];
    ucf_data[1] = buffer[4*ii+3];
    ucf_data[2] = '\0';
    
    reg = strtol(ucf_reg, NULL, 16);      
    data = strtol(ucf_data, NULL, 16);
    
    *p++ = 'A';
    *p++ = 'c';
    *p++ = ' ';
    *p++ = buffer[4*ii];
    *p++ = buffer[4*ii+1];
    *p++ = ' ';
    *p++ = buffer[4*ii+2];
    *p++ = buffer[4*ii+3];
    *p++ = '\n';
    
    ism330dhcx_write_reg(&ism330dhcx_ctx_instance, (uint8_t)reg, (uint8_t*)&data, 1);
  }
  
  SDM_WriteUCF(dataRows, size);  
  MLC_loaded = 1;  
  HSD_free(dataRows);
  BSP_LED_Off(LED1);
}

static void ISM330DHCX_MLC_Int_Callback(void)
{
  TimeStamp_mlc = SM_GetTimeStamp_fromISR();
  MLC_Data_Ready = 1;
}

static void ISM330DHCX_Int_Callback(void)
{
  TimeStamp_ism330dhcx = SM_GetTimeStamp_fromISR();
  osSemaphoreRelease(ism330dhcx_DreadySem_id);    
}

void ISM330DHCX_Set_State(SM_Sensor_State_t state)
{
  ISM330DHCX_Sensor_State = state;
}

void ISM330DHCX_Start(void)
{
  ISM330DHCX_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(ISM330DHCX_Thread_Id);
}

void ISM330DHCX_SetUCF(uint32_t mlcConfigSize, char *mlcConfigData)
{
  UCFSize = mlcConfigSize;
  UCFData = mlcConfigData;
  ISM330DHCX_Set_State(SM_SENSOR_STATE_MLC_CONFIG);
  osThreadResume(ISM330DHCX_Thread_Id);
}

void ISM330DHCX_Stop(void)
{
  ISM330DHCX_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void ISM330DHCX_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
