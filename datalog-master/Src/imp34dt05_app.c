/**
  ******************************************************************************
  * @file    imp34dt05_app.c
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief   This file provides a set of functions to handle imp34dt05 microphone
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
#include "imp34dt05_app.h"
#include "main.h"

SM_Init_Param_t IMP34DT05_Init_Param;
SM_Sensor_State_t IMP34DT05_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
osMessageQId dmicDreadyQueue_id;
static osMessageQDef(dmicdreadyqueue, 1, int);

void DFSDM_Filter_0_HalfComplete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void DFSDM_Filter_0_Complete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void IMP34DT05_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef* hdfsdm_filter);

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


uint32_t dmic_mem[((IMP34DT05_SAMPLING_FREQUENCY/1000) * IMP34DT05_MS * 2)*4];

uint16_t newDataIdxDMic;
uint16_t oldDataLenDMic;
uint16_t newDataLenDMic;

osThreadId IMP34DT05_Thread_Id;

static void IMP34DT05_Thread(void const *argument);

static void IMP34DT05_DFSDM_Init(void);
static void IMP34DT05_DMA_Init(void) ;

static void Error_Handler(void);
static void Error_Handler(void)
{
  while(1);
}

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel5;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

static volatile uint32_t tim_value = 0, tim_value_old = 0, period = 0;
static volatile uint64_t ts_imp34dt05 = 0;
static volatile uint32_t periodCounter = 0;
static volatile double TimeStamp_imp34dt05a;

/* Private function prototypes -----------------------------------------------*/


/**
* @brief IIS3DWB GPIO Initialization Function
* @param None
* @retval None
*/
void IMP34DT05_Peripheral_Init(void)
{
  IMP34DT05_DMA_Init();
  IMP34DT05_DFSDM_Init();  
}

static void IMP34DT05_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  
}

static void IMP34DT05_DFSDM_Init(void)
{
  
  HAL_DFSDM_Filter_RegisterCallback(&hdfsdm1_filter0, HAL_DFSDM_FILTER_MSPINIT_CB_ID, IMP34DT05_DFSDM_FilterMspInit);
  
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 64;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  
  hdfsdm1_channel5.Instance = DFSDM1_Channel5;
  hdfsdm1_channel5.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel5.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  hdfsdm1_channel5.Init.OutputClock.Divider = 4;
  hdfsdm1_channel5.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel5.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel5.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel5.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel5.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel5.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel5.Init.Awd.Oversampling = 1;
  hdfsdm1_channel5.Init.Offset = 0;
  hdfsdm1_channel5.Init.RightBitShift = 0x0C;
  
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel5) != HAL_OK)
  {
    Error_Handler();
  }  
  
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_5, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  
  HAL_DFSDM_Filter_RegisterCallback(&hdfsdm1_filter0, HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_Filter_0_HalfComplete_Callback);
  HAL_DFSDM_Filter_RegisterCallback(&hdfsdm1_filter0, HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_Filter_0_Complete_Callback); 
}

/**
* @brief IIS3DWB Threads Creation
* @param None
* @retval None
*/
void IMP34DT05_OS_Init(void)
{  
  /* Data read complete semaphore initialization */  
  dmicDreadyQueue_id = osMessageCreate(osMessageQ(dmicdreadyqueue), NULL);
//  dmicPool_id = osPoolCreate(osPool(dmicPool));
  
  /* Thread definition: read data */  
  osThreadDef(IMP34DT05_Acquisition_Thread, IMP34DT05_Thread, IMP34DT05_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread */
  IMP34DT05_Thread_Id = osThreadCreate(osThread(IMP34DT05_Acquisition_Thread), NULL);
  /* Suspend thread */
  osThreadSuspend(IMP34DT05_Thread_Id);
}


static void IMP34DT05_Thread(void const *argument)
{
  (void) argument;
  
#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_IMP34DT05_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_IMP34DT05_DEBUG_PIN );
#endif
  
#if (HSD_USE_FAKE_DATA == 1)
  static uint16_t fakeDataCounter = 0;
#endif
  
  osEvent evt;
  for (;;)
  {
    if (IMP34DT05_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {  
      ts_imp34dt05 = 0;
      tim_value_old = 0;
      HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, (int32_t*)dmic_mem, (IMP34DT05_SAMPLING_FREQUENCY / 1000) * IMP34DT05_MS * 2);
      IMP34DT05_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if(IMP34DT05_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {  
      evt = osMessageGet(dmicDreadyQueue_id, osWaitForever);
      
      if(IMP34DT05_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {        
        uint16_t idx = 0;
        void * data_ptr = evt.value.p; /* void since it is independent from data format */

#if (HSD_USE_FAKE_DATA != 1)        
        static int oldIn = 0;
        static int oldOut = 0;
        int32_t * p32 = (int32_t *)data_ptr;        
        int16_t * p16 = (int16_t *)data_ptr;
#else
        uint16_t *p16 = (uint16_t *)data_ptr;
#endif
        
        for (idx = 0; idx < ((IMP34DT05_SAMPLING_FREQUENCY/1000) * IMP34DT05_MS) ; idx++)
        {
#if (HSD_USE_FAKE_DATA == 1)
          *p16++ = fakeDataCounter++;
#else
          *p16++ = oldOut = (0xFC * (oldOut + ((*p32)>>10) - oldIn)) / 0xFF;
          oldIn = (*p32 ++)>>10;
#endif                  
        }      
        
        IMP34DT05_Data_Ready(0, (uint8_t *)data_ptr, (IMP34DT05_SAMPLING_FREQUENCY / 1000) * IMP34DT05_MS * 2, TimeStamp_imp34dt05a);
      }         
    } 
    else if ( IMP34DT05_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {  
#if (HSD_USE_FAKE_DATA == 1)
      fakeDataCounter = 0;
#endif      
      HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
      IMP34DT05_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(IMP34DT05_Thread_Id);
    }   
  }  
}


void DFSDM_Filter_0_HalfComplete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  TimeStamp_imp34dt05a = SM_GetTimeStamp_fromISR();
  
  osMessagePut(dmicDreadyQueue_id, (uint32_t)(dmic_mem), osWaitForever);
}

void DFSDM_Filter_0_Complete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  TimeStamp_imp34dt05a = SM_GetTimeStamp_fromISR();
  
  osMessagePut(dmicDreadyQueue_id, (uint32_t)(&dmic_mem[((IMP34DT05_SAMPLING_FREQUENCY/1000) * IMP34DT05_MS)]), osWaitForever);
}

void IMP34DT05_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef* hdfsdm_filter)
{
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  IMP34DT05_DFSDM_CLK_ENABLE();
  
  
  IMP34DT05_DFSDM_PDM_PIN_CLK_ENABLE();
  IMP34DT05_DFSDM_CLK_PIN_CLK_ENABLE();
  /**DFSDM1 GPIO Configuration    
  PB6     ------> DFSDM1_DATIN5
  PE9     ------> DFSDM1_CKOUT 
  */
  GPIO_InitStruct.Pin = IMP34DT05_DFSDM_PDM_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(IMP34DT05_DFSDM_PDM_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = IMP34DT05_DFSDM_CLK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(IMP34DT05_DFSDM_CLK_GPIO_PORT, &GPIO_InitStruct);
  
  
  hdma_dfsdm1_flt0.Instance = IMP34DT05_DFSDM_RX_DMA_CHANNEL;
  hdma_dfsdm1_flt0.Init.Request = IMP34DT05_DFSDM_RX_DMA_REQUEST;
  hdma_dfsdm1_flt0.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_dfsdm1_flt0.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dfsdm1_flt0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dfsdm1_flt0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_dfsdm1_flt0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_dfsdm1_flt0.Init.Mode = DMA_CIRCULAR;
  hdma_dfsdm1_flt0.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_dfsdm1_flt0) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Several peripheral DMA handle pointers point to the same DMA handle.
  Be aware that there is only one channel to perform all the requested DMAs. */
  __HAL_LINKDMA(hdfsdm_filter,hdmaInj,hdma_dfsdm1_flt0);
  __HAL_LINKDMA(hdfsdm_filter,hdmaReg,hdma_dfsdm1_flt0);
  
}

void IMP34DT05_Set_State(SM_Sensor_State_t state)
{
  IMP34DT05_Sensor_State = state;
}

void IMP34DT05_Start(void)
{
  IMP34DT05_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(IMP34DT05_Thread_Id);
}

void IMP34DT05_Stop(void)
{
  IMP34DT05_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void IMP34DT05_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
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
