/**
  ******************************************************************************
  * @file    mp23abs1_app.c
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief   This file provides a set of functions to handle mp23abs1 microphone
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
#include "mp23abs1_app.h"
#include "main.h"
#include "sensors_manager.h"

SM_Init_Param_t MP23ABS1_Init_Param;
SM_Sensor_State_t MP23ABS1_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

static void MP23ABS1_ADC_Init(void);
static void  MP23ABS1_DMA_Init(void); 
static void MP23ABS1_DFSDM_Init(void);
void MP23ABS1_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef* hdfsdm_filter);
void MP23ABS1_ADC_MspInit(ADC_HandleTypeDef* hadc);

void DFSDM_Filter_1_HalfComplete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter); 
void DFSDM_Filter_1_Complete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);

ADC_HandleTypeDef hadc1;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DMA_HandleTypeDef hdma_dfsdm1_flt1;

static uint32_t amic_mem[((AMIC_SAMPLING_FREQUENCY/1000) * AMIC_MS * 2)];

osThreadId MP23ABS1_Thread_Id;

static void MP23ABS1_Thread(void const *argument);

osMessageQId amicDreadyQueue_id;
osMessageQDef(amicdreadyqueue, 1, int);

static void Error_Handler(void);
static void Error_Handler(void)
{
  while(1);
}

static volatile uint32_t tim_value = 0, tim_value_old = 0, period = 0;
static volatile uint64_t ts_mp23abs1;
static volatile double TimeStamp_mp23abs1;

/**
* @brief IIS3DWB GPIO Initialization Function
* @param None
* @retval None
*/
void MP23ABS1_Peripheral_Init(void)
{
  MP23ABS1_DMA_Init();
  MP23ABS1_DFSDM_Init();  
  MP23ABS1_ADC_Init();
}

/**
* @brief ADC1 Initialization Function
* @param None
* @retval None
*/
static void MP23ABS1_ADC_Init(void)
{
  HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_MSPINIT_CB_ID, MP23ABS1_ADC_MspInit);
  
  ADC_ChannelConfTypeDef sConfig = {0};
  
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.DFSDMConfig = ADC_DFSDM_MODE_ENABLE;
  
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    while(1);
  }
  
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_1;
  sConfig.Offset = 0x800;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
}

static void  MP23ABS1_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
    
  /* DMA1_Channel_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);  
}


static void MP23ABS1_DFSDM_Init(void)
{
  HAL_DFSDM_Filter_RegisterCallback(&hdfsdm1_filter1, HAL_DFSDM_FILTER_MSPINIT_CB_ID, MP23ABS1_DFSDM_FilterMspInit);
  
  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter1.Init.FilterParam.Oversampling = 16;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    Error_Handler();
  }
  
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  hdfsdm1_channel0.Init.OutputClock.Divider = 4;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_ADC_OUTPUT;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0x00;
  hdfsdm1_channel0.Init.RightBitShift = 0x08;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  
  HAL_DFSDM_Filter_RegisterCallback(&hdfsdm1_filter1, HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID, DFSDM_Filter_1_HalfComplete_Callback);
  HAL_DFSDM_Filter_RegisterCallback(&hdfsdm1_filter1, HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID, DFSDM_Filter_1_Complete_Callback);
  
}

void MP23ABS1_OS_Init(void)
{
  /* Thread definition */  
  osThreadDef(MP23ABS1_RD_USR_THREAD, MP23ABS1_Thread, MP23ABS1_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);  
  /* Start thread */
  MP23ABS1_Thread_Id = osThreadCreate(osThread(MP23ABS1_RD_USR_THREAD), NULL);
  /* Suspend thread */
  osThreadSuspend(MP23ABS1_Thread_Id);
  
  amicDreadyQueue_id = osMessageCreate(osMessageQ(amicdreadyqueue), NULL);  
}


static void MP23ABS1_Thread(void const *argument)
{
  (void) argument;
  
#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_MP23ABS1_DEBUG_PIN))
  vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t)TASK_MP23ABS1_DEBUG_PIN );
#endif
  
#if (HSD_USE_FAKE_DATA == 1)
  static uint16_t fakeDataCounter = 0;
#endif
  
  osEvent evt;
  for (;;)
  {
    if (MP23ABS1_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      tim_value_old = 0;
      HAL_ADC_Start(&hadc1);
      HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, (int32_t*)amic_mem, (AMIC_SAMPLING_FREQUENCY / 1000) * AMIC_MS * 2);
      MP23ABS1_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if(MP23ABS1_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {  
      evt = osMessageGet(amicDreadyQueue_id, osWaitForever);  
      
      if(MP23ABS1_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {      
        void * data_ptr = evt.value.p; /* void since it is independent from data format*/
        /* Do something */
        uint16_t idx = 0;
        
#if (HSD_USE_FAKE_DATA != 1)
        static int oldIn = 0;
        static int oldOut = 0;
        int32_t * p32 = (int32_t *)data_ptr;
#endif
        int16_t * p16 = (int16_t *)data_ptr;
        
        for (idx = 0; idx < ((AMIC_SAMPLING_FREQUENCY/1000) * AMIC_MS) ; idx++)
        {        
#if (HSD_USE_FAKE_DATA == 1)
          *p16++ = fakeDataCounter++;
#else
          *p16++ = oldOut = (0xFC * (oldOut + ((*p32)>>10) - oldIn)) / 0xFF;
          oldIn = (*p32 ++)>>10;
#endif        
        }
        
        MP23ABS1_Data_Ready(0, (uint8_t *)data_ptr, (AMIC_SAMPLING_FREQUENCY / 1000) * AMIC_MS * 2, TimeStamp_mp23abs1);
      } 
    } 
    else if ( MP23ABS1_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_FAKE_DATA == 1)
      fakeDataCounter = 0;
#endif      
      HAL_ADC_Stop(&hadc1);
      MP23ABS1_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      ts_mp23abs1 = 0;
      osThreadSuspend(MP23ABS1_Thread_Id);
    }    
  }
}  



/**
* @brief  Regular conversion complete callback.
* @note   This function performs an HP filter in order to remove DC offset and arranges PCM data following the standard PCM format.
* @param  hdfsdm_filter : DFSDM filter handle.
* @retval None
*/
uint16_t amicTest = 0;
extern uint32_t dmic_mem[];

void DFSDM_Filter_1_Complete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) 
{
  TimeStamp_mp23abs1 = SM_GetTimeStamp_fromISR();
  
  if(hdfsdm_filter == &hdfsdm1_filter1)  /* Analog Mic */  
  {
    osMessagePut(amicDreadyQueue_id, (uint32_t)(&amic_mem[((AMIC_SAMPLING_FREQUENCY/1000) * AMIC_MS)]), osWaitForever);
  }
  
}

/**
* @brief  Half regular conversion complete callback.
* @param  hdfsdm_filter : DFSDM filter handle.
* @retval None
*/
void DFSDM_Filter_1_HalfComplete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) 
{ 
  TimeStamp_mp23abs1 = SM_GetTimeStamp_fromISR();
  
  if(hdfsdm_filter == &hdfsdm1_filter1)  /* Analog Mic */  
  {
    osMessagePut(amicDreadyQueue_id, (uint32_t)(amic_mem), osWaitForever);
  }
}



void MP23ABS1_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef* hdfsdm_filter)
{
  /* Peripheral clock enable */
  __HAL_RCC_DFSDM1_CLK_ENABLE();
  
  /* DFSDM1_FLT1 Init */
  if(hdfsdm_filter->Instance == DFSDM1_Filter1){
    hdma_dfsdm1_flt1.Instance = DMA1_Channel6;
    hdma_dfsdm1_flt1.Init.Request = DMA_REQUEST_DFSDM1_FLT1;
    hdma_dfsdm1_flt1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dfsdm1_flt1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dfsdm1_flt1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dfsdm1_flt1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dfsdm1_flt1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dfsdm1_flt1.Init.Mode = DMA_CIRCULAR;
    hdma_dfsdm1_flt1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_dfsdm1_flt1) != HAL_OK)
    {
      Error_Handler();
    }
    
    /* Several peripheral DMA handle pointers point to the same DMA handle.
    Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hdfsdm_filter,hdmaInj,hdma_dfsdm1_flt1);
    __HAL_LINKDMA(hdfsdm_filter,hdmaReg,hdma_dfsdm1_flt1);
  }
}

void MP23ABS1_StartAcquisition(void)
{
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, (int32_t*)amic_mem, (AMIC_SAMPLING_FREQUENCY / 1000) * AMIC_MS * 2);
}

void MP23ABS1_StopAcquisition(void)
{
  HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter1);
}

void MP23ABS1_PauseAcquisition(void)
{
  HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter1);
}

void MP23ABS1_ResumeAcquisition(void)
{
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, (int32_t*)amic_mem, (AMIC_SAMPLING_FREQUENCY / 1000) * AMIC_MS * 2);
}

void MP23ABS1_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();
    
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN1
    PC1     ------> ADC1_IN2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}

void MP23ABS1_Set_State(SM_Sensor_State_t state)
{
  MP23ABS1_Sensor_State = state;
}

void MP23ABS1_Start(void)
{
  MP23ABS1_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(MP23ABS1_Thread_Id);
}

void MP23ABS1_Stop(void)
{
  MP23ABS1_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void MP23ABS1_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
