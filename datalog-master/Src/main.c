/**
******************************************************************************
* @file    main.c
* @author  SRA

* @version v3.0.0
* @date    19-Jun-2020
* @brief   Main program body
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
#include "main.h"

#include "mp23abs1_app.h"
#include "imp34dt05_app.h"
#include "ism330dhcx_app.h"
#include "iis3dwb_app.h"
#include "iis2mdc_app.h"
#include "iis2dh_app.h"
#include "hts221_app.h"
#include "lps22hh_app.h"
#include "stts751_app.h"

#include "usbd_desc.h"
#include "usbd_wcid_streaming.h"
#include "usbd_wcid_interface.h"

#include "sdcard_manager.h"

#include "ble_config_service.h"
#include "ble_comm_manager.h"

//#include "stm32l4xx_hal_rtc.h"

#include "cpu_utils.h"
#include "HSD_tags.h"
#include "HSD_json.h"

/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device;
extern volatile COM_Device_t COM_device;

uint8_t iis3dwb_com_id;
uint8_t hts221_com_id;
uint8_t iis2dh_com_id;
uint8_t iis2mdc_com_id;
uint8_t imp34dt05_com_id;
uint8_t mp23abs1_com_id;
uint8_t ism330dhcx_com_id;
uint8_t lps22hh_com_id;
uint8_t stts751_com_id;
uint8_t mlc_com_id;

//RTC_HandleTypeDef hrtc;

/* ****** */
extern osSemaphoreId bleSendThreadSem_id;
extern uint8_t commandMask;
extern uint8_t SD_Logging_Active;
extern uint8_t SD_present;

EXTI_HandleTypeDef BC_exti;
volatile uint32_t t_stwin = 0;

extern uint32_t t_start;


#if (HSD_SD_LOGGING_MODE == HSD_SD_LOGGING_MODE_INTERMITTENT)
extern uint32_t SD_Logging_Time_Start;
extern uint32_t SD_Logging_Enabled;
#endif

/* Private function prototypes -----------------------------------------------*/
void HSD_traceTASK_SWITCHED_IN(int pxTaskTag);
void HSD_traceTASK_SWITCHED_OUT(int pxTaskTag);

//static void MX_RTC_Init(void);
static void Peripheral_MSP_Init_All(void);
static void Peripheral_OS_Init_All(void);
static void BattChrg_Init(void);
static void BC_Int_Callback(void);
static void Error_Handler(void);
void PVD_Config(void);
void SystemClock_Config(void);
void MX_USB_DEVICE_Init(void);
static void RND_Init(void);
volatile uint8_t HSD_ResetUSB = 0;

/**
* @brief  Main program
* @param  None
* @retval None
*/  
int main(void)
{
  HAL_Init();
  
  SystemClock_Config();

  /* Enable Power Clock*/
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddUSB(); 
  HAL_PWREx_EnableVddIO2();
  BSP_Enable_DCDC2();


  /* Configure the Battery Charger */
  BattChrg_Init();
  
  /* Configure Power Voltage Detector(PVD) to detect if battery voltage is low */
  PVD_Config();
  
  /* Configure DEBUG PIN and LED */
  BSP_DEBUG_PIN_Init_All(); 
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Initialize srand() using STM32 TRNG peripheral */
  RND_Init();
  
  HSD_JSON_set_allocation_functions(HSD_malloc, HSD_free);
  
  /* Start USB */  
  MX_USB_DEVICE_Init();
  HAL_Delay(100);
  
  /* Set default device description */
  set_default_description();
  
  HSD_TAGS_init(COM_GetDevice());
  
  /* USER Button initialization */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  BSP_PB_PWR_Init();
  
  /* Sensor Manager initialization */
  SM_Peripheral_Init();
  SM_OS_Init();
  Peripheral_MSP_Init_All();
  
  /* SD card Manager initialization */
  SDM_Peripheral_Init();
  
  Peripheral_OS_Init_All();  
  SDM_OS_Init();


 /* MX_RTC_Init();
      RTC_TimeTypeDef sTime;

      sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
      sTime.Hours = 18;
      sTime.Minutes = 18;
      sTime.Seconds=15;
      sTime.SecondFraction = 0;
      sTime.TimeFormat = RTC_HOURFORMAT12_PM;
      sTime.SubSeconds = 0;
      uint32_t Format =  RTC_FORMAT_BIN;

      HAL_RTC_SetTime(&hrtc, &sTime, Format);*/

#if (HSD_BLE_ENABLE == 1)
  BLE_CM_OS_Init(); 
#endif
  
  /* Start scheduler */
  osKernelStart();
  
  while(1);
  
}


/*static void MX_RTC_Init(void)
{
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
}*/

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}


/**
* Init USB device Library, add supported class and start the library
* @retval None
*/
void MX_USB_DEVICE_Init(void)
{
  USBD_Init(&USBD_Device, &WCID_STREAMING_Desc, 0);
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_WCID_STREAMING_CLASS);
  /* Add Interface callbacks for AUDIO and CDC Class */
  USBD_WCID_STREAMING_RegisterInterface(&USBD_Device, &USBD_WCID_STREAMING_fops);
  /* Start Device Process */  
  USBD_Start(&USBD_Device);
}

/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  //HAL_PWR_EnableBkUpAccess();
  //__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE ;
 // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE ;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  //RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV5;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C2
    |RCC_PERIPHCLK_DFSDM1|RCC_PERIPHCLK_USB|RCC_PERIPHCLK_SDMMC1
      |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_DFSDM1AUDIO|RCC_PERIPHCLK_RNG;
 /* PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C2
      |RCC_PERIPHCLK_DFSDM1|RCC_PERIPHCLK_USB|RCC_PERIPHCLK_SDMMC1
        |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_DFSDM1AUDIO|RCC_PERIPHCLK_RNG | RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;*/
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.Dfsdm1AudioClockSelection = RCC_DFSDM1AUDIOCLKSOURCE_SAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLP;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 5;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 96;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV25;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADC1CLK;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }  
 // HAL_RCCEx_EnableLSECSS();
}


void BattChrg_Init(void)
{
  BSP_BC_Init();
  BSP_BC_BatMS_Init();
  BSP_BC_CmdSend(BATMS_ON);  
  
  HAL_EXTI_GetHandle(&BC_exti, EXTI_LINE_10);  
  HAL_EXTI_RegisterCallback(&BC_exti,  HAL_EXTI_COMMON_CB_ID, BC_Int_Callback);
  
  t_stwin = HAL_GetTick(); 
}

/**
* @brief  Battery Charger Interrupt callback
* @param  None
* @retval None
*/
void BC_Int_Callback(void)
{
  if(HAL_GetTick() - t_stwin > 4000)
  {
    BSP_BC_CmdSend(SHIPPING_MODE_ON);
  }
}


/**
* @brief  Configures the PVD resources.
* @param  None
* @retval None
*/
void PVD_Config(void)
{
  PWR_PVDTypeDef sConfigPVD;
  
  /*##-1- Enable Power Clock #################################################*/
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /*##-2- Configure the NVIC for PVD #########################################*/
  HAL_NVIC_SetPriority(PVD_PVM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(PVD_PVM_IRQn);
  
  /* Configure the PVD Level to 6 and generate an interrupt on falling
  edge(PVD detection level set to 2.9V, refer to the electrical characteristics
  of you device datasheet for more details) */
  sConfigPVD.PVDLevel = PWR_PVDLEVEL_6;
  sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING;
  HAL_PWR_ConfigPVD(&sConfigPVD);
  
  /* Enable the PVD Output */
  HAL_PWR_EnablePVD();
}

void vApplicationIdleHook( void )
{
  storeIdleHook();
  
#if (HSD_SD_LOGGING_MODE == HSD_SD_LOGGING_MODE_INTERMITTENT) 
  if(SD_Logging_Enabled)
  {
    SDM_AutosaveFile();
  }
#endif
  
  if (!SD_Logging_Active)
  {
    if(com_status == HS_DATALOG_USB_STARTED)
    {
        if(!(HAL_GetTick()%100)) 
        {
          BSP_LED_On(LED_ORANGE);
        }
        else 
        {
          if(!(HAL_GetTick()%50)) 
          {
            BSP_LED_Off(LED_ORANGE);
          }
        }        
    }
    else
    {
      if (!BSP_SD_IsDetected())
      {
        if(!(HAL_GetTick()%200)) 
        {
          BSP_LED_On(LED_ORANGE);
        }
        else 
        {
          if(!(HAL_GetTick()%100)) 
          {
            BSP_LED_Off(LED_ORANGE);
          }
        }    
      }
      else
      {
        if(!(HAL_GetTick()%1000)) 
        {
          BSP_LED_On(LED_ORANGE);
        }
        else 
        {
          if(!(HAL_GetTick()%50)) 
          {
            BSP_LED_Off(LED_ORANGE);
          }
        }
      }
    }
  }
  
  /** USB reset: to clean the USB buffers **/
  if(HSD_ResetUSB)
  {
    HSD_ResetUSB = 0;
    HAL_Delay(1000);
    USBD_Stop(&USBD_Device);
    HAL_Delay(1000);
    USBD_Start(&USBD_Device);
  }
}

void HSD_traceTASK_SWITCHED_IN(int pxTaskTag)
{
#if (HSD_TASK_DEBUG_PINS_ENABLE)
  BSP_DEBUG_PIN_On((Debug_Pin_TypeDef)pxTaskTag);
#endif
  
  StartIdleMonitor();
}

void HSD_traceTASK_SWITCHED_OUT(int pxTaskTag)
{
#if (HSD_TASK_DEBUG_PINS_ENABLE)
  BSP_DEBUG_PIN_Off((Debug_Pin_TypeDef)pxTaskTag);
#endif
  
  EndIdleMonitor();
}



/**
* @brief  Sensor Data Ready generic callback. Takes the latest data coming from
*         a sensor and send it to the active communication interface.
* @param  sId: Sensor Id
* @param  buf: input data buffer
* @param  size: input data buffer size [bytes]
* @param  timeStamp: timestamp of the latest sample in the input buffer
* @retval 
*/
void SENSOR_Generic_Data_Ready(uint8_t sensorId, uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{ 
  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(sensorId, subSensorId); 
  COM_SubSensorDescriptor_t *pSubSensorDescriptor = COM_GetSubSensorDescriptor(sensorId, subSensorId);
  COM_SubSensorContext_t *pSubSensorContext = COM_GetSubSensorContext(sensorId, subSensorId);
  uint16_t samplesToSend = 0, nBytesPerSample = 0;
  
  if (pSubSensorContext->first_dataReady) // Discard first set of sensor data
  {
    pSubSensorStatus->initialOffset = (float)timeStamp;

    pSubSensorContext->first_dataReady = 0;
    pSubSensorContext->n_samples_acc = 0.0f;
    pSubSensorContext->old_time_stamp = timeStamp;
    pSubSensorContext->n_samples_to_timestamp = pSubSensorStatus->samplesPerTimestamp;
  }
  else
  {
    if(pSubSensorDescriptor->dataType == DATA_TYPE_FLOAT || pSubSensorDescriptor->dataType == DATA_TYPE_INT32 || pSubSensorDescriptor->dataType == DATA_TYPE_UINT32) 
    {
      nBytesPerSample = pSubSensorDescriptor->dimensions * 4;
    }
    else if(pSubSensorDescriptor->dataType == DATA_TYPE_UINT16 || pSubSensorDescriptor->dataType == DATA_TYPE_INT16) 
    {
      nBytesPerSample = pSubSensorDescriptor->dimensions * 2;
    }
    else if(pSubSensorDescriptor->dataType == DATA_TYPE_UINT8 || pSubSensorDescriptor->dataType == DATA_TYPE_INT8) 
    {
      nBytesPerSample = pSubSensorDescriptor->dimensions;
    }
    
//    uint8_t totalDataPerSample = 0;
//    for (int i = 0; i < pSensorDescriptor->nSubSensors; i++)
//    {
//      if (pSensorStatus->subSensorStatus[i].isActive)
//      {
//        pSubSensorDescriptor = COM_GetSubSensorDescriptor(sensorId, i);
//        totalDataPerSample += pSubSensorDescriptor->dataPerSample;
//      }
//    }
//    nBytesPerSample *= totalDataPerSample;
    
//    nBytesPerSample *= pSubSensorDescriptor->dimensions;
    pSubSensorContext->n_samples_acc = (float)(size / nBytesPerSample);

    /* Both Digital and Analog microphones are sampled using STM32 clock (DFSDM or ADC) so the
     * measuredODR should be put equal to ODR */
    /* measuredODR has no meaning for MLC subsensor in ISM330DHCX */
    if(sensorId == imp34dt05_com_id || sensorId == mp23abs1_com_id || (sensorId == ism330dhcx_com_id && subSensorId == 2))
    {
      pSubSensorStatus->measuredODR = pSubSensorStatus->ODR;
    }
    else
    {
      pSubSensorStatus->measuredODR = pSubSensorContext->n_samples_acc/(timeStamp - pSubSensorContext->old_time_stamp);
    }
    pSubSensorContext->old_time_stamp = timeStamp;
    samplesToSend = size/nBytesPerSample;
    
    while(samplesToSend > 0)
    {
      if(samplesToSend < pSubSensorContext->n_samples_to_timestamp || pSubSensorContext->n_samples_to_timestamp == 0)
      { /* Pass the complete buffer at once since there is enough space before next Timestamp */
        if(com_status == HS_DATALOG_SD_STARTED)
        {
          SDM_Fill_Buffer(sensorId, subSensorId, (uint8_t *)buf, samplesToSend * nBytesPerSample);
        }
        else if (com_status == HS_DATALOG_USB_STARTED )
        {
          USBD_WCID_STREAMING_FillTxDataBuffer(&USBD_Device, pSubSensorStatus->comChannelNumber, (uint8_t *)buf,
                                               samplesToSend*nBytesPerSample);
        }
        if(pSubSensorContext->n_samples_to_timestamp != 0)
        {
          pSubSensorContext->n_samples_to_timestamp -= samplesToSend;
        }        
        samplesToSend = 0;
      }
      else
      {
        /* Pass only a part of the buffer (or the whole buffer if "samplesToSend==pSubSensorContext->n_samples_to_timestamp"),
         * then pass the TimeStamp, remaining part (if any) is managed in the next iteration of the loop */
        if(com_status == HS_DATALOG_SD_STARTED)
        {
          SDM_Fill_Buffer(sensorId, subSensorId, (uint8_t *)buf, pSubSensorContext->n_samples_to_timestamp * nBytesPerSample);
        }
        else if (com_status == HS_DATALOG_USB_STARTED )
        {
          USBD_WCID_STREAMING_FillTxDataBuffer(&USBD_Device, pSubSensorStatus->comChannelNumber, (uint8_t *)buf,
                                                pSubSensorContext->n_samples_to_timestamp * nBytesPerSample);
        }

        buf += pSubSensorContext->n_samples_to_timestamp*nBytesPerSample;
        samplesToSend -= pSubSensorContext->n_samples_to_timestamp;
        
        double newTS;
        if(pSubSensorStatus->measuredODR != 0.0f)
        {
            newTS = timeStamp - ((1.0 / (double)pSubSensorStatus->measuredODR) * samplesToSend);
        }
        else
        {
            newTS = timeStamp;
        }
        
        if(com_status == HS_DATALOG_SD_STARTED)
        {
          SDM_Fill_Buffer(sensorId, subSensorId, (uint8_t *)&newTS, 8);
        }
        else if (com_status == HS_DATALOG_USB_STARTED )
        {
          USBD_WCID_STREAMING_FillTxDataBuffer(&USBD_Device, pSubSensorStatus->comChannelNumber, (uint8_t *)&newTS, 8);
        }
        pSubSensorContext->n_samples_to_timestamp = pSubSensorStatus->samplesPerTimestamp;
      }      
    }    
  }    
}


/*  ---------- Sensors data ready functions ----------- */
void IIS3DWB_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{  
  SENSOR_Generic_Data_Ready(iis3dwb_com_id, subSensorId, buf, size, timeStamp);
}

void HTS221_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(hts221_com_id, subSensorId, buf, size, timeStamp);
}

void IIS2DH_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(iis2dh_com_id, subSensorId, buf, size, timeStamp);
}

void IIS2MDC_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(iis2mdc_com_id, subSensorId, buf, size, timeStamp);
}

void IMP34DT05_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(imp34dt05_com_id, subSensorId, buf, size, timeStamp);
}

void ISM330DHCX_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(ism330dhcx_com_id, subSensorId, buf, size, timeStamp);
}

void LPS22HH_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(lps22hh_com_id, subSensorId, buf, size, timeStamp);
}

void MP23ABS1_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(mp23abs1_com_id, subSensorId, buf, size, timeStamp);
}

void STTS751_Data_Ready(uint8_t subSensorId, uint8_t * buf, uint16_t size, double timeStamp)
{
  SENSOR_Generic_Data_Ready(stts751_com_id, subSensorId, buf, size, timeStamp);
}


void Peripheral_MSP_Init_All(void)
{  
  IIS2MDC_Peripheral_Init();
  STTS751_Peripheral_Init();
  LPS22HH_Peripheral_Init();
  HTS221_Peripheral_Init();
  IMP34DT05_Peripheral_Init();
  MP23ABS1_Peripheral_Init();
  ISM330DHCX_Peripheral_Init();
  IIS3DWB_Peripheral_Init();
  IIS2DH_Peripheral_Init();  
}


void Peripheral_OS_Init_All(void)
{
  IIS2MDC_OS_Init();
  STTS751_OS_Init();
  LPS22HH_OS_Init();
  HTS221_OS_Init();
  IMP34DT05_OS_Init();
  MP23ABS1_OS_Init();
  ISM330DHCX_OS_Init();
  IIS3DWB_OS_Init();
  IIS2DH_OS_Init();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
  case USER_BUTTON_PIN:
    if( HAL_GetTick() - t_start > 1000 )
    {
      if (com_status == HS_DATALOG_IDLE || com_status == HS_DATALOG_SD_STARTED )
      {  		
        // Cannot wait since we are in an ISR
        if(osMessagePut(sdThreadQueue_id, SDM_START_STOP, 0) != osOK)
        {
          while (1){}
        }
        
        commandMask = COM_REQUEST_STATUS_LOGGING;
#if (HSD_BLE_ENABLE == 1)
        osSemaphoreRelease(bleSendThreadSem_id);
#endif
        t_start = HAL_GetTick();
        
#if (HSD_SD_LOGGING_MODE == HSD_SD_LOGGING_MODE_INTERMITTENT)
        t_start = SD_Logging_Time_Start = HAL_GetTick();
        
        if (SD_Logging_Enabled == 1)
          SD_Logging_Enabled = 0;
        else 
          SD_Logging_Enabled = 1;
#endif        
      }
    }
#if (HSD_BLE_ENABLE == 1)
  case BLE_CM_SPI_EXTI_PIN:
    {
      hci_tl_lowlevel_isr();
    }
    break;
#endif
  default:
    break;
  }
}


/**
  * @brief RNG MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param hrng: RNG handle pointer
  * @retval None
  */
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{  
  /* RNG Peripheral clock enable */
  __HAL_RCC_RNG_CLK_ENABLE();
}

/**
  * @brief RNG MSP De-Initialization
  *        This function freeze the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hrng: RNG handle pointer
  * @retval None
  */
void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng)
{
  /* Enable RNG reset state */
  __HAL_RCC_RNG_FORCE_RESET();

  /* Release RNG from reset state */
  __HAL_RCC_RNG_RELEASE_RESET();
  
  __HAL_RCC_RNG_CLK_DISABLE();
}

/**
  * @brief Random function initialization.
  *        Initialize srand() using a random number generated by RNG peripheral
  * @param None
  * @retval None
  */
void RND_Init(void)
{
  RNG_HandleTypeDef RngHandle;

  /* Used for storing Random 32bit Number */
  uint32_t aRandom32bit;

  /* Configure the RNG peripheral */
  RngHandle.Instance = RNG;
  
  /* DeInitialize the RNG peripheral */
  if(HAL_RNG_DeInit(&RngHandle) != HAL_OK)
  {
    /* DeInitialization Error */
    Error_Handler();
  }    

  /* Initialize the RNG peripheral */
  if(HAL_RNG_Init(&RngHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  if (HAL_RNG_GenerateRandomNumber(&RngHandle, &aRandom32bit) != HAL_OK)
  {
    /* Random number generation error */
    Error_Handler();      
  }

  HAL_RNG_DeInit(&RngHandle);
  
  /* Initialize srand() using the random number generated by RNG peripheral */
  srand(aRandom32bit);
}


void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
  while(1);
}

/**
* @brief  This function is executed in case of error occurrence
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
  while(1);
}


/*
 * Enable IAR_MALLOC_DEBUG in main.h to debug heap
 * You can monitor "m" struct in live watch
 */
#if (IAR_MALLOC_DEBUG == 1)
#include "mallocstats.h"

volatile size_t minFreeHeap = 0x75000;
volatile struct mallinfo m;

void *malloc_debug(size_t size)
{
  void *p = malloc(size);
  
  m = __iar_dlmallinfo();
  if(m.fordblks < minFreeHeap)
  {
    minFreeHeap = m.fordblks;
  }
  return p;
}

void *calloc_debug(size_t num, size_t size)
{
  void *p = calloc(num, size);
  
  m = __iar_dlmallinfo();
  if(m.fordblks < minFreeHeap)
  {
    minFreeHeap = m.fordblks;
  }
  return p;
}
#endif


#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
