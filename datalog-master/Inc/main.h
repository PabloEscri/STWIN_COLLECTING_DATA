/**
  ******************************************************************************
  * @file    main.h
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief   This file contains all the functions prototypes for the main.c
  *          file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_conf.h"
  
#include "STWIN.h"
#include "STWIN_debug_pins.h"
  
#ifndef M_PI   
  #define M_PI   3.14159265358979323846264338327950288
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define HSD_BLE_ENABLE                  1
#define HSD_BLE_STATUS_TIMER_ENABLE     1

/*
 * SD task priority shoul be lower than data aquisition tasks
 */
#define SD_THREAD_PRIO                  osPriorityNormal

/* 
 * BLE threads are not critical and they shouldn't interfere with the data acquisition,
 * so they have a lower priority than the others.
 * BLE_USER_EVT_PROC_THREAD priority must be lower than BLE_SEND_THREAD priority.
 * Otherwise the EVT_PROC thread intecepts a message that should instead be received
 * by the BLE_SEND thread.
 */
#define BLE_USER_EVT_PROC_THREAD_PRIO   osPriorityLow
#define BLE_INIT_THREAD_PRIO            osPriorityBelowNormal 
#define BLE_SEND_THREAD_PRIO            osPriorityBelowNormal

/*
 * Priority should be high so that there isn't any data loss
 */
#define I2C_RD_THREAD_PRIO              osPriorityAboveNormal
#define SPI_RD_THREAD_PRIO              osPriorityAboveNormal
#define IIS3DWB_THREAD_PRIO             osPriorityAboveNormal
#define IIS2MDC_THREAD_PRIO             osPriorityAboveNormal
#define IIS2DH_THREAD_PRIO              osPriorityAboveNormal
#define STTS751_THREAD_PRIO             osPriorityAboveNormal
#define LPS22HH_THREAD_PRIO             osPriorityAboveNormal
#define HTS221_THREAD_PRIO              osPriorityAboveNormal
#define IMP34DT05_THREAD_PRIO           osPriorityAboveNormal
#define MP23ABS1_THREAD_PRIO            osPriorityAboveNormal
#define ISM330DHCX_THREAD_PRIO          osPriorityAboveNormal


/* 
 * Set configUSE_APPLICATION_TASK_TAG to 1 in FreeRTOSConfig.h to enable the Task debugging mode.
 * Each time a task is executing the corresponding pin is SET otherwise is RESET
 */
#define HSD_TASK_DEBUG_PINS_ENABLE    0
    
#define TASK_IDLE_DEBUG_PIN                     DEBUG_PIN7        /* IDLE Task Pin cannot be changed. */
#define TASK_IIS3DWB_DEBUG_PIN                  DEBUG_PIN8
#define TASK_SDM_DEBUG_PIN                      DEBUG_PIN9
#define TASK_IIS2MDC_DEBUG_PIN                  DEBUG_PIN10
//#define TASK_STTS751_DEBUG_PIN                DEBUG_PIN11
//#define TASK_LPS22HH_DEBUG_PIN                DEBUG_PIN12
//#define TASK_HTS221_DEBUG_PIN                 DEBUG_PIN13
#define TASK_IMP34DT05_DEBUG_PIN                DEBUG_PIN14
#define TASK_MP23ABS1_DEBUG_PIN                 DEBUG_PIN17
#define TASK_ISM330DHCX_DEBUG_PIN               DEBUG_PIN18
#define TASK_SM_SPI_DEBUG_PIN                   DEBUG_PIN19
#define TASK_SM_I2C_DEBUG_PIN                   DEBUG_PIN20
#define TASK_BLE_USER_EVT_PROC_DEBUG_PIN        DEBUG_PIN11
#define TASK_BLE_INIT_DEBUG_PIN                 DEBUG_PIN12
#define TASK_BLE_SEND_DEBUG_PIN                 DEBUG_PIN13
 

/* Exported macro ------------------------------------------------------------*/  

/*
 * HSD_USE_FAKE_DATA, if enabled, replaces real sensor data with a 2 bytes idependend counter
 * for each sensor. Useful to debug the complete application and verify that data are stored or
 * streamed correctly.
 * Sensitivity parameter is forced to to 1.
 * Timestamp is not replaced, if you want to disable it, set samplesPerTS parameter to 0.
 */
#define HSD_USE_FAKE_DATA       0

/* Select the SD Logging Mode: Continuous or intermittent */
#define HSD_SD_LOGGING_MODE     HSD_SD_LOGGING_MODE_INTERMITTENT

/* Memory management macros */
#ifdef __ICCARM__
  /* Use malloc debug functions (IAR only) */
  #define IAR_MALLOC_DEBUG      0
#endif
   
#if (IAR_MALLOC_DEBUG == 1)
 #define HSD_malloc               malloc_debug
 #define HSD_calloc               calloc_debug
#else
 #define HSD_malloc               malloc
 #define HSD_calloc               calloc
#endif
#define HSD_free                 free
#define HSD_memset               memset
#define HSD_memcpy               memcpy
  
/* Exported variables ------------------------------------------------------- */
extern uint8_t iis3dwb_com_id;
extern uint8_t hts221_com_id;
extern uint8_t iis2dh_com_id;
extern uint8_t iis2mdc_com_id;
extern uint8_t imp34dt05_com_id;
extern uint8_t mp23abs1_com_id;
extern uint8_t ism330dhcx_com_id;
extern uint8_t lps22hh_com_id;
extern uint8_t stts751_com_id;
extern uint8_t mlc_com_id;

void *malloc_debug(size_t size);
void *calloc_debug(size_t num, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
