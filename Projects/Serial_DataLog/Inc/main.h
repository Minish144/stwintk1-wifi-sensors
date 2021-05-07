/**
  ******************************************************************************
  * @file    main.h
  * @author  SRA
  * @version v1.3.0
  * @date    13-Nov-2020
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
#include "STWIN.h"
#include "STWIN_bc.h"
#include "stm32l4xx_hal_conf.h"
#include "stm32l4xx_hal_def.h"
#include "STWIN_motion_sensors.h"
#include "iis2dh_settings.h"
#include "iis2mdc_settings.h"
#include "ism330dhcx_settings.h"  
#include "STWIN_env_sensors.h"
#include "hts221_settings.h"
#include "lps22hh_settings.h"
#include "stts751_settings.h"  
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Includes ------------------------------------------------------------------*/

#include "usbd_def.h"

/** USB device core handle. */
extern USBD_HandleTypeDef hUsbDeviceFS;


/** USB Device initialization function. */
void MX_USB_DEVICE_Init(void);

  
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
