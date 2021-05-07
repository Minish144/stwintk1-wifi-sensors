/**
  ******************************************************************************
  * @file    iis3dwb_settings.h
  * @author  SRA
  * @version v1.3.0
  * @date    13-Nov-2020
  * @brief   This file contains definitions for the IIS3DWB settings
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
#ifndef __IIS3DWB_SETTINGS_H__
#define __IIS3DWB_SETTINGS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "iis3dwb.h"
                      
#define IIS3DWB_ACC_ODR 6667.0f  
#define IIS3DWB_ACC_FS 2 /* FS = 2g */

#ifdef __cplusplus
}
#endif

#endif /* __IIS3DWB_SETTINGS_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
