/**
  ******************************************************************************
  * @file    ism330dhc_settings.h
  * @author  SRA
  * @version v1.3.0
  * @date    13-Nov-2020
  * @brief   This file contains definitions for the ISM330DLC settings
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
#ifndef __ISM330DHC_SETTINGS_H__
#define __ISM330DHC_SETTINGS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ism330dhcx.h"

#define ISM330DHC_ACC_ODR 52.0f /* ODR = 52Hz */

#define ISM330DHC_ACC_FS 2 /* FS = 2g */

#define ISM330DHC_GYRO_ODR 52.0f /* ODR = 52Hz */

#define ISM330DHC_GYRO_FS 2000 /* FS = 2000dps */

#ifdef __cplusplus
}
#endif

#endif /* __ISM330DHC_SETTINGS_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
