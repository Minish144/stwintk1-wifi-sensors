/**
  ******************************************************************************
  * @file    datalog_application.h
  * @author  SRA
  * @version v1.3.0
  * @date    13-Nov-2020
  * @brief   Header for datalog_application.c module.
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
#ifndef __DATALOG_APPLICATION_H
#define __DATALOG_APPLICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cube_hal.h"
#include "STWIN_motion_sensors.h"
#include "iis2dh_settings.h"
#include "iis2mdc_settings.h"
#include "ism330dhcx_settings.h"  
#include "STWIN_env_sensors.h"
#include "hts221_settings.h"
#include "lps22hh_settings.h"
#include "stts751_settings.h"  
  
#define USE_IIS2DH 1
//#define USE_ISM330DHCX 1 
  
typedef enum
{
  USB_Datalog = 0,
  SDCARD_Datalog
} LogInterface_TypeDef;


typedef struct
{
  uint32_t ms_counter;
  float pressure;
  float humidity;
  float temperature;
  BSP_MOTION_SENSOR_Axes_t acc;
  BSP_MOTION_SENSOR_Axes_t gyro;
  BSP_MOTION_SENSOR_Axes_t mag;
} T_SensorsData;
  
extern LogInterface_TypeDef LoggingInterface;
extern volatile uint8_t SD_Log_Enabled;

void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec );

void DATALOG_SD_Init(void);
uint8_t DATALOG_SD_Log_Enable(void);
uint8_t DATALOG_SD_writeBuf(char *s, uint32_t size);
void DATALOG_SD_Log_Disable(void);
void DATALOG_SD_DeInit(void);
void DATALOG_SD_NewLine(void);
int32_t getSensorsData( T_SensorsData *mptr);

void MX_X_CUBE_MEMS1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DATALOG_APPLICATION_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
