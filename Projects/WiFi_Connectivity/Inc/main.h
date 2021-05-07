/**
  ******************************************************************************
  * @file    main.h 
  * @author  SRA
  * @version v1.4.0
  * @date    13-Nov-2020
  * @brief   Header for main.c module
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
#include "STWIN_conf.h"
#include "STWIN_wifi.h"
#include "STWIN_env_sensors.h"
   
#include "usbd_def.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
   
#include "net_connect.h"
#include "wifi.h"
#include "es_wifi.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void	Periph_Config(void);
void	Error_Handler(void);
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
void HAL_Delay(uint32_t Delay);

uint32_t sys_now(void);
   
#define WIFI_PRINTF(...){\
   char TmpBufferToWrite[800];\
     int32_t TmpBytesToWrite;\
       TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
         CDC_Transmit_FS(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
           HAL_Delay(5);\
 }
    
#ifdef __cplusplus
}
#endif    

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
