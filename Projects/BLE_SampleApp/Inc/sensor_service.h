/**
  ******************************************************************************
  * @file    sensor_service.h 
  * @author  SRA
  * @version v1.4.0
  * @date    13-Nov-2020
  * @brief   Sensors services APIs
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
#ifndef _SENSOR_SERVICE_H_
#define _SENSOR_SERVICE_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"

#include "bluenrg1_gatt_server.h"
#include "bluenrg1_gap.h"
#include "string.h"
#include "bluenrg1_gap_aci.h"
#include "bluenrg1_gatt_aci.h"
#include "hci_const.h"
#include "bluenrg1_hal_aci.h" 
#include "hci.h"
#include "bluenrg1_hci_le.h"
#include "sm.h"
#include "gatt_db.h"


#include <stdlib.h>
#include "main.h"

/* Exported functions ------------------------------------------------------- */
extern tBleStatus Add_HWServW2ST_Service(void);
extern tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1);

extern tBleStatus LED_Update(uint8_t LedStatus);

extern tBleStatus Stderr_Update(uint8_t *data,uint8_t length);
extern tBleStatus Term_Update(uint8_t *data,uint8_t length);

extern tBleStatus Add_ConfigW2ST_Service(void);
extern tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t val);

extern void setConnectable(void);
extern void HCI_Event_CB(void *pckt);

/* Exported constants --------------------------------------------------------*/

/* For enabling the capability to handle BlueNRG Congestion */
#define ACC_BLUENRG_CONGESTION

#ifdef ACC_BLUENRG_CONGESTION
/* For defining how many events skip when there is a congestion */
#define ACC_BLUENRG_CONGESTION_SKIP 30
#endif /* ACC_BLUENRG_CONGESTION */

/*************** Don't Change the following defines *************/

/* Define the Max dimesion of the Bluetooth characteristics
for each packet used for Console Service */
#define W2ST_CONSOLE_MAX_CHAR_LEN 20

/* Define the symbol used for defining each termination string
used in Console service */
#define W2ST_CONSOLE_END_STRING "\0"

/* @brief  Scale factor. It is used to scale acceleration from mg to g */ 
#define FROM_MG_TO_G    0.001

/* Feature mask for Accelerometer events */
#define FEATURE_MASK_ACC_EVENTS 0x00000400

/* Feature mask for LED */
#define FEATURE_MASK_LED 0x20000000

/* BLE Characteristic connection control */
/* Environmental Data */
#define W2ST_CONNECT_ENV           (1    )
/* LED status */
#define W2ST_CONNECT_LED           (1<<1 )
/* Acceleration/Gyroscope/Magneto */
#define W2ST_CONNECT_ACC_GYRO_MAG  (1<<2 )

/* Standard Terminal */
#define W2ST_CONNECT_STD_TERM      (1<<8 )

/* Standard Error */
#define W2ST_CONNECT_STD_ERR       (1<<9 )

/* HW Advance Features */
#define W2ST_CONNECT_ACC_EVENT     (1<<10)

/* Gas Gouge Feature */
#define W2ST_CONNECT_GG_EVENT      (1<<11)

#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))

#ifdef __cplusplus
}
#endif

#endif /* _SENSOR_SERVICE_H_ */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
