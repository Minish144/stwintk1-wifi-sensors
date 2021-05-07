/**
  ******************************************************************************
  * @file    TargetPlatform.c
  * @author  SRA
  * @version v1.4.0
  * @date    13-Nov-2020
  * @brief   Initialization of the Target Platform
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
  
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"

/* Imported variables ---------------------------------------------------------*/
extern USBD_DescriptorsTypeDef FS_Desc;

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

USBD_HandleTypeDef  hUsbDeviceFS;

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void Init_MEM1_Sensors(void);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  TargetType_t BoardType Nucleo/BlueCoin/SensorTile/STWIN
  * @retval None
  */
void InitTargetPlatform(TargetType_t BoardType)
{
  TargetBoardFeatures.BoardType = BoardType;

#ifdef ENABLE_USB_DEBUG
  /* enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  /* Configure the CDC */
  /* Init Device Library */
  USBD_Init(&hUsbDeviceFS, &FS_Desc, 0);
  /* Add Supported Class */
  USBD_RegisterClass(&hUsbDeviceFS, USBD_CDC_CLASS);
  /* Add Interface callbacks for AUDIO and CDC Class */
  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
  /* Start Device Process */
  USBD_Start(&hUsbDeviceFS);
  /* 10 seconds ... for having time to open the Terminal
   * for looking the MOTENV1 Initialization phase */
  HAL_Delay(10000);
#endif /* ENABLE_USB_DEBUG */
  
  /* Initialize LED */
  BSP_LED_Init( LED1 );

  STLBLE_PRINTF("\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
         "\tSTWIN"
          "\r\n",
          STLBLE_PACKAGENAME,
          STLBLE_VERSION_MAJOR,STLBLE_VERSION_MINOR,STLBLE_VERSION_PATCH);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Intialize all the Target's Features */
  Init_MEM1_Sensors();
  
  TargetBoardFeatures.LedStatus = 0; /*Off by default */
  TargetBoardFeatures.bnrg_expansion_board = IDB04A1; /* IDB04A1 by default */
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
  /* Humidity */  
  if(BSP_ENV_SENSOR_Init(HTS221_0, ENV_HUMIDITY)==BSP_ERROR_NONE)
  {
    STLBLE_PRINTF("OK Humidity Sensor\n\r");
    TargetBoardFeatures.HandleHumSensor=1;
  }

  /* Temperature1 */
  if(BSP_ENV_SENSOR_Init(STTS751_0, ENV_TEMPERATURE)==BSP_ERROR_NONE)
  {
     STLBLE_PRINTF("OK Temperature Sensor1\n\r");
     TargetBoardFeatures.NumTempSensors++; 
     TargetBoardFeatures.HandleTempSensors[0]=1;
  }

  /* Temperature2 */
  if(BSP_ENV_SENSOR_Init(LPS22HH_0, ENV_TEMPERATURE)==BSP_ERROR_NONE)  
  {
     STLBLE_PRINTF("OK Temperature Sensor2\n\r");
     TargetBoardFeatures.NumTempSensors++;
     TargetBoardFeatures.HandleTempSensors[1]=1;
  }
  
  /* Pressure */
  if(BSP_ENV_SENSOR_Init(LPS22HH_0, ENV_PRESSURE)==BSP_ERROR_NONE)
  {
    STLBLE_PRINTF("OK Pressure Sensor\n\r");
    TargetBoardFeatures.HandlePressSensor=1;
  }

  if(TargetBoardFeatures.HandleHumSensor) {
    if(BSP_ENV_SENSOR_Enable(HTS221_0, ENV_HUMIDITY)==BSP_ERROR_NONE)
    {
      STLBLE_PRINTF("Enabled Humidity Sensor\n\r");
    }
  }
  
  if(TargetBoardFeatures.HandleTempSensors[0]){
    if(BSP_ENV_SENSOR_Enable(STTS751_0, ENV_TEMPERATURE)==BSP_ERROR_NONE)
    {
      STLBLE_PRINTF("Enabled Temperature Sensor1\n\r");
    }
  }
  
  if(TargetBoardFeatures.HandleTempSensors[1]){
    if(BSP_ENV_SENSOR_Enable( LPS22HH_0, ENV_TEMPERATURE)==BSP_ERROR_NONE) 
    {
      STLBLE_PRINTF("Enabled Temperature Sensor2\n\r");
    }
  }
  
  if(TargetBoardFeatures.HandlePressSensor) {
    if(BSP_ENV_SENSOR_Enable(LPS22HH_0, ENV_PRESSURE)==BSP_ERROR_NONE)
    {
      STLBLE_PRINTF("Enabled Pressure Sensor\n\r");
    }
  }
}

/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
  BSP_LED_On( LED1 );
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
  BSP_LED_Off( LED1 );
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
