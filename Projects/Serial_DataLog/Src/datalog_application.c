/**
  ******************************************************************************
  * @file    datalog_application.c
  * @author  SRA
  * @version v1.3.0
  * @date    13-Nov-2020
  * @brief   This file provides a set of functions to handle the datalog
  *          application.
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
#include "datalog_application.h"
#include "main.h"
#include "usbd_cdc_interface.h"
#include "string.h"
#include "STWIN.h"
#include <math.h>

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private variables ---------------------------------------------------------*/
FRESULT res;                                          /* FatFs function common result code */
uint32_t byteswritten, bytesread;                     /* File write/read counts */
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */
    
volatile uint8_t SD_Log_Enabled = 0;

char newLine[] = "\r\n";

/* Private function prototypes -----------------------------------------------*/
static void MX_DataLogTerminal_Init(void);


/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
void DATALOG_SD_Init(void)
{
  BSP_SD_Detect_Init();

      
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      while(1)
      {
        BSP_LED_On(LED1);
        HAL_Delay(500);
        BSP_LED_Off(LED1);
        HAL_Delay(100);
      }
    }
  }
}
  
/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
uint8_t DATALOG_SD_Log_Enable(void)
{
  static uint16_t sdcard_file_counter = 0;
  char header[] = "T [ms],AccX [mg],AccY [mg],AccZ [mg],GyroX [mdps],GyroY [mdps],GyroZ [mdps],MagX [mgauss],MagY [mgauss],MagZ [mgauss],P [mB],T [°C],H [%]\r\n";
  uint32_t byteswritten; /* written byte count */
  char file_name[30] = {0};
 
  sprintf(file_name, "%s%.3d%s", "ind", sdcard_file_counter, ".csv");
  sdcard_file_counter++;

  HAL_Delay(100);

  if(f_open(&MyFile, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    sdcard_file_counter--;
    return 0;
  }
  
  if(f_write(&MyFile, (const void*)&header, sizeof(header)-1, (void *)&byteswritten) != FR_OK)
  {
    return 0;
  }
  
  return 1;
  
}



uint8_t DATALOG_SD_writeBuf(char *s, uint32_t size)
{
  uint32_t byteswritten;
  
  BSP_LED_Toggle(LED1);

  if(f_write(&MyFile, s, size, (void *)&byteswritten) != FR_OK)
  {
    return 0;
  }
  return 1;
}


/**
  * @brief  Disable SDCard Log
  * @param  None
  * @retval None
  */
void DATALOG_SD_Log_Disable(void)
{
  BSP_LED_Off(LED1);
  
  f_close(&MyFile);

}

void DATALOG_SD_DeInit(void)
{
  FATFS_UnLinkDriver(SDPath);
}

/**
  * @brief  Write New Line to file
  * @param  None
  * @retval None
  */
void DATALOG_SD_NewLine(void)
{
  uint32_t byteswritten; /* written byte count */
  f_write(&MyFile, (const void*)&newLine, 2, (void *)&byteswritten);
}



int32_t getSensorsData( T_SensorsData *mptr)
{
  int32_t ret = BSP_ERROR_NONE;
  mptr->ms_counter = HAL_GetTick();
  
#ifdef USE_IIS2DH
  
  if ( BSP_MOTION_SENSOR_GetAxes( IIS2DH_0, MOTION_ACCELERO, &mptr->acc ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->acc.x = 0;
    mptr->acc.y = 0;
    mptr->acc.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
#elif USE_ISM330DHCX
  
  if ( BSP_MOTION_SENSOR_GetAxes( ISM330DHCX_0, MOTION_ACCELERO, &mptr->acc ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->acc.x = 0;
    mptr->acc.y = 0;
    mptr->acc.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
#endif

  if ( BSP_MOTION_SENSOR_GetAxes(ISM330DHCX_0, MOTION_GYRO, &mptr->gyro ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->gyro.x = 0;
    mptr->gyro.y = 0;
    mptr->gyro.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
  if ( BSP_MOTION_SENSOR_GetAxes(IIS2MDC_0, MOTION_MAGNETO, &mptr->mag ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->mag.x = 0;
    mptr->mag.y = 0;
    mptr->mag.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
  if ( BSP_ENV_SENSOR_GetValue(LPS22HH_0, ENV_PRESSURE, &mptr->pressure ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->pressure = 0.0f;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
  if ( BSP_ENV_SENSOR_GetValue(STTS751_0, ENV_TEMPERATURE, &mptr->temperature ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->temperature = 0.0f;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
  if ( BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_HUMIDITY, &mptr->humidity ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->humidity = 0.0f;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
  return ret;
}

/**
* @brief  Splits a float into two integer values.
* @param  in the float value as input
* @param  out_int the pointer to the integer part as output
* @param  out_dec the pointer to the decimal part as output
* @param  dec_prec the decimal precision to be used
* @retval None
*/
void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec )
{
  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}



void MX_X_CUBE_MEMS1_Init(void)
{
  Sensor_IO_SPI_CS_Init_All();
  MX_DataLogTerminal_Init();
}


/**
  * @brief  Initialize the DataLogTerminal application
  * @retval None
  */
void MX_DataLogTerminal_Init(void)
{
  BSP_MOTION_SENSOR_Init(IIS2MDC_0, MOTION_MAGNETO);
  BSP_MOTION_SENSOR_SetOutputDataRate(IIS2MDC_0, MOTION_MAGNETO, IIS2MDC_MAG_ODR);
  BSP_MOTION_SENSOR_SetFullScale(IIS2MDC_0, MOTION_MAGNETO, IIS2MDC_MAG_FS);
  
#ifdef USE_IIS2DH
  
  BSP_MOTION_SENSOR_Init(IIS2DH_0, MOTION_ACCELERO);
  BSP_MOTION_SENSOR_SetOutputDataRate(IIS2DH_0, MOTION_ACCELERO, IIS2DH_ACC_ODR);
  BSP_MOTION_SENSOR_SetFullScale(IIS2DH_0, MOTION_ACCELERO, IIS2DH_ACC_FS);
    
  BSP_MOTION_SENSOR_Init(ISM330DHCX_0, MOTION_GYRO);
  BSP_MOTION_SENSOR_SetOutputDataRate(ISM330DHCX_0, MOTION_GYRO, ISM330DHC_GYRO_ODR);
  BSP_MOTION_SENSOR_SetFullScale(ISM330DHCX_0, MOTION_GYRO, ISM330DHC_GYRO_FS);
  
#elif USE_ISM330DHCX
  
  BSP_MOTION_SENSOR_Init(ISM330DHCX_0, MOTION_ACCELERO | MOTION_GYRO);
  BSP_MOTION_SENSOR_SetOutputDataRate(ISM330DHCX_0, MOTION_ACCELERO, ISM330DHC_ACC_ODR);
  BSP_MOTION_SENSOR_SetFullScale(ISM330DHCX_0, MOTION_ACCELERO, ISM330DHC_ACC_FS);
  BSP_MOTION_SENSOR_SetOutputDataRate(ISM330DHCX_0, MOTION_GYRO, ISM330DHC_GYRO_ODR);
  BSP_MOTION_SENSOR_SetFullScale(ISM330DHCX_0, MOTION_GYRO, ISM330DHC_GYRO_FS);
  
#endif
  
  BSP_ENV_SENSOR_Init(HTS221_0, ENV_HUMIDITY);
  BSP_ENV_SENSOR_SetOutputDataRate(HTS221_0, ENV_HUMIDITY, HTS221_ODR);
 
  BSP_ENV_SENSOR_Init(LPS22HH_0, ENV_PRESSURE);
  BSP_ENV_SENSOR_SetOutputDataRate(LPS22HH_0, ENV_PRESSURE, LPS22HH_ODR);
  
  BSP_ENV_SENSOR_Init(STTS751_0, ENV_TEMPERATURE);
  BSP_ENV_SENSOR_SetOutputDataRate(STTS751_0, ENV_TEMPERATURE, STTS751_ODR);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
