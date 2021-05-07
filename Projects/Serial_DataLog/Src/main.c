/**
  ******************************************************************************
  * @file    main.c
  * @author  SRA
  * @version v1.3.0
  * @date    13-Nov-2020
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
#include "cmsis_os.h"
#include "datalog_application.h"

#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private define ---------------------------------------------------------*/
#define DATAQUEUE_SIZE     ((uint32_t)100)

/* Sensor data acquisition period [ms] */
#define DATA_PERIOD_MS     (20)

#define MSG_ENABLE_DISABLE   (0x00000007)
 
typedef enum
{
  THREAD_1 = 0,
  THREAD_2
} Thread_TypeDef;

/* Private variables ---------------------------------------------------------*/

/* LoggingInterface = USB_Datalog  --> Save sensors data via USB */
/* LoggingInterface = SDCARD_Datalog  --> Save sensors data on SDCard */
LogInterface_TypeDef LoggingInterface = USB_Datalog;

osThreadId GetDataThreadId, WriteDataThreadId;

osMessageQId dataQueue_id;
osMessageQDef(dataqueue, DATAQUEUE_SIZE, int);

osPoolId sensorPool_id;
osPoolDef(sensorPool, DATAQUEUE_SIZE, T_SensorsData);

osSemaphoreId readDataSem_id;
osSemaphoreDef(readDataSem);

osSemaphoreId doubleTapSem_id;
osSemaphoreDef(doubleTapSem);

SD_HandleTypeDef hsd1;
USBD_HandleTypeDef USBD_Device;
/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

static volatile uint8_t MEMSInterrupt = 0;
uint32_t  exec;
volatile uint32_t t_stwin = 0;

/* Private function prototypes -----------------------------------------------*/
static void GetData_Thread(void const *argument);
static void WriteData_Thread(void const *argument);

void dataTimer_Callback(void const *arg);
void dataTimerStart(void);
void dataTimerStop(void);

osTimerId sensorTimId;
osTimerDef(SensorTimer, dataTimer_Callback);

void SystemClock_Config(void);
static void _Error_Handler( void );


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
    HAL_PWREx_EnableVddIO2();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddUSB(); 
  
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
    
  /* Initialize LED */
  BSP_LED_Init(LED1);
  BSP_LED_Off(LED1);
  
  /* Initialize Battery Charger */
  BSP_PB_PWR_Init();  
  BSP_Enable_DCDC2();
  BSP_BC_Init();
  BSP_BC_BatMS_Init();
  BSP_BC_CmdSend(BATMS_ON);
  t_stwin = HAL_GetTick(); 
  
  if(LoggingInterface == USB_Datalog) /* Configure the USB */
  {
    MX_USB_DEVICE_Init();
  }  
  else /* Configure the SDCard */
  {
    DATALOG_SD_Init();
  }
  
  /* Thread 1 definition */
  osThreadDef(THREAD_1, GetData_Thread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE*4);
  
  /* Thread 2 definition */
  osThreadDef(THREAD_2, WriteData_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*4);
  
  /* Start thread 1 */
  GetDataThreadId = osThreadCreate(osThread(THREAD_1), NULL);

  /* Start thread 2 */
  WriteDataThreadId = osThreadCreate(osThread(THREAD_2), NULL);  
  
  /* Start scheduler */
  osKernelStart();
  
  while(1);  
}


/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
  USBD_Start(&hUsbDeviceFS);
}

/**
  * @brief  Send BCD message to user layer
  * @param  hpcd: PCD handle
  * @param  msg: LPM message
  * @retval None
  */
void HAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg)
{
  USBD_HandleTypeDef usbdHandle = hUsbDeviceFS;

  if (hpcd->battery_charging_active == ENABLE)
  {
    switch(msg)
    {
      case PCD_BCD_CONTACT_DETECTION:

      break;

      case PCD_BCD_STD_DOWNSTREAM_PORT:

      break;

      case PCD_BCD_CHARGING_DOWNSTREAM_PORT:

      break;

      case PCD_BCD_DEDICATED_CHARGING_PORT:

      break;

      case PCD_BCD_DISCOVERY_COMPLETED:
        USBD_Start(&usbdHandle);
      break;

      case PCD_BCD_ERROR:
      default:
      break;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static int t_old=0;
  
  if(GPIO_Pin == SD_DETECT_GPIO_PIN)
  {   
      BSP_SD_DetectCallback();
  }
  
  if(GPIO_Pin == USER_BUTTON_PIN)
  {
    if( HAL_GetTick() - t_old > 1000 )
    {
      MEMSInterrupt=1;
      osSemaphoreRelease(readDataSem_id);
      t_old= HAL_GetTick();
    }
  }
  
  if (GPIO_Pin == GPIO_PIN_10)
  {
    if(HAL_GetTick() - t_stwin > 4000)
    {
      BSP_BC_CmdSend(SHIPPING_MODE_ON);
    }
  }
  
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == STBC02_USED_TIM)
  {
    BC_CmdMng();
  }
}


/**
  * @brief  Get data raw from sensors to queue
  * @param  thread not used
  * @retval None
  */
static void GetData_Thread(void const *argument)
{
  (void) argument;
  T_SensorsData *mptr;
  
  sensorPool_id = osPoolCreate(osPool(sensorPool));     
  dataQueue_id = osMessageCreate(osMessageQ(dataqueue), NULL);
  
  readDataSem_id = osSemaphoreCreate(osSemaphore(readDataSem), 1);
  osSemaphoreWait(readDataSem_id, osWaitForever);
  
  doubleTapSem_id = osSemaphoreCreate(osSemaphore(doubleTapSem), 1);
  osSemaphoreWait(doubleTapSem_id, osWaitForever);
  
  /* Initialize and Enable the available sensors */
  MX_X_CUBE_MEMS1_Init();
  
  if(LoggingInterface == USB_Datalog)
  {
    dataTimerStart();
  }
  
  for (;;)
  {
    osSemaphoreWait(readDataSem_id, osWaitForever);
    if(MEMSInterrupt && LoggingInterface == SDCARD_Datalog)
    {
      MEMSInterrupt = 0;
      if(1)
      {
        if(SD_Log_Enabled) 
        {
          dataTimerStop();
          osMessagePut(dataQueue_id, MSG_ENABLE_DISABLE, osWaitForever);
        }
        else
        {
          osMessagePut(dataQueue_id, MSG_ENABLE_DISABLE, osWaitForever);
        }
      }
    }
    else
    {
    /* Try to allocate a memory block and check if is not NULL */
    mptr = osPoolAlloc(sensorPool_id);
    if(mptr != NULL)
    {
      /* Get Data from Sensors */
      if(getSensorsData(mptr) == BSP_ERROR_NONE)
      {
        /* Push the new memory Block in the Data Queue */
        if(osMessagePut(dataQueue_id, (uint32_t)mptr, osWaitForever) != osOK)
        {
          _Error_Handler();
        }
      }
      else
      {
        _Error_Handler();
      }
    }
    else
    {
      _Error_Handler();
    }
  }
}
}


/**
  * @brief  Write data in the queue on file or streaming via USB
  * @param  argument not used
  * @retval None
  */
static void WriteData_Thread(void const *argument)
{
  (void) argument;
  osEvent evt;
  T_SensorsData *rptr;
  int size;
  char data_s[256];
  
  for (;;)
  {
    evt = osMessageGet(dataQueue_id, osWaitForever);  // wait for message
    if (evt.status == osEventMessage)
    {
      if(evt.value.v == MSG_ENABLE_DISABLE)
      {
        if (SD_Log_Enabled) 
        {
          DATALOG_SD_Log_Disable();
          SD_Log_Enabled=0;
        }
        else
        {
          while(SD_Log_Enabled != 1)
          {
            if(DATALOG_SD_Log_Enable())
            {
              SD_Log_Enabled=1;
              osDelay(100);
              dataTimerStart();
            }
            else
            {
              DATALOG_SD_Log_Disable();
              DATALOG_SD_DeInit();
              DATALOG_SD_Init();
              osDelay(100);
            }
          }
        }
      }
      else
      {
      rptr = evt.value.p;
      
      if(LoggingInterface == USB_Datalog)
      {
        size = sprintf(data_s, "TimeStamp: %ld\r\n Acc_X: %d, Acc_Y: %d, Acc_Z :%d\r\n Gyro_X:%d, Gyro_Y:%d, Gyro_Z:%d\r\n Magn_X:%d, Magn_Y:%d, Magn_Z:%d\r\n Press:%5.2f, Temp:%5.2f, Hum:%4.1f\r\n",
                       rptr->ms_counter,
                       (int)rptr->acc.x, (int)rptr->acc.y, (int)rptr->acc.z,
                       (int)rptr->gyro.x, (int)rptr->gyro.y, (int)rptr->gyro.z,
                       (int)rptr->mag.x, (int)rptr->mag.y, (int)rptr->mag.z,
                       rptr->pressure, rptr->temperature, rptr->humidity);
        osPoolFree(sensorPool_id, rptr);      // free memory allocated for message
        BSP_LED_Toggle(LED1);
        CDC_Transmit_FS(( uint8_t * )data_s, size);
      }
        else
        {
          size = sprintf(data_s, "%ld, %d, %d, %d, %d, %d, %d, %d, %d, %d, %5.2f, %5.2f, %4.1f\r\n",
                       rptr->ms_counter,
                       (int)rptr->acc.x, (int)rptr->acc.y, (int)rptr->acc.z,
                       (int)rptr->gyro.x, (int)rptr->gyro.y, (int)rptr->gyro.z,
                       (int)rptr->mag.x, (int)rptr->mag.y, (int)rptr->mag.z,
					   rptr->pressure, rptr->temperature, rptr->humidity);
          osPoolFree(sensorPool_id, rptr);      // free memory allocated for message
          DATALOG_SD_writeBuf(data_s, size);
    }
  }
}
  }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    while(1);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
    _Error_Handler();
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
    _Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_SDMMC1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

void dataTimer_Callback(void const *arg)
{ 
  osSemaphoreRelease(readDataSem_id);
} 


void dataTimerStart(void)
{
  osStatus  status;
  
  // Create periodic timer
  exec = 1;
  sensorTimId = osTimerCreate(osTimer(SensorTimer), osTimerPeriodic, &exec);
  if (sensorTimId)  {
    status = osTimerStart (sensorTimId, DATA_PERIOD_MS);                // start timer
    if (status != osOK)  {
      // Timer could not be started
    } 
  }
}

void dataTimerStop(void)
{
  osTimerStop(sensorTimId);
  osTimerDelete(sensorTimId);
}


/**
* @brief  This function is executed in case of error occurrence
* @param  None
* @retval None
*/
static void _Error_Handler( void )
{
  while (1)
  {}
}


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
