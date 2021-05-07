/**
  ******************************************************************************
  * @file    main.c
  * @author  SRA
  * @version v1.4.0
  * @date    13-Nov-2020
  * @brief   This file provides main program functions
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
#include <stdio.h>
#include <stdint.h>

#ifndef SENSORS_IMPORT
#define SENSORS_IMPORT

#include "STWIN_env_sensors.h"
#include "STWIN_env_sensors.c"

#endif

/* Global variables ---------------------------------------------------------*/
#define STATE_TRANSITION_TIMEOUT        10000

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

#define SSID        "OLGA"
#define PASSWORD    "9651810010"

void SPI_WIFI_ISR(void);

static void RTC_Init(void);
static void Console_Init(void);
void SystemClock_Config(void);
static void hnet_notify (void *context, uint32_t event_class,uint32_t event_id, void  *event_data);

void    TestClient(void);
void    TestServer(void);
void    TestTLSServerConnection(bool checkserveidentity);
void    TestPing(void);

RTC_HandleTypeDef hrtc;
RNG_HandleTypeDef hrng;

int32_t es_wifi_driver(net_if_handle_t * pnetif);

net_if_handle_t netif;
const   net_event_handler_t  net_handler   = { hnet_notify, &netif };



/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

int main(void)
{
  unsigned int random_number = 0;
  float temp = 0;
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  SystemClock_Config();

  Console_Init();

  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }

  RTC_Init();

  if (HAL_RNG_GenerateRandomNumber(&hrng, (uint32_t *) &random_number) == HAL_OK)
  {
    srand(random_number);
  }


  /* HTS221 temperature sensor init */
  if(BSP_ENV_SENSOR_Init(HTS221_0, ENV_TEMPERATURE)==BSP_ERROR_NONE)
  {
	  BSP_ENV_SENSOR_SetOutputDataRate(HTS221_0, ENV_TEMPERATURE, 12.5f);
      WIFI_PRINTF("HTS221 Initialized\n\r");
  }

//  BSP_ENV_SENSOR_Get_Temperature_Limit_Status(HTS221_0, &temp, &temp, &temp);

  /* Network */
  if(net_if_init (&netif, &es_wifi_driver, &net_handler) == NET_OK )
  {
    net_if_wait_state(&netif,NET_STATE_INITIALIZED,STATE_TRANSITION_TIMEOUT);
    if ( net_if_start (&netif) == NET_OK )
    {
      net_wifi_credentials_t Credentials =
      {
        SSID,
        PASSWORD,
        NET_WIFI_SM_WPA2_MIXED_PSK
      };

      if (NET_OK != net_wifi_set_credentials(&netif, &Credentials))
      {
        WIFI_PRINTF("Can not set wifi credentials\n");
        return -1;
      }

      if(net_if_connect (&netif) == NET_OK)
      {
		  while(1)
		  {
			  /* HTTP REQUEST */
			  TestClient();
		      BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_TEMPERATURE, (float *)&temp);
		      WIFI_PRINTF("Temp: %f", temp);
			  HAL_Delay(1000); // 1s delay
		  }
      }
    }
  }

  while(1)
  {
      BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_TEMPERATURE, (float *)&temp);
	  HAL_Delay(1000); // 1s delay
  }
}


/**
  * @brief RTC init function
  */

static void RTC_Init(void)
{
  RTC_TimeTypeDef xsTime;
  RTC_DateTypeDef xsDate;

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  /* 32.768kHz LSE clock input */
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
#ifdef RTC_OUTPUT_REMAP_NONE
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
#endif
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize RTC and set the Time and Date. */
  xsTime.Hours = 0x12;
  xsTime.Minutes = 0x0;
  xsTime.Seconds = 0x0;
  xsTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  xsTime.StoreOperation = RTC_STOREOPERATION_RESET;

  if( HAL_RTC_SetTime( &hrtc, &xsTime, RTC_FORMAT_BCD ) != HAL_OK )
  {
    Error_Handler();
  }

  xsDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  xsDate.Month = RTC_MONTH_MAY;
  xsDate.Date = 0x29;
  xsDate.Year = 0x19;

  if( HAL_RTC_SetDate( &hrtc, &xsDate, RTC_FORMAT_BCD ) != HAL_OK )
  {
    Error_Handler();
  }
}

/**
  * @brief UART console init function
  */
static void Console_Init(void)
{
  HAL_PWREx_EnableVddIO2();
  HAL_PWREx_EnableVddUSB();
  BSP_Enable_DCDC2();

  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
  USBD_Start(&hUsbDeviceFS);
  HAL_Delay(3000);
}



uint32_t sys_now(void)
{
  return HAL_GetTick();
}

#if (defined(__GNUC__) && !defined(__CC_ARM))
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  return ch;
}

/**
  * @brief  Retargets the C library scanf function to the USART.
  * @param  None
  * @retval None
  */
GETCHAR_PROTOTYPE
{
  /* Place your implementation of fgetc here */
  uint8_t ch = 0;

  ch =  USB_getchar();
  return ch;
}

static void hnet_notify (void *context, uint32_t event_class,uint32_t event_id, void  *event_data)
{
  net_if_handle_t *netif=context;

  if (NET_EVENT_STATE_CHANGE == event_class)
  {
    net_state_t new_state= (net_state_t) event_id;
    switch(new_state)
    {
      case NET_STATE_INITIALIZED:
        WIFI_PRINTF("- Network Interface initialized:\n");
        break;

      case NET_STATE_STARTING:
        WIFI_PRINTF("- Network Interface starting:\n");
        break;

      case NET_STATE_READY:
        WIFI_PRINTF("- Network Interface started:\n");
        WIFI_PRINTF("   - Device Name : %s\n", netif->DeviceName );
        WIFI_PRINTF("   - Device ID   : %s\n", netif->DeviceID );
        WIFI_PRINTF("   - Device Version : %s\n", netif->DeviceVer );
        break;

      case NET_STATE_CONNECTING:
        WIFI_PRINTF("- Network Interface connecting:\n");
        break;

      case NET_STATE_CONNECTED:
        WIFI_PRINTF("- Network Interface connected:\n");
        WIFI_PRINTF("   - IP address :  %s\n", net_ntoa(&netif->ipaddr));
        break;

      case NET_STATE_DISCONNECTING:
        WIFI_PRINTF("- Network Interface disconnecting\n");
        break;

      case NET_STATE_STOPPING:
        WIFI_PRINTF("- Network Interface stopping\n");
        break;

      case NET_STATE_DEINITIALIZED:
        WIFI_PRINTF("- Network Interface de-initialized\n");
        break;

      default:
        break;
    }
  }
}


/**
* @brief  System Clock Configuration
* @param  None
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE
    |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC
    |RCC_PERIPHCLK_DFSDM1
      |RCC_PERIPHCLK_USB
        |RCC_PERIPHCLK_ADC
          |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 5;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 96;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV25;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    __HAL_RCC_PWR_CLK_ENABLE();

}


/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == WIFI_DATA_READY_PIN)
  {
    SPI_WIFI_ISR();
  }
}


void Error_Handler(void)
{
  while(1)
  {
    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(200);
  }
}

#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

#ifdef configUSE_MALLOC_FAILED_HOOK
void vApplicationMallocFailedHook( void );

void vApplicationMallocFailedHook( void )
{
  NET_ASSERT(0,"Failed:Run out of memory");
}
#endif

#ifdef configCHECK_FOR_STACK_OVERFLOW

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName );
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
  NET_ASSERT(0,"Failed:Run out of stack ");
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

