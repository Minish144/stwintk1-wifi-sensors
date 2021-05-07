/**
  ******************************************************************************
  * @file    net_conf.h
  * @author  SRA
  * @version v1.4.0
  * @date    13-Nov-2020
  * @brief   This file provides the configuration for net
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

#ifndef NET_CONF_H
#define NET_CONF_H

#ifdef __cplusplus
extern "C" {
#endif
  
  
#include <stdio.h>
#include <stdint.h>
#include "STWIN_wifi.h"

// Please comment if MBEDTLS on host side is not used
#define NET_MBEDTLS_HOST_SUPPORT

//Please uncomment if device supports Secure TCP connection
//#define NET_MBEDTLS_DEVICE_SUPPORT

#ifdef NET_USE_RTOS
#include "cmsis_os.h"
#define net_malloc pvPortMalloc
#define net_free   vPortFree
#else
#define net_malloc malloc
#define net_calloc calloc
#define net_free  free
#endif


/* MbedTLS configuration */
#ifdef NET_MBEDTLS_HOST_SUPPORT


#if !defined NET_MBEDTLS_DEBUG_LEVEL
#define NET_MBEDTLS_DEBUG_LEVEL 1
#endif /*   NET_MBEDTLS_DEBUG_LEVEL */

#if !defined NET_MBEDTLS_CONNECT_TIMEOUT
#define NET_MBEDTLS_CONNECT_TIMEOUT     10000U
#endif /* NET_MBEDTLS_CONNECT_TIMEOUT */

#if !defined(MBEDTLS_CONFIG_FILE)
#define MBEDTLS_CONFIG_FILE "mbedtls/config.h"
#endif /* MBEDTLS_CONFIG_FILE */
#endif /* NET_MBEDTLS_HOST_SUPPORT */

#if !defined(NET_MAX_SOCKETS_NBR)
#define NET_MAX_SOCKETS_NBR            5
#endif /* NET_MAX_SOCKETS_NBR */

#define NET_IF_NAME_LEN                128
#define NET_DEVICE_NAME_LEN            64
#define NET_DEVICE_ID_LEN              64
#define NET_DEVICE_VER_LEN             64


#define NET_SOCK_DEFAULT_RECEIVE_TO    60000
#define NET_SOCK_DEFAULT_SEND_TO       60000
#define NET_UDP_MAX_SEND_BLOCK_TO      1024

#if !defined(NET_USE_DEFAULT_INTERFACE)
#define NET_USE_DEFAULT_INTERFACE      1
#endif /* NET_USE_DEFAULT_INTERFACE */

#ifdef  ENABLE_NET_DBG_INFO
#define NET_DBG_INFO(...)  do { \
                                printf(__VA_ARGS__); \
                              } while (0)
#else
#define NET_DBG_INFO(...)
#endif

#if !defined(NET_DBG_ERROR)
#define NET_DBG_ERROR(...)  do { \
                                 printf("\nERROR: %s:%d ",__FILE__,__LINE__) ;\
                                 printf(__VA_ARGS__);\
                                 printf("\n"); \
                               } while (0)
#endif /* NET_DBG_ERROR */

#if !defined(NET_DBG_PRINT)  
#define NET_DBG_PRINT(...)  do { \
                                 printf("%s:%d ",__FILE__,__LINE__) ;\
                                 printf(__VA_ARGS__);\
                                 printf("\n"); \
                               } while (0)
#endif /* NET_DBG_PRINT */

#if !defined(NET_ASSERT)                                  
#define NET_ASSERT(test,s)  do { if (!(test)) {\
                                 printf("Assert Failed %s %d : %s\n",__FILE__,__LINE__,s); \
                                   while(1); }\
                               } while (0)
#endif /* NET_ASSERT */

#if !defined(NET_PRINT)                                 
#define NET_PRINT(...)  do { \
                                 printf(__VA_ARGS__);\
                                 printf("\n"); \
                               } while (0)
#endif /* NET_PRINT */

#if !defined(NET_PRINT_WO_CR)
#define NET_PRINT_WO_CR(...)   do { \
                                 printf(__VA_ARGS__);\
                               } while (0)
#endif /* NET_PRINT_WO_CR */

#if !defined(NET_WARNING)
#define NET_WARNING(...)  do { \
                                 printf("Warning %s:%d ",__FILE__,__LINE__) ;\
                                 printf(__VA_ARGS__);\
                                 printf("\n"); \
                               } while (0)
#endif /* NET_WARNING */



                                
#ifndef NET_PERF_MAXTHREAD
#define NET_PERF_MAXTHREAD      10U
#endif /* NET_PERF_MAXTHREAD  */

                                 
#ifdef __cplusplus
}
#endif

#endif /* NET_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
