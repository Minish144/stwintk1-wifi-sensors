/**
  ******************************************************************************
  * @file    entropy_hardware_poll.c
  * @author  SRA
  * @version v1.4.0
  * @date    13-Nov-2020
  * @brief   This file provides code for the entropy collector.
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

#include "main.h"
#ifdef  NET_MBEDTLS_HOST_SUPPORT

extern RNG_HandleTypeDef hrng;


int mbedtls_rng_poll_cb( void *data,
                    unsigned char *output, size_t len, size_t *olen );


int mbedtls_rng_poll_cb( void *data,
                    unsigned char *output, size_t len, size_t *olen )
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t random_number = 0;
  
  status = HAL_RNG_GenerateRandomNumber(&hrng, &random_number);
  ((void) data);
  *olen = 0;
  
  if ((len < sizeof(uint32_t)) || (HAL_OK != status))
  {
    return 0;
  }
  
  memcpy(output, &random_number, sizeof(uint32_t));
  *olen = sizeof(uint32_t);
  
  return 0;
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
