/**
  ******************************************************************************
  * @file    sensor_service.c
  * @author  SRA
  * @version v1.4.0
  * @date    13-Nov-2020
  * @brief   Add bluetooth services using vendor specific profiles.
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
#include "sensor_service.h"
#include "console.h"
#include "bluenrg1_l2cap_aci.h"
#include "uuid_ble_service.h"

/* Exported variables ---------------------------------------------------------*/
int connected = FALSE;
uint8_t set_connectable = TRUE;

/* Imported Variables -------------------------------------------------------------*/
extern uint32_t ConnectionBleStatus;

extern TIM_HandleTypeDef    TimCCHandle;

extern uint8_t bdaddr[6];

/* Private variables ------------------------------------------------------------*/
static uint16_t HWServW2STHandle;
static uint16_t EnvironmentalCharHandle;
static uint16_t LedCharHandle;

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

static uint8_t LastStderrBuffer[HCI_MAX_PAYLOAD_SIZE];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[HCI_MAX_PAYLOAD_SIZE];
static uint8_t LastTermLen;

static uint8_t  EnvironmentalCharSize=2; /* Size for Environmental BLE characteristic */

static uint16_t connection_handle = 0;

static uint16_t MaxBLECharLen = 155;

/* Private functions ------------------------------------------------------------*/
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);
static void ConfigCommandParsing(uint8_t * att_data, uint8_t data_length);

/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;


/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
				      const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  
  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,(uint8_t *)charValue);
    
    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }
  
  return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */

  Service_UUID_t service_uuid;
  Char_UUID_t char_uuid;
/**
 * @brief  Add the Config service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConfigW2ST_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];
  
  COPY_CONFIG_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 1+3,&ConfigServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_CONFIG_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);  
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, &char_uuid, 20 /* Max Dimension */,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ConfigCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //STLBLE_PRINTF("Error while adding Configuration service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update Stderr characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages*/
  for(Offset =0; Offset<length; Offset += MaxBLECharLen ){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>MaxBLECharLen ) ?  MaxBLECharLen  : DataToSend;

    /* keep a copy */
    memcpy(LastStderrBuffer,data+Offset,DataToSend);
    LastStderrLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(10);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages */
  for(Offset =0; Offset<length; Offset += MaxBLECharLen ){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>MaxBLECharLen ) ?  MaxBLECharLen  : DataToSend;

    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
        STLBLE_PRINTF("Error Updating Stdout Char\r\n");
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(20);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Stderr characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, LastStderrLen , LastStderrBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, LastTermLen , LastTermBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Stdout Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}
/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data)
{
  tBleStatus ret;
  uint8_t buff[2+4+1+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7] = data;

  ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Configuration Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Configuration Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the HW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HWServW2ST_Service(void)
{
  tBleStatus ret;
  int32_t NumberChars = 4;

  uint8_t uuid[16];
  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE,
                          1+3*NumberChars,
                          &HWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  /* Fill the Environmental BLE Characteristc */
  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
    if(TargetBoardFeatures.NumTempSensors==2) {
      uuid[14] |= 0x05; /* Two Temperature values*/
      EnvironmentalCharSize+=2*2;
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      uuid[14] |= 0x04; /* One Temperature value*/
      EnvironmentalCharSize+=2;
    }

    if(TargetBoardFeatures.HandleHumSensor) {
     uuid[14] |= 0x08; /* Humidity */
     EnvironmentalCharSize+=2;
    }

    if(TargetBoardFeatures.HandlePressSensor) {
      uuid[14] |= 0x10; /* Pressure value*/
      EnvironmentalCharSize+=4;
    }
  
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16); 
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, EnvironmentalCharSize,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &EnvironmentalCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_LED_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16); 
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &LedCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //STLBLE_PRINTF("Error while adding HW's Characteristcs service.\n");
  return BLE_STATUS_ERROR;
}


/**
 * @brief  Update Environmental characteristic value
 * @param  int32_t Press Pressure in mbar
 * @param  uint16_t Hum humidity RH (Relative Humidity) in thenths of %
 * @param  int16_t Temp2 Temperature in tenths of degree second sensor
 * @param  int16_t Temp1 Temperature in tenths of degree first sensor
 * @retval tBleStatus   Status
 */
tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1)
{
  tBleStatus ret;
  uint8_t BuffPos;
  
  uint8_t buff[2+4/*Press*/+2/*Hum*/+2/*Temp2*/+2/*Temp1*/];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  BuffPos=2;

  
    if(TargetBoardFeatures.HandlePressSensor) {
      STORE_LE_32(buff+BuffPos,Press);
      BuffPos+=4;
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      STORE_LE_16(buff+BuffPos,Hum);
      BuffPos+=2;
    }

    if(TargetBoardFeatures.NumTempSensors==2) {
      STORE_LE_16(buff+BuffPos,Temp2);
      BuffPos+=2;

      STORE_LE_16(buff+BuffPos,Temp1);
      BuffPos+=2;
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      STORE_LE_16(buff+BuffPos,Temp1);
      BuffPos+=2;
    }

  ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, EnvironmentalCharSize,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Environmental Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Environmental Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update LEDs characteristic value
 * @param  uint8_t LedStatus LEDs status 0/1 (off/on)
 * @retval tBleStatus   Status
 */
tBleStatus LED_Update(uint8_t LedStatus)
{
  tBleStatus ret;

  uint8_t buff[2+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = LedStatus;

  ret = aci_gatt_update_char_value(HWServW2STHandle, LedCharHandle, 0, 2+1,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating LED Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Temp Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Puts the device in connectable mode.
 * @param  None 
 * @retval None
 */
void setConnectable(void)
{  
  char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NAME_STLBLE};
  uint8_t manuf_data[28] = {
    2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
    8,0x09,NAME_STLBLE, // Complete Name
    15,0xFF,
    0x30,0x00, /* STM Manufacter AD */
    0x01/*SDK version */,
    0x08, /* STEVAL-STWINKIT1 */
    0x00, /* No Sensor 1 */
    0x00, /* No Sensor 2 */
    0x00, /*  */
    0x00, /*  */
    0x00, /* BLE MAC start */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
  };


  /* BLE MAC */
  manuf_data[22] = bdaddr[5];
  manuf_data[23] = bdaddr[4];
  manuf_data[24] = bdaddr[3];
  manuf_data[25] = bdaddr[2];
  manuf_data[26] = bdaddr[1];
  manuf_data[27] = bdaddr[0];

  manuf_data[18] |= 0x20; /* Led */

  if(TargetBoardFeatures.HandleGGComponent){
    manuf_data[19] |= 0x02; /* Battery Present */
  }

    if(TargetBoardFeatures.NumTempSensors==2) {
      manuf_data[19] |= 0x05; /* Two Temperature values*/
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      manuf_data[19] |= 0x04; /* One Temperature value*/
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      manuf_data[19] |= 0x08; /* Humidity */
    }

    if(TargetBoardFeatures.HandlePressSensor) {
      manuf_data[19] |= 0x10; /* Pressure value*/
    }

    /* DS3 DIL24  present*/
    if(TargetBoardFeatures.HWAdvanceFeatures) {
      /* Accelerometer Events */
      manuf_data[20] |=0x04;
    }


  /* disable scan response */
  hci_le_set_scan_response_data(0,NULL);
  aci_gap_set_discoverable(ADV_IND, 0, 0,
#ifndef STATIC_BLE_MAC
                           STATIC_RANDOM_ADDR,
#else /* STATIC_BLE_MAC */
                           PUBLIC_ADDR,
#endif /* STATIC_BLE_MAC */
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), (uint8_t*)local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
  aci_gap_update_adv_data(28, manuf_data);
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;

#ifdef ENABLE_USB_DEBUG_CONNECTION
  STLBLE_PRINTF(">>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
#endif /* ENABLE_USB_DEBUG_CONNECTION */

  ConnectionBleStatus=0;
  
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;

#ifdef ENABLE_USB_DEBUG_CONNECTION  
  STLBLE_PRINTF("<<<<<<DISCONNECTED\r\n");
#endif /* ENABLE_USB_DEBUG_CONNECTION */  

  /* Make the device connectable again. */
  set_connectable = TRUE;

  ConnectionBleStatus=0;

  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
}

/**
 * @brief  This function is called when there is a Bluetooth Read request
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
  if(handle == EnvironmentalCharHandle + 1)
  {
    /* Read Request for Pressure,Humidity, and Temperatures*/
    float SensorValue;
    int32_t PressToSend=0;
    uint16_t HumToSend=0;
    int16_t Temp2ToSend=0,Temp1ToSend=0;
    int32_t decPart, intPart;
    
    if(TargetBoardFeatures.HandlePressSensor) 
    {
      BSP_ENV_SENSOR_GetValue(LPS22HH_0, ENV_PRESSURE,(float *)&SensorValue);
      MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
      PressToSend=intPart*100+decPart;
    }
    
    if(TargetBoardFeatures.HandleHumSensor)
    {
      BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_HUMIDITY, (float *)&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      HumToSend = intPart*10+decPart;
    }
    
    if(TargetBoardFeatures.NumTempSensors==2) 
    {
      BSP_ENV_SENSOR_GetValue(STTS751_0, ENV_TEMPERATURE,(float *)&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      Temp1ToSend = intPart*10+decPart; 
      
      BSP_ENV_SENSOR_GetValue(LPS22HH_0, ENV_TEMPERATURE,(float *)&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      Temp2ToSend = intPart*10+decPart;
    } 
    else if(TargetBoardFeatures.NumTempSensors==1)
    {
      if (BSP_ENV_SENSOR_GetValue(STTS751_0, ENV_TEMPERATURE,(float *)&SensorValue)!=BSP_ERROR_NONE)
      {
        BSP_ENV_SENSOR_GetValue(LPS22HH_0, ENV_TEMPERATURE,(float *)&SensorValue);
      }
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      Temp1ToSend = intPart*10+decPart;
    }
    
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  } 
  else if(handle == LedCharHandle + 1)
  {
    /* Read Request for Led Status */
    LED_Update(TargetBoardFeatures.LedStatus);
  }
  else if (handle == StdErrCharHandle + 1) 
  {
    /* Send again the last packet for StdError */
    Stderr_Update_AfterRead();
  } 
  else if (handle == TermCharHandle + 1)
  {
    /* Send again the last packet for Terminal */
    Term_Update_AfterRead();
  }
  
  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application 
 * is subscribed or not to the one service
 * @param uint16_t att_handle Handle of the attribute
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{
  if(attr_handle == ConfigCharHandle + 2) {
    ;/* do nothing... only for removing the message "Notification UNKNOW handle" */
  }
  else if(attr_handle == EnvironmentalCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ENV);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }

      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
      }
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ENV);

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
    }
#ifdef ENABLE_USB_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Env=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      STLBLE_PRINTF("--->Env=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? "ON" : "OFF");
#endif /* ENABLE_USB_DEBUG_CONNECTION */
  }
  else if(attr_handle == StdErrCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_ERR);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_ERR);
    }
  } else if(attr_handle == TermCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_TERM);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_TERM);
    }
  } 
  else if(attr_handle == LedCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_LED);
      /* Update the LED feature */
      LED_Update(TargetBoardFeatures.LedStatus);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_LED);
    }
#ifdef ENABLE_USB_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Led=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED) ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      STLBLE_PRINTF("--->Led=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED) ? "ON" : "OFF");
#endif /* ENABLE_USB_DEBUG_CONNECTION */
  } else if (attr_handle == ConfigCharHandle + 1) {
    /* Received one write command from Client on Configuration characteristc */
    ConfigCommandParsing(att_data, data_length);    
  } else {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Notification UNKNOW handle\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Notification UNKNOW handle\r\n");
    }
  }
}

/**
 * @brief  This function makes the parsing of the Configuration Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
static void ConfigCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t FeatureMask = (att_data[3]) | (att_data[2]<<8) | (att_data[1]<<16) | (att_data[0]<<24);
  uint8_t Command = att_data[4];
  uint8_t Data    = att_data[5];

  switch (FeatureMask) {
    case FEATURE_MASK_LED:
      /* Led events */
#ifdef ENABLE_USB_DEBUG_CONNECTION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
        BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%2x\n\r",FeatureMask,Command);
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        STLBLE_PRINTF("Conf Sig F=%lx C=%2x\r\n",FeatureMask,Command);
      }
#endif /* ENABLE_USB_DEBUG_CONNECTION */
     switch(Command) {
      case 1:
        TargetBoardFeatures.LedStatus=1;
        LedOnTargetPlatform();
        Config_Notify(FEATURE_MASK_LED,Command,Data);
        break;
      case 0:
        TargetBoardFeatures.LedStatus=0;
        LedOffTargetPlatform();
        Config_Notify(FEATURE_MASK_LED,Command,Data);
        break;
     }
     /* Update the LED feature */
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED)) {
       LED_Update(TargetBoardFeatures.LedStatus);
     }
    break;
  }
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{   
  /* Start one Exchange configuration for understaning the maxium ATT_MTU */
  aci_gatt_exchange_config(Connection_Handle);
  
  GAP_ConnectionComplete_CB(Peer_Address, Connection_Handle);     
}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  GAP_DisconnectionComplete_CB();
}/* end hci_disconnection_complete_event() */


/*******************************************************************************
 * Function Name  : aci_gatt_read_permit_req_event.
 * Description    : This event is given when a read request is received
 *                  by the server from the client.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
  Read_Request_CB(Attribute_Handle);
}


void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  Attribute_Modified_CB(Attr_Handle, Attr_Data, Attr_Data_Length); 
}


void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle, uint16_t Server_RX_MTU)
{
  if((Server_RX_MTU-3)<MaxBLECharLen) {
    MaxBLECharLen = Server_RX_MTU-3;
  }
  STLBLE_PRINTF("aci_att_exchange_mtu_resp_event Server_RX_MTU=%d\r\n",Server_RX_MTU);
}


void aci_gatt_indication_event(uint16_t Connection_Handle,
                               uint16_t Attribute_Handle,
                               uint8_t Attribute_Value_Length,
                               uint8_t Attribute_Value[])
{
  tBleStatus RetStatus; 
  
  /* This callback should be called when we connect the .box also to something
  * that could work also like server mode.
  * In our case we don't need to do nothing when we receive this indication,
  * except it's confirmation
  */ 
  
  STLBLE_PRINTF("aci_gatt_indication_event:\r\n");
  STLBLE_PRINTF("\tConnection_Handle=0x%x\r\n",Connection_Handle);
  STLBLE_PRINTF("\tAttribute_Handle=0x%x\r\n",Attribute_Handle);
  
  if(Attribute_Value_Length==4) {
    /* Should be the range of Handles */
    uint16_t StartHandle = (((uint16_t) Attribute_Value[1])<<8) | Attribute_Value[0];
    uint16_t StopHandle  = (((uint16_t) Attribute_Value[3])<<8) | Attribute_Value[2];
    STLBLE_PRINTF("\tFrom Handles =0x%x to 0x%x\r\n",StartHandle,StopHandle);
  } 
  
  STLBLE_PRINTF("Nothing to do except send confirmation\r\n"); 
  
  RetStatus = aci_gatt_confirm_indication(Connection_Handle);
  if (RetStatus != BLE_STATUS_SUCCESS) {
    STLBLE_PRINTF("aci_gatt_confirm_indicationt failed %d\r\n",RetStatus);
  } else {
    STLBLE_PRINTF("aci_gatt_confirm_indication Done\r\n");
  }
}


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
