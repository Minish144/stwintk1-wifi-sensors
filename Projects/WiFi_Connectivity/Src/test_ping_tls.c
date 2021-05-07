/**
  ******************************************************************************
  * @file    test_ping_tls.c
  * @author  SRA
  * @version v1.4.0
  * @date    13-Nov-2020
  * @brief   Implements ping and TLS test examples
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
#include <string.h>
#include <stdio.h>
#include "main.h"


int TestTLSServerConnection(bool checkserveidentity);
int TestPing(void);
int     setting_rtc(char * dow,int day,char *month, int year, int hour, int min, int sec);


char tls_root_ca_cert[]=
"-----BEGIN CERTIFICATE-----\n"
"MIIF3jCCA8agAwIBAgIQAf1tMPyjylGoG7xkDjUDLTANBgkqhkiG9w0BAQwFADCB"
"iDELMAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0pl"
"cnNleSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNV"
"BAMTJVVTRVJUcnVzdCBSU0EgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTAw"
"MjAxMDAwMDAwWhcNMzgwMTE4MjM1OTU5WjCBiDELMAkGA1UEBhMCVVMxEzARBgNV"
"BAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNleSBDaXR5MR4wHAYDVQQKExVU"
"aGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMTJVVTRVJUcnVzdCBSU0EgQ2Vy"
"dGlmaWNhdGlvbiBBdXRob3JpdHkwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAwggIK"
"AoICAQCAEmUXNg7D2wiz0KxXDXbtzSfTTK1Qg2HiqiBNCS1kCdzOiZ/MPans9s/B"
"3PHTsdZ7NygRK0faOca8Ohm0X6a9fZ2jY0K2dvKpOyuR+OJv0OwWIJAJPuLodMkY"
"tJHUYmTbf6MG8YgYapAiPLz+E/CHFHv25B+O1ORRxhFnRghRy4YUVD+8M/5+bJz/"
"Fp0YvVGONaanZshyZ9shZrHUm3gDwFA66Mzw3LyeTP6vBZY1H1dat//O+T23LLb2"
"VN3I5xI6Ta5MirdcmrS3ID3KfyI0rn47aGYBROcBTkZTmzNg95S+UzeQc0PzMsNT"
"79uq/nROacdrjGCT3sTHDN/hMq7MkztReJVni+49Vv4M0GkPGw/zJSZrM233bkf6"
"c0Plfg6lZrEpfDKEY1WJxA3Bk1QwGROs0303p+tdOmw1XNtB1xLaqUkL39iAigmT"
"Yo61Zs8liM2EuLE/pDkP2QKe6xJMlXzzawWpXhaDzLhn4ugTncxbgtNMs+1b/97l"
"c6wjOy0AvzVVdAlJ2ElYGn+SNuZRkg7zJn0cTRe8yexDJtC/QV9AqURE9JnnV4ee"
"UB9XVKg+/XRjL7FQZQnmWEIuQxpMtPAlR1n6BB6T1CZGSlCBst6+eLf8ZxXhyVeE"
"Hg9j1uliutZfVS7qXMYoCAQlObgOK6nyTJccBz8NUvXt7y+CDwIDAQABo0IwQDAd"
"BgNVHQ4EFgQUU3m/WqorSs9UgOHYm8Cd8rIDZsswDgYDVR0PAQH/BAQDAgEGMA8G"
"A1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQEMBQADggIBAFzUfA3P9wF9QZllDHPF"
"Up/L+M+ZBn8b2kMVn54CVVeWFPFSPCeHlCjtHzoBN6J2/FNQwISbxmtOuowhT6KO"
"VWKR82kV2LyI48SqC/3vqOlLVSoGIG1VeCkZ7l8wXEskEVX/JJpuXior7gtNn3/3"
"ATiUFJVDBwn7YKnuHKsSjKCaXqeYalltiz8I+8jRRa8YFWSQEg9zKC7F4iRO/Fjs"
"8PRF/iKz6y+O0tlFYQXBl2+odnKPi4w2r78NBc5xjeambx9spnFixdjQg3IM8WcR"
"iQycE0xyNN+81XHfqnHd4blsjDwSXWXavVcStkNr/+XeTWYRUc+ZruwXtuhxkYze"
"Sf7dNXGiFSeUHM9h4ya7b6NnJSFd5t0dCy5oGzuCr+yDZ4XUmFF0sbmZgIn/f3gZ"
"XHlKYC6SQK5MNyosycdiyA5d9zZbyuAlJQG03RoHnHcAP9Dc1ew91Pq7P8yF1m9/"
"qS3fuQL39ZeatTXaw2ewh0qpKJ4jjv9cJ2vhsE/zB+4ALtRZh8tSQZXq9EfX7mRB"
"VXyNWQKV3WKdwrnuWih0hKWbt5DHDAff9Yk2dDLWKMGwsAvgnEzDHNb842m1R0aB"
"L6KCq9NjRHDEjf8tM7qtj3u1cIiuPhnPQCjY/MiQu12ZIvVS5ljFH4gxQ+6IHdfG"
"jjxDah2nGN59PRbxYvnKkKj9"
"-----END CERTIFICATE-----\n";

#define TIME_SOURCE_HTTP_HOST   "www.gandi.net"
#define TIME_SOURCE_HTTP_PORT   443
#define NET_READ_TIMEOUT        5000

#define NET_BUF_SIZE  1000

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char http_request[] = "HEAD / HTTP/1.1\r\nHost: "TIME_SOURCE_HTTP_HOST"\r\n\r\n";

/* Functions Definition ------------------------------------------------------*/
#define PING_ITERATION  10
#define PING_DELAY      100

int TestPing(void)
{
  int   ping_res[PING_ITERATION];
  sockaddr_in_t addr;

  addr.sin_len = sizeof(sockaddr_in_t);
  if (net_if_gethostbyname(NULL,(sockaddr_t *)&addr,TIME_SOURCE_HTTP_HOST) < 0)
  {
    WIFI_PRINTF("Could not find hostname ipaddr %s\n",TIME_SOURCE_HTTP_HOST);
    return -1;
 }

  if (net_if_ping(NULL,(sockaddr_t *)&addr,PING_ITERATION,PING_DELAY,ping_res) >= 0)
  {
    for(int i=0;i<PING_ITERATION ;i++)
    {
      if (ping_res[i]>=0) WIFI_PRINTF("Ping iteration #%d roundtrip %d\n",i,ping_res[i]);
    }
  }
  return 0;
}


/**
 * @brief Connect to a server using TLS protocol and check HTTP header.
 * @param In:  None
 * @note  Pre-conditions:
 *   . Wifi network connected
 *   . One free socket
 * @retval  Error code
 *              0        OK
 *              -1       FAILURE
 *                                .
 */
int TestTLSServerConnection(bool check_server_identity)
{
  int ret;
  sockaddr_in_t addr;
  int sock;
  int timeout=NET_READ_TIMEOUT;
  char buffer[NET_BUF_SIZE + 1]; /* +1 to be sure that the buffer is closed by a \0, so that it may be parsed by string commands. */

  addr.sin_len = sizeof(sockaddr_in_t);
  if (net_if_gethostbyname(NULL,(sockaddr_t *)&addr,TIME_SOURCE_HTTP_HOST) < 0)
  {
    WIFI_PRINTF("Could not find hostname ipaddr %s\n",TIME_SOURCE_HTTP_HOST);
    return -1;
  }

//  WIFI_PRINTF("Connecting to %s at  ipaddress: %s\n",TIME_SOURCE_HTTP_HOST,net_ntoa_r(&addr.sin_addr,buffer,NET_BUF_SIZE));
  memset(buffer, 0, sizeof(buffer));

  sock = net_socket (NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);
  if (sock < 0 )
  {
    WIFI_PRINTF("Could not create the socket.\n");
    return -1;
  }
  ret  = net_setsockopt(sock, NET_SOL_SOCKET, NET_SO_SECURE, NULL, 0);
  ret |= net_setsockopt(sock, NET_SOL_SOCKET, NET_SO_RCVTIMEO, &timeout, sizeof(uint32_t));
  ret |= net_setsockopt(sock, NET_SOL_SOCKET, NET_SO_SNDTIMEO, &timeout, sizeof(uint32_t));
  ret |= net_setsockopt(sock, NET_SOL_SOCKET, NET_SO_TLS_CA_CERT, tls_root_ca_cert, strlen(tls_root_ca_cert)+1);
  ret |= net_setsockopt(sock, NET_SOL_SOCKET, NET_SO_TLS_SERVER_NAME, TIME_SOURCE_HTTP_HOST, strlen(TIME_SOURCE_HTTP_HOST)+1);
  
  if (check_server_identity==true)
  {
    WIFI_PRINTF("Check TLS server connection before getting time \n");
  }
    
  ret |= net_setsockopt(sock, NET_SOL_SOCKET, NET_SO_TLS_SERVER_VERIFICATION,&check_server_identity, sizeof(bool));

  if (ret != NET_OK)
  {
    WIFI_PRINTF("Could not set the socket options.\n");
    return -1;
  }

  addr.sin_port = NET_HTONS(TIME_SOURCE_HTTP_PORT);
  ret = net_connect( sock,  (sockaddr_t *)&addr, sizeof( addr ) );
  if (ret == NET_ERROR_MBEDTLS_REMOTE_AUTH)
  {
    WIFI_PRINTF("An incorrect system time may have resulted in a TLS authentication error.\n");
    return -1;
  }

  if (ret != NET_OK)
  {
    WIFI_PRINTF("Could not open the socket.\n");
    return -1;
  }

  ret = net_send(sock, (uint8_t *) http_request,strlen(http_request),0);
  if (ret != strlen(http_request))
  {
      WIFI_PRINTF("Could not send %d bytes.\n",  strlen(http_request));
      return -1;
  }

  char *dateStr = NULL;
  int read = 0;
  int len;
  do
  {
    len = net_recv(sock, (uint8_t *) buffer + read, NET_BUF_SIZE - read,0);
    if (len > 0)
    {
      read += len;
      dateStr = strstr(buffer, "Date: ");
    }
  } while ( (dateStr == NULL) && ((len >= 0) || (len == NET_TIMEOUT)) && (read < NET_BUF_SIZE));

  if (dateStr == NULL)
  {
    if (len > 0) 
    {
      WIFI_PRINTF("No 'Date:' line found in the HTTP response header.\n");
    }
    else 
    {
      WIFI_PRINTF("No Data read at all or error in the communication : %d\n",len);
    }
    net_closesocket(sock);
    return -1;
  }

  char prefix[8], dow[8], month[4];
  int day, year, hour, min, sec;

  memset(dow, 0, sizeof(dow));
  memset(month, 0, sizeof(month));
  day = year = hour = min = sec = 0;

  int count = sscanf(dateStr, "%s %s %d %s %d %02d:%02d:%02d ", prefix, dow, &day, month, &year, &hour, &min, &sec);
  if (count < 8)
  {
    WIFI_PRINTF("At time initialization, only %d out of the 8 time/date data could be parsed from the HTTP response %s\n", count, buffer);
    net_closesocket(sock);
    return -1;
  }
  else
  {
    WIFI_PRINTF("Success get time from server %s : %d %s %d: %02d:%02d:%02d\n",TIME_SOURCE_HTTP_HOST,day, month, year, hour, min, sec);
    setting_rtc(dow,day,month,year,hour,min,sec);

  }


  net_closesocket(sock);
  return 0;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
