  /**
  *******************************************************************************
  * @file    test_client_server.c
  * @author  SRA
  * @version v1.4.0
  * @date    13-Nov-2020
  * @brief   Implement client and server testing examples
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
#include "time.h"
void TestServer(void);
void TestClient(void);

static void SendWebPage(int sock);

//#define CHECK
static void fillbuffer(uint8_t  *buff,int n);
#ifdef CHECK
static int checkbuffer(uint8_t *buff,int n,int offset);
#endif

extern net_if_handle_t netif;


#define REMOTE_IP_ADDR "192.168.1.193"
#define REMOTE_PORT     65432
#define LOCAL_PORT      8001




#define LOOP    20
#define N       2048

//#define CHECK

static  uint8_t  buffer[N+LOOP];
static  uint8_t  buffin[N+LOOP];

/**
  * @brief  network_app
  * @param  None
  * @retval None
  */


static void fillbuffer(uint8_t  *buff,int n)
{
  int i;
  for(i=0;i<n;i++) buff[i]=i;
}
#ifdef CHECK
static int checkbuffer(uint8_t *buff,int n,int offset)
{
  int i;
  for(i=0;i<n;i++)
  {
    if (buff[i] != ((i+offset) & 0xff))
    {
      WIFI_PRINTF("index %d\n",i);
      return 0;
    }
  }
  return 1;
}
#endif

void TestClient(void)
{
  int sock;
  sockaddr_in_t addr;
  int timeout=10000;
  char sendline[N];
  char jsonbody[N];
  char apikey[] = "Bearer 46dcfch7981236tdf98dc7asd6ftg9ef8o0asgfa6s8ofas76fsf";

//  char jsonbody[] = "{\"phone\":12345,\"code\":1000}";
//
//  sprintf(
//      sendline,
//      "GET %s HTTP/1.0\r\nHost: %s\r\nContent-type: application/json\r\nContent-length: %d\r\n\r\n%s\r\n",
//	  "/", "192.168.1.193:65432", strlen(jsonbody), jsonbody
//  );

  sprintf(jsonbody, "{\"ts\":%d,\"data\":{\"temperature\":%f}}", (int)time(NULL), 26.35);

  sprintf(
      sendline,
      "POST %s HTTP/1.0\r\nHost: %s\r\nAuthorization: %s\r\nContent-type: application/json\r\nContent-length: %d\r\n\r\n%s\r\n",
	  "/", "192.168.1.193:65432", apikey, strlen(jsonbody), jsonbody
  );

  sock = net_socket (NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);
  addr.sin_port        = NET_HTONS(REMOTE_PORT);
  addr.sin_family      = NET_AF_INET;
  S_ADDR(addr.sin_addr) = net_aton_r (REMOTE_IP_ADDR);

  net_setsockopt(sock, NET_SOL_SOCKET, NET_SO_RCVTIMEO, &timeout, sizeof(int32_t*));
  net_setsockopt(sock, NET_SOL_SOCKET, NET_SO_SNDTIMEO, &timeout, sizeof(int32_t*));

  if( net_connect( sock,  (sockaddr_t *)&addr, sizeof( addr ) ) == 0 )
  {
        net_send( sock, sendline, strlen(sendline), 0 );
  }
  net_closesocket(sock);
}

void TestClient_Old(void)
{
  int   i;
  int   tstart,tstop;
  int transfer=0;
  int sock;
  sockaddr_in_t addr;
  int timeout=10000;

  sock = net_socket (NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);
  addr.sin_port        = NET_HTONS(REMOTE_PORT);
  addr.sin_family      = NET_AF_INET;
  S_ADDR(addr.sin_addr) = net_aton_r (REMOTE_IP_ADDR);

  net_setsockopt(sock, NET_SOL_SOCKET, NET_SO_RCVTIMEO, &timeout, sizeof(int32_t*));
  net_setsockopt(sock, NET_SOL_SOCKET, NET_SO_SNDTIMEO, &timeout, sizeof(int32_t*));

  if( net_connect( sock,  (sockaddr_t *)&addr, sizeof( addr ) ) == 0 )
  {
    WIFI_PRINTF("- Device connected to the %s \n", REMOTE_IP_ADDR);

    fillbuffer(buffer,N+LOOP);
    tstart = HAL_GetTick();
    for( i=0; i<LOOP; i++)
    {
      int       n,m;
      m=0;
      do
      {
        n=net_send (sock, &buffer[i+m],N-m,0);
		if ( n == NET_TIMEOUT)
        {
          n=0;
          WIFI_PRINTF("Oups should not timeout while sending %d data\n",N-m);
        }

        if (n<0)
        {
          WIFI_PRINTF("Error Transmit %d : ",n);
          goto end;
        }
#ifdef CHECK
        WIFI_PRINTF("send to echo %d\n",n);
#endif
        m+=n;
      }
      while(m<N);
      transfer+=m;

      m=0;
      do
      {
        n= net_recv (sock, &buffin[m], N-m,0);
		if ( n == NET_TIMEOUT)
        {
          n=0;
          WIFI_PRINTF("Oups should not timeout while reading %d data\n",N-m);
        }

        if (n<0)
        {
          WIFI_PRINTF("Error Transmit %d : ",n);
          goto end;
        }

#ifdef CHECK
       WIFI_PRINTF("recv from echo %d\n",n);
#endif
        m+=n;
      }
      while(m<N);
      transfer+=m;

#ifdef CHECK
      if (!checkbuffer(buffin,N,i))
      {
        WIFI_PRINTF("Error: Unexpected echoed data: iteration #%d\n",i);
        goto end;
      }
#endif
    }
    tstop = HAL_GetTick();
    WIFI_PRINTF("Transfer %d bytes in %d ms , br = %d Kbit/sec\n",transfer,tstop-tstart,(transfer*8)/(tstop-tstart));
  }
  else
  {
    WIFI_PRINTF("Failed to connect to %s\n",REMOTE_IP_ADDR);
  }


end:
  net_closesocket(sock);
}


#define HTTP_PORT       80
void TestServer(void)
{
  int sock;
  sockaddr_in_t addr;
  uint8_t buffer[200];
  uint32_t addrlen=sizeof(sockaddr_in_t);
  int32_t sd;


  sock = net_socket ( NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);
  addr.sin_port        = NET_HTONS(HTTP_PORT);
  addr.sin_family      = NET_AF_INET;
  addr.sin_addr.s_addr = netif.ipaddr.addr;

  if(net_bind (sock, (net_sockaddr_t *)&addr, sizeof(addr))== NET_OK)
  {
    if( net_listen( sock, 5) == NET_OK)
    {
      WIFI_PRINTF("Please open a browser and go to address %s\n", net_ntoa(&netif.ipaddr));
      WIFI_PRINTF("\n Timeout for a valid connection: %d msec \n", ES_WIFI_MAX_SO_TIMEOUT);

      while((sd = net_accept (sock, (sockaddr_t *)&addr, &addrlen))>=0)
      {
        int n;
        WIFI_PRINTF("\n- Device %s connected to socket %d derived from %d! \n", net_ntoa(&netif.ipaddr),(int)sd,sock);

        do
        {
          n =net_recv (sd, buffer, sizeof(buffer),0);
          if (n > 0)
          {
            int l = n;
            if (l > 100) l=100;
            buffer[l]=0;
            WIFI_PRINTF("\n socket %d: received %d bytes -> %s \n",(int)sd,n,buffer);
          }
          else
          {
          SendWebPage(sd);
          }
        }
        while(n>0);
        net_closesocket(sd);
      }
      WIFI_PRINTF("no more connection to server\n");
      net_closesocket(sock);

    }
  }
}


/**
* @brief  Send HTML page
* @param  None
* @retval None
*/
static void SendWebPage(int sock)
{
  uint8_t http[1024];
  int n;

  /* construct web page content */
  strcpy((char *)http, (char *)"Test client - server connection\r\n");

  n=net_send (sock,(uint8_t *)http, strlen((char *)http),0);
  if ( n == NET_TIMEOUT)
  {
    n=0;
    WIFI_PRINTF("Oups should not timeout while sending %d data\n",strlen((char *)http));
  }
}


