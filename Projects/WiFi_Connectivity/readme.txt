/**
  @page WiFi_Connectivity example for STWIN

  @verbatim
  ******************************************************************************
  * @file    readme.txt
  * @author  SRA
  * @version v1.4.0
  * @date    13-Nov-2020
  * @brief   Description of WiFi_Connectivity example for STWIN
  ******************************************************************************
  *
  * Copyright (c) 2020 STMicroelectronics. All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                               www.st.com/SLA0044
  *
  ******************************************************************************
   @endverbatim

@par Example Description

< This example implements basic network function using the connectivity framework:
	 - Ping a remote station
     - Connection to a TLS secure server without server identification check
     - Connection to a TLS secure server with server identification check 
     - Send data to an echo server and check return data
	 - Run an Server, waiting for connection from a remote client

@par Hardware and Software environment
 

 - Edit the file "main.c" to define the connection parameter for your network access point or set
 up an hotspot with the the default ones:
#define SSID        "stwintest"
#define PASSWORD    "stwintest"

 - Edit the file test_client_server.c to setup the correct address for the TCP echo server.
  By default the ARM-Mbed echo server is used. Normally this server is available and you don't have
  to change it.
  
  #define REMOTE_IP_ADDR "52.215.34.155"
  #define REMOTE_PORT     7

@par How to use it? 

This package contains projects for 3 IDEs viz. IAR, µVision and STM32CubeIDE. In order to make
the program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:
 - Open IAR toolchain (this firmware has been successfully tested with
   Embedded Workbench V8.50.5).
 - Open the IAR project file EWARM\Project.eww
 - Rebuild all files and load your image into target memory.
 - Run the example.

For µVision:
 - Open µVision 5 toolchain (this firmware has been successfully tested with MDK-ARM Professional 
   Version: 5.30).
 - Open the µVision project file MDK-ARM\Project.uvprojx
 - Rebuild all files and load your image into target memory.
 - Run the example.

For STM32CubeIDE:
 - Open STM32CubeIDE (this firmware has been successfully tested with STM32CubeIDE v1.4.2).
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the
   workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select 
   root directory" and choose the path where the project is located
 - Rebuild all files and load your image into target memory.
 - Run the example.

 
 - Configure the required settings (to be done only once):
   - When the board is connected to a PC with USB (ST-LINK USB port),
     open a serial terminal emulator, find the board's COM port and configure it with:
      - 8N1, 115200 bauds, no HW flow control
      - set the line endings to LF or CR-LF (Transmit) and LF (receive).
 
 - Run the program, you should get following kind of output to console:
  		- Network Interface initialized:
		- Network Interface started:
		- Device Name : Inventek eS-WiFi
		- Device ID   : ISM43362-M3G-L44-SPI
		- Device Version : C3.5.2.3.BETA9
		- Network Interface connected:
		- IP address :  192.168.3.115
			Ping iteration #0 roundtrip 40
			Ping iteration #1 roundtrip 36
			Ping iteration #2 roundtrip 28
			Ping iteration #3 roundtrip 27
			Ping iteration #4 roundtrip 35
			Ping iteration #5 roundtrip 31
			Ping iteration #6 roundtrip 34
			Ping iteration #7 roundtrip 36
			Ping iteration #8 roundtrip 30
			Ping iteration #9 roundtrip 31
		Connecting to www.gandi.net at  ipaddress: 151.101.121.103
		Success get time from server www.gandi.net : 20 Mar 2019: 13:38:53
		Connecting to www.gandi.net at  ipaddress: 151.101.121.103
		Success get time from server www.gandi.net : 20 Mar 2019: 13:38:55
		- Device connected to the 52.215.34.155
			Transfer 116800 bytes in 3727 ms , br = 250 Kbit/sec
		Please connect to 192.168.3.115
  
 - At that stage, please connect a browser to the propose address, you should get such message on console.  
		- Device 192.168.3.107 connected to socket 1 derived from 0!
		socket 1: received 100
		socket 1: received 100
		socket 1: received 100
		socket 1: received 100
		socket 1: received 54
		socket 1: received 0
		ERROR: C:\git\IiotCLoud\repo1\Firmware\Middlewares\ST\STM32_Connect_Library\core\net_socket.c:271No connection has been established.
		no more connection to server
		
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */

