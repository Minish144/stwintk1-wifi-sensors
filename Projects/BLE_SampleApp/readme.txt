/**
  @page BLE_SampleApp application for STWIN
  
  @verbatim
  ******************************************************************************
  * @file    readme.txt  
  * @author  SRA
  * @version v1.4.0
  * @date    13-Nov-2020
  * @brief   This application contains an example which shows how to configure 
  *          the STWIN to stream Environmental sensors data via Bluetooth
  *	    	 Low Energy.
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

@par Application Description 

BLE_SampleApp provides an example of Bluetooth Low Energy configuration that
 enables STWIN to stream environmental sensors data; it is compatible 
 with STBLESensor app available for both Android and iOS.

 After reset the firmware performs the following actions:
 - Configure HAL and clocks
 - Configure and disable sensors Chip Select pins
 - Initilize the target platform:
   - USB peripheral (for debugging)
   - LED1
   - Environmental sensors
 - Initialize Bluetooth Low Energy stack
 - Initialize Bluetooth Low Energy services
 - Initialize timers
 - Main Loop:
   - Led management
   - BLE events management
   - Environmental sensors data mangement


@par Hardware and Software environment

  - This application runs on STWIN platform available in STEVAL-STWINKT1.
  - STBLESensor app (http://www.st.com/stblesensor) is available for both Android and iOS.
    
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


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
