/**
  @page Serial_DataLog application for STWIN
  
  @verbatim
  ******************************************************************************
  * @file    readme.txt  
  * @author  SRA
  * @version v1.3.0
  * @date    13-Nov-2020
  * @brief   This application contains an example which shows how to obtain data
  *          from the various sensors on the STWIN. The data can either be
  *  	     streamed to the PC via USB or saved on SD Card
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

Serial_DataLog application has two major features: sensor raw data streaming via USB
(Virtual COM Port class) and sensor data storage on SD card.
If USB is selected, the flowing data can be read using a generic application
like TeraTerm/Putty or any other.
If SD Card is selected, the data stream is logged to a csv file (Comma Separated Values).
After the application is started:
- the user can view/record data from the on-board environmental sensors like Temperature, 
Humidity (if available) and Pressure.
- the user can view/record data from the on-board MEMS sensors like Accelerometer,
Gyroscope and Magnetometer.

@par Hardware and Software environment

  - This application runs on STWIN platform available in STEVAL-STWINKT1.
  - STM32 Virtual COM Port Driver for Windows can be downloaded from st.com - STSW-STM32102
    
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
