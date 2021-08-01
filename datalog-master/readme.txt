/**
  @page HSDatalog application for STWIN
  
  @verbatim
  ******************************************************************************
  * @file    readme.txt  
  * @author  SRA
  * @version v3.0.0
  * @date    19-Jun-2020
  * @brief   This application contains an example which shows how to obtain data
  *          from the various sensors on the STWIN. The data can be saved on SD 
  *          Card
  ******************************************************************************
  *
  * Copyright (c) 2019 STMicroelectronics. All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                               www.st.com/SLA0044
  *
  ******************************************************************************
  @endverbatim

@par Application Description 

The HSDatalog (High-Speed DataLog) works with all sensors configured at their maximum sampling rate.
Sensor data can be stored on micro SD Card or can be streamed via USB to a PC host
 - WARNING: SD card performances are very different between each other.
   This example works properly with microSD card Speed Class 10 (C10) and UHS Speed Class 1 (U1)
   (i.e: SanDisk Industrial 32GB U1 C10, Verbatim Premium 16GB U1 C10 or Transcend Premium 16 GB U1 C10).
   It is not granted that the application can still work properly if a different kind of microSD card
   is used (i.e: you can see some data loss due to a SD Card low write speed).

To automatically read and plot the data, MATLAB and Python scripts are available in Utilites folder.
In order to handle JSON files the script requires MATLAB v2019a or newer.
Python class was developed and tested using Python 3.7.

Also a MATLAB app 'ReadSensorDataApp.mlapp' is available, developed and tested using the App Designer
tool available in MATLAB v2019a.


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
   Embedded Workbench V8.32.3).
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
 - Open STM32CubeIDE (this firmware has been successfully tested with STM32CubeIDE v1.3.0).
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the
   workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select 
   root directory" and choose the path where the project is located
 - Rebuild all files and load your image into target memory.
 - Run the example.


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
