@echo off

rem /////////////////////////// CaveScan_Start.bat ///////////////////////////

rem This file is the CaveScan startup batch file.
rem Created by Y.Fujii on November 1, 2023.
rem License declaration 2.0.

rem //////////////////////////////////////////////////////////////////////////


setlocal ENABLEDELAYEDEXPANSION

rem Directory Move
cd %~dp0
set LUNCH_DIR=%~dp0
rem WorkSpase Directory
cd %LUNCH_DIR%..
set CaveScan_DIR=%CD%\

rem If a computer name error occurs, set the following variables.
rem set COMPUTERNAME=yuki-PC

rem Startup of RTC
start "" /d %CaveScan_DIR%EtheURG\build\src\Debug EtheURGComp.exe
start "" /d %CaveScan_DIR%MeasurementSystem\build\src\Debug MeasurementSystemComp.exe
timeout 10

cd %LUNCH_DIR%
rem Automatic ports connection
rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/EtheURG0.rtc:range ^
/localhost/%COMPUTERNAME%.host_cxt/MeasurementSystem0.rtc:range ^
--property dataport.interface_type=shared_memory

rem Automatically configuration
rem rtconf /localhost/%COMPUTERNAME%.host_cxt/MeasurementSystem0.rtc set DEVICE_NAME \\.\COM5

endlocal