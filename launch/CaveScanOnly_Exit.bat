@echo off

rem /////////////////////////// CaveScan_Exit.bat ///////////////////////////

rem This file is the CaveScan Exit batch file.
rem Created by Y.Fujii on November 1, 2023.
rem License declaration 2.0.

rem //////////////////////////////////////////////////////////////////////////


setlocal ENABLEDELAYEDEXPANSION

rem Directory Move
cd %~dp0
set LAUNCH_DIR=%~dp0
rem WorkSpase Directory
cd %LAUNCH_DIR%..
set CaveScan_DIR=%CD%\

rem If a computer name error occurs, set the following variables.
rem set COMPUTERNAME=yuki-PC

rem System shutdown
rem RTC Deactive
rtdeact /localhost/%COMPUTERNAME%.host_cxt/EtheURG0.rtc
rtdeact /localhost/%COMPUTERNAME%.host_cxt/MeasurementSystem0.rtc
rtdeact /localhost/%COMPUTERNAME%.host_cxt/PointCloud_Viewer0.rtc
timeout 1

rem Disconnect between ports
rtdis ^
/localhost/%COMPUTERNAME%.host_cxt/EtheURG0.rtc:range ^
/localhost/%COMPUTERNAME%.host_cxt/MeasurementSystem0.rtc:range

rtdis ^
/localhost/%COMPUTERNAME%.host_cxt/PointCloud_Viewer0.rtc
timeout 1

rem RTCs termination process
rtexit /localhost/%COMPUTERNAME%.host_cxt/EtheURG0.rtc
rtexit /localhost/%COMPUTERNAME%.host_cxt/MeasurementSystem0.rtc
rtexit /localhost/%COMPUTERNAME%.host_cxt/PointCloud_Viewer0.rtc

endlocal