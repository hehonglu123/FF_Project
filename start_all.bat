@echo off

C:
cd C:\Users\fusing\Desktop\abb_driver_csharp-2021-05-03
if %errorlevel% neq 0 exit /b %errorlevel%
start cmd /c dotnet ABBRobotRaconteurDriver.dll -robot-info-file=abb_1200_5_90_robot_default_config.yml --robotraconteur-nodename=abb_robot 
if %errorlevel% neq 0 exit /b %errorlevel%

Y:
cd Y:\FF_Project\EA_Gripper
if %errorlevel% neq 0 exit /b %errorlevel%
start cmd /c python m1k_simple.py
if %errorlevel% neq 0 exit /b %errorlevel%

start cmd /c python gripper_service.py
if %errorlevel% neq 0 exit /b %errorlevel%

cd Y:\FF_Project\camera
start cmd /c python robotraconteur_camera_driver.py
if %errorlevel% neq 0 exit /b %errorlevel%

cd Y:\FF_Project