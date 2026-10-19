@echo off
title STM32 ADC Data Logger
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" call INSTALL.bat
if errorlevel 1 exit /b 1
".venv\Scripts\python.exe" -c "import serial, tkinter" >nul 2>nul
if errorlevel 1 call INSTALL.bat
if errorlevel 1 exit /b 1
".venv\Scripts\python.exe" main.py
if errorlevel 1 pause
