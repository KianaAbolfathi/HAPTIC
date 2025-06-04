@echo off

taskkill /F /IM "python.exe"
taskkill /F /IM "Novint.exe"

start /MAX "PYTHON OUTPUT" python "%CD%\DobotControl.py"
start /MIN "READING DATA FROM FALCON..." "%CD%\bin\Master-Slave_Device.exe"
