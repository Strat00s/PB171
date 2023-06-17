@echo off

echo Uploading to %2...
cd %1
tools\avrdude-v7.1-windows-x64\avrdude.exe -F -v -c jtag2updi -p attiny1624 -P %2 -b 115200 -U flash:w:build\main.hex