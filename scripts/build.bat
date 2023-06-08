@echo off

cd %1

echo Cleaning up...
del /f /q build\*.o build\*.elf build\*.hex

echo Building...
tools\avr8-gnu-toolchain-win32_x86_64\bin\avr-gcc.exe -v -mmcu=atmega328p -Os -DF_CPU=8000000UL -c -o build\main.o src\main.c
tools\avr8-gnu-toolchain-win32_x86_64\bin\avr-gcc.exe -v -mmcu=atmega328p build\main.o -o build\main.elf

echo Generating hex file...
tools\avr8-gnu-toolchain-win32_x86_64\bin\avr-objcopy.exe -v -O ihex -R .eeprom build\main.elf build\main.hex