@echo off

cd %1

echo Cleaning up...
del /f /q build\*.o build\*.elf build\*.hex

echo Building...
for %%i in (src\*.cpp) do (
    echo Building %%i
    tools\avr8-gnu-toolchain-win32_x86_64\bin\avr-gcc.exe -mmcu=atmega328p -Os -DF_CPU=16000000UL -c -o build\%%~ni.o %%i
)

tools\avr8-gnu-toolchain-win32_x86_64\bin\avr-gcc.exe  -mmcu=atmega328p build\*.o -o build\main.elf

echo Generating hex file...
tools\avr8-gnu-toolchain-win32_x86_64\bin\avr-objcopy.exe  -O ihex -R .eeprom build\main.elf build\main.hex