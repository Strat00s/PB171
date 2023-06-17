@echo off

cd %1

echo Cleaning up...
del /f /q build\*.o build\*.elf build\*.hex

echo Building...
for %%i in (src\*.cpp) do (
    echo Building %%i
    tools\avr8-gnu-toolchain-win32_x86_64\bin\avr-gcc.exe -mmcu=attiny1624 -B tools\atpack\Atmel.ATtiny_DFP.2.0.368\gcc\dev\attiny1624 -Os -I tools\atpack\Atmel.ATtiny_DFP.2.0.368\include\ -DF_CPU=10000000UL -c -o build\%%~ni.o %%i
)

tools\avr8-gnu-toolchain-win32_x86_64\bin\avr-gcc.exe -mmcu=attiny1624 -B tools\atpack\Atmel.ATtiny_DFP.2.0.368\gcc\dev\attiny1624 -I tools\atpack\Atmel.ATtiny_DFP.2.0.368\include\ build\*.o -o build\main.elf

echo Generating hex file...
tools\avr8-gnu-toolchain-win32_x86_64\bin\avr-objcopy.exe  -O ihex -R .eeprom build\main.elf build\main.hex