@echo off

REM Add GCC compiler to system path. See: http://www.mingw.org/
set path=C:\MinGW\bin;%PATH%

REM Run GCC compiler
gcc -Wall -pedantic -std=c99 ..\Fusion\FusionAhrs.c ..\Fusion\\FusionBias.c ..\Fusion\\FusionCompass.c ExampleAhrs.c -o ExampleAhrs
gcc -Wall -pedantic -std=c99 ..\Fusion\FusionAhrs.c ..\Fusion\\FusionBias.c ..\Fusion\\FusionCompass.c ExampleAhrsWithoutMagnetometer.c -o ExampleAhrsWithoutMagnetometer
gcc -Wall -pedantic -std=c99 ..\Fusion\FusionAhrs.c ..\Fusion\\FusionBias.c ..\Fusion\\FusionCompass.c ExampleCompass.c -o ExampleCompass

REM Pause to view warnings or errors
pause

REM Run compiled executable
ExampleAhrs
ExampleAhrsWithoutMagnetometer
ExampleCompass

REM Pause to view output
pause
