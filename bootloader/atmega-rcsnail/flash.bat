@REM SET PATH=..\..\..\..\tools\avr\bin\;%PATH%
@REM make OS=windows ENV=arduino %*
@REM PATH=C:\tools\arduino-1.7.8\hardware\tools\avr\bin\;%PATH%
@SET PATH=C:\msys64\usr\bin\;C:\msys64\mingw64\bin\;C:\tools\arduino-1.7.8\hardware\tools\avr\bin\;%PATH%
avrdude.exe -v
make atmega328_rcsnail8_isp
@REM make atmega328_pro8
