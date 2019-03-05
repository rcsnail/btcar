SET PATH=C:\tools\arduino-1.8.8\hardware\tools\avr\bin;%PATH%
avrdude.exe -v
avrdude -c usbasp -p atmega328p -P usb -b 115200 -e -u -U lock:w:0xff:m -U efuse:w:0xFF:m -U hfuse:w:0xD8:m -U lfuse:w:0xFF:m
avrdude -c usbasp -p atmega328p -P usb -b 115200 -U flash:w:RCSnail.ino.with_bootloader.arduino_eightanaloginputs.hex -U lock:w:0xcf:m
