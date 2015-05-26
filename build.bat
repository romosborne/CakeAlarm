avr-gcc.exe -Iusbdrv -I. -Os -mmcu=attiny85 -DF_CPU=16500000 -c .\usbdrv\usbdrv.c -o .\usbdrv\usbdrv.o
avr-gcc.exe -Iusbdrv -I. -Os -mmcu=attiny85 -DF_CPU=16500000 -c .\usbdrv\usbdrvasm.S -o .\usbdrv\usbdrvasm.o
avr-gcc.exe -Iusbdrv -I. -Os -mmcu=attiny85 -DF_CPU=16500000 -c .\usbdrv\oddebug.c -o .\usbdrv\oddebug.o
avr-gcc.exe -Iusbdrv -I. -Os -mmcu=attiny85 -DF_CPU=16500000 -c .\main.c -o .\main.o


avr-gcc.exe -Iusbdrv -I. -Os -mmcu=attiny85 -DF_CPU=16500000 -o .\main.bin .\usbdrv\usbdrv.o .\usbdrv\usbdrvasm.o .\usbdrv\oddebug.o .\main.o

avr-objcopy.exe -j .text -j .data -O ihex main.bin main.hex