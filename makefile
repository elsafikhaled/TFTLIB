MMU = atmega32u4
HEX = main.hex
FILES = main.c TFTLIB.c
PROGRAMMER = usbtiny

all:
	avr-g++ -g -Os -mmcu=$(MMU) -c $(FILES)
	avr-g++ -g -mmcu=$(MMU) -o main.elf main.o TFTLIB.o
	avr-objcopy -j .text -j .data -O ihex main.elf $(HEX)

burn:
	avrdude -p $(MMU) -c $(PROGRAMMER) -U flash:w:$(HEX)

clean:
	rm *.hex
	rm *.elf
	rm *.o
