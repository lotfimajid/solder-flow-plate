MCU = atmega328p
F_CPU = 8000000
CC = avr-gcc
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=c99
OBJ = main.o ssd1306.o twi.o

all: clean reflow.hex flash

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

reflow.elf: $(OBJ)
	$(CC) $(CFLAGS) -o $@ $(OBJ)

reflow.hex: reflow.elf
	avr-objcopy -O ihex $< $@

flash: reflow.hex
	avrdude -c usbasp -p m328p -B 0.5 -U flash:w:$<

clean:
	rm -f *.o *.elf *.hex

