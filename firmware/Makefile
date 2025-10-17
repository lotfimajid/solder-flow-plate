# ============================
# Configurable Parameters
# ============================
MCU ?= atmega328p     # Default MCU (can be overridden via command line)
F_CPU ?= 8000000       # Default clock frequency
CC = avr-gcc
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=c99
OBJ = main.o ssd1306.o twi.o

# Detect avrdude part name automatically from MCU
AVRDUDE_PART = $(shell echo $(MCU) | tr A-Z a-z)

# ============================
# Main Targets
# ============================

all: clean reflow.hex flash

%.o: %.c
	@echo "🔧 Compiling $< ..."
	@$(CC) $(CFLAGS) -c $< -o $@

reflow.elf: $(OBJ)
	@echo "🔗 Linking..."
	@$(CC) $(CFLAGS) -o $@ $(OBJ)

reflow.hex: reflow.elf
	@echo "📦 Creating HEX file..."
	@avr-objcopy -O ihex $< $@

flash: reflow.hex
	@echo "⚡ Flashing firmware to $(MCU)..."
	@avrdude -c usbasp -p $(AVRDUDE_PART) -B 8 -U flash:w:$<

fusebits_write:
	@echo "🧩 Writing fuse bits for $(MCU)..."
	@avrdude -c usbasp -p $(AVRDUDE_PART) -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m -U efuse:w:0xF9:m

fusebits_read: 
	@avrdude -c usbasp -p $(AVRDUDE_PART) -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h
	@echo "Fuse bits (expected): lfuse=0xE2, hfuse=0xDF, efuse=0xF9"

clean:
	@echo "🧹 Cleaning build files..."
	@rm -f *.o *.elf *.hex

