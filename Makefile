# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy

INCLUDES = -I.

# Compiler flags (same for all STM32F411 projects)
CFLAGS = -mcpu=cortex-m4 -mthumb -nostartfiles -g3 -O0 --specs=nano.specs --specs=nosys.specs -lc -DUSE_APP_CONFIG
LDFLAGS = -T stm32f411.ld

# Project files - exactly ONE .c per unique driver, despite nested submodule duplication
SRC = startup_stm32f411ceux.s main.c systick/systick.c led/led.c uart/uart.c uart/ring_buffer/ring_buffer.c bmp280/bmp280.c bmp280.c/i2c/i2c.c mcp2515/mcp2515.c mcp2515/spi/spi.c

ELF = program.elf
BIN = program.bin

# Default target
all: $(BIN)

# Link: .c and .s -> .elf
$(ELF): $(SRC)
	$(CC) $(CFLAGS) $(INCLUDES) $(LDFLAGS) $(SRC) -o $(ELF)

# Convert: .elf -> .bin
$(BIN): $(ELF)
	$(OBJCOPY) -O binary $(ELF) $(BIN)

# Flash using ST-LINK
flash: $(BIN)
	STM32_Programmer_CLI -c port=SWD -w $(BIN) 0x08000000 -v -rst

# Flash using st-flash (alternative)
flash-st: $(BIN)
	st-flash --reset write $(BIN) 0x08000000

# Clean build artifacts
clean:
	rm -f $(ELF) $(BIN)

# Mark these as not real files
.PHONY: all flash flash-st clean
