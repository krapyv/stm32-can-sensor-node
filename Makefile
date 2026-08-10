# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy

# --- Centralized Driver Path ---
SHARED_CORE_DIR = /mnt/hdd/embedded-journey/my_reusable_drivers

INCLUDES = -I. -I${SHARED_CORE_DIR}/core -I${SHARED_CORE_DIR}/periph -I${SHARED_CORE_DIR}/utils -I${SHARED_CORE_DIR}/devices/mcp2515 -I${SHARED_CORE_DIR}/devices/bmp280 -I${SHARED_CORE_DIR}/devices/led

# Compiler flags (same for all STM32F411 projects)
CFLAGS = -mcpu=cortex-m4 -mthumb -nostartfiles -g3 -O0 --specs=nano.specs --specs=nosys.specs -lc -DUSE_APP_CONFIG
LDFLAGS = -T stm32f411.ld

# Project files (CHANGE THIS FOR EACH PROJECT)
SRC = startup_stm32f411ceux.s ${SHARED_CORE_DIR}/periph/i2c.c ${SHARED_CORE_DIR}/periph/spi.c ${SHARED_CORE_DIR}/periph/uart.c ${SHARED_CORE_DIR}/periph/systick.c ${SHARED_CORE_DIR}/devices/mcp2515/mcp2515.c ${SHARED_CORE_DIR}/devices/bmp280/bmp280.c ${SHARED_CORE_DIR}/devices/led/led.c main.c
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
