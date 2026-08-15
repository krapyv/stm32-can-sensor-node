# Integration Project — CAN Sensor Node

The project that handles integration of BMP280/I2C, CAN/SPI, SysTick and UART drivers in one fully functional program. 
BMP280 over I2C -> STM32F411 -> MCP2515 over SPI -> CAN bus -> CANable with temperature/pressure broadcasting over CAN and UART debug print every 500ms and error handling to prevent bus-off events across a multi-minute run.

## Project Structure

```text
reusable_drivers/
├── core/   # ARM Cortex-M4 and STM32F411 register definitions
│     ├── stm32f411.h     # Memory boundaries and register definitions for AHB/APB peripherals
│     └── core_cm4.h      # Register layout definitions for NVIC and SysTick architectures
├── devices/bmp280      
│    ├── bmp280.h        # Device handle, state machine enums, register map, and public API signatures
│    └── bmp280.c     # BMP280 peripheral driver: state machine functions, calculation and compensation functions
├── devices/mcp2515      
│    ├── mcp2515.h        # SPI instructions, CAN register addresses, state enums, and public API signatures
│    └── mcp2515.c       # MCP2515 peripheral driver:  transaction, ISR and util functions
├── devices/led
│   ├── led.c       # Initialization and toggle functions implementation
│   └── led.h       # Struct layouts, configuration handles, and function prototypes   
├── periph/     # Portable peripheral drivers
│    ├── spi.c   # Register-level SPI peripheral driver: initialization and transfer functions
│    ├── spi.h   # SPI_Channel_t enum for future multi-channel expansion and function headers
│    ├── i2c.c      # Register-level I2C peripheral driver: EV/ER ISR logic, transaction state machine, and SWRST recovery
│    ├── i2c.h      # I2C control and state enums, handler struct declaration, extern variable declaration, and function headers
│    ├── uart.c      # USART2 register configuration and DMA1 Stream 6 transfer invocation
│    ├── uart.h      # USART2 bit definitions, control macros, and function
│    ├── systick.c   # SysTick initialization, counter variable and functions
│    └── systick.h   # SysTick mode, BRR, clock and register configuration and function headers
│
├── main.c      # Application entry point: BMP280 state machine, CAN broadcast loop, UART debug output
└── decode_bmp280.py    # Python/python-can decoder for human-readable CAN frame output
```

## Description

BMP280 reads temperature and pressure over I2C. The STM32 packages the readings into a CAN frame and broadcasts on the bus every 500ms. A second node (or loopback) receives the frame and prints the decoded values over UART.

This is a real automotive pattern. A sensor ECU reads a physical quantity and broadcasts it on the bus. Any other ECU on the bus can consume it.

**CAN frame design:**
ID: 0x100 for temperature, 0x101 for pressure (or pack both into one 8-byte frame)
DLC: 4 bytes per value
Data: raw int32_t / uint32_t from BMP280 compensation functions

**Integration requirements:**
- BMP280 state machine running on I2C interrupt driver.
- SPI transaction to the MCP2515 triggered from an interrupt
- UART output for debugging
- SysTick for 500ms broadcast timer

**Done bar:**
Stable temperature and pressure values broadcast on CAN bus every 500ms, decoded and printed on receiving end. No bus-off events after 60 seconds of continuous operation.


## Breadboard circuit

Two breadboards: 
- the first one contains the Blackpill and TXS0108E (A).
- the second one has the BMP280 (B).

The power rails of the second breadboard are connected to the 3.3V rails on the breadboard A since BMP280 requires only 3.3V (orange jumper wires).
Both ground rails of the breadboard B are also connected to the ground rails of the breadboard A (brown jumper wires).

The TXS0108E level shifter is sitting on the breadboard straddling the separation channel. The MCP2515 module sits off-board (male pins, not breadboard-mounted) as well as the CANable USB to CAN debugger-analyzer. 

**Breadboard A:**
Rail split: right rails = 3.3V, left rails = 5V. TXS0108E straddles the separation row — VA (3.3V) side faces right rails, VB (5V) side faces left rails.
Level shifter placed directly behind Black Pill to keep A-side hop short per capacitance budget.

TXS0108E side A faces 3.3V power rail, side B - 5V power rail.

MCP2515 VCC is connected to the 5V power rail.
MCP2515 GND is connected to the common ground rail (GND).
MCP2515 SCK is connected to the TXS0108E B1 pin.
MCP2515 SI (MOSI) is connected to the TXS0108E B2 pin.

MCP2515 SO (MISO) is connected directly to STM32 PA6, bypassing the level shifter. STM32 input pins are 5V-tolerant on this line, and MISO is driven by the MCP2515 - the 3.3V STM32 receiver correctly interprets the 5V logic high.

MCP2515 CS is connected to the TXS0108E B3 pin.
MCP2515 INT is connected to the TXS0108E B4 pin.

STM32 PA5 (SCK) is connected to the TXS0108E A1.
STM32 PA7 (MOSI) is connected to the TXS0108E A2.
STM32 PA4 (CS) is connected to the TXS0108E A3.
STM32 PB15 (INT) is connected to the TXS0108E A4.

TXS0108E VA (3.3V side) connected to the 3.3V power rail.
TXS0108E VB (5V side) connected to the 5V power rail.
TXS0108E GND is connected to the common ground rail (GND).
TXS0108E OE pin is connected to 3.3V power rail.

The CANable GND is connected to the common ground rail (GND).
The CANable CAN_H and CAN_L are connected to the MCP2515 CAN_H and CAN_L respectively.

The Normal mode transition error led:
The 220 Ohms resistor terminal 1 is connected to the PA0 of the STM32.
The 220 Ohms resistor terminal 2 is connected to the anode (the longer leg) of the LED.
The cathode (the shorter leg) of the LED is connected to the GND rail.

**Breadboard B:**
- VCC is connected to the power rail (red jumper wire).
- GND is connected to the Ground rail (black jumper wire).
- CSB is connected to the power rail  (orange jumper wire). If the CSB is connected to the VDDIO (VCC), the I2C interface is active. Otherwise, if the CSB is connected to the GND (0V), the SPI interface is active. 
- SDO is connected to the GND rail (brown wire). The driver uses by default address 1110110 (0x76). 
* The driver uses by default SDA1 and SCL1:
- SDA of the BMP280 is connected to the PB7 of the Blackpill (green wire).
- SCL of the BMP280 is connected to the PB6 of the Blackpill (yellow wire).

* I2C is open-drain. The lines need pull-ups to VCC to define the idle high state. Without pull-ups configured either externally or internally, the lines float and the bus never reaches a defined idle state.
- In the breadboard circuit there were used 4.7k Ohms pull-up resistors on both SDA and SCL connected to power rails (rise time 217ns from 1000ns available).

* The UART to USB adapter:
- GND is connected to ground rail (black wire).
- TXD is connected to PA3 of the Blackpill (RX2) by a blue wire. 
- RXD is connected to PA2 of the Blackpill (TX2) by a yellow wire.

IMPORTANT: the UART-to-USB adapter's VCC pin SHOULD be left unconnected. The Blackpill itself has the USB powering, so with the UART also having a connection to the 3.3V with the STM32, there are two power supplies with different or unregulated voltages fighting to drive the same rail. The STM32F411's 3.3V rail is being driven by its onboard regulator (fed from USB 5V -> 3.3V), and the UART adapter's VCC pin is also trying to source 3.3V onto that same rail, there are two low-impedance voltage sources both trying to set the same node - the one with a slightly higher effective voltage will attempt to push current backward into the other's regulator output. Some regulators can be damaged over time or immediately, depending on current and design margin.

## How to run

Clone the repository, navigate to the project directory, and execute the toolchain commands:

```bash
# Clean previous build artifacts and compile the firmware binary
make clean
make all

# Flash the binary to the microcontroller using ST-LINK
make flash

# Check if the CANable is seen by the Linux
ip link show type can

# Configure and activate a CAN interface with the name can0 
sudo ip link set can0 up type can bitrate 500000
```

Then there are two options:

```bash
# Option A - to use a command-line tool to spy on and display traffic running through the can0 network interface in real time
candump can0

# Option B - to use a written Python/python-can decoder to print the frames in human-readable way
python3 decode_bmp280.py can0
```

In new terminal window or tab:
```bash
# Check what name the UART-to-USB adapter has
ls /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/* 2>/dev/null

# Open an UART serial communication session using minicom
minicom -D /dev/ttyUSB0 -b 115200
```

To debug via OpenOCD/GDB:
In the project directory:
```bash
# Start OpenOCD (Open On-Chip Debugger) - to creat a bridge between a computer and an STM32F4 microcontroller, so you can program, flash or debug it
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
```

In new terminal window in the same project directory:
```bash
# Start the GNU Debugger (GDB) specifically tailored for ARM Cortex-M microcontrollers. It loads the compiled code's blueprint (program.elf), so you can control, pause, and inspect the code running on your hardware
arm-none-eabi-gdb program.elf 

# Inside the GDB prompt:

# Connect to OpenOCD:
target remote :3333

# Load the code - this physically flashes the program.elf binary onto the STM32F4 chip's memory:
load

# Reset and stop the chip - this freezes the microcontroller right at its starting point
monitor reset halt
```

and start debugging.
