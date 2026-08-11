#include <stdio.h>
#include <inttypes.h>
#include "mcp2515.h"
#include "systick.h"
#include "led.h"
#include "bmp280.h"

volatile uint8_t can_int_flag = 0;
volatile uint8_t can_bus_off = 0;
volatile uint8_t can_intf_stuck = 0;

// globally declared vaiable with physically allocated memory in RAM
I2C_HandleTypeDef hi2c;

/*

Week 21: Integration Project — CAN Sensor Node

This is not a tutorial week. You build a complete system from everything in Phase 3.

Project: BMP280 reads temperature and pressure over I2C. The STM32 packages the readings into a CAN frame and broadcasts on the bus every 500ms. A second node (or loopback) receives the frame and prints the decoded values over UART.

This is a real automotive pattern. A sensor ECU reads a physical quantity and broadcasts it on the bus. Any other ECU on the bus can consume it.

CAN frame design:

ID: 0x100 for temperature, 0x101 for pressure (or pack both into one 8-byte frame)
DLC: 4 bytes per value
Data: raw int32_t / uint32_t from BMP280 compensation functions

Integration requirements:

BMP280 state machine running on I2C interrupt driver (your existing code)
SPI transaction to the MCP2515 triggered from an interrupt
UART output for debugging
SysTick for 500ms broadcast timer

Done bar: stable temperature and pressure values broadcast on CAN bus every 500ms, decoded and printed on receiving end. No bus-off events after 60 seconds of continuous operation.

*/

int main(void)
{
    // hi2c: define and initialize structs
    hi2c.channel = I2C_CHANNEL_1;
    hi2c.scl_port = GPIOB;
    hi2c.scl_pin = 6;
    hi2c.sda_port = GPIOB;
    hi2c.sda_pin = 7;
    hi2c.state = I2C_STATE_IDLE;
    hi2c.sb_hits = 0;
    hi2c.stop_hits = 0;
    hi2c.start_pending_hits = 0;

    BMP280_HandleTypeDef hbmp = {
        .hi2c = &hi2c,
        .slave_addr = BMP280_I2C_ADDR,
        .isInitialized = 0};

    BMP280_Ctrl_Meas_t meas = {
        .osrs_p = BMP280_OSRS_P_OVRSMP_1,
        .osrs_t = BMP280_OSRS_T_OVRSMP_1,
        .mode = BMP280_FORCED_MODE};

    I2C_Init();
    SysTick_Init(SYSTICK_FREQUENCY_16MHZ);
    spi_init(SPI_BR_8);
    usart2_init();

    // error led initialization
    LED_HandleTypeDef ledHandle = {
        .pin = 0,
        .rcc_bit = 0,
        .moder_reg = &(GPIOA->MODER),
        .odr_reg = &(GPIOA->ODR),
        .rcc_clk_reg = &(RCC->AHB1ENR)};

    led_init(&ledHandle);
    // error led initialization

    uint8_t isSuccess;

    mcp2515_reset();

    uint8_t rx_byte = 0;

    mcp2515_read(CANSTAT1, &rx_byte, 1U);

    // CNF3, CNF2, CNF1
    uint8_t CNF_vals[3] = {0x01, 0x91, 0x00};

    // CNF3 = 0x28, CNF2 = 0x29, CNF1 = 0x2A
    // since all three registers are consecutive, we can write them in one write operation
    mcp2515_write(CNF3, CNF_vals, 3U);

    uint8_t read_CNF_vals[3];

    mcp2515_read(CNF3, read_CNF_vals, 3U);

    uint8_t CANCTRL_val;

    mcp2515_read(CANCTRL1, &CANCTRL_val, 1);

    mcp2515_init();

    // enable interrupts via MCP2515 CANINTE register
    // bit 5 ERRIE, bit 1 RX1IE, bit 0 RX0IE

    // mask: 0010 0011 = 2^5 + 2^1 + 2^0 = 32 + 2 + 1 = 35 = 0x23
    // data byte: 0010 0011 = 0x23

    mcp2515_bit_modify(CANINTE, 0x23, 0x23);

    // request Normal mode (bits 7-5: 000)

    // 0b1110 0000 = 0xE0 <- mask, bits 7-5 are requested to change
    // 0b0000 0000 = 0x00 <- data byte, bits 7-5 are requested to changed to 000
    mcp2515_bit_modify(CANCTRL1, 0xE0, 0x00);

    // waiting loop for mode to change
    // mask: 0b1110 0000 = 0xE0 - bits 7-5 are relevant only

    mcp2515_poll_bit_timeout(CANSTAT1, 0xE0, 0x00, 10U, &isSuccess);

    // mode has not been set when the timeout hit
    // indefinite red led error blink
    if (isSuccess != 1)
    {
        while (1)
        {
            led_on(&ledHandle);

            SysTick_Delay_ms(300);

            led_off(&ledHandle);

            SysTick_Delay_ms(300);
        }
    }

    uint32_t last_broadcast_tick = SysTick_GetTick();

    // // poll the RX0IF in READ_STATUS
    // // RX0IF - receive buffer 0 Full Interrupt Flag bit
    // // when RX0IF is 1 - interrupt is pending (must be cleared by MCU to reset the interrupt condition)
    // uint8_t status_val = 0;

    // // TODO
    // // an unbounded poll - a deliberate, correct, test-harness wait for a human-triggered external event, with a clear exit condition (cansend)
    // // should be refactored into the timeout-and-defined-behavior version for a real deployment (or the Week 21 integration project)
    // do
    // {
    //     mcp2515_read_status(&status_val, 1U);
    // } while (!(status_val & (1 << 0)));

    /*
    // now this EFLG polling loop is unbounded
    // a real deployment would want this non-blocking (checked periodically rather than blocking main())
    do
    {
        // read the EFLG register (0x2D) awaiting for the RX0VR
        mcp2515_read(EFLG, &status_val, 1U);
    } while (!(status_val & (1 << 6)));
    // after the second frame arrived, causing OVR, the bit 6 RXOVR is set, so the loop exits

    // once RX buffer has the data frame, retrieve it
    uint8_t rx_frame_bytes[9];

    mcp2515_read_rx_buffer(MCP_Read_RXB0SIDH, rx_frame_bytes, 9U);
    // the READ RX BUFFER instruction automatically clears the associated receive flag, RXnIF (CANINTF), when CS is raised at the end of the command
    // so the RX0IF flag will be cleared automatically, so the hardware condition to reset the interrupt condition is satisfied
    */

    uint8_t can_intf_val;
    uint8_t can_erlg_val;

    uint32_t grace_window_ms;

    // a data frame:
    // SIDH, SIDL, EID8 (zeroed out, don't care), EID0 (zeroed out, don't care), DLC, up to 8 data bytes
    // 1 + 1 + 1 + 1 + 1 + 8 = 5 + 8 = 13
    uint8_t can_int_rx0_header[5];
    uint8_t can_int_rx0_payload[8];
    uint8_t can_int_rx0_flag = 0;

    uint8_t can_int_rx1_header[5];
    uint8_t can_int_rx1_payload[8];
    uint8_t can_int_rx1_flag = 0;

    while (1)
    {

        if (can_int_flag)
        {
            do
            {
                mcp2515_read(CANINTF, &can_intf_val, 1U);

                if (can_intf_val)
                {
                    mcp2515_canintf_handler(can_intf_val, can_int_rx0_header, can_int_rx0_payload, &can_int_rx0_flag, can_int_rx1_header, can_int_rx1_payload, &can_int_rx1_flag);
                }
            } while (can_intf_val & ~(1 << 5));

            can_int_flag = 0;
        }

        if (SysTick_GetTick() - last_broadcast_tick >= 500U)
        {
            last_broadcast_tick = SysTick_GetTick();

            if (can_intf_stuck)
            {
                printf("The MCP2515 experiences unhandled error condition!");
                fflush(stdout);
            }

            if (can_bus_off)
            {
                mcp2515_read(EFLG, &can_erlg_val, 1U);

                if (!(can_erlg_val & MCP_EFLG_TXBO))
                {

                    grace_window_ms = SysTick_GetTick();

                    while (SysTick_GetTick() - grace_window_ms < 10U)
                        ;

                    can_bus_off = 0;
                }
            }

            if (!can_bus_off && !can_intf_stuck)
            {
                // SIDH, SIDL, EID8 (zeroed out, don't care), EID0 (zeroed out, don't care), DLC, up to 8 data bytes

                // normal path: read BMP280 values, build frames, load TX buffers, RTS, done
                uint8_t test_bytes[4] = {0xAA, 0x45, 0xB1, 0x22};
                uint16_t ID = 0x100; // 11 bits; 11-0, 15-12 are unused
                uint8_t SIDH = ID >> 3;
                uint8_t SIDL = (ID & 0x7) << 5;

                // DLC: bit 6 RTR - 0, bits 3-0 DLC
                // 4 bytes = 0100
                // TXB0DLC - 0 0 00 0100 => 0000 0100 = 0x4 = 2^2
                uint8_t DLC = 0x4;

                // SIDH, SIDL, EID8 (zeroed out, don't care), EID0 (zeroed out, don't care), DLC, 4 data bytes

                uint8_t data_payload[9] = {SIDH, SIDL, 0x00, 0x00, DLC, test_bytes[0], test_bytes[1], test_bytes[2], test_bytes[3]};

                mcp2515_load_tx_buffer(MCP_Load_TXB0D0, data_payload, 9);

                MCP_RTS_locations_t location = MCP_RTS_TXB0;
                mcp2515_rts(&location, 1U);
            }
        }
    }

    return 0;
}