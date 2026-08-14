#include <stdio.h>
#include <inttypes.h>
#include "mcp2515.h"
#include "systick.h"
#include "led.h"
#include "bmp280.h"
#include "uart.h"

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

    hbmp.state = BMP280_STATE_INIT;

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

    // NOTE: RXnIE enablement here too
    // // enable interrupts via MCP2515 CANINTE register
    // // bit 5 ERRIE, bit 1 RX1IE, bit 0 RX0IE

    // // mask: 0010 0011 = 2^5 + 2^1 + 2^0 = 32 + 2 + 1 = 35 = 0x23
    // // data byte: 0010 0011 = 0x23
    // mcp2515_bit_modify(CANINTE, 0x23, 0x23);

    // enable interrupts via MCP2515 CANINTE register
    // bit 5 ERRIE

    // mask: 0010 0011 = 2^5 = 32 = 0x20
    // data byte: 0010 0000 = 0x20

    mcp2515_bit_modify(CANINTE, 0x20, 0x20);

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

    uint8_t can_intf_val;
    uint8_t can_erlg_val;

    uint32_t grace_window_ms;
    uint8_t grace_active = 0;

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
        I2C_Process();

        // NOTE: Legacy code block
        // if (can_int_flag)
        // {
        //     do
        //     {
        //         mcp2515_read(CANINTF, &can_intf_val, 1U);

        //         if (can_intf_val)
        //         {
        //             mcp2515_canintf_handler(can_intf_val, can_int_rx0_header, can_int_rx0_payload, &can_int_rx0_flag, can_int_rx1_header, can_int_rx1_payload, &can_int_rx1_flag);
        //         }
        //     } while (can_intf_val & ~((1 << 7) | (1 << 5)));

        //     can_int_flag = 0;
        // }

        if (SysTick_GetTick() - last_broadcast_tick >= 500U)
        {
            last_broadcast_tick = SysTick_GetTick();

            do
            {
                mcp2515_read(CANINTF, &can_intf_val, 1U);

                if (can_intf_val)
                {
                    mcp2515_canintf_handler(can_intf_val, can_int_rx0_header, can_int_rx0_payload, &can_int_rx0_flag, can_int_rx1_header, can_int_rx1_payload, &can_int_rx1_flag);
                }
            } while (can_intf_val & ~((1 << 7) | (1 << 5)));

            if (can_intf_stuck)
            {
                printf("The MCP2515 experiences unhandled error condition!\r\n");
                fflush(stdout);
            }

            if (can_bus_off)
            {
                printf("The bus is off!\r\n");
                fflush(stdout);

                if (!grace_active)
                {
                    mcp2515_read(EFLG, &can_erlg_val, 1U);

                    if (!(can_erlg_val & MCP_EFLG_TXBO))
                    {

                        grace_window_ms = SysTick_GetTick();

                        grace_active = 1;
                    }
                }
                else
                {
                    printf("The grace window is ongoing!\r\n");
                    fflush(stdout);

                    if (SysTick_GetTick() - grace_window_ms >= 10U)
                    {
                        can_bus_off = 0;
                        grace_active = 0;
                    }
                }
            }

            if (!can_bus_off && !can_intf_stuck)
            {
                if (hbmp.state == BMP280_STATE_READY)
                {

                    printf("Temp: %" PRId32 " degC | Press: %" PRIu32 " hPa\r\n", hbmp.temp_value / 100, hbmp.press_value / 256 / 100);
                    fflush(stdout);

                    // SIDH, SIDL, EID8 (zeroed out, don't care), EID0 (zeroed out, don't care), DLC, up to 8 data bytes

                    // normal path: read BMP280 values, build frames, load TX buffers, RTS, done

                    /* --- settings for both press and temp */

                    // DLC: bit 6 RTR - 0, bits 3-0 DLC
                    // 4 bytes = 0100
                    // TXB0DLC - 0 0 00 0100 => 0000 0100 = 0x4 = 2^2
                    uint8_t DLC = 0x4;
                    MCP_RTS_locations_t location1 = MCP_RTS_TXB0;
                    MCP_RTS_locations_t location2 = MCP_RTS_TXB1;

                    /* --- settings for both press and temp */

                    // 1. Temperature
                    // since the temp is int32_t => it consists of 4 int8_t bytes

                    int8_t temp_part_31_24 = (int8_t)(hbmp.temp_value >> 24); // 31 30 29 28 27 26 25 24 - 8 bytes
                    int8_t temp_part_23_16 = (int8_t)(hbmp.temp_value >> 16); // 23 22 21 20 19 18 17 16 - 8 bytes
                    int8_t temp_part_15_8 = (int8_t)(hbmp.temp_value >> 8);   // 15 14 13 12 11 10 9 8 - 8 bytes
                    int8_t temp_part_8_0 = (int8_t)(hbmp.temp_value >> 0);    // 15 14 13 12 11 10 9 8 - 8 bytes

                    // reconstruction of the int32_t => (temp_payload[3] << 24) | (temp_payload[2] << 16) | (temp_payload[1] << 8)  | (temp_payload[0] << 0)
                    uint16_t temp_id = 0x100; // 11 bits; 11-0, 15-12 are unused
                    uint8_t temp_sidh = temp_id >> 3;
                    uint8_t temp_sidl = (temp_id & 0x7) << 5;

                    // SIDH, SIDL, EID8 (zeroed out, don't care), EID0 (zeroed out, don't care), DLC, 4 data bytes

                    // the temperature can genuinely be negative
                    // the implicit int8_t -> uin8_t conversion preserving the bit pattern when the array is initialized
                    // the conversion in C is well-defined (wraps via modulo, effectively a reinterpret for two's-complement), so the bits going out over CAN will be correct
                    uint8_t temp_payload[9] = {temp_sidh, temp_sidl, 0x00, 0x00, DLC, temp_part_31_24, temp_part_23_16, temp_part_15_8, temp_part_8_0};

                    mcp2515_load_tx_buffer(MCP_Load_TXB0SIDH, temp_payload, 9);

                    mcp2515_rts(&location1, 1U);

                    // 2. Pressure
                    // since the pressure is uint32_t => it consists of 4 uint8_t bytes

                    uint8_t press_part_31_24 = (uint8_t)(hbmp.press_value >> 24); // 31 30 29 28 27 26 25 24 - 8 bytes
                    uint8_t press_part_23_16 = (uint8_t)(hbmp.press_value >> 16); // 23 22 21 20 19 18 17 16 - 8 bytes
                    uint8_t press_part_15_8 = (uint8_t)(hbmp.press_value >> 8);   // 15 14 13 12 11 10 9 8 - 8 bytes
                    uint8_t press_part_8_0 = (uint8_t)(hbmp.press_value >> 0);    // 15 14 13 12 11 10 9 8 - 8 bytes

                    // reconstruction of the uint32_t => (press_part_31_24 << 24) | (press_part_23_16 << 16) | (press_part_15_8 << 8)  | (press_part_8_0 << 0)
                    uint16_t press_id = 0x101; // 11 bits; 11-0, 15-12 are unused
                    uint8_t press_sidh = press_id >> 3;
                    uint8_t press_sidl = (press_id & 0x7) << 5;

                    // SIDH, SIDL, EID8 (zeroed out, don't care), EID0 (zeroed out, don't care), DLC, 4 data bytes

                    uint8_t press_payload[9] = {press_sidh, press_sidl, 0x00, 0x00, DLC, press_part_31_24, press_part_23_16, press_part_15_8, press_part_8_0};

                    mcp2515_load_tx_buffer(MCP_Load_TXB1SIDH, press_payload, 9);

                    mcp2515_rts(&location2, 1U);

                    // printf("start_pending_hits: %d | sb_hits: %d\r\n", hi2c.start_pending_hits, hi2c.sb_hits);

                    hbmp.request_status = BMP280_REQUEST_NONE;
                    hbmp.retries = 0;
                    hbmp.measure_start_tick = 0;
                    hbmp.measure_start_tick_status = BMP280_START_TICK_NEVER_CAPTURED;
                    // BMP start measurements
                    hbmp.state = BMP280_STATE_CTRL_MEAS;
                }
                else
                {
                    printf("The BMP280 state is not READY during the 500ms tick check. Frames sending skipped!");
                    fflush(stdout);
                }
            }
        }

        switch (hbmp.state)
        {
        case BMP280_STATE_IDLE:
            break;
        case BMP280_STATE_INIT:
            if (BMP280_Init(&hbmp, meas) != BMP280_OK)
            {
                hbmp.state = BMP280_STATE_ERROR;
            }
            break;
        case BMP280_STATE_READ_CALIBRATION:
            if (BMP280_ReadCalibration(&hbmp) != BMP280_OK)
            {
                hbmp.state = BMP280_STATE_ERROR;
            }
            break;
        case BMP280_STATE_RECONSTRUCT_CALIBRATION:
            BMP280_ReconstructCalibration(&hbmp);
            break;
        case BMP280_STATE_CTRL_MEAS:
            if (BMP280_WriteCtrlMeas(&hbmp) != BMP280_OK)
            {
                hbmp.state = BMP280_STATE_ERROR;
            }
            break;
        case BMP280_STATE_MEASURING:
            if (BMP280_Measuring(&hbmp) != BMP280_OK)
            {
                hbmp.state = BMP280_STATE_ERROR;
            }
            break;
        case BMP280_STATE_READ_MEASURAMENTS:
            if (BMP280_ReadMeasurements(&hbmp) != BMP280_OK)
            {
                hbmp.state = BMP280_STATE_ERROR;
            }
            break;
        case BMP280_STATE_RECONSTRUCT_MEASURAMENTS:
            BMP280_ReconstructMeasurements(&hbmp);
            break;
        case BMP280_STATE_COMPENSATE:
            BMP280_CalculateData(&hbmp);
            break;
        case BMP280_STATE_READY:
            break;
        case BMP280_STATE_ERROR:
            if (hbmp.retries >= 3)
            {
                // the counter is exhausted
                hbmp.state = BMP280_STATE_FAULT;
                break;
            }

            if (hbmp.hi2c->state == I2C_STATE_IDLE)
            {
                hbmp.retries++;

                // begin a transaction from the beginning (Calibration is read once at the very beginning, so omit the state)
                hbmp.state = BMP280_STATE_CTRL_MEAS;
            }

            break;
        case BMP280_STATE_FAULT:
            printf("The BMP280 sensor experienced hard fault!");
            fflush(stdout);
            break;
        }
    }

    return 0;
}