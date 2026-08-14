#/usr/bin/env python3
"""
Decoder for the Week 21 BMP280 CAN sensor node.

Frame layout (matches mcp2515.c / main.c):
    ID 0x100 - temperature, 4 data bytes, MSB first signed int32
               reconstructed value is in units of 0.01 degC (firmware does /100 for display)

    ID 0x101 - pressure, 4 data bytes, MSB first, unsigned uint32
               reconstructed value is Q24.8 Pa (firmware does /256/100 for display, i.e. hPa)

    Usage:
        python decode_bmp280.py [channel]

        channel defaults to 'can0'
"""

import sys
import can

TEMP_ID = 0x100
PRESS_ID = 0x101


def decode_temp(data: bytes) -> float:
    # bytes are MSB-first, signed 32-bit, matches firmware's >>24/>>16/>>8/>>0 packing
    raw = int.from_bytes(data, byteorder="big", signed=True)
    return raw / 100.0 # degC, matches firmware's temp_value / 100

def decode_press(data:bytes) -> float:
    # bytes are MSB-first, unsigned 32-bit, Q24.8 format
    raw = int.from_bytes(data, byteorder="big", signed=False)
    return raw / 256.0 / 100.0 # hPa, matches firmware's press_value / 256 / 100


def main():
    channel = sys.argv[1] if len(sys.argv) > 1 else "can0"

    bus = can.interface.Bus(channel=channel, bustype="socketcan")
    print(f"Listening on {channel} for IDs 0x{TEMP_ID:X} / 0x{PRESS_ID:X}... (Ctrl+C to stop)")

    try:
        for msg in bus:
            if msg.arbitration_id == TEMP_ID and msg.dlc == 4:
                temp = decode_temp(msg.data)
                print(f"[TEMP ] raw={msg.data.hex()} {temp:.2f} degC")
            elif msg.arbitration_id == PRESS_ID and msg.dlc == 4:
                press = decode_press(msg.data)
                print(f"[PRESS] raw = {msg.data.hex()} {press:.2f} hPa")
            # ignore anything else (e.g. stray RX during bring-up} silently
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
