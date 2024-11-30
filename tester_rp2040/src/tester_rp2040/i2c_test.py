import time

import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice


def main() -> None:
    with busio.I2C(board.GP5, board.GP4) as i2c:  # type: ignore[call-arg]
        device = I2CDevice(i2c, 0x20)

        with device:  # type: ignore[call-arg]
            print("write 1")
            device.write(bytes([0x12, 0xF0, 0xD0]))
            time.sleep(0.1)

            buf = bytearray(2)
            device.write_then_readinto(bytes([0x12]), buf)
            print("buf: ", buf)
            time.sleep(0.1)

        with device:  # type: ignore[call-arg]
            print("write 2")
            device.write(bytes([0x12, 0x0F, 0x0D]))
            time.sleep(0.1)
            device.write_then_readinto(bytes([0x12]), buf)
            print("buf: ", buf)
            time.sleep(0.1)

        with device:  # type: ignore[call-arg]
            print("write 3")
            device.write(bytes([0x12, 0xBB, 0xAA]))
            time.sleep(0.1)
            device.write_then_readinto(bytes([0x12]), buf)
            print("buf: ", buf)


main()
