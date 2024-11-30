import time

import board
import busio
import digitalio
from adafruit_bus_device.i2c_device import I2CDevice

gpa_pins = [
    board.GP16,
    board.GP17,
    board.GP18,
    board.GP19,
    board.GP20,
    board.GP21,
    board.GP22,
    board.GP26,
]
gpb_pins = [
    board.GP8,
    board.GP9,
    board.GP10,
    board.GP11,
    board.GP12,
    board.GP13,
    board.GP14,
    board.GP15,
]


def get_pin_value(
    a_ios: list[digitalio.DigitalInOut], b_ios: list[digitalio.DigitalInOut]
) -> tuple[int, int]:
    a_value = 0
    b_value = 0
    for i, p in enumerate(a_ios):
        if p.value:
            a_value |= 1 << i
    for i, p in enumerate(b_ios):
        if p.value:
            b_value |= 1 << i
    return (a_value, b_value)


def validate_pin_value(
    a_value: int,
    b_value: int,
    expect_a: int,
    expect_b: int,
) -> None:
    if a_value != expect_a or b_value != expect_b:
        print(f"Expect GPA: {expect_a:08b}, GPB: {expect_b:08b}")
        print(f"   got GPA: {a_value:08b}, GPB: {b_value:08b}")
    else:
        print("OK")


def test_all_write(
    device: I2CDevice,
    a_ios: list[digitalio.DigitalInOut],
    b_ios: list[digitalio.DigitalInOut],
) -> None:
    for p in a_ios:
        p.switch_to_input()
    for p in b_ios:
        p.switch_to_input()

    with device:  # type: ignore[call-arg]
        # 全てWriteにする
        device.write(bytes([0x00, 0x00, 0x00]))
        time.sleep(0.1)

        device.write(bytes([0x12, 0b01010101, 0b01010101]))
        time.sleep(0.1)
        a_value, b_value = get_pin_value(a_ios, b_ios)
        validate_pin_value(a_value, b_value, 0b01010101, 0b01010101)

        device.write(bytes([0x12, 0b11110000, 0b11110000]))
        time.sleep(0.1)
        a_value, b_value = get_pin_value(a_ios, b_ios)
        validate_pin_value(a_value, b_value, 0b11110000, 0b11110000)


def test_all_read(
    device: I2CDevice,
    gpa_ios: list[digitalio.DigitalInOut],
    gpb_ios: list[digitalio.DigitalInOut],
) -> None:
    for p in gpa_ios:
        p.switch_to_output()
    for p in gpb_ios:
        p.switch_to_output()

    with device:  # type: ignore[call-arg]
        # 全てReadにする
        device.write(bytes([0x00, 0xFF, 0xFF]))
        time.sleep(0.1)

        for i, p in enumerate(gpa_ios):
            p.value = i % 2 == 0
        for i, p in enumerate(gpb_ios):
            p.value = i % 2 == 0
        time.sleep(0.1)

        buf = bytearray(2)
        device.write_then_readinto(bytes([0x12]), buf)
        time.sleep(0.1)
        validate_pin_value(buf[0], buf[1], 0b01010101, 0b01010101)

        for i, p in enumerate(gpa_ios):
            p.value = i > 3
        for i, p in enumerate(gpb_ios):
            p.value = i > 3
        time.sleep(0.1)

        buf = bytearray(2)
        device.write_then_readinto(bytes([0x12]), buf)
        time.sleep(0.1)
        validate_pin_value(buf[0], buf[1], 0b11110000, 0b11110000)


def test_complex(
    device: I2CDevice,
    gpa_ios: list[digitalio.DigitalInOut],
    gpb_ios: list[digitalio.DigitalInOut],
) -> None:
    read_gpa = [gpa_ios[0], gpa_ios[1], gpa_ios[2], gpa_ios[3]]
    write_gpa = [gpa_ios[4], gpa_ios[5], gpa_ios[6], gpa_ios[7]]
    read_gpb = [gpb_ios[0], gpb_ios[1], gpb_ios[2], gpb_ios[3]]
    write_gpb = [gpb_ios[4], gpb_ios[5], gpb_ios[6], gpb_ios[7]]

    for p in write_gpa:
        p.switch_to_input()
    for p in write_gpb:
        p.switch_to_input()
    for p in read_gpa:
        p.switch_to_output()
    for p in read_gpb:
        p.switch_to_output()

    with device:  # type: ignore[call-arg]
        # 半分Readにする
        device.write(bytes([0x00, 0x0F, 0x0F]))
        time.sleep(0.1)

        for i, p in enumerate(read_gpa):
            p.value = i % 2 == 0
        for i, p in enumerate(read_gpb):
            p.value = i % 2 == 0

        time.sleep(0.1)

        device.write(bytes([0x12, 0b01010000, 0b01010000]))

        buf = bytearray(2)
        device.write_then_readinto(bytes([0x12]), buf)
        a_value, b_value = get_pin_value(write_gpa, write_gpb)

        validate_pin_value(a_value, b_value, 0b0101, 0b0101)
        validate_pin_value(buf[0] & 0x0F, buf[1] & 0x0F, 0b0101, 0b0101)


def main() -> None:
    gpa_ios = [digitalio.DigitalInOut(pin) for pin in gpa_pins]
    gpb_ios = [digitalio.DigitalInOut(pin) for pin in gpb_pins]
    for p in gpa_ios:
        p.switch_to_input()
    for p in gpb_ios:
        p.switch_to_input()

    with busio.I2C(board.GP5, board.GP4) as i2c:  # type: ignore[call-arg]
        device = I2CDevice(i2c, 0x20)

        print("write test")
        test_all_write(device, gpa_ios, gpb_ios)

        print("read test")
        test_all_read(device, gpa_ios, gpb_ios)

        print("complex test")
        test_complex(device, gpa_ios, gpb_ios)


main()
