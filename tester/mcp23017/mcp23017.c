#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MCP23017_I2C_ADDRESS 0x20
#define MCP23017_IODIR_A 0x00
#define MCP23017_IODIR_B 0x01
#define MCP23017_GPIO_A 0x12
#define MCP23017_GPIO_B 0x13
#define MCP23017_GPPU_A 0x0C
#define MCP23017_GPPU_B 0x0D

#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

#define SLEEP_MS 200

uint8_t GPIOA_PINS[8] = {16, 17, 18, 19, 20, 21, 22, 26};
uint8_t GPIOB_PINS[8] = {8, 9, 10, 11, 12, 13, 14, 15};

uint8_t get_pin_value(uint8_t *pins, int num)
{
    uint8_t r = 0;
    for (int i = 0; i < num; i++)
    {
        bool v = gpio_get(pins[i]);
        r |= v << i;
    }
    return r;
}

void set_pin_value(uint8_t *pins, uint8_t value, int num)
{
    for (int i = 0; i < num; i++)
    {
        gpio_put(pins[i], (value >> i) & 1);
    }
}

void print_bits(uint8_t v)
{
    printf("0b");
    for (int i = 0; i < 8; i++)
    {
        printf("%d", (v >> (7 - i)) & 1);
    }
}

void validate_pin_value(uint8_t a_value, uint8_t b_value, uint8_t expect_a, uint8_t expect_b)
{
    printf("Expect GPA:");
    print_bits(expect_a);
    printf(" GPB:");
    print_bits(expect_b);
    printf("\r\n   got GPA:");
    print_bits(a_value);
    printf(" GPB:");
    print_bits(b_value);
    printf("\r\n");

    if (a_value == expect_a && b_value == expect_b)
    {
        printf("OK\r\n");
    }
    else
    {
        printf("NG\r\n");
    }
}

void test_write_only()
{
    printf("write 1\n");

    for (int i = 0; i < 8; i++)
    {
        gpio_set_dir(GPIOA_PINS[i], GPIO_IN);
        gpio_set_dir(GPIOB_PINS[i], GPIO_IN);
    }

    uint8_t buf[3];
    buf[0] = MCP23017_IODIR_A;
    buf[1] = 0b00000000;
    buf[2] = 0b00000000;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 3, false);
    sleep_ms(SLEEP_MS);

    buf[0] = MCP23017_GPIO_A;
    buf[1] = 0b01010101;
    buf[2] = 0b10101010;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 3, false);
    sleep_ms(SLEEP_MS);

    uint8_t va = get_pin_value(GPIOA_PINS, 8);
    uint8_t vb = get_pin_value(GPIOB_PINS, 8);

    validate_pin_value(va, vb, buf[1], buf[2]);

    uint8_t rbuf[2];
    buf[0] = MCP23017_GPIO_A;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 1, false);
    i2c_read_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, rbuf, 2, false);

    printf("write 2\n");
    buf[0] = MCP23017_GPIO_A;
    buf[1] = 0b11110000;
    buf[2] = 0b00001111;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 3, false);
    sleep_ms(SLEEP_MS);

    va = get_pin_value(GPIOA_PINS, 8);
    vb = get_pin_value(GPIOB_PINS, 8);

    validate_pin_value(va, vb, buf[1], buf[2]);

    rbuf[2];
    buf[0] = MCP23017_GPIO_A;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 1, false);
    i2c_read_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, rbuf, 2, false);
}

void test_read_only()
{
    for (int i = 0; i < 8; i++)
    {
        gpio_set_dir(GPIOA_PINS[i], GPIO_OUT);
        gpio_set_dir(GPIOB_PINS[i], GPIO_OUT);
    }
    printf("read 1\n");
    uint8_t buf[3];
    uint8_t rbuf[2];
    buf[0] = MCP23017_IODIR_A;
    buf[1] = 0xff;
    buf[2] = 0xff;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 3, false);
    sleep_ms(SLEEP_MS);

    set_pin_value(GPIOA_PINS, 0b01010101, 8);
    set_pin_value(GPIOB_PINS, 0b10101010, 8);
    sleep_ms(SLEEP_MS);

    buf[0] = MCP23017_GPIO_A;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 1, false);
    i2c_read_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, rbuf, 2, false);
    validate_pin_value(rbuf[0], rbuf[1], 0b01010101, 0b10101010);

    printf("read 2\n");
    set_pin_value(GPIOA_PINS, 0b11110000, 8);
    set_pin_value(GPIOB_PINS, 0b00001111, 8);
    sleep_ms(SLEEP_MS);

    buf[0] = MCP23017_GPIO_A;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 1, false);
    i2c_read_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, rbuf, 2, false);
    validate_pin_value(rbuf[0], rbuf[1], 0b11110000, 0b00001111);
}

void test_mixed()
{
    printf("mixed 1\n");
    for (int i = 0; i < 4; i++)
    {
        gpio_set_dir(GPIOA_PINS[i], GPIO_IN);
        gpio_set_dir(GPIOB_PINS[i], GPIO_IN);
    }

    for (int i = 4; i < 8; i++)
    {
        gpio_set_dir(GPIOA_PINS[i], GPIO_OUT);
        gpio_set_dir(GPIOB_PINS[i], GPIO_OUT);
    }
    sleep_ms(SLEEP_MS);

    uint8_t buf[3];
    uint8_t rbuf[2];
    buf[0] = MCP23017_IODIR_A;
    buf[1] = 0xf0;
    buf[2] = 0xf0;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 3, false);
    sleep_ms(SLEEP_MS);

    buf[0] = MCP23017_GPIO_A;
    buf[1] = 0b000000101;
    buf[2] = 0b000001010;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 3, false);

    set_pin_value(&GPIOA_PINS[4], 0b0101, 4);
    set_pin_value(&GPIOB_PINS[4], 0b1010, 4);
    sleep_ms(SLEEP_MS);

    uint8_t va = get_pin_value(GPIOA_PINS, 4);
    uint8_t vb = get_pin_value(GPIOB_PINS, 4);
    validate_pin_value(va, vb, 0b0101, 0b1010);

    buf[0] = MCP23017_GPIO_A;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 1, false);
    i2c_read_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, rbuf, 2, false);
    validate_pin_value(rbuf[0] & 0xf0, rbuf[1] & 0xf0, 0b01010000, 0b10100000);
    sleep_ms(SLEEP_MS);

    printf("mixed 2\n");
    for (int i = 0; i < 4; i++)
    {
        gpio_set_dir(GPIOA_PINS[i], GPIO_OUT);
        gpio_set_dir(GPIOB_PINS[i], GPIO_OUT);
    }
    for (int i = 4; i < 8; i++)
    {
        gpio_set_dir(GPIOA_PINS[i], GPIO_IN);
        gpio_set_dir(GPIOB_PINS[i], GPIO_IN);
    }

    sleep_ms(SLEEP_MS);
    buf[0] = MCP23017_IODIR_A;
    buf[1] = 0x0f;
    buf[2] = 0x0f;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 3, false);
    sleep_ms(SLEEP_MS);

    buf[0] = MCP23017_GPIO_A;
    buf[1] = 0b10100000;
    buf[2] = 0b01010000;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 3, false);

    set_pin_value(GPIOA_PINS, 0b1010, 4);
    set_pin_value(GPIOB_PINS, 0b0101, 4);
    sleep_ms(SLEEP_MS);

    va = get_pin_value(&GPIOA_PINS[4], 4);
    vb = get_pin_value(&GPIOB_PINS[4], 4);
    validate_pin_value(va << 4, vb << 4, 0b10100000, 0b01010000);

    buf[0] = MCP23017_GPIO_A;
    i2c_write_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, buf, 1, false);
    i2c_read_blocking(I2C_PORT, MCP23017_I2C_ADDRESS, rbuf, 2, false);
    validate_pin_value(rbuf[0] & 0x0f, rbuf[1] & 0x0f, 0b00001010, 0b00000101);
}

void init()
{
    stdio_init_all();

    printf("init start\n");

    i2c_init(I2C_PORT, 100 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    for (int i = 0; i < 8; i++)
    {
        gpio_init(GPIOA_PINS[i]);
        gpio_init(GPIOB_PINS[i]);
        gpio_set_dir(GPIOA_PINS[i], GPIO_IN);
        gpio_set_dir(GPIOB_PINS[i], GPIO_IN);
    }

    printf("init done\n");
    sleep_ms(1000);
}

int main()
{
    init();

    test_write_only();

    test_read_only();

    test_mixed();

    while (true)
    {
        printf(".");
        sleep_ms(1000);
    }
}
