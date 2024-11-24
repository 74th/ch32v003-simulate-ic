#include "ch32v003fun.h"
#include "i2c_slave.h"
#include "ch32v003_GPIO_branchless.h"
#include <stdio.h>
#include <stdbool.h>

#define GPB0 GPIOv_from_PORT_PIN(GPIO_port_D, 4)
#define GPB1 GPIOv_from_PORT_PIN(GPIO_port_D, 5)
#define GPB2 GPIOv_from_PORT_PIN(GPIO_port_D, 6)
#define GPB3 GPIOv_from_PORT_PIN(GPIO_port_D, 7)
#define GPB4 GPIOv_from_PORT_PIN(GPIO_port_A, 1)
#define GPB5 GPIOv_from_PORT_PIN(GPIO_port_A, 2)
#define GPB6 GPIOv_from_PORT_PIN(GPIO_port_D, 0)
#define GPB7 GPIOv_from_PORT_PIN(GPIO_port_C, 0)
#define GPA0 GPIOv_from_PORT_PIN(GPIO_port_C, 3)
#define GPA1 GPIOv_from_PORT_PIN(GPIO_port_C, 4)
#define GPA2 GPIOv_from_PORT_PIN(GPIO_port_C, 5)
#define GPA3 GPIOv_from_PORT_PIN(GPIO_port_C, 6)
#define GPA4 GPIOv_from_PORT_PIN(GPIO_port_C, 7)
// P18 SWDIC
#define GPA5 GPIOv_from_PORT_PIN(GPIO_port_D, 1)
#define GPA6 GPIOv_from_PORT_PIN(GPIO_port_D, 2)
#define GPA7 GPIOv_from_PORT_PIN(GPIO_port_D, 3)

#define MCP23017_I2C_ADDRESS 0x20
#define MCP23017_IODIR_A 0x00
#define MCP23017_IODIR_B 0x01
#define MCP23017_GPIO_A 0x12
#define MCP23017_GPIO_B 0x13
#define MCP23017_GPPU_A 0x0C
#define MCP23017_GPPU_B 0x0D

#define ENABLE_DEBUGGING 0

// volatile uint16_t gpa_pin[8] = {GPA0, GPA1, GPA2, GPA3, GPA4, GPA5, GPA6, GPA7};
volatile uint16_t gpa_pin[8] = {GPA0, GPA1, GPA2, GPA3, GPA4, GPA4, GPA6, GPA7};
volatile uint16_t gpb_pin[8] = {GPB0, GPB1, GPB2, GPB3, GPB4, GPB5, GPB6, GPB7};
volatile uint8_t i2c_registers[32] = {0x00};

void print_bits(uint8_t value)
{
	for (int i = 7; i >= 0; i--)
	{
		printf("%d", (value >> i) & 1);
	}
}

void setup()
{
	GPIO_port_enable(GPIO_port_A);
	GPIO_port_enable(GPIO_port_C);
	GPIO_port_enable(GPIO_port_D);

	funPinMode(PC1, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SDA
	funPinMode(PC2, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SCL
	SetupI2CSlave(0x9, i2c_registers, sizeof(i2c_registers), on_write, NULL, false);

	for (uint8_t i = 0; i < 32; i++)
	{
		i2c_registers[i] = 0x00;
	}
	i2c_registers[MCP23017_IODIR_A] = 0xFF;
	i2c_registers[MCP23017_IODIR_B] = 0xFF;

	Delay_Us(100);

	set_iodir(gpa_pin, i2c_registers[MCP23017_IODIR_A], i2c_registers[MCP23017_GPPU_A], i2c_registers[MCP23017_GPIO_A]);

	Delay_Us(100);
}

#define DURATION_MS 200

void set_iodir(uint16_t *gp_pin, uint8_t iodir, uint8_t gppu, uint8_t gpio)
{
#ifdef ENABLE_DEBUGGING
	print("set_iodir: iodir");
	print_bits(iodir);
	print(" gppu");
	print_bits(gppu);
	print(" gpio");
	print_bits(gpio);
	print("\r\n");
#endif
	for (uint8_t i = 0; i < 8; i++)
	{
		uint16_t pin = gp_pin[i];
		if (iodir & (1 << i))
		{
			if (iodir & (1 << i))
			{
				funPinMode(pin, GPIO_pinMode_I_pullUp);
			}
			else
			{
				funPinMode(pin, GPIO_pinMode_I_pullDown);
			}
		}
		else
		{
			funPinMode(pin, GPIO_pinMode_O_pushPull);
			if (gpio & (1 << i))
			{
				GPIO_digitalWrite_1(pin);
			}
			else
			{
				GPIO_digitalWrite_0(pin);
			}
		}
	}
}

void output_gpio(uint16_t *gp_pin, uint8_t iodir, uint8_t gpio)
{
#ifdef ENABLE_DEBUGGING
	print("output_gpio: iodir");
	print_bits(iodir);
	print(" gpio");
	print_bits(gpio);
	print("\r\n");
#endif
	for (uint8_t i = 0; i < 8; i++)
	{
		uint16_t pin = gp_pin[i];
		if (iodir & (1 << i) == 0)
		{
			if (gpio & (1 << i))
			{
				GPIO_digitalWrite_1(pin);
			}
			else
			{
				GPIO_digitalWrite_0(pin);
			}
		}
	}
}

void input_gpio(uint16_t *gp_pin, uint8_t iodir, uint8_t *gpio)
{
	uint8_t input = 0;
	for (uint8_t i = 0; i < 8; i++)
	{
		uint16_t pin = gp_pin[i];
		if (iodir & (1 << i))
		{
			if (GPIO_digitalRead(pin))
			{
				input |= (1 << i);
			}
		}
	}
	uint8_t tmp = *gpio & ~iodir;
	*gpio = tmp | input;
}

void on_write(uint8_t reg, uint8_t length)
{
	for (uint8_t p = reg; p < reg + length; p++)
	{
		switch (p)
		{
		case MCP23017_IODIR_A:
		case MCP23017_GPPU_A:
			set_iodir(gpa_pin, i2c_registers[MCP23017_IODIR_A], i2c_registers[MCP23017_GPPU_A], i2c_registers[MCP23017_GPIO_A]);
			break;
		case MCP23017_IODIR_B:
		case MCP23017_GPPU_B:
			set_iodir(gpa_pin, i2c_registers[MCP23017_IODIR_B], i2c_registers[MCP23017_GPPU_B], i2c_registers[MCP23017_GPIO_B]);
			break;
		case MCP23017_GPIO_A:
			output_gpio(gpa_pin, i2c_registers[MCP23017_IODIR_A], i2c_registers[MCP23017_GPIO_A]);
			break;
		case MCP23017_GPIO_B:
			output_gpio(gpa_pin, i2c_registers[MCP23017_IODIR_B], &i2c_registers[MCP23017_GPIO_B]);
			break;
		}
	}
}

void main_loop()
{
	if (i2c_registers[MCP23017_IODIR_A])
	{
		input_gpio(gpa_pin, i2c_registers[MCP23017_IODIR_A], &i2c_registers[MCP23017_GPIO_A]);
	}
	if (i2c_registers[MCP23017_IODIR_B])
	{
		input_gpio(gpa_pin, i2c_registers[MCP23017_IODIR_B], &i2c_registers[MCP23017_GPIO_B]);
	}
}

int main()
{
	SystemInit();
	funGpioInitAll();

	setup();

	for (;;)
		main_loop();
}
