#include "ch32v003fun.h"
#include "ch32v003_GPIO_branchless.h"
#include <stdio.h>
#include <stdbool.h>

#define GPA0 GPIOv_from_PORT_PIN(GPIO_port_C, 3)
#define GPA1 GPIOv_from_PORT_PIN(GPIO_port_C, 4)
#define GPA2 GPIOv_from_PORT_PIN(GPIO_port_C, 5)
#define GPA3 GPIOv_from_PORT_PIN(GPIO_port_C, 6)
#define GPA4 GPIOv_from_PORT_PIN(GPIO_port_C, 7)
// P18 SWDIC
#define GPA5 GPIOv_from_PORT_PIN(GPIO_port_D, 1)
#define GPA6 GPIOv_from_PORT_PIN(GPIO_port_D, 2)
#define GPA7 GPIOv_from_PORT_PIN(GPIO_port_D, 3)

#define GPB0 GPIOv_from_PORT_PIN(GPIO_port_D, 4)
#define GPB1 GPIOv_from_PORT_PIN(GPIO_port_D, 5)
#define GPB2 GPIOv_from_PORT_PIN(GPIO_port_D, 6)
#define GPB3 GPIOv_from_PORT_PIN(GPIO_port_D, 7)
#define GPB4 GPIOv_from_PORT_PIN(GPIO_port_A, 1)
#define GPB5 GPIOv_from_PORT_PIN(GPIO_port_A, 2)
#define GPB6 GPIOv_from_PORT_PIN(GPIO_port_D, 0)
#define GPB7 GPIOv_from_PORT_PIN(GPIO_port_C, 0)

#define MCP23017_I2C_ADDRESS 0x20
#define MCP23017_IODIR_A 0x00
#define MCP23017_IODIR_B 0x01
#define MCP23017_GPIO_A 0x12
#define MCP23017_GPIO_B 0x13
#define MCP23017_GPPU_A 0x0C
#define MCP23017_GPPU_B 0x0D

// #define ENABLE_DEBUGGING 0

volatile uint16_t gpa_pin[8] = {GPA0, GPA1, GPA2, GPA3, GPA4, GPA5, GPA6, GPA7};
volatile uint16_t gpb_pin[8] = {GPB0, GPB1, GPB2, GPB3, GPB4, GPB5, GPB6, GPB7};
// volatile uint16_t gpa_pin[8] = {GPA0, GPA1, GPA2, GPA3, GPA4, GPA4, GPA6, GPA7}; // SWDIO回避
// volatile uint16_t gpb_pin[8] = {GPB0, GPB1, GPB2, GPB3, GPB4, GPB5, GPB6, GPB7}; // NRST回避
volatile uint8_t i2c_registers[32] = {0x00};

typedef struct
{
	uint8_t reg;
	uint8_t length;
} I2CMosiEvent;

void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void) __attribute__((interrupt));

void init_rcc(void)
{
	RCC->CFGR0 &= ~(0x1F << 11);

	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO;
	RCC->APB1PCENR |= RCC_APB1Periph_I2C1;
}

void init_i2c_slave(uint8_t address)
{
	// https://github.com/cnlohr/ch32v003fun/blob/master/examples/i2c_slave/i2c_slave.h

	// PC1 is SDA, 10MHz Output, alt func, open-drain
	GPIOC->CFGLR &= ~(0xf << (4 * 1));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * 1);

	// PC2 is SCL, 10MHz Output, alt func, open-drain
	GPIOC->CFGLR &= ~(0xf << (4 * 2));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * 2);

	// Reset I2C1 to init all regs
	RCC->APB1PRSTR |= RCC_APB1Periph_I2C1;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C1;

	// disable SWD
	// AFIO->PCFR1 |= AFIO_PCFR1_SWJ_CFG_DISABLE;

	I2C1->CTLR1 |= I2C_CTLR1_SWRST;
	I2C1->CTLR1 &= ~I2C_CTLR1_SWRST;

	// Set module clock frequency
	uint32_t prerate = 2000000; // I2C Logic clock rate, must be higher than the bus clock rate
	I2C1->CTLR2 |= (FUNCONF_SYSTEM_CORE_CLOCK / prerate) & I2C_CTLR2_FREQ;

	// Enable interrupts
	I2C1->CTLR2 |= I2C_CTLR2_ITBUFEN;
	I2C1->CTLR2 |= I2C_CTLR2_ITEVTEN; // Event interrupt
	I2C1->CTLR2 |= I2C_CTLR2_ITERREN; // Error interrupt

	NVIC_EnableIRQ(I2C1_EV_IRQn); // Event interrupt
	NVIC_SetPriority(I2C1_EV_IRQn, 2 << 4);
	NVIC_EnableIRQ(I2C1_ER_IRQn); // Error interrupt
	// Set clock configuration
	uint32_t clockrate = 1000000;																	 // I2C Bus clock rate, must be lower than the logic clock rate
	I2C1->CKCFGR = ((FUNCONF_SYSTEM_CORE_CLOCK / (3 * clockrate)) & I2C_CKCFGR_CCR) | I2C_CKCFGR_FS; // Fast mode 33% duty cycle
	// I2C1->CKCFGR = ((APB_CLOCK/(25*clockrate))&I2C_CKCFGR_CCR) | I2C_CKCFGR_DUTY | I2C_CKCFGR_FS; // Fast mode 36% duty cycle
	// I2C1->CKCFGR = (APB_CLOCK/(2*clockrate))&I2C_CKCFGR_CCR; // Standard mode good to 100kHz

	// Set I2C address
	I2C1->OADDR1 = address << 1;

	// Enable I2C
	I2C1->CTLR1 |= I2C_CTLR1_PE;

	// Acknowledge the first address match event when it happens
	I2C1->CTLR1 |= I2C_CTLR1_ACK;
}

uint16_t i2c_scan_start_position = 0;
uint16_t i2c_scan_length = 0;
bool i2c_firse_byte = true;
bool i2c_mosi_event = false;
bool on_mosi_event = false;
I2CMosiEvent latest_mosi_event = {0, 0};
bool address2matched = false;

void I2C1_EV_IRQHandler(void)
{
	uint16_t STAR1, STAR2 __attribute__((unused));
	STAR1 = I2C1->STAR1;
	STAR2 = I2C1->STAR2;

#ifdef FUNCONF_USE_UARTPRINTF
	printf("EV STAR1: 0x%04x STAR2: 0x%04x\r\n", STAR1, STAR2);
#endif

	I2C1->CTLR1 |= I2C_CTLR1_ACK;

	if (STAR1 & I2C_STAR1_ADDR) // 0x0002
	{
		// 最初のイベント
		// read でも write でも必ず最初に呼ばれる
		address2matched = !!(STAR2 & I2C_STAR2_DUALF);
		// printf("ADDR %d\r\n", address2matched);
		i2c_firse_byte = true;
		i2c_mosi_event = false;
		i2c_scan_start_position = 0;
		i2c_scan_length = 0;
	}

	if (STAR1 & I2C_STAR1_RXNE) // 0x0040
	{
		if (i2c_firse_byte)
		{
			i2c_scan_start_position = I2C1->DATAR;
			i2c_scan_length = 0;
			i2c_firse_byte = false;
			i2c_mosi_event = true;
			// printf("RXNE write event: first_write 0x%x\r\n", i2c_scan_start_position);
		}
		else
		{
			uint16_t pos = i2c_scan_start_position + i2c_scan_length;
			uint8_t v = I2C1->DATAR;
			// printf("RXNE write event: pos:0x%x v:0x%x\r\n", pos, v);
			if (pos < 32)
			{
				i2c_registers[pos] = v;
				i2c_scan_length++;
			}
		}
	}

	if (STAR1 & I2C_STAR1_TXE) // 0x0080
	{
		if (i2c_firse_byte)
		{
			i2c_scan_start_position = I2C1->DATAR;
			i2c_scan_length = 0;
			i2c_firse_byte = false;
			i2c_mosi_event = true;
			// printf("TXE write event: first_write:%x\r\n", i2c_scan_start_position);
		}
		else
		{
			// 1byte の read イベント（slave -> master）
			// 1byte 送信
			uint16_t pos = i2c_scan_start_position + i2c_scan_length;
			uint8_t v = 0x00;
			if (pos < 32)
			{
				v = i2c_registers[pos];
			}
			// printf("TXE write event: pos:0x%x v:0x%x\r\n", pos, v);
			I2C1->DATAR = v;
			i2c_scan_length++;
		}
	}

	if (STAR1 & I2C_STAR1_STOPF)
	{
		I2C1->CTLR1 &= ~(I2C_CTLR1_STOP);
		if (i2c_mosi_event)
		{
			// printf("on_mosi_event\r\n");
			latest_mosi_event.reg = i2c_scan_start_position;
			latest_mosi_event.length = i2c_scan_length;
			on_mosi_event = true;
		}
	}
}

void I2C1_ER_IRQHandler(void)
{
	uint16_t STAR1 = I2C1->STAR1;

#ifdef FUNCONF_USE_UARTPRINTF
	printf("ER STAR1: 0x%04x\r\n", STAR1);
#endif

	if (STAR1 & I2C_STAR1_BERR)			  // 0x0100
	{									  // Bus error
		I2C1->STAR1 &= ~(I2C_STAR1_BERR); // Clear error
	}

	if (STAR1 & I2C_STAR1_ARLO)			  // 0x0200
	{									  // Arbitration lost error
		I2C1->STAR1 &= ~(I2C_STAR1_ARLO); // Clear error
	}

	if (STAR1 & I2C_STAR1_AF)			// 0x0400
	{									// Acknowledge failure
		I2C1->STAR1 &= ~(I2C_STAR1_AF); // Clear error
	}
}

void print_bits(uint8_t value)
{
	for (int i = 7; i >= 0; i--)
	{
		printf("%d", (value >> i) & 1);
	}
}

#define DURATION_MS 200

void set_iodir(uint16_t *gp_pin, uint8_t iodir, uint8_t gppu, uint8_t gpio)
{
#ifdef ENABLE_DEBUGGING
	printf("set_iodir ");
	if (gp_pin == gpa_pin)
	{
		printf("a");
	}
	else
	{
		printf("b");
	}
	printf(": iodir");
	print_bits(iodir);
	printf(" gppu");
	print_bits(gppu);
	printf(" gpio");
	print_bits(gpio);
	printf("\r\n");
#endif
	for (uint8_t i = 0; i < 8; i++)
	{
		uint16_t pin = gp_pin[i];
		if (iodir & (1 << i))
		{
			if (iodir & (1 << i))
			{
				GPIO_pinMode(pin, GPIO_pinMode_I_pullUp, GPIO_Speed_10MHz);
			}
			else
			{
				GPIO_pinMode(pin, GPIO_pinMode_I_pullUDown, GPIO_Speed_10MHz);
			}
		}
		else
		{
			GPIO_pinMode(pin, GPIO_pinMode_O_pushPull, GPIO_Speed_10MHz);
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
	printf("output_gpio ");
	if (gp_pin == gpa_pin)
	{
		printf("a");
	}
	else
	{
		printf("b");
	}
	printf(": iodir");
	print_bits(iodir);
	printf(" gpio");
	print_bits(gpio);
	printf("\r\n");
#endif
	for (uint8_t i = 0; i < 8; i++)
	{
		uint16_t pin = gp_pin[i];
		if ((iodir & (1 << i)) == 0)
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
			set_iodir(gpb_pin, i2c_registers[MCP23017_IODIR_B], i2c_registers[MCP23017_GPPU_B], i2c_registers[MCP23017_GPIO_B]);
			break;
		case MCP23017_GPIO_A:
			output_gpio(gpa_pin, i2c_registers[MCP23017_IODIR_A], i2c_registers[MCP23017_GPIO_A]);
			break;
		case MCP23017_GPIO_B:
			output_gpio(gpb_pin, i2c_registers[MCP23017_IODIR_B], i2c_registers[MCP23017_GPIO_B]);
			break;
		}
	}
}

void setup()
{
	GPIO_port_enable(GPIO_port_A);
	GPIO_port_enable(GPIO_port_C);
	GPIO_port_enable(GPIO_port_D);

	init_rcc();
	init_i2c_slave(MCP23017_I2C_ADDRESS);

	// funPinMode(PC1, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SDA
	// funPinMode(PC2, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SCL
	// SetupI2CSlave(MCP23017_I2C_ADDRESS, i2c_registers, sizeof(i2c_registers), on_write, NULL, false);

	for (uint8_t i = 0; i < 32; i++)
	{
		i2c_registers[i] = 0x00;
	}
	i2c_registers[MCP23017_IODIR_A] = 0xFF;
	i2c_registers[MCP23017_IODIR_B] = 0xFF;

	Delay_Ms(1);

	set_iodir(gpa_pin, i2c_registers[MCP23017_IODIR_A], i2c_registers[MCP23017_GPPU_A], i2c_registers[MCP23017_GPIO_A]);
	set_iodir(gpb_pin, i2c_registers[MCP23017_IODIR_B], i2c_registers[MCP23017_GPPU_B], i2c_registers[MCP23017_GPIO_B]);

	Delay_Ms(500);
}

void main_loop()
{
	if (i2c_registers[MCP23017_IODIR_A])
	{
		input_gpio(gpa_pin, i2c_registers[MCP23017_IODIR_A], &i2c_registers[MCP23017_GPIO_A]);
	}
	if (i2c_registers[MCP23017_IODIR_B])
	{
		input_gpio(gpb_pin, i2c_registers[MCP23017_IODIR_B], &i2c_registers[MCP23017_GPIO_B]);
	}
	if (on_mosi_event)
	{
		// printf("do_mosi_event\r\n");
		on_mosi_event = false;
		on_write(latest_mosi_event.reg, latest_mosi_event.length);
		// printf("done_mosi_event\r\n");
	}
}

int main()
{
	SystemInit();
	funGpioInitAll();

	setup();
	// printf("Start!");

	for (;;)
		main_loop();
}
