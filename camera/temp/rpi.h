/* rpi.h

   C and C++ support for Broadcom BCM 2835 as used in Raspberry Pi
   Author: Mike McCauley
   Copyright (C) 2011-2013 Mike McCauley
   $Id: rpi.h,v 1.20 2015/03/31 04:55:41 mikem Exp mikem $

   Only support version >= RPi2
   Modified by Xipeng Wang
   Copyright (C) 2016
*/

#ifndef _RPI_H
#define _RPI_H

#include <stdint.h>
#include "mem_op.h"

#define RPI_VERSION 10000 /* Version 1.00 */

/*! This means pin HIGH, true, 3.3volts on a pin. */
#define HIGH 0x1
/*! This means pin LOW, false, 0volts on a pin. */
#define LOW  0x0

/*! Speed of the core clock core_clk */
#define RPI_CORE_CLK_HZ		250000000	/*!< 250 MHz */

/*! On RPi2 with BCM2836, and all recent OSs, the base of the peripherals is read from a /proc file */
#define BCM2835_RPI2_DT_FILENAME "/proc/device-tree/soc/ranges"
/*! Offset into BMC2835_RPI2_DT_FILENAME for the peripherals base address */
#define BCM2835_RPI2_DT_PERI_BASE_ADDRESS_OFFSET 4
/*! Offset into BMC2835_RPI2_DT_FILENAME for the peripherals size address */
#define BCM2835_RPI2_DT_PERI_SIZE_OFFSET 8

/*! Peripherals block base address on RPi. However, this may be changed based on info in BCM2835_RPI2_DT_FILENAME */
#define RPI_PERI_BASE               0x20000000
/*! Size of the peripherals block on RPi */
#define RPI_PERI_SIZE               0x01000000

/*! Offsets for the bases of various peripherals within the peripherals block */

/*!   Base Address of the System Timer registers */
#define RPI_ST_BASE			0x3000

/*! Base Address of the interrupt registers */
#define RPI_INTERRUPT_BASE  0xB000

/*! Base Address of the Pads registers */
#define RPI_GPIO_PADS               0x100000
/*! Base Address of the Clock/timer registers */
#define RPI_CLOCK_BASE              0x101000
/*! Base Address of the GPIO registers */
#define RPI_GPIO_BASE               0x200000
/*! Base Address of the SPI0 registers */
#define RPI_SPI0_BASE               0x204000
/*! Base Address of the BSC0 registers */
#define RPI_BSC0_BASE 		0x205000
/*! Base Address of the PWM registers */
#define RPI_GPIO_PWM                0x20C000
/*! Base Address of the BSC1 registers */
#define RPI_BSC1_BASE		0x804000
/*! Base Address of the BSC2 registers */
#define RPI_BSC2_BASE		0x805000

/**----------------- TODO:

 */

#define BCM2835_INTC_TOTAL_IRQ		64 + 8

//The base address for the ARM interrupt register is 0x7E00B000.
// 0x200 IRQ basic pending

#define BCM2835_IRQ_ID_AUX			29
#define BCM2835_IRQ_ID_SPI_SLAVE 	43
#define BCM2835_IRQ_ID_PWA0			45
#define BCM2835_IRQ_ID_PWA1		   	46
#define BCM2835_IRQ_ID_SMI 			48
#define BCM2835_IRQ_ID_GPIO_0 		49
#define BCM2835_IRQ_ID_GPIO_1 		50
#define BCM2835_IRQ_ID_GPIO_2 		51
#define BCM2835_IRQ_ID_GPIO_3 		52
#define BCM2835_IRQ_ID_I2C 			53
#define BCM2835_IRQ_ID_SPI 			54
#define BCM2835_IRQ_ID_PCM 			55
#define BCM2835_IRQ_ID_UART 		57


#define BCM2835_IRQ_ID_TIMER_0 		64
#define BCM2835_IRQ_ID_MAILBOX_0	65
#define BCM2835_IRQ_ID_DOORBELL_0 	66
#define BCM2835_IRQ_ID_DOORBELL_1 	67
#define BCM2835_IRQ_ID_GPU0_HALTED 	68



/*! Physical address and size of the peripherals block */
uint32_t *rpi_peripherals_base;

/*! Size of the peripherals block to be mapped */
uint32_t rpi_peripherals_size;

/*! Virtual memory address of the mapped peripherals block */
uint32_t *rpi_peripherals;

/*! Base of the ST (System Timer) registers.
  Available after rpi_init has been called (as root)
*/
volatile uint32_t *rpi_st;

/*! Base of the GPIO registers.
  Available after rpi_init has been called
*/
volatile uint32_t *rpi_gpio;

/*! Base of the PWM registers.
  Available after rpi_init has been called (as root)
*/
volatile uint32_t *rpi_pwm;

/*! Base of the CLK registers.
  Available after rpi_init has been called (as root)
*/
volatile uint32_t *rpi_clk;

/*! Base of the PADS registers.
  Available after rpi_init has been called (as root)
*/
volatile uint32_t *rpi_pads;

/*! Base of the SPI0 registers.
  Available after rpi_init has been called (as root)
*/
volatile uint32_t *rpi_spi0;

/*! Base of the BSC1 registers.
  Available after rpi_init has been called (as root)
*/
volatile uint32_t *rpi_bsc1;

typedef enum
{
    RPI_REGBASE_ST   = 1, /*!< Base of the ST (System Timer) registers. */
    RPI_REGBASE_GPIO = 2, /*!< Base of the GPIO registers. */
    RPI_REGBASE_PWM  = 3, /*!< Base of the PWM registers. */
    RPI_REGBASE_CLK  = 4, /*!< Base of the CLK registers. */
    RPI_REGBASE_PADS = 5, /*!< Base of the PADS registers. */
    RPI_REGBASE_SPI0 = 6, /*!< Base of the SPI0 registers. */
    RPI_REGBASE_BSC1 = 7, /*!< Base of the BSC0 registers. */
} rpiRegisterBase;

/* Defines for GPIO
   The RPI has 54 GPIO pins.
   RPI data sheet, Page 90 onwards.
*/
/*! GPIO register offsets from RPI_GPIO_BASE.
  Offsets into the GPIO Peripheral block in bytes per 6.1 Register View
*/
#define RPI_GPFSEL0                      0x0000 /*!< GPIO Function Select 0 */
#define RPI_GPFSEL1                      0x0004 /*!< GPIO Function Select 1 */
#define RPI_GPFSEL2                      0x0008 /*!< GPIO Function Select 2 */
#define RPI_GPFSEL3                      0x000c /*!< GPIO Function Select 3 */
#define RPI_GPFSEL4                      0x0010 /*!< GPIO Function Select 4 */
#define RPI_GPFSEL5                      0x0014 /*!< GPIO Function Select 5 */
#define RPI_GPSET0                       0x001c /*!< GPIO Pin Output Set 0 */
#define RPI_GPSET1                       0x0020 /*!< GPIO Pin Output Set 1 */
#define RPI_GPCLR0                       0x0028 /*!< GPIO Pin Output Clear 0 */
#define RPI_GPCLR1                       0x002c /*!< GPIO Pin Output Clear 1 */
#define RPI_GPLEV0                       0x0034 /*!< GPIO Pin Level 0 */
#define RPI_GPLEV1                       0x0038 /*!< GPIO Pin Level 1 */
#define RPI_GPEDS0                       0x0040 /*!< GPIO Pin Event Detect Status 0 */
#define RPI_GPEDS1                       0x0044 /*!< GPIO Pin Event Detect Status 1 */
#define RPI_GPREN0                       0x004c /*!< GPIO Pin Rising Edge Detect Enable 0 */
#define RPI_GPREN1                       0x0050 /*!< GPIO Pin Rising Edge Detect Enable 1 */
#define RPI_GPFEN0                       0x0058 /*!< GPIO Pin Falling Edge Detect Enable 0 */
#define RPI_GPFEN1                       0x005c /*!< GPIO Pin Falling Edge Detect Enable 1 */
#define RPI_GPHEN0                       0x0064 /*!< GPIO Pin High Detect Enable 0 */
#define RPI_GPHEN1                       0x0068 /*!< GPIO Pin High Detect Enable 1 */
#define RPI_GPLEN0                       0x0070 /*!< GPIO Pin Low Detect Enable 0 */
#define RPI_GPLEN1                       0x0074 /*!< GPIO Pin Low Detect Enable 1 */
#define RPI_GPAREN0                      0x007c /*!< GPIO Pin Async. Rising Edge Detect 0 */
#define RPI_GPAREN1                      0x0080 /*!< GPIO Pin Async. Rising Edge Detect 1 */
#define RPI_GPAFEN0                      0x0088 /*!< GPIO Pin Async. Falling Edge Detect 0 */
#define RPI_GPAFEN1                      0x008c /*!< GPIO Pin Async. Falling Edge Detect 1 */
#define RPI_GPPUD                        0x0094 /*!< GPIO Pin Pull-up/down Enable */
#define RPI_GPPUDCLK0                    0x0098 /*!< GPIO Pin Pull-up/down Enable Clock 0 */
#define RPI_GPPUDCLK1                    0x009c /*!< GPIO Pin Pull-up/down Enable Clock 1 */

/*!   \brief rpiPortFunction
  Port function select modes for rpi_gpio_fsel()
*/
typedef enum
{
    RPI_GPIO_FSEL_INPT  = 0x00,   /*!< Input 0b000 */
    RPI_GPIO_FSEL_OUTP  = 0x01,   /*!< Output 0b001 */
    RPI_GPIO_FSEL_ALT0  = 0x04,   /*!< Alternate function 0 0b100 */
    RPI_GPIO_FSEL_ALT1  = 0x05,   /*!< Alternate function 1 0b101 */
    RPI_GPIO_FSEL_ALT2  = 0x06,   /*!< Alternate function 2 0b110, */
    RPI_GPIO_FSEL_ALT3  = 0x07,   /*!< Alternate function 3 0b111 */
    RPI_GPIO_FSEL_ALT4  = 0x03,   /*!< Alternate function 4 0b011 */
    RPI_GPIO_FSEL_ALT5  = 0x02,   /*!< Alternate function 5 0b010 */
    RPI_GPIO_FSEL_MASK  = 0x07    /*!< Function select bits mask 0b111 */
} rpiFunctionSelect;

/*! \brief rpiPUDControl
  Pullup/Pulldown defines for rpi_gpio_pud()
*/
typedef enum
{
    RPI_GPIO_PUD_OFF     = 0x00,   /*!< Off ? disable pull-up/down 0b00 */
    RPI_GPIO_PUD_DOWN    = 0x01,   /*!< Enable Pull Down control 0b01 */
    RPI_GPIO_PUD_UP      = 0x02    /*!< Enable Pull Up control 0b10  */
} rpiPUDControl;

/*! Pad control register offsets from RPI_GPIO_PADS */
#define RPI_PADS_GPIO_0_27               0x002c /*!< Pad control register for pads 0 to 27 */
#define RPI_PADS_GPIO_28_45              0x0030 /*!< Pad control register for pads 28 to 45 */
#define RPI_PADS_GPIO_46_53              0x0034 /*!< Pad control register for pads 46 to 53 */

/*! Pad Control masks */
#define RPI_PAD_PASSWRD                  (0x5A << 24)  /*!< Password to enable setting pad mask */
#define RPI_PAD_SLEW_RATE_UNLIMITED      0x10 /*!< Slew rate unlimited */
#define RPI_PAD_HYSTERESIS_ENABLED       0x08 /*!< Hysteresis enabled */
#define RPI_PAD_DRIVE_2mA                0x00 /*!< 2mA drive current */
#define RPI_PAD_DRIVE_4mA                0x01 /*!< 4mA drive current */
#define RPI_PAD_DRIVE_6mA                0x02 /*!< 6mA drive current */
#define RPI_PAD_DRIVE_8mA                0x03 /*!< 8mA drive current */
#define RPI_PAD_DRIVE_10mA               0x04 /*!< 10mA drive current */
#define RPI_PAD_DRIVE_12mA               0x05 /*!< 12mA drive current */
#define RPI_PAD_DRIVE_14mA               0x06 /*!< 14mA drive current */
#define RPI_PAD_DRIVE_16mA               0x07 /*!< 16mA drive current */

/*! \brief rpiPadGroup
  Pad group specification for rpi_gpio_pad()
*/
typedef enum
{
    RPI_PAD_GROUP_GPIO_0_27         = 0, /*!< Pad group for GPIO pads 0 to 27 */
    RPI_PAD_GROUP_GPIO_28_45        = 1, /*!< Pad group for GPIO pads 28 to 45 */
    RPI_PAD_GROUP_GPIO_46_53        = 2  /*!< Pad group for GPIO pads 46 to 53 */
} rpiPadGroup;

/*! \brief GPIO Pin Numbers

 */
typedef enum
{
    /* RPi Version 2, new plug P5 */
    RPI_V2_GPIO_P5_03     = 28,  /*!< Version 2, Pin P5-03 */
    RPI_V2_GPIO_P5_04     = 29,  /*!< Version 2, Pin P5-04 */
    RPI_V2_GPIO_P5_05     = 30,  /*!< Version 2, Pin P5-05 */
    RPI_V2_GPIO_P5_06     = 31,  /*!< Version 2, Pin P5-06 */

    /* RPi B+, RPi2, and RPi3 all use the same header 40 pins header*/
    RPI_V3_GPIO_03     =  2,  /*!< B+, Pin J8-03, defaults to i2c  */
    RPI_V3_GPIO_05     =  3,  /*!< B+, Pin J8-05, */
    RPI_V3_GPIO_07     =  4,  /*!< B+, Pin J8-07 */
    RPI_V3_GPIO_08     = 14,  /*!< B+, Pin J8-08, defaults to alt function 0 UART0_TXD */
    RPI_V3_GPIO_10     = 15,  /*!< B+, Pin J8-10, defaults to alt function 0 UART0_RXD */
    RPI_V3_GPIO_11     = 17,  /*!< B+, Pin J8-11 */
    RPI_V3_GPIO_12     = 18,  /*!< B+, Pin J8-12, can be PWM channel 0 in ALT FUN 5 */
    RPI_V3_GPIO_13     = 27,  /*!< B+, Pin J8-13 */
    RPI_V3_GPIO_15     = 22,  /*!< B+, Pin J8-15 */
    RPI_V3_GPIO_16     = 23,  /*!< B+, Pin J8-16 */
    RPI_V3_GPIO_18     = 24,  /*!< B+, Pin J8-18 */
    RPI_V3_GPIO_19     = 10,  /*!< B+, Pin J8-19, MOSI when SPI0 in use */
    RPI_V3_GPIO_21     =  9,  /*!< B+, Pin J8-21, MISO when SPI0 in use */
    RPI_V3_GPIO_22     = 25,  /*!< B+, Pin J8-22 */
    RPI_V3_GPIO_23     = 11,  /*!< B+, Pin J8-23, CLK when SPI0 in use */
    RPI_V3_GPIO_24     =  8,  /*!< B+, Pin J8-24, CE0 when SPI0 in use */
    RPI_V3_GPIO_26     =  7,  /*!< B+, Pin J8-26, CE1 when SPI0 in use */
    RPI_V3_GPIO_29     =  5,  /*!< B+, Pin J8-29,  */
    RPI_V3_GPIO_31     =  6,  /*!< B+, Pin J8-31,  */
    RPI_V3_GPIO_32     = 12,  /*!< B+, Pin J8-32,  */
    RPI_V3_GPIO_33     = 13,  /*!< B+, Pin J8-33,  */
    RPI_V3_GPIO_35     = 19,  /*!< B+, Pin J8-35,  */
    RPI_V3_GPIO_36     = 16,  /*!< B+, Pin J8-36,  */
    RPI_V3_GPIO_37     = 26,  /*!< B+, Pin J8-37,  */
    RPI_V3_GPIO_38     = 20,  /*!< B+, Pin J8-38,  */
    RPI_V3_GPIO_40     = 21   /*!< B+, Pin J8-40,  */

} RPiGPIOPin;

/* Defines for SPI
   GPIO register offsets from RPI_SPI0_BASE.
   Offsets into the SPI Peripheral block in bytes per 10.5 SPI Register Map
*/
#define RPI_SPI0_CS                      0x0000 /*!< SPI Master Control and Status */
#define RPI_SPI0_FIFO                    0x0004 /*!< SPI Master TX and RX FIFOs */
#define RPI_SPI0_CLK                     0x0008 /*!< SPI Master Clock Divider */
#define RPI_SPI0_DLEN                    0x000c /*!< SPI Master Data Length */
#define RPI_SPI0_LTOH                    0x0010 /*!< SPI LOSSI mode TOH */
#define RPI_SPI0_DC                      0x0014 /*!< SPI DMA DREQ Controls */

/* Register masks for SPI0_CS */
#define RPI_SPI0_CS_LEN_LONG             0x02000000 /*!< Enable Long data word in Lossi mode if DMA_LEN is set */
#define RPI_SPI0_CS_DMA_LEN              0x01000000 /*!< Enable DMA mode in Lossi mode */
#define RPI_SPI0_CS_CSPOL2               0x00800000 /*!< Chip Select 2 Polarity */
#define RPI_SPI0_CS_CSPOL1               0x00400000 /*!< Chip Select 1 Polarity */
#define RPI_SPI0_CS_CSPOL0               0x00200000 /*!< Chip Select 0 Polarity */
#define RPI_SPI0_CS_RXF                  0x00100000 /*!< RXF - RX FIFO Full */
#define RPI_SPI0_CS_RXR                  0x00080000 /*!< RXR RX FIFO needs Reading (full) */
#define RPI_SPI0_CS_TXD                  0x00040000 /*!< TXD TX FIFO can accept Data */
#define RPI_SPI0_CS_RXD                  0x00020000 /*!< RXD RX FIFO contains Data */
#define RPI_SPI0_CS_DONE                 0x00010000 /*!< Done transfer Done */
#define RPI_SPI0_CS_TE_EN                0x00008000 /*!< Unused */
#define RPI_SPI0_CS_LMONO                0x00004000 /*!< Unused */
#define RPI_SPI0_CS_LEN                  0x00002000 /*!< LEN LoSSI enable */
#define RPI_SPI0_CS_REN                  0x00001000 /*!< REN Read Enable */
#define RPI_SPI0_CS_ADCS                 0x00000800 /*!< ADCS Automatically Deassert Chip Select */
#define RPI_SPI0_CS_INTR                 0x00000400 /*!< INTR Interrupt on RXR */
#define RPI_SPI0_CS_INTD                 0x00000200 /*!< INTD Interrupt on Done */
#define RPI_SPI0_CS_DMAEN                0x00000100 /*!< DMAEN DMA Enable */
#define RPI_SPI0_CS_TA                   0x00000080 /*!< Transfer Active */
#define RPI_SPI0_CS_CSPOL                0x00000040 /*!< Chip Select Polarity */
#define RPI_SPI0_CS_CLEAR                0x00000030 /*!< Clear FIFO Clear RX and TX */
#define RPI_SPI0_CS_CLEAR_RX             0x00000020 /*!< Clear FIFO Clear RX  */
#define RPI_SPI0_CS_CLEAR_TX             0x00000010 /*!< Clear FIFO Clear TX  */
#define RPI_SPI0_CS_CPOL                 0x00000008 /*!< Clock Polarity */
#define RPI_SPI0_CS_CPHA                 0x00000004 /*!< Clock Phase */
#define RPI_SPI0_CS_CS                   0x00000003 /*!< Chip Select */

/*! \brief rpiSPIBitOrder SPI Bit order
  Specifies the SPI data bit ordering for rpi_spi_setBitOrder()
*/
typedef enum
{
    RPI_SPI_BIT_ORDER_LSBFIRST = 0,  /*!< LSB First */
    RPI_SPI_BIT_ORDER_MSBFIRST = 1   /*!< MSB First */
}rpiSPIBitOrder;

/*! \brief SPI Data mode
  Specify the SPI data mode to be passed to rpi_spi_setDataMode()
*/
typedef enum
{
    RPI_SPI_MODE0 = 0,  /*!< CPOL = 0, CPHA = 0 */
    RPI_SPI_MODE1 = 1,  /*!< CPOL = 0, CPHA = 1 */
    RPI_SPI_MODE2 = 2,  /*!< CPOL = 1, CPHA = 0 */
    RPI_SPI_MODE3 = 3   /*!< CPOL = 1, CPHA = 1 */
}rpiSPIMode;

/*! \brief rpiSPIChipSelect
  Specify the SPI chip select pin(s)
*/
typedef enum
{
    RPI_SPI_CS0 = 0,     /*!< Chip Select 0 */
    RPI_SPI_CS1 = 1,     /*!< Chip Select 1 */
    RPI_SPI_CS2 = 2,     /*!< Chip Select 2 (ie pins CS1 and CS2 are asserted) */
    RPI_SPI_CS_NONE = 3  /*!< No CS, control it yourself */
} rpiSPIChipSelect;

/*! \brief rpiSPIClockDivider
  Specifies the divider used to generate the SPI clock from the system clock.
  Figures below give the divider, clock period and clock frequency.
  Clock divided is based on nominal base clock rate of 250MHz
  It is reported that (contrary to the documentation) any even divider may used.
  The frequencies shown for each divider have been confirmed by measurement.
*/

typedef enum
{
    RPI_SPI_CLOCK_DIVIDER_65536 = 0,       /*!< 65536 = 262.144us = 3.814697260kHz */
    RPI_SPI_CLOCK_DIVIDER_32768 = 32768,   /*!< 32768 = 131.072us = 7.629394531kHz */
    RPI_SPI_CLOCK_DIVIDER_16384 = 16384,   /*!< 16384 = 65.536us = 15.25878906kHz */
    RPI_SPI_CLOCK_DIVIDER_8192  = 8192,    /*!< 8192 = 32.768us = 30/51757813kHz */
    RPI_SPI_CLOCK_DIVIDER_4096  = 4096,    /*!< 4096 = 16.384us = 61.03515625kHz */
    RPI_SPI_CLOCK_DIVIDER_2048  = 2048,    /*!< 2048 = 8.192us = 122.0703125kHz */
    RPI_SPI_CLOCK_DIVIDER_1024  = 1024,    /*!< 1024 = 4.096us = 244.140625kHz */
    RPI_SPI_CLOCK_DIVIDER_512   = 512,     /*!< 512 = 2.048us = 488.28125kHz */
    RPI_SPI_CLOCK_DIVIDER_256   = 256,     /*!< 256 = 1.024us = 976.5625kHz */
    RPI_SPI_CLOCK_DIVIDER_128   = 128,     /*!< 128 = 512ns = = 1.953125MHz */
    RPI_SPI_CLOCK_DIVIDER_64    = 64,      /*!< 64 = 256ns = 3.90625MHz */
    RPI_SPI_CLOCK_DIVIDER_32    = 32,      /*!< 32 = 128ns = 7.8125MHz */
    RPI_SPI_CLOCK_DIVIDER_16    = 16,      /*!< 16 = 64ns = 15.625MHz */
    RPI_SPI_CLOCK_DIVIDER_8     = 8,       /*!< 8 = 32ns = 31.25MHz */
    RPI_SPI_CLOCK_DIVIDER_4     = 4,       /*!< 4 = 16ns = 62.5MHz */
    RPI_SPI_CLOCK_DIVIDER_2     = 2,       /*!< 2 = 8ns = 125MHz, fastest you can get */
    RPI_SPI_CLOCK_DIVIDER_1     = 1        /*!< 1 = 262.144us = 3.814697260kHz, same as 0/65536 */
} rpiSPIClockDivider;

/* Defines for I2C
   GPIO register offsets from RPI_BSC*_BASE.
   Offsets into the BSC Peripheral block in bytes per 3.1 BSC Register Map
*/
#define RPI_BSC_C 			0x0000 /*!< BSC Master Control */
#define RPI_BSC_S 			0x0004 /*!< BSC Master Status */
#define RPI_BSC_DLEN		0x0008 /*!< BSC Master Data Length */
#define RPI_BSC_A 			0x000c /*!< BSC Master Slave Address */
#define RPI_BSC_FIFO		0x0010 /*!< BSC Master Data FIFO */
#define RPI_BSC_DIV			0x0014 /*!< BSC Master Clock Divider */
#define RPI_BSC_DEL			0x0018 /*!< BSC Master Data Delay */
#define RPI_BSC_CLKT		0x001c /*!< BSC Master Clock Stretch Timeout */

/* Register masks for BSC_C */
#define RPI_BSC_C_I2CEN 		0x00008000 /*!< I2C Enable, 0 = disabled, 1 = enabled */
#define RPI_BSC_C_INTR 		0x00000400 /*!< Interrupt on RX */
#define RPI_BSC_C_INTT 		0x00000200 /*!< Interrupt on TX */
#define RPI_BSC_C_INTD 		0x00000100 /*!< Interrupt on DONE */
#define RPI_BSC_C_ST 		0x00000080 /*!< Start transfer, 1 = Start a new transfer */
#define RPI_BSC_C_CLEAR_1 		0x00000020 /*!< Clear FIFO Clear */
#define RPI_BSC_C_CLEAR_2 		0x00000010 /*!< Clear FIFO Clear */
#define RPI_BSC_C_READ 		0x00000001 /*!<	Read transfer */

/* Register masks for BSC_S */
#define RPI_BSC_S_CLKT 		0x00000200 /*!< Clock stretch timeout */
#define RPI_BSC_S_ERR 		0x00000100 /*!< ACK error */
#define RPI_BSC_S_RXF 		0x00000080 /*!< RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full */
#define RPI_BSC_S_TXE 		0x00000040 /*!< TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full */
#define RPI_BSC_S_RXD 		0x00000020 /*!< RXD FIFO contains data */
#define RPI_BSC_S_TXD 		0x00000010 /*!< TXD FIFO can accept data */
#define RPI_BSC_S_RXR 		0x00000008 /*!< RXR FIFO needs reading (full) */
#define RPI_BSC_S_TXW 		0x00000004 /*!< TXW FIFO needs writing (full) */
#define RPI_BSC_S_DONE 		0x00000002 /*!< Transfer DONE */
#define RPI_BSC_S_TA 		0x00000001 /*!< Transfer Active */

#define RPI_BSC_FIFO_SIZE   	16 /*!< BSC FIFO size */

/*! \brief rpiI2CClockDivider
  Specifies the divider used to generate the I2C clock from the system clock.
  Clock divided is based on nominal base clock rate of 250MHz
*/
typedef enum
{
    RPI_I2C_CLOCK_DIVIDER_2500   = 2500,      /*!< 2500 = 10us = 100 kHz */
    RPI_I2C_CLOCK_DIVIDER_626    = 626,       /*!< 622 = 2.504us = 399.3610 kHz */
    RPI_I2C_CLOCK_DIVIDER_150    = 150,       /*!< 150 = 60ns = 1.666 MHz (default at reset) */
    RPI_I2C_CLOCK_DIVIDER_148    = 148        /*!< 148 = 59ns = 1.689 MHz */
} rpiI2CClockDivider;

/*! \brief rpiI2CReasonCodes
  Specifies the reason codes for the rpi_i2c_write and rpi_i2c_read functions.
*/
typedef enum
{
    RPI_I2C_REASON_OK   	     = 0x00,      /*!< Success */
    RPI_I2C_REASON_ERROR_NACK    = 0x01,      /*!< Received a NACK */
    RPI_I2C_REASON_ERROR_CLKT    = 0x02,      /*!< Received Clock Stretch Timeout */
    RPI_I2C_REASON_ERROR_DATA    = 0x04       /*!< Not all data is sent / received */
} rpiI2CReasonCodes;

/* Defines for ST
   GPIO register offsets from RPI_ST_BASE.
   Offsets into the ST Peripheral block in bytes per 12.1 System Timer Registers
   The System Timer peripheral provides four 32-bit timer channels and a single 64-bit free running counter.
   RPI_ST_CLO is the System Timer Counter Lower bits register.
   The system timer free-running counter lower register is a read-only register that returns the current value
   of the lower 32-bits of the free running counter.
   RPI_ST_CHI is the System Timer Counter Upper bits register.
   The system timer free-running counter upper register is a read-only register that returns the current value
   of the upper 32-bits of the free running counter.
*/
#define RPI_ST_CS 			0x0000 /*!< System Timer Control/Status */
#define RPI_ST_CLO 			0x0004 /*!< System Timer Counter Lower 32 bits */
#define RPI_ST_CHI 			0x0008 /*!< System Timer Counter Upper 32 bits */

/* Defines for PWM, word offsets (ie 4 byte multiples) */
#define RPI_PWM_CONTROL 0
#define RPI_PWM_STATUS  1
#define RPI_PWM_DMAC    2
#define RPI_PWM0_RANGE  4
#define RPI_PWM0_DATA   5
#define RPI_PWM_FIF1    6
#define RPI_PWM1_RANGE  8
#define RPI_PWM1_DATA   9

/* Defines for PWM Clock, word offsets (ie 4 byte multiples) */
#define RPI_PWMCLK_CNTL     40
#define RPI_PWMCLK_DIV      41
#define RPI_PWM_PASSWRD     (0x5A << 24)  /*!< Password to enable setting PWM clock */

#define RPI_PWM_CLEAR_FIFO  0x0040  /*!< Clear FIFO */
#define RPI_PWM0_MS_MODE    0x0080  /*!< Run in Mark/Space mode */
#define RPI_PWM0_USEFIFO    0x0020  /*!< Data from FIFO */
#define RPI_PWM0_REVPOLAR   0x0010  /*!< Reverse polarity */
#define RPI_PWM0_OFFSTATE   0x0008  /*!< Ouput Off state */
#define RPI_PWM0_REPEATFF   0x0004  /*!< Repeat last value if FIFO empty */
#define RPI_PWM0_SERIAL     0x0002  /*!< Run in serial mode */
#define RPI_PWM0_ENABLE     0x0001  /*!< Channel Enable */

/*! \brief rpiPWMClockDivider
  Specifies the divider used to generate the PWM clock from the system clock.
  Figures below give the divider, clock period and clock frequency.
  Clock divided is based on nominal PWM base clock rate of 19.2MHz
  The frequencies shown for each divider have been confirmed by measurement
*/
typedef enum
{
    RPI_PWM_CLOCK_DIVIDER_2048  = 2048,    /*!< 2048 = 9.375kHz */
    RPI_PWM_CLOCK_DIVIDER_1024  = 1024,    /*!< 1024 = 18.75kHz */
    RPI_PWM_CLOCK_DIVIDER_512   = 512,     /*!< 512 = 37.5kHz */
    RPI_PWM_CLOCK_DIVIDER_256   = 256,     /*!< 256 = 75kHz */
    RPI_PWM_CLOCK_DIVIDER_128   = 128,     /*!< 128 = 150kHz */
    RPI_PWM_CLOCK_DIVIDER_64    = 64,      /*!< 64 = 300kHz */
    RPI_PWM_CLOCK_DIVIDER_32    = 32,      /*!< 32 = 600.0kHz */
    RPI_PWM_CLOCK_DIVIDER_16    = 16,      /*!< 16 = 1.2MHz */
    RPI_PWM_CLOCK_DIVIDER_8     = 8,       /*!< 8 = 2.4MHz */
    RPI_PWM_CLOCK_DIVIDER_4     = 4,       /*!< 4 = 4.8MHz */
    RPI_PWM_CLOCK_DIVIDER_2     = 2,       /*!< 2 = 9.6MHz, fastest you can get */
    RPI_PWM_CLOCK_DIVIDER_1     = 1        /*!< 1 = 4.6875kHz, same as divider 4096 */
} rpiPWMClockDivider;

/* Historical name compatibility */
#ifndef RPI_NO_DELAY_COMPATIBILITY
#define delay(x) rpi_delay(x)
#define delayMicroseconds(x) rpi_delayMicroseconds(x)
#endif

#ifdef __cplusplus
extern "C" {
#endif
    /*! Open the library.
      \return 1 if successful else 0
    */
    int rpi_init(void);

    /*! Close the library, deallocating any allocated memory and closing /dev/mem
      \return 1 if successful else 0
    */
    int rpi_close(void);
    unsigned int rpi_version(void);

    /*! Gets the base of a register
      \param[in] regbase You can use one of the common values RPI_REGBASE_*
      in \ref rpiRegisterBase
      \return the register base
    */
    uint32_t* rpi_regbase(uint8_t regbase);
    uint32_t rpi_peri_read(volatile uint32_t* paddr);
    uint32_t rpi_peri_read_nb(volatile uint32_t* paddr);
    void rpi_peri_write(volatile uint32_t* paddr, uint32_t value);
    void rpi_peri_write_nb(volatile uint32_t* paddr, uint32_t value);
    void rpi_peri_set_bits(volatile uint32_t* paddr, uint32_t value, uint32_t mask);

    /*! \defgroup gpio GPIO register access
      These functions allow you to control the GPIO interface. You can set the
      function of each GPIO pin, read the input state and set the output state.
      @{
    */

    /*! Sets the Function Select register for the given pin, which configures
      the pin as Input, Output or one of the 6 alternate functions.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
      \param[in] mode Mode to set the pin to, one of RPI_GPIO_FSEL_* from \ref rpiFunctionSelect
    */
    void rpi_gpio_fsel(uint8_t pin, uint8_t mode);

    /*! Sets the specified pin output to
      HIGH.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
      \sa rpi_gpio_write()
    */
    void rpi_gpio_set(uint8_t pin);

    /*! Sets the specified pin output to
      LOW.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
      \sa rpi_gpio_write()
    */
    void rpi_gpio_clr(uint8_t pin);

    /*! Sets any of the first 32 GPIO output pins specified in the mask to
      HIGH.
      \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
      \sa rpi_gpio_write_multi()
    */
    void rpi_gpio_set_multi(uint32_t mask);

    /*! Sets any of the first 32 GPIO output pins specified in the mask to
      LOW.
      \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
      \sa rpi_gpio_write_multi()
    */
    void rpi_gpio_clr_multi(uint32_t mask);

    /*! Reads the current level on the specified
      pin and returns either HIGH or LOW. Works whether or not the pin
      is an input or an output.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
      \return the current level  either HIGH or LOW
    */
    uint8_t rpi_gpio_lev(uint8_t pin);

    /*! Event Detect Status.
      Tests whether the specified pin has detected a level or edge
      as requested by rpi_gpio_ren(), rpi_gpio_fen(), rpi_gpio_hen(),
      rpi_gpio_len(), rpi_gpio_aren(), rpi_gpio_afen().
      Clear the flag for a given pin by calling rpi_gpio_set_eds(pin);
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
      \return HIGH if the event detect status for the given pin is true.
    */
    uint8_t rpi_gpio_eds(uint8_t pin);

    /*! Same as rpi_gpio_eds() but checks if any of the pins specified in
      the mask have detected a level or edge.
      \param[in] mask Mask of pins to check. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
      \return Mask of pins HIGH if the event detect status for the given pin is true.
    */
    uint32_t rpi_gpio_eds_multi(uint32_t mask);

    /*! Sets the Event Detect Status register for a given pin to 1,
      which has the effect of clearing the flag. Use this afer seeing
      an Event Detect Status on the pin.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_set_eds(uint8_t pin);

    /*! Same as rpi_gpio_set_eds() but clears the flag for any pin which
      is set in the mask.
      \param[in] mask Mask of pins to clear. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
    */
    void rpi_gpio_set_eds_multi(uint32_t mask);

    /*! Enable Rising Edge Detect Enable for the specified pin.
      When a rising edge is detected, sets the appropriate pin in Event Detect Status.
      The GPRENn registers use
      synchronous edge detection. This means the input signal is sampled using the
      system clock and then it is looking for a ?011? pattern on the sampled signal. This
      has the effect of suppressing glitches.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_ren(uint8_t pin);

    /*! Disable Rising Edge Detect Enable for the specified pin.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_clr_ren(uint8_t pin);

    /*! Enable Falling Edge Detect Enable for the specified pin.
      When a falling edge is detected, sets the appropriate pin in Event Detect Status.
      The GPRENn registers use
      synchronous edge detection. This means the input signal is sampled using the
      system clock and then it is looking for a ?100? pattern on the sampled signal. This
      has the effect of suppressing glitches.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_fen(uint8_t pin);

    /*! Disable Falling Edge Detect Enable for the specified pin.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_clr_fen(uint8_t pin);

    /*! Enable High Detect Enable for the specified pin.
      When a HIGH level is detected on the pin, sets the appropriate pin in Event Detect Status.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_hen(uint8_t pin);

    /*! Disable High Detect Enable for the specified pin.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_clr_hen(uint8_t pin);

    /*! Enable Low Detect Enable for the specified pin.
      When a LOW level is detected on the pin, sets the appropriate pin in Event Detect Status.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_len(uint8_t pin);

    /*! Disable Low Detect Enable for the specified pin.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_clr_len(uint8_t pin);

    /*! Enable Asynchronous Rising Edge Detect Enable for the specified pin.
      When a rising edge is detected, sets the appropriate pin in Event Detect Status.
      Asynchronous means the incoming signal is not sampled by the system clock. As such
      rising edges of very short duration can be detected.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_aren(uint8_t pin);

    /*! Disable Asynchronous Rising Edge Detect Enable for the specified pin.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_clr_aren(uint8_t pin);

    /*! Enable Asynchronous Falling Edge Detect Enable for the specified pin.
      When a falling edge is detected, sets the appropriate pin in Event Detect Status.
      Asynchronous means the incoming signal is not sampled by the system clock. As such
      falling edges of very short duration can be detected.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_afen(uint8_t pin);

    /*! Disable Asynchronous Falling Edge Detect Enable for the specified pin.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
    */
    void rpi_gpio_clr_afen(uint8_t pin);

    /*! Sets the Pull-up/down register for the given pin. This is
      used with rpi_gpio_pudclk() to set the  Pull-up/down resistor for the given pin.
      However, it is usually more convenient to use rpi_gpio_set_pud().
      \param[in] pud The desired Pull-up/down mode. One of RPI_GPIO_PUD_* from rpiPUDControl
      \sa rpi_gpio_set_pud()
    */
    void rpi_gpio_pud(uint8_t pud);

    /*! Clocks the Pull-up/down value set earlier by rpi_gpio_pud() into the pin.
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
      \param[in] on HIGH to clock the value from rpi_gpio_pud() into the pin.
      LOW to remove the clock.
      \sa rpi_gpio_set_pud()
    */
    void rpi_gpio_pudclk(uint8_t pin, uint8_t on);

    /*! Reads and returns the Pad Control for the given GPIO group.
      \param[in] group The GPIO pad group number, one of RPI_PAD_GROUP_GPIO_*
      \return Mask of bits from RPI_PAD_* from \ref rpiPadGroup
    */
    uint32_t rpi_gpio_pad(uint8_t group);

    /*! Sets the Pad Control for the given GPIO group.
      \param[in] group The GPIO pad group number, one of RPI_PAD_GROUP_GPIO_*
      \param[in] control Mask of bits from RPI_PAD_* from \ref rpiPadGroup. Note
      that it is not necessary to include RPI_PAD_PASSWRD in the mask as this
      is automatically included.
    */
    void rpi_gpio_set_pad(uint8_t group, uint32_t control);

    /*! Delays for the specified number of milliseconds.
      Uses nanosleep(), and therefore does not use CPU until the time is up.
      However, you are at the mercy of nanosleep(). From the manual for nanosleep():
      If the interval specified in req is not an exact multiple of the granularity
      underlying  clock  (see  time(7)),  then the interval will be
      rounded up to the next multiple. Furthermore, after the sleep completes,
      there may still be a delay before the CPU becomes free to once
      again execute the calling thread.
      \param[in] millis Delay in milliseconds
    */
    void rpi_delay (unsigned int millis);

    /*! Delays for the specified number of microseconds.
      Uses a combination of nanosleep() and a busy wait loop on the RPI system timers,
      However, you are at the mercy of nanosleep(). From the manual for nanosleep():
      If the interval specified in req is not an exact multiple of the granularity
      underlying  clock  (see  time(7)),  then the interval will be
      rounded up to the next multiple. Furthermore, after the sleep completes,
      there may still be a delay before the CPU becomes free to once
      again execute the calling thread.
      For times less than about 450 microseconds, uses a busy wait on the System Timer.
      It is reported that a delay of 0 microseconds on RaspberryPi will in fact
      result in a delay of about 80 microseconds. Your mileage may vary.
      \param[in] micros Delay in microseconds
    */
    void rpi_delayMicroseconds (uint64_t micros);

    /*! Sets the output state of the specified pin
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
      \param[in] on HIGH sets the output to HIGH and LOW to LOW.
    */
    void rpi_gpio_write(uint8_t pin, uint8_t on);

    /*! Sets any of the first 32 GPIO output pins specified in the mask to the state given by on
      \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
      \param[in] on HIGH sets the output to HIGH and LOW to LOW.
    */
    void rpi_gpio_write_multi(uint32_t mask, uint8_t on);

    /*! Sets the first 32 GPIO output pins specified in the mask to the value given by value
      \param[in] value values required for each bit masked in by mask, eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
      \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
    */
    void rpi_gpio_write_mask(uint32_t value, uint32_t mask);

    /*! Sets the Pull-up/down mode for the specified pin. This is more convenient than
      clocking the mode in with rpi_gpio_pud() and rpi_gpio_pudclk().
      \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
      \param[in] pud The desired Pull-up/down mode. One of RPI_GPIO_PUD_* from rpiPUDControl
    */
    void rpi_gpio_set_pud(uint8_t pin, uint8_t pud);

    /*! \defgroup spi SPI access
      These functions let you use SPI0 (Serial Peripheral Interface) to
      interface with an  al SPI device.
      @{
    */

    /*! Start SPI operations.
      Forces RPi SPI0 pins P1-19 (MOSI), P1-21 (MISO), P1-23 (CLK), P1-24 (CE0) and P1-26 (CE1)
      to alternate function ALT0, which enables those pins for SPI interface.
      You should call rpi_spi_end() when all SPI funcitons are complete to return the pins to
      their default functions.
      \sa  rpi_spi_end()
      \return 1 if successful, 0 otherwise (perhaps because you are not running as root)
    */
    int rpi_spi_begin(void);

    /*! End SPI operations.
      SPI0 pins P1-19 (MOSI), P1-21 (MISO), P1-23 (CLK), P1-24 (CE0) and P1-26 (CE1)
      are returned to their default INPUT behaviour.
    */
    void rpi_spi_end(void);

    /*! Sets the SPI bit order
      NOTE: has no effect. Not supported by SPI0.
      Defaults to
      \param[in] order The desired bit order, one of RPI_SPI_BIT_ORDER_*,
      see \ref rpiSPIBitOrder
    */
    void rpi_spi_setBitOrder(uint8_t order);

    /*! Sets the SPI clock divider and therefore the
      SPI clock speed.
      \param[in] divider The desired SPI clock divider, one of RPI_SPI_CLOCK_DIVIDER_*,
      see \ref rpiSPIClockDivider
    */
    void rpi_spi_setClockDivider(uint16_t divider);

    /*! Sets the SPI data mode
      Sets the clock polariy and phase
      \param[in] mode The desired data mode, one of RPI_SPI_MODE*,
      see \ref rpiSPIMode
    */
    void rpi_spi_setDataMode(uint8_t mode);

    /*! Sets the chip select pin(s)
      When an rpi_spi_transfer() is made, the selected pin(s) will be asserted during the
      transfer.
      \param[in] cs Specifies the CS pins(s) that are used to activate the desired slave.
      One of RPI_SPI_CS*, see \ref rpiSPIChipSelect
    */
    void rpi_spi_chipSelect(uint8_t cs);

    /*! Sets the chip select pin polarity for a given pin
      When an rpi_spi_transfer() occurs, the currently selected chip select pin(s)
      will be asserted to the
      value given by active. When transfers are not happening, the chip select pin(s)
      return to the complement (inactive) value.
      \param[in] cs The chip select pin to affect
      \param[in] active Whether the chip select pin is to be active HIGH
    */
    void rpi_spi_setChipSelectPolarity(uint8_t cs, uint8_t active);

    /*! Transfers one byte to and from the currently selected SPI slave.
      Asserts the currently selected CS pins (as previously set by rpi_spi_chipSelect)
      during the transfer.
      Clocks the 8 bit value out on MOSI, and simultaneously clocks in data from MISO.
      Returns the read data byte from the slave.
      Uses polled transfer as per section 10.6.1 of the BCM 2835 ARM Peripherls manual
      \param[in] value The 8 bit data byte to write to MOSI
      \return The 8 bit byte simultaneously read from  MISO
      \sa rpi_spi_transfern()
    */
    uint8_t rpi_spi_transfer(uint8_t value);

    /*! Transfers any number of bytes to and from the currently selected SPI slave.
      Asserts the currently selected CS pins (as previously set by rpi_spi_chipSelect)
      during the transfer.
      Clocks the len 8 bit bytes out on MOSI, and simultaneously clocks in data from MISO.
      The data read read from the slave is placed into rbuf. rbuf must be at least len bytes long
      Uses polled transfer as per section 10.6.1 of the BCM 2835 ARM Peripherls manual
      \param[in] tbuf Buffer of bytes to send.
      \param[out] rbuf Received bytes will by put in this buffer
      \param[in] len Number of bytes in the tbuf buffer, and the number of bytes to send/received
      \sa rpi_spi_transfer()
    */
    void rpi_spi_transfernb(char* tbuf, char* rbuf, uint32_t len);

    /*! Transfers any number of bytes to and from the currently selected SPI slave
      using rpi_spi_transfernb.
      The returned data from the slave replaces the transmitted data in the buffer.
      \param[in,out] buf Buffer of bytes to send. Received bytes will replace the contents
      \param[in] len Number of bytes int eh buffer, and the number of bytes to send/received
      \sa rpi_spi_transfer()
    */
    void rpi_spi_transfern(char* buf, uint32_t len);

    /*! Transfers any number of bytes to the currently selected SPI slave.
      Asserts the currently selected CS pins (as previously set by rpi_spi_chipSelect)
      during the transfer.
      \param[in] buf Buffer of bytes to send.
      \param[in] len Number of bytes in the tbuf buffer, and the number of bytes to send
    */
    void rpi_spi_writenb(char* buf, uint32_t len);

    /*! @} */

    /*! \defgroup i2c I2C access
      These functions let you use I2C (The Broadcom Serial Control bus with the Philips
      I2C bus/interface version 2.1 January 2000.) to interface with an  al I2C device.
      @{
    */

    /*! Start I2C operations.
      Forces RPi I2C pins P1-03 (SDA) and P1-05 (SCL)
      to alternate function ALT0, which enables those pins for I2C interface.
      You should call rpi_i2c_end() when all I2C functions are complete to return the pins to
      their default functions
      \return 1 if successful, 0 otherwise (perhaps because you are not running as root)
      \sa  rpi_i2c_end()
    */
    int rpi_i2c_init(void);

    /*! End I2C operations.
      I2C pins P1-03 (SDA) and P1-05 (SCL)
      are returned to their default INPUT behaviour.
    */
    void rpi_i2c_close(void);

    /*! Sets the I2C slave address.
      \param[in] addr The I2C slave address.
    */
    void rpi_i2c_setSlaveAddress(uint8_t addr);

    /*! Sets the I2C clock divider and therefore the I2C clock speed.
      \param[in] divider The desired I2C clock divider, one of RPI_I2C_CLOCK_DIVIDER_*,
      see \ref rpiI2CClockDivider
    */
    void rpi_i2c_setClockDivider(uint16_t divider);

    /*! Sets the I2C clock divider by converting the baudrate parameter to
      the equivalent I2C clock divider. ( see \sa rpi_i2c_setClockDivider)
      For the I2C standard 100khz you would set baudrate to 100000
      The use of baudrate corresponds to its use in the I2C kernel device
      driver. (Of course, rpi has nothing to do with the kernel driver)
    */
    void rpi_i2c_set_baudrate(uint32_t baudrate);

    /*! Transfers any number of bytes to the currently selected I2C slave.
      (as previously set by \sa rpi_i2c_setSlaveAddress)
      \param[in] buf Buffer of bytes to send.
      \param[in] len Number of bytes in the buf buffer, and the number of bytes to send.
      \return reason see \ref rpiI2CReasonCodes
    */
    uint8_t rpi_i2c_write(const char * buf, uint32_t len);

    /*! Transfers any number of bytes from the currently selected I2C slave.
      (as previously set by \sa rpi_i2c_setSlaveAddress)
      \param[in] buf Buffer of bytes to receive.
      \param[in] len Number of bytes in the buf buffer, and the number of bytes to received.
      \return reason see \ref rpiI2CReasonCodes
    */
    uint8_t rpi_i2c_read(char* buf, uint32_t len);

    /*! Allows reading from I2C slaves that require a repeated start (without any prior stop)
      to read after the required slave register has been set. For example, the popular
      MPL3115A2 pressure and temperature sensor. Note that your device must support or
      require this mode. If your device does not require this mode then the standard
      combined:
      \sa rpi_i2c_write
      \sa rpi_i2c_read
      are a better choice.
      Will read from the slave previously set by \sa rpi_i2c_setSlaveAddress
      \param[in] regaddr Buffer containing the slave register you wish to read from.
      \param[in] buf Buffer of bytes to receive.
      \param[in] len Number of bytes in the buf buffer, and the number of bytes to received.
      \return reason see \ref rpiI2CReasonCodes
    */
    uint8_t rpi_i2c_read_register_rs(char* regaddr, char* buf, uint32_t len);

    /*! Allows sending an arbitrary number of bytes to I2C slaves before issuing a repeated
      start (with no prior stop) and reading a response.
      Necessary for devices that require such behavior, such as the MLX90620.
      Will write to and read from the slave previously set by \sa rpi_i2c_setSlaveAddress
      \param[in] cmds Buffer containing the bytes to send before the repeated start condition.
      \param[in] cmds_len Number of bytes to send from cmds buffer
      \param[in] buf Buffer of bytes to receive.
      \param[in] buf_len Number of bytes to receive in the buf buffer.
      \return reason see \ref rpiI2CReasonCodes
    */
    uint8_t rpi_i2c_write_read_rs(char* cmds, uint32_t cmds_len, char* buf, uint32_t buf_len);

    /*! @} */

    /*! \defgroup st System Timer access
      Allows access to and delays using the System Timer Counter.
      @{
    */

    /*! Read the System Timer Counter register.
      \return the value read from the System Timer Counter Lower 32 bits register
    */
    uint64_t rpi_st_read(void);

    /*! Delays for the specified number of microseconds with offset.
      \param[in] offset_micros Offset in microseconds
      \param[in] micros Delay in microseconds
    */
    void rpi_st_delay(uint64_t offset_micros, uint64_t micros);

    /*! @}  */

    /*! \defgroup pwm Pulse Width Modulation
      Allows control of 2 independent PWM channels. A limited subset of GPIO pins
      can be connected to one of these 2 channels, allowing PWM control of GPIO pins.
      You have to set the desired pin into a particular Alt Fun to PWM output. See the PWM
      documentation on the Main Page.
      @{
    */

    void rpi_pwm_init();
    void rpi_pwm_close();

    /*! Sets the PWM clock divisor,
      to control the basic PWM pulse widths.
      \param[in] divisor Divides the basic 19.2MHz PWM clock. You can use one of the common
      values RPI_PWM_CLOCK_DIVIDER_* in \ref rpiPWMClockDivider
    */
    void rpi_pwm_set_clock(uint32_t divisor);

    /*! Sets the mode of the given PWM channel,
      allowing you to control the PWM mode and enable/disable that channel
      \param[in] channel The PWM channel. 0 or 1.
      \param[in] markspace Set true if you want Mark-Space mode. 0 for Balanced mode.
      \param[in] enabled Set true to enable this channel and produce PWM pulses.
    */
    void rpi_pwm_set_mode(uint8_t markspace, uint8_t enabled);

    /*! Sets the maximum range of the PWM output.
      The data value can vary between 0 and this range to control PWM output
      \param[in] channel The PWM channel. 0 or 1.
      \param[in] range The maximum value permitted for DATA.
    */
    void rpi_pwm_set_range(uint32_t range);

    /*! Sets the PWM pulse ratio to emit to DATA/RANGE,
      where RANGE is set by rpi_pwm_set_range().
      \param[in] channel The PWM channel. 0 or 1.
      \param[in] data Controls the PWM output ratio as a fraction of the range.
      Can vary from 0 to RANGE.
    */
    void rpi_pwm_set_data(uint32_t data);

    /*! @}  */
#ifdef __cplusplus
}
#endif

#endif /* RPI_H */
