/* rpi.c
   C and C++ support for Broadcom BCM 2835 as used in Raspberry Pi
   http://elinux.org/RPi_Low-level_peripherals
   http://www.raspberrypi.org/wp-content/uploads/2012/02/RPI-ARM-Peripherals.pdf

   Author: Mike McCauley
   Copyright (C) 2011-2013 Mike McCauley
   $Id: rpi.c,v 1.23 2015/03/31 04:55:41 mikem Exp mikem $

   Modified by Xipeng Wang
   Copyright (C) 2016
*/


#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>

#include "rpi.h"

/* Physical address and size of the peripherals block
// May be overridden on RPi2
*/
uint32_t *rpi_peripherals_base = (uint32_t *)RPI_PERI_BASE;
uint32_t rpi_peripherals_size = RPI_PERI_SIZE;

/* Virtual memory address of the mapped peripherals block
 */
uint32_t *rpi_peripherals = (uint32_t *)MAP_FAILED;

/* And the register bases within the peripherals block
 */
volatile uint32_t *rpi_interrupt   = (uint32_t *)MAP_FAILED;
volatile uint32_t *rpi_gpio        = (uint32_t *)MAP_FAILED;
volatile uint32_t *rpi_pwm         = (uint32_t *)MAP_FAILED;
volatile uint32_t *rpi_clk         = (uint32_t *)MAP_FAILED;
volatile uint32_t *rpi_pads        = (uint32_t *)MAP_FAILED;
volatile uint32_t *rpi_spi0        = (uint32_t *)MAP_FAILED;
volatile uint32_t *rpi_bsc0        = (uint32_t *)MAP_FAILED;
volatile uint32_t *rpi_bsc1        = (uint32_t *)MAP_FAILED;
volatile uint32_t *rpi_st	       = (uint32_t *)MAP_FAILED;

/* This variable allows us to test on hardware other than RPi.
// It prevents access to the kernel memory, and does not do any peripheral access
// Instead it prints out what it _would_ do if debug were 0
*/
static uint8_t debug = 0;

/* I2C The time needed to transmit one byte. In microseconds.
 */
static int i2c_byte_wait_us = 0;

/*
// Low level register access functions
*/

/* Function to return the pointers to the hardware register bases */
uint32_t* rpi_regbase(uint8_t regbase)
{
    switch (regbase)
    {
        case RPI_REGBASE_ST:
            return (uint32_t *)rpi_st;
        case RPI_REGBASE_GPIO:
            return (uint32_t *)rpi_gpio;
        case RPI_REGBASE_PWM:
            return (uint32_t *)rpi_pwm;
        case RPI_REGBASE_CLK:
            return (uint32_t *)rpi_clk;
        case RPI_REGBASE_PADS:
            return (uint32_t *)rpi_pads;
        case RPI_REGBASE_SPI0:
            return (uint32_t *)rpi_spi0;
        case RPI_REGBASE_BSC1:
            return (uint32_t *)rpi_bsc1;
    }
    return (uint32_t *)MAP_FAILED;
}

void  rpi_set_debug(uint8_t d)
{
    debug = d;
}

unsigned int rpi_version(void)
{
    return RPI_VERSION;
}

uint32_t rpi_peri_read(volatile uint32_t* paddr)
{
    return mem_read_32(paddr);
}

uint32_t rpi_peri_read_nb(volatile uint32_t* paddr)
{
    return mem_read_32_nb(paddr);
}

void rpi_peri_write(volatile uint32_t* paddr, uint32_t value)
{
    mem_write_32(paddr,value);
}

void rpi_peri_write_nb(volatile uint32_t* paddr, uint32_t value)
{
    return mem_write_32_nb(paddr,value);
}

void rpi_peri_set_bits(volatile uint32_t* paddr, uint32_t value, uint32_t mask)
{
    mem_set_bits_32(paddr,value,mask);
}

/* Function select
   pin is a RPI GPIO pin number NOT RPi pin number
   There are 6 control registers, each control the functions of a block
   of 10 pins.
   Each control register has 10 sets of 3 bits per GPIO pin:

   RPI_GPIO_FSEL_INPT  = 0x00,   Input 0b000
   RPI_GPIO_FSEL_OUTP  = 0x01,   Output 0b001
   RPI_GPIO_FSEL_ALT0  = 0x04,   Alternate function 0 0b100
   RPI_GPIO_FSEL_ALT1  = 0x05,   Alternate function 1 0b101
   RPI_GPIO_FSEL_ALT2  = 0x06,   Alternate function 2 0b110
   RPI_GPIO_FSEL_ALT3  = 0x07,   Alternate function 3 0b111
   RPI_GPIO_FSEL_ALT4  = 0x03,   Alternate function 4 0b011
   RPI_GPIO_FSEL_ALT5  = 0x02,   Alternate function 5 0b010
   RPI_GPIO_FSEL_MASK  = 0x07    Function select bits mask 0b111 */

void rpi_gpio_fsel(uint8_t pin, uint8_t mode)
{
    /* Function selects are 10 pins per 32 bit word, 3 bits per pin */
    volatile uint32_t* paddr = rpi_gpio + RPI_GPFSEL0/4 + (pin/10);
    uint8_t   shift = (pin % 10) * 3;
    uint32_t  mask = RPI_GPIO_FSEL_MASK << shift;
    uint32_t  value = mode << shift;
    rpi_peri_set_bits(paddr, value, mask);
}

/* Set output pin */
void rpi_gpio_set(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPSET0/4 + pin/32;
    uint8_t shift = pin % 32;
    rpi_peri_write(paddr, 1 << shift);
}

/* Clear output pin */
void rpi_gpio_clr(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPCLR0/4 + pin/32;
    uint8_t shift = pin % 32;
    rpi_peri_write(paddr, 1 << shift);
}

/* Set all output pins in the mask */
void rpi_gpio_set_multi(uint32_t mask)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPSET0/4;
    rpi_peri_write(paddr, mask);
}

/* Clear all output pins in the mask */
void rpi_gpio_clr_multi(uint32_t mask)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPCLR0/4;
    rpi_peri_write(paddr, mask);
}

/* Read input pin */
uint8_t rpi_gpio_lev(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPLEV0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = rpi_peri_read(paddr);
    return (value & (1 << shift)) ? HIGH : LOW;
}

/* See if an event detection bit is set
// Sigh cant support interrupts yet
*/
uint8_t rpi_gpio_eds(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPEDS0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = rpi_peri_read(paddr);
    return (value & (1 << shift)) ? HIGH : LOW;
}

uint32_t rpi_gpio_eds_multi(uint32_t mask)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPEDS0/4;
    uint32_t value = rpi_peri_read(paddr);
    return (value & mask);
}

/* Write a 1 to clear the bit in EDS */
void rpi_gpio_set_eds(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPEDS0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_write(paddr, value);
}

void rpi_gpio_set_eds_multi(uint32_t mask)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPEDS0/4;
    rpi_peri_write(paddr, mask);
}

/* Rising edge detect enable */
void rpi_gpio_ren(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPREN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, value, value);
}
void rpi_gpio_clr_ren(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPREN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, 0, value);
}

/* Falling edge detect enable */
void rpi_gpio_fen(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPFEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, value, value);
}
void rpi_gpio_clr_fen(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPFEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, 0, value);
}

/* High detect enable */
void rpi_gpio_hen(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPHEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, value, value);
}
void rpi_gpio_clr_hen(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPHEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, 0, value);
}

/* Low detect enable */
void rpi_gpio_len(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPLEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, value, value);
}
void rpi_gpio_clr_len(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPLEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, 0, value);
}

/* Async rising edge detect enable */
void rpi_gpio_aren(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPAREN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, value, value);
}
void rpi_gpio_clr_aren(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPAREN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, 0, value);
}

/* Async falling edge detect enable */
void rpi_gpio_afen(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPAFEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, value, value);
}
void rpi_gpio_clr_afen(uint8_t pin)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPAFEN0/4 + pin/32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    rpi_peri_set_bits(paddr, 0, value);
}

/* Set pullup/down */
void rpi_gpio_pud(uint8_t pud)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPPUD/4;
    rpi_peri_write(paddr, pud);
}

/* Pullup/down clock
// Clocks the value of pud into the GPIO pin
*/
void rpi_gpio_pudclk(uint8_t pin, uint8_t on)
{
    volatile uint32_t* paddr = rpi_gpio + RPI_GPPUDCLK0/4 + pin/32;
    uint8_t shift = pin % 32;
    rpi_peri_write(paddr, (on ? 1 : 0) << shift);
}

/* Read GPIO pad behaviour for groups of GPIOs */
uint32_t rpi_gpio_pad(uint8_t group)
{
    if (rpi_pads == MAP_FAILED)
        return 0;

    volatile uint32_t* paddr = rpi_pads + RPI_PADS_GPIO_0_27/4 + group;
    return rpi_peri_read(paddr);
}

/* Set GPIO pad behaviour for groups of GPIOs
// powerup value for all pads is
// RPI_PAD_SLEW_RATE_UNLIMITED | RPI_PAD_HYSTERESIS_ENABLED | RPI_PAD_DRIVE_8mA
*/
void rpi_gpio_set_pad(uint8_t group, uint32_t control)
{
    if (rpi_pads == MAP_FAILED)
        return;

    volatile uint32_t* paddr = rpi_pads + RPI_PADS_GPIO_0_27/4 + group;
    rpi_peri_write(paddr, control | RPI_PAD_PASSWRD);
}

/* Some convenient arduino-like functions
// milliseconds
*/
void rpi_delay(unsigned int millis)
{
    struct timespec sleeper;

    sleeper.tv_sec  = (time_t)(millis / 1000);
    sleeper.tv_nsec = (long)(millis % 1000) * 1000000;
    nanosleep(&sleeper, NULL);
}

/* microseconds */
void rpi_delayMicroseconds(uint64_t micros)
{
    struct timespec t1;
    uint64_t        start;

    if (debug)
    {
        /* Cant access sytem timers in debug mode */
        printf("rpi_delayMicroseconds %lld\n", micros);
        return;
    }

    /* Calling nanosleep() takes at least 100-200 us, so use it for
    // long waits and use a busy wait on the System Timer for the rest.
    */
    start =  rpi_st_read();

    if (micros > 450)
    {
        t1.tv_sec = 0;
        t1.tv_nsec = 1000 * (long)(micros - 200);
        nanosleep(&t1, NULL);
    }

    rpi_st_delay(start, micros);
}

/*
// Higher level convenience functions
*/

/* Set the state of an output */
void rpi_gpio_write(uint8_t pin, uint8_t on)
{
    if (on)
        rpi_gpio_set(pin);
    else
        rpi_gpio_clr(pin);
}

/* Set the state of a all 32 outputs in the mask to on or off */
void rpi_gpio_write_multi(uint32_t mask, uint8_t on)
{
    if (on)
        rpi_gpio_set_multi(mask);
    else
        rpi_gpio_clr_multi(mask);
}

/* Set the state of a all 32 outputs in the mask to the values in value */
void rpi_gpio_write_mask(uint32_t value, uint32_t mask)
{
    rpi_gpio_set_multi(value & mask);
    rpi_gpio_clr_multi((~value) & mask);
}

/* Set the pullup/down resistor for a pin
//
// The GPIO Pull-up/down Clock Registers control the actuation of internal pull-downs on
// the respective GPIO pins. These registers must be used in conjunction with the GPPUD
// register to effect GPIO Pull-up/down changes. The following sequence of events is
// required:
// 1. Write to GPPUD to set the required control signal (i.e. Pull-up or Pull-Down or neither
// to remove the current Pull-up/down)
// 2. Wait 150 cycles ? this provides the required set-up time for the control signal
// 3. Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you wish to
// modify ? NOTE only the pads which receive a clock will be modified, all others will
// retain their previous state.
// 4. Wait 150 cycles ? this provides the required hold time for the control signal
// 5. Write to GPPUD to remove the control signal
// 6. Write to GPPUDCLK0/1 to remove the clock
//
// RPi has P1-03 and P1-05 with 1k8 pullup resistor
*/
void rpi_gpio_set_pud(uint8_t pin, uint8_t pud)
{
    rpi_gpio_pud(pud);
    delayMicroseconds(10);
    rpi_gpio_pudclk(pin, 1);
    delayMicroseconds(10);
    rpi_gpio_pud(RPI_GPIO_PUD_OFF);
    rpi_gpio_pudclk(pin, 0);
}

int rpi_spi_begin(void)
{
    volatile uint32_t* paddr;

    if (rpi_spi0 == MAP_FAILED)
        return 0; /* rpi_init() failed, or not root */

    /* Set the SPI0 pins to the Alt 0 function to enable SPI0 access on them */
    rpi_gpio_fsel(RPI_V3_GPIO_26, RPI_GPIO_FSEL_ALT0); /* CE1 */
    rpi_gpio_fsel(RPI_V3_GPIO_24, RPI_GPIO_FSEL_ALT0); /* CE0 */
    rpi_gpio_fsel(RPI_V3_GPIO_21, RPI_GPIO_FSEL_ALT0); /* MISO */
    rpi_gpio_fsel(RPI_V3_GPIO_19, RPI_GPIO_FSEL_ALT0); /* MOSI */
    rpi_gpio_fsel(RPI_V3_GPIO_23, RPI_GPIO_FSEL_ALT0); /* CLK */

    /* Set the SPI CS register to the some sensible defaults */
    paddr = rpi_spi0 + RPI_SPI0_CS/4;
    rpi_peri_write(paddr, 0); /* All 0s */

    /* Clear TX and RX fifos */
    rpi_peri_write_nb(paddr, RPI_SPI0_CS_CLEAR);

    return 1; // OK
}

void rpi_spi_end(void)
{
    /* Set all the SPI0 pins back to input */
    rpi_gpio_fsel(RPI_V3_GPIO_26, RPI_GPIO_FSEL_INPT); /* CE1 */
    rpi_gpio_fsel(RPI_V3_GPIO_24, RPI_GPIO_FSEL_INPT); /* CE0 */
    rpi_gpio_fsel(RPI_V3_GPIO_21, RPI_GPIO_FSEL_INPT); /* MISO */
    rpi_gpio_fsel(RPI_V3_GPIO_19, RPI_GPIO_FSEL_INPT); /* MOSI */
    rpi_gpio_fsel(RPI_V3_GPIO_23, RPI_GPIO_FSEL_INPT); /* CLK */
}

/* defaults to 0, which means a divider of 65536.
// The divisor must be a power of 2. Odd numbers
// rounded down. The maximum SPI clock rate is
// of the APB clock
RPI_SPI_CLOCK_DIVIDER_65536 = 0,       65536 = 262.144us = 3.814697260kHz
*/
void rpi_spi_setClockDivider(uint16_t divider)
{
    volatile uint32_t* paddr = rpi_spi0 + RPI_SPI0_CLK/4;
    rpi_peri_write(paddr, divider);
}

/* RPI_SPI_MODE0 = 0,  CPOL = 0, CPHA = 0 */
void rpi_spi_setDataMode(uint8_t mode)
{
    volatile uint32_t* paddr = rpi_spi0 + RPI_SPI0_CS/4;
    /* Mask in the CPO and CPHA bits of CS */
    rpi_peri_set_bits(paddr, mode << 2, RPI_SPI0_CS_CPOL | RPI_SPI0_CS_CPHA);
}

/* Writes (and reads) a single byte to SPI */
uint8_t rpi_spi_transfer(uint8_t value)
{
    volatile uint32_t* paddr = rpi_spi0 + RPI_SPI0_CS/4;
    volatile uint32_t* fifo = rpi_spi0 + RPI_SPI0_FIFO/4;
    uint32_t ret;

    /* This is Polled transfer as per section 10.6.1
    // BUG ALERT: what happens if we get interupted in this section, and someone else
    // accesses a different peripheral?
    // Clear TX and RX fifos
    */
    rpi_peri_set_bits(paddr, RPI_SPI0_CS_CLEAR, RPI_SPI0_CS_CLEAR);

    /* Set TA = 1 */
    rpi_peri_set_bits(paddr, RPI_SPI0_CS_TA, RPI_SPI0_CS_TA);

    /* Maybe wait for TXD */
    while (!(rpi_peri_read(paddr) & RPI_SPI0_CS_TXD))
        ;

    /* Write to FIFO, no barrier */
    rpi_peri_write_nb(fifo, value);

    /* Wait for DONE to be set */
    while (!(rpi_peri_read_nb(paddr) & RPI_SPI0_CS_DONE))
        ;

    /* Read any byte that was sent back by the slave while we sere sending to it */
    ret = rpi_peri_read_nb(fifo);

    /* Set TA = 0, and also set the barrier */
    rpi_peri_set_bits(paddr, 0, RPI_SPI0_CS_TA);

    return ret;
}

/* Writes (and reads) an number of bytes to SPI */
void rpi_spi_transfernb(char* tbuf, char* rbuf, uint32_t len)
{
    volatile uint32_t* paddr = rpi_spi0 + RPI_SPI0_CS/4;
    volatile uint32_t* fifo = rpi_spi0 + RPI_SPI0_FIFO/4;
    uint32_t TXCnt=0;
    uint32_t RXCnt=0;

    /* This is Polled transfer as per section 10.6.1
    // BUG ALERT: what happens if we get interupted in this section, and someone else
    // accesses a different peripheral?
    */

    /* Clear TX and RX fifos */
    rpi_peri_set_bits(paddr, RPI_SPI0_CS_CLEAR, RPI_SPI0_CS_CLEAR);

    /* Set TA = 1 */
    rpi_peri_set_bits(paddr, RPI_SPI0_CS_TA, RPI_SPI0_CS_TA);

    /* Use the FIFO's to reduce the interbyte times */
    while((TXCnt < len)||(RXCnt < len))
    {
        /* TX fifo not full, so add some more bytes */
        while(((rpi_peri_read(paddr) & RPI_SPI0_CS_TXD))&&(TXCnt < len ))
        {
            rpi_peri_write_nb(fifo, tbuf[TXCnt]);
            TXCnt++;
        }
        /* Rx fifo not empty, so get the next received bytes */
        while(((rpi_peri_read(paddr) & RPI_SPI0_CS_RXD))&&( RXCnt < len ))
        {
            rbuf[RXCnt] = rpi_peri_read_nb(fifo);
            RXCnt++;
        }
    }
    /* Wait for DONE to be set */
    while (!(rpi_peri_read_nb(paddr) & RPI_SPI0_CS_DONE))
        ;

    /* Set TA = 0, and also set the barrier */
    rpi_peri_set_bits(paddr, 0, RPI_SPI0_CS_TA);
}

/* Writes an number of bytes to SPI */
void rpi_spi_writenb(char* tbuf, uint32_t len)
{
    volatile uint32_t* paddr = rpi_spi0 + RPI_SPI0_CS/4;
    volatile uint32_t* fifo = rpi_spi0 + RPI_SPI0_FIFO/4;
    uint32_t i;

    /* This is Polled transfer as per section 10.6.1
    // BUG ALERT: what happens if we get interupted in this section, and someone else
    // accesses a different peripheral?
    // Answer: an ISR is required to issue the required memory barriers.
    */

    /* Clear TX and RX fifos */
    rpi_peri_set_bits(paddr, RPI_SPI0_CS_CLEAR, RPI_SPI0_CS_CLEAR);

    /* Set TA = 1 */
    rpi_peri_set_bits(paddr, RPI_SPI0_CS_TA, RPI_SPI0_CS_TA);

    for (i = 0; i < len; i++)
    {
        /* Maybe wait for TXD */
        while (!(rpi_peri_read(paddr) & RPI_SPI0_CS_TXD))
            ;

        /* Write to FIFO, no barrier */
        rpi_peri_write_nb(fifo, tbuf[i]);

        /* Read from FIFO to prevent stalling */
        while (rpi_peri_read(paddr) & RPI_SPI0_CS_RXD)
            (void) rpi_peri_read_nb(fifo);
    }

    /* Wait for DONE to be set */
    while (!(rpi_peri_read_nb(paddr) & RPI_SPI0_CS_DONE)) {
        while (rpi_peri_read(paddr) & RPI_SPI0_CS_RXD)
            (void) rpi_peri_read_nb(fifo);
    };

    /* Set TA = 0, and also set the barrier */
    rpi_peri_set_bits(paddr, 0, RPI_SPI0_CS_TA);
}

/* Writes (and reads) an number of bytes to SPI
// Read bytes are copied over onto the transmit buffer
*/
void rpi_spi_transfern(char* buf, uint32_t len)
{
    rpi_spi_transfernb(buf, buf, len);
}

/*     RPI_SPI_CS0 = 0,  Chip Select 0 */
void rpi_spi_chipSelect(uint8_t cs)
{
    volatile uint32_t* paddr = rpi_spi0 + RPI_SPI0_CS/4;
    /* Mask in the CS bits of CS */
    rpi_peri_set_bits(paddr, cs, RPI_SPI0_CS_CS);
}

void rpi_spi_setChipSelectPolarity(uint8_t cs, uint8_t active)
{
    volatile uint32_t* paddr = rpi_spi0 + RPI_SPI0_CS/4;
    uint8_t shift = 21 + cs;
    /* Mask in the appropriate CSPOLn bit */
    rpi_peri_set_bits(paddr, active << shift, 1 << shift);
}

int rpi_i2c_init(void)
{
    uint16_t cdiv;

    if (   rpi_bsc0 == MAP_FAILED
           || rpi_bsc1 == MAP_FAILED)
        return 0; /* rpi_init() failed, or not root */

    volatile uint32_t* paddr = rpi_bsc1 + RPI_BSC_DIV/4;
    /* Set the I2C/BSC1 pins to the Alt 0 function to enable I2C access on them */
    rpi_gpio_fsel(RPI_V3_GPIO_03, RPI_GPIO_FSEL_ALT0); /* SDA */
    rpi_gpio_fsel(RPI_V3_GPIO_05, RPI_GPIO_FSEL_ALT0); /* SCL */

    /* Read the clock divider register */
    cdiv = rpi_peri_read(paddr);
    /* Calculate time for transmitting one byte
    // 1000000 = micros seconds in a second
    // 9 = Clocks per byte : 8 bits + ACK
    */
    i2c_byte_wait_us = ((float)cdiv / RPI_CORE_CLK_HZ) * 1000000 * 9;

    return 1;
}

void rpi_i2c_close(void)
{
    /* Set all the I2C/BSC1 pins back to input */
    rpi_gpio_fsel(RPI_V3_GPIO_03, RPI_GPIO_FSEL_INPT); /* SDA */
    rpi_gpio_fsel(RPI_V3_GPIO_05, RPI_GPIO_FSEL_INPT); /* SCL */
}

void rpi_i2c_setSlaveAddress(uint8_t addr)
{
    /* Set I2C Device Address */
    volatile uint32_t* paddr = rpi_bsc1 + RPI_BSC_A/4;
    rpi_peri_write(paddr, addr);
}

/* defaults to 0x5dc, should result in a 166.666 kHz I2C clock frequency.
// The divisor must be a power of 2. Odd numbers
// rounded down.

RPI_I2C_CLOCK_DIVIDER_2500   = 2500,      2500 = 10us = 100 kHz
RPI_I2C_CLOCK_DIVIDER_626    = 626,       622 = 2.504us = 399.3610 kHz
RPI_I2C_CLOCK_DIVIDER_150    = 150,       150 = 60ns = 1.666 MHz (default at reset)
RPI_I2C_CLOCK_DIVIDER_148    = 148        148 = 59ns = 1.689 MHz

*/
void rpi_i2c_setClockDivider(uint16_t divider)
{

    volatile uint32_t* paddr = rpi_bsc1 + RPI_BSC_DIV/4;
    rpi_peri_write(paddr, divider);
    /* Calculate time for transmitting one byte
    // 1000000 = micros seconds in a second
    // 9 = Clocks per byte : 8 bits + ACK
    */
    i2c_byte_wait_us = ((float)divider / RPI_CORE_CLK_HZ) * 1000000 * 9;
}

/* set I2C clock divider by means of a baudrate number */
void rpi_i2c_set_baudrate(uint32_t baudrate)
{
	uint32_t divider;
	/* use 0xFFFE mask to limit a max value and round down any odd number */
	divider = (RPI_CORE_CLK_HZ / baudrate) & 0xFFFE;
	rpi_i2c_setClockDivider( (uint16_t)divider );
}

/* Writes an number of bytes to I2C */
uint8_t rpi_i2c_write(const char * buf, uint32_t len)
{
    volatile uint32_t* dlen    = rpi_bsc1 + RPI_BSC_DLEN/4;
    volatile uint32_t* fifo    = rpi_bsc1 + RPI_BSC_FIFO/4;
    volatile uint32_t* status  = rpi_bsc1 + RPI_BSC_S/4;
    volatile uint32_t* control = rpi_bsc1 + RPI_BSC_C/4;

    uint32_t remaining = len;
    uint32_t i = 0;
    uint8_t reason = RPI_I2C_REASON_OK;

    /* Clear FIFO */
    rpi_peri_set_bits(control, RPI_BSC_C_CLEAR_1 , RPI_BSC_C_CLEAR_1 );
    /* Clear Status */
    rpi_peri_write(status, RPI_BSC_S_CLKT | RPI_BSC_S_ERR | RPI_BSC_S_DONE);
    /* Set Data Length */
    rpi_peri_write(dlen, len);
    /* pre populate FIFO with max buffer */
    while( remaining && ( i < RPI_BSC_FIFO_SIZE ) )
    {
        rpi_peri_write_nb(fifo, buf[i]);
        i++;
        remaining--;
    }

    /* Enable device and start transfer */
    rpi_peri_write(control, RPI_BSC_C_I2CEN | RPI_BSC_C_ST);

    /* Transfer is over when RPI_BSC_S_DONE */
    while(!(rpi_peri_read(status) & RPI_BSC_S_DONE ))
    {
        while ( remaining && (rpi_peri_read(status) & RPI_BSC_S_TXD ))
    	{
            /* Write to FIFO */
            rpi_peri_write(fifo, buf[i]);
            i++;
            remaining--;
    	}
    }

    /* Received a NACK */
    if (rpi_peri_read(status) & RPI_BSC_S_ERR)
    {
        reason = RPI_I2C_REASON_ERROR_NACK;
    }

    /* Received Clock Stretch Timeout */
    else if (rpi_peri_read(status) & RPI_BSC_S_CLKT)
    {
        reason = RPI_I2C_REASON_ERROR_CLKT;
    }

    /* Not all data is sent */
    else if (remaining)
    {
        reason = RPI_I2C_REASON_ERROR_DATA;
        /* Clear FIFO */
        rpi_peri_set_bits(control, RPI_BSC_C_CLEAR_1 , RPI_BSC_C_CLEAR_1 );
    }

    rpi_peri_set_bits(status, RPI_BSC_S_DONE , RPI_BSC_S_DONE);

    return reason;
}

/* Read an number of bytes from I2C */
uint8_t rpi_i2c_read(char* buf, uint32_t len)
{
    volatile uint32_t* dlen    = rpi_bsc1 + RPI_BSC_DLEN/4;
    volatile uint32_t* fifo    = rpi_bsc1 + RPI_BSC_FIFO/4;
    volatile uint32_t* status  = rpi_bsc1 + RPI_BSC_S/4;
    volatile uint32_t* control = rpi_bsc1 + RPI_BSC_C/4;

    uint32_t remaining = len;
    uint32_t i = 0;
    uint8_t reason = RPI_I2C_REASON_OK;

    /* Clear FIFO */
    rpi_peri_set_bits(control, RPI_BSC_C_CLEAR_1 , RPI_BSC_C_CLEAR_1 );
    /* Clear Status */
    rpi_peri_write_nb(status, RPI_BSC_S_CLKT | RPI_BSC_S_ERR | RPI_BSC_S_DONE);
    /* Set Data Length */
    rpi_peri_write_nb(dlen, len);
    /* Start read */
    rpi_peri_write_nb(control, RPI_BSC_C_I2CEN | RPI_BSC_C_ST | RPI_BSC_C_READ);

    /* wait for transfer to complete */
    while (!(rpi_peri_read_nb(status) & RPI_BSC_S_DONE))
    {
        /* we must empty the FIFO as it is populated and not use any delay */
        while (rpi_peri_read_nb(status) & RPI_BSC_S_RXD)
    	{
            /* Read from FIFO, no barrier */
            buf[i] = rpi_peri_read_nb(fifo);
            i++;
            remaining--;
    	}
    }

    /* transfer has finished - grab any remaining stuff in FIFO */
    while (remaining && (rpi_peri_read_nb(status) & RPI_BSC_S_RXD))
    {
        /* Read from FIFO, no barrier */
        buf[i] = rpi_peri_read_nb(fifo);
        i++;
        remaining--;
    }

    /* Received a NACK */
    if (rpi_peri_read(status) & RPI_BSC_S_ERR)
    {
        reason = RPI_I2C_REASON_ERROR_NACK;
    }

    /* Received Clock Stretch Timeout */
    else if (rpi_peri_read(status) & RPI_BSC_S_CLKT)
    {
        reason = RPI_I2C_REASON_ERROR_CLKT;
    }

    /* Not all data is received */
    else if (remaining)
    {
        reason = RPI_I2C_REASON_ERROR_DATA;
    }

    rpi_peri_set_bits(status, RPI_BSC_S_DONE , RPI_BSC_S_DONE);

    return reason;
}

/* Sending an arbitrary number of bytes before issuing a repeated start
// (with no prior stop) and reading a response. Some devices require this behavior.
*/
uint8_t rpi_i2c_write_read_rs(char* cmds, uint32_t cmds_len, char* buf, uint32_t buf_len)
{

    volatile uint32_t* dlen    = rpi_bsc1 + RPI_BSC_DLEN/4;
    volatile uint32_t* fifo    = rpi_bsc1 + RPI_BSC_FIFO/4;
    volatile uint32_t* status  = rpi_bsc1 + RPI_BSC_S/4;
    volatile uint32_t* control = rpi_bsc1 + RPI_BSC_C/4;

    uint32_t remaining = cmds_len;
    uint32_t i = 0;
    uint8_t reason = RPI_I2C_REASON_OK;

    /* Clear FIFO */
    rpi_peri_set_bits(control, RPI_BSC_C_CLEAR_1 , RPI_BSC_C_CLEAR_1 );

    /* Clear Status */
    rpi_peri_write(status, RPI_BSC_S_CLKT | RPI_BSC_S_ERR | RPI_BSC_S_DONE);

    /* Set Data Length */
    rpi_peri_write(dlen, cmds_len);

    /* pre populate FIFO with max buffer */
    while( remaining && ( i < RPI_BSC_FIFO_SIZE ) )
    {
        rpi_peri_write_nb(fifo, cmds[i]);
        i++;
        remaining--;
    }

    /* Enable device and start transfer */
    rpi_peri_write(control, RPI_BSC_C_I2CEN | RPI_BSC_C_ST);

    /* poll for transfer has started (way to do repeated start, from RPI datasheet) */
    while ( !( rpi_peri_read(status) & RPI_BSC_S_TA ) )
    {
        /* Linux may cause us to miss entire transfer stage */
        if(rpi_peri_read_nb(status) & RPI_BSC_S_DONE)
            break;
    }

    remaining = buf_len;
    i = 0;

    /* Send a repeated start with read bit set in address */
    rpi_peri_write(dlen, buf_len);
    rpi_peri_write(control, RPI_BSC_C_I2CEN | RPI_BSC_C_ST  | RPI_BSC_C_READ );

    /* Wait for write to complete and first byte back. */
    rpi_delayMicroseconds(i2c_byte_wait_us * (cmds_len + 1));

    /* wait for transfer to complete */
    while (!(rpi_peri_read_nb(status) & RPI_BSC_S_DONE))
    {
        /* we must empty the FIFO as it is populated and not use any delay */
        while (remaining && rpi_peri_read(status) & RPI_BSC_S_RXD)
    	{
            /* Read from FIFO, no barrier */
            buf[i] = rpi_peri_read_nb(fifo);
            i++;
            remaining--;
    	}
    }

    /* transfer has finished - grab any remaining stuff in FIFO */
    while (remaining && (rpi_peri_read(status) & RPI_BSC_S_RXD))
    {
        /* Read from FIFO */
        buf[i] = rpi_peri_read(fifo);
        i++;
        remaining--;
    }

    /* Received a NACK */
    if (rpi_peri_read(status) & RPI_BSC_S_ERR)
    {
        reason = RPI_I2C_REASON_ERROR_NACK;
    }

    /* Received Clock Stretch Timeout */
    else if (rpi_peri_read(status) & RPI_BSC_S_CLKT)
    {
        reason = RPI_I2C_REASON_ERROR_CLKT;
    }

    /* Not all data is sent */
    else if (remaining)
    {
        reason = RPI_I2C_REASON_ERROR_DATA;
    }

    rpi_peri_set_bits(control, RPI_BSC_S_DONE , RPI_BSC_S_DONE);

    return reason;
}

/* Read an number of bytes from I2C sending a repeated start after writing
// the required register. Only works if your device supports this mode
*/
uint8_t rpi_i2c_read_register_rs(char* regaddr, char* buf, uint32_t len)
{
    return rpi_i2c_write_read_rs(regaddr,1,buf,len);
}

/* Read the System Timer Counter (64-bits) */
uint64_t rpi_st_read(void)
{
    volatile uint32_t* paddr;
    uint32_t hi, lo;
    uint64_t st;
    paddr = rpi_st + RPI_ST_CHI/4;
    hi = rpi_peri_read(paddr);

    paddr = rpi_st + RPI_ST_CLO/4;
    lo = rpi_peri_read(paddr);

    paddr = rpi_st + RPI_ST_CHI/4;
    st = rpi_peri_read(paddr);

    /* Test for overflow */
    if (st == hi)
    {
        st <<= 32;
        st += lo;
    }
    else
    {
        st <<= 32;
        paddr = rpi_st + RPI_ST_CLO/4;
        st += rpi_peri_read(paddr);
    }
    return st;
}

/* Delays for the specified number of microseconds with offset */
void rpi_st_delay(uint64_t offset_micros, uint64_t micros)
{
    uint64_t compare = offset_micros + micros;

    while(rpi_st_read() < compare)
        ;
}

/* PWM */

void rpi_pwm_init()
{
    rpi_gpio_fsel(RPI_V3_GPIO_12, RPI_GPIO_FSEL_ALT5);
}
void rpi_pwm_close()
{
    rpi_gpio_fsel(RPI_V3_GPIO_12, RPI_GPIO_FSEL_INPT);
}

void rpi_pwm_set_clock(uint32_t divisor)
{
    if (   rpi_clk == MAP_FAILED
           || rpi_pwm == MAP_FAILED)
        return; /* rpi_init() failed or not root */

    /* From Gerts code */
    divisor &= 0xfff;
    /* Stop PWM clock */
    rpi_peri_write(rpi_clk + RPI_PWMCLK_CNTL, RPI_PWM_PASSWRD | 0x01);
    rpi_delay(110); /* Prevents clock going slow */
    /* Wait for the clock to be not busy */
    while ((rpi_peri_read(rpi_clk + RPI_PWMCLK_CNTL) & 0x80) != 0)
        rpi_delay(1);
    /* set the clock divider and enable PWM clock */
    rpi_peri_write(rpi_clk + RPI_PWMCLK_DIV, RPI_PWM_PASSWRD | (divisor << 12));
    rpi_peri_write(rpi_clk + RPI_PWMCLK_CNTL, RPI_PWM_PASSWRD | 0x11); /* Source=osc and enable */
}

void rpi_pwm_set_mode(uint8_t markspace, uint8_t enabled)
{
    if (   rpi_clk == MAP_FAILED
           || rpi_pwm == MAP_FAILED)
        return; /* rpi_init() failed or not root */

    uint32_t control = rpi_peri_read(rpi_pwm + RPI_PWM_CONTROL);

    if (markspace)
        control |= RPI_PWM0_MS_MODE;
    else
        control &= ~RPI_PWM0_MS_MODE;
    if (enabled)
        control |= RPI_PWM0_ENABLE;
    else
        control &= ~RPI_PWM0_ENABLE;
    /* If you use the barrier here, wierd things happen, and the commands dont work */
    rpi_peri_write_nb(rpi_pwm + RPI_PWM_CONTROL, control);
    /*  rpi_peri_write_nb(rpi_pwm + RPI_PWM_CONTROL, RPI_PWM0_ENABLE | RPI_PWM1_ENABLE | RPI_PWM0_MS_MODE | RPI_PWM1_MS_MODE); */

}

void rpi_pwm_set_range(uint32_t range)
{
    if (   rpi_clk == MAP_FAILED
           || rpi_pwm == MAP_FAILED)
        return; /* rpi_init() failed or not root */
    rpi_peri_write_nb(rpi_pwm + RPI_PWM0_RANGE, range);
}

void rpi_pwm_set_data(uint32_t data)
{
    if (   rpi_clk == MAP_FAILED
           || rpi_pwm == MAP_FAILED)
        return; /* rpi_init() failed or not root */
    rpi_peri_write_nb(rpi_pwm + RPI_PWM0_DATA, data);
}

/* Map 'size' bytes starting at 'off' in file 'fd' to memory.
// Return mapped address on success, MAP_FAILED otherwise.
// On error print message.
*/
static void *mapmem(const char *msg, size_t size, int fd, off_t off)
{
    void *map = mmap(NULL, size, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, off);
    if (map == MAP_FAILED)
        fprintf(stderr, "rpi_init: %s mmap failed: %s\n", msg, strerror(errno));
    return map;
}

static void unmapmem(void **pmem, size_t size)
{
    if (*pmem == MAP_FAILED) return;
    munmap(*pmem, size);
    *pmem = MAP_FAILED;
}

/* Initialise this library. */
int rpi_init(void)
{
    int  memfd;
    int  ok;
    FILE *fp;

    if (debug)
    {
        rpi_peripherals = (uint32_t*)RPI_PERI_BASE;
        rpi_pads = rpi_peripherals + RPI_GPIO_PADS/4;
        rpi_clk  = rpi_peripherals + RPI_CLOCK_BASE/4;
        rpi_gpio = rpi_peripherals + RPI_GPIO_BASE/4;
        rpi_pwm  = rpi_peripherals + RPI_GPIO_PWM/4;
        rpi_spi0 = rpi_peripherals + RPI_SPI0_BASE/4;
        rpi_bsc0 = rpi_peripherals + RPI_BSC0_BASE/4;
        rpi_bsc1 = rpi_peripherals + RPI_BSC1_BASE/4;
        rpi_st   = rpi_peripherals + RPI_ST_BASE/4;
        return 1; /* Success */
    }

    /* Figure out the base and size of the peripheral address block
    // using the device-tree. Required for RPi2 and RPi3, optional for RPi 1
    */
    if ((fp = fopen(BCM2835_RPI2_DT_FILENAME , "rb")))
    {
        unsigned char buf[4];
        fseek(fp, BCM2835_RPI2_DT_PERI_BASE_ADDRESS_OFFSET, SEEK_SET);
        if (fread(buf, 1, sizeof(buf), fp) == sizeof(buf))
            rpi_peripherals_base = (uint32_t *)(buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3] << 0);
        fseek(fp, BCM2835_RPI2_DT_PERI_SIZE_OFFSET, SEEK_SET);
        if (fread(buf, 1, sizeof(buf), fp) == sizeof(buf))
            rpi_peripherals_size = (buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3] << 0);
        printf("rpi_peripherals_base:%p\n",(rpi_peripherals_base));
        printf("rpi_peripherals_size:%d\n",(rpi_peripherals_size));
        fclose(fp);
    }
    /* Now get ready to map the peripherals block
     * If we are not root, try for the new /dev/gpiomem interface and accept
     * the fact that we can only access GPIO
     * else try for the /dev/mem interface and get access to everything
     */
    memfd = -1;
    ok = 0;
    if (geteuid() == 0)
    {
        /* Open the master /dev/mem device */
        if ((memfd = open("/dev/mem", O_RDWR | O_SYNC) ) < 0)
        {
            fprintf(stderr, "rpi_init: Unable to open /dev/mem: %s\n",
                    strerror(errno)) ;
            goto exit;
        }

        /* Base of the peripherals block is mapped to VM */
        rpi_peripherals = mapmem("gpio", rpi_peripherals_size, memfd, (uint32_t)rpi_peripherals_base);
        if (rpi_peripherals == MAP_FAILED) goto exit;

        /* Now compute the base addresses of various peripherals,
        // which are at fixed offsets within the mapped peripherals block
        // Caution: rpi_peripherals is uint32_t*, so divide offsets by 4
        */
        rpi_interrupt = rpi_peripherals + RPI_INTERRUPT_BASE/4;
        rpi_gpio = rpi_peripherals + RPI_GPIO_BASE/4;
        rpi_pwm  = rpi_peripherals + RPI_GPIO_PWM/4;
        rpi_clk  = rpi_peripherals + RPI_CLOCK_BASE/4;
        rpi_pads = rpi_peripherals + RPI_GPIO_PADS/4;
        rpi_spi0 = rpi_peripherals + RPI_SPI0_BASE/4;
        rpi_bsc0 = rpi_peripherals + RPI_BSC0_BASE/4; /* I2C */
        rpi_bsc1 = rpi_peripherals + RPI_BSC1_BASE/4; /* I2C */
        rpi_st   = rpi_peripherals + RPI_ST_BASE/4;

        ok = 1;
    }
    else
    {
        /* Not root, try /dev/gpiomem */
        /* Open the master /dev/mem device */
        if ((memfd = open("/dev/gpiomem", O_RDWR | O_SYNC) ) < 0)
        {
            fprintf(stderr, "rpi_init: Unable to open /dev/gpiomem: %s\n",
                    strerror(errno)) ;
            goto exit;
        }

        /* Base of the peripherals block is mapped to VM */
        rpi_peripherals_base = 0;
        rpi_peripherals = mapmem("gpio", rpi_peripherals_size, memfd, (uint32_t)rpi_peripherals_base);
        if (rpi_peripherals == MAP_FAILED) goto exit;
        rpi_gpio = rpi_peripherals;
        ok = 1;
    }

  exit:
    if (memfd >= 0)
        close(memfd);

    if (!ok)
        rpi_close();

    return ok;
}

/* Close this library and deallocate everything */
int rpi_close(void)
{
    if (debug) return 1; /* Success */

    unmapmem((void**) &rpi_peripherals, rpi_peripherals_size);
    rpi_peripherals = MAP_FAILED;
    rpi_interrupt = MAP_FAILED;
    rpi_gpio = MAP_FAILED;
    rpi_pwm  = MAP_FAILED;
    rpi_clk  = MAP_FAILED;
    rpi_pads = MAP_FAILED;
    rpi_spi0 = MAP_FAILED;
    rpi_bsc0 = MAP_FAILED;
    rpi_bsc1 = MAP_FAILED;
    rpi_st   = MAP_FAILED;
    return 1; /* Success */
}

#ifdef RPI_TEST
/* this is a simple test program that prints out what it will do rather than
// actually doing it
*/
int main(int argc, char **argv)
{
    /* Be non-destructive */
    rpi_set_debug(1);

    if (!rpi_init())
        return 1;

    /* Configure some GPIO pins fo some testing
    // Set RPI pin P1-11 to be an output
    */
    rpi_gpio_fsel(RPI_V3_GPIO_11, RPI_GPIO_FSEL_OUTP);
    /* Set RPI pin P1-15 to be an input */
    rpi_gpio_fsel(RPI_V3_GPIO_15, RPI_GPIO_FSEL_INPT);
    /*  with a pullup */
    rpi_gpio_set_pud(RPI_V3_GPIO_15, RPI_GPIO_PUD_UP);
    /* And a low detect enable */
    rpi_gpio_len(RPI_V3_GPIO_15);
    /* and input hysteresis disabled on GPIOs 0 to 27 */
    rpi_gpio_set_pad(RPI_PAD_GROUP_GPIO_0_27, RPI_PAD_SLEW_RATE_UNLIMITED|RPI_PAD_DRIVE_8mA);

#if 1
    /* Blink */
    while (1)
    {
        /* Turn it on */
        rpi_gpio_write(RPI_V3_GPIO_11, HIGH);

        /* wait a bit */
        rpi_delay(500);

        /* turn it off */
        rpi_gpio_write(RPI_V3_GPIO_11, LOW);

        /* wait a bit */
        rpi_delay(500);
    }
#endif

#if 0
    /* Read input */
    while (1)
    {
        /* Read some data */
        uint8_t value = rpi_gpio_lev(RPI_V3_GPIO_15);
        printf("read from pin 15: %d\n", value);

        /* wait a bit */
        rpi_delay(500);
    }
#endif

#if 0
    /* Look for a low event detection
    // eds will be set whenever pin 15 goes low
    */
    while (1)
    {
        if (rpi_gpio_eds(RPI_V3_GPIO_15))
        {
            /* Now clear the eds flag by setting it to 1 */
            rpi_gpio_set_eds(RPI_V3_GPIO_15);
            printf("low event detect for pin 15\n");
        }

        /* wait a bit */
        rpi_delay(500);
    }
#endif

    if (!rpi_close())
        return 1;

    return 0;
}
#endif
