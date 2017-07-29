
#include "spi.h"
#include "utils.h"

#include <stm32f0xx_ll_gpio.h>

void spi_init()
{
    // reset SPI
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

    // set up alternate functions for SP1 pins
    // TBD: PA4 - SPI1_NSS (must be pulled high in case of software NSS)
    // PA5 - SPI1_SCK
    // PA6 - SPI1_MISO
    // PA7 - SPI1_MOSI
                  
    gpio_set_af_mode(GPIOA, 5, LL_GPIO_AF_0);
    gpio_set_af_mode(GPIOA, 6, LL_GPIO_AF_0);
    gpio_set_af_mode(GPIOA, 7, LL_GPIO_AF_0);

    // set up baud rate - PCLK/2
    SPI1->CR1 &= ~SPI_CR1_BR;

    // enable software NSS pin management
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    
    // set up master configuration
    SPI1->CR1 |= SPI_CR1_MSTR;
    // set up 8 bit data size
    SPI1->CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
    // report non-empty receive FIFO if there is at least one byte in buffer
    SPI1->CR2 |= SPI_CR2_FRXTH;

    // enable SPI
    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_nss_init(const spi_nss_t *nss)
{
    gpio_set_mode(nss->port, nss->pin, LL_GPIO_MODE_OUTPUT);

    // idle state of NSS is high
    nss->port->ODR |= 1 << nss->pin;
}

void spi_tx_buf(const char *out, unsigned int out_size)
{
    while (out_size-- > 0) {
        while (!(SPI1->SR & SPI_SR_TXE));

        MMIO8(SPI1->DR) = *out++;

        while (!(SPI1->SR & SPI_SR_RXNE));
        
        // receive unused byte
        unsigned char c =  MMIO8(SPI1->DR);
    }

    // wait until SPI is no longer busy
    while (SPI1->SR & SPI_SR_BSY);
}

void spi_rx_buf(char *in, unsigned int in_size)
{
    while (in_size-- > 0) {
        while (!(SPI1->SR & SPI_SR_TXE));
        // send dummy byte
        MMIO8(SPI1->DR) = 0x00;

        while (!(SPI1->SR & SPI_SR_RXNE));
        *in++ = MMIO8(SPI1->DR);
    }

    // wait until SPI is no longer busy
    while (SPI1->SR & SPI_SR_BSY);
}
