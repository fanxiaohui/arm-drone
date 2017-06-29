#pragma once

#include "utils.h"

#include <stm32f0xx.h>

// data structure to store NSS software PIN details
struct spi_nss_st {
    GPIO_TypeDef *port;		// user to set
    unsigned int  pin;		// user to set
};

typedef struct spi_nss_st spi_nss_t;

/*----------------------------------------------------------------------
 * Public interface
 *----------------------------------------------------------------------*/

extern void spi_nss_init(const spi_nss_t *nss);
extern void spi_init();

INLINE void spi_begin_trans(const spi_nss_t *nss);
INLINE void spi_end_trans(const spi_nss_t *nss);
INLINE unsigned int spi_txrx_byte(unsigned int out);
INLINE unsigned int spi_txrx_byte_trans(const spi_nss_t *nss, unsigned int out);
extern void spi_tx_buf(const char *out, unsigned int out_size);
extern void spi_rx_buf(char *in, unsigned int in_size);


/*----------------------------------------------------------------------
 * Inline definitions
 *----------------------------------------------------------------------*/

INLINE void spi_begin_trans(const spi_nss_t *nss)
{
    // mark start of transmission by setting NSS pin to low
    nss->port->ODR &= ~(1 << nss->pin);
}

INLINE void spi_end_trans(const spi_nss_t *nss)
{
    nss->port->ODR |= 1 << nss->pin;
}

INLINE unsigned int spi_txrx_byte(unsigned int out)
{
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = out;

    while (!(SPI1->SR & SPI_SR_RXNE));
    unsigned int in = SPI1->DR;

    // wait until SPI is no longer busy
    while (SPI1->SR & SPI_SR_BSY);

    return in;
}

INLINE unsigned int spi_txrx_byte_trans(const spi_nss_t *nss, unsigned int out)
{
    spi_begin_trans(nss);
    unsigned int rv = spi_txrx_byte(out);
    spi_end_trans(nss);

    return rv;
}
