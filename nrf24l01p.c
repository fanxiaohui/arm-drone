
#include "nrf24l01p.h"
#include "exti.h"

#include <stm32f0xx_ll_gpio.h>

/*----------------------------------------------------------------------
 * nRF24L01+ commands
 *----------------------------------------------------------------------*/

#define CMD_R_REGISTER		0x00
#define CMD_W_REGISTER		0x20
#define CMD_R_RX_PAYLOAD	0x61
#define CMD_W_TX_PAYLOAD	0xA0
#define CMD_FLUSH_TX		0xE1
#define CMD_FLUSH_RX		0xE2
#define CMD_REUSE_TX_PL		0xE3
#define CMD_W_ACK_PAYLOAD	0xA8
#define CMD_W_TX_PAYLOAD_NO_ACK	0xB0
#define CMD_NOP			0xFF

/*----------------------------------------------------------------------
 * nRF24L01+ registers
 *----------------------------------------------------------------------*/

#define REG_CONFIG	0x00
#define REG_EN_AA	0x01
#define REG_EN_RXADDR	0x02
#define REG_SETUP_AW	0x03
#define REG_SETUP_RETR	0x04
#define REG_RF_CH	0x05
#define REG_RF_SETUP	0x06
#define REG_STATUS	0x07
#define REG_OBSERVE_TX	0x08
#define REG_RPD		0x09
#define REG_RX_ADDR_P0	0x0A
#define REG_RX_ADDR_P1	0x0B
#define REG_RX_ADDR_P2	0x0C
#define REG_RX_ADDR_P3	0x0D
#define REG_RX_ADDR_P4	0x0E
#define REG_RX_ADDR_P5	0x0F
#define REG_TX_ADDR	0x10
#define REG_RX_PW_P0	0x11
#define REG_RX_PW_P1	0x12
#define REG_RX_PW_P2	0x13
#define REG_RX_PW_P3	0x14
#define REG_RX_PW_P4	0x15
#define REG_RX_PW_P5	0x16
#define REG_FIFO_STATUS	0x17
#define REG_DYNPD	0x1C

// config register bits
#define REG_CONFIG_PRIM_RX	(1 << 0)
#define REG_CONFIG_PWR_UP	(1 << 1)

// interrupt handler for packet receipt notifications
static void nrf24l_irq_handler(unsigned int irqn, void *client_data);

void nrf24l_init(nrf24l_t *nrf24l)
{
    gpio_set_mode(nrf24l->irq_port, nrf24l->irq_pin, LL_GPIO_MODE_INPUT);
    gpio_set_mode(nrf24l->ce_port, nrf24l->ce_pin, LL_GPIO_MODE_OUTPUT);
    
    spi_nss_init(&nrf24l->nss);

    exti_irq_handler_init(&nrf24l->irq_handler, &nrf24l_irq_handler, nrf24l);
    exti_register(&nrf24l->irq_handler, nrf24l->irq_pin);

    // enable interrupt line on IRQ pin, we assume that no other port will have
    // an input pin with the same pin number.
    // raise interrupt only on falling edge
    EXTI->RTSR &= ~(1 << nrf24l->irq_pin);
    EXTI->FTSR |= 1 << nrf24l->irq_pin;
    EXTI->IMR |= 1 << nrf24l->irq_pin;

    // power up radio
    spi_begin_trans(&nrf24l->nss);
    spi_txrx_byte(CMD_W_REGISTER + REG_CONFIG);
    spi_txrx_byte(REG_CONFIG_PWR_UP);
    spi_end_trans(&nrf24l->nss);

    // set up 4 byte addresses
    spi_begin_trans(&nrf24l->nss);
    spi_txrx_byte(CMD_W_REGISTER + REG_SETUP_AW);
    spi_txrx_byte(0b10);
    spi_end_trans(&nrf24l->nss);
}

void nrf24l_set_rf_ch(nrf24l_t *nrf24l, unsigned int channel)
{
    spi_begin_trans(&nrf24l->nss);
    spi_txrx_byte(CMD_W_REGISTER + REG_RF_CH);
    spi_txrx_byte(channel);
    spi_end_trans(&nrf24l->nss);
}

void nrf24l_set_rf_pwr(nrf24l_t *nrf24l, nrf24l_power_t power)
{
    unsigned rf_setup = spi_txrx_byte_trans(&nrf24l->nss,
                                            CMD_R_REGISTER + REG_RF_SETUP);

    rf_setup &= ~(0b11 << 1);
    rf_setup |= power;
    
    spi_begin_trans(&nrf24l->nss);
    spi_txrx_byte(CMD_W_REGISTER + REG_RF_SETUP);
    spi_txrx_byte(power);
    spi_end_trans(&nrf24l->nss);
}

void nrf24l_set_tx_address(nrf24l_t *nrf24l, uint32_t addr)
{
    spi_begin_trans(&nrf24l->nss);
    spi_txrx_byte(CMD_W_REGISTER + REG_TX_ADDR);
    spi_tx_buf((char *) &addr, sizeof(addr));
    spi_end_trans(&nrf24l->nss);
}

void nrf24l_set_rx_address(nrf24l_t *nrf24l, uint32_t addr)
{
    spi_begin_trans(&nrf24l->nss);
    spi_txrx_byte(CMD_W_REGISTER + REG_RX_ADDR_P0);
    spi_tx_buf((char *) &addr, sizeof(addr));
    spi_end_trans(&nrf24l->nss);
}

static void nrf24l_irq_handler(unsigned int irqn, void *client_data)
{
    // TBD
}
