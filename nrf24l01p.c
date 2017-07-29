
#include "nrf24l01p.h"
#include "exti.h"

#include <stm32f0xx_ll_gpio.h>
#include <stdio.h>
#include <assert.h>

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
#define REG_CONFIG_CRCO		(1 << 2)
#define REG_CONFIG_EN_CRC	(1 << 3)
#define REG_CONFIG_MASK_MAX_RT	(1 << 4)
#define REG_CONFIG_MASK_TX_DS	(1 << 5)
#define REG_CONFIG_MASK_RX_DR	(1 << 6)

// status register bits
#define REG_STATUS_TX_FULL	(1 << 0)
#define REG_STATUS_RX_P_NO	(0b111 << 1)
#define REG_STATUS_MAX_RT	(1 << 4)
#define REG_STATUS_TX_DS	(1 << 5)
#define REG_STATUS_RX_DR	(1 << 6)

// interrupt handler for packet receipt notifications
static void nrf24l_init(nrf24l_t *nrf24l);
static void nrf24l_irq_handler_ptx(unsigned int irqn, void *client_data);
static void nrf24l_irq_handler_prx(unsigned int irqn, void *client_data);
INLINE unsigned int nrf24l_status(nrf24l_t *nrf24l);
INLINE unsigned int nrf24l_reg_write(nrf24l_t *nrf24l, int reg, uint32_t data, int size);
INLINE unsigned int nrf24l_reg_read(nrf24l_t *nrf24l, int reg, void *data, int size);

void nrf24l_init_ptx(nrf24l_t *nrf24l)
{
    assert(nrf24l->payload_size <= 32);
    
    exti_irq_handler_init(&nrf24l->irq_handler, &nrf24l_irq_handler_ptx, nrf24l);
    exti_register(&nrf24l->irq_handler, nrf24l->irq_pin);

    nrf24l_init(nrf24l);

    // configure transmit mode, power up radio, enable 2 byte CRC
    nrf24l_reg_write(nrf24l, REG_CONFIG, REG_CONFIG_CRCO | REG_CONFIG_EN_CRC
                     | REG_CONFIG_PWR_UP, 1);
}

void nrf24l_init_prx(nrf24l_t *nrf24l)
{
    exti_irq_handler_init(&nrf24l->irq_handler, &nrf24l_irq_handler_prx, nrf24l);
    exti_register(&nrf24l->irq_handler, nrf24l->irq_pin);

    nrf24l_init(nrf24l);

    // configure receive mode, power up radio, enable 2 byte CRC
    nrf24l_reg_write(nrf24l, REG_CONFIG, REG_CONFIG_PRIM_RX | REG_CONFIG_CRCO
                     | REG_CONFIG_EN_CRC | REG_CONFIG_PWR_UP, 1);
}

void nrf24l_init(nrf24l_t *nrf24l)
{
    gpio_set_mode(nrf24l->irq_port, nrf24l->irq_pin, LL_GPIO_MODE_INPUT);
    gpio_set_mode(nrf24l->ce_port, nrf24l->ce_pin, LL_GPIO_MODE_OUTPUT);
    
    spi_nss_init(&nrf24l->nss);

    // enable interrupt line on IRQ pin, we assume that no other port will have
    // an input pin with the same pin number.
    // raise interrupt only on falling edge
    EXTI->RTSR &= ~(1 << nrf24l->irq_pin);
    EXTI->FTSR |= 1 << nrf24l->irq_pin;
    EXTI->IMR |= 1 << nrf24l->irq_pin;

    // set up 4 byte addresses
    nrf24l_reg_write(nrf24l, REG_SETUP_AW, 0b10, 1);
}

void nrf24l_set_rf_ch(nrf24l_t *nrf24l, unsigned int channel)
{
    nrf24l_reg_write(nrf24l, REG_RF_CH, channel, 1);
}

void nrf24l_set_rf_pwr(nrf24l_t *nrf24l, nrf24l_power_t power)
{
    unsigned char rf_setup;
    nrf24l_reg_read(nrf24l, REG_RF_SETUP, &rf_setup, 1);

    rf_setup &= ~(0b11 << 1);
    rf_setup |= power;
    
    nrf24l_reg_write(nrf24l, REG_RF_SETUP, rf_setup, 1);
}

void nrf24l_set_tx_address(nrf24l_t *nrf24l, uint32_t addr)
{
    nrf24l_reg_write(nrf24l, REG_TX_ADDR, addr, sizeof(addr));
}

void nrf24l_set_rx_address(nrf24l_t *nrf24l, uint32_t addr)
{
    nrf24l_reg_write(nrf24l, REG_RX_ADDR_P0, addr, sizeof(addr));
}

uint32_t nrf24l_get_tx_address(nrf24l_t *nrf24l)
{
    uint32_t addr;
    nrf24l_reg_read(nrf24l, REG_TX_ADDR, &addr, sizeof(addr));
    return addr;
}

uint32_t nrf24l_get_rx_address(nrf24l_t *nrf24l)
{
    uint32_t addr;
    nrf24l_reg_read(nrf24l, REG_RX_ADDR_P0, &addr, sizeof(addr));
    return addr;
}

void nrf24l_send(nrf24l_t *nrf24l, void *payload)
{
    spi_begin_trans(&nrf24l->nss);
    spi_txrx_byte(CMD_W_TX_PAYLOAD);
    spi_tx_buf(payload, nrf24l->payload_size);
    spi_end_trans(&nrf24l->nss);
}

static void nrf24l_irq_handler_ptx(unsigned int irqn, void *client_data)
{
    nrf24l_t *nrf24l = client_data;
    
    unsigned int status = nrf24l_status(nrf24l);
    unsigned int pstatus = status;
    if (status & REG_STATUS_MAX_RT) {
        // maximum number of retransmission occurred, log error
        printf("max retransmissions reached\n");

        // flush TX queue to prevent resend of undelivered packet
        spi_txrx_byte_trans(&nrf24l->nss, CMD_FLUSH_TX);
        
        status &= ~REG_STATUS_MAX_RT;
    }
    if (status & REG_STATUS_TX_DS) {
        printf("data sent and acked\n");

        status &= ~REG_STATUS_TX_DS;
    }
    if (status != pstatus) {
        nrf24l_reg_write(nrf24l, REG_STATUS, status, 1);
    }
}

static void nrf24l_irq_handler_prx(unsigned int irqn, void *client_data)
{
    nrf24l_t *nrf24l = client_data;
    
    unsigned int status = nrf24l_status(nrf24l);
    unsigned int pstatus = status;

    // TBD
    
    if (status != pstatus) {
        nrf24l_reg_write(nrf24l, REG_STATUS, status, 1);
    }
}

INLINE unsigned int nrf24l_status(nrf24l_t *nrf24l)
{
    return spi_txrx_byte_trans(&nrf24l->nss, CMD_NOP);
}

INLINE unsigned int nrf24l_reg_write(nrf24l_t *nrf24l, int reg, uint32_t data, int size)
{
    spi_begin_trans(&nrf24l->nss);
    unsigned int status = spi_txrx_byte(CMD_W_REGISTER + reg);
    spi_tx_buf((char *) &data, size);
    spi_end_trans(&nrf24l->nss);

    return status;
}

INLINE unsigned int nrf24l_reg_read(nrf24l_t *nrf24l, int reg, void *data, int size)
{
    spi_begin_trans(&nrf24l->nss);
    unsigned int status = spi_txrx_byte(CMD_R_REGISTER + reg);
    spi_rx_buf(data, size);
    spi_end_trans(&nrf24l->nss);

    return status;
}
