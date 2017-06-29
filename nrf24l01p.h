#pragma once

#include "spi.h"
#include "exti.h"

struct nrf24l_st {
    // user to set fields before init
    GPIO_TypeDef	*irq_port;
    GPIO_TypeDef	*ce_port;
    spi_nss_t		nss;
    unsigned char	irq_pin;
    unsigned char	ce_pin;
    
    exti_irq_handler_t	irq_handler;	// receives notifications on packet arrival
};

typedef struct nrf24l_st nrf24l_t;

// RF power level, 0x00 lowest, 0x03 highest
enum nrf24l_power_st {
    NRF24L_RF_PWR_0 = 0x00,
    NRF24L_RF_PWR_1 = 0x01,
    NRF24L_RF_PWR_2 = 0x02,
    NRF24L_RF_PWR_3 = 0x03
};

typedef enum nrf24l_power_st nrf24l_power_t;

/*----------------------------------------------------------------------
 * Public function declarations
 *----------------------------------------------------------------------*/

extern void nrf24l_init(nrf24l_t *nrf24l);

// channel must be between 0 and 125 inclusive
extern void nrf24l_set_rf_ch(nrf24l_t *nrf24l, unsigned int channel);

// channel must be between 0 and 125 inclusive
extern void nrf24l_set_rf_pwr(nrf24l_t *nrf24l, nrf24l_power_t power);

// we use 4 byte addresses
extern void nrf24l_set_tx_address(nrf24l_t *nrf24l, uint32_t addr);
extern void nrf24l_set_rx_address(nrf24l_t *nrf24l, uint32_t addr);

