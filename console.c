
#include "console.h"

#include <system_stm32f0xx.h>
#include <stm32f0xx_ll_gpio.h>
#include <string.h>

#define BAUD_RATE		115200
#define DMA_IRQ_PRIORITY	3

static char buffer[128];

void console_init()
{
    gpio_set_af_mode(GPIOA, 9, LL_GPIO_AF_1);

    // remap USART1 TX DMA channel to channel 4, it is a lower priority channel
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;

    // enable DMA interrupts with appropriate priority
    NVIC_ClearPendingIRQ(DMA1_Channel4_5_IRQn);
    NVIC_SetPriority(DMA1_Channel4_5_IRQn, DMA_IRQ_PRIORITY);
    NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
    
    // set up DMA channel 4, memory-to-peripheral copy for USART, 8 bit transfer size,
    // low priority, memory increment mode, transfer-copy interrupt enabled
    DMA1_Channel4->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
    DMA1_Channel4->CPAR = (uint32_t) &USART1->TDR;
    DMA1_Channel4->CMAR = (uint32_t) buffer;
    DMA1_Channel4->CNDTR = 0;

    // reset USART
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

    // set baud rate, oversampling by 16: fclk/baud rate, round it
    USART1->CR1 &= ~USART_CR1_OVER8;
    USART1->BRR = (2 * SystemCoreClock + BAUD_RATE) / (2 * BAUD_RATE);

    // 8 data bits, 1 stop bit
    USART1->CR1 &= ~USART_CR1_M;
    USART1->CR2 &= ~USART_CR2_STOP_Msk;

    // no parity
    USART1->CR1 &= ~USART_CR1_PCE;
    
    // no flow control
    USART1->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);

    // enable USART1
    USART1->CR1 |= USART_CR1_UE;

    // enable transmit mode
    USART1->CR1 |= USART_CR1_TE;

    // wait until transmission of idle frame has completed and clear flag
    while (!(USART1->ISR & USART_ISR_TC));
    USART1->ICR |= USART_ICR_TCCF;

    // enable DMA for USART transmission
    USART1->CR3 |= USART_CR3_DMAT;
}

int console_write(const char *buf, int size)
{
    /*
    const char *ebuf;
    for (ebuf = buf + size; buf != ebuf; ++buf) {
	while (!(USART1->ISR & USART_ISR_TXE));
	USART1->TDR = *buf;
    }
    */
    memcpy(buffer, buf, size);
    DMA1_Channel4->CNDTR = size;
    DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void DMA1_Channel4_5_IRQHandler()
{
    if (DMA1->ISR & DMA_ISR_TCIF4) {
	DMA1->IFCR = DMA_IFCR_CTCIF4;

	DMA1_Channel4->CCR &= ~DMA_CCR_EN;
	// TBD
    }
}
