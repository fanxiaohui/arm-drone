
#include "console.h"

#include <system_stm32f0xx.h>
#include <stm32f0xx_ll_gpio.h>

void console_init()
{
    gpio_set_af_mode(GPIOA, 9, LL_GPIO_AF_1);

    // reset USART
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

    // set baud rate 115200, oversampling by 16: fclk/baud rate, round it
    USART1->CR1 &= ~USART_CR1_OVER8;
    USART1->BRR = (2 * SystemCoreClock + 115200) / (2 * 115200);

    // 9 data bits, 1 stop bit
    USART1->CR1 |= USART_CR1_M;
    USART1->CR2 &= ~USART_CR2_STOP_Msk;

    // no parity
    USART1->CR1 &= ~USART_CR1_PCE;
    
    // no flow control
    USART1->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);

    // enable USART1 transmit only
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_UE;
}

int console_write(const char *buf, int size)
{
    const char *ebuf;
    for (ebuf = buf + size; buf != ebuf; ++buf) {
	while (!(USART1->ISR & USART_ISR_TXE));
	USART1->TDR = *buf;
    }
}
