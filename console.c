
#include "console.h"
#include "utils.h"

#include <system_stm32f0xx.h>
#include <stm32f0xx_ll_gpio.h>
#include <string.h>
#include <stdbool.h>

/*----------------------------------------------------------------------
 * Private declarations
 *----------------------------------------------------------------------*/

#define BAUD_RATE		115200
#define DMA_IRQ_PRIORITY	3

// must be a power of 2
#define CONSOLE_BUF_SIZE	16
#define CONSOLE_BUF_MASK	(CONSOLE_BUF_SIZE - 1)

struct console_st {
    // ring buffer of pending data
    char	buffer[CONSOLE_BUF_SIZE];
    uint16_t	head;	// index of first byte pending to be written
    uint16_t	tail;	// index of first free byte in buffer
    			// tail is always the same or ahead of head
    			// buffer is empty if head == tail
    			// full if next(tail) == head
    uint16_t	dma_pending_size;	// number of bytes being DMA-ed from buffer
};

typedef volatile struct console_st	console_t;

console_t console;

INLINE uint16_t console_get_space();
static void console_check_dma();

/*----------------------------------------------------------------------
 * Function definitions
 *----------------------------------------------------------------------*/

void console_init()
{
    memset((char *) &console, 0, sizeof(console));
    
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

    // enable USART1 with transmit mode
    USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;

    // wait until transmission of idle frame has completed and clear TC flag
    while (!(USART1->ISR & USART_ISR_TC));
    USART1->ICR |= USART_ICR_TCCF;

    // enable DMA for USART transmission
    USART1->CR3 |= USART_CR3_DMAT;
}

// return 1 if successful, 0 if there was no buffer space available
int console_write(const char *buf, int size)
{
    int rv = 0;
    crit_state_t crit_state;
    enter_crit_rec(&crit_state);
      
    // check if we have enough buffer space
    if (console_get_space() >= size) {
	// we may need to split the copy if adding to the end of the buffer
	uint16_t n = min(CONSOLE_BUF_SIZE - console.tail, size);
	memcpy((char *) console.buffer + console.tail, buf, n);
	size -= n;
	if (size > 0) {
	    // copy the rest to the front of the buffer
	    buf += n;
	    n = size;
	    console.tail = 0;
	    memcpy((char *) console.buffer, buf, n);
	}
	console.tail += n;
	console.tail &= CONSOLE_BUF_MASK;

	console_check_dma();
	rv = 1;
    }
    
    exit_crit_rec(&crit_state);

    return rv;
}

INLINE uint16_t console_get_space()
{
    crit_state_t crit_state;
    enter_crit_rec(&crit_state);
    
    uint16_t space = (console.head - console.tail - 1) & CONSOLE_BUF_MASK;

    exit_crit_rec(&crit_state);
    return space;
}

static void console_check_dma()
{
    if (console.head == console.tail || console.dma_pending_size > 0)
	return;
    
    console.dma_pending_size = console.head < console.tail
	? console.tail - console.head : CONSOLE_BUF_SIZE - console.head;

    DMA1_Channel4->CMAR = (uint32_t) (console.buffer + console.head);
    DMA1_Channel4->CNDTR = console.dma_pending_size;
    DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void DMA1_Channel4_5_IRQHandler()
{
    crit_state_t crit_state;
    enter_crit_rec(&crit_state);

    if (DMA1->ISR & DMA_ISR_TCIF4) {
	// DMA transaction has completed
	DMA1->IFCR = DMA_IFCR_CTCIF4;
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;

	console.head += console.dma_pending_size;
	console.head &= CONSOLE_BUF_MASK;
	console.dma_pending_size = 0;
	
	// new data may have arrived to be sent
	console_check_dma();
    }

    exit_crit_rec(&crit_state);
}
