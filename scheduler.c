
#include "scheduler.h"

#include <system_stm32f0xx.h>
#include <string.h>
#include <assert.h>

/*----------------------------------------------------------------------
 * Scheduler implementation
 *----------------------------------------------------------------------*/

scheduler_t scheduler;

static inline void sched_timer_update();

static inline void sched_scheduled_list_add(task_t *task);
static inline void sched_scheduled_list_remove(task_t *task);
static inline task_t *sched_scheduled_list_pop();

static inline void sched_cancelled_list_add(task_t *task);
static inline void sched_cancelled_list_remove(task_t *task);
static inline task_t *sched_cancelled_list_pop();

static inline void _sched_list_remove(task_t * volatile*head, task_t *task);
static inline task_t *_sched_list_pop(task_t * volatile*head);

static inline uint32_t sched_time_add(uint32_t t1, uint32_t t2);
static inline int sched_counter_lt(uint16_t t1, uint16_t t2);
static inline int sched_time_lte(uint32_t t1, uint32_t t2);
static inline int sched_task_due_soon(task_t *task, uint32_t now);

void sched_init()
{
    memset((char *) &scheduler, 0, sizeof(scheduler));

    // clear pending PendSV exception and set up priority
    SCB->ICSR &= ~SCB_ICSR_PENDSVSET_Msk;
    NVIC_SetPriority(PendSV_IRQn, 3);
    // NVIC_EnableIRQ(PendSV_IRQn);
    
    // initialise timer peripheral
    TIM14->PSC = SystemCoreClock / 1000000 - 1;	 // 1MHz counter clock, 1 microsec period
    TIM14->ARR = TIMER_RELOAD;	                 // ~65.5ms counter overflow
    TIM14->CNT = 0;
    TIM14->CCR1 = 0;		// no specific compare event required initially

    // channel is configured as output
    TIM14->CCMR1 &= ~TIM_CCMR1_CC1S;

    // enable compare and update interrupts
    TIM14->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE;
    NVIC_EnableIRQ(TIM14_IRQn);

    // enable counter
    TIM14->CR1 |= TIM_CR1_CEN;
}

void sched_task_init(task_t *task, TaskFn task_fn, void *client_data)
{
    memset((char *) task, 0, sizeof(*task));

    task->task_fn = task_fn;
    task->client_data = client_data;
    task->state = TASK_INITIALISED;
}

void _sched_task_schedule(task_t *task)
{
    assert(task->state != TASK_UNDEFINED);

    int update = 0;
    if (task->state == TASK_SCHEDULED) {
	// task is waiting in scheduled queue, remove and re-schedule
	int update = scheduler.scheduled_head == task;
	sched_scheduled_list_remove(task);
    }
    
    if (task->state == TASK_CANCELLED) {
	// task is waiting in cancelled queue, remove and re-schedule
	sched_cancelled_list_remove(task);
    }

    sched_scheduled_list_add(task);

    if (scheduler.scheduled_head == task) {
	// the earliest task has changed, check if we need to start
	// an execution or need to update the compare register
	if (sched_task_due_soon(task, _sched_now())) {
	    // task is ready to run immediately, no need to change compare registers
	    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
	} else {
	    update = 1;
	}
    }
    if (update) {
	// first task has changed, we may need to update compare register
	sched_timer_update();
    }
}

void _sched_task_cancel(task_t *task)
{
    if (task->state != TASK_SCHEDULED)
	return;

    int update = scheduler.scheduled_head == task;
    sched_scheduled_list_remove(task);
    sched_cancelled_list_add(task);
    if (update) {
	// this was the first task, we may need to update compare register
	sched_timer_update();
    }
}

/**
 * Update compare register as per the earliest deadline, if any.
 *
 * Interrupts must be disabled when calling this function.
 */
static inline void sched_timer_update()
{
    uint32_t next_cmp = 0;
    if (scheduler.scheduled_head) {
	next_cmp = scheduler.scheduled_head->deadline - scheduler.timer_offset;
	if (next_cmp > TIMER_RELOAD) {
	    next_cmp = 0;
	}
    }
    // we assume that we have enough time to set up the next counter event 
    TIM14->CCR1 = (uint16_t) next_cmp;
    TIM14->SR = ~TIM_SR_CC1IF;
}

static inline void sched_scheduled_list_add(task_t *task)
{
    assert(task->state == TASK_INITIALISED);

    task_t * volatile*prev = &scheduler.scheduled_head;
    while (*prev && sched_time_lte((*prev)->deadline, task->deadline)) {
	prev = &(*prev)->next;
    }
    // task needs to be inserted before *prev
    task->next = *prev;
    *prev = task;

    task->state = TASK_SCHEDULED;
}

static inline void sched_scheduled_list_remove(task_t *task)
{
    _sched_list_remove(&scheduler.scheduled_head, task);
}

static inline task_t *sched_scheduled_list_pop()
{
    return _sched_list_pop(&scheduler.scheduled_head);
}

static inline void sched_cancelled_list_add(task_t *task)
{
    assert(task->state == TASK_INITIALISED);

    // no need to keep cancelled tasks in any particular order
    task->next = scheduler.cancelled_head;
    scheduler.cancelled_head = task;
    task->state = TASK_CANCELLED;
}

static inline void sched_cancelled_list_remove(task_t *task)
{
    _sched_list_remove(&scheduler.cancelled_head, task);
}

static inline task_t *sched_cancelled_list_pop()
{
    return _sched_list_pop(&scheduler.cancelled_head);
}

static inline void _sched_list_remove(task_t * volatile*head, task_t *task)
{
    task_t * volatile*prev = head;
    while (*prev && *prev != task) {
	prev = &(*prev)->next;
    }
    if (!*prev) {
	// not found
	return;
    }
    *prev = task->next;
    task->next = NULL;
    task->state = TASK_INITIALISED;
}

static inline task_t *_sched_list_pop(task_t * volatile*head)
{
    task_t *task = *head;
    if (!task) {
	return NULL;
    }

    *head = task->next;
    task->next = NULL;
    task->state = TASK_INITIALISED;

    return task;
}

static inline uint32_t sched_time_add(uint32_t t1, uint32_t t2)
{
    // it is ok to overflow
    return t1 + t2;
}

static inline int sched_counter_lt(uint16_t t1, uint16_t t2)
{
    // note that t1 and t2 wraps around after 2^16 - 1
    return t1 != t2 && (uint16_t) (t2 - t1) <= 1u << 15;
}

static inline int sched_time_lte(uint32_t t1, uint32_t t2)
{
    // note that t1 and t2 wraps around after 2^32 - 1
    return t2 - t1 <= 1u << 31;
}

static inline int sched_task_due_soon(task_t *task, uint32_t now)
{
    return sched_time_lte(task->deadline, sched_time_add(now, MAX_SPIN_DELAY));
}

void TIM14_IRQHandler()
{
    // note that the cost of this function can be around 30 microsecs
    __disable_irq();

    if (TIM14->SR & TIM_SR_UIF) {
	// clear update events, note that there may have been a compare event
	// as well just before the overflow
	TIM14->SR = ~TIM_SR_UIF;
	scheduler.timer_offset += TIMER_RELOAD + 1;

	GPIOA->ODR |= GPIO_ODR_2;
	GPIOA->ODR &= ~GPIO_ODR_2;
    }
    
    if (TIM14->SR & TIM_SR_CC1IF) {
	// clear counter compare event, we will set a new compare value later
	TIM14->CCR1 = 0;
	TIM14->SR = ~TIM_SR_CC1IF;
    
	if (scheduler.scheduled_head
	    && sched_task_due_soon(scheduler.scheduled_head, _sched_now())) {
	    // a task is ready to run immediately, no need to set up compare register
	    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
	} else {
	    sched_timer_update();
	}
    }

    __enable_irq();
}

/**
 * Execute all pending timer tasks.
 */
void PendSV_Handler()
{
    __disable_irq();

    // execute expired tasks
    uint32_t now = _sched_now();

    while (scheduler.cancelled_head
	   || (scheduler.scheduled_head
	       && sched_task_due_soon(scheduler.scheduled_head, now))) {
	state_t prev_state;
	
	// call all cancelled tasks first
	while (scheduler.cancelled_head) {
	    prev_state = scheduler.cancelled_head->state;
	    task_t *current = sched_cancelled_list_pop();
	
	    __enable_irq();
	    (*current->task_fn)(current, prev_state, 0);
	    __disable_irq();
	}

	// call expired tasks
	prev_state = scheduler.scheduled_head->state;
	task_t *current = sched_scheduled_list_pop();
	uint32_t expiry = current->deadline;
	if (current->interval) {
	    current->deadline += current->interval;
	    // if we are behind schedule we may need to skip a few iterations
	    now = _sched_now();
	    if (current->deadline != now && sched_time_lte(current->deadline, now)) {
		current->deadline += ((now - current->deadline - 1) / current->interval + 1)
		    * current->interval;
	    }
	    sched_scheduled_list_add(current);
	}

	__enable_irq();
	(*current->task_fn)(current, prev_state, expiry);
	__disable_irq();

	now = _sched_now();
    }

    // one of the re-schedules above may have triggered this IRQ again,
    // but we have processed all imminent tasks so no need
    SCB->ICSR &= ~SCB_ICSR_PENDSVSET_Msk;
    sched_timer_update();

    __enable_irq();
}
