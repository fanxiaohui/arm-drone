
#include "scheduler.h"

#include <string.h>
#include <assert.h>

/*----------------------------------------------------------------------
 * Scheduler implementation
 *----------------------------------------------------------------------*/

scheduler_t scheduler;

// timer ticks for which we don't schedule a timer, but execute immediately
#define MIN_TIMER_DELAY	20

INLINE void sched_timer_update();

INLINE void sched_timer_list_add(tim_task_t *task);
INLINE void sched_timer_list_remove(tim_task_t *task);
INLINE tim_task_t *sched_timer_list_pop();

INLINE void sched_cancelled_list_add(tim_task_t *task);
INLINE void sched_cancelled_list_remove(tim_task_t *task);
INLINE tim_task_t *sched_cancelled_list_pop();

INLINE void _sched_list_remove(tim_task_t * volatile*head, tim_task_t *task);
INLINE tim_task_t *_sched_list_pop(tim_task_t * volatile*head);

INLINE void sched_task_list_add(task_t *task);
INLINE task_t *sched_task_list_pop();

INLINE uint32_t sched_time_add(uint32_t t1, uint32_t t2);
INLINE int sched_timer_due_soon(tim_task_t *task, uint32_t now);

void sched_init()
{
    memset((char *) &scheduler, 0, sizeof(scheduler));

    // clear pending PendSV exception and set up priority
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;
    NVIC_SetPriority(PendSV_IRQn, SCHED_TASK_PRIORITY);
    
    // initialise timer peripheral
    TIM14->PSC = SystemCoreClock / 1000000 - 1;	 // 1MHz counter clock, 1 microsec period
    TIM14->ARR = TIMER_RELOAD;	                 // ~65.5ms counter overflow
    TIM14->CNT = 0;
    TIM14->CCR1 = 0;		// no specific compare event required initially

    // channel is configured as output
    TIM14->CCMR1 &= ~TIM_CCMR1_CC1S;

    // enable compare and update interrupts
    TIM14->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE;
    NVIC_SetPriority(TIM14_IRQn, SCHED_TIMER_IRQ_PRIORITY);
    NVIC_EnableIRQ(TIM14_IRQn);

    // enable counter
    TIM14->CR1 |= TIM_CR1_CEN;
}

void sched_timer_init(tim_task_t *task, TimerFn tim_task_fn, void *client_data)
{
    memset((char *) task, 0, sizeof(*task));

    task->tim_task_fn = tim_task_fn;
    task->client_data = client_data;
    task->state = TASK_INITIALISED;
}

void _sched_timer_schedule(tim_task_t *task)
{
    assert(task->state != TASK_UNDEFINED);

    int update = 0;
    if (task->state == TASK_SCHEDULED) {
	// task is waiting in scheduled queue, remove and re-schedule
	update = scheduler.timer_head == task;
	sched_timer_list_remove(task);
    }
    
    if (task->state == TASK_CANCELLED) {
	// task is waiting in cancelled queue, remove and re-schedule
	sched_cancelled_list_remove(task);
    }

    sched_timer_list_add(task);

    if (scheduler.timer_head == task) {
	// the earliest task has changed, check if we need to start
	// an execution or need to update the compare register
	if (sched_timer_due_soon(task, _sched_now())) {
	    // task is ready to run immediately, no need to change compare registers
	    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
	} else {
	    update = 1;
	}
    }
    if (update) {
	// first task has changed, we may need to update compare register
	sched_timer_update();
    }
}

void _sched_timer_cancel(tim_task_t *task)
{
    if (task->state != TASK_SCHEDULED)
	return;

    int update = scheduler.timer_head == task;
    sched_timer_list_remove(task);
    sched_cancelled_list_add(task);
    if (update) {
	// this was the first task, we may need to update compare register
	sched_timer_update();
    }
}

void sched_task_init(task_t *task, TaskFn task_fn, void *client_data)
{
    memset((char *) task, 0, sizeof(*task));

    task->task_fn = task_fn;
    task->client_data = client_data;
    task->state = TASK_INITIALISED;
}

void _sched_task_pending(task_t *task)
{
    assert(task->state != TASK_UNDEFINED);

    if (task->state == TASK_SCHEDULED)
	return;

    sched_task_list_add(task);
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

/**
 * Update compare register as per the earliest deadline, if any.
 *
 * Interrupts must be disabled when calling this function.
 */
INLINE void sched_timer_update()
{
    uint32_t next_cmp = 0;
    if (scheduler.timer_head) {
	next_cmp = scheduler.timer_head->deadline - scheduler.timer_offset;
	if (next_cmp > TIMER_RELOAD) {
	    next_cmp = 0;
	}
    }
    // we assume that we have enough time to set up the next counter event 
    TIM14->CCR1 = (uint16_t) next_cmp;
    TIM14->SR = ~TIM_SR_CC1IF;
}

INLINE void sched_timer_list_add(tim_task_t *task)
{
    assert(task->state == TASK_INITIALISED);

    tim_task_t * volatile*prev = &scheduler.timer_head;
    while (*prev && sched_time_lte((*prev)->deadline, task->deadline)) {
	prev = &(*prev)->next;
    }
    // task needs to be inserted before *prev
    task->next = *prev;
    *prev = task;

    task->state = TASK_SCHEDULED;
}

INLINE void sched_timer_list_remove(tim_task_t *task)
{
    _sched_list_remove(&scheduler.timer_head, task);
}

INLINE tim_task_t *sched_timer_list_pop()
{
    return _sched_list_pop(&scheduler.timer_head);
}

INLINE void sched_cancelled_list_add(tim_task_t *task)
{
    assert(task->state == TASK_INITIALISED);

    // no need to keep cancelled tasks in any particular order
    task->next = scheduler.cancelled_head;
    scheduler.cancelled_head = task;
    task->state = TASK_CANCELLED;
}

INLINE void sched_cancelled_list_remove(tim_task_t *task)
{
    _sched_list_remove(&scheduler.cancelled_head, task);
}

INLINE tim_task_t *sched_cancelled_list_pop()
{
    return _sched_list_pop(&scheduler.cancelled_head);
}

INLINE void _sched_list_remove(tim_task_t * volatile*head, tim_task_t *task)
{
    tim_task_t * volatile*prev = head;
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

INLINE tim_task_t *_sched_list_pop(tim_task_t * volatile*head)
{
    tim_task_t *task = *head;
    if (!task) {
	return NULL;
    }

    *head = task->next;
    task->next = NULL;
    task->state = TASK_INITIALISED;

    return task;
}

INLINE void sched_task_list_add(task_t *task)
{
    // add the task to the end of the circular task list
    task_t *tail = scheduler.task_tail;
    if (!tail) {
	// task will be the only item in the list, link it to itself
	task->next = task;
    } else {
	task->next = tail->next;
	tail->next = task;
    }
    scheduler.task_tail = task;
    task->state = TASK_SCHEDULED;
}

INLINE task_t *sched_task_list_pop()
{
    // remove the front of the circular task list, which is task_tail->next
    task_t *task = scheduler.task_tail;
    if (!task) {
	return NULL;
    }
    if (task == task->next) {
	// there was only one task in the list
	scheduler.task_tail = NULL;
    } else {
	// remove the next task after the tail
	task = task->next;
	scheduler.task_tail->next = task->next;
    }
    task->next = NULL;
    task->state = TASK_INITIALISED;

    return task;
}

INLINE uint32_t sched_time_add(uint32_t t1, uint32_t t2)
{
    // it is ok to overflow
    return t1 + t2;
}

INLINE int sched_timer_due_soon(tim_task_t *task, uint32_t now)
{
    return sched_time_lte(task->deadline, sched_time_add(now, MIN_TIMER_DELAY));
}

void TIM14_IRQHandler()
{
    // note that the cost of this function can be around 30 microsecs
    enter_crit();
    
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
    
	if (scheduler.timer_head
	    && sched_timer_due_soon(scheduler.timer_head, _sched_now())) {
	    // a task is ready to run immediately, no need to set up compare register
	    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
	} else {
	    sched_timer_update();
	}
    }
    exit_crit();
}

/**
 * Execute all pending timer tasks.
 */
void PendSV_Handler()
{
    enter_crit();

    // execute expired tasks
    uint32_t now = _sched_now();
    while (scheduler.task_tail
	   || scheduler.cancelled_head
	   || (scheduler.timer_head
	       && sched_timer_due_soon(scheduler.timer_head, now))) {

	// call all immediate tasks
	while (scheduler.task_tail) {
	    task_t *current = sched_task_list_pop();
	    exit_crit();
	    (*current->task_fn)(current);
	    enter_crit();
	}

	// call all cancelled tasks
	state_t prev_state;
	while (scheduler.cancelled_head) {
	    prev_state = scheduler.cancelled_head->state;
	    tim_task_t *current = sched_cancelled_list_pop();
	
	    exit_crit();
	    (*current->tim_task_fn)(current, prev_state, 0);
	    enter_crit();
	}

	// call one expired task, if any, it may cancel/schedule other tasks
	if (scheduler.timer_head
	    && sched_timer_due_soon(scheduler.timer_head, now)) {
	    prev_state = scheduler.timer_head->state;
	    tim_task_t *current = sched_timer_list_pop();

	    uint32_t expiry = current->deadline;
	    if (current->interval) {
		current->deadline += current->interval;
		// if we are behind schedule we may need to skip a few iterations
		if (current->deadline != now && sched_time_lte(current->deadline, now)) {
		    current->deadline += ((now - current->deadline - 1) / current->interval + 1)
			* current->interval;
		}
		sched_timer_list_add(current);
	    }

	    exit_crit();
	    (*current->tim_task_fn)(current, prev_state, expiry);
	    enter_crit();
	}
	now = _sched_now();
    }

    // one of the re-schedules above may have triggered this IRQ again,
    // but we have processed all imminent tasks so no need
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;
    sched_timer_update();

    exit_crit();
}
