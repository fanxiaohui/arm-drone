#pragma once

#include <stm32f0xx.h>

#include "utils.h"

#define SCHED_TASK_PRIORITY		3
#define SCHED_TIMER_IRQ_PRIORITY	0

// 16-bit timer counter, count up to maximum value
#define TIMER_RELOAD	((uint16_t) ~0)

/*----------------------------------------------------------------------
 * Scheduler public interface
 *----------------------------------------------------------------------*/

struct tim_task_st;
typedef volatile struct tim_task_st tim_task_t;

struct task_st;
typedef volatile struct task_st task_t;

typedef enum {
    TASK_UNDEFINED = 0,
    TASK_INITIALISED,		// task/timer is initialised
    TASK_SCHEDULED,		// task/timer is in the scheduled queue
    TASK_CANCELLED		// timer is in the cancelled queue
} state_t;

typedef void (*TimerFn)(tim_task_t *task, state_t prev_state, uint32_t expiry_time);

struct tim_task_st {
    state_t	state;

    tim_task_t	*next;		// linked list of tasks
    uint32_t	deadline;	// absolute deadline of next execution in timer
    				// clock ticks, updated after each execution
    uint32_t	interval;	// interval in timer clock ticks for repetitive tasks,
                                // or 0 for one-off tasks
    TimerFn	tim_task_fn;
    void	*client_data;
};

typedef void (*TaskFn)(task_t *task);

struct task_st {
    state_t	state;		// TASK_CANCELLED is not a valid state

    task_t	*next;		// linked list of tasks
    TaskFn	task_fn;
    void	*client_data;
};

struct scheduler_st {
    tim_task_t	*timer_head;	 // sorted linked list of SCHEDULED timers, or NULL
    tim_task_t	*cancelled_head; // non-sorted linked list of CANCELLED timers, or NULL
    task_t	*task_tail;	 // FIFO circular list of SCHEDULED tasks, or NULL
    
    uint32_t	timer_offset;	 // current time at most recent TIMx->CNT overflow
};

typedef volatile struct scheduler_st scheduler_t;

extern scheduler_t scheduler;

extern void sched_init();
extern void sched_timer_init(tim_task_t *task, TimerFn tim_task_fn, void *client_data);
INLINE void sched_timer_schedule(tim_task_t *task, uint32_t deadline, uint32_t interval);
INLINE void sched_timer_cancel(tim_task_t *task);
extern void _sched_timer_schedule(tim_task_t *task);
extern void _sched_timer_cancel(tim_task_t *task);
extern void sched_task_init(task_t *task, TaskFn task_fn, void *client_data);
INLINE void sched_task_pending(task_t *task);
extern void _sched_task_pending(task_t *task);

INLINE uint32_t sched_now();
INLINE uint32_t _sched_now();
INLINE int sched_time_lte(uint32_t t1, uint32_t t2);

/*----------------------------------------------------------------------
 * Scheduler inline implementation
 *----------------------------------------------------------------------*/

INLINE void sched_timer_schedule(tim_task_t *task, uint32_t deadline,
				       uint32_t interval)
{
    crit_state_t crit_state;
    enter_crit_rec(&crit_state);

    task->deadline = deadline;
    task->interval = interval;
    _sched_timer_schedule(task);

    exit_crit_rec(&crit_state);
}

INLINE void sched_timer_cancel(tim_task_t *task)
{
    crit_state_t crit_state;
    enter_crit_rec(&crit_state);

    _sched_timer_cancel(task);

    exit_crit_rec(&crit_state);
}

INLINE void sched_task_pending(task_t *task)
{
    crit_state_t crit_state;
    enter_crit_rec(&crit_state);

    _sched_task_pending(task);

    exit_crit_rec(&crit_state);
}

INLINE uint32_t sched_now()
{
    crit_state_t crit_state;
    enter_crit_rec(&crit_state);

    uint32_t now = _sched_now();

    exit_crit_rec(&crit_state);

    return now;
}

/**
 * Returns current time in timer clocks since start.
 *
 * Interrupts must be disabled when calling this function.
 */
INLINE uint32_t _sched_now()
{
    uint32_t offset = scheduler.timer_offset;
    
    // we don't know if a counter overflow happened or not when sampling
    // the counter, so we may need to resample again in case of overflow
    uint16_t cnt = (uint16_t) TIM14->CNT;
    if (TIM14->SR & TIM_SR_UIF) {
	// there was an overflow and scheduler.timer_offset has not been updated yet
	cnt = (uint16_t) TIM14->CNT;
	offset += TIMER_RELOAD + 1;
    }

    return offset + cnt;
}

INLINE int sched_time_lte(uint32_t t1, uint32_t t2)
{
    // note that t1 and t2 wraps around after 2^32 - 1
    return t2 - t1 <= 1u << 31;
}
