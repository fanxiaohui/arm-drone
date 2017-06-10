#pragma once

#include <stm32f0xx.h>

#include "utils.h"

// 16-bit timer counter, count up to maximum value
// #define TIMER_RELOAD	((uint16_t) ~0)
// 1ms
#define TIMER_RELOAD	999

/*----------------------------------------------------------------------
 * Scheduler public interface
 *----------------------------------------------------------------------*/

struct task_st;
typedef volatile struct task_st task_t;

typedef enum {
    TASK_UNDEFINED = 0,
    TASK_INITIALISED,
    TASK_SCHEDULED,		// task is in the scheduled queue
    TASK_CANCELLED		// task is in the cancelled queue
} state_t;

typedef void (*TaskFn)(task_t *task, state_t prev_state, uint32_t expiry_time);

struct task_st {
    state_t	state;

    task_t	*next;		// linked list of tasks
    uint32_t	deadline;	// absolute deadline of next execution in timer
    				// clock ticks, updated after each execution
    uint32_t	interval;	// interval in timer clock ticks for repetitive tasks,
                                // or 0 for one-off tasks
    TaskFn	task_fn;
    void	*client_data;
};

struct scheduler_st {
    task_t	*scheduled_head; // sorted linked list of SCHEDULED tasks, or NULL
    task_t	*cancelled_head; // non-sorted linked list of CANCELLED tasks, or NULL

    uint32_t	timer_offset;	 // current time at most recent TIMx->CNT overflow
};

typedef volatile struct scheduler_st scheduler_t;

extern scheduler_t scheduler;

extern void sched_init();
extern void sched_task_init(task_t *task, TaskFn task_fn, void *client_data);
INLINE void sched_task_schedule(task_t *task, uint32_t deadline, uint32_t interval);
INLINE void sched_task_cancel(task_t *task);
extern void _sched_task_schedule(task_t *task);
extern void _sched_task_cancel(task_t *task);

INLINE uint32_t sched_now();
INLINE uint32_t _sched_now();

/*----------------------------------------------------------------------
 * Scheduler inline implementation
 *----------------------------------------------------------------------*/

INLINE void sched_task_schedule(task_t *task, uint32_t deadline,
				       uint32_t interval)
{
    crit_state_t crit_state;
    enter_crit_rec(&crit_state);

    task->deadline = deadline;
    task->interval = interval;
    _sched_task_schedule(task);

    exit_crit_rec(&crit_state);
}

INLINE void sched_task_cancel(task_t *task)
{
    crit_state_t crit_state;
    enter_crit_rec(&crit_state);

    _sched_task_cancel(task);

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
