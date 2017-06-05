#pragma once

#include <stm32f0xx.h>

// 16-bit timer counter, count up to maximum value
#define TIMER_RELOAD	((uint16_t) ~0)
// timer ticks for which we spin instead of scheduling another interrupt
#define MAX_SPIN_DELAY	5

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
static inline void sched_task_schedule(task_t *task, uint32_t deadline, uint32_t interval);
static inline void sched_task_cancel(task_t *task);
extern void _sched_task_schedule(task_t *task);
extern void _sched_task_cancel(task_t *task);

static inline uint32_t sched_now();
static inline uint32_t _sched_now();
static inline uint32_t sched_now2(uint16_t *cnt);
static inline uint32_t _sched_now2(uint16_t *cnt);

/*----------------------------------------------------------------------
 * Scheduler inline implementation
 *----------------------------------------------------------------------*/

static inline void sched_task_schedule(task_t *task, uint32_t deadline,
				       uint32_t interval)
{
    __disable_irq();
    task->deadline = deadline;
    task->interval = interval;
    _sched_task_schedule(task);
    __enable_irq();
}

static inline void sched_task_cancel(task_t *task)
{
    __disable_irq();
    _sched_task_cancel(task);
    __enable_irq();
}

static inline uint32_t sched_now()
{
    __disable_irq();
    uint32_t now = _sched_now();
    __enable_irq();

    return now;
}

/**
 * Returns current time in timer clocks since start.
 *
 * Interrupts must be disabled when calling this function.
 */
static inline uint32_t _sched_now()
{
    uint16_t cnt;
    return _sched_now2(&cnt);
}

static inline uint32_t sched_now2(uint16_t *cnt)
{
    __disable_irq();
    uint32_t now = _sched_now2(cnt);
    __enable_irq();

    return now;
}

/**
 * Returns current time in timer clocks since start and current counter value.
 *
 * Interrupts must be disabled when calling this function.
 */
static inline uint32_t _sched_now2(uint16_t *cnt)
{
    uint32_t offset = scheduler.timer_offset;
    
    // we don't know if a counter overflow happened or not when sampling
    // the counter, so we may need to resample again in case of overflow
    *cnt = (uint16_t) TIM14->CNT;
    if (TIM14->SR & TIM_SR_UIF) {
	// there was an overflow and scheduler.timer_offset may not have been
	// updated yet
	*cnt = (uint16_t) TIM14->CNT;
	offset += TIMER_RELOAD + 1;
    }

    return offset + *cnt;
}
