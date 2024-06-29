// Commands for counting edges on GPIO input pins
//
// Copyright (C) 2021  Adrian Keet <arkeet@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h"    // oid_alloc
#include "board/gpio.h" // struct gpio_in
#include "board/irq.h"  // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h"    // DECL_COMMAND
#include "sched.h"      // DECL_TASK

struct pwm_in
{
    struct timer timer;
    uint32_t oid;
    uint32_t max_task_ticks;
    uint8_t flags;
    struct gpio_in pin;
    uint32_t interval;
};

enum
{
    CF_PENDING = 1,
};

static struct task_wake counter_wake;

static uint_fast8_t
pwm_in_event(struct timer *timer)
{
    // called when timer starts
    // measures high time in a single period
    // waits for pin to go low, then high - sets start time
    // once the pin goes low again, curtime - start time = high time
    // if curtime exceeds timeout, high time is 0
    struct pwm_in *c = container_of(timer, struct pwm_in, timer);

    irq_disable();
    uint32_t high_ticks = get_high_ticks(&c->max_task_ticks, &c->pin);
    irq_enable();
    uint32_t oid = c->oid;
    sendf("pwm_in_state oid=%c high_ticks=%u",
          oid, high_ticks);

    // wake in interval ticks
    c->timer.waketime = timer_read_time() + c->interval;
    return SF_RESCHEDULE;
}

static uint_fast32_t
get_high_ticks(uint32_t *max_task_ticks, struct gpio_in *pin)
{
    uint32_t timeout = timer_read_time() + *max_task_ticks;
    uint32_t high_start;
    uint8_t state_val = 0;
    for (;;)
    {
        uint8_t value = gpio_in_read(pin);
        switch (state_val)
        {
        case 0: // waiting for low
            if (!value)
            {
                state_val = 1;
            }
            break;
        case 1: // first high
            if (value)
            {
                high_start = timer_read_time();
                state_val = 2;
            }
            break;
        case 2: // low after high again
            if (!value)
            {
                return timer_read_time() - high_start;
            }
            break;
        }
        if (timer_is_before(timeout, timer_read_time()))
        {
            if (state_val == 0)
            // never seen low, so timeout means 100% pwm
            {
                return max_task_ticks;
            }
            else
            // we've seen low, so timeout means 0% pwm
            {
                return 0;
            }
        }
    }
}

void command_config_pwm_in(uint32_t *args)
{
    struct counter *c = oid_alloc(
        args[0], command_config_pwm_in, sizeof(*c));
    c->pin = gpio_in_setup(args[1], args[2]);
    c->max_task_ticks = args[3];
    c->timer.func = pwm_in_event;
}
DECL_COMMAND(command_config_pwm_in,
             "config_pwm_in oid=%c pin=%u pull_up=%c max_task_ticks=%u");

void command_query_counter(uint32_t *args)
{
    struct counter *c = oid_lookup(args[0], command_config_pwm_in);
    sched_del_timer(&c->timer);
    uint32_t cur = timer_read_time();
    c->timer.waketime = args[1];
    c->interval = args[2];
    sched_add_timer(&c->timer);
}
DECL_COMMAND(command_query_counter,
             "query_pwm_in oid=%c clock=%u interval=%u");

// void counter_task(void)
// {
//     if (!sched_check_wake(&counter_wake))
//         return;

//     uint8_t oid;
//     struct counter *c;
//     foreach_oid(oid, c, command_config_pwm_in)
//     {
//         if (!(c->flags & CF_PENDING))
//             continue;
//         irq_disable();
//         uint32_t waketime = c->timer.waketime;
//         uint32_t count = c->count;
//         uint32_t count_time = c->last_count_time;
//         c->flags &= ~CF_PENDING;
//         irq_enable();
//         sendf("counter_state oid=%c next_clock=%u count=%u count_clock=%u",
//               oid, waketime, count, count_time);
//     }
// }
// DECL_TASK(counter_task);
