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
    struct gpio_in pin;
    uint32_t interval;
    uint32_t period;
};

static uint32_t
get_high_time_and_set_period(struct pwm_in *p)
{
    // should see at most 1.5 periods
    uint32_t max_high_time = p->period;
    uint32_t deadline_ticks = (uint32_t)(p->period * 1.7);// extra 0.2 for timing grace

    uint32_t deadline = timer_read_time() + deadline_ticks;
    uint32_t first_high_time = 0;
    uint32_t second_low_time = 0;
    uint32_t second_high_time = 0;
    uint32_t cur_time;
    struct gpio_in pin = p->pin;
    uint8_t value;

    for (;;) // waiting for first low
    {
        value = gpio_in_read(pin);
        cur_time = timer_read_time();

        if (deadline < cur_time)
            return (uint32_t)max_high_time; // never see low, so 100% pwm
        if (!value)
            break;
    }

    for (;;) // waiting for first high
    {
        value = gpio_in_read(pin);
        cur_time = timer_read_time();

        if (deadline < cur_time)
            return 0; // never see high, so 0% pwm
        if (value)
        {
            first_high_time = cur_time;
            break;
        }
    }

    for (;;) // waiting for second low
    {
        value = gpio_in_read(pin);
        cur_time = timer_read_time();

        if (deadline < cur_time)
            return (uint32_t)max_high_time; // time out waiting for second low means 100%
        if (!value)
        {
            second_low_time = cur_time;
            break;
        }
    }

    for (;;) // waiting for second high
    {
        value = gpio_in_read(pin);
        cur_time = timer_read_time();

        if (deadline < cur_time)
            break;
        if (value)
        {
            second_high_time = cur_time;
            break;
        }
    }
    if (second_high_time) // can only calc period after second high
        p->period = second_high_time - first_high_time;

    uint32_t high_time = second_low_time - first_high_time;
    return high_time;
}

static uint_fast8_t
pwm_in_event(struct timer *timer)
{
    // called when timer starts
    // measures high time in a single period
    // waits for pin to go low, then high - sets start time
    // once the pin goes low again, curtime - start time = high time
    // if curtime exceeds timeout, high time is 0
    struct pwm_in *p = container_of(timer, struct pwm_in, timer);

    irq_disable();
    uint32_t high_ticks = get_high_time_and_set_period(p);
    irq_enable();
    uint32_t oid = p->oid;
    uint32_t period = p->period;
    sendf("pwm_in_state oid=%c high_ticks=%u period=%u",
          oid, high_ticks, period);

    // wake in interval ticks
    p->timer.waketime = timer_read_time() + p->interval;
    return SF_RESCHEDULE;
}

void command_config_pwm_in(uint32_t *args)
{
    struct pwm_in *p = oid_alloc(
        args[0], command_config_pwm_in, sizeof(*p));
    p->pin = gpio_in_setup(args[1], args[2]);
    p->timer.func = pwm_in_event;
}
DECL_COMMAND(command_config_pwm_in,
             "config_pwm_in oid=%c pin=%u pull_up=%c");

void command_query_pwm_in(uint32_t *args)
{
    struct pwm_in *p = oid_lookup(args[0], command_config_pwm_in);
    sched_del_timer(&p->timer);
    p->timer.waketime = args[1];
    p->interval = args[2];
    p->period = args[3];
    sched_add_timer(&p->timer);
}
DECL_COMMAND(command_query_pwm_in,
             "query_pwm_in oid=%c clock=%u interval=%u period=%u");

// void counter_task(void)
// {
//     if (!sched_check_wake(&counter_wake))
//         return;

//     uint8_t oid;
//     struct counter *c;
//     foreach_oid(oid, c, command_config_pwm_in)
//     {
//         if (!(p->flags & CF_PENDING))
//             continue;
//         irq_disable();
//         uint32_t waketime = p->timer.waketime;
//         uint32_t count = p->count;
//         uint32_t count_time = p->last_count_time;
//         p->flags &= ~CF_PENDING;
//         irq_enable();
//         sendf("counter_state oid=%c next_clock=%u count=%u count_clock=%u",
//               oid, waketime, count, count_time);
//     }
// }
// DECL_TASK(counter_task);
