/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  main loop scheduler for APM
 *  Author: Andrew Tridgell, January 2013
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

#include <AP_HAL.h>
#include <AP_Scheduler.h>
#include <AP_Param.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Scheduler::var_info[] PROGMEM = {
    // @Param: DEBUG
    // @DisplayName: Scheduler debug level
    // @Description: Set to non-zero to enable scheduler debug messages
    // @Values: 0:Disabled,1:ShowSlipe,2:ShowOverruns
    // @User: Advanced
    AP_GROUPINFO("DEBUG",    0, AP_Scheduler, _debug, 0),
    AP_GROUPEND
};

// initialise the scheduler
void AP_Scheduler::init(const AP_Scheduler::Task *tasks, uint8_t num_tasks) 
{
    _tasks = tasks;
    _num_tasks = num_tasks;
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _tick_counter = 0;
}

// one tick has passed
void AP_Scheduler::tick(void)
{
    _tick_counter++;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void AP_Scheduler::run(uint16_t time_available)
{
    for (uint8_t i=0; i<_num_tasks; i++) {
        uint16_t dt = _tick_counter - _last_run[i];
        uint16_t interval_ticks = pgm_read_word(&_tasks[i].interval_ticks);
        if (dt >= interval_ticks) {
            // this task is due to run. Do we have enough time to run it?
            _task_time_allowed = pgm_read_word(&_tasks[i].max_time_micros);

            if (dt >= interval_ticks*2) {
                // we've slipped a whole run of this task!
                if (_debug != 0) {
                    hal.console->printf_P(PSTR("Scheduler slip task[%u] (%u/%u/%u)\n"), 
                                          (unsigned)i, 
                                          (unsigned)dt,
                                          (unsigned)interval_ticks,
                                          (unsigned)_task_time_allowed);
                }
            }
            if (_task_time_allowed <= time_available) {
                // run it
                _task_time_started = hal.scheduler->micros();
                task_fn_t func = (task_fn_t)pgm_read_pointer(&_tasks[i].function);
                func();
                
                // record the tick counter when we ran. This drives
                // when we next run the event
                _last_run[i] = _tick_counter;
                
                // work out how long the event actually took
                uint32_t time_taken = hal.scheduler->micros() - _task_time_started;
                
                if (time_taken > _task_time_allowed) {
                    // the event overran!
                    if (_debug > 1) {
                        hal.console->printf_P(PSTR("Scheduler overrun task[%u] (%u/%u)\n"), 
                                              (unsigned)i, 
                                              (unsigned)time_taken,
                                              (unsigned)_task_time_allowed);
                    }
                    return;
                }
                time_available -= time_taken;
            }
        }
    }
}

/*
  return number of micros until the current task reaches its deadline
 */
uint16_t AP_Scheduler::time_available_usec(void)
{
    uint32_t dt = hal.scheduler->micros() - _task_time_started;
    if (dt > _task_time_allowed) {
        return 0;
    }
    return _task_time_allowed - dt;
}

