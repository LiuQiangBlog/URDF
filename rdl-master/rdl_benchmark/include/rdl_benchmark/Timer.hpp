// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef _TIMER_H
#define _TIMER_H

#include <ctime>

struct TimerInfo
{
    /// time stamp when timer_start() gets called
    double clock_start_value;

    /// time stamp when the timer was stopped
    double clock_end_value;

    /// duration between clock_start_value and clock_end_value in seconds
    double duration_sec;
};

inline void timer_start(TimerInfo* timer)
{
    struct timespec system_time_; /**< time struct */
    clock_gettime(CLOCK_REALTIME, &system_time_);

    timer->clock_start_value = static_cast<double>(system_time_.tv_sec) + static_cast<double>(system_time_.tv_nsec) * 1.e-9;
}

inline double timer_stop(TimerInfo* timer)
{
    struct timespec system_time_; /**< time struct */
    clock_gettime(CLOCK_REALTIME, &system_time_);

    timer->clock_end_value = static_cast<double>(system_time_.tv_sec) + static_cast<double>(system_time_.tv_nsec) * 1.e-9;

    timer->duration_sec = timer->clock_end_value - timer->clock_start_value;

    return timer->duration_sec;
}

#endif
