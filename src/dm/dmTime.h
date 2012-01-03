/*****************************************************************************
 * DynaMechs: A Multibody Dynamic Simulation Library
 *
 * Copyright (C) 1994-2001  Scott McMillan   All Rights Reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *****************************************************************************
 *     File: dmTime.h
 *   Author: Scott McMillan
 *  Created: 30 June 1999
 *  Summary: Cross platform timing for test programs
 *****************************************************************************/

#ifndef __DM_TIME_H
#define __DM_TIME_H

#include <dm.h>

//----------------------------------------------------------------------------
#if defined(WIN32)
#   include <sys/timeb.h>
#   include <time.h>
struct dmTimespec
{
    time_t   tv_sec;     // seconds
    long     tv_nsec;    // and nanoseconds
};

#else
    #include <unistd.h>
    #if defined(_POSIX_TIMERS)
        #include <time.h>
    #endif
    //#if defined(__APPLE__)
    #include <sys/time.h>
    //#endif

    typedef timespec dmTimespec;
#endif

inline void dmGetSysTime(dmTimespec *ts)
{
#if defined(WIN32)
    struct _timeb timebuffer;
    _ftime(&timebuffer);

    ts->tv_sec = timebuffer.time;
    ts->tv_nsec = timebuffer.millitm*1000000;

#elif defined(_POSIX_TIMERS)
    if (clock_gettime(CLOCK_REALTIME, ts) != 0)
    {
        throw ts;
    }
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    ts->tv_sec = tv.tv_sec;
    ts->tv_nsec = 1000*tv.tv_usec;
#endif
}

inline double dmTimespecToDouble(dmTimespec *ts)
{
   return (double)ts->tv_sec + ((double)ts->tv_nsec * 1.0e-9);
}

#endif
