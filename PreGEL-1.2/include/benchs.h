/* Benchmarks
   Copyright (C) 2001 Free Software Foundation, Inc.
   Written by Stephane Carrez (stcarrez@worldnet.fr)	

This file is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option) any
later version.

In addition to the permissions in the GNU General Public License, the
Free Software Foundation gives you unlimited permission to link the
compiled version of this file with other programs, and to distribute
those programs without any restriction coming from the use of this
file.  (The General Public License restrictions do apply in other
respects; for example, they cover modification of the file, and
distribution when not linked into another program.)

This file is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; see the file COPYING.  If not, write to
the Free Software Foundation, 59 Temple Place - Suite 330,
Boston, MA 02111-1307, USA.  */

#ifndef _BENCHS_H
#define _BENCHS_H

/*! @defgroup benchs Benchmark Operations

   The benchmark functions are designed to benchmark some piece of
   code in a precise and deterministic manner.  Interrupts are masked
   during the benchmark.  The benchmark uses the free running counter
   that exists in 68HC11 and 68HC12 (see \c get_timer_counter ).  The
   implementation assumes that the free running counter can overflow
   only once during a benchmark.  This imposes a restrictions on the
   benchmarked operation: they must be fast enough to not exceed
   65536 times the update of the free running counter. For example
   on a 68HC11 @ 8Mhz they should not exceed 32ms.

   The benchmark data is recorded in the \c bench_t data type.
   It must be initialized using \c bench_init. For example:

   <pre>
       bench_t b;

       bench_init (&b);
   </pre>

   Then, the piece of code that must be benchmarked is enclosed by
   calls to \c bench_start and \c bench_stop as follows:

   <pre>
       bench_start (&b);
       ...
       bench_stop (&b);
       bench_report (&b, "Operation");
   </pre>

   The \c bench_report function must be called to compute the time
   and print the result.

*/
/*@{*/

#include <sys/param.h>
#include <sys/ports.h>

/*! Benchmark data type.

    The \c bench structure records the data for the benchmark.
 */
struct bench
{
  /** Start time (value of TCNT at beginning of benchmark).  */
  unsigned short b_start;

  /** Stop time (value of TCNT at end of benchmark).  */
  unsigned short b_stop;

  /** Correction factor that must be applied to take into account
      the overhead due to the 'bench_start' and 'bench_end' calls.  */
  unsigned short b_correction;

  /** After the call to 'bench_report', the corrected result.  */
  unsigned short b_delta;
};
typedef struct bench bench_t;

/*! Initialize the benchmark object.

    This function must be called to initialize the benchmark object.
    It locks the interrupts and computes the benchmark correction
    that will be applied to the results.  The benchmark correction
    is based on the benchmark operations overhead computed using
    \c bench_empty.

    @param b pointer to benchmark record object
    @see bench_start, bench_stop, bench_empty
*/
extern void
bench_init (bench_t *b);

/*! Start the benchmark.

    This function starts the benchmark operation by recording
    the precise time at which the measure begins.

    @param b pointer to benchmark record object
    @see bench_stop, bench_init
*/
extern __inline__ void
bench_start (bench_t *b)
{
  b->b_start = get_timer_counter ();
}

/*! Stop the benchmark.

    This function finishes the benchmark operation by recording
    the precise time at which the measure stops.

    @param b pointer to benchmark record object
    @see bench_start, bench_init
*/
extern __inline__ void
bench_stop (bench_t *b)
{
  b->b_stop = get_timer_counter ();
}

/*! Report benchmark results.

    This function computes the time spent between the \c bench_start
    and \c bench_stop calls.  It then reports the result.

    @param b pointer to benchmark record object
    @param msg format description string
    @see bench_start, bench_stop
 */
extern void
bench_report (bench_t *b, const char *msg, ...);


/*! Benchmark overhead computation.

    This function computes the overhead of the benchmark operations
    \c bench_start and \c bench_stop.

    @param b pointer to benchmark record object
    @see bench_init
*/
extern void
bench_empty (bench_t *b);

/*@}*/

#endif
