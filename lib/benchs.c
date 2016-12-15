/* Benchmark Library
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
Boston, MA 02111-1307, USA.

*/

#include <stdio.h>
#include <benchs.h>
#include <stdarg.h>
#include <stddef.h>
#include <sys/sio.h>
#include <sys/locks.h>

#define HEX_CVT_MASK  (0x0fffffffL)

/* Setup for a 8Mhz quartz, and prescaler set to 1
   (500ns period).  */
#define USEC_PER_TICK (1)
#define USEC_DIVISOR  (2)

static inline void
print (const char *msg)
{
  serial_print (msg);
}

/* Implementation of some libc methods. */
size_t
strlen (const char *s)
{
  const char *p = s;

  while (*p)
    p++;

  return (size_t) (p - s);
}

/* Report the result of the benchmark.  A correction corresponding
   to the benchmark operation overhead is applied.  */
void
bench_report (bench_t *b, const char *msg, ...)
{
  va_list argp;
  unsigned short dt;
  unsigned long usec;
  char buf[40];
  size_t len;

  /* Compute duration of the benchmarked operation (in number of TCNT
     ticks).  A wrap arround of the TCNT counter is possible but the
     computation will be good since values are truncated to 16-bit.
     More than one wrap arround will produce incorrect result, but
     we assume all benchmark operations are less than few ms.  */
  dt = b->b_stop - b->b_start;
  if (dt > b->b_correction)
    dt -= b->b_correction;

  b->b_delta = dt;

  /* Translate number of ticks to micro-seconds.  */
  usec = ((unsigned long) (dt) * USEC_PER_TICK) / USEC_DIVISOR;

  va_start (argp, msg);
  vsprintf (buf, msg, argp);
  va_end (argp);

  print (buf);
  len = strlen (buf);
  while (len < 40)
    {
      print (" ");
      len++;
    }
  sprintf (buf, "%ld us", usec);
  len = strlen (buf);
  while (len < 10)
    {
      print (" ");
      len++;
    }
  print (buf);

  sprintf (buf, " [%ld]\n", (long) dt);
  print (buf);
}

/* Benchmark overhead.  This operation benchmarks a "nop" to compute
   the correction applied on each benchmark computation.  */
void
bench_empty (bench_t *b)
{
  
  bench_start (b);
  __asm__ __volatile__ ("nop");
  bench_stop (b);

  b->b_correction = 0;
  bench_report (b, "Empty bench (correction)");
  b->b_correction = b->b_delta;
}

void
bench_init (bench_t *b)
{
  /* Lock interrupts to make sure we are not interrupted while
     doing a measure.  */
  lock ();

  bench_empty (b);
}
