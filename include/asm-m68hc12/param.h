/* asm-m68hc12/param.h - Board specific parameters
   Copyright (C) 2000 Free Software Foundation, Inc.
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

#ifndef _M68HC12_PARAM_H
#define _M68HC12_PARAM_H

/**@name M68HC12 Board Parameters.

   This section contains several '#define' to give configuration
   characteristics of the target board.  */
/*@{*/

/** CPU Clock frequency.
 
   Define the frequency of the oscillator plugged on the processor.
   The value is in hertz.  By default, use 32Mhz.  */
#ifndef M6812_CPU_CLOCK
# define M6812_CPU_CLOCK (32000000L)
#endif

/** CPU E clock.
    
   The E clock frequency.  This frequency is used as the
   basis for timer computation.  The value is in hertz.  */
#ifndef M6812_CPU_E_CLOCK
# define M6812_CPU_E_CLOCK (M6812_CPU_CLOCK / 4)
#endif

#ifndef M6812_CPU_M_CLOCK
# define M6812_CPU_M_CLOCK (M6812_CPU_E_CLOCK)
#endif

/** SIO default baud rate.
    
   Defines the default baud rate of the SIO.  This baud rate
   is used to configure the M6812_SCxBD register.  */
#ifndef M6812_DEF_BAUD
/*# define M6812_DEF_BAUD   (M6812_CPU_M_CLOCK / (16 * 9600L))*/
# define M6812_DEF_BAUD   (M6812_CPU_M_CLOCK / (16 * 38400L))
#endif

#ifndef M6812_STDOUT_PORT
# define M6812_STDOUT_PORT (0)
#endif

/** Use the 68HC11 COP.
    
   Define this if you are using the COP timer.
   This activate the COP reset while polling and writing on
   the serial line.  */
#define M6811_USE_COP 0

/*@}*/

#endif
