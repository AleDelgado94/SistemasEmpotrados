/* param.h - Board specific parameters
   Copyright (C) 2000, 2001 Free Software Foundation, Inc.
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

#ifndef _M68HC11_PARAM_H
#define _M68HC11_PARAM_H

/*! @defgroup params System and Board Parameters.

   This section contains several \b #define to give configuration
   characteristics of the target board.
   
*/
/*@{*/

/* Axiom Manufacturing CME11E9-EVBU board definitions.  */
#ifdef AX_CME11E9
# define M6811_DEF_BAUD 0x30            /* 9600 baud.  */
# define RAM_SIZE       32768           /* 32Kb of memory (43256).  */
# define ROM_SIZE       8192            /* 8Kb eeprom (2864).  */
# define DATA_SIZE      (0x1000-0x040)  /* Data section size.  */
# define TEXT_SIZE      8192            /* Text section size.  */
#endif

/* Generic 32K board.  */
#ifdef M6811_32K
# define M6811_DEF_BAUD 0x30            /* 9600 baud.  */
# define RAM_SIZE       32768           /* 32Kb of memory.  */
# define ROM_SIZE       32768           /* 32Kb of rom.  */
# define DATA_SIZE      (0x8000-0x1040) /* Data section size.  */
# define TEXT_SIZE      ROM_SIZE        /* Text section size.  */
#endif

/* EBCS board.  */
#ifdef EBCS
# define M6811_DEF_BAUD 0x30            /* 1200 baud (bootstrap mode).  */
# define RAM_SIZE       8192            /* 8K of ram, 512b for data.  */
# define ROM_SIZE       7680            /* the rest for text  */
# define DATA_SIZE      (0x1ff)         /* Data section size.  */
# define TEXT_SIZE      ROM_SIZE        /* Text section size.  */
#endif

/*@{*/

/*! Data section size.

    Define the size of the data section.  This is used by some examples
    to tune and setup some of their internal tables.  */
#ifndef DATA_SIZE
# define DATA_SIZE      (1024)
#endif

/*! Text section size.

    Define the size of the text section.  This is used by some examples
    to tune and setup some of their data types and features.  */
#ifndef TEXT_SIZE
# define TEXT_SIZE      (1024)
#endif

/*! CPU Clock frequency.
 
    Define the frequency of the oscillator plugged on the processor.
    The value is in hertz.  By default, use 8Mhz.  */
#ifndef M6811_CPU_CLOCK
# define M6811_CPU_CLOCK (8000000L)
#endif

/*! CPU E clock.
    
    The E clock frequency.  This frequency is used as the
    basis for timer computation.  The value is in hertz.  */
#ifndef M6811_CPU_E_CLOCK
# define M6811_CPU_E_CLOCK (M6811_CPU_CLOCK / 4)
#endif


/*! SIO default baud rate.
    
    Defines the default baud rate of the SIO.  This baud rate
    is used to configure the M6811_BAUD register.
    <ul>
      <li>0x33 -> 1200 baud (8Mhz cpu)
      <li>0x30 -> 9600 baud (8Mhz cpu)
    </ul>  */
#ifndef M6811_DEF_BAUD
# define M6811_DEF_BAUD   0x33
#endif

/*! Use the 68HC11 COP.
    
    Define this if you are using the COP timer.
    This activate the COP reset while polling and writing on
    the serial line.  */
#ifndef M6811_USE_COP
# define M6811_USE_COP 0
#endif

/*@}*/

#endif
