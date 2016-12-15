/* M68HC11/68HC12 exit function (board specific)
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

#include <sys/param.h>

#ifdef AX_CME11E9
# define USE_BUFFALO_EXIT
#endif

#ifdef USE_BUFFALO_EXIT
/* For Buffalo monitor, use swi to enter in the monitor upon exit.  */
static inline void
do_exit (short status)
{
  __asm__ __volatile__ ("swi" : : "d"(status));
}
#else

/* For the simulator, the wai stops everything and exits with the
   error code stored in register d.  */
static inline void
do_exit (short status)
{
  /* Use 'd' constraint to force the status to be in the D
     register before execution of the asm.  */
  __asm__ __volatile__ ("cli\n"
                        "wai" : : "d"(status));
}
#endif

volatile void
exit (int status)
{
  while (1)
    {
      do_exit (status);
    }
}
