/* Locks -- Operations to protect critical sections
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

#ifndef _M68HC11_LOCKS_H
#define _M68HC11_LOCKS_H

/**@name Locking operations.

   Locking operations are defined to allow the use of two strategies to
   protect critical sections from interrupts:

   <dl>
   <dt>Lock/Unlock
   <dd>
      This simple strategy consists in disabling interrupts (sei) and
      enabling them after the critical section.  In general,
      this method assumes the processor accepts interrupts when lock()
      is called.  This strategy is not suitable to implement functions
      that will be called with interrupts masked because this will create
      a side-effect: enable interrupts.

      <p>
      Example:
      <pre>
        lock ();
        ...
        unlock ();
      </pre>

   <dt>Lock/Restore
   <dd>
      This strategy is suitable to protect critical sections in
      functions that will be called with interrupts masked.
      The processor mask is saved before locking interrupts.
      It is restored at the end of the critical section.  When entering
      the critical section, if interrupts are masked, they will
      remain masked at end of critical section.  There is no side effect.

      <p>
      Example:
      <pre>
        unsigned short mask;

        mask = lock ();
        ...
        restore (mask);
      </pre>
    </dl>
 */
/*@{*/
  
/** Lock the processor to prevent interrupts to occur.
    When the processor is locked, interrupts are not accepted.
    They are deferred until the processor is unlocked.
    The previous locking mask is returned in the high part
    (bits 15..8) to avoid a `tab' instruction.

    @return the previous locking mask.
    @see unlock, restore   */
static __inline__ unsigned short
lock (void)
{
  unsigned short mask;

  __asm__ __volatile__ ("tpa\n\tsei" : "=d"(mask));
  return mask;
}

/** Unlock the processor to accept interrupts.
    When the processor is unlocked, interrupts are accepted.
    Use this operation if you want, authoritatively, accept
    the interrupts.

    @see lock, restore  */
static __inline__ void
unlock (void)
{
  __asm__ __volatile__ ("cli");
}

/** Restore the interrupt mask of the processor.
    The mask is assumed to be in the high part (bits 15..8)
    to avoid a `tba' instruction. 

    @param mask the previous interrupt mask to restore
    @see lock, unlock  */
static __inline__ void
restore (unsigned short mask)
{
  __asm__ __volatile__ ("tap" : : "d"(mask));
}
/*@}*/

#endif
