/* Integer Math - the math.h for integer operations
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

#ifndef _IMATH_H
#define _IMATH_H

/*! @defgroup imath Integer Maths Operations.

*/
/*@{*/

/*! Compute the square root of a number.

    This function computes the square root of \a x using integer operations
    exclusively.  It returns the integer that corresponds to \c sqrt(x).

    <p>
    This function operates on unsigned numbers and as such it always
    succeeds.

    @return the square root of x.
    @see sqrt, lsqrt64
*/
extern unsigned long
lsqrt (unsigned long x);

/*! Compute the square root of a number.

    This function computes the square root of \a x using integer operations
    exclusively.  It returns the integer that corresponds to \c sqrt(x).
    \c lsqrt64 deals with 64-bit integers while \c lsqrt only handles
    32-bit integers.

    <p>
    This function operates on unsigned numbers and as such it always
    succeeds.

    @return the square root of x.
    @see sqrt, lsqrt
*/
extern unsigned long long
lsqrt64 (unsigned long long x);

/*@}*/

#endif
