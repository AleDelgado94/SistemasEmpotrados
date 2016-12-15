/* sys/ports_def.h -- Definition of system ports
   Copyright 2000 Free Software Foundation, Inc.
   Written by Stephane Carrez (stcarrez@worldnet.fr)

This file is part of GDB, GAS, and the GNU binutils.

GDB, GAS, and the GNU binutils are free software; you can redistribute
them and/or modify them under the terms of the GNU General Public
License as published by the Free Software Foundation; either version
1, or (at your option) any later version.

GDB, GAS, and the GNU binutils are distributed in the hope that they
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this file; see the file COPYING.  If not, write to the Free
Software Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

#ifndef _SYS_PORTS_DEF_H
#define _SYS_PORTS_DEF_H

#ifdef mc6811
# include <asm-m68hc11/ports_def.h>
#endif

#ifdef mc6812
# include <asm-m68hc12/ports_def.h>
#endif

#endif /* _SYS_PORTS_DEF_H */

