/* asm-m68hc11/sio.h -- Utility methods to read/write the SIO
   Copyright 1999, 2000, 2001 Free Software Foundation, Inc.
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

#ifndef _M68HC11_SIO_H
#define _M68HC11_SIO_H

/*! @defgroup sio Serial Input Output

   
 */
/*@{*/

/*! Initialize the SCI.

    This function configures the SCI to send at \c M6811_DEF_BAUD baud
    rate.  It must be called before any other serial operation unless
    the program is started from another one (such as a monitor or
    in bootstrap mode).

    @see serial_print, M6811_DEF_BAUD
*/
extern inline void
serial_init (void)
{
  _io_ports[M6811_BAUD] = M6811_DEF_BAUD;

  /* Setup character format 1 start, 8-bits, 1 stop.  */
  _io_ports[M6811_SCCR1] = 0;

  /* Enable reciever and transmitter.  */
  _io_ports[M6811_SCCR2] = 0xc;
}

/*! Test for pending character on the reception.
    
    Return != 0 if there is something to read on the serial line.

    @return 1 if there is something to read on the serial line.
*/
extern inline unsigned char
serial_receive_pending (void)
{
  return _io_ports[M6811_SCSR] & M6811_RDRF;
}

/*! Wait until the SIO has finished to send the character.

    This function waits for the transmission of the current character.
    While waiting, the COP is reset using \c cop_optional_reset.

    @see cop_optional_reset, serial_init, serial_send
*/
extern inline void
serial_flush (void)
{
  while (!(_io_ports[M6811_SCSR] & M6811_TDRE))
    cop_optional_reset ();
}

/*! Send the character on the serial line.

    This function sends the character \a c on the serial line.
    Before sending, it calls \c serial_flush to make sure the
    transmitter is ready.  Once the function returns, the
    character is in the SCI queue and it may not be sent completely
    over the serial line.

    @param c character to send on the serial line.

    @see serial_init, serial_flush
*/
extern inline void
serial_send (char c)
{
  serial_flush ();
  _io_ports[M6811_SCDR] = c;
  _io_ports[M6811_SCCR2] |= M6811_TE;
}

/*! Wait for a character on the serial line and return it.

    This function waits for a character to be received by the SCI.
    While waiting, it calls \c cop_optional_reset to probe the COP
    regularly.

    @return the character received from the SCI.

    @see serial_init, cop_optional_reset
*/
extern inline unsigned char
serial_recv (void)
{
  while (!(_io_ports[M6811_SCSR] & M6811_RDRF))
    cop_optional_reset ();

  return _io_ports[M6811_SCDR];
}

/*@}*/

#endif /* _M68HC11_SIO_H */

