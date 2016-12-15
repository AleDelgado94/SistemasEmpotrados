/* ************************************************

  Ejemplo de uso de las distintas funciones de lectura y escritura de números
  que están disponibles en la librería <sys/sio.h>


   Copyright (C) Alberto F. Hamilton Castro
   Dpto. de Ingeniería de Sistemas y Automática
        y Arquitectura y Tecnología de Comutadores
   Universidad de La Laguna

   $Id: UtilidadesSeriales.c $

  Este programa es software libre. Puede redistribuirlo y/o modificarlo bajo
  los términos de la Licencia Pública General de GNU según es publicada
  por la Free Software Foundation, bien de la versión 2 de dicha Licencia
  o bien (según su elección) de cualquier versión posterior.

  Este programa se distribuye con la esperanza de que sea útil, pero
  SIN NINGÚN TIPO DE GARANTÍA, incluso sin la garantía MERCANTIL implícita
  o sin garantizar la CONVENIENCIA PARA UN PROPÓSITO PARTICULAR.
  Véase la Licencia Pública General de GNU para más detalles.


  *************************************** */

#include <sys/interrupts.h>
#include <sys/sio.h>



typedef unsigned char byte;  /*por comodidad*/
typedef unsigned short word;  /*por comodidad*/

int main( ) {
    byte nb,nb2;
    word sa;

    serial_init( );
    serial_print( "\n$Id: UtilidadesSeriales.c $\n" );
    while( 1 ) {

        serial_print( "\n\nEntra byte decimal:" );
        nb = serial_getdecbyte( );
        serial_print( "\nNumero entrado = 0b" );
        serial_printbinbyte( nb );
        serial_print( "\nNumero entrado = 0x" );
        serial_printhexbyte( nb );
        serial_print( "\nNumero entrado = " );
        serial_printdecbyte( nb );

        serial_print( "\nEntra número hexa:" );
        nb2 = serial_gethexbyte( );
        serial_print( "\nNumero entrado = 0x" );
        serial_printhexbyte( nb2 );
        serial_print( "\nNumero entrado = 0b" );
        serial_printbinbyte( nb2 );
        serial_print( "\nNumero entrado = " );
        serial_printdecbyte( nb2 );

        serial_print( "\n\nEntra word decimal:" );
        sa = serial_getdecword( );
        serial_print( "\nNumero entrado = 0b" );
        serial_printbinword( sa );
        serial_print( "\nNumero entrado = 0x" );
        serial_printhexword( sa );
        serial_print( "\nNumero entrado = y" );
        serial_printdecword( sa );

    }
}

