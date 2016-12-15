
/*****************************************
 *  Programa de manejo del conjunto de 7 segmentos
 *  de la placa DISEN-EXP
 * Se suponene conectados los 7 segmentos al puerto H
 * Es necesario realizar un refresco del orden de 100MHz
 * para que aprezcan los números estables

   Copyright (C) Alberto F. Hamilton Castro
   Dpto. de Ingeniería de Sistemas y Automática
        y Arquitectura y Tecnología de Comutadores
   Universidad de La Laguna

   $Id: Saca7Segmentos.c $

  Este programa es software libre. Puede redistribuirlo y/o modificarlo bajo
  los términos de la Licencia Pública General de GNU según es publicada
  por la Free Software Foundation, bien de la versión 2 de dicha Licencia
  o bien (según su elección) de cualquier versión posterior.

  Este programa se distribuye con la esperanza de que sea útil, pero
  SIN NINGÚN TIPO DE GARANTÍA, incluso sin la garantía MERCANTIL implícita
  o sin garantizar la CONVENIENCIA PARA UN PROPÓSITO PARTICULAR.
  Véase la Licencia Pública General de GNU para más detalles.


  *************************************** */



#include <sys/param.h>
#include <sys/interrupts.h>
#include <sys/sio.h>
#include <sys/locks.h>

typedef unsigned char byte;  /*por comodidad*/

/*Acceso a IO PORTS como palabra*/
#define _IO_PORTS_W(d)	(((unsigned volatile short*) & _io_ports[(d)])[0])

#define  TCM_FACTOR ( 3 )  /*La potencia de 2 a aplicar al factor*/
#define  TCM_FREQ ( M6812_CPU_E_CLOCK/( 1 << TCM_FACTOR ) )
/*Pasa de microsegundos a ticks*/
#define  USG_2_TICKS(us)  ( ( us ) * ( TCM_FREQ/1000000L ) )
/*Pasa de milisegundos a ticks*/
#define  MSG_2_TICKS(ms)  ( ( ms ) * ( TCM_FREQ/1000L ) )

unsigned short Periodo;
unsigned short cuenta_irqs;

/*Byte que corresponde a cada 7 segmento */
byte digi7s[ 4 ];
byte da; /* digito actual */

int main ( )
{

    /* Deshabilitamos interrupciones */
    lock ( );

    /*Encendemos led*/
    _io_ports[ M6812_DDRG ] |= M6812B_PG7;
    _io_ports[ M6812_PORTG ] |= M6812B_PG7;


    serial_init( );
    serial_print( "\n$Id: Saca7Segmentos.c $\n" );

    /*Inicializamos las variables */
    cuenta_irqs = 0;
    Periodo = USG_2_TICKS( 5000 );
    serial_print( "\n usg del periodo: " );
    serial_printdecword( Periodo/USG_2_TICKS( 1 ) );
    digi7s[ 0 ] = 1;
    digi7s[ 1 ] = 2;
    digi7s[ 2 ] = 3;
    digi7s[ 3 ] = 4;
    da = 0;

    /* Inicializamos los dispositivos */
    /*Inicializamos el puerto H*/
    _io_ports[ M6812_DDRH ] = 0xff; /*todos los pines como salida*/
    _io_ports[ M6812_PORTH ] = 0x91; /*valor inicial*/

    /*Inicialización del Temporizador*/
    _io_ports[ M6812_TMSK2 ] = TCM_FACTOR;
    /* OC2 Invierte el pin en cada disparo */
    _io_ports[ M6812_TCTL2 ] &= ~M6812B_OM2;
    _io_ports[ M6812_TCTL2 ] |= M6812B_OL2;

    /*preparamos disparo*/
    _IO_PORTS_W( M6812_TC2 ) = _IO_PORTS_W( M6812_TCNT ) + Periodo;


    /*configuramos canal 2 como comparador salida*/
    _io_ports[ M6812_TIOS ] |= M6812B_IOS2;


    _io_ports[ M6812_TFLG1 ] = M6812B_IOS2; /*Bajamos el banderín de OC2 */
    _io_ports[ M6812_TMSK1 ] |= M6812B_IOS2; /*habilitamos sus interrupciones*/
    _io_ports[ M6812_TSCR ] = M6812B_TEN; /*Habilitamos temporizador*/

    unlock( ); /* habilitamos interrupciones */

    serial_print( "\n\rTerminada inicialización\n" );

    while( 1 ) {
        unsigned short nv;

        serial_print( "\n\n Cuenta irqs: " );
        serial_printdecword( cuenta_irqs );

        serial_print( "\n\nNuevos valores:" );
        nv = serial_gethexword( );
        /*serparamos los dígitos */
        digi7s[ 0 ] = ( byte )( nv & 0x0f );
        digi7s[ 1 ] = ( byte )( ( nv >> 4 ) & 0x0f );
        digi7s[ 2 ] = ( byte )( ( nv >> 8 ) & 0x0f );
        digi7s[ 3 ] = ( byte )( ( nv >> 12 ) & 0x0f );

        /*Invertimos el led*/
        _io_ports[ M6812_PORTG ] ^= M6812B_PG7;
    }
}


/* Manejador interrupciones del OC2  */
void __attribute__( ( interrupt ) ) vi_ioc2 ( void )
{
    /*Bajamos el banderín de OC2 */
    _io_ports[ M6812_TFLG1 ] = M6812B_IOS2;

    /*preparamos siguiente disparo*/
    _IO_PORTS_W( M6812_TC2 ) = _IO_PORTS_W( M6812_TC2 ) + Periodo;
    cuenta_irqs++;

    /*Refrescamos el display */
    _io_ports[ M6812_PORTH ] = ( digi7s[ da ] & 0x0f ) | ( 1 << ( da + 4 ) );
    da = ( da + 1 ) % 4;
}

