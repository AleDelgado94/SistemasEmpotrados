
/*****************************************
* Salida onda cuadrada por el OC2
* Se especifica periondo en variable.

   Copyright (C) Alberto F. Hamilton Castro
   Dpto. de Ingeniería de Sistemas y Automática
        y Arquitectura y Tecnología de Comutadores
   Universidad de La Laguna

   $Id: OndaCuadradaOC2.c $

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
#define  TCM_FREQ ( M6812_CPU_E_CLOCK / ( 1 << TCM_FACTOR ) )
/*Pasa de microsegundos a ticks*/
#define  USG_2_TICKS(us)  ( ( us ) * ( TCM_FREQ / 1000000L ) )
/*Pasa de milisegundos a ticks*/
#define  MSG_2_TICKS(ms)  ( ( ms ) * ( TCM_FREQ / 1000L ) )

unsigned short Periodo; /* Ticks del temporizador que dura el periodo */
unsigned short cuenta_irqs; /* Se incremente en cada interrupción */


int main ( )
{

    /* Deshabilitamos interrupciones */
    lock ( );

    /*Encendemos led*/
    _io_ports[ M6812_DDRG ] |= M6812B_PG7;
    _io_ports[ M6812_PORTG ] |= M6812B_PG7;


    serial_init( );
    serial_print( "\r\n$Id: OndaCuadradaOC2.c $\r\n" );

    cuenta_irqs = 0;

    /* Iniciamos periodo según microsegundos que queremos que dure */
    Periodo = USG_2_TICKS( 5000 );  /* se hace en tiempo de COMPILACION */
    serial_print( "\r\n usg del periodo: " );
    serial_printdecword( Periodo/USG_2_TICKS( 1 ) );

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

    serial_print( "\n\rTerminada inicialización" );

    while( 1 ) {
        char c;
        c = serial_recv( );
        //Hacemos eco del caracter recibido
        serial_send( c );

        serial_print( "\n\r Cuenta irqs: " );
        serial_printdecword( cuenta_irqs );

        if( c == '+' ) Periodo += 10;
        if( c == '-' ) Periodo -= 10;

        serial_print( "\n\r Periodo: ");
        serial_printdecword( Periodo );
        serial_print( "\tusg del periodo: " );
        serial_printdecword( Periodo / USG_2_TICKS( 1 ) );

        /*Invertimos el led*/
        _io_ports[ M6812_PORTG ] ^= M6812B_PG7;

    }
}


/* Manejador interrupciones del OC2  */
void __attribute__( ( interrupt ) ) vi_ioc2 ( void )
{
    _io_ports[ M6812_TFLG1 ] = M6812B_IOS2; /*Bajamos el banderín de OC2 */

    /*preparamos siguiente disparo*/
    _IO_PORTS_W( M6812_TC2 ) = _IO_PORTS_W( M6812_TC2 ) + Periodo;
    cuenta_irqs++;

}
