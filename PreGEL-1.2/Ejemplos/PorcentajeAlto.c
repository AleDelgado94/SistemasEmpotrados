
/*****************************************
* Determina el procentaje (respecto al peridodo total)
* que está una señal en alto y lo envía por la serial
* Señal se recibe por IC1
*

   Copyright (C) Alberto F. Hamilton Castro
   Dpto. de Ingeniería de Sistemas y Automática
        y Arquitectura y Tecnología de Comutadores
   Universidad de La Laguna

   $Id: PorcentajeAlto.c $

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
typedef unsigned short word;  /*por comodidad*/

/*Acceso a IO PORTS como palabra*/
#define _IO_PORTS_W(d)	(((unsigned volatile short*) & _io_ports[(d)])[0])

#define  TCM_FACTOR (3 )  /*La potencia de 2 a aplicar al factor*/
#define  TCM_FREQ ( M6812_CPU_E_CLOCK/( 1 << TCM_FACTOR ) )
/*Pasa de microsegundos a ticks*/
#define  USG_2_TICKS(us)  ( ( us ) * ( TCM_FREQ/1000000L ) )
/*Pasa de milisegundos a ticks*/
#define  MSG_2_TICKS(ms)  ( ( ms ) * ( TCM_FREQ/1000L ) )

/* Variables globales para almacenar los tres instantes de tiempo necesarios*/
word Bajada0, Bajada1, Subida;

/* Banderín para que la RTI indique que tiene los tres tiempos */
/* Debe ser volatile para que el compilador no optimize las comprobaciones */
volatile byte Calcular;

word cuenta_irqs;

int main ( )
{
    word PorAnt = 0; /* Porcentaje anterior */

    /* Deshabilitamos interrupciones */
    lock ( );

    /*Encendemos led*/
    _io_ports[ M6812_DDRG ] |= M6812B_PG7;
    _io_ports[ M6812_PORTG ] |= M6812B_PG7;


    serial_init( );
    serial_print( "\n$Id: PorcentajeAlto.c $\n" );

    /*Inizializamos las Variables globales*/
    Bajada0 = Bajada1 = Subida = 0;
    Calcular = 0;

    cuenta_irqs = 0;

    /*Inicializamos periféricos*/
    /*Inicialización del Temporizador*/
    _io_ports[ M6812_TMSK2 ] = TCM_FACTOR;

    /*configuramos canal 1 como capturador de entrada*/
    _io_ports[ M6812_TIOS ] &= ~M6812B_IOS1;

    /*Detectamos cualquier flaco*/
    _io_ports[ M6812_TCTL4 ] = M6812B_EDG1B | M6812B_EDG1A;

    _io_ports[ M6812_TFLG1 ] = M6812B_IOS1; /*Bajamos el banderín de OC1 */
    _io_ports[ M6812_TMSK1 ] |= M6812B_IOS1; /*habilitamos sus interrupciones*/
    _io_ports[ M6812_TSCR ] = M6812B_TEN; /*Habilitamos temporizador*/

    unlock( ); /* habilitamos interrupciones */

    serial_print( "\nTerminada inicialización\n" );

    while( 1 ) {
        word Porcentaje, Periodo, Altos;

        /*Esperamos a que calcular pase a 1*/
        while( ! Calcular ) {
            //__asm__ __volatile__ ( "wai" );
        };

        /*Hacemos cáculo*/
        Periodo = Bajada1 - Bajada0;
        Altos = ( Subida - Bajada0 );
        Porcentaje = ( long )Altos * 100/( long )Periodo;
        /*Solo enviamos si ha habido cambios*/
        if( Porcentaje != PorAnt ) {
            PorAnt = Porcentaje;
            serial_print( "\rPorcentaje = " );
            serial_printdecword( Porcentaje );
            serial_print( "\t" );
            serial_printdecword( Periodo );
            serial_print( "\t" );
            serial_printdecword( Altos );
            serial_print( "\t" );
            serial_printdecword( cuenta_irqs );
        }
        Bajada0 = 0;
        Calcular = 0; /*indica hemos terminado*/
    }
}


/* Manejador interrupciones del IC1  */
void __attribute__( ( interrupt ) ) vi_ioc1 ( void )
{
    _io_ports[ M6812_TFLG1 ] = M6812B_IOS1; /*Bajamos el banderín de IC1 */
    cuenta_irqs++;

    if( Calcular ) return; /* El programa principal está calculando, no tocamos nada */

    /*Vemos si fue bajada o subida */
    if( _io_ports[ M6812_PORTT ] & M6812B_PT1 ) {
        /*Subida, sólo almacenamos*/
        Subida = _IO_PORTS_W( M6812_TC1 );
        _io_ports[ M6812_PORTG ] |= M6812B_PG7; /*Encendemos led*/
    } else if ( ! Bajada0 )
        /* Bajada0 es == 0, significa que aún no la tenemos */
        Bajada0 = _IO_PORTS_W( M6812_TC1 );
    else {
        Bajada1 = _IO_PORTS_W( M6812_TC1 );
        Calcular = 1;
        _io_ports[ M6812_PORTG ] &= ~M6812B_PG7; /*Apagamos led*/
    }

}

