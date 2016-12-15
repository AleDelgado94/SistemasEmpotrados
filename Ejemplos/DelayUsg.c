
/*****************************************

  Ejemplo de función para generar un retraso
  con precisón de microsegundos.
  Si retraso es mayor que alcance del temporizador
  se cuentan más de un disparo.
  Utiliza el canal 6 del temporizador.

   Copyright (C) Alberto F. Hamilton Castro
   Dpto. de Ingeniería de Sistemas y Automática
        y Arquitectura y Tecnología de Comutadores
   Universidad de La Laguna

   $Id: DelayUsg.c $

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
#define _IO_PORTS_W(d)  (((unsigned volatile short*) & _io_ports[(d)])[0])

/*
   función que realiza un retarso del el número de microsegundos indicados
   en el parámetro usg
   Utiliza canal 6 del temporizador
*/
void delayusg( unsigned long useg ) {
    unsigned int numCiclos;
    unsigned long numCiclosL;

    /* Desconectamos para no afectar al pin */
    _io_ports[ M6812_TCTL1 ] &= ~(M6812B_OM6 | M6812B_OL6);

    /* Vemos velocidad del temporizador*/
    byte factorT = _io_ports[ M6812_TMSK2 ] & 0x07; /*Factor de escalado actual*/
    unsigned long frec = M6812_CPU_E_CLOCK/( 1 << factorT ); /* Frecuencia del temporizador*/
    /* Según la frecuencia elegimos el modo de dividir para evitar desbordamientos */
    if( frec/1000000 )
        numCiclosL = frec/1000000 * useg;
    else
        numCiclosL = frec/100 * useg/10000;

    unsigned int numDisparos = numCiclosL >> 16;  /* Numero de disparos necesarios */
    numCiclos = numCiclosL & 0xffff; /* Número restante de ciclos */

    /* Por si escalado muy grande y useg pequeño */
    if( ( numCiclos == 0 ) && ( numDisparos == 0 ) ) numCiclos = 1;

    _io_ports[ M6812_TIOS ] |= M6812B_IOS6; /*configuramos canal como comparador salida*/
    _io_ports[ M6812_TFLG1 ] = M6812B_C6F; /*Bajamos el banderín  */
    /*preparamos disparo*/
    _IO_PORTS_W( M6812_TC6 ) = _IO_PORTS_W( M6812_TCNT ) + numCiclos ;

    /*Habilitamos el temporizador, por si no lo está*/
    _io_ports[ M6812_TSCR ] |= M6812B_TEN;

    /* Esparamos los desboradmientos necesarios */
    do {
        /* Nos quedamos esperando a que se produzca la igualdad*/
        while ( ! ( _io_ports[ M6812_TFLG1 ] & M6812B_C6F ) );
        _io_ports[ M6812_TFLG1 ] = M6812B_C6F; /* Bajamos el banderín */
    } while( numDisparos-- );
}

int main ( )
{
    unsigned int i;

    /* Deshabilitamos interrupciones */
    lock ( );

    /*Inicialización del Temporizador */
    /* podemos poner cualquier valor de escalado de 0 a 7
       ya que la función se adapta el valor configurado*/
    _io_ports[ M6812_TMSK2 ] = 7;

    /*Encendemos led*/
    _io_ports[ M6812_DDRG ] |= M6812B_PG7;
    _io_ports[ M6812_PORTG ] |= M6812B_PG7;


    serial_init( );
    serial_print( "\n$Id: DelayUsg.c $\n" );

    unlock( ); /* habilitamos interrupciones */
    serial_print( "\n\rTerminada inicialización\n" );

    i = 0;
    while( 1 ) {
        serial_print( "\n\rVamos a esperar un segundo: " );
        serial_printdecword( i++ );
        /*Invertimos el led*/
        _io_ports[ M6812_PORTG ] ^= M6812B_PG7;

        delayusg( 1000UL * 1000UL );
    }
}

