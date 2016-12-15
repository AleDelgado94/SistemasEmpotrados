/*****************************************

  Esqueleto del programa del gestión del display
  LCD. Necesario completar a partir de comentarios
  que contienen ============

  El puerto de datos es el H.
  El puerto de control es el T: E=7, RW=6, RS=5
  Estos valores se pueden cabiar en los define iniciales

   Copyright (C) Alberto F. Hamilton Castro
   Dpto. de Ingeniería de Sistemas y Automática
        y Arquitectura y Tecnología de Comutadores
   Universidad de La Laguna

   $Id: EsqueletoDisplay.c $

  Este programa es software libre. Puede redistribuirlo y/o modificarlo bajo
  los términos de la Licencia Pública General de GNU según es publicada
  por la Free Software Foundation, bien de la versión 2 de dicha Licencia
  o bien (según su elección) de cualquier versión posterior.

  Este programa se distribuye con la esperanza de que sea útil, pero
  SIN NINGÚN TIPO DE GARANTÍA, incluso sin la garantía MERCANTIL implícita
  o sin garantizar la CONVENIENCIA PARA UN PROPÓSITO PARTICULAR.
  Véase la Licencia Pública General de GNU para más detalles.


  *************************************** */

#define DEBUG 0

#include <sys/param.h>
#include <sys/interrupts.h>
#include <sys/sio.h>
#include <sys/locks.h>

typedef unsigned char byte;  /*por comodidad*/
typedef unsigned short word;  /*por comodidad*/

/*Acceso a IO PORTS como palabra*/
#define _IO_PORTS_W(d)  (((unsigned volatile short*) & _io_ports[(d)])[0])

#define NOP __asm__ __volatile__ ( "nop" )

/* Puerto de datos del display */
#define P_DATOS   M6812_PORTH
#define P_DATOS_DDR   M6812_DDRH
/* Puerto de control de dislplay */
#define P_CONT    M6812_PORTT
#define P_CONT_DDR    M6812_DDRT

/* Bits de control del display */
#define B_EN      M6812B_PT7
#define B_RW      M6812B_PT6
#define B_RS      M6812B_PT5

/* Bits de control del display */
#define CLEAR     ( 1 )

#define RETURN    ( 1 << 1 )

#define CUR_INC    ( ( 1 << 2 ) | ( 1 << 1 ) )
#define SHIFT    ( ( 1 << 2 ) | 1 )

#define DISP_OFF    ( ( 1 << 3 ) )
#define DISP_ON    ( ( 1 << 3 ) | ( 1 << 2 ) )
#define CUR_ON      ( ( 1 << 3 ) | ( 1 << 1 ) )
#define CUR_BLIK      ( ( 1 << 3 ) | ( 1 ) )

#define SHIFT_DISP  ( ( 1 << 4 ) | ( 1 << 3 ) )
#define SHIFT_LEFT  ( ( 1 << 4 ) | ( 1 << 2 ) )

#define DL_8BITS   ( ( 1 << 5 ) | ( 1 << 4 ) )
#define DOS_FILAS   ( ( 1 << 5 ) | ( 1 << 3 ) )
#define FUENTE_5X10   ( ( 1 << 5 ) | ( 1 << 2 ) )


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

/* función que genera un cilo de la señal E para realizar un acceso al display
   los valores de las señales RW, RS y datos deben de fijarse antes de llamar a esta
   función */
void cicloAcceso( ) {
    NOP;
    NOP;
    NOP;
    _io_ports[ P_CONT ] |= B_EN; /* subimos señal E */
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    _io_ports[ P_CONT ] &= ~B_EN; /* bajamos señal E */
    NOP;
    NOP;
    NOP;
}

/* Envía un byte como comando */
void enviaComando( byte b ) {
    /* Ciclo de escritura con RS = 0 */
    _io_ports[ P_CONT ] &= ~( B_RS | B_RW );
    _io_ports[ P_DATOS ] = b;

    cicloAcceso( );

    /* Esperamos el tiempo que corresponda */
    if( ( b == 1 ) || ( ( b & 0xfe ) == 2 ) )
        delayusg( 1520UL );
    else
        delayusg( 37 );
}

/* Envía un byte como dato */
void enviaDato( byte b ) {
    /* Ciclo de escritura con RS = 1 */
    _io_ports[ P_CONT ] &= ~B_RW;
    _io_ports[ P_CONT ] |= B_RS;
    _io_ports[ P_DATOS ] = b;

    cicloAcceso( );

    /* Esperamos el tiempo que corresponde */
    delayusg( 37 );
}

void inicializaDisplay( ) {

    /*======== Configurar puertos del display como salida ============= */

    delayusg( 15000UL );
    enviaComando( DL_8BITS );

    /*======== Resto de la inicialización ============= */


    /* Encendemos display con cursor parpadeante */
    enviaComando( DISP_ON | CUR_ON );

    /*Sacamos mensaje */
    enviaDato( 'H' );
    enviaDato( 'o' );
    enviaDato( 'l' );
    enviaDato( 'a' );


}

void sacaDisplay( byte c ) {

    /*========= Implementar el código de la función de gestión =============== */

}



int main ( ) {

    /* Deshabilitamos interrupciones */
    lock ( );

    /* Inicializamos la serial */
    serial_init( );
    serial_print( "\n$Id: EsqueletoDisplay.c $\n" );


    /*Encendemos led*/
    _io_ports[ M6812_DDRG ] |= M6812B_PG7;
    _io_ports[ M6812_PORTG ] |= M6812B_PG7;

    inicializaDisplay( );

    unlock( ); /* habilitamos interrupciones */
    serial_print( "\n\rTerminada inicialización\n" );

    while( 1 ) {
        char c;
        c = serial_recv( );
        serial_send( c ); /* Hacemos eco para confirmar la recepción */

        sacaDisplay( c );

        /*Invertimos el led*/
        _io_ports[ M6812_PORTG ] ^= M6812B_PG7;

    }
}
