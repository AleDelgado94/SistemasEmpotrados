/*****************************************

  Esqueleto del programa del gestión del display
  LCD. Necesario completar a partir de comentarios
  que contienen ============

  El puerto de datos es el H.
  El puerto de control es el T: E=7, RW=6, RS=5
  Estos valores se pueden cabiar en los define iniciales

  AUTORES
    Alejandro Delgado Martel
    Alberto Martinez Chincho



  PUERTOS
    H (display datos)
      PINES:  42-35

    T (display control)
      PINES:
        RS  8
        RW  7
        EN  6

    G (teclado 7 pines)
      PINES: 14-21 (14-20)

    P (botones)
      PINES:
        P1  47
        P2  46
        P3  45
        P4  44




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


#define DESPLAZA_IZDA   ((1 << 4))
#define DESPLAZA_DCHA   ((1 << 4) | (1 << 3))


/*Puerto de datos del teclado*/
#define P_TECLADO   M6812_PORTG
#define P_TECLADO_DDR   M6812_DDRG

/*Puerto de datos de los pulsadores*/
#define P_PULSADOR  M6812_PORTP
#define P_PULSADOR_DDR  M6812_DDRP

#define P1 ( 1 << 7 )
#define P2 ( 1 << 6 )
#define P3 ( 1 << 5 )
#define P4 ( 1 << 4 )

//teclado
#define C1 ( 1 << 4 )
#define C2 ( 1 << 6 )
#define C3 ( 1 << 2 )

#define F1 ( 1 << 5 )
#define F2 ( 1 )
#define F3 ( 1 << 1 )
#define F4 ( 1 << 3 )


//Sonido

#define  TCM_FACTOR ( 3 )  /*La potencia de 2 a aplicar al factor*/
#define  TCM_FREQ ( M6812_CPU_E_CLOCK / ( 1 << TCM_FACTOR ) )
/*Pasa de microsegundos a ticks*/
#define  USG_2_TICKS(us)  ( ( us ) * ( TCM_FREQ / 1000000L ) )
/*Pasa de milisegundos a ticks*/
#define  MSG_2_TICKS(ms)  ( ( ms ) * ( TCM_FREQ / 1000L ) )


unsigned short Periodo; /* Ticks del temporizador que dura el periodo */
unsigned short cuenta_irqs; /* Se incremente en cada interrupción */


/*
   función que realiza un retarso del el número de microsegundos indicadosDESPLAZA_IZDA
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

/*****************
*****DISPLAY******
*****************/

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
    _io_ports[P_DATOS_DDR] = 0xff;
    _io_ports[P_CONT_DDR] = _io_ports[P_CONT_DDR] | (B_EN | B_RW | B_RS);

    delayusg( 15000UL );
    enviaComando( DL_8BITS );

    /*======== Resto de la inicialización ============= */
    delayusg(5000UL); //AÑADIR TIEMPO EQUIVALENTE A MAS DE 4.1 ms
    enviaComando(DL_8BITS);
    delayusg(100UL); //AÑADIR TIEMPO EQUIVALENTE A MAS DE 100 us
    enviaComando(DL_8BITS);
    enviaComando(DL_8BITS | DOS_FILAS | FUENTE_5X10);
    enviaComando(DISP_OFF);
    enviaComando(CLEAR);
    enviaComando(CUR_INC);

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
    enviaDato(c);
}


/*

2 Borrar display
8 Home
11  Cursor izda
12  Cursor dcha
15 Apagado display
3 Ocultación cursor
16 Parpadeo cursor

*/

void controlDisplay(int c){
  switch (c) {
    case 2: enviaComando(CLEAR); break;
    case 8: enviaComando(RETURN); break;
    case 11: enviaComando(DESPLAZA_IZDA); break;
    case 12: enviaComando(DESPLAZA_DCHA); break;
    case 15: enviaComando(DISP_OFF); break;
    case 3: enviaComando(CUR_BLIK); break;
    case 16: enviaComando(CUR_ON); break;
    default: break;
  }
}


void sacaDisplayCadena(char* c){
    int i;
    for(i=0; i!='\0'; i++)
      enviaDato(c[i]);
}


/*****************
******SONIDO******
*****************/

void inicializaSonido(){

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

}


void emiteSonido(){
  /* OC2 Invierte el pin en cada disparo */
  _io_ports[ M6812_TCTL2 ] &= ~M6812B_OM2;
  _io_ports[ M6812_TCTL2 ] |= M6812B_OL2;
  delayusg(100000);
  //Apagamos los PINES
  /* OC2 Invierte el pin en cada disparo */
  _io_ports[ M6812_TCTL2 ] &= ~M6812B_OM2;
  _io_ports[ M6812_TCTL2 ] &= ~M6812B_OL2;
}



/*****************
*****TECLADO******
*****************/

void inicializaTeclado(void){
  //PONER TODOS LOS PINES DE LA FILAS A 0 PERO
  //COMO SE TRATA DE PULLDOWN SE
  //TIENEN QUE PONER A 1
  _io_ports[P_TECLADO] = ( F1 | F2 | F3 | F4 );
  _io_ports[P_TECLADO_DDR] = ( F1 | F2 | F3 | F4 );
  _io_ports[P_PULSADOR_DDR] = (P1 | P2 | P3 | P4);



}



byte leerTeclado(void){



  byte caracter;

  _io_ports[P_TECLADO] = ( F1 | F2 | F3 | F4 );

  while(((_io_ports[P_TECLADO] & C1) || (_io_ports[P_TECLADO] & C2) || (_io_ports[P_TECLADO] & C3) || (_io_ports[P_PULSADOR] & P3) || (_io_ports[P_PULSADOR] & P4)));
  delayusg(20000UL); //ESPERAMOS UNOS 20 ms

  _io_ports[P_TECLADO] = ( F1 | F2 | F3 | F4 );
  //ESPERAR NUEVA PULSACIÓN
  while(!((_io_ports[P_TECLADO] & C1) || (_io_ports[P_TECLADO] & C2) || (_io_ports[P_TECLADO] & C3) || (_io_ports[P_PULSADOR] & P3) || (_io_ports[P_PULSADOR] & P4)));
  delayusg(20000UL); //ESPERAMOS UNOS 20 ms


  //NUMERO 0
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F1 | F2 | F3);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F4;
  if(_io_ports[P_TECLADO] & C2)
  caracter = '0';

  //NUMERO 1
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F2 | F3 | F4);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F1;
  if(_io_ports[P_TECLADO] & C1)
  caracter = '1';

  //NUMERO 2
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F2 | F3 | F4);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F1;
  if(_io_ports[P_TECLADO] & C2)
  caracter = '2';

  //NUMERO 3
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F2 | F3 | F4);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F1;
  if(_io_ports[P_TECLADO] & C3)
  caracter = '3';

  //NUMERO 4
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F1 | F3 | F4);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F2;
  if(_io_ports[P_TECLADO] & C1)
  caracter = '4';
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F1 | F3 | F4);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F2;
  if(_io_ports[P_TECLADO] & C2)
  caracter = '5';

  //NUMERO 6
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F1 | F3 | F4);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F2;
  if(_io_ports[P_TECLADO] & C3)
  caracter = '6';

  //NUMERO 7
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F1 | F2 | F4);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F3;
  if(_io_ports[P_TECLADO] & C1)
  caracter = '7';

  //NUMERO 8
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F1 | F2 | F4);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F3;
  if(_io_ports[P_TECLADO] & C2)
  caracter = '8';

  //NUMERO 9
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F1 | F2 | F4);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F3;
  if(_io_ports[P_TECLADO] & C3)
  caracter = '9';


  //CLEAR (#)
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F1 | F2 | F3);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F4;
  if(_io_ports[P_TECLADO] & C3)
  controlDisplay(2);

  //IGUAL (*)
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] & ~(F1 | F2 | F3);
  _io_ports[P_TECLADO] = _io_ports[P_TECLADO] | F4;
  if(_io_ports[P_TECLADO] & C1)
  caracter = '=';

  //SUMA P1
  if(_io_ports[P_PULSADOR] & ~P1)
  caracter = '+';

  //RESTA P2
  if(_io_ports[P_PULSADOR] & ~P2)
  caracter = '-';

  //MULTIPLICACIÓN P3
  if(_io_ports[P_PULSADOR] & P3)
  caracter = '*';

  //DIVISION P4
  if(_io_ports[P_PULSADOR] & P4)
  caracter = '/';


emiteSonido();


return caracter;

}

//CONVERTIR DE CHAR* A INT
int atoi(char * cadena){
  int sign = 1;

  if(*cadena == '-')
    sign = -1;

  int num=0;
  while(*cadena){
    num = ((*cadena) - '0') + num * 10;
    cadena++;
  }

  return num * sign;
}




void my_reverse(char str[], int len)
{
    int start, end;
    char temp;
    for(start=0, end=len-1; start < end; start++, end--) {
        temp = *(str+start);
        *(str+start) = *(str+end);
        *(str+end) = temp;
    }
}

char* my_itoa(int num, char* str, int base)
{
    int i = 0;
    int isNegative = 0;

    /* A zero is same "0" string in all base */
    if (num == 0) {
        str[i] = '0';
        str[i + 1] = '\0';
        return str;
    }

    /* negative numbers are only handled if base is 10
       otherwise considered unsigned number */
    if (num < 0 && base == 10) {
        isNegative = 1;
        num = -num;
    }

    while (num != 0) {
        int rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'A' : rem + '0';
        num = num/base;
    }

    /* Append negative sign for negative numbers */
    if (isNegative == 1){
        str[i++] = '-';
    }

    str[i] = '\0';

    my_reverse(str, i);

    return str;
}






//FUNCION CALCULADORA (SUMA, RESTA, MULTIPLICACIÓN, DIVISIÓN)
int calculadora(char* numero_operando, int acumulador){

  char op;
  int i=0;
  int j;
  int ac= acumulador;

//MIRAMOS QUE OPERACION HAY QUE REALIZAR
  while(numero_operando[i]!= '\0'){
    i++;
  }
  op = (char)numero_operando[i-1];

  char c[i-1];
  for(j=0; j<(i-1); j++)
    c[j]=numero_operando[j];

  c[j] = '\0';

  int numero = atoi(c);

  switch (op) {
    case '+': ac += numero;  break;
    case '-': ac -= numero; break;
    case '*': ac *= numero; break;
    case '/': ac /= numero; break;
    default: ac = numero; break;
  }


  return ac;
}

//FUNCION PARA INTRODUCIR EL NUMERO
//DESDE TECLADO
char* introduceNumero(){
    char* numero;
    int i=0;
    while( (numero[i]!='\0')   && (numero[i] != '+' || numero[i] != '-' || numero[i] != '*' || numero[i] != '/' || numero[i] != '=' )){
      numero[i] = leerTeclado();
      sacaDisplay(numero[i]);
      i++;
    }
    return numero;
}


int main ( ) {

    /* Deshabilitamos interrupciones */
    lock ( );

    /* Inicializamos la serial */
    serial_init( );
    serial_print( "\n$Id: Calculator.c $\n" );

    char* numero_introducido;
    int acumulador = 0;
    char ac[128];
    /*Encendemos led*/
    _io_ports[ M6812_DDRG ] |= M6812B_PG7;
    _io_ports[ M6812_PORTG ] |= M6812B_PG7;

    inicializaDisplay( );
    inicializaTeclado( );
    inicializaSonido( );

    unlock( ); /* habilitamos interrupciones */
    serial_print( "\n\rTerminada inicialización\n" );

    while( 1 ) {
        //char c;
        //c = serial_recv( );
        //serial_send( c ); /* Hacemos eco para confirmar la recepción */

        //sacaDisplay( c );

        //sacaDisplay( "Calc" );
        //enviaComando( CLEAR );
        //enviaComando( DISP_ON | CUR_ON );
        //acumulador = atoi(introduceNumero(numero_introducido));
        acumulador = calculadora(introduceNumero(), acumulador);
        //sacaDisplayCadena(dec_convert(ac, acumulador));
        sacaDisplayCadena(my_itoa(acumulador, ac, 10));

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
