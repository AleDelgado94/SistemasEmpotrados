/********************************
* Fichero de inclusión que define la tabla de vectores.
* Se definen todos los vectores a _start salvo en aquellos para los 
*  que exista una etiqueta vi_nombre, en cuyo caso se usará esa etiqueta.

   Copyright (C) Alberto F. Hamilton Castro
   Dpto. de Ingeniería de Sistemas y Automática 
        y Arquitectura y Tecnología de Comutadores
   Universidad de La Laguna

   $Id: interrupts.h,v 1.2 2008/03/28 13:44:56 alberto Exp $

  Este programa es software libre. Puede redistribuirlo y/o modificarlo bajo 
  los términos de la Licencia Pública General de GNU según es publicada 
  por la Free Software Foundation, bien de la versión 2 de dicha Licencia 
  o bien (según su elección) de cualquier versión posterior. 

  Este programa se distribuye con la esperanza de que sea útil, pero 
  SIN NINGÚN TIPO DE GARANTÍA, incluso sin la garantía MERCANTIL implícita 
  o sin garantizar la CONVENIENCIA PARA UN PROPÓSITO PARTICULAR. 
  Véase la Licencia Pública General de GNU para más detalles. 

******************************************/

/*
            
Todos los vectores de interrupción quedan apuntando 
  a la etiqueta 'vi_default', salvo el vector de reset de 
  encendido que queda apuntando a la etiqueta '_start'.
  A su vez, si no se define, 'vi_default'  apuntará a '_start'.


  Para modificar algún vector de interrupción es necesario
    definir la etiqueta correspondiente, asignándole un valor 
    con un equ o utilizándola como nombre de 
    la rutina de tratamiento de la interrupción. 
    El nombre de las etiquetas es el que aparece en la siguiente lista:

  * vi_osc   Interrupción del sistema oscilador
  * vi_cantx   Transmisión en el subsistema CAN
  * vi_canrx   Recepción en el subsistema CAN
  * vi_canerr   Errores en el subsistema CAN
  * vi_pabov   Desbordamiento del acumulador de pulsos B
  * vi_cmuv   Cuenta final del contador de módulo
  * vi_kwgh   Activación de los puestos G ó H
  * vi_canwu   Activación del subsistema CAN
  * vi_atd   Conversores analógicos
  * vi_sci1   Interrupción del sistema SCI 1
  * vi_sci0   Interrupción del sistema SCI 0
  * vi_spi   Interrupción del sistema SPI
  * vi_pai   Flanco del acumulador de pulsos
  * vi_paov   Desbordamiento del acumulador de pulsosA
  * vi_tov   Desbordamiento del temporizador
  * vi_ioc7   Canal 7 del temporizador
  * vi_ioc6   Canal 6 del temporizador
  * vi_ioc5   Canal 5 del temporizador
  * vi_ioc4   Canal 4 del temporizador
  * vi_ioc3   Canal 3 del temporizador
  * vi_ioc2   Canal 2 del temporizador
  * vi_ioc1   Canal 1 del temporizador
  * vi_ioc0   Canal 0 del temporizador
  * vi_rti   Interrupción de tiempo real
  * vi_irq   Interrupción línea IRQ
  * vi_xirq   Interrupción línea XIRQ 
  * vi_swi   Instrucción SWI
  * vi_trap   Instrucción Ilegal
  * vi_copreset   Reset por temporizador COP
  * vi_clkreset   Reset por fallo del reloj
  * vi_poreset   Reset externo o reset de encendido

 Estas etiquetas han de estar definidas cuando se carga este fichero,
  por ello, debe incluirse el final del código fuente de cada programa.

*/
             

void __attribute__((interrupt)) vi_default(void){/* do nothing */}

extern void  __attribute__((interrupt))    vi_osc(void);
extern void  __attribute__((interrupt))    vi_cantx(void);
extern void  __attribute__((interrupt))    vi_canrx(void);
extern void  __attribute__((interrupt))    vi_canerr(void);
extern void  __attribute__((interrupt))    vi_pabov(void);
extern void  __attribute__((interrupt))    vi_cmuv(void);
extern void  __attribute__((interrupt))    vi_kwgh(void);
extern void  __attribute__((interrupt))    vi_canwu(void);
extern void  __attribute__((interrupt))    vi_atd(void);
extern void  __attribute__((interrupt))    vi_sci1(void);
extern void  __attribute__((interrupt))    vi_sci0(void);
extern void  __attribute__((interrupt))    vi_spi(void);
extern void  __attribute__((interrupt))    vi_pai(void);
extern void  __attribute__((interrupt))    vi_paov(void);
extern void  __attribute__((interrupt))    vi_tov(void);
extern void  __attribute__((interrupt))    vi_ioc7(void);
extern void  __attribute__((interrupt))    vi_ioc6(void);
extern void  __attribute__((interrupt))    vi_ioc5(void);
extern void  __attribute__((interrupt))    vi_ioc4(void);
extern void  __attribute__((interrupt))    vi_ioc3(void);
extern void  __attribute__((interrupt))    vi_ioc2(void);
extern void  __attribute__((interrupt))    vi_ioc1(void);
extern void  __attribute__((interrupt))    vi_ioc0(void);
extern void  __attribute__((interrupt))    vi_rti(void);
extern void  __attribute__((interrupt))    vi_irq(void);
extern void  __attribute__((interrupt))    vi_xirq(void);
extern void  __attribute__((interrupt))    vi_swi(void);
extern void  __attribute__((interrupt))    vi_trap(void);
extern void  __attribute__((interrupt))    vi_copreset(void);
extern void  __attribute__((interrupt))    vi_clkreset(void);


extern void _start(void);/* entry point in crt0.s */

void __attribute__ (( section (".vectors") )) (* const interrupt_vectors[])(void) = {
   vi_osc,  //Interrupción del sistema oscilador
   vi_cantx,  //Transmisión en el subsistema CAN
   vi_canrx,  //Recepción en el subsistema CAN
   vi_canerr,  //Errores en el subsistema CAN
   vi_pabov,  //Desbordamiento del acumulador de pulsos B
   vi_cmuv,  //Cuenta final del contador de módulo
   vi_kwgh,  //Activación de los puestos G ó H
   vi_canwu,  //Activación del subsistema CAN
   vi_atd,  //Conversores analógicos
   vi_sci1, //  Interrupción del sistema SCI 1
   vi_sci0, //  Interrupción del sistema SCI 0
   vi_spi,  //Interrupción del sistema SPI
   vi_pai,  //Flanco del acumulador de pulsos
   vi_paov,  //Desbordamiento del acumulador de pulsosA
   vi_tov,  //Desbordamiento del temporizador
   vi_ioc7, //  Canal 7 del temporizador
   vi_ioc6, //  Canal 6 del temporizador
   vi_ioc5, //  Canal 5 del temporizador
   vi_ioc4, //  Canal 4 del temporizador
   vi_ioc3, //  Canal 3 del temporizador
   vi_ioc2, //  Canal 2 del temporizador
   vi_ioc1, //  Canal 1 del temporizador
   vi_ioc0, //  Canal 0 del temporizador
   vi_rti,  //Interrupción de tiempo real
   vi_irq,  //Interrupción línea IRQ
   vi_xirq,  //Interrupción línea XIRQ 
   vi_swi,  //Instrucción SWI
   vi_trap,  //Instrucción Ilegal
   vi_copreset,  //Reset por temporizador COP
   vi_clkreset,  //Reset por fallo del reloj
   _start,  //Reset externo o reset de encendido
};

/* fin tabla vectores*/
