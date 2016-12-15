/* 
 Distribucion de memoria para Adpat912DT60 
 con programa en memoria RAM externa utilizando 
el CargaRam 2.7 que permite activar ram externa pero
sólo quita bloque bajo de la flash => hay memoria RAM de
 0x1000 a 0x7fff

Los seudovectores están en 0x07c2

Los registros de control están en la dirección 0x0800

Dejamos pila bajo los vectores.

*/

vectors_addr = 0x07c2;
_stack = 0x07c2;
_io_ports = 0x0800;

/*No parece posible usar variables en las siguientes expresiones,
pero se pueden poner las expersiones con constantes
*/

MEMORY
{
  page0 (rwx) : ORIGIN = 0x0, LENGTH = 256    
  data        : ORIGIN = 0x1000, LENGTH = 0x3000
  text  (rx)  : ORIGIN = 0x4000, LENGTH = 0x7fff - 0x4000
  eeprom      : ORIGIN = 0x0C00, LENGTH = 1024
}

PROVIDE(vi_osc = vi_default);
PROVIDE(vi_cantx = vi_default);
PROVIDE(vi_canrx = vi_default);
PROVIDE(vi_canerr = vi_default);
PROVIDE(vi_pabov = vi_default);
PROVIDE(vi_cmuv = vi_default);
PROVIDE(vi_kwgh = vi_default);
PROVIDE(vi_canwu = vi_default);
PROVIDE(vi_atd = vi_default);
PROVIDE(vi_sci1 = vi_default);
PROVIDE(vi_sci0 = vi_default);
PROVIDE(vi_spi = vi_default);
PROVIDE(vi_pai = vi_default);
PROVIDE(vi_paov = vi_default);
PROVIDE(vi_tov = vi_default);
PROVIDE(vi_ioc7 = vi_default);
PROVIDE(vi_ioc6 = vi_default);
PROVIDE(vi_ioc5 = vi_default);
PROVIDE(vi_ioc4 = vi_default);
PROVIDE(vi_ioc3 = vi_default);
PROVIDE(vi_ioc2 = vi_default);
PROVIDE(vi_ioc1 = vi_default);
PROVIDE(vi_ioc0 = vi_default);
PROVIDE(vi_rti = vi_default);
PROVIDE(vi_irq = vi_default);
PROVIDE(vi_xirq = vi_default);
PROVIDE(vi_swi = vi_default);
PROVIDE(vi_trap = vi_default);
PROVIDE(vi_copreset = vi_default);
PROVIDE(vi_clkreset = vi_default);
