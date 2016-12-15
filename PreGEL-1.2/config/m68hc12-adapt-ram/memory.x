/* 
 Distribucion de memoria para Adpat912DT60 
 con programa en memoria RAM

  */
/* Definimos etiqueta para la pila en lo alto de la memoria RAM no usada.  */
vectors_addr = 0x07c2; 
_stack = _vectors_addr - 1;
_io_ports = 0x0800;

/*No parece posible usar variables en las siguientes expresiones,
pero se pueden poner las expersiones con constantes
*/

MEMORY
{
  page0 (rwx) : ORIGIN = 0x0, LENGTH = 256    
  data        : ORIGIN = 0x0000, LENGTH = 40
  text  (rx)  : ORIGIN = 20, LENGTH = 	0x07c2 - 10   - 40
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
