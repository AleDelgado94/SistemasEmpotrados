/* Fixed definition of the available memory banks.
   See generic emulation script for a user defined configuration.  */
MEMORY
{
  page0 (rwx) : ORIGIN = 0x0, LENGTH = 256
  /* Only 8Kb of RAM (well, 32K rom and 32K ram capable but I only plugged
     a 6264...).  */
  text  (rx)  : ORIGIN = 0xE200, LENGTH = 0xFFFF - 0xE200
  data        : ORIGIN = 0xE000, LENGTH = 0xE1FF - 0xE000
}
/* Setup the stack on the top of the data memory bank.  */
PROVIDE (_stack = 0x100 - 1);
