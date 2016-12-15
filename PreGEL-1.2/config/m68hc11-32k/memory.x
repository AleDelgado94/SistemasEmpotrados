/* Fixed definition of the available memory banks.
   See generic emulation script for a user defined configuration.  */
MEMORY
{
  page0 (rwx) : ORIGIN = 0x0, LENGTH = 256
  /* Board with 32K ROM.  */
  text  (rx)  : ORIGIN = 0x8000, LENGTH = 0x8000

  /* And 32K RAM.  */
  data        : ORIGIN = 0x1040, LENGTH = 0x8000-0x1040
}
/* Setup the stack on the top of the data memory bank.  */
PROVIDE (_stack = 0x8000 - 1);
