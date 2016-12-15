/* Memory banks for Axiom Manufacturing CMD12 Board
   (http://www.axman.com)

   The memory banks defined here do not take complete advantage
   of the CMD12 memory.

  */
MEMORY
{
  page0 (rwx) : ORIGIN = 0x0, LENGTH = 256

  /* Use CMD12 RAM Space for text and data, 20Kb + 4Kb at 0x2000 */
  text  (rx)  : ORIGIN = 0x3000, LENGTH = 0x5000

  data        : ORIGIN = 0x2000, LENGTH = 0x1000
}
/* Setup the stack on the top of the data memory bank.  */
PROVIDE (_stack = 0x3000);
