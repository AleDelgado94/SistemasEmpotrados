/* Memory banks for Axiom Manufacturing CME11E9-EVBU
   (http://www.axman.com)

  */
MEMORY
{
  page0 (rwx) : ORIGIN = 0x0, LENGTH = 256
  text  (rx)  : ORIGIN = 0x2000, LENGTH = 0x6000
  data        : ORIGIN = 0x1040, LENGTH = 0x1000-0x040
}
/* Setup the stack on the top of the data memory bank.  */
PROVIDE (_stack = 0x2000 - 1);
