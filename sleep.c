// Wait for approx 350 nanoseconds
void ns_sleep_a()
{
  asm volatile("mov r1, #0xC8; dsa: subs r1, #1; bne dsa;" :  :  : "r1" );
}
void ns_sleep_b()
{
  asm volatile("mov r1, #0x160; dsb: subs r1, #1; bne dsb;" :  :  : "r1" );
}
void ns_sleep_c()
{
  asm volatile("mov r1, #0x1C0; dsc: subs r1, #1; bne dsc;" :  :  : "r1" );
}
