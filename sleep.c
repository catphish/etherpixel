// Wait for approx 350 nanoseconds
void ns_sleep()
{
 asm volatile("mov r1, #20; ds: subs r1, #1; bne ds;" :  :  : "r1" );
}
