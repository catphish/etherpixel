// Wait for 400 nanoseconds
void ns_sleep()
{
 asm volatile("mov r1, #48; ds: subs r1, #1; bne ds;" :  :  : "r1" );
}