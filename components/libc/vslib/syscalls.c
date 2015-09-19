
int*
__errno(void)
{
	static volatile int gun_errno;
	return (int *)&gun_errno;
}
