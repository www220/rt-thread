
int*
__errno(void)
{
	static volatile int gun_errno;
	return (int *)&gun_errno;
}

int
link(const char *__path1, const char *__path2)
{
    return -1;
}

int 
mkstemp(char *path)
{
    return -1;
}

int	
gettimeofday(struct timeval *tp, struct timezone *tzp)
{
    return -1;
}