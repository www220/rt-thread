#include <rtthread.h>
#include <sys/types.h>
#include <time.h>
#include <Windows.h>

#ifdef RT_USING_DFS
#include <dfs_posix.h>

#ifdef RT_USING_DFS_DEVFS
#include <devfs.h>
#endif

#endif

void libc_system_init(void)
{
#ifdef RT_USING_DFS
    int fd;
    struct rt_device *console_dev;

#ifndef RT_USING_DFS_DEVFS
#error Please enable devfs by defining RT_USING_DFS_DEVFS in rtconfig.h
#endif

    console_dev = rt_console_get_device();
    if (console_dev)
    {
        /* initialize console device */
        rt_console_init(console_dev->parent.name);

        /* open console as stdin/stdout/stderr */
        fd = open("/dev/console", O_RDONLY, 0); /* for stdin */
        fd = open("/dev/console", O_WRONLY, 0); /* for stdout */
        fd = open("/dev/console", O_WRONLY, 0); /* for stderr */

        /* skip warning */
        fd = fd;
    }
#endif
}

errno_t _sopen_s (
    int * pfh,
    const char *path,
    int oflag,
    int shflag,
    int pmode
    )
{
#ifndef RT_USING_DFS
    return -1;
#else
    *pfh = open(path,oflag);
    return (*pfh < 0)?(-1):(0);
#endif
}

long _lseek (
    int fh,
    long pos,
    int mthd
    )
{
#ifndef RT_USING_DFS
    return -1;
#else
    return lseek(fh, pos, mthd);
#endif
}

int _read (int fh, void *buf, unsigned cnt)
{
#ifndef RT_USING_DFS
    return 0;
#else
    return read(fh, buf, cnt);
#endif
}

int _write (int fh, const void *buf, unsigned cnt)
{
    if (fh < 3)
    {
#ifdef RT_USING_CONSOLE
        rt_device_t console_device;
        console_device = rt_console_get_device();
        if (console_device != 0) rt_device_write(console_device, 0, buf, cnt);
        return cnt;
#else
        return 0;
#endif
    }
    else
    {
#ifdef RT_USING_DFS
        return write(fh, buf, cnt);;
#else
        return 0;
#endif
    }
}

int _close (int fh)
{
#ifndef RT_USING_DFS
    return 0;
#else
    return close(fh);
#endif
}

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
    struct tm atm;
    SYSTEMTIME st;
    struct timeval tps;

    GetLocalTime(&st);
    atm.tm_sec = st.wSecond;
    atm.tm_min = st.wMinute;
    atm.tm_hour = st.wHour;
    atm.tm_mday = st.wDay;
    atm.tm_mon = st.wMonth - 1;        // tm_mon is 0 based
    atm.tm_year = st.wYear - 1900;     // tm_year is 1900 based
    tps.tv_sec = mktime(&atm);
    tps.tv_usec = st.wMilliseconds*1000;
    if (tp != NULL)
        *tp = tps;

    return tps.tv_sec;
}