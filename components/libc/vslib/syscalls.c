#include <rtthread.h>
#include <rtdevice.h>
#include <sys/types.h>

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