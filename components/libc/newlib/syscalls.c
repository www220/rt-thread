#include <reent.h>
#include <sys/errno.h>
#include <sys/time.h>
#include <rtthread.h>

#ifdef RT_USING_DFS
#include <dfs_posix.h>
#endif

#ifdef RT_USING_PTHREADS 
#include <pthread.h>
#endif

/* Reentrant versions of system calls.  */

int
_close_r(struct _reent *ptr, int fd)
{
#ifndef RT_USING_DFS
	return 0;
#else
	return close(fd);
#endif
}

int
_execve_r(struct _reent *ptr, const char * name, char *const *argv, char *const *env)
{
	/* return "not supported" */
	ptr->_errno = ENOTSUP;
	return -1;
}

int
_fcntl_r(struct _reent *ptr, int fd, int cmd, int arg)
{
	/* return "not supported" */
	ptr->_errno = ENOTSUP;
	return -1;
}

int
_fork_r(struct _reent *ptr)
{
	/* return "not supported" */
	ptr->_errno = ENOTSUP;
	return -1;
}

int
_fstat_r(struct _reent *ptr, int fd, struct stat *pstat)
{
#ifndef RT_USING_DFS
	return 0;
#else
	int rc;

	rc = fstat(fd, pstat);
	return rc;
#endif
}

int
_getpid_r(struct _reent *ptr)
{
	return 0;
}

int
_isatty_r(struct _reent *ptr, int fd)
{
	if (fd >=0 && fd < 3) return 1;

	return 0;
}

int
_isatty(int fd)
{
	if (fd >=0 && fd < 3) return 1;

	return 0;
}

int
_kill_r(struct _reent *ptr, int pid, int sig)
{
	/* return "not supported" */
	ptr->_errno = ENOTSUP;
	return -1;
}

int
_link_r(struct _reent *ptr, const char *old, const char *new)
{
	/* return "not supported" */
	ptr->_errno = ENOTSUP;
	return -1;
}

_off_t
_lseek_r(struct _reent *ptr, int fd, _off_t pos, int whence)
{
#ifndef RT_USING_DFS
	return 0;
#else
	_off_t rc;

	rc = lseek(fd, pos, whence);
	return rc;
#endif
}

int
_mkdir_r(struct _reent *ptr, const char *name, int mode)
{
#ifndef RT_USING_DFS
	return 0;
#else
	int rc;

	rc = mkdir(name, mode);
	return rc;
#endif
}

int
_open_r(struct _reent *ptr, const char *file, int flags, int mode)
{
#ifndef RT_USING_DFS
	return 0;
#else
	int rc;

	rc = open(file, flags, mode);
	return rc;
#endif
}

_ssize_t 
_read_r(struct _reent *ptr, int fd, void *buf, size_t nbytes)
{
#ifndef RT_USING_DFS
	return 0;
#else
	_ssize_t rc;

	rc = read(fd, buf, nbytes);
	return rc;
#endif
}

int
_rename_r(struct _reent *ptr, const char *old, const char *new)
{
#ifndef RT_USING_DFS
	return 0;
#else
	int rc;

	rc = rename(old, new);
	return rc;
#endif
}

void *
_sbrk_r(struct _reent *ptr, ptrdiff_t incr)
{
	/* no use this routine to get memory */
	return RT_NULL;
}

int
_stat_r(struct _reent *ptr, const char *file, struct stat *pstat)
{
#ifndef RT_USING_DFS
	return 0;
#else
	int rc;

	rc = stat(file, pstat);
	return rc;
#endif
}

_CLOCK_T_
_times_r(struct _reent *ptr, struct tms *ptms)
{
	/* return "not supported" */
	ptr->_errno = ENOTSUP;
	return -1;
}

int
_unlink_r(struct _reent *ptr, const char *file)
{
#ifndef RT_USING_DFS
	return 0;
#else
	int rc;

	rc = unlink(file);
	return rc;
#endif
}

int
_wait_r(struct _reent *ptr, int *status)
{
	/* return "not supported" */
	ptr->_errno = ENOTSUP;
	return -1;
}

_ssize_t
_write_r(struct _reent *ptr, int fd, const void *buf, size_t nbytes)
{
#ifndef RT_USING_DFS
	return 0;
#else
	_ssize_t rc;

	rc = write(fd, buf, nbytes);
	return rc;
#endif
}

int
_gettimeofday_r(struct _reent *ptr, struct timeval *__tp, void *__tzp)
{
#ifndef RT_USING_RTC
	/* return "not supported" */
	ptr->_errno = ENOTSUP;
	return -1;
#else
	static time_t sysnow = 0;
	static uint16_t sysms = 0;
	time_t nownow = time(NULL);
	uint16_t nowms = rt_tick_get() % 1000;
	struct timezone *zone = (struct timezone*)__tzp;
	if (sysnow == 0 && sysms == 0)
	{
		sysnow = nownow;
		sysms = nowms;
	}
	else
	{
		//如果毫秒走的比较快，在秒上面增加
		if ((sysnow == nownow)
				&& (nowms<sysnow))
			sysnow = nownow+1;
		else
			sysnow = nownow;
		sysms = nowms;
	}
	__tp->tv_sec = sysnow;
	__tp->tv_usec = sysms * 1000l;
	if (zone)
	{
		zone->tz_dsttime = 0;
		zone->tz_minuteswest = -480;
	}
	return 0;
#endif
}

int
settimeofday(const struct timeval *__tp, const struct timezone *__tzp)
{
#ifndef RT_USING_RTC
	/* return "not supported" */
	return -1;
#else
	rt_device_t device;
	device = rt_device_find("rtc");
    if (device == RT_NULL)
        return -1;
    /* update to RTC device. */
    rt_device_control(device, RT_DEVICE_CTRL_RTC_SET_TIME, (void *)&__tp->tv_sec);
	return 0;
#endif
}
RTM_EXPORT(settimeofday);

/* Memory routine */
void *
_malloc_r (struct _reent *ptr, size_t size)
{
	void* result;

	result = (void*)rt_malloc (size);
	if (result == RT_NULL)
	{
		ptr->_errno = ENOMEM;
	}

	return result;
}

void *
_realloc_r (struct _reent *ptr, void *old, size_t newlen)
{
	void* result;

	result = (void*)rt_realloc (old, newlen);
	if (result == RT_NULL)
	{
		ptr->_errno = ENOMEM;
	}

	return result;
}

void *_calloc_r (struct _reent *ptr, size_t size, size_t len)
{
	void* result;

	result = (void*)rt_calloc (size, len);
	if (result == RT_NULL)
	{
		ptr->_errno = ENOMEM;
	}

	return result;
}

void 
_free_r (struct _reent *ptr, void *addr)
{
	rt_free (addr);
}

void
_exit (int status)
{
#ifdef RT_USING_MODULE
	rt_module_t module;

	module = rt_module_self();
	if (module != RT_NULL)
	{
		/* unload assertion module */
		rt_module_unload(module);

		/* re-schedule */
		rt_schedule();
	}
#endif
	
	rt_kprintf("thread:%s exit with %d\n", rt_thread_self()->name, status);
	RT_ASSERT(0);

	while (1);
}

void 
_system(const char *s)
{
    /* not support this call */
    return;
}

void __libc_init_array(void)
{
	/* we not use __libc init_aray to initialize C++ objects */
}

void abort(void)
{
    if (rt_thread_self())
    {
        rt_thread_t self = rt_thread_self();

        rt_kprintf("thread:%-8.*s abort!\n", RT_NAME_MAX, self->name);
        rt_thread_suspend(self);

        rt_schedule();
    }

	while (1);
}

#ifdef RT_USING_PROCESS
#include "linux-syscall.h"
extern void *rt_module_conv_ptr(rt_module_t module, rt_uint32_t ptr, rt_uint32_t size);
extern rt_uint32_t rt_module_brk(rt_module_t module, rt_uint32_t addr);
static inline int ret_err(int ret)
{
    if (ret < 0)
    {
        int err = errno;
        if (err == 0)
            err = rt_get_errno();
        return (err>0)?(-err):(err);
    }
    return ret;
}

#define _UTSNAME_LENGTH 65
#define _UTSNAME_DOMAIN_LENGTH _UTSNAME_LENGTH

#ifndef _UTSNAME_SYSNAME_LENGTH
# define _UTSNAME_SYSNAME_LENGTH _UTSNAME_LENGTH
#endif
#ifndef _UTSNAME_NODENAME_LENGTH
# define _UTSNAME_NODENAME_LENGTH _UTSNAME_LENGTH
#endif
#ifndef _UTSNAME_RELEASE_LENGTH
# define _UTSNAME_RELEASE_LENGTH _UTSNAME_LENGTH
#endif
#ifndef _UTSNAME_VERSION_LENGTH
# define _UTSNAME_VERSION_LENGTH _UTSNAME_LENGTH
#endif
#ifndef _UTSNAME_MACHINE_LENGTH
# define _UTSNAME_MACHINE_LENGTH _UTSNAME_LENGTH
#endif

/* Structure describing the system and machine.  */
struct utsname
{
    /* Name of the implementation of the operating system.  */
    char sysname[_UTSNAME_SYSNAME_LENGTH];

    /* Name of this node on the network.  */
    char nodename[_UTSNAME_NODENAME_LENGTH];

    /* Current release level of this implementation.  */
    char release[_UTSNAME_RELEASE_LENGTH];
    /* Current version level of this release.  */
    char version[_UTSNAME_VERSION_LENGTH];

    /* Name of the hardware type the system is running on.  */
    char machine[_UTSNAME_MACHINE_LENGTH];

#if _UTSNAME_DOMAIN_LENGTH - 0
    /* Name of the domain of this node on the network.  */
# ifdef __USE_GNU
    char domainname[_UTSNAME_DOMAIN_LENGTH];
# else
    char __domainname[_UTSNAME_DOMAIN_LENGTH];
# endif
#endif
};

rt_uint32_t sys_call_switch(rt_uint32_t nbr, rt_uint32_t parm1,
        rt_uint32_t parm2, rt_uint32_t parm3,
        rt_uint32_t parm4, rt_uint32_t parm5,
        rt_uint32_t parm6)
{
    rt_module_t module = rt_module_self();

    RT_ASSERT(module != RT_NULL);
    RT_ASSERT((nbr&SYS_BASE) == SYS_BASE);
    //rt_kprintf("syscall %d in\n",nbr-SYS_BASE);

    switch(nbr)
    {
    case SYS_exit:
    {
        _exit(parm1);
        return -ENOTSUP;
    }
    case SYS_brk:
    {
        return rt_module_brk(module,parm1);
    }
    case SYS_chmod:
    case SYS_fchmod:
    {
        return 0;
    }
    case SYS_chown:
    case SYS_fchown:
    case SYS_lchown:
    {
        return -EPERM;
    }
    case SYS_alarm:
    {
        rt_thread_delay(parm1*1000);
        return 0;
    }
    case SYS_nanosleep:
    {
        struct timespec *tim1 = (parm1)?(rt_module_conv_ptr(module,parm1,sizeof(struct timespec))):(0);
        struct timespec *tim2 = (parm2)?(rt_module_conv_ptr(module,parm2,sizeof(struct timespec))):(0);
        if (tim1 == RT_NULL)
            return -EINVAL;
        extern void __udelay(unsigned long usecs);
        int ms = tim1->tv_sec*1000+tim1->tv_nsec/1000000l;
        if (ms <= 0)
            __udelay(tim1->tv_nsec/1000l+1);
        else
            rt_thread_delay(ms);
        if (tim2)
        {
            tim2->tv_sec = 0;
            tim2->tv_nsec = 0;
        }
        return 0;
    }
    case SYS_getuid:
    case SYS_getgid:
    case SYS_geteuid:
    case SYS_getegid:
    {
        return 0;
    }
    case SYS_getgroups:
    {
        if (parm1 == 0 || parm2 == 0)
            return 1;
        gid_t *groups = rt_module_conv_ptr(module,parm2,parm1*sizeof(gid_t));
        groups[0] = 0;
        return 1;
    }
    case SYS_link:
    {
        rt_kprintf("syscall link %s=>%s\n",(const char *)rt_module_conv_ptr(module,parm1,0),
                (const char *)rt_module_conv_ptr(module,parm2,0));
        return -ENOTSUP;
    }
    case SYS_symlink:
    {
        rt_kprintf("syscall symlink %s=>%s\n",(const char *)rt_module_conv_ptr(module,parm1,0),
                (const char *)rt_module_conv_ptr(module,parm2,0));
        return -ENOTSUP;
    }
    case SYS_uname:
    {
        struct utsname* uname = (struct utsname*)rt_module_conv_ptr(module,parm1,sizeof(struct utsname));
        strcpy(uname->sysname,"Linux");
        strcpy(uname->nodename,"root");
        strcpy(uname->release,"2.6.35");
        strcpy(uname->version,"7/12/2014");
        strcpy(uname->machine,"ARM9");
        strcpy(uname->__domainname,"root");
        return 0;
    }
    case SYS_unlink:
    case SYS_rmdir:
    {
        errno = 0;
        return ret_err(unlink((const char *)rt_module_conv_ptr(module,parm1,0)));
    }
    case SYS_rename:
    {
        errno = 0;
        return ret_err(rename((const char *)rt_module_conv_ptr(module,parm1,0),
                (const char *)rt_module_conv_ptr(module,parm2,0)));
    }
    case SYS_mkdir:
    {
        errno = 0;
        return ret_err(mkdir((const char *)rt_module_conv_ptr(module,parm1,0),parm2));
    }
    case SYS_lseek:
    {
        errno = 0;
        return ret_err(lseek(parm1, parm2, parm3));
    }
    case SYS_stat:
    case SYS_lstat:
    {
        errno = 0;
        return ret_err(stat((const char *)rt_module_conv_ptr(module,parm1,0),
                (struct stat *)rt_module_conv_ptr(module,parm2,sizeof(struct stat))));
    }
    case SYS_fstat:
    {
        errno = 0;
        return ret_err(fstat(parm1, (struct stat *)rt_module_conv_ptr(module,parm2,sizeof(struct stat))));
    }
    case SYS_creat:
    {
        errno = 0;
        return ret_err(open((const char*)rt_module_conv_ptr(module,parm1,0), O_TRUNC|O_WRONLY|O_CREAT, parm2));
    }
    case SYS_open:
    {
       errno = 0;
       return ret_err(open((const char*)rt_module_conv_ptr(module,parm1,0), parm2, parm3));
    }
    case SYS_read:
    {
       errno = 0;
       return ret_err(read(parm1, rt_module_conv_ptr(module,parm2,parm3), parm3));
    }
    case SYS_write:
    {
       errno = 0;
       return ret_err(write(parm1, rt_module_conv_ptr(module,parm2,parm3), parm3));
    }
    case SYS_close:
    {
       errno = 0;
       return ret_err(close(parm1));
    }
    case SYS_getdents:
    {
        errno = 0;
        extern int getdents(int file, struct dirent *dirp, rt_size_t nbytes);
        return ret_err(getdents(parm1, (struct dirent*)rt_module_conv_ptr(module,parm2,parm3), parm3));
    }
    case SYS_sendfile:
    {
        char buf[4096];
        int rc,len,readlen = 0;
        off_t *off = (parm3)?(rt_module_conv_ptr(module,parm3,sizeof(off_t))):(0);
        if (off)
        {
            errno = 0;
            rc = lseek(parm2,*off,0);
            if (rc < 0)
                return ret_err(rc);
        }
        while (parm4)
        {
            errno = 0;
            len = (parm4>sizeof(buf))?(sizeof(buf)):(parm4);
            rc = read(parm2,buf,len);
            if (rc < 0)
                return ret_err(rc);
            rc = write(parm1,buf,rc);
            if (rc < 0)
                return ret_err(rc);
            readlen += rc;
            if (rc < len)
                break;
            parm4 -= rc;
        }
        if (off)
            *off += readlen;
        return readlen;
    }
    case SYS_getcwd:
    {
        char *buf = rt_module_conv_ptr(module,parm1,parm2);
        return (rt_uint32_t)getcwd(buf,parm2);
    }
    case SYS_fsync:
    {
        errno = 0;
        return ret_err(parm1);
    }
    case SYS_gettimeofday:
    {
        errno = 0;
        struct timezone *zone = (parm2)?((struct timezone *)rt_module_conv_ptr(module,parm2,sizeof(struct timezone))):RT_NULL;
        return ret_err(_gettimeofday_r(RT_NULL, (struct timeval *)rt_module_conv_ptr(module,parm1,sizeof(struct timeval)),zone));
    }
    case SYS_ioctl:
    {
        errno = 0;
        rt_kprintf("ioctl file:%d cmd:0x%x data:0x%x\n",parm1,parm2,parm3);
        return -ENOTSUP;
    }
    case SYS_BASE+901:
    case SYS_BASE+903:
    {
        rt_mutex_take(module->mod_mutex, RT_WAITING_FOREVER);
        return 0;
    }
    case SYS_BASE+902:
    case SYS_BASE+904:
    {
        rt_mutex_release(module->mod_mutex);
        return 0;
    }
    }
    rt_kprintf("syscall %d not supported\n",nbr-SYS_BASE);
    return -ENOTSUP;
}
#endif
