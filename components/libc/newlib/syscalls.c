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
	/* return "not supported" */
	ptr->_errno = ENOTSUP;
	return -1;
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

	/* return "not supported" */
	ptr->_errno = ENOTSUP;
	return -1;
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
gettimeofday(struct timeval *__tp, void *__tzp)
{
#ifndef RT_USING_RTC
	/* return "not supported" */
	return -1;
#else
	static time_t sysnow = 0;
	static uint16_t sysms = 0;
	time_t nownow = time(NULL);
	uint16_t nowms = rt_tick_get() % 1000;
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
	return 0;
#endif
}
RTM_EXPORT(gettimeofday);

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
		struct rt_list_node *list;
		struct rt_object *object;

		rt_enter_critical();
		
        /* delete all threads in the module */
        list = &module->module_object[RT_Object_Class_Thread].object_list;
        while (list->next != list)
        {
            object = rt_list_entry(list->next, struct rt_object, list);
            if (rt_object_is_systemobject(object) == RT_TRUE)
            {
                /* detach static object */
                rt_thread_detach((rt_thread_t)object);
            }
            else
            {
                /* delete dynamic object */
                rt_thread_delete((rt_thread_t)object);
            }
        }
		/* delete main thread */
		rt_thread_delete(module->module_thread);
		rt_exit_critical();

		/* re-schedule */
		rt_schedule();
	}
#endif
#ifdef RT_USING_PROCESS
	rt_process_t process;

	process = rt_process_self();
	if (process != RT_NULL)
	{
		/* unload assertion module */
		rt_process_unload(process, (status&0xff)<<8);

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
#include "linux-usedef.h"
extern void *rt_process_conv_ptr(rt_process_t module, rt_uint32_t ptr, rt_uint32_t size);
extern rt_uint32_t rt_process_brk(rt_process_t module, rt_uint32_t addr);
extern int rt_process_fork(rt_process_t module);
extern int rt_process_vfork(rt_process_t module);
extern int rt_process_waitpid(rt_process_t module, pid_t pid, int* status, int opt);
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

rt_uint32_t sys_call_switch(rt_uint32_t nbr, rt_uint32_t parm1,
        rt_uint32_t parm2, rt_uint32_t parm3,
        rt_uint32_t parm4, rt_uint32_t parm5,
        rt_uint32_t parm6)
{
	rt_process_t module = rt_process_self();

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
        return rt_process_brk(module,parm1);
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
    case SYS_nanosleep:
    {
        struct timespec *tim1 = (parm1)?(rt_process_conv_ptr(module,parm1,sizeof(struct timespec))):(0);
        struct timespec *tim2 = (parm2)?(rt_process_conv_ptr(module,parm2,sizeof(struct timespec))):(0);
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
    case SYS_getpid:
    {
        return module->tpid;
    }
    case SYS_getppid:
    {
        return pidinfo[module->tpid-1][1];
    }
    case SYS_getpgrp:
    {
        return pidinfo[module->tpid-1][2];
    }
    case SYS_getpgid:
    {
    	if (parm1 == 0)
    		parm1 = module->tpid;
    	if (parm1 < 0 && parm1 > MAX_PID_SIZE)
    		return -EINVAL;
    	if (pidinfo[parm1-1][0] < 100)
    		return -ESRCH;
    	return pidinfo[parm1-1][2];
    }
    case SYS_getsid:
    {
    	if (parm1 == 0)
    		parm1 = module->tpid;
    	if (parm1 < 0 && parm1 > MAX_PID_SIZE)
    		return -EINVAL;
    	if (pidinfo[parm1-1][0] < 100)
    		return -ESRCH;
    	return pidinfo[parm1-1][3];
    }
    case SYS_setpgid:
    {
    	if (parm1 == 0)
    		parm1 = module->tpid;
    	if (parm2 == 0)
    		parm2 = module->tpid;
    	if (parm1 < 0 && parm1 > MAX_PID_SIZE)
    		return -EINVAL;
    	if (pidinfo[parm1-1][0] < 100)
    		return -ESRCH;
    	if (parm2 < 0 && parm2 > MAX_PID_SIZE)
    		return -EINVAL;
    	if (pidinfo[parm2-1][0] < 100)
    		return -ESRCH;
    	return pidinfo[parm1-1][2] = parm2;
    }
    case SYS_setsid:
    {
    	if (module->tpid == pidinfo[module->tpid-1][2])
    		return -EPERM;
    	pidinfo[module->tpid-1][2] = module->tpid;
    	pidinfo[module->tpid-1][3] = module->tpid;
    	return module->tpid;
    }
    case SYS_reboot:
    {
        if ((parm1 != 0xfee1dead) || (parm1 != 0x28121969))
            return -EINVAL;
        if (parm3 == RB_ENABLE_CAD || parm3 == RB_DISABLE_CAD)
            return 0;
        extern void sys_reboot(void);
        sys_reboot();
        return 0;
    }
    case SYS_fork:
    {
    	return rt_process_fork(module);
    }
    case SYS_vfork:
    {
    	return rt_process_vfork(module);
    }
    case SYS_execve:
    {
        const char *file = (const char *)rt_process_conv_ptr(module,parm1,0);
        rt_kprintf("syscall execve %s\n",file);
        return 0;
    }
    case SYS_wait4+1111:
    {
        int *status = (parm2)?((int *)rt_process_conv_ptr(module,parm2,sizeof(int))):RT_NULL;
        return rt_process_waitpid(module,(pid_t)parm1,status,parm3);
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
        gid_t *groups = rt_process_conv_ptr(module,parm2,parm1*sizeof(gid_t));
        groups[0] = 0;
        return 1;
    }
    case SYS_setgroups:
    {
        return -ENOTSUP;
    }
    case SYS_link:
    {
        errno = 0;
        return ret_err(link((const char *)rt_process_conv_ptr(module,parm1,0),
                (const char *)rt_process_conv_ptr(module,parm2,0)));
    }
    case SYS_symlink:
    {
        errno = 0;
        return ret_err(symlink((const char *)rt_process_conv_ptr(module,parm1,0),
                (const char *)rt_process_conv_ptr(module,parm2,0)));
    }
    case SYS_readlink:
    {
        errno = 0;
        return ret_err(readlink((const char *)rt_process_conv_ptr(module,parm1,0),
                (char *)rt_process_conv_ptr(module,parm2,parm3),parm3));
    }
    case SYS_truncate:
    {
        errno = 0;
        return ret_err(truncate((const char *)rt_process_conv_ptr(module,parm1,0),parm2));
    }
    case SYS_ftruncate:
    {
        errno = 0;
        return ret_err(ftruncate(parm1,parm2));
    }
    case SYS_uname:
    {
        struct utsname* uname = (struct utsname*)rt_process_conv_ptr(module,parm1,sizeof(struct utsname));
        strcpy(uname->sysname,"RT-Thread");
        strcpy(uname->nodename,"local");
        sprintf(uname->release,"%ld.%ld.%ld",RT_VERSION,RT_SUBVERSION,RT_REVISION);
        strcpy(uname->version,__DATE__);
        strcpy(uname->machine,"ARM");
        strcpy(uname->__domainname,"localdomain");
        return 0;
    }
    case SYS_unlink:
    case SYS_rmdir:
    {
        errno = 0;
        return ret_err(unlink((const char *)rt_process_conv_ptr(module,parm1,0)));
    }
    case SYS_rename:
    {
        errno = 0;
        return ret_err(rename((const char *)rt_process_conv_ptr(module,parm1,0),
                (const char *)rt_process_conv_ptr(module,parm2,0)));
    }
    case SYS_mkdir:
    {
        errno = 0;
        return ret_err(mkdir((const char *)rt_process_conv_ptr(module,parm1,0),parm2));
    }
    case SYS_lseek:
    {
        errno = 0;
        return ret_err(lseek(parm1, parm2, parm3));
    }
    case SYS_stat:
    {
        errno = 0;
        return ret_err(stat((const char *)rt_process_conv_ptr(module,parm1,0),
                (struct stat *)rt_process_conv_ptr(module,parm2,sizeof(struct stat))));
    }
    case SYS_lstat:
    {
        errno = 0;
        return ret_err(lstat((const char *)rt_process_conv_ptr(module,parm1,0),
                (struct stat *)rt_process_conv_ptr(module,parm2,sizeof(struct stat))));
    }
    case SYS_fstat:
    {
        errno = 0;
        return ret_err(fstat(parm1, (struct stat *)rt_process_conv_ptr(module,parm2,sizeof(struct stat))));
    }
    case SYS_statfs:
    {
        errno = 0;
        return ret_err(statfs((const char *)rt_process_conv_ptr(module,parm1,0),
                (struct statfs *)rt_process_conv_ptr(module,parm2,sizeof(struct statfs))));
    }
    case SYS_fstatfs:
    {
        errno = 0;
        return ret_err(fstatfs(parm1,(struct statfs *)rt_process_conv_ptr(module,parm2,sizeof(struct statfs))));
    }
    case SYS_creat:
    {
        errno = 0;
        return ret_err(open((const char*)rt_process_conv_ptr(module,parm1,0), O_TRUNC|O_WRONLY|O_CREAT, parm2));
    }
    case SYS_open:
    {
       errno = 0;
       return ret_err(open((const char*)rt_process_conv_ptr(module,parm1,0), parm2, parm3));
    }
    case SYS_read:
    {
       errno = 0;
       return ret_err(read(parm1, rt_process_conv_ptr(module,parm2,parm3), parm3));
    }
    case SYS_write:
    {
       errno = 0;
       return ret_err(write(parm1, rt_process_conv_ptr(module,parm2,parm3), parm3));
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
        return ret_err(getdents(parm1, (struct dirent*)rt_process_conv_ptr(module,parm2,parm3), parm3));
    }
    case SYS_sendfile:
    {
        char buf[4096];
        int rc,len,readlen = 0;
        off_t *off = (parm3)?(rt_process_conv_ptr(module,parm3,sizeof(off_t))):(0);
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
        char *buf = rt_process_conv_ptr(module,parm1,parm2);
        return (rt_uint32_t)getcwd(buf,parm2);
    }
    case SYS_chdir:
    {
        errno = 0;
        return ret_err(chdir((const char*)rt_process_conv_ptr(module,parm1,0)));
    }
    case SYS_fsync:
    {
        errno = 0;
        return ret_err(fsync(parm1));
    }
    case SYS_poll:
    {
        return parm2;
    }
    case SYS_gettimeofday:
    {
        errno = 0;
        struct timezone *zone = (parm2)?((struct timezone *)rt_process_conv_ptr(module,parm2,sizeof(struct timezone))):RT_NULL;
        return ret_err(gettimeofday((struct timeval *)rt_process_conv_ptr(module,parm1,sizeof(struct timeval)),zone));
    }
    case SYS_settimeofday:
    {
        errno = 0;
        struct timezone *zone = (parm2)?((struct timezone *)rt_process_conv_ptr(module,parm2,sizeof(struct timezone))):RT_NULL;
        return ret_err(settimeofday((struct timeval *)rt_process_conv_ptr(module,parm1,sizeof(struct timeval)),zone));
    }
    case SYS_ioctl:
    {
        errno = 0;
        switch(parm2)
        {
        case TIOCGWINSZ:
        {
            struct winsize *win = (struct winsize *)rt_process_conv_ptr(module,parm3,sizeof(struct winsize));
            memset(win,0,sizeof(struct winsize));
            win->ws_col = 80;
            win->ws_row = 60;
            return 0;
        }
        case TIOCSWINSZ:
        {
            struct winsize *win = (struct winsize *)rt_process_conv_ptr(module,parm3,sizeof(struct winsize));
            return 0;
        }
        case TCGETS:
        {
            struct termios *ios = (struct termios *)rt_process_conv_ptr(module,parm3,sizeof(struct termios));
            memset(ios,0,sizeof(struct termios));
            ios->c_iflag = ICRNL | IXON | IXOFF;
            ios->c_oflag = OPOST | ONLCR;
            ios->c_cflag = B115200 | CS8 | CREAD | HUPCL | CLOCAL;
            ios->c_lflag = ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | IEXTEN;
            return 0;
        }
        case TCSETS:
        {
            struct termios *ios = (struct termios *)rt_process_conv_ptr(module,parm3,sizeof(struct termios));
            return 0;
        }
        }
        rt_kprintf("ioctl file:%d cmd:0x%x data:0x%x\n",parm1,parm2,parm3);
        return -ENOTSUP;
    }
    case SYS_fcntl:
    {
        rt_kprintf("fcntl file:%d cmd:0x%x data:0x%x\n",parm1,parm2,parm3);
        return -ENOTSUP;
    }
    case SYS_BASE+901:
    case SYS_BASE+903:
    {
        rt_mutex_take(module->page_mutex, RT_WAITING_FOREVER);
        return 0;
    }
    case SYS_BASE+902:
    case SYS_BASE+904:
    {
        rt_mutex_release(module->page_mutex);
        return 0;
    }
    }
    rt_kprintf("syscall %d not supported\n",nbr-SYS_BASE);
    return -ENOTSUP;
}
#endif
