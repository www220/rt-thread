#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdarg.h>
#include <sys/syscall.h>
#include <reent.h>
#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/termios.h>
#include <signal.h>
#include <sys/poll.h>
#include <grp.h>
#include <pwd.h>
#include <limits.h>
#include <time.h>
#include <sys/types.h>

extern int _set_errno(int n);
extern int _getdents(int file, struct dirent *dirp, size_t nbytes);
extern int _fcntl(int fd, int flag, int arg);
extern pid_t _getpid(void);

/* SWI with SYS_ call number and no parameters */

static inline uintptr_t sys_call0(unsigned int nbr)
{
  register long reg0 __asm__("r0") = 0;

  __asm__ __volatile__
  (
    "swi %1"
    : "=r"(reg0)
    : "i"(nbr), "r"(reg0)
    : "memory", "r14"
  );

  return reg0;
}

/* SWI with SYS_ call number and one parameter */

static inline uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1)
{
  register long reg0 __asm__("r0") = (long)(parm1);

  __asm__ __volatile__
  (
    "swi %1"
    : "=r"(reg0)
    : "i"(nbr), "r"(reg0)
    : "memory", "r14"
  );

  return reg0;
}

/* SWI with SYS_ call number and two parameters */

static inline uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2)
{
  register long reg0 __asm__("r0") = (long)(parm1);
  register long reg1 __asm__("r1") = (long)(parm2);

  __asm__ __volatile__
  (
    "swi %1"
    : "=r"(reg0)
    : "i"(nbr), "r"(reg0), "r"(reg1)
    : "memory", "r14"
  );

  return reg0;
}

/* SWI with SYS_ call number and three parameters */

static inline uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3)
{
  register long reg0 __asm__("r0") = (long)(parm1);
  register long reg1 __asm__("r1") = (long)(parm2);
  register long reg2 __asm__("r2") = (long)(parm3);

  __asm__ __volatile__
  (
    "swi %1"
    : "=r"(reg0)
    : "i"(nbr), "r"(reg0), "r"(reg1), "r"(reg2)
    : "memory", "r14"
  );

  return reg0;
}

/* SWI with SYS_ call number and four parameters */

static inline uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4)
{
  register long reg0 __asm__("r0") = (long)(parm1);
  register long reg1 __asm__("r1") = (long)(parm2);
  register long reg2 __asm__("r2") = (long)(parm3);
  register long reg3 __asm__("r3") = (long)(parm4);

  __asm__ __volatile__
  (
    "swi %1"
    : "=r"(reg0)
    : "i"(nbr), "r"(reg0), "r"(reg1), "r"(reg2), "r"(reg3)
    : "memory", "r14"
  );

  return reg0;
}

int
_isatty(int fd)
{
    struct stat buf;
    if (fstat(fd, &buf) == 0 && S_ISCHR(buf.st_mode))
        return 1;
    return 0;
}

int
fcntl(int fd, int flag, ...)
{
	int arg;
	va_list ap;

	va_start(ap, flag);
	arg = va_arg(ap, int);
	int rc = _fcntl(fd, flag, arg);
	return _set_errno(rc);
}

int
ttyname_r (int fd, char *buf, size_t buflen)
{
	int rc = sys_call3(__NR_SYSCALL_BASE+1001, fd, (uintptr_t)buf, buflen);
	return _set_errno(rc);
}

int
lutimes (const char *path, const struct timeval tvp[2])
{
	int rc = sys_call2(__NR_SYSCALL_BASE+1002, (uintptr_t)path, (uintptr_t)tvp);
	return _set_errno(rc);
}

void
__malloc_lock (struct _reent *ptr)
{
    sys_call0(__NR_SYSCALL_BASE+901);
}

void
__malloc_unlock (struct _reent *ptr)
{
    sys_call0(__NR_SYSCALL_BASE+902);
}

void
__env_lock (struct _reent *ptr)
{
    sys_call0(__NR_SYSCALL_BASE+903);
}

void
__env_unlock (struct _reent *ptr)
{
    sys_call0(__NR_SYSCALL_BASE+904);
}

//#include<pwd.h>

int fchmod (int filedes, mode_t mode)
{
	int rc = sys_call2(__NR_fchmod, filedes, mode);
	return _set_errno(rc);
}

int fchown (int __filedes, uid_t __owner, gid_t __group)
{
	int rc = sys_call3(__NR_fchown, __filedes, __owner, __group);
	return _set_errno(rc);
}

int setgroups (int ngroups, const gid_t *grouplist)
{
	int rc = sys_call2(__NR_setgroups, ngroups, (uintptr_t)grouplist);
	return _set_errno(rc);
}

//#include<dirent.h>

#define O_DIRECTORY     0x0200000
DIR *opendir(const char *name)
{
	register DIR *dirp;
	register int fd;
	int rc = 0;

	if ((fd = open(name, O_RDONLY|O_DIRECTORY)) == -1)
		return NULL;
	if (rc == -1 ||
	    (dirp = (DIR *)malloc(sizeof(DIR))) == NULL) {
		close (fd);
		return NULL;
	}
    memset(dirp, 0, sizeof(DIR));
    dirp->fd = fd;
	return dirp;
}

struct dirent *readdir(DIR *d)
{
	register struct dirent *dp;
	int rc = 0;

	if (d->fd == -1)
		return NULL;

    if (d->num)
    {
        struct dirent* dirent_ptr;
        dirent_ptr = (struct dirent*)&d->buf[d->cur];
        d->cur += dirent_ptr->d_reclen;
    }

    if (!d->num || d->cur >= d->num)
    {
        /* get a new entry */
        rc = _getdents(d->fd,(struct dirent*)&d->buf,sizeof(d->buf)-1);
        if (rc <= 0)
        {
            _set_errno(rc);
            return NULL;
        }

        d->num = rc;
        d->cur = 0; /* current entry index */
    }

    return (struct dirent *)(d->buf+d->cur);
}

int closedir(DIR *d)
{
	int rc = 0;

	if (d->fd == -1)
		return -1;

    rc = close(d->fd);
    d->fd = -1;
    free(d);
	return rc;
}

//#include<time.h>

int utimes (const char *file, const struct timeval tvp[2])
{
	int rc = sys_call2(__NR_utimes, (uintptr_t)file, (uintptr_t)tvp);
	return _set_errno(rc);
}

unsigned sleep(unsigned int seconds)
{
    struct timespec ts;

    ts.tv_sec = seconds;
    ts.tv_nsec = 0;
    if (!nanosleep(&ts,&ts)) return 0;
    if (errno == EINTR) return ts.tv_sec;
    return -1;
}

int usleep(useconds_t useconds)
{
    struct timespec ts;

    ts.tv_sec = (long int)useconds / 1000000;
    ts.tv_nsec = ((long int)useconds % 1000000) * 1000;
    if (!nanosleep(&ts,&ts)) return 0;
    if (errno == EINTR) return ts.tv_sec;
    return -1;
}

//#include<sendfile.h>

ssize_t sendfile (int out_fd, int in_fd, off_t *offset, size_t count)
{
	int rc = sys_call4(__NR_sendfile, out_fd, in_fd, (uintptr_t)offset, count);
	return _set_errno(rc);
}
//#include<lwip.h>

/**
 * Convert an u16_t from host- to network byte order.
 *
 * @param n u16_t in host byte order
 * @return n in network byte order
 */
u16_t
lwip_htons(u16_t n)
{
  return ((n & 0xff) << 8) | ((n & 0xff00) >> 8);
}

/**
 * Convert an u16_t from network- to host byte order.
 *
 * @param n u16_t in network byte order
 * @return n in host byte order
 */
u16_t
lwip_ntohs(u16_t n)
{
  return lwip_htons(n);
}

/**
 * Convert an u32_t from host- to network byte order.
 *
 * @param n u32_t in host byte order
 * @return n in network byte order
 */
u32_t
lwip_htonl(u32_t n)
{
  return ((n & 0xff) << 24) |
    ((n & 0xff00) << 8) |
    ((n & 0xff0000UL) >> 8) |
    ((n & 0xff000000UL) >> 24);
}

/**
 * Convert an u32_t from network- to host byte order.
 *
 * @param n u32_t in network byte order
 * @return n in host byte order
 */
u32_t
lwip_ntohl(u32_t n)
{
  return lwip_htonl(n);
}

//#include<unistd.h>

int _fork(void)
{
	int rc = sys_call0(__NR_fork);
	return _set_errno(rc);
}

long int syscall (long int __sysno, ...)
{
	int rc,parm[4] = {0};
	va_list ap;

	va_start(ap, __sysno);
	switch(__sysno)
	{
	case __NR_clock_gettime:
		parm[0] = va_arg(ap, int);
		parm[1] = va_arg(ap, int);
		rc = sys_call2(__NR_clock_gettime,parm[0],parm[1]);
	default:
		printf("syscall %d not supported\n",(int)__sysno-__NR_SYSCALL_BASE);
		errno = ENOTSUP;
		return -1;
	}
	return _set_errno(rc);
}

pid_t getpid(void)
{
	return _getpid();
}

pid_t getppid(void)
{
	int rc = sys_call0(__NR_getppid);
	return _set_errno(rc);
}

pid_t getsid(pid_t __pid)
{
	int rc = sys_call1(__NR_getsid, __pid);
	return _set_errno(rc);
}

int issetugid(void)
{
	return 0;
}

int setpgrp(void)
{
	return setpgid(0,0);
}

int getpagesize(void)
{
	return 4096;
}

//#include<stdlib.h>

char *getcwd (char *pt, size_t size)
{
	//alloc buffer
	if (pt == NULL || size == 0)
	{
		size = 300;
		pt = (char *)malloc(size);
	}
	return (char *)sys_call2(__NR_getcwd, (uintptr_t)pt, size);
}

extern char **environ;
static char ***p_environ = &environ;
int clearenv (void)
{
	//make environ alloc
	setenv("CLEAR","PASS",0);
	//
	*p_environ = realloc(*p_environ,sizeof(char *));
	(*p_environ)[0] = NULL;
	return 0;
}

int fsync (int fd)
{
	int rc = sys_call1(__NR_fsync, fd);
	return _set_errno(rc);
}

int creat (const char *__file, mode_t mod)
{
	int rc = sys_call2(__NR_creat, (uintptr_t)__file, mod);
	return _set_errno(rc);
}

//#include<signal.h>

int sigaction (int sig, const struct sigaction *act, struct sigaction *oact)
{
	int rc = sys_call3(__NR_sigaction, sig, (uintptr_t)act, (uintptr_t)oact);
	return _set_errno(rc);
}

int sigsuspend (const sigset_t *mask)
{
	int rc = sys_call1(__NR_sigsuspend, (uintptr_t)mask);
	return _set_errno(rc);
}

int sigpending (sigset_t *set)
{
	int rc = sys_call1(__NR_sigpending, (uintptr_t)set);
	return _set_errno(rc);
}

# define __sigisemptyset(set)   (*(set) == (sigset_t) 0)
# define __sigandset(dest, left, right) \
                ((*(dest) = (*(left) & *(right))), 0)
# define __sigorset(dest, left, right) \
                ((*(dest) = (*(left) | *(right))), 0)
int sigisemptyset (const sigset_t *set)
{
	if (set == NULL)
		return _set_errno(-EINVAL);
	return __sigisemptyset(set);
}

int killpg (__pid_t __pgrp, int __sig)
{
	return kill(-__pgrp,__sig);
}

void
siglongjmp (sigjmp_buf env, int val)
{
  if (env.__is_mask_saved)
    sigprocmask (SIG_SETMASK, &env.__saved_mask, NULL);

  longjmp (env.__buf, val);
}

//#include<sys/poll.h>

int poll (struct pollfd *__fds, nfds_t __nfds, int __timeout)
{
	int rc = sys_call3(__NR_poll, (uintptr_t)__fds, __nfds, __timeout);
	return _set_errno(rc);
}

//#include<resource.h>

int getrlimit (__rlimit_resource_t __resource, struct rlimit *__rlimits)
{
	int rc = sys_call2(__NR_getrlimit, __resource, (uintptr_t)__rlimits);
	return _set_errno(rc);
}

int setrlimit (__rlimit_resource_t __resource, __const struct rlimit *__rlimits)
{
	int rc = sys_call2(__NR_setrlimit, __resource, (uintptr_t)__rlimits);
	return _set_errno(rc);
}

//#include<syslog.h>

static char *LogTag = NULL;      /* string to tag the entry with */
static int  LogMask = 0xff;     /* mask of priorities to be logged */

void closelog (void)
{
	printf("call closelog [%s]\n",LogTag);
	free(LogTag);
	LogTag = NULL;
}

void openlog (__const char *__ident, int __option, int __facility)
{
	if (LogTag)
		free(LogTag);
	LogTag = strdup(__ident?__ident:"");
	printf("call openlog  [%s] %d,%d\n",LogTag,__option,__facility);
}

int setlogmask (int __mask)
{
    int omask = LogMask;
    if (__mask != 0)
        LogMask = __mask;
    return (omask);
}

void syslog (int __pri, __const char *__fmt, ...)
{
	va_list __arg;

	va_start(__arg,__fmt);
	printf("call syslog   [%s] %d ",LogTag,__pri);
	vprintf(__fmt,__arg);printf("\n");
	va_end(__arg);
}

