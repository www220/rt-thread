#ifndef	_NODEFFUNC_H
#define	_NODEFFUNC_H	1

#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>

//#include<sys/stat.h>
typedef __int32_t blksize_t;
typedef __int32_t blkcnt_t;
int	_EXFUN(lstat,( const char *__restrict __path, struct stat *__restrict __buf ));
int	_EXFUN(mknod,( const char *__path, mode_t __mode, dev_t __dev ));

//#include<string.h>
int strverscmp (__const char *__s1, __const char *__s2);

//#include<grp.h>
int initgroups (__const char *__user, __gid_t __group);
int getgrouplist (const char *user, gid_t group, gid_t *groups, int *ngroups);

//#include<signal.h>
#define SA_RESTART	0x10000000
int _EXFUN(sigaction, (int, const struct sigaction *, struct sigaction *));
int _EXFUN(sigsuspend, (const sigset_t *));
int _EXFUN(kill, (pid_t, int));

//#include<stdio.h>
#define getline __getline

//#include<stdlib.h>
int clearenv (void);

//#include<unistd.h>
int _EXFUN(setegid, (gid_t __gid ));
int _EXFUN(seteuid, (uid_t __uid ));
int _EXFUN(fchdir, (int __fildes));
int _EXFUN(chroot, (const char *__path ));
int _EXFUN(ttyname_r, (int, char *, size_t));
void _EXFUN(sync, (void));
pid_t _EXFUN(getsid, (pid_t));
extern long int syscall (long int __sysno, ...) __THROW;
extern char *getlogin (void);
extern int getlogin_r (char *__name, size_t __name_len) __nonnull ((1));

//#include<time.h>
extern int stime (__const time_t *__when) __THROW;
extern int nanosleep (__const struct timespec *__requested_time, struct timespec *__remaining);

//#include<limit.h>
#define NAME_MAX         255	/* # chars in a file name */
#define	PATH_MAX		 1024	/* max bytes in pathname */

//#include<resource.h>
typedef int __rlimit_resource_t;
typedef unsigned long int rlim_t;

struct rlimit
{
    /* The current (soft) limit.  */
    rlim_t rlim_cur;
    /* The hard limit.  */
    rlim_t rlim_max;
};

# define RLIM_INFINITY ((unsigned long int)(~0UL))
extern int getrlimit (__rlimit_resource_t __resource, struct rlimit *__rlimits) __THROW;
extern int setrlimit (__rlimit_resource_t __resource, __const struct rlimit *__rlimits) __THROW;

//#include<wait.h>
#define	__WCOREFLAG		0x80
#define	__WCOREDUMP(status)	((status) & __WCOREFLAG)
# define __WAIT_INT(status)	(status)
# define WCOREDUMP(status)	__WCOREDUMP (__WAIT_INT (status))

#endif //_NODEFFUNC
