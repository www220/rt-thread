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
#define SA_THIRTYTWO	0x02000000
#define SA_RESTORER		0x04000000
#define SA_ONSTACK		0x08000000
#define SA_RESTART		0x10000000
#define SA_NODEFER		0x40000000
#define SA_RESETHAND	0x80000000
int _EXFUN(sigaction, (int, const struct sigaction *, struct sigaction *));
int _EXFUN(sigsuspend, (const sigset_t *));
int _EXFUN(kill, (pid_t, int));
int _EXFUN(sigpending, (sigset_t *));
int _EXFUN(sigpause, (int));
extern int sigisemptyset (__const sigset_t *__set) __THROW;
extern int killpg (__pid_t __pgrp, int __sig) __THROW;

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
int _EXFUN(_execve, (const char *__path, char * const __argv[], char * const __envp[] ));

//#include<time.h>
extern int stime (__const time_t *__when) __THROW;
extern int nanosleep (__const struct timespec *__requested_time, struct timespec *__remaining);

//#include<limit.h>
#define NAME_MAX         255	/* # chars in a file name */
#define	PATH_MAX		 1024	/* max bytes in pathname */
#define MAXPATHLEN		 PATH_MAX

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

# define RLIMIT_CPU			0	/* CPU time in sec */
# define RLIMIT_FSIZE		1	/* Maximum filesize */
# define RLIMIT_DATA		2	/* max data size */
# define RLIMIT_STACK		3	/* max stack size */
# define RLIMIT_CORE		4	/* max core file size */
# define RLIMIT_RSS			5	/* max resident set size */
# define RLIMIT_NPROC		6	/* max number of processes */
# define RLIMIT_NOFILE		7	/* max number of open files */
# define RLIMIT_MEMLOCK		8	/* max locked-in-memory address space */
# define RLIMIT_AS			9	/* address space limit */
# define RLIMIT_LOCKS		10	/* maximum file locks held */
# define RLIMIT_SIGPENDING	11	/* max number of pending signals */
# define RLIMIT_MSGQUEUE	12	/* maximum bytes in POSIX mqueues */
# define RLIMIT_NICE		13	/* max nice prio allowed to raise to
					   0-39 for nice level 19 .. -20 */
# define RLIMIT_RTPRIO		14	/* maximum realtime priority */
# define RLIMIT_RTTIME		15	/* timeout for RT tasks in us */
# define RLIM_NLIMITS		16
# define RLIM_INFINITY ((unsigned long int)(~0UL))
extern int getrlimit (__rlimit_resource_t __resource, struct rlimit *__rlimits) __THROW;
extern int setrlimit (__rlimit_resource_t __resource, __const struct rlimit *__rlimits) __THROW;

//#include<wait.h>
#define WCOREDUMP(w)	(((w) & 0x80) == 0x80)

//#include<glob.h>
/* Error returns from `glob'.  */
#define	GLOB_NOMATCH	(-3)	/* No matches found.  */

//#include<paths.h>
#define	_PATH_TTY	"/dev/tty"

#endif //_NODEFFUNC
