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
extern long int syscall (long int __sysno, ...) __THROW;
extern char *getlogin (void);
extern int getlogin_r (char *__name, size_t __name_len) __nonnull ((1));

//#include<time.h>
extern int stime (__const time_t *__when) __THROW;
extern int nanosleep (__const struct timespec *__requested_time, struct timespec *__remaining);

//#include<limit.h>
#define NAME_MAX         255	/* # chars in a file name */
#define	PATH_MAX		 1024	/* max bytes in pathname */

#endif //_NODEFFUNC
