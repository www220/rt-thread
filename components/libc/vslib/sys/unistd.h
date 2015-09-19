#ifndef _SYS_UNISTD_H
#define _SYS_UNISTD_H

# define	SEEK_SET	0
# define	SEEK_CUR	1
# define	SEEK_END	2

#include <sys/types.h>

pid_t getpid(void );
int access(const char *__path, int __amode );
int link(const char *__path1, const char *__path2 );

#endif