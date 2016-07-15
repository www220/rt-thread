#ifndef __RTT_FCNTL_H__
#define __RTT_FCNTL_H__

/* Operation flags */
#define O_RDONLY        0x0000000
#define O_WRONLY        0x0000001
#define O_RDWR          0x0000002
#define O_ACCMODE       0x0000003
#define O_CREAT         0x0000200
#define O_EXCL          0x0000800
#define O_TRUNC         0x0000400
#define O_APPEND        0x0000008
#define O_BINARY        0x0010000
#define O_TEXT          0x0020000
#define O_DIRECTORY     0x0200000

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <dfs_posix.h>

#define	F_DUPFD		0	/* Duplicate fildes */
#define	F_GETFD		1	/* Get fildes flags (close on exec) */
#define	F_SETFD		2	/* Set fildes flags (close on exec) */
#define	F_GETFL		3	/* Get file flags */
#define	F_SETFL		4	/* Set file flags */

#endif
