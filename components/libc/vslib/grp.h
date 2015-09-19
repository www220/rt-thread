#ifndef _GRP_H_
#define	_GRP_H_

#include <sys/types.h>

struct group {
	char	*gr_name;		/* group name */
	char	*gr_passwd;		/* group password */
	gid_t	gr_gid;			/* group id */
	char	**gr_mem;		/* group members */
};

struct group* getgrnam (const char *name);
struct group* getgrgid (gid_t id);

#endif