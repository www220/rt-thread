#ifndef _PWD_H_
#define	_PWD_H_

#include <sys/types.h>
struct passwd {
	char	*pw_name;		/* user name */
	char	*pw_passwd;		/* encrypted password */
	uid_t	pw_uid;			/* user uid */
	gid_t	pw_gid;			/* user gid */
	char	*pw_comment;		/* comment */
	char	*pw_gecos;		/* Honeywell login info */
	char	*pw_dir;		/* home directory */
	char	*pw_shell;		/* default shell */
};

struct passwd* getpwuid (uid_t id);
struct passwd* getpwnam (const char *name);

#endif