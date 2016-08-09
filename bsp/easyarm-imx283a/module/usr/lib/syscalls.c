#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <reent.h>
#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <regex.h>
#include <fnmatch.h>
#include <sys/socket.h>
#include <sys/termios.h>
#include <signal.h>
#include <sys/poll.h>
#include <grp.h>

extern int _set_errno(int n);
extern int _getdents(int file, struct dirent *dirp, size_t nbytes);


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

void
__malloc_lock (struct _reent *ptr)
{
    sys_call0(0x900000+901);
}

void
__malloc_unlock (struct _reent *ptr)
{
    sys_call0(0x900000+902);
}

void
__env_lock (struct _reent *ptr)
{
    sys_call0(0x900000+903);
}

void
__env_unlock (struct _reent *ptr)
{
    sys_call0(0x900000+904);
}

//#include<string.h>

/* states: S_N: normal, S_I: comparing integral part, S_F: comparing
           fractional parts, S_Z: idem but with leading Zeroes only */
#define  S_N    0x0
#define  S_I    0x4
#define  S_F    0x8
#define  S_Z    0xC

/* result_type: CMP: return diff; LEN: compare using len_diff/diff */
#define  CMP    2
#define  LEN    3

#define ISDIGIT(c) ((unsigned int) (c) - '0' <= 9)

int strverscmp (__const char *s1, __const char *s2)
{
  const unsigned char *p1 = (const unsigned char *) s1;
  const unsigned char *p2 = (const unsigned char *) s2;
  unsigned char c1, c2;
  int state;
  int diff;

  /* Symbol(s)    0       [1-9]   others  (padding)
     Transition   (10) 0  (01) d  (00) x  (11) -   */
  static const unsigned int next_state[] =
    {
      /* state    x    d    0    - */
      /* S_N */  S_N, S_I, S_Z, S_N,
      /* S_I */  S_N, S_I, S_I, S_I,
      /* S_F */  S_N, S_F, S_F, S_F,
      /* S_Z */  S_N, S_F, S_Z, S_Z
    };

  static const int result_type[] =
    {
      /* state   x/x  x/d  x/0  x/-  d/x  d/d  d/0  d/-
                 0/x  0/d  0/0  0/-  -/x  -/d  -/0  -/- */

      /* S_N */  CMP, CMP, CMP, CMP, CMP, LEN, CMP, CMP,
                 CMP, CMP, CMP, CMP, CMP, CMP, CMP, CMP,
      /* S_I */  CMP, -1,  -1,  CMP, +1,  LEN, LEN, CMP,
                 +1,  LEN, LEN, CMP, CMP, CMP, CMP, CMP,
      /* S_F */  CMP, CMP, CMP, CMP, CMP, LEN, CMP, CMP,
                 CMP, CMP, CMP, CMP, CMP, CMP, CMP, CMP,
      /* S_Z */  CMP, +1,  +1,  CMP, -1,  CMP, CMP, CMP,
                 -1,  CMP, CMP, CMP
    };

  if (p1 == p2)
    return 0;

  c1 = *p1++;
  c2 = *p2++;
  /* Hint: '0' is a digit too.  */
  state = S_N | ((c1 == '0') + (ISDIGIT (c1) != 0));

  while ((diff = c1 - c2) == 0 && c1 != '\0')
    {
      state = next_state[state];
      c1 = *p1++;
      c2 = *p2++;
      state |= (c1 == '0') + (ISDIGIT (c1) != 0);
    }

  state = result_type[state << 2 | (((c2 == '0') + (ISDIGIT (c2) != 0)))];

  switch (state)
    {
    case CMP:
      return diff;
      
    case LEN:
      while (ISDIGIT (*p1++))
	if (!ISDIGIT (*p2++))
	  return 1;
      
      return ISDIGIT (*p2) ? -1 : diff;
      
    default:
      return state;
    }
}

//#include<pwd.h>

struct passwd *getpwuid (uid_t pid)
{
	errno = ENOSYS;
	return NULL;
}

int getpwuid_r (__uid_t __uid,
       struct passwd *__restrict __resultbuf,
       char *__restrict __buffer, size_t __buflen,
       struct passwd **__restrict __result)
{
	errno = ENOSYS;
	if (__result)
		*__result = NULL;
	return -1;
}

struct passwd *getpwnam (const char *name)
{
	errno = ENOSYS;
	return NULL;
}

int getpwnam_r (__const char *__restrict __name,
       struct passwd *__restrict __resultbuf,
       char *__restrict __buffer, size_t __buflen,
       struct passwd **__restrict __result)
{
	errno = ENOSYS;
	if (__result)
		*__result = NULL;
	return -1;
}

int fchmod (int filedes, mode_t mode)
{
	return 0;
}

static char* defuser = "root";
char *getlogin (void)
{
	return defuser;
}

int getlogin_r (char *__name, size_t __name_len)
{
	if (__name == NULL || __name_len == 0)
	{
		errno = ERANGE;
		return -1;
	}
	strncpy(__name,defuser,__name_len);
	return 0;
}

//#include<grp.h>
static char *defgroupuser[] = {"root", NULL};
static struct group defgroup = {"root", "", 1, defgroupuser};
struct group *getgrgid (gid_t gid)
{
	if (gid != defgroup.gr_gid)
	{
		errno = EINTR;
		return NULL;
	}
	return &defgroup;
}

struct group *getgrnam (const char *name)
{
	if (name == NULL || strcmp(name,defgroup.gr_name) != 0)
	{
		errno = EINTR;
		return NULL;
	}
	return &defgroup;
}

int getgrgid_r (__gid_t __gid, struct group *__restrict __resultbuf,
       char *__restrict __buffer, size_t __buflen,
       struct group **__restrict __result)
{
	if (__resultbuf == NULL || __gid != defgroup.gr_gid)
	{
		errno = EINTR;
		if (__result)
			*__result = NULL;
		return -1;
	}
	*__resultbuf = defgroup;
	if (__result)
		*__result = __resultbuf;
	return 0;
}

int getgrnam_r (__const char *__restrict __name,
       struct group *__restrict __resultbuf,
       char *__restrict __buffer, size_t __buflen,
       struct group **__restrict __result)
{
	if (__resultbuf == NULL || __name == NULL || strcmp(__name,defgroup.gr_name) != 0)
	{
		errno = EINTR;
		if (__result)
			*__result = NULL;
		return -1;
	}
	*__resultbuf = defgroup;
	if (__result)
		*__result = __resultbuf;
	return 0;
}

int initgroups (__const char *__user, __gid_t __group)
{
	return 0;
}

int getgrouplist(const char *user, gid_t group, gid_t *groups, int *ngroups)
{
	if (groups == NULL || ngroups == NULL 
		|| user == NULL || strcmp(user,defgroup.gr_name) != 0
		|| group != defgroup.gr_gid)
	{
		errno = EINTR;
		if (ngroups)
			*ngroups = 0;
		return -1;
	}
	groups[0] = defgroup.gr_gid;
	*ngroups = 1;
	return 0;
}

int fchown (int __filedes, uid_t __owner, gid_t __group)
{
	errno = EPERM;
	return -1;
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
    free(d);
	return rc;
}

//#include<libgen.h>

char *dirname(char *path)
{
	char *p;
	if( path == NULL || *path == '\0' )
		return ".";
	p = path + strlen(path) - 1;
	while( *p == '/' ) {
		if( p == path )
			return path;
		*p-- = '\0';
	}
	while( p >= path && *p != '/' )
		p--;
	return
		p < path ? "." :
		p == path ? "/" :
		(*p = '\0', path);
}

//#include<time.h>

int utimes(const char *path, const struct timeval *tvp)
{
	return 0;	
}
int lutimes(const char *path, const struct timeval *tvp)
{
	return 0;	
}

unsigned sleep(unsigned int __seconds)
{
	return 0;
}

int usleep(useconds_t __useconds)
{
	return 0;
}

//#include<sendfile.h>

ssize_t sendfile(int out_fd, int in_fd, off_t *offset, size_t count)
{
	return 0;
}

//#include<regex.h>

int regcomp (regex_t *__restrict __preg, const char *__restrict __pattern, int __cflags)
{
	return 0;
}

size_t regerror (int __errcode, const regex_t *__restrict __preg, char *__restrict __errbuf, size_t __errbuf_size)
{
	return 0;
}

int regexec(const regex_t *__restrict __preg, const char *__restrict __string, size_t __nmatch, regmatch_t __pmatch[__restrict], int __eflags)
{
	return 0;
}

void regfree(regex_t *__preg)
{
}

//#include<fnmatch.h>

#undef	EOS
#define	EOS	'\0'

/* Macros to set/clear/test flags. */
#undef SET
#define SET(t, f)	((t) |= (f))
#undef CLR
#define CLR(t, f)	((t) &= ~(f))
#undef ISSET
#define ISSET(t, f)	((t) & (f))

#define	RANGE_MATCH	1
#define	RANGE_NOMATCH	0
#define	RANGE_ERROR	(-1)

static int rangematch __P((const char *, char, int, char **));

int
fnmatch(pattern, string, flags)
	const char *pattern, *string;
	int flags;
{
	const char *stringstart;
	char *newp;
	char c, test;

	for (stringstart = string;;)
		switch (c = *pattern++) {
		case EOS:
			if (ISSET(flags, FNM_LEADING_DIR) && *string == '/')
				return (0);
			return (*string == EOS ? 0 : FNM_NOMATCH);
		case '?':
			if (*string == EOS)
				return (FNM_NOMATCH);
			if (*string == '/' && ISSET(flags, FNM_PATHNAME))
				return (FNM_NOMATCH);
			if (*string == '.' && ISSET(flags, FNM_PERIOD) &&
			    (string == stringstart ||
			    (ISSET(flags, FNM_PATHNAME) && *(string - 1) == '/')))
				return (FNM_NOMATCH);
			++string;
			break;
		case '*':
			c = *pattern;
			/* Collapse multiple stars. */
			while (c == '*')
				c = *++pattern;

			if (*string == '.' && ISSET(flags, FNM_PERIOD) &&
			    (string == stringstart ||
			    (ISSET(flags, FNM_PATHNAME) && *(string - 1) == '/')))
				return (FNM_NOMATCH);

			/* Optimize for pattern with * at end or before /. */
			if (c == EOS) {
				if (ISSET(flags, FNM_PATHNAME))
					return (ISSET(flags, FNM_LEADING_DIR) ||
					    strchr(string, '/') == NULL ?
					    0 : FNM_NOMATCH);
				else
					return (0);
			} else if (c == '/' && ISSET(flags, FNM_PATHNAME)) {
				if ((string = strchr(string, '/')) == NULL)
					return (FNM_NOMATCH);
				break;
			}

			/* General case, use recursion. */
			while ((test = *string) != EOS) {
				if (!fnmatch(pattern, string, flags & ~FNM_PERIOD))
					return (0);
				if (test == '/' && ISSET(flags, FNM_PATHNAME))
					break;
				++string;
			}
			return (FNM_NOMATCH);
		case '[':
			if (*string == EOS)
				return (FNM_NOMATCH);
			if (*string == '/' && ISSET(flags, FNM_PATHNAME))
				return (FNM_NOMATCH);
			if (*string == '.' && ISSET(flags, FNM_PERIOD) &&
			    (string == stringstart ||
			    (ISSET(flags, FNM_PATHNAME) && *(string - 1) == '/')))
				return (FNM_NOMATCH);

			switch (rangematch(pattern, *string, flags, &newp)) {
			case RANGE_ERROR:
				/* not a good range, treat as normal text */
				goto normal;
			case RANGE_MATCH:
				pattern = newp;
				break;
			case RANGE_NOMATCH:
				return (FNM_NOMATCH);
			}
			++string;
			break;
		case '\\':
			if (!ISSET(flags, FNM_NOESCAPE)) {
				if ((c = *pattern++) == EOS) {
					c = '\\';
					--pattern;
				}
			}
			/* FALLTHROUGH */
		default:
		normal:
			if (c != *string && !(ISSET(flags, FNM_CASEFOLD) &&
				 (tolower((unsigned char)c) ==
				 tolower((unsigned char)*string))))
				return (FNM_NOMATCH);
			++string;
			break;
		}
	/* NOTREACHED */
}

static int
#ifdef __STDC__
rangematch(const char *pattern, char test, int flags, char **newp)
#else
rangematch(pattern, test, flags, newp)
	const char *pattern;
	char test;
	int flags;
	char **newp;
#endif
{
	int negate, ok;
	char c, c2;

	/*
	 * A bracket expression starting with an unquoted circumflex
	 * character produces unspecified results (IEEE 1003.2-1992,
	 * 3.13.2).  This implementation treats it like '!', for
	 * consistency with the regular expression syntax.
	 * J.T. Conklin (conklin@ngai.kaleida.com)
	 */
	if ((negate = (*pattern == '!' || *pattern == '^')))
		++pattern;

	if (ISSET(flags, FNM_CASEFOLD))
		test = tolower((unsigned char)test);

	/*
	 * A right bracket shall lose its special meaning and represent
	 * itself in a bracket expression if it occurs first in the list.
	 * -- POSIX.2 2.8.3.2
	 */
	ok = 0;
	c = *pattern++;
	do {
		if (c == '\\' && !ISSET(flags, FNM_NOESCAPE))
			c = *pattern++;
		if (c == EOS)
			return (RANGE_ERROR);
		if (c == '/' && ISSET(flags, FNM_PATHNAME))
			return (RANGE_NOMATCH);
		if (ISSET(flags, FNM_CASEFOLD))
			c = tolower((unsigned char)c);
		if (*pattern == '-'
		    && (c2 = *(pattern+1)) != EOS && c2 != ']') {
			pattern += 2;
			if (c2 == '\\' && !ISSET(flags, FNM_NOESCAPE))
				c2 = *pattern++;
			if (c2 == EOS)
				return (RANGE_ERROR);
			if (ISSET(flags, FNM_CASEFOLD))
				c2 = tolower((unsigned char)c2);
			if (c <= test && test <= c2)
				ok = 1;
		} else if (c == test)
			ok = 1;
	} while ((c = *pattern++) != ']');

	*newp = (char *)pattern;
	return (ok == negate ? RANGE_NOMATCH : RANGE_MATCH);
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

int execl(const char *__path, const char *__arg, ... )
{
	return -1;
}

int execle(const char *__path, const char *__arg, ... )
{
	return -1;
}

int execlp(const char *__file, const char *__arg, ... )
{
	return -1;
}

int execv(const char *__file ,char * const __argv [])
{
	return -1;
}

int execve(const char *__path ,char * const __argv [] ,char * const __envp[])
{
	return -1;
}

int execvp(const char *__file ,char * const __argv [])
{
	return -1;
}

int _fork(void)
{
    return -1;
}

long int sysconf (int name)
{
	return -1;
}

long int syscall (long int __sysno, ...)
{
	return -1;
}

//#include<stdlib.h>

char *getcwd (char *pt, size_t size)
{
	return pt;
}

static int resolve_path(char *path,char *result,char *pos)
{
    if (*path == '/') {
	*result = '/';
	pos = result+1;
	path++;
    }
    *pos = 0;
    if (!*path) return 0;
    while (1) {
	char *slash;
	struct stat st;

	slash = *path ? strchr(path,'/') : NULL;
	if (slash) *slash = 0;
	if (!path[0] || (path[0] == '.' &&
	  (!path[1] || (path[1] == '.' && !path[2])))) {
	    pos--;
	    if (pos != result && path[0] && path[1])
		while (*--pos != '/');
	}
	else {
	    strcpy(pos,path);
	    if (lstat(result,&st) < 0) return -1;
	    if (S_ISLNK(st.st_mode)) {
		char buf[PATH_MAX];

		if (readlink(result,buf,sizeof(buf)) < 0) return -1;
		*pos = 0;
		if (slash) {
		    *slash = '/';
		    strcat(buf,slash);
		}
		strcpy(path,buf);
		if (*path == '/') result[1] = 0;
		pos = strchr(result,0);
		continue;
	    }
	    pos = strchr(result,0);
	}
	if (slash) {
	    *pos++ = '/';
	    path = slash+1;
	}
	*pos = 0;
	if (!slash) break;
    }
    return 0;
}

char *realpath(const char *__restrict path,char *__restrict resolved_path)
{
    char cwd[PATH_MAX];
    char *path_copy;
    int res;

    if (!*path) {
	errno = ENOENT; /* SUSv2 */
	return NULL;
    }
    if (!getcwd(cwd,sizeof(cwd))) return NULL;
    strcpy(resolved_path,"/");
    if (resolve_path(cwd,resolved_path,resolved_path)) return NULL;
    strcat(resolved_path,"/");
    path_copy = strdup(path);
    if (!path_copy) return NULL;
    res = resolve_path(path_copy,resolved_path,strchr(resolved_path,0));
    free(path_copy);
    if (res) return NULL;
    return resolved_path;
}

int clearenv (void)
{
	return -1;
}

int fsync(int fd)
{
	return -1;
}

int creat(const char *__file, mode_t mod)
{
	return -1;
}

//#include<sys/termios.h>

int tcgetattr (int __fd, struct termios *__termios_p) __THROW
{
	return -1;
}

int tcsetattr (int __fd, int __optional_actions, __const struct termios *__termios_p) __THROW
{
	return -1;
}

//#include<signal.h>

int sigaction(int a, const struct sigaction * b, struct sigaction * c)
{
	return -1;
}

//#include<sys/poll.h>

int poll (struct pollfd *__fds, nfds_t __nfds, int __timeout)
{
	return -1;
}
