/* vi: set sw=4 ts=4: */
/* Copyright (C) 2014 Tito Ragusa <farmatito@tiscali.it>
 *
 * Licensed under GPLv2 or later, see file LICENSE in this source tree.
 */
/* This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY!!
 *
 * Rewrite of some parts. Main differences are:
 *
 * 1) the buffer for getpwuid, getgrgid, getpwnam, getgrnam is dynamically
 *    allocated.
 *    If ENABLE_FEATURE_CLEAN_UP is set the buffers are freed at program
 *    exit using the atexit function to make valgrind happy.
 * 2) the passwd/group files:
 *      a) must contain the expected number of fields (as per count of field
 *         delimeters ":") or we will complain with a error message.
 *      b) leading and trailing whitespace in fields is stripped.
 *      c) some fields are not allowed to be empty (e.g. username, uid/gid),
 *         and in this case NULL is returned and errno is set to EINVAL.
 *         This behaviour could be easily changed by modifying PW_DEF, GR_DEF,
 *         SP_DEF strings (uppercase makes a field mandatory).
 *      d) the string representing uid/gid must be convertible by strtoXX
 *         functions, or errno is set to EINVAL.
 *      e) leading and trailing whitespace in group member names is stripped.
 * 3) the internal function for getgrouplist uses dynamically allocated buffer.
 * 4) at the moment only the functions really used by busybox code are
 *    implemented, if you need a particular missing function it should be
 *    easy to write it by using the internal common code.
 */

//#include "libbb.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <grp.h>
#include <pwd.h>
#include <stdarg.h>
#include <errno.h>
#include <ctype.h>
#include <fcntl.h>

/* FAST_FUNC is a qualifier which (possibly) makes function call faster
 * and/or smaller by using modified ABI. It is usually only needed
 * on non-static, busybox internal functions. Recent versions of gcc
 * optimize statics automatically. FAST_FUNC on static is required
 * only if you need to match a function pointer's type */
#if __GNUC_PREREQ(3,0) && defined(i386) /* || defined(__x86_64__)? */
/* stdcall makes callee to pop arguments from stack, not caller */
# define FAST_FUNC __attribute__((regparm(3),stdcall))
/* #elif ... - add your favorite arch today! */
#else
# define FAST_FUNC
#endif

/* Make all declarations hidden (-fvisibility flag only affects definitions) */
/* (don't include system headers after this until corresponding pop!) */
#if __GNUC_PREREQ(4,1) && !defined(__CYGWIN__)
# define PUSH_AND_SET_FUNCTION_VISIBILITY_TO_HIDDEN _Pragma("GCC visibility push(hidden)")
# define POP_SAVED_FUNCTION_VISIBILITY              _Pragma("GCC visibility pop")
#else
# define PUSH_AND_SET_FUNCTION_VISIBILITY_TO_HIDDEN
# define POP_SAVED_FUNCTION_VISIBILITY
#endif

#include <shadow.h>

#define UNUSED
#define ENABLE_USE_BB_SHADOW 1
#define _PATH_PASSWD  "/etc/passwd"
#define _PATH_GROUP   "/etc/group"
#define _PATH_SHADOW  "/etc/shadow"

static FILE* FAST_FUNC fopen_for_read(const char *path)
{
	return fopen(path, "r");
}
static ssize_t FAST_FUNC safe_write(int fd, const void *buf, size_t count)
{
	ssize_t n;

	do {
		n = write(fd, buf, count);
	} while (n < 0 && errno == EINTR);

	return n;
}
static ssize_t FAST_FUNC full_write(int fd, const void *buf, size_t len)
{
	ssize_t cc;
	ssize_t total;

	total = 0;

	while (len) {
		cc = safe_write(fd, buf, len);

		if (cc < 0) {
			if (total) {
				/* we already wrote some! */
				/* user can do another write to know the error code */
				return total;
			}
			return cc;  /* write() returns -1 on failure. */
		}

		total += cc;
		buf = ((const char *)buf) + cc;
		len -= cc;
	}

	return total;
}
static int FAST_FUNC fflush_all(void)
{
	return fflush(NULL);
}

#undef isblank
#define isblank(a) ({ unsigned char bb__isblank = (a); bb__isblank == ' ' || bb__isblank == '\t'; })
#define LOGMODE_STDIO 1
static char local_buf[40];
static int logmode = LOGMODE_STDIO;
static const char *applet_name = "applet: ";
static const char *bb_msg_memory_exhausted = "out of memory";
static const char *msg_eol = "\n";
static void FAST_FUNC bb_verror_msg(const char *s, va_list p, const char* strerr)
{
	char *msg, *msg1;
	char stack_msg[80];
	int applet_len, strerr_len, msgeol_len, used;

	if (!logmode)
		return;

	if (!s) /* nomsg[_and_die] uses NULL fmt */
		s = ""; /* some libc don't like printf(NULL) */

	applet_len = strlen(applet_name) + 2; /* "applet: " */
	strerr_len = strerr ? strlen(strerr) : 0;
	msgeol_len = strlen(msg_eol);

	/* This costs ~90 bytes of code, but avoids costly
	 * malloc()[in vasprintf]+realloc()+memmove()+free() in 99% of cases.
	 * ~40% speedup.
	 */
	if ((int)sizeof(stack_msg) - applet_len > 0) {
		va_list p2;

		/* It is not portable to use va_list twice, need to va_copy it */
		va_copy(p2, p);
		used = vsnprintf(stack_msg + applet_len, (int)sizeof(stack_msg) - applet_len, s, p2);
		va_end(p2);
		msg = stack_msg;
		used += applet_len;
		if (used < (int)sizeof(stack_msg) - 3 - msgeol_len - strerr_len)
			goto add_pfx_and_sfx;
	}

	used = vasprintf(&msg, s, p);
	if (used < 0)
		return;

	/* This is ugly and costs +60 bytes compared to multiple
	 * fprintf's, but is guaranteed to do a single write.
	 * This is needed for e.g. httpd logging, when multiple
	 * children can produce log messages simultaneously. */

	/* can't use xrealloc: it calls error_msg on failure,
	 * that may result in a recursion */
	/* +3 is for ": " before strerr and for terminating NUL */
	msg1 = realloc(msg, applet_len + used + strerr_len + msgeol_len + 3);
	if (!msg1) {
		msg[used++] = '\n'; /* overwrites NUL */
		applet_len = 0;
	} else {
		msg = msg1;
		/* TODO: maybe use writev instead of memmoving? Need full_writev? */
		memmove(msg + applet_len, msg, used);
		used += applet_len;
 add_pfx_and_sfx:
		strcpy(msg, applet_name);
		msg[applet_len - 2] = ':';
		msg[applet_len - 1] = ' ';
		if (strerr) {
			if (s[0]) { /* not perror_nomsg? */
				msg[used++] = ':';
				msg[used++] = ' ';
			}
			strcpy(&msg[used], strerr);
			used += strerr_len;
		}
		strcpy(&msg[used], msg_eol);
		used += msgeol_len;
	}

	if (logmode & LOGMODE_STDIO) {
		fflush_all();
		full_write(STDERR_FILENO, msg, used);
	}
#if ENABLE_FEATURE_SYSLOG
	if (logmode & LOGMODE_SYSLOG) {
		syslog(syslog_level, "%s", msg + applet_len);
	}
#endif
	if (msg != stack_msg)
		free(msg);
}
static void FAST_FUNC bb_error_msg(const char *s, ...)
{
	va_list p;

	va_start(p, s);
	bb_verror_msg(s, p, NULL);
	va_end(p);
}
static void FAST_FUNC bb_error_msg_and_die(const char *s, ...)
{
	va_list p;

	va_start(p, s);
	bb_verror_msg(s, p, NULL);
	va_end(p);
	exit(EXIT_FAILURE);
}

// Die if we can't allocate size bytes of memory.
static void* FAST_FUNC xmalloc(size_t size)
{
	void *ptr = malloc(size);
	if (ptr == NULL && size != 0)
		bb_error_msg_and_die(bb_msg_memory_exhausted);
	return ptr;
}
// Die if we can't allocate and zero size bytes of memory.
static void* FAST_FUNC xzalloc(size_t size)
{
	void *ptr = xmalloc(size);
	memset(ptr, 0, size);
	return ptr;
}
// Die if we can't resize previously allocated memory.  (This returns a pointer
// to the new memory, which may or may not be the same as the old memory.
// It'll copy the contents to a new chunk and free the old one if necessary.)
static void* FAST_FUNC xrealloc(void *ptr, size_t size)
{
	ptr = realloc(ptr, size);
	if (ptr == NULL && size != 0)
		bb_error_msg_and_die(bb_msg_memory_exhausted);
	return ptr;
}

static char* FAST_FUNC bb_get_chunk_from_file(FILE *file, int *end)
{
	int ch;
	unsigned idx = 0;
	char *linebuf = NULL;

	while ((ch = getc(file)) != EOF) {
		/* grow the line buffer as necessary */
		if (!(idx & 0xff))
			linebuf = xrealloc(linebuf, idx + 0x100);
		linebuf[idx++] = (char) ch;
		if (ch == '\0')
			break;
		if (end && ch == '\n')
			break;
	}
	if (end)
		*end = idx;
	if (linebuf) {
		// huh, does fgets discard prior data on error like this?
		// I don't think so....
		//if (ferror(file)) {
		//	free(linebuf);
		//	return NULL;
		//}
		linebuf = xrealloc(linebuf, idx + 1);
		linebuf[idx] = '\0';
	}
	return linebuf;
}
/* Get line.  Remove trailing \n */
static char* FAST_FUNC xmalloc_fgetline(FILE *file)
{
	int i;
	char *c = bb_get_chunk_from_file(file, &i);

	if (i && c[--i] == '\n')
		c[i] = '\0';

	return c;
}

/* Like strcpy but can copy overlapping strings. */
static void FAST_FUNC overlapping_strcpy(char *dst, const char *src)
{
	/* Cheap optimization for dst == src case -
	 * better to have it here than in many callers.
	 */
	if (dst != src) {
		while ((*dst = *src) != '\0') {
			dst++;
			src++;
		}
	}
}
static char* FAST_FUNC skip_whitespace(const char *s)
{
	/* In POSIX/C locale (the only locale we care about: do we REALLY want
	 * to allow Unicode whitespace in, say, .conf files? nuts!)
	 * isspace is only these chars: "\t\n\v\f\r" and space.
	 * "\t\n\v\f\r" happen to have ASCII codes 9,10,11,12,13.
	 * Use that.
	 */
	while (*s == ' ' || (unsigned char)(*s - 9) <= (13 - 9))
		s++;

	return (char *) s;
}
static const char* FAST_FUNC nth_string(const char *strings, int n)
{
	while (n) {
		n--;
		strings += strlen(strings) + 1;
	}
	return strings;
}

#define xrealloc_vector(vector, shift, idx) \
	xrealloc_vector_helper((vector), (sizeof((vector)[0]) << 8) + (shift), (idx))
static void* FAST_FUNC xrealloc_vector_helper(void *vector, unsigned sizeof_and_shift, int idx)
{
	int mask = 1 << (uint8_t)sizeof_and_shift;

	if (!(idx & (mask - 1))) {
		sizeof_and_shift >>= 8; /* sizeof(vector[0]) */
		vector = xrealloc(vector, sizeof_and_shift * (idx + mask + 1));
		memset((char*)vector + (sizeof_and_shift * idx), 0, sizeof_and_shift * (mask + 1));
	}
	return vector;
}

static void FAST_FUNC close_on_exec_on(int fd)
{
	fcntl(fd, F_SETFD, FD_CLOEXEC);
}

char *getlogin (void)
{
	struct passwd* pw = getpwuid(getuid());
	if (pw == NULL)
		return "";
	return pw->pw_name;
}

int getlogin_r (char *__name, size_t __name_len)
{
	char *logname = getlogin();
	if (__name == NULL || __name_len <= strlen(logname))
	{
		errno = ERANGE;
		return -1;
	}
	strcpy(__name, logname);
	return 0;
}
//#include "libbb.h"

struct const_passdb {
	const char *filename;
	char def[7 + 2*ENABLE_USE_BB_SHADOW];
	uint8_t off[7 + 2*ENABLE_USE_BB_SHADOW];
	uint8_t numfields;
	uint8_t size_of;
};
struct passdb {
	const char *filename;
	char def[7 + 2*ENABLE_USE_BB_SHADOW];
	uint8_t off[7 + 2*ENABLE_USE_BB_SHADOW];
	uint8_t numfields;
	uint8_t size_of;
	FILE *fp;
	char *malloced;
};
/* Note: for shadow db, def[] will not contain terminating NUL,
 * but convert_to_struct() logic detects def[] end by "less than SP?",
 * not by "is it NUL?" condition; and off[0] happens to be zero
 * for every db anyway, so there _is_ in fact a terminating NUL there.
 */

/* S = string not empty, s = string maybe empty,
 * I = uid,gid, l = long maybe empty, m = members,
 * r = reserved
 */
#define PW_DEF "SsIIsss"
#define GR_DEF "SsIm"
#define SP_DEF "Ssllllllr"

static const struct const_passdb const_pw_db = {
	_PATH_PASSWD, PW_DEF,
	{
		offsetof(struct passwd, pw_name),       /* 0 S */
		offsetof(struct passwd, pw_passwd),     /* 1 s */
		offsetof(struct passwd, pw_uid),        /* 2 I */
		offsetof(struct passwd, pw_gid),        /* 3 I */
		offsetof(struct passwd, pw_gecos),      /* 4 s */
		offsetof(struct passwd, pw_dir),        /* 5 s */
		offsetof(struct passwd, pw_shell)       /* 6 s */
	},
	sizeof(PW_DEF)-1, sizeof(struct passwd)
};
static const struct const_passdb const_gr_db = {
	_PATH_GROUP, GR_DEF,
	{
		offsetof(struct group, gr_name),        /* 0 S */
		offsetof(struct group, gr_passwd),      /* 1 s */
		offsetof(struct group, gr_gid),         /* 2 I */
		offsetof(struct group, gr_mem)          /* 3 m (char **) */
	},
	sizeof(GR_DEF)-1, sizeof(struct group)
};
#if ENABLE_USE_BB_SHADOW
static const struct const_passdb const_sp_db = {
	_PATH_SHADOW, SP_DEF,
	{
		offsetof(struct spwd, sp_namp),         /* 0 S Login name */
		offsetof(struct spwd, sp_pwdp),         /* 1 s Encrypted password */
		offsetof(struct spwd, sp_lstchg),       /* 2 l */
		offsetof(struct spwd, sp_min),          /* 3 l */
		offsetof(struct spwd, sp_max),          /* 4 l */
		offsetof(struct spwd, sp_warn),         /* 5 l */
		offsetof(struct spwd, sp_inact),        /* 6 l */
		offsetof(struct spwd, sp_expire),       /* 7 l */
		offsetof(struct spwd, sp_flag)          /* 8 r Reserved */
	},
	sizeof(SP_DEF)-1, sizeof(struct spwd)
};
#endif

/* We avoid having big global data. */
struct statics {
	/* We use same buffer (db[0].malloced) for getpwuid and getpwnam.
	 * Manpage says:
	 * "The return value may point to a static area, and may be overwritten
	 * by subsequent calls to getpwent(), getpwnam(), or getpwuid()."
	 */
	struct passdb db[2 + ENABLE_USE_BB_SHADOW];
	char *tokenize_end;
	unsigned string_size;
};

static struct statics *ptr_to_statics;
#define S     (*ptr_to_statics)
#define has_S (ptr_to_statics)

#if ENABLE_FEATURE_CLEAN_UP
static void free_static(void)
{
	free(S.db[0].malloced);
	free(S.db[1].malloced);
# if ENABLE_USE_BB_SHADOW
	free(S.db[2].malloced);
# endif
	free(ptr_to_statics);
}
#endif

static struct statics *get_S(void)
{
	if (!ptr_to_statics) {
		ptr_to_statics = xzalloc(sizeof(S));
		memcpy(&S.db[0], &const_pw_db, sizeof(const_pw_db));
		memcpy(&S.db[1], &const_gr_db, sizeof(const_gr_db));
#if ENABLE_USE_BB_SHADOW
		memcpy(&S.db[2], &const_sp_db, sizeof(const_sp_db));
#endif
#if ENABLE_FEATURE_CLEAN_UP
		atexit(free_static);
#endif
	}
	return ptr_to_statics;
}

/* Internal functions */

/* Divide the passwd/group/shadow record in fields
 * by substituting the given delimeter
 * e.g. ':' or ',' with '\0'.
 * Returns the number of fields found.
 * Strips leading and trailing whitespace in fields.
 */
static int tokenize(char *buffer, int ch)
{
	char *p = buffer;
	char *s = p;
	int num_fields = 0;

	for (;;) {
		if (isblank(*s)) {
			overlapping_strcpy(s, skip_whitespace(s));
		}
		if (*p == ch || *p == '\0') {
			char *end = p;
			while (p != s && isblank(p[-1]))
				p--;
			if (p != end)
				overlapping_strcpy(p, end);
			num_fields++;
			if (*end == '\0') {
				S.tokenize_end = p + 1;
				return num_fields;
			}
			*p = '\0';
			s = p + 1;
		}
		p++;
	}
}

/* Returns !NULL on success and matching line broken up in fields by '\0' in buf.
 * We require the expected number of fields to be found.
 */
static char *parse_common(FILE *fp, struct passdb *db,
		const char *key, int field_pos)
{
	char *buf;

	while ((buf = xmalloc_fgetline(fp)) != NULL) {
		/* Skip empty lines, comment lines */
		if (buf[0] == '\0' || buf[0] == '#')
			goto free_and_next;
		if (tokenize(buf, ':') != db->numfields) {
			/* number of fields is wrong */
			bb_error_msg("%s: bad record", db->filename);
			goto free_and_next;
		}

		if (field_pos == -1) {
			/* no key specified: sequential read, return a record */
			break;
		}
		if (strcmp(key, nth_string(buf, field_pos)) == 0) {
			/* record found */
			break;
		}
 free_and_next:
		free(buf);
	}

	S.string_size = S.tokenize_end - buf;
/*
 * Ugly hack: group db requires additional buffer space
 * for members[] array. If there is only one group, we need space
 * for 3 pointers: alignment padding, group name, NULL.
 * +1 for every additional group.
 */
	if (buf && db->numfields == sizeof(GR_DEF)-1) { /* if we read group file... */
		int cnt = 3;
		char *p = buf;
		while (p < S.tokenize_end)
			if (*p++ == ',')
				cnt++;
		S.string_size += cnt * sizeof(char*);
//bb_error_msg("+%d words = %u key:%s buf:'%s'", cnt, S.string_size, key, buf);
		buf = xrealloc(buf, S.string_size);
	}

	return buf;
}

static char *parse_file(struct passdb *db,
		const char *key, int field_pos)
{
	char *buf = NULL;
	FILE *fp = fopen_for_read(db->filename);

	if (fp) {
		buf = parse_common(fp, db, key, field_pos);
		fclose(fp);
	}
	return buf;
}

/* Convert passwd/group/shadow file record in buffer to a struct */
static void *convert_to_struct(struct passdb *db,
		char *buffer, void *result)
{
	const char *def = db->def;
	const uint8_t *off = db->off;

	/* reset errno = 0 */
	errno = 0;
	/* For consistency, zero out all fields */
	memset(result, 0, db->size_of);

	for (;;) {
		void *member = (char*)result + (*off++);

		if ((*def | 0x20) == 's') { /* s or S */
			*(char **)member = (char*)buffer;
			if (!buffer[0] && (*def == 'S')) {
				errno = EINVAL;
			}
		}
		if (*def == 'I') {
#if defined(_NEWLIB_VERSION)
			*(unsigned short *)member = strtoul(buffer, NULL, 10);
#else
			*(int *)member = strtoul(buffer, NULL, 10);
#endif
		}
#if ENABLE_USE_BB_SHADOW
		if (*def == 'l') {
			long n = -1;
			if (buffer[0])
				n = strtol(buffer, NULL, 10);
			*(long *)member = n;
		}
#endif
		if (*def == 'm') {
			char **members;
			int i = tokenize(buffer, ',');

			/* Store members[] after buffer's end.
			 * This is safe ONLY because there is a hack
			 * in parse_common() which allocates additional space
			 * at the end of malloced buffer!
			 */
			members = (char **)
				( ((intptr_t)S.tokenize_end + sizeof(members[0]))
				& -(intptr_t)sizeof(members[0])
				);
			((struct group *)result)->gr_mem = members;
			while (--i >= 0) {
				if (buffer[0]) {
					*members++ = buffer;
					// bb_error_msg("member[]='%s'", buffer);
				}
				buffer += strlen(buffer) + 1;
			}
			*members = NULL;
		}
		/* def "r" does nothing */

		def++;
		if ((unsigned char)*def <= (unsigned char)' ')
			break;
		buffer += strlen(buffer) + 1;
	}

	if (errno)
		result = NULL;
	return result;
}

static int massage_data_for_r_func(struct passdb *db,
		char *buffer, size_t buflen,
		void **result,
		char *buf)
{
	void *result_buf = *result;
	*result = NULL;
	if (buf) {
		if (S.string_size > buflen) {
			errno = ERANGE;
		} else {
			memcpy(buffer, buf, S.string_size);
			*result = convert_to_struct(db, buffer, result_buf);
		}
		free(buf);
	}
	/* "The reentrant functions return zero on success.
	 * In case of error, an error number is returned."
	 * NB: not finding the record is also a "success" here:
	 */
	return errno;
}

static void* massage_data_for_non_r_func(struct passdb *db, char *buf)
{
	if (!buf)
		return NULL;

	free(db->malloced);
	/* We enlarge buf and move string data up, freeing space
	 * for struct passwd/group/spwd at the beginning. This way,
	 * entire result of getXXnam is in a single malloced block.
	 * This enables easy creation of xmalloc_getpwnam() API.
	 */
	db->malloced = buf = xrealloc(buf, db->size_of + S.string_size);
	memmove(buf + db->size_of, buf, S.string_size);
	return convert_to_struct(db, buf + db->size_of, buf);
}

/****** getXXnam/id_r */

static int FAST_FUNC getXXnam_r(const char *name, uintptr_t db_and_field_pos,
		char *buffer, size_t buflen,
		void *result)
{
	char *buf;
	struct passdb *db = &get_S()->db[db_and_field_pos >> 2];

	buf = parse_file(db, name, 0 /*db_and_field_pos & 3*/);
	/* "db_and_field_pos & 3" is commented out since so far we don't implement
	 * getXXXid_r() functions which would use that to pass 2 here */

	return massage_data_for_r_func(db, buffer, buflen, result, buf);
}

int FAST_FUNC getpwnam_r(const char *name, struct passwd *struct_buf,
		char *buffer, size_t buflen,
		struct passwd **result)
{
	/* Why the "store buffer address in result" trick?
	 * This way, getXXnam_r has the same ABI signature as getpwnam_r,
	 * hopefully compiler can optimize tail call better in this case.
	 */
	*result = struct_buf;
	return getXXnam_r(name, (0 << 2) + 0, buffer, buflen, result);
}
#if ENABLE_USE_BB_SHADOW
int FAST_FUNC getspnam_r(const char *name, struct spwd *struct_buf, char *buffer, size_t buflen,
		struct spwd **result)
{
	*result = struct_buf;
	return getXXnam_r(name, (2 << 2) + 0, buffer, buflen, result);
}
#endif

#ifdef UNUSED
/****** getXXent_r */

static int FAST_FUNC getXXent_r(uintptr_t db_idx, char *buffer, size_t buflen,
		void *result)
{
	char *buf;
	struct passdb *db = &get_S()->db[db_idx];

	if (!db->fp) {
		db->fp = fopen_for_read(db->filename);
		if (!db->fp) {
			return errno;
		}
		close_on_exec_on(fileno(db->fp));
	}

	buf = parse_common(db->fp, db, /*no search key:*/ NULL, -1);
	if (!buf && !errno)
		errno = ENOENT;
	return massage_data_for_r_func(db, buffer, buflen, result, buf);
}

int FAST_FUNC getpwent_r(struct passwd *struct_buf, char *buffer, size_t buflen,
		struct passwd **result)
{
	*result = struct_buf;
	return getXXent_r(0, buffer, buflen, result);
}
#endif

/****** getXXent */

static void* FAST_FUNC getXXent(uintptr_t db_idx)
{
	char *buf;
	struct passdb *db = &get_S()->db[db_idx];

	if (!db->fp) {
		db->fp = fopen_for_read(db->filename);
		if (!db->fp) {
			return NULL;
		}
		close_on_exec_on(fileno(db->fp));
	}

	buf = parse_common(db->fp, db, /*no search key:*/ NULL, -1);
	return massage_data_for_non_r_func(db, buf);
}

struct passwd* FAST_FUNC getpwent(void)
{
	return getXXent(0);
}

/****** getXXnam/id */

static void* FAST_FUNC getXXnam(const char *name, unsigned db_and_field_pos)
{
	char *buf;
	struct passdb *db = &get_S()->db[db_and_field_pos >> 2];

	buf = parse_file(db, name, db_and_field_pos & 3);
	return massage_data_for_non_r_func(db, buf);
}

struct passwd* FAST_FUNC getpwnam(const char *name)
{
	return getXXnam(name, (0 << 2) + 0);
}
struct group* FAST_FUNC getgrnam(const char *name)
{
	return getXXnam(name, (1 << 2) + 0);
}
struct passwd* FAST_FUNC getpwuid(uid_t id)
{
	return getXXnam(utoa(id,local_buf,10), (0 << 2) + 2);
}
struct group* FAST_FUNC getgrgid(gid_t id)
{
	return getXXnam(utoa(id,local_buf,10), (1 << 2) + 2);
}

/****** end/setXXend */

void FAST_FUNC endpwent(void)
{
	if (has_S && S.db[0].fp) {
		fclose(S.db[0].fp);
		S.db[0].fp = NULL;
	}
}
void FAST_FUNC setpwent(void)
{
	if (has_S && S.db[0].fp) {
		rewind(S.db[0].fp);
	}
}
void FAST_FUNC endgrent(void)
{
	if (has_S && S.db[1].fp) {
		fclose(S.db[1].fp);
		S.db[1].fp = NULL;
	}
}

/****** initgroups and getgrouplist */

static gid_t* FAST_FUNC getgrouplist_internal(int *ngroups_ptr,
		const char *user, gid_t gid)
{
	FILE *fp;
	gid_t *group_list;
	int ngroups;

	/* We alloc space for 8 gids at a time. */
	group_list = xzalloc(8 * sizeof(group_list[0]));
	group_list[0] = gid;
	ngroups = 1;

	fp = fopen_for_read(_PATH_GROUP);
	if (fp) {
		struct passdb *db = &get_S()->db[1];
		char *buf;
		while ((buf = parse_common(fp, db, NULL, -1)) != NULL) {
			char **m;
			struct group group;
			if (!convert_to_struct(db, buf, &group))
				goto next;
			if (group.gr_gid == gid)
				goto next;
			for (m = group.gr_mem; *m; m++) {
				if (strcmp(*m, user) != 0)
					continue;
				group_list = xrealloc_vector(group_list, /*8=2^3:*/ 3, ngroups);
				group_list[ngroups++] = group.gr_gid;
				goto next;
			}
 next:
			free(buf);
		}
		fclose(fp);
	}
	*ngroups_ptr = ngroups;
	return group_list;
}

int FAST_FUNC initgroups(const char *user, gid_t gid)
{
	int ngroups;
	gid_t *group_list = getgrouplist_internal(&ngroups, user, gid);

	ngroups = setgroups(ngroups, group_list);
	free(group_list);
	return ngroups;
}

int FAST_FUNC getgrouplist(const char *user, gid_t gid, gid_t *groups, int *ngroups)
{
	int ngroups_old = *ngroups;
	gid_t *group_list = getgrouplist_internal(ngroups, user, gid);

	if (*ngroups <= ngroups_old) {
		ngroups_old = *ngroups;
		memcpy(groups, group_list, ngroups_old * sizeof(groups[0]));
	} else {
		ngroups_old = -1;
	}
	free(group_list);
	return ngroups_old;
}
