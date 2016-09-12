/* libc/sys/linux/include/getopt.h - Extended command line parsing */

/* Written 2000 by Werner Almesberger */


#ifndef _NEWLIB_GETOPT_H
#define _NEWLIB_GETOPT_H

#include <unistd.h>
#define optarg optarg_l
#define optind optind_l
#define opterr opterr_l
#define optopt optopt_l
extern char *optarg;
extern int optind, opterr, optopt;
#define getopt getopt_l
#define getopt_long getopt_long_l
#define getopt_long_only getopt_long_only_l
int	getopt(int, char * const [], const char *);

enum { NO_ARG, REQUIRED_ARG, OPTIONAL_ARG };
/* Define glibc names as well for compatibility.  */
#define no_argument NO_ARG
#define required_argument REQUIRED_ARG
#define optional_argument OPTIONAL_ARG

struct option {
    const char *name;
    int has_arg;
    int *flag;
    int val;
};

int getopt_long(int argc,char *const argv[],const char *optstring,
  const struct option *longopts,int *longindex);

int getopt_long_only(int argc,char *const argv[],const char *optstring,
  const struct option *longopts,int *longindex);

#endif
