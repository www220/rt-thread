#ifndef	_USERDEF_STDLIB_H_
#define	_USERDEF_STDLIB_H_ 1

#define utoa utoa_l
#define itoa itoa_l
#include <sys/stdlib.h>
#undef utoa
#undef itoa
#include <nodeffunc.h>

#endif //_USERDEF_STDLIB_H_
