#ifndef _SYS_UTIME_H
#define _SYS_UTIME_H

struct utimbuf {
    time_t actime;
    time_t modtime;
};

#endif