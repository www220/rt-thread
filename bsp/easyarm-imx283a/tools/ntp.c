/*
*
* mini-ntpclient: a stripped-down & simplified ntpclient -- tofu
*
*/

/*
*
* ntpclient.c - NTP client
*
* Copyright 1997, 1999, 2000, 2003  Larry Doolittle  <larry@doolittle.boa.org>
* Last hack: July 5, 2003
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License (Version 2,
*  June 1991) as published by the Free Software Foundation.  At the
*  time of writing, that license was published by the FSF with the URL
*  http://www.gnu.org/copyleft/gpl.html, and is incorporated herein by
*  reference.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  Possible future improvements:
*      - Double check that the originate timestamp in the received packet
*        corresponds to what we sent.
*      - Verify that the return packet came from the host we think
*        we're talking to.  Not necessarily useful since UDP packets
*        are so easy to forge.
*      - Write more documentation  :-(
*
*  Compile with -D_PRECISION_SIOCGSTAMP if your machine really has it.
*  There are patches floating around to add this to Linux, but
*  usually you only get an answer to the nearest jiffy.
*  Hint for Linux hacker wannabes: look at the usage of get_fast_time()
*  in net/core/dev.c, and its definition in kernel/time.c .
*
*  If the compile gives you any flak, check below in the section
*  labelled "XXXX fixme - non-automatic build configuration".
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#define JAN_1970        0x83aa7e80      /* 2208988800 1970 - 1900 in seconds */
#define NTP_PORT (123)

/* How to multiply by 4294.967296 quickly (and not quite exactly)
* without using floating point or greater than 32-bit integers.
* If you want to fix the last 12 microseconds of error, add in
* (2911*(x))>>28)
*/
#define NTPFRAC(x) ( 4294*(x) + ( (1981*(x))>>11 ) )

/* The reverse of the above, needed if we want to set our microsecond
* clock (via settimeofday) based on the incoming time in NTP format.
* Basically exact.
*/
#define USEC(x) ( ( (x) >> 12 ) - 759 * ( ( ( (x) >> 10 ) + 32768 ) >> 16 ) )

/* Converts NTP delay and dispersion, apparently in seconds scaled
* by 65536, to microseconds.  RFC1305 states this time is in seconds,
* doesn't mention the scaling.
* Should somehow be the same as 1000000 * x / 65536
*/
#define sec2u(x) ( (x) * 15.2587890625 )

struct ntptime {
    unsigned int coarse;
    unsigned int fine;
};

void send_packet(int usd)
{
    uint32_t data[12];
    struct timeval now;
#define LI 0
#define VN 3
#define MODE 3
#define STRATUM 0
#define POLL 4
#define PREC -6

    memset((char *) data, 0, sizeof(data));
    data[0] = htonl (
        ( LI << 30 ) | ( VN << 27 ) | ( MODE << 24 ) |
        ( STRATUM << 16) | ( POLL << 8 ) | ( PREC & 0xff ) );
    data[1] = htonl(1<<16);  /* Root Delay (seconds) */
    data[2] = htonl(1<<16);  /* Root Dispersion (seconds) */
    gettimeofday(&now,NULL);
    data[10] = htonl(now.tv_sec + JAN_1970); /* Transmit Timestamp coarse */
    data[11] = htonl(NTPFRAC(now.tv_usec));  /* Transmit Timestamp fine   */
    send(usd,data,48,0);
}

void rfc1305(uint32_t *data)
{
    struct timeval tv_set;
    tv_set.tv_sec  = ntohl(((uint32_t *)data)[10]) - JAN_1970;
    tv_set.tv_usec = USEC(ntohl(((uint32_t *)data)[11]));
    if (settimeofday(&tv_set, NULL) < 0) {
        printf("settimeofday\n");
    }
}

int cmd_ntp(int argc, char *argv[])
{
    int usd;
    struct sockaddr_in sa;
    struct hostent *he;
    struct timeval tv;
    fd_set fds;
    int i;
    char *srv;
    char buf[256];

    if (argc <= 1) {
        printf("Usage: %s <server> [server [...]]\n", argv[0]);
        return 1;
    }

    for (i = 1; i < argc; ++i) {
        // ntp.c passes servers as one block of string, so we need to break it down
        strcpy(buf, argv[i]);
        srv = strtok(buf, " ");
        while (srv) {
            if ((usd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
                printf("socket error\n");
                return 1;
            }
            if ((he = gethostbyname(srv)) != NULL) {
                memset(&sa, 0, sizeof(sa));
                memcpy(&sa.sin_addr, he->h_addr_list[0], sizeof(sa.sin_addr));
                sa.sin_port = htons(NTP_PORT);
                sa.sin_family = AF_INET;

                printf("trying %s [%s]\n", argv[i], inet_ntoa(sa.sin_addr));
                if (connect(usd, (struct sockaddr*)&sa, sizeof(sa)) != -1) {
                    send_packet(usd);
                    tv.tv_sec = 3;
                    tv.tv_usec = 0;
                    FD_ZERO(&fds);
                    FD_SET(usd, &fds);
                    if (select(usd + 1, &fds, NULL, NULL, &tv) == 1) {
                        int len;
                        uint32_t packet[12];
                        len = recv(usd, packet, sizeof(packet), 0);
                        if (len == sizeof(packet)) {
                            time_t now;
                            rfc1305(packet);
                            closesocket(usd);
                            now = time(NULL);
                            printf("time updated %s",ctime(&now));
                            return 0;
                        }
                    }
                    else {
                        printf("recv\n");
                    }
                }
                else {
                    printf("connect\n");
                }
            }
            else {
                printf("gethostbyname\n");
            }
            closesocket(usd);
            srv = strtok(NULL, " ");
        }
    }
    return 1;
}

#if defined(FINSH_USING_MSH)
#include <finsh.h>

FINSH_FUNCTION_EXPORT_ALIAS(cmd_ntp, __cmd_ntp, SetLocalTime From NTP.)
#endif
