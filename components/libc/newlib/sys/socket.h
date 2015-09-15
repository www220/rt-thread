/*
 * File      : socket.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-02-17     Bernard      First version
 */

#ifndef SOCKET_H__
#define SOCKET_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "lwip/opt.h"

struct  servent {
        char    * s_name;           /* official service name */
        char    ** s_aliases;  /* alias list */
        int   s_port;                 /* port # */
        char    * s_proto;          /* protocol to use */
};

struct in_addr6 {
  u32_t s6_addr[4];
};

struct sockaddr_in6 {
  u8_t sin_len;
  u8_t sin_family;
  u16_t sin6_port;
  u32_t sin6_flowinfo;
  struct in_addr6 sin6_addr;
  u32_t sin6_scope_id;
};

struct sockaddr_storage {
  u8_t sin_len;
  u8_t sin_family;
  u8_t padding[128];
};

#define AI_PASSIVE                  0x00000001  // Socket address will be used in bind() call
#define AI_CANONNAME                0x00000002  // Return canonical name in first ai_canonname
#define AI_NUMERICHOST              0x00000004  // Nodename must be a numeric address string
#define AI_NUMERICSERV              0x00000008  // Servicename must be a numeric port number

#define AI_ALL                      0x00000100  // Query both IP6 and IP4 with AI_V4MAPPED
#define AI_ADDRCONFIG               0x00000400  // Resolution only if global address configured
#define AI_V4MAPPED                 0x00000800  // On v6 failure, query v4 and convert to V4MAPPED format

#define AI_NON_AUTHORITATIVE        0x00004000  // LUP_NON_AUTHORITATIVE
#define AI_SECURE                   0x00008000  // LUP_SECURE
#define AI_RETURN_PREFERRED_NAMES   0x00010000  // LUP_RETURN_PREFERRED_NAMES

#define AI_FQDN                     0x00020000  // Return the FQDN in ai_canonname
#define AI_FILESERVER               0x00040000  // Resolving fileserver name resolution

static inline struct servent *getservbyname(const char *name, const char *proto) { return NULL; }
static inline struct servent *getservbyport(int port, const char *proto) { return NULL; }

#include <lwip/sockets.h>

#ifdef __cplusplus
}
#endif

#endif
