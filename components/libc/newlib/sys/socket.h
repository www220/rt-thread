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

static inline struct servent *getservbyname(const char *name, const char *proto) { return NULL; }
static inline struct servent *getservbyport(int port, const char *proto) { return NULL; }

#include <lwip/sockets.h>

#ifdef __cplusplus
}
#endif

#endif
