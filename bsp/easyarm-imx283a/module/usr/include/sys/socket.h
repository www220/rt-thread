/**
 * @file
 * This file is a posix wrapper for lwip/sockets.h.
 */

/*
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 */

#ifndef	_SYS_SOCKET_H
#define	_SYS_SOCKET_H	1

#include <sys/un.h>
#include "lwip/sockets.h"

#define SOCK_RDM        4
#define SOCK_SEQPACKET  5
#define AF_UNIX		1

# define NI_NUMERICHOST	1	/* Don't try to look up hostname.  */
# define NI_NUMERICSERV 2	/* Don't convert port number to name.  */
# define NI_NOFQDN	4	/* Only return nodename portion.  */
# define NI_NAMEREQD	8	/* Don't return numeric addresses.  */
# define NI_DGRAM	16	/* Look up UDP service rather than TCP.  */
# ifdef __USE_GNU
#  define NI_IDN	32	/* Convert name from IDN format.  */
#  define NI_IDN_ALLOW_UNASSIGNED 64 /* Don't reject unassigned Unicode
					code points.  */
#  define NI_IDN_USE_STD3_ASCII_RULES 128 /* Validate strings according to
					     STD3 rules.  */
# endif

/* Description of data base entry for a single network.  NOTE: here a
   poor assumption is made.  The network number is expected to fit
   into an unsigned long int variable.  */
struct netent
{
  char *n_name;			/* Official name of network.  */
  char **n_aliases;		/* Alias list.  */
  int n_addrtype;		/* Net address type.  */
  uint32_t n_net;		/* Network number.  */
};

/* Description of data base entry for a single service.  */
struct servent
{
  char *s_name;			/* Official service name.  */
  char **s_aliases;		/* Alias list.  */
  int s_port;			/* Port number.  */
  char *s_proto;		/* Protocol to use.  */
};

/* Return entry from network data base for network with NAME and
   protocol PROTO.

   This function is a possible cancellation point and therefore not
   marked with __THROW.  */
struct servent *getservbyname (__const char *__name,
				      __const char *__proto);

# define __SOCKADDR_ARG		struct sockaddr *__restrict
# define __CONST_SOCKADDR_ARG	__const struct sockaddr *

/* Put the local address of FD into *ADDR and its length in *LEN.  */
int getsockname (int __fd, __SOCKADDR_ARG __addr,
			socklen_t *__restrict __len) __THROW;

/* Put the address of the peer connected to socket FD into *ADDR
   (which is *LEN bytes long), and its actual length into *LEN.  */
int getpeername (int __fd, __SOCKADDR_ARG __addr,
			socklen_t *__restrict __len) __THROW;


__const char *hstrerror (int __err_num) __THROW;
struct netent *getnetbyaddr (uint32_t __net, int __type);

int getnameinfo (__const struct sockaddr *__restrict __sa,
			socklen_t __salen, char *__restrict __host,
			socklen_t __hostlen, char *__restrict __serv,
			socklen_t __servlen, unsigned int __flags);

#endif /* sys/socket.h */

