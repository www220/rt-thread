/* RT-Thread config file */
#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

/* RT_NAME_MAX*/
#define RT_NAME_MAX	   16

/* RT_ALIGN_SIZE*/
#define RT_ALIGN_SIZE	4

/* PRIORITY_MAX */
#define RT_THREAD_PRIORITY_MAX	32

/* Tick per Second */
#define RT_TICK_PER_SECOND	1000

/* SECTION: RT_DEBUG */
/* Thread Debug */
#define RT_DEBUG

#define RT_USING_OVERFLOW_CHECK
#define RT_USING_CPU_FFS

/* Using Hook */
#define RT_USING_HOOK

#define IDLE_THREAD_STACK_SIZE     512
#define RT_USING_INTERRUPT_INFO

/* Using Software Timer */
/* #define RT_USING_TIMER_SOFT */
#define RT_TIMER_THREAD_PRIO		4
#define RT_TIMER_THREAD_STACK_SIZE	512
#define RT_TIMER_TICK_PER_SECOND	10

/* SECTION: IPC */
/* Using Semaphore*/
#define RT_USING_SEMAPHORE

/* Using Mutex */
#define RT_USING_MUTEX

/* Using Event */
#define RT_USING_EVENT

/* Using MailBox */
#define RT_USING_MAILBOX

/* Using Message Queue */
#define RT_USING_MESSAGEQUEUE

/* SECTION: Memory Management */
/* Using Memory Pool Management*/
#define RT_USING_MEMPOOL

/* Using Dynamic Heap Management */
#define RT_USING_HEAP
#define RT_USING_MEMHEAP
//#define RT_USING_MEMDUMP

/* Using Small MM */
//#define RT_USING_SMALL_MEM
/* Using SLAB Allocator */
#define RT_USING_SLAB
#define RT_USING_MODULE
#define RT_USING_PROCESS
#define RT_SERIAL_RB_BUFSZ 4096

/* Using Libc */
#define RT_USING_LIBC

/* SECTION: Device System */
/* Using Device System */
#define RT_USING_DEVICE
#define RT_USING_DEVICE_IPC
#define RT_USING_SERIAL
//#define RT_USING_SPI
#define RT_USING_RTC
//#define RT_MTD_NAND_DEBUG
#define RT_USING_MTD_NAND
//#define RT_MMCSD_DBG
#define RT_USING_SDIO
#define RT_MMCSD_STACK_SIZE 4096
//#define RT_USING_USB_HOSTU

/* SECTION: Console options */
#define RT_USING_CONSOLE
/* the buffer size of console*/
#define RT_CONSOLEBUF_SIZE	4096

/* SECTION: finsh, a C-Express shell */
#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT
//#define FINSH_USING_MSH_ONLY
#define RT_USING_RYM

/* Using symbol table */
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_STACK_SIZE	131072

/* SECTION: device filesystem */
#define RT_USING_DFS
#define DFS_USING_WORKDIR

#define RT_USING_DFS_ELMFAT
#define RT_DFS_ELM_DRIVES			2
#define RT_DFS_ELM_REENTRANT
//#define RT_DFS_ELM_USE_ERASE
#define RT_DFS_ELM_CODE_PAGE		1252
#define RT_DFS_ELM_USE_LFN			3
#define RT_DFS_ELM_MAX_LFN			255
#define RT_DFS_ELM_MAX_SECTOR_SIZE  512

#define RT_USING_DFS_YAFFS2

#define RT_USING_DFS_ROMFS
#define DFS_ROMFS_ROOT
#define RT_USING_DFS_DEVFS

/* the max number of mounted filesystem */
#define DFS_FILESYSTEMS_MAX			10
#define DFS_FILESYSTEM_TYPES_MAX	10
/* the max number of opened files 		*/
#define DFS_FD_MAX					32

#define rtt_LogDebug	7
#define rtt_LogInfo		6
#define rtt_LogNotice	5
#define rtt_LogWarn		4
#define rtt_LogErr		3
#define rtt_LogCrit		2
#define rtt_LogAlert	1
#define rtt_LogEmerg	0

#define rttTempFileDir "/tmp"
#define rttLogFileDir "/tmp/log"
#define rttCfgFileDir "/etc"
#define rttLibFileDir "/usr"

/* SECTION: lwip, a lighwight TCP/IP protocol stack */
//#define RT_USING_LWIP_HEAD
//#define RT_USING_LWIP
/* LwIP uses RT-Thread Memory Management */
#define RT_LWIP_USING_RT_MEM
/* Enable ICMP protocol*/
#define RT_LWIP_ICMP
/* Enable UDP protocol*/
#define RT_LWIP_UDP
/* Enable TCP protocol*/
#define RT_LWIP_TCP
/* Enable DNS */
#define RT_LWIP_DNS

#define MEMP_NUM_NETCONN 64
#define SO_REUSE 1
#define LWIP_RAW 1
#define MPPE_SUPPORT 1
#define LWIP_TCP_KEEPALIVE 1
#define SA_FAMILY_T_DEFINED 1
#define LWIP_IPV6 1

#define RT_LWIP_DHCP
#define RT_LWIP_PPP
#define RT_LWIP_PPPOS
#define RT_LWIP_PPPOE
#define PPPOL2TP_SUPPORT 1
#define PPP_THREAD_NAME "ppp_inp"
#define PPP_THREAD_STACKSIZE 8192
#define PPP_THREAD_PRIO 20

/* ip address of target*/
#define RT_LWIP_IPADDR0	192
#define RT_LWIP_IPADDR1	168
#define RT_LWIP_IPADDR2	99
#define RT_LWIP_IPADDR3	22

/* gateway address of target*/
#define RT_LWIP_GWADDR0	192
#define RT_LWIP_GWADDR1	168
#define RT_LWIP_GWADDR2	99
#define RT_LWIP_GWADDR3	1

/* mask address of target*/
#define RT_LWIP_MSKADDR0	255
#define RT_LWIP_MSKADDR1	255
#define RT_LWIP_MSKADDR2	255
#define RT_LWIP_MSKADDR3	0

/* tcp thread options */
#define RT_LWIP_TCPTHREAD_PRIORITY		12
#define RT_LWIP_TCPTHREAD_MBOX_SIZE		16
#define RT_LWIP_TCPTHREAD_STACKSIZE		8192

/* ethernet if thread options */
#define RT_LWIP_ETHTHREAD_PRIORITY		15
#define RT_LWIP_ETHTHREAD_MBOX_SIZE		16
#define RT_LWIP_ETHTHREAD_STACKSIZE		4096

/* TCP sender buffer space */
#define RT_LWIP_TCP_SND_BUF	8192
/* TCP receive window. */
#define RT_LWIP_TCP_WND		8192

#define CHECKSUM_CHECK_TCP              0
#define CHECKSUM_CHECK_IP               0
#define CHECKSUM_CHECK_UDP              0

#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1

#endif
