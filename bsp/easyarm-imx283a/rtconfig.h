/* RT-Thread config file */
#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

#ifdef _MSC_VER
#define malloc   rt_malloc
#define calloc   rt_calloc
#define realloc  rt_realloc
#define free     rt_free

#define strtok_r strtok_s

//从“type1”转换到“type2”可能丢失数据
#pragma warning(disable:4244)
//有符号/无符号不匹配
#pragma warning(disable:4018)
#endif

/* RT_NAME_MAX*/
#define RT_NAME_MAX	   8

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

/* Using Small MM */
//#define RT_USING_SMALL_MEM
/* Using SLAB Allocator */
#define RT_USING_SLAB

/* Using Libc */
#define RT_USING_LIBC

/* SECTION: Device System */
/* Using Device System */
#define RT_USING_DEVICE
#define RT_USING_DEVICE_IPC
#define RT_USING_SERIAL
//#define RT_USING_SPI
//#define RT_USING_RTC

/* SECTION: Console options */
#define RT_USING_CONSOLE
/* the buffer size of console*/
#define RT_CONSOLEBUF_SIZE	256

/* SECTION: finsh, a C-Express shell */
#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_USING_MSH_ONLY
/* Using symbol table */
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_STACK_SIZE	4096

/* SECTION: device filesystem */
#define RT_USING_DFS
#define DFS_USING_WORKDIR

//#define RT_USING_DFS_ELMFAT
#define RT_DFS_ELM_REENTRANT
#define RT_DFS_ELM_USE_ERASE
#define RT_DFS_ELM_CODE_PAGE		1252
#define RT_DFS_ELM_USE_LFN			3
#define RT_DFS_ELM_MAX_LFN			255
#define RT_DFS_ELM_MAX_SECTOR_SIZE  4096

//#define RT_USING_DFS_ROMFS
//#define DFS_ROMFS_ROOT
#define RT_USING_DFS_DEVFS

/* the max number of mounted filesystem */
#define DFS_FILESYSTEMS_MAX			4
/* the max number of opened files 		*/
#define DFS_FD_MAX					20

/* SECTION: sqlite, sql */
//#define RT_USING_SQLITE
#define rttTempFileDir "/spi/tmp"
extern int rttGetTempname(int nBuf, char *zBuf);

/* SECTION: lwip, a lighwight TCP/IP protocol stack */
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

/* ip address of target*/
#define RT_LWIP_IPADDR0	192
#define RT_LWIP_IPADDR1	168
#define RT_LWIP_IPADDR2	40
#define RT_LWIP_IPADDR3	22

/* gateway address of target*/
#define RT_LWIP_GWADDR0	192
#define RT_LWIP_GWADDR1	168
#define RT_LWIP_GWADDR2	40
#define RT_LWIP_GWADDR3	254

/* mask address of target*/
#define RT_LWIP_MSKADDR0	255
#define RT_LWIP_MSKADDR1	255
#define RT_LWIP_MSKADDR2	255
#define RT_LWIP_MSKADDR3	0

/* tcp thread options */
#define RT_LWIP_TCPTHREAD_PRIORITY		12
#define RT_LWIP_TCPTHREAD_MBOX_SIZE		4
#define RT_LWIP_TCPTHREAD_STACKSIZE		1024

/* ethernet if thread options */
#define RT_LWIP_ETHTHREAD_PRIORITY		15
#define RT_LWIP_ETHTHREAD_MBOX_SIZE		4
#define RT_LWIP_ETHTHREAD_STACKSIZE		512

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
