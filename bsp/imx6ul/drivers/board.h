/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-07-06     Bernard    the first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <registers.h>
#include <irq_numbers.h>

#define HEAP_BEGIN        0x80E00000
#define HEAP_END          0x83E00000

#define CONSOLE_DEVICE "uart1"
#define FINSH_DEVICE_NAME "uart1"

void rt_hw_board_init(void);
void rt_hw_interrupt_init(void);
int rt_hw_uart_init(void);
int rt_hw_timer_init(void);
int rt_hw_spi_init(void);
int rt_hw_rtc_init(void);
int rt_hw_mtd_nand_init(void);
int rt_hw_ssp_init(void);
int rt_hw_usbh_init(void);

#ifdef _MSC_VER
#ifdef _DLL
#define EXTVAL __declspec(dllimport)
#else
#define EXTVAL
#endif
#else
#define EXTVAL
#endif

EXTVAL extern volatile int wtdog_count;
EXTVAL extern volatile int sys_stauts;
EXTVAL extern volatile int uptime_count;
EXTVAL extern volatile int fs_system_init;
EXTVAL extern volatile int tty_rx_inxpz;
EXTVAL extern unsigned char PZ[4];
void inittmppath(void);
void cleartmppath(void);
char *GetPrivateStringData(const char *name, char *buf, int buflen, const char *file);
int SetPrivateStringData(const char *name, const char *buf, const char *file);

#ifndef _MSC_VER
extern void __delay(int loops);
extern void __bad_udelay(void);
extern void __udelay(unsigned long usecs);
extern void __const_udelay(unsigned long);

#define HZ 1000
#define MAX_UDELAY_MS 2

#define udelay(n)							\
	(__builtin_constant_p(n) ?					\
	  ((n) > (MAX_UDELAY_MS * 1000) ? __bad_udelay() :		\
			__const_udelay((n) * ((2199023U*HZ)>>11))) :	\
	  __udelay(n))

#endif
#endif
