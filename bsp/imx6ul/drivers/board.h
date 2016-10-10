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

#include <gic.h>
#include <irq_numbers.h>

int timer_init(void);
void udelay(unsigned long usec);
void mdelay(unsigned long msec);
void __udelay(unsigned long usec);
ulong get_timer_masked(void);
int print_cpuinfo(void);

#define HEAP_BEGIN      (void*)(0x80000000 + 32 * 1024 * 1024)
#define HEAP_END        (void*)(0x80000000 + 224 * 1024 * 1024)

#define RT_USING_UART1
#define RT_USING_UART2
//#define RT_USING_UART3
//#define RT_USING_UART4
//#define RT_USING_UART5
//#define RT_USING_UART6
//#define RT_USING_UART7
//#define RT_USING_UART8

#define CONSOLE_DEVICE "uart1"
#define FINSH_DEVICE_NAME "uart1"
#define get_timer_usec() get_timer_masked()

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
EXTVAL extern unsigned char PZ[4];
void inittmppath(void);
void cleartmppath(void);
char *GetPrivateStringData(const char *name, char *buf, int buflen, const char *file);
int SetPrivateStringData(const char *name, const char *buf, const char *file);

#endif
