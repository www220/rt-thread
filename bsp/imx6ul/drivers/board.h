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
#include <asm-generic/gpio.h>

void s_init(void);
int arch_cpu_init(void);
int timer_init(void);
int board_postclk_init(void);
void enable_caches(void);

void udelay(unsigned long usec);
void mdelay(unsigned long msec);
void __udelay(unsigned long usec);
ulong get_timer_masked(void);
int print_cpuinfo(void);

#define HEAP_BEGIN      0x80A00000
#define HEAP_END        0x8FF00000

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
void rt_hw_uart_init(void);
void rt_hw_timer_init(void);
void rt_hw_spi_init(void);
void rt_hw_rtc_init(void);
void rt_hw_mtd_nand_init(void);
void rt_hw_ssp_init(void);
void rt_hw_usbh_init(void);

#ifdef _MSC_VER
#ifdef _DLL
#define EXTVAL __declspec(dllimport)
#else
#define EXTVAL
#endif
#else
#define EXTVAL
#endif

#define IMX_GPIO_NR(port, index)		((((port)-1)*32)+((index)&31))
#define PIN_WDT		IMX_GPIO_NR(5,0)
#define PIN_RUN		IMX_GPIO_NR(4,16)
#define PIN_ERR		IMX_GPIO_NR(4,14)
#define PIN_PZ1		IMX_GPIO_NR(5,8)
#define PIN_PZ2		IMX_GPIO_NR(5,9)
#define PIN_NET0	IMX_GPIO_NR(5,3)
#define PIN_NET1	IMX_GPIO_NR(5,4)

EXTVAL extern volatile int wtdog_count;
EXTVAL extern volatile int sys_stauts;
EXTVAL extern volatile int uptime_count;
EXTVAL extern unsigned char PZ[4];
EXTVAL extern char RTT_USER[30];
EXTVAL extern char RTT_PASS[30];

void inittmppath(void);
void cleartmppath(void);
char *GetPrivateStringData(const char *name, char *buf, int buflen, const char *file);
int SetPrivateStringData(const char *name, const char *buf, const char *file);

extern unsigned char *dma_align_mem;
#define rt_memalign(x,y) (void *)dma_align_mem; dma_align_mem += (u32)(((u32)(y)+63)&(u32)(~63))
#define rt_freealign(x)

extern unsigned char *dma_align_max;
#define rt_memalign_max(x,y) (void *)dma_align_max; dma_align_max += (u32)(((u32)(y)+63)&(u32)(~63))
#define rt_freealign_max(x)

#endif
