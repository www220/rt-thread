/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Develop Team
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
 * 2011-01-13     weety      add board.h to this bsp
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include "common.h"
#include "regs-uartdbg.h"
#include "regs-digctl.h"
#include "regs-timrot.h"
#include "regs-uartdbg.h"
#include "regs-uartapp.h"
#include "regs-clkctrl.h"
#include "regs-ocotp.h"
#include "regs-enet.h"
#include "regs-rtc.h"
#include "regs-lcdif.h"
#include "regs-pwm.h"
#include "regs-lradc.h"
#include "regs-ssp.h"
#include "regs-usbphy.h"
#include "pinctrl.h"
#include "mmu.h"
#include <stdarg.h>

#define RT_USING_DBGU
#define RT_USING_UART1
#define RT_USING_UART2
#define RT_USING_UART3
#define RT_USING_UART4
#define RT_USING_UART5

#define DIS_BEEP	0
#define MXS280A		1

#define PIN_WDT		PINID_GPMI_RDY1
#if !MXS280A
#define PIN_BEEP		PINID_LCD_D21
#define PIN_RUN		PINID_LCD_D22
#define PIN_ERR		PINID_LCD_D23
#define PIN_NET0	PINID_LCD_D16
#define PIN_GPRS	PINID_LCD_D07
#define PIN_UART1	PINID_SAIF0_MCLK
#define PIN_UART2	PINID_SSP1_DATA3
#define PIN_UART3	PINID_ENCODE(2, 14)
#define PIN_UART4	PINID_SSP1_SCK
#define PIN_PZ1		PINID_LCD_D19
#define PIN_PZ2		PINID_LCD_D20
#else
#define PIN_BEEP		PINID_SSP0_DATA6
#define PIN_RUN		PINID_SSP0_DATA7
#define PIN_ERR		PINID_SAIF0_MCLK
#define PIN_NET0	PINID_LCD_RS
#define PIN_GPRS	PINID_SAIF0_LRCLK
#define PIN_UART1	PINID_SAIF1_SDATA0
#define PIN_UART2	PINID_PWM3
#define PIN_UART3	PINID_LCD_WR_RWN
#define PIN_UART4	PINID_LCD_RD_E
#define PIN_PZ1		PINID_SSP0_DATA4
#define PIN_PZ2		PINID_SSP0_DATA5
#endif

#define CONSOLE_DEVICE "dbgu"
#define FINSH_DEVICE_NAME "dbgu"
#define PPP_DEVICE_NAME "uart5"

void rt_hw_board_init(void);
void rt_hw_interrupt_init(void);
void rt_hw_usart_init(void);
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

EXTVAL extern volatile int eth_wtdog;
EXTVAL extern volatile int eth_linkstatus;
EXTVAL extern volatile int wtdog_count;
EXTVAL extern volatile int sys_stauts;
EXTVAL extern volatile int ppp_linkstauts;
EXTVAL extern volatile int uptime_count;
EXTVAL extern unsigned char PZ[4];
EXTVAL extern char RTT_USER[30];
EXTVAL extern char RTT_PASS[30];
void inittmppath(void);
void cleartmppath(void);
char *GetPrivateStringData(const char *name, char *buf, int buflen, const char *file);
int SetPrivateStringData(const char *name, const char *buf, const char *file);

#define SysLog_Main(debug_level, format, ...) debug_printf_def(debug_level, "Main", format, ##__VA_ARGS__)
#define SysLog_Gprs(debug_level, format, ...) debug_printf_def(debug_level, "Gprs", format, ##__VA_ARGS__)
#define SysLog_Vpn(debug_level, format, ...) debug_printf_def(debug_level, "Vpn", format, ##__VA_ARGS__)
#define SysLog_Ioscan(debug_level, format, ...) rt_kprintf(format, ##__VA_ARGS__); \
    debug_printf_def(debug_level, "Ioscan", format, ##__VA_ARGS__)
#define SysLog_Web(debug_level, format, ...) rt_kprintf(format, ##__VA_ARGS__); \
    debug_printf_def(debug_level, "Web", format, ##__VA_ARGS__)

extern struct rt_semaphore syslog_sem;
extern volatile int g_nSaveSysLog;

extern int debug_printf_init();
extern int debug_printf_def(int debug_level, const char *module, const char *format, ...);
extern int debug_printf_ap(int debug_level, const char *module, const char *format, va_list ap);

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

#include <rtthread.h>
static inline int mxs_reset_clock(u32 hwreg, int is_enable)
{
	int timeout;
#define SFTRST 0x80000000
#define CLKGATE 0x40000000

	/* the process of software reset of IP block is done
	   in several steps:

	   - clear SFTRST and wait for block is enabled;
	   - clear clock gating (CLKGATE bit);
	   - set the SFTRST again and wait for block is in reset;
	   - clear SFTRST and wait for reset completion.
	 */
	/* clear SFTRST */
	REG_CLR_ADDR(hwreg, SFTRST);

	for (timeout = 1000000; timeout > 0; timeout--)
		/* still in SFTRST state ? */
		if ((REG_RD_ADDR(hwreg) & SFTRST) == 0)
			break;
		if (timeout <= 0) {
			rt_kprintf("%s(%p): timeout when enabling\n",
				__func__, hwreg);
			return -1;
	}

	/* clear CLKGATE */
	REG_CLR_ADDR(hwreg, CLKGATE);

	if (is_enable) {
		/* now again set SFTRST */
		REG_SET_ADDR(hwreg, SFTRST);
		for (timeout = 1000000; timeout > 0; timeout--)
			/* poll until CLKGATE set */
			if (REG_RD_ADDR(hwreg) & CLKGATE)
				break;
		if (timeout <= 0) {
			rt_kprintf("%s(%p): timeout when resetting\n",
				__func__, hwreg);
			return -1;
		}

		REG_CLR_ADDR(hwreg, SFTRST);
		for (timeout = 1000000; timeout > 0; timeout--)
			/* still in SFTRST state ? */
			if ((REG_RD_ADDR(hwreg) & SFTRST) == 0)
				break;
		if (timeout <= 0) {
			rt_kprintf("%s(%p): timeout when enabling "
				"after reset\n", __func__, hwreg);
			return -1;
		}

		/* clear CLKGATE */
		REG_CLR_ADDR(hwreg, CLKGATE);
	}
	for (timeout = 1000000; timeout > 0; timeout--)
		/* still in SFTRST state ? */
		if ((REG_RD_ADDR(hwreg) & CLKGATE) == 0)
			break;

	if (timeout <= 0) {
		rt_kprintf("%s(%p): timeout when unclockgating\n",
			__func__, hwreg);
		return -1;
	}

	return 0;
}

static inline void mxs_clock_enable(u32 hwreg, u32 gate)
{
	unsigned int reg = readl(hwreg);
	reg &= ~gate;
	writel(reg, hwreg);
}

static inline void mxs_clock_disable(u32 hwreg, u32 gate)
{
	unsigned int reg = readl(hwreg);
	reg |= gate;
	writel(reg, hwreg);
}

#endif
#endif
