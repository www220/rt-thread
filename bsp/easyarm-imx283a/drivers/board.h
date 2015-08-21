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
#include "regs-clkctrl.h"
#include "pinctrl.h"

//	<i>Default: 64
#define HEAP_SIZE         32
#define HEAP_END          (0x40000000 + HEAP_SIZE * 1024 * 1024)

#define RT_USING_DBGU
//#deiine RT_USING_UART1

#define CONSOLE_DEVICE "dbgu"
#define FINSH_DEVICE_NAME "dbgu"

void rt_hw_board_init(void);
void rt_hw_interrupt_init(void);
void rt_hw_usart_init(void);
void rt_hw_timer_init(void);
void rt_hw_spi_init(void);
void rt_hw_rtc_init(void);

extern volatile int wtdog_count;
void inittmppath(void);

#endif
