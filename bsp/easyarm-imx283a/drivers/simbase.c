/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 */

#include <rthw.h>
#include <rtthread.h>
#include <windows.h>
#include "board.h"
#include <stdio.h>
#include <dfs_posix.h>
#pragma comment (lib, "./drivers/vslib/LIBCMTD.lib")

/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
    rt_hw_usart_init();
#ifdef RT_USING_CONSOLE
    rt_console_set_device(CONSOLE_DEVICE);
#endif
#ifdef RT_USING_MTD_NAND
    rt_hw_mtd_nand_init();
#endif
}

extern void *rt_interrupt_main_thread;
void cpu_usage_idle_hook(void)
{
    static unsigned char status = 0;
    if (status && rt_interrupt_main_thread)
        SwitchToFiber(rt_interrupt_main_thread);
    status = !status;
}

int cmd_beep(int argc, char** argv)
{
    int beep = 100,i;
    if (argc > 1)
        beep = atol(argv[1]);
    for (i=0;i<beep;i+=100)
    {
        Beep(2000,100);
        rt_thread_delay(50);
    }
    return 0;
}

/*@}*/
