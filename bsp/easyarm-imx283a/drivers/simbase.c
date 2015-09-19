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

static int libcinit =0;
void libc_system_init()
{
#ifdef RT_USING_DFS
    int fd;
    struct rt_device *console_dev;

#ifndef RT_USING_DFS_DEVFS
#error Please enable devfs by defining RT_USING_DFS_DEVFS in rtconfig.h
#endif

    /* init console device */
    extern void rt_console_init(const char* device_name);
    console_dev = rt_console_get_device();
    if (console_dev)
        rt_console_init(console_dev->parent.name);

    /* open console as stdin/stdout/stderr */
    fd = open("/dev/console", O_RDONLY, 0);	/* for stdin */
    fd = open("/dev/console", O_WRONLY, 0);	/* for stdout */
    fd = open("/dev/console", O_WRONLY, 0);	/* for stderr */
#endif
    libcinit = 1;
}

int _write (int fh, const void *buf, unsigned cnt)
{
    if (fh < 3) {
        //lib初始化在后面，直接输出无法printf
        if (libcinit) {
            return write(fh, buf, cnt);
        } else {
            rt_device_t dev = rt_console_get_device();
            rt_uint16_t old_flag = dev->open_flag;
            dev->open_flag |= RT_DEVICE_FLAG_STREAM;
            rt_device_write(dev, 0, buf, cnt);
            dev->open_flag = old_flag;
            return cnt;
        }
    } else {
        return -1;
    }
}

errno_t _tsopen_s (
        int * pfh,
        const char *path,
        int oflag,
        int shflag,
        int pmode
        )
{
    return -1;
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
    return 0;
}

/*@}*/
