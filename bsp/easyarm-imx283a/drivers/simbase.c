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

/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
    rt_hw_usart_init();
#ifdef RT_USING_CONSOLE
    rt_console_set_device(CONSOLE_DEVICE);
#endif
}

void libc_system_init(const char* tty_name)
{
#ifdef RT_USING_DFS
    int fd;

#ifndef RT_USING_DFS_DEVFS
#error Please enable devfs by defining RT_USING_DFS_DEVFS in rtconfig.h
#endif

    /* init console device */
    extern void rt_console_init(const char* device_name);
    rt_console_init(tty_name);

    /* open console as stdin/stdout/stderr */
    fd = open("/dev/console", O_RDONLY, 0);	/* for stdin */
    fd = open("/dev/console", O_WRONLY, 0);	/* for stdout */
    fd = open("/dev/console", O_WRONLY, 0);	/* for stderr */
#endif
}

int _write (int fh, const void *buf, unsigned cnt)
{
    if (fh < 3)
        return write(fh, buf, cnt);
    else
        return -1;
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

#include <finsh.h>
struct finsh_syscall* finsh_syscall_next(struct finsh_syscall* call)
{
    unsigned int *ptr_begin = (unsigned int*)call;
    ptr_begin += (sizeof(struct finsh_syscall)/sizeof(unsigned int));
    while (*ptr_begin == 0) ptr_begin ++;
    return (struct finsh_syscall*) ptr_begin;
}

struct finsh_sysvar* finsh_sysvar_next(struct finsh_sysvar* call)
{
    unsigned int *ptr_begin = (unsigned int*)call;
    ptr_begin += (sizeof(struct finsh_sysvar)/sizeof(unsigned int));
    while (*ptr_begin == 0) ptr_begin ++;
    return (struct finsh_sysvar*) ptr_begin;
}

extern void *rt_interrupt_main_thread;
void cpu_usage_idle_hook(void)
{
    static unsigned char status = 0;
    if (status && rt_interrupt_main_thread)
        SwitchToFiber(rt_interrupt_main_thread);
    status = !status;
}

/*@}*/
