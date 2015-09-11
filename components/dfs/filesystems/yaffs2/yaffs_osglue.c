/*
 * YAFFS: Yet Another Flash File System. A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2011 Aleph One Ltd.
 *   for Toby Churchill Ltd and Brightstar Engineering
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include "yaffscfg.h"
#include "yaffs_guts.h"
#include "yaffsfs.h"
#include "yaffs_trace.h"
#include <rtthread.h>
#include <errno.h>

static int yaffsfs_lastError;
void yaffsfs_SetError(int err)
{
	//Do whatever to set error
	yaffsfs_lastError = err;
}

int yaffsfs_GetLastError(void)
{
	return yaffsfs_lastError;
}

static rt_mutex_t mutex = RT_NULL;
void yaffsfs_Lock(void)
{
	rt_mutex_take(mutex, RT_WAITING_FOREVER);
}

void yaffsfs_Unlock(void)
{
	rt_mutex_release(mutex);
}

extern void yaffs_dev_rewind();
extern void* yaffs_next_dev();

ALIGN(RT_ALIGN_SIZE)
static char bg_gc_func_stack[4096];
struct rt_thread thread_gc_func;
static void bg_gc_func(void *parameter)
{
	struct yaffs_dev *dev;
	int urgent = 0;
	int result;
	int next_urgent;
    int erased_chunks,free_chunks;

	/* Sleep for a bit to allow start up */
	rt_thread_delay(20000);
    
	while (1) {
		/* Iterate through devices, do bg gc updating ungency */
		yaffs_dev_rewind();
		next_urgent = 0;

		while ((dev = (struct yaffs_dev*)yaffs_next_dev()) != NULL) {
            yaffsfs_Lock();
			result = yaffs_bg_gc(dev, urgent);
            erased_chunks = dev->n_erased_blocks * dev->param.chunks_per_block;
            free_chunks = dev->n_free_chunks;
            yaffsfs_Unlock();
			if (result <= 0)
				next_urgent = 1;
            yaffs_trace(YAFFS_TRACE_GC, "GC erased %d free %d next %d", 
                        erased_chunks, free_chunks, next_urgent);
		}

		urgent = next_urgent;
		if (next_urgent)
			rt_thread_delay(1000);
		else
			rt_thread_delay(2000);
	}
}

void yaffsfs_LockInit(void)
{
    //已经初始化过了，不用重复初始化
    if (mutex != NULL)
        return;
    
	mutex = rt_mutex_create("yaffs", RT_IPC_FLAG_FIFO);
    if (mutex == RT_NULL)
    {
		yaffs_trace(YAFFS_TRACE_ERROR,
			"**>> yaffs error :yaffs_LockInit can't get a mutex!");
    }

#if 1
    rt_thread_init(&thread_gc_func,
                   "gc_yfs",
                   bg_gc_func,
                   RT_NULL,
                   &bg_gc_func_stack[0],
                   sizeof(bg_gc_func_stack),RT_THREAD_PRIORITY_MAX - 2,20);
    rt_thread_startup(&thread_gc_func);
#endif
}

u32 yaffsfs_CurrentTime(void)
{
	return 0;
}

void *yaffsfs_malloc(size_t size)
{
	return rt_malloc(size);
}

void yaffsfs_free(void *ptr)
{
	rt_free(ptr);
}

void yaffsfs_OSInitialisation(void)
{
    yaffsfs_LockInit();
}


