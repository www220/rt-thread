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

//#include <errno.h>


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


#ifdef CONFIG_YAFFS_USE_PTHREADS
#include <pthread.h>
static pthread_mutex_t mutex1;


void yaffsfs_Lock(void)
{
	pthread_mutex_lock( &mutex1 );
}

void yaffsfs_Unlock(void)
{
	pthread_mutex_unlock( &mutex1 );
}

void yaffsfs_LockInit(void)
{
	pthread_mutex_init( &mutex1, NULL);
}

#else

static rt_mutex_t mutex = RT_NULL;
void yaffsfs_Lock(void)
{
	rt_mutex_take(mutex, RT_WAITING_FOREVER);
}

void yaffsfs_Unlock(void)
{
	rt_mutex_release(mutex);
}

void yaffsfs_LockInit(void)
{
	mutex = rt_mutex_create("mutex", RT_IPC_FLAG_FIFO);
    if (mutex == RT_NULL)
    {
		yaffs_trace(YAFFS_TRACE_ERROR,
			"**>> yaffs error :yaffs_LockInit can't get a mutex!");
    }

}
#endif

u32 yaffsfs_CurrentTime(void)
{
	return 0;
}


static int yaffs_kill_alloc = 0;
static size_t total_malloced = 0;
static size_t malloc_limit = 0 & 6000000;

void *yaffsfs_malloc(size_t size)
{
	void * this;
	if(yaffs_kill_alloc)
		return NULL;
	if(malloc_limit && malloc_limit <(total_malloced + size) )
		return NULL;

	this = rt_malloc(size);
	if(this)
		total_malloced += size;
	return this;
}

void yaffsfs_free(void *ptr)
{
	rt_free(ptr);
}

void yaffsfs_OSInitialisation(void)
{
	yaffsfs_LockInit();
}


