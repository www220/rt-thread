/*
 * File      : writespeed.c
 * This file is part of RT-TestCase in RT-Thread RTOS
 * COPYRIGHT (C) 2010, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2010-02-10     Bernard      first version
 */
#include <rtthread.h>
#include <dfs_posix.h>
#include <stdlib.h>

void writespeed(const char* filename, int total_length, int block_size)
{
    int fd, index, length;
    char *buff_ptr;
    rt_tick_t tick;

    fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0)
    {
        rt_kprintf("open file:%s failed\n", filename);
        return;
    }

    buff_ptr = rt_malloc(block_size);
    if (buff_ptr == RT_NULL)
    {
        rt_kprintf("no memory\n");
        close(fd);

        return;
    }

	/* prepare write data */
	for (index = 0; index < block_size; index++)
	{
		buff_ptr[index] = index;
	}

	/* get the beginning tick */
    tick = rt_tick_get();
	index = 0;
	while (index < total_length / block_size)
	{
		length = write(fd, buff_ptr, block_size);
		if (length != block_size)
		{
			rt_kprintf("write failed\n");
			break;
		}
		index ++;
	}
    tick = rt_tick_get() - tick;

	/* close file and release memory */
    close(fd);
	rt_free(buff_ptr);

    /* calculate write speed */
	length = index * block_size * 10/ tick;
    rt_kprintf("File write speed: %d %d.%d Kbyte/s\n", index * block_size, length/10, length%10);
}

#ifdef RT_USING_FINSH
#include <finsh.h>
int cmd_writespeed(int argc, char** argv)
{
	int blocksize = 8192,total_length = 10*1024*1024;
	if (argc < 2)
	{
		rt_kprintf("Usag: writespeed /tmp/test.db\n");
		return 1;
	}
	if (argc > 2)
		total_length = atol(argv[2]);
	if (argc > 3)
		blocksize = atol(argv[3]);
	writespeed(argv[1],total_length,blocksize);
	return 0;
}

FINSH_FUNCTION_EXPORT(writespeed, perform file write test);
FINSH_FUNCTION_EXPORT_ALIAS(cmd_writespeed, __cmd_writespeed, perform file write test);
#endif
