/*
 * File      : readspeed.c
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

void readspeed(const char* filename, int block_size)
{
    int fd,length;
    char *buff_ptr;
    rt_size_t total_length;
    rt_tick_t tick;

    fd = open(filename, 0, O_RDONLY);
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

    tick = rt_tick_get();
    total_length = 0;
    while (1)
    {
        length = read(fd, buff_ptr, block_size);
        if (length != block_size)
		{
			rt_kprintf("read failed\n");
			break;
		}
        total_length += length;
    }
    tick = rt_tick_get() - tick;

	/* close file and release memory */
    close(fd);
	rt_free(buff_ptr);

    /* calculate read speed */
    rt_kprintf("File read speed: %d byte/s\n", total_length * RT_TICK_PER_SECOND / tick);
}

#ifdef RT_USING_FINSH
#include <finsh.h>
int cmd_readspeed(int argc, char** argv)
{
	int blocksize = 4096;
	if (argc < 2)
	{
		rt_kprintf("Usag: readspeed /tmp/test.db\n");
		return 1;
	}
	if (argc > 2)
		blocksize = atol(argv[2]);
	readspeed(argv[1],blocksize);
	return 0;
}

FINSH_FUNCTION_EXPORT(readspeed, perform file read test);
FINSH_FUNCTION_EXPORT_ALIAS(cmd_readspeed, __cmd_readspeed, perform file read test);
#endif
