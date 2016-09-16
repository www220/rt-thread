/*
 * File      : console.c
 * This file is part of Device File System in RT-Thread RTOS
 * COPYRIGHT (C) 2004-2011, RT-Thread Development Team
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
 * 2015.03.27     Bernard      Add author information.
 */

#include <rtthread.h>
#ifdef RT_USING_PROCESS
#include <rthw.h>
#include <dfs_def.h>
#include <dfs_file.h>
extern int rt_process_convfile(rt_process_t module, int fileno);
#endif

struct console_device
{
	struct rt_device parent;

	rt_device_t device; /* the actual device */
	int 		device_type;
};
struct console_device _console,_tty,_null,_zero;

/* common device interface */
static rt_err_t console_init(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t console_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t console_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_size_t console_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	struct console_device* device;

	device = (struct console_device*)dev;
	RT_ASSERT(device != RT_NULL);

	if (device->device_type == 3)
	{
		errno = EBADF;
		return -1;
	}
	else if (device->device_type == 4)
	{
		rt_memset(buffer, 0, size);
		return size;
	}
#ifdef RT_USING_PROCESS
	else if (device->device_type == 2)
	{
		rt_process_t process = rt_process_self();
		if (process == RT_NULL)
		{
			errno = EBADF;
			return -1;
		}
		int fileno = rt_process_convfile(process,0);
		if (fileno < 0)
		{
			errno = EBADF;
			return -1;
		}
		struct dfs_fd *d = fd_get(fileno);
		//必须是devfs
		if (d == RT_NULL || d->fs == RT_NULL || rt_strcmp(d->fs->path,"/dev") != 0)
		{
			if (d)
				fd_put(d);
			errno = EBADF;
			return -1;
		}
		fd_put(d);
		return rt_device_read((rt_device_t)d->data, pos, buffer, size);
	}
#endif
	return rt_device_read(device->device, pos, buffer, size);
}

static rt_size_t console_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	struct console_device* device;

	device = (struct console_device*)dev;
	RT_ASSERT(device != RT_NULL);

	if (device->device_type == 3)
	{
		return size;
	}
	else if (device->device_type == 4)
	{
		errno = EBADF;
		return -1;
	}
#ifdef RT_USING_PROCESS
	else if (device->device_type == 2)
	{
		rt_process_t process = rt_process_self();
		if (process == RT_NULL)
		{
			errno = EBADF;
			return -1;
		}
		int fileno = rt_process_convfile(process,0);
		if (fileno < 0)
		{
			errno = EBADF;
			return -1;
		}
		struct dfs_fd *d = fd_get(fileno);
		//必须是devfs
		if (d == RT_NULL || d->fs == RT_NULL || rt_strcmp(d->fs->path,"/dev") != 0)
		{
			if (d)
				fd_put(d);
			errno = EBADF;
			return -1;
		}
		fd_put(d);
		return rt_device_write((rt_device_t)d->data, pos, buffer, size);
	}
#endif
	return rt_device_write(device->device, pos, buffer, size);
}

static rt_err_t console_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	struct console_device* device;

	device = (struct console_device*)dev;
	RT_ASSERT(device != RT_NULL);

	if (device->device_type == 3)
	{
		errno = ENOTTY;
		return -RT_ENOSYS;
	}
	else if (device->device_type == 4)
	{
		errno = ENOTTY;
		return -RT_ENOSYS;
	}
#ifdef RT_USING_PROCESS
	else if (device->device_type == 2)
	{
		rt_process_t process = rt_process_self();
		if (process == RT_NULL)
		{
			errno = ENOTTY;
			return -RT_ENOSYS;
		}
		int fileno = rt_process_convfile(process,0);
		if (fileno < 0)
		{
			errno = ENOTTY;
			return -RT_ENOSYS;
		}
		struct dfs_fd *d = fd_get(fileno);
		//必须是devfs
		if (d == RT_NULL || d->fs == RT_NULL || rt_strcmp(d->fs->path,"/dev") != 0)
		{
			if (d)
				fd_put(d);
			errno = ENOTTY;
			return -RT_ENOSYS;
		}
		fd_put(d);
		return rt_device_control((rt_device_t)d->data, cmd, args);
	}
#endif
	return rt_device_control(device->device, cmd, args);
}

void rt_console_init(const char* device_name)
{
	rt_device_t device;
	/* register to device framework */
	struct console_device* console;

	device = rt_device_find(device_name);
	if (device != RT_NULL)
	{
		/* get console device */
		console = &_console;
		rt_memset(console, 0, sizeof(_console));

		/* device initialization */
		console->parent.type = RT_Device_Class_Char;
		/* set device interface */
		console->parent.init 	= console_init;
		console->parent.open 	= console_open;
		console->parent.close   = console_close;
		console->parent.read 	= console_read;
		console->parent.write   = console_write;
		console->parent.control	= console_control;
		console->parent.user_data = RT_NULL;

		console->device = device;
		console->device_type = 1;
		rt_device_register(&console->parent, "console", RT_DEVICE_FLAG_RDWR);
	}
#ifdef RT_USING_PROCESS
	{
		/* get tty device */
		console = &_tty;
		rt_memset(console, 0, sizeof(_tty));

		/* device initialization */
		console->parent.type = RT_Device_Class_Char;
		/* set device interface */
		console->parent.init 	= console_init;
		console->parent.open 	= console_open;
		console->parent.close   = console_close;
		console->parent.read 	= console_read;
		console->parent.write   = console_write;
		console->parent.control	= console_control;
		console->parent.user_data = RT_NULL;

		console->device = RT_NULL;
		console->device_type = 2;
		rt_device_register(&console->parent, "tty", RT_DEVICE_FLAG_RDWR);
	}
#endif
	{
		/* get null device */
		console = &_null;
		rt_memset(console, 0, sizeof(_null));

		/* device initialization */
		console->parent.type = RT_Device_Class_Miscellaneous;
		/* set device interface */
		console->parent.init 	= console_init;
		console->parent.open 	= console_open;
		console->parent.close   = console_close;
		console->parent.read 	= console_read;
		console->parent.write   = console_write;
		console->parent.control	= console_control;
		console->parent.user_data = RT_NULL;

		console->device = RT_NULL;
		console->device_type = 3;
		rt_device_register(&console->parent, "null", RT_DEVICE_FLAG_RDWR);
	}

	{
		/* get zero device */
		console = &_zero;
		rt_memset(console, 0, sizeof(_zero));

		/* device initialization */
		console->parent.type = RT_Device_Class_Miscellaneous;
		/* set device interface */
		console->parent.init 	= console_init;
		console->parent.open 	= console_open;
		console->parent.close   = console_close;
		console->parent.read 	= console_read;
		console->parent.write   = console_write;
		console->parent.control	= console_control;
		console->parent.user_data = RT_NULL;

		console->device = RT_NULL;
		console->device_type = 4;
		rt_device_register(&console->parent, "zero", RT_DEVICE_FLAG_RDWR);
	}
}

