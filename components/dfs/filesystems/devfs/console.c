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
#include <rthw.h>
#include "linux-syscall.h"
#include "linux-usedef.h"

static cc_t def_c_cc[NCCS] = {CINTR,CQUIT,CERASE,CKILL,CEOF,CTIME,CMIN,0,CSTART,CSTOP,CSUSP,
														 0,CREPRINT,CDISCARD,CWERASE,CLNEXT};

struct console_device
{
	struct rt_device parent;

	rt_device_t device; /* the actual device */
	struct winsize win;
	struct termios ios;
	int file[32];
};
struct console_device _console,_tty;

/* common device interface */
static rt_err_t console_init(rt_device_t dev)
{
	rt_err_t ret = RT_EOK;
	struct console_device* device;

	device = (struct console_device*)dev;
	RT_ASSERT(device != RT_NULL);

	memset(&device->win,0,sizeof(struct winsize));
	device->win.ws_col = 132;
	device->win.ws_row = 43;

    memset(&device->ios,0,sizeof(struct termios));
    device->ios.c_iflag = (BRKINT | ISTRIP | ICRNL | IMAXBEL | IXON | IXANY);
    device->ios.c_oflag = (OPOST | ONLCR);
    device->ios.c_cflag = (B115200 | CS8 | CREAD | HUPCL | CLOCAL);
    device->ios.c_lflag = (ICANON | ISIG | ECHO | IEXTEN | ECHOE|ECHOKE|ECHOCTL);
    memcpy(device->ios.c_cc,def_c_cc,sizeof(def_c_cc));

    memset(&device->file,0,sizeof(device->file));
	return ret;
}

static rt_err_t console_open(rt_device_t dev, rt_uint16_t oflag)
{
	rt_err_t ret = RT_EOK;
	struct console_device* device;

	device = (struct console_device*)dev;
	RT_ASSERT(device != RT_NULL);

	/* open this device and set the new device in finsh shell */
	return rt_device_open(device->device, RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_INT_RX|RT_DEVICE_FLAG_STREAM);
}

static rt_err_t console_close(rt_device_t dev)
{
	rt_err_t ret = RT_EOK;
	struct console_device* device;

	device = (struct console_device*)dev;
	RT_ASSERT(device != RT_NULL);

	return rt_device_close(device->device);
}

#include <shell.h>
#include <errno.h>
extern struct finsh_shell *shell;

static rt_size_t console_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	int ret = 0;
	struct console_device* device;

	device = (struct console_device*)dev;
	RT_ASSERT(device != RT_NULL);

	rt_sem_take(&shell->rx_sem, 0);
	ret = rt_device_read(device->device, pos, buffer, size);
	if (ret <= 0)
	{
		while (rt_sem_take(&shell->rx_sem, 0) == RT_EOK);
		rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER);
		ret = rt_device_read(device->device, pos, buffer, size);
	}
	if (ret>0 && device->ios.c_lflag & ECHO)
	{
		//回车换行的转换
		if (device->ios.c_iflag & (ICRNL|INLCR))
		{
			char read = '\r',write = '\n';
			if (device->ios.c_iflag & INLCR)
			{
				read = '\n';
				write = '\r';
			}
			size = ret;
			while (size--)
			{
				if (((char *)buffer)[size] == read)
					((char *)buffer)[size] = write;
			}
		}
		rt_device_write(device->device, pos, buffer, ret);
	}
	return ret;
}

static rt_size_t console_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	struct console_device* device;

	device = (struct console_device*)dev;
	RT_ASSERT(device != RT_NULL);

	//回车换行的转换
	if (device->ios.c_oflag & ONLCR)
		device->parent.open_flag |= RT_DEVICE_FLAG_STREAM;
	else
		device->parent.open_flag &= ~RT_DEVICE_FLAG_STREAM;

	return rt_device_write(device->device, pos, buffer, size);
}

static rt_err_t console_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	struct console_device* device;

	device = (struct console_device*)dev;
	RT_ASSERT(device != RT_NULL);

	switch(cmd)
	{
	case RT_DEVICE_CTRL_GETWS:
		rt_memcpy(args,&device->win,sizeof(device->win));
		return RT_EOK;
	case RT_DEVICE_CTRL_SETWS:
		rt_memcpy(&device->win,args,sizeof(device->win));
		return RT_EOK;
	case RT_DEVICE_CTRL_GETS:
		rt_memcpy(args,&device->ios,sizeof(device->ios));
		return RT_EOK;
	case RT_DEVICE_CTRL_SETS:
	case RT_DEVICE_CTRL_SETSW:
	case RT_DEVICE_CTRL_SETSF:
		rt_memcpy(&device->ios,args,sizeof(device->ios));
		return RT_EOK;
	}
	return rt_device_control(device->device, cmd, args);
}

struct null_device
{
	struct rt_device parent;
};
struct null_device _nulldev,_zerodev;

/* common device interface */
static rt_err_t nulldev_init(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t nulldev_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t nulldev_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_size_t nulldev_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	struct null_dev* device;

	device = (struct null_dev*)dev;
	RT_ASSERT(device != RT_NULL);

	if (dev->parent.name[0] == 'z' && dev->parent.name[1] == 'e')
	{
		rt_memset(buffer, 0, size);
		return size;
	}
	return 0;
}

static rt_size_t nulldev_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	return size;
}

void rt_console_init(const char* device_name)
{
	rt_device_t device;
	/* register to device framework */

	device = rt_device_find(device_name);
	if (device != RT_NULL)
	{
		struct console_device* console;
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

		rt_device_register(&console->parent, "console", RT_DEVICE_FLAG_RDWR);

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

		console->device = device;

		rt_device_register(&console->parent, "tty", RT_DEVICE_FLAG_RDWR);
	}

	{
		struct null_device* nulldev;
		/* get null device */
		nulldev = &_nulldev;
		rt_memset(nulldev, 0, sizeof(_nulldev));

		/* device initialization */
		nulldev->parent.type = RT_Device_Class_Char;
		/* set device interface */
		nulldev->parent.init 	= nulldev_init;
		nulldev->parent.open 	= nulldev_open;
		nulldev->parent.close   = nulldev_close;
		nulldev->parent.read 	= nulldev_read;
		nulldev->parent.write   = nulldev_write;
		nulldev->parent.user_data = RT_NULL;

		rt_device_register(&nulldev->parent, "null", RT_DEVICE_FLAG_RDWR);

		/* get zero device */
		nulldev = &_zerodev;
		rt_memset(nulldev, 0, sizeof(_nulldev));

		/* device initialization */
		nulldev->parent.type = RT_Device_Class_Char;
		/* set device interface */
		nulldev->parent.init 	= nulldev_init;
		nulldev->parent.open 	= nulldev_open;
		nulldev->parent.close   = nulldev_close;
		nulldev->parent.read 	= nulldev_read;
		nulldev->parent.write   = nulldev_write;
		nulldev->parent.user_data = RT_NULL;

		rt_device_register(&nulldev->parent, "zero", RT_DEVICE_FLAG_RDWR);
	}
}

