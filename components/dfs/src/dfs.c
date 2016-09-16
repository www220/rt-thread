/*
 * File      : dfs.c
 * This file is part of Device File System in RT-Thread RTOS
 * COPYRIGHT (C) 2004-2012, RT-Thread Development Team
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
 * 2005-02-22     Bernard      The first version.
 */

#include <dfs.h>
#include <dfs_fs.h>
#include <dfs_file.h>

/* Global variables */
const struct dfs_filesystem_operation *filesystem_operation_table[DFS_FILESYSTEM_TYPES_MAX];
struct dfs_filesystem filesystem_table[DFS_FILESYSTEMS_MAX];

/* device filesystem lock */
static struct rt_mutex fslock;

#ifdef DFS_USING_WORKDIR
char working_directory[DFS_PATH_MAX] = {"/"};
#endif

#ifdef DFS_USING_STDIO
struct dfs_fd fd_table[3 + DFS_FD_MAX];
#else
struct dfs_fd fd_table[DFS_FD_MAX];
#endif

#ifdef DFS_USING_SELECT
#include <rthw.h>
static int dfs_select_semindex;
struct dfs_select_info dfs_select_list;
#endif

/**
 * @addtogroup DFS
 */

/*@{*/

/**
 * this function will initialize device file system.
 */
int dfs_init(void)
{
    /* clear filesystem operations table */
    rt_memset((void *)filesystem_operation_table, 0, sizeof(filesystem_operation_table));
    /* clear filesystem table */
    rt_memset(filesystem_table, 0, sizeof(filesystem_table));
    /* clean fd table */
    rt_memset(fd_table, 0, sizeof(fd_table));

    /* create device filesystem lock */
    rt_mutex_init(&fslock, "fslock", RT_IPC_FLAG_FIFO);

#ifdef DFS_USING_WORKDIR
    /* set current working directory */
    rt_memset(working_directory, 0, sizeof(working_directory));
    working_directory[0] = '/';
#endif

#ifdef DFS_USING_SELECT
    rt_memset(&dfs_select_list, 0, sizeof(dfs_select_list));
    rt_list_init(&(dfs_select_list.list));
    dfs_select_semindex = 0;
#endif
	return 0;
}
INIT_COMPONENT_EXPORT(dfs_init);

/**
 * this function will lock device file system.
 *
 * @note please don't invoke it on ISR.
 */
void dfs_lock(void)
{
    rt_err_t result;

    result = rt_mutex_take(&fslock, RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        RT_ASSERT(0);
    }
}

/**
 * this function will lock device file system.
 *
 * @note please don't invoke it on ISR.
 */
void dfs_unlock(void)
{
    rt_mutex_release(&fslock);
}

/**
 * @ingroup Fd
 * This function will allocate a file descriptor.
 *
 * @return -1 on failed or the allocated file descriptor.
 */
int fd_new(void)
{
    struct dfs_fd *d;
    int idx;

    /* lock filesystem */
    dfs_lock();

    /* find an empty fd entry */
#ifdef DFS_USING_STDIO
    for (idx = 3; idx < DFS_FD_MAX + 3 && fd_table[idx].ref_count > 0; idx++);
#else
    for (idx = 0; idx < DFS_FD_MAX && fd_table[idx].ref_count > 0; idx++);
#endif

    /* can't find an empty fd entry */
#ifdef DFS_USING_STDIO
    if (idx == DFS_FD_MAX + 3)
#else
    if (idx == DFS_FD_MAX)
#endif
    {
        idx = -1;
        goto __result;
    }

    d = &(fd_table[idx]);
    d->ref_count = 1;
    d->magic = DFS_FD_MAGIC;

__result:
    dfs_unlock();
    return idx;
}

/**
 * @ingroup Fd
 *
 * This function will return a file descriptor structure according to file
 * descriptor.
 *
 * @return NULL on on this file descriptor or the file descriptor structure
 * pointer.
 */
struct dfs_fd *fd_get(int fd)
{
    struct dfs_fd *d;

#ifdef DFS_USING_STDIO
    if (fd < 3 || fd >= DFS_FD_MAX + 3)
        return RT_NULL;
#else
    if (fd < 0 || fd >= DFS_FD_MAX)
        return RT_NULL;
#endif

    dfs_lock();
    d = &fd_table[fd];

    /* check dfs_fd valid or not */
    if (d->magic != DFS_FD_MAGIC)
    {
        dfs_unlock();
        return RT_NULL;
    }

    /* increase the reference count */
    d->ref_count ++;
    dfs_unlock();

    return d;
}

/**
 * @ingroup Fd
 *
 * This function will put the file descriptor.
 */
void fd_put(struct dfs_fd *fd)
{
    RT_ASSERT(fd != RT_NULL);

    dfs_lock();
    fd->ref_count --;

    /* clear this fd entry */
    if (fd->ref_count == 0)
    {
        rt_memset(fd, 0, sizeof(struct dfs_fd));
    }
    dfs_unlock();
};

/**
 * @ingroup Fd
 *
 * This function will return whether this file has been opend.
 *
 * @param pathname the file path name.
 *
 * @return 0 on file has been open successfully, -1 on open failed.
 */
int fd_is_open(const char *pathname)
{
    char *fullpath;
    unsigned int index;
    struct dfs_filesystem *fs;
    struct dfs_fd *fd;

    fullpath = dfs_normalize_path(RT_NULL, pathname);
    if (fullpath != RT_NULL)
    {
        const char *mountpath;
        fs = dfs_filesystem_lookup(fullpath);
        if (fs == RT_NULL)
        {
            /* can't find mounted file system */
            rt_free(fullpath);

            return -1;
        }

        /* get file path name under mounted file system */
        if (fs->path[0] == '/' && fs->path[1] == '\0')
        {
            mountpath = fullpath;
        }
        else
        {
            if (fs->ops->flags & DFS_FS_FLAG_FULLPATH)
                mountpath = fullpath;
            else
                mountpath = dfs_subdir(fs->path, fullpath);
        }

        dfs_lock();
#ifdef DFS_USING_STDIO
        for (index = 3; index < DFS_FD_MAX+3; index++)
#else
        for (index = 0; index < DFS_FD_MAX; index++)
#endif
        {
            fd = &(fd_table[index]);
            if (fd->fs == RT_NULL)
                continue;

            if (fd->fs == fs && strcmp(fd->path, mountpath) == 0)
            {
                /* found file in file descriptor table */
                rt_free(fullpath);
                dfs_unlock();

                return 0;
            }
        }
        dfs_unlock();

        rt_free(fullpath);
    }

    return -1;
}

/**
 * this function will return a sub-path name under directory.
 *
 * @param directory the parent directory.
 * @param filename the filename.
 *
 * @return the subdir pointer in filename
 */
const char *dfs_subdir(const char *directory, const char *filename)
{
    const char *dir;

    if (strlen(directory) == strlen(filename)) /* it's a same path */
        return RT_NULL;

    dir = filename + strlen(directory);
    if ((*dir != '/') && (dir != filename))
    {
        dir --;
    }

    return dir;
}
RTM_EXPORT(dfs_subdir);

/**
 * this function will normalize a path according to specified parent directory
 * and file name.
 *
 * @param directory the parent path
 * @param filename the file name
 *
 * @return the built full file path (absolute path)
 */
char *dfs_normalize_path(const char *directory, const char *filename)
{
    char *fullpath;
    char *dst0, *dst, *src;

    /* check parameters */
    RT_ASSERT(filename != RT_NULL);

#ifdef DFS_USING_WORKDIR
    if (directory == RT_NULL) /* shall use working directory */
        directory = &working_directory[0];
#else
    if ((directory == RT_NULL) && (filename[0] != '/'))
    {
        rt_kprintf(NO_WORKING_DIR);

        return RT_NULL;
    }
#endif

    if (filename[0] != '/') /* it's a absolute path, use it directly */
    {
        fullpath = rt_malloc(strlen(directory) + strlen(filename) + 2);

        if (fullpath == RT_NULL)
            return RT_NULL;

        /* join path and file name */
        rt_snprintf(fullpath, strlen(directory) + strlen(filename) + 2,
            "%s/%s", directory, filename);
    }
    else
    {
        fullpath = rt_strdup(filename); /* copy string */

        if (fullpath == RT_NULL)
            return RT_NULL;
    }

    src = fullpath;
    dst = fullpath;

    dst0 = dst;
    while (1)
    {
        char c = *src;

        if (c == '.')
        {
            if (!src[1]) src ++; /* '.' and ends */
            else if (src[1] == '/')
            {
                /* './' case */
                src += 2;

                while ((*src == '/') && (*src != '\0'))
                    src ++;
                continue;
            }
            else if (src[1] == '.')
            {
                if (!src[2])
                {
                    /* '..' and ends case */
                    src += 2;
                    goto up_one;
                }
                else if (src[2] == '/')
                {
                    /* '../' case */
                    src += 3;

                    while ((*src == '/') && (*src != '\0'))
                        src ++;
                    goto up_one;
                }
            }
        }

        /* copy up the next '/' and erase all '/' */
        while ((c = *src++) != '\0' && c != '/')
            *dst ++ = c;

        if (c == '/')
        {
            *dst ++ = '/';
            while (c == '/')
                c = *src++;

            src --;
        }
        else if (!c)
            break;

        continue;

up_one:
        dst --;
        if (dst < dst0)
        {
            rt_free(fullpath);
            return RT_NULL;
        }
        while (dst0 < dst && dst[-1] != '/')
            dst --;
    }

    *dst = '\0';

    /* remove '/' in the end of path if exist */
    dst --;
    if ((dst != fullpath) && (*dst == '/'))
        *dst = '\0';

    return fullpath;
}
RTM_EXPORT(dfs_normalize_path);

#ifdef DFS_USING_SELECT
static struct dfs_fd *dfs_file_isdevfs(int fileno)
{
	int devfiles = fileno+1;
	rt_err_t ret;
	rt_device_t device;
	struct dfs_fd *d = fd_get(fileno);
	//必须是devfs
	if (d == RT_NULL || d->fs == RT_NULL || rt_strcmp(d->fs->path,"/dev") != 0)
	{
		if (d)
			fd_put(d);
		return RT_NULL;
	}
	device = (rt_device_t)d->data;
	if (device == RT_NULL || device->type != RT_Device_Class_Char)
	{
		fd_put(d);
		return RT_NULL;
	}
	//设备必须支持CHKFILE而且文件句柄需要一致
	ret = rt_device_control(device,RT_DEVICE_CTRL_CHAR_CHKFILE,&devfiles);
	if (ret != RT_EOK || devfiles != fileno)
	{
		fd_put(d);
		return RT_NULL;
	}
	return d;
}

int dfs_file_select(int maxfdp,
		rt_uint32_t *readset,
		rt_uint32_t *writeset,
		rt_uint32_t *exceptset,
		rt_int32_t timeout)
{
	register rt_base_t temp;
	dfs_select_info_t sel;
	int i,j,count[3];
	rt_err_t ret;
	int result;

	if (maxfdp <= 0 || maxfdp > DFS_FD_MAX
		|| (readset == RT_NULL && writeset == RT_NULL && exceptset == RT_NULL))
		return -1;

	result = 0;
	count[0] = count[1] = count[2] = 0;
	//先读取一次别浪费了
	{rt_uint32_t recvset[3][DFS_FD_MAX+31/32] = {{0}};
	int status = RT_EOK;
	struct dfs_fd *d = RT_NULL;
	for (i=0; i<maxfdp; i++)
	{
		if (readset && (readset[i/32] & (1<<(i%32))))
		{
			d = dfs_file_isdevfs(i);
			if (d != RT_NULL)
			{
				int recvbyte = 0;
				rt_device_t device = (rt_device_t)d->data;
				status = rt_device_control(device,RT_DEVICE_CTRL_CHAR_GETREAD,&recvbyte);
				if (status == RT_EOK && recvbyte > 0)
				{
					recvset[0][i/32] |= (1<<(i%32));
					count[0]++;
				}
				fd_put(d);
			}
			if (d == RT_NULL || status != RT_EOK)
			{
				if (exceptset && (exceptset[i/32] & (1<<(i%32)))
						&& !(recvset[2][i/32] & (1<<(i%32))))
				{
					recvset[2][i/32] |= (1<<(i%32));
					count[2]++;
				}
			}
		}
		if (writeset && (writeset[i/32] & (1<<(i%32))))
		{
			d = dfs_file_isdevfs(i);
			if (d != RT_NULL)
			{
				int writebyte = 1;
				rt_device_t device = (rt_device_t)d->data;
				status = rt_device_control(device,RT_DEVICE_CTRL_CHAR_GETWRITE,&writebyte);
				if (status == RT_EOK && writebyte == 0)
				{
					recvset[1][i/32] |= (1<<(i%32));
					count[1]++;
				}
				fd_put(d);
			}
			if (d == RT_NULL || status != RT_EOK)
			{
				if (exceptset && (exceptset[i/32] & (1<<(i%32)))
						&& !(recvset[2][i/32] & (1<<(i%32))))
				{
					recvset[2][i/32] |= (1<<(i%32));
					count[2]++;
				}
			}
		}
		if (exceptset && (exceptset[i/32] & (1<<(i%32)))
				&& !(recvset[2][i/32] & (1<<(i%32))))
		{
			d = dfs_file_isdevfs(i);
			if (d == RT_NULL)
			{
				recvset[2][i/32] |= (1<<(i%32));
				count[2]++;
			}
			else
			{
				fd_put(d);
				d = RT_NULL;
			}
		}
	}
	//取最大值
	result = count[0];
	if (result < count[1])
		result = count[1];
	if (result < count[2])
		result = count[2];
	if (result > 0)
	{
		//复制触发数据
		if (readset)
			rt_memcpy(readset,recvset[0],maxfdp+31/32);
		if (writeset)
			rt_memcpy(writeset,recvset[1],maxfdp+31/32);
		if (exceptset)
			rt_memcpy(exceptset,recvset[2],maxfdp+31/32);
		return result;
	}}

	//创建对象
	sel = (dfs_select_info_t)rt_malloc(sizeof(struct dfs_select_info));
	if (sel == RT_NULL)
		return -1;

	{char name[100];
	rt_sprintf(name,"dfs_s_%d",dfs_select_semindex++);
	if (dfs_select_semindex>=1000)
		dfs_select_semindex = 0;
	rt_sem_init(&(sel->sem), name, 0, RT_IPC_FLAG_FIFO);}
	sel->maxfdp = maxfdp;
	sel->reqset[0] = readset;
	sel->reqset[1] = writeset;
	sel->reqset[2] = exceptset;
	rt_memset(sel->recvset,0,sizeof(sel->recvset));

    /* lock interrupt */
    temp = rt_hw_interrupt_disable();
    /* insert object into information object list */
    rt_list_insert_after(&(dfs_select_list.list), &(sel->list));
    /* unlock interrupt */
    rt_hw_interrupt_enable(temp);

    result = 0;
	count[0] = count[1] = count[2] = 0;
	//等待别人触发
	ret = rt_sem_take(&sel->sem,timeout);
	/* lock interrupt */
	temp = rt_hw_interrupt_disable();
	/* remove object into information object list */
	rt_list_remove(&(sel->list));
	/* unlock interrupt */
	rt_hw_interrupt_enable(temp);

	if (ret == RT_EOK)
	{
		//复制触发数据
		for (i=0; i<3; i++)
		{
			if (sel->reqset[i] == RT_NULL)
				continue;
			//计算返回个数
			rt_memcpy(sel->reqset[i],sel->recvset[i],maxfdp+31/32);
			for (j=0; j<maxfdp; j++)
			{
				if (sel->reqset[i][j/32] & (1<<(j%32)))
					count[i]++;
			}
		}
		//取最大值
		result = count[0];
		if (result < count[1])
			result = count[1];
		if (result < count[2])
			result = count[2];
	}

	rt_sem_detach(&(sel->sem));
	rt_free(sel);
    return result;
}
RTM_EXPORT(dfs_file_select);
#endif

/*@}*/

