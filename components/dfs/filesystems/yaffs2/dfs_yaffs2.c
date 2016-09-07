/*
 * File      : rtthread.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2011, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-10-22     prife        the first version
 * 2012-04-14     prife        use mtd device interface
*/
#include <rtthread.h>
#include <rtdevice.h>

#include "yaffsfs.h"
#include "yaffs_nandif.h"

#include <dfs_fs.h>
#include <dfs_def.h>

#define YAFFS_FILE_PATH_MAX	             256 /* the longest file path */

#define NAND_DEVICE_PART_MAX   4  /* the max partitions on a nand deivce*/
struct device_part
{
	struct rt_mtd_nand_device *dev;
	ynandif_Geometry g;
    void *yaffs_dev;
};
static struct device_part nand_part[NAND_DEVICE_PART_MAX] = {{0}};

static int nand_find_dev = -1;
void yaffs_dev_rewind() { nand_find_dev = -1; }
void* yaffs_next_dev() 
{
 	for (nand_find_dev ++; nand_find_dev < NAND_DEVICE_PART_MAX ; nand_find_dev ++)
	{
		if (nand_part[nand_find_dev].dev != RT_NULL
            && nand_part[nand_find_dev].yaffs_dev != RT_NULL)
			return nand_part[nand_find_dev].yaffs_dev;
	}
    return RT_NULL;
}

extern void* yaffs_start(const char * mountpath, ynandif_Geometry * g);
extern void yaffs_config(ynandif_Geometry * g, struct rt_mtd_nand_device * dev);

static int dfs_yaffs_mount(struct dfs_filesystem* fs,
                    unsigned long rwflag, 
				    const void* data)
{
	unsigned index;

	/*1. find a empty entry in partition table */
	for (index = 0; index < NAND_DEVICE_PART_MAX ; index ++)
	{
		if (nand_part[index].dev == RT_NULL)
			break;
	}
	if (index == NAND_DEVICE_PART_MAX)
		return -DFS_STATUS_ENOSPC;

	/*2. fill nand_part*/
	nand_part[index].dev = RT_MTD_NAND_DEVICE(fs->dev_id);
	yaffs_config(&nand_part[index].g, RT_MTD_NAND_DEVICE(fs->dev_id));

	/*3. start up yaffs2 */
	nand_part[index].yaffs_dev = yaffs_start(fs->path, &nand_part[index].g);
	if (yaffs_mount(fs->path) < 0)
	{	
		return yaffsfs_GetLastError();
	}
	return 0;
}

static int dfs_yaffs_unmount(struct dfs_filesystem* fs)
{
	unsigned index;
	if (yaffs_unmount(fs->path) < 0)
		return yaffsfs_GetLastError();

	/* find device index, then umount it */
	for (index = 0; index < NAND_DEVICE_PART_MAX; index++)
	{
		if (nand_part[index].dev == RT_MTD_NAND_DEVICE(fs->dev_id))
		{
			nand_part[index].dev = RT_NULL;
			return DFS_STATUS_OK;
		}
	}
	return -DFS_STATUS_ENOENT;
}

static int dfs_yaffs_mkfs(rt_device_t devid)
{
	/* just erase all blocks on this nand partition */
	return -DFS_STATUS_ENOSYS; 
}

static int dfs_yaffs_statfs(struct dfs_filesystem* fs,
                     struct statfs *buf)
{
	struct rt_mtd_nand_device * mtd = RT_MTD_NAND_DEVICE(fs->dev_id);

	RT_ASSERT(mtd != RT_NULL);

	buf->f_bsize = mtd->page_size;
	buf->f_blocks = mtd->pages_per_block*mtd->block_total;
	buf->f_bfree = yaffs_freespace(fs->path) / mtd->page_size;

	return 0;
}

static int dfs_yaffs_open(struct dfs_fd* file)
{
	int fd;
	int oflag;
	int result;
		
	oflag = file->flags;
	if (oflag & DFS_O_DIRECTORY) /* operations about dir */
	{
		yaffs_DIR * dir;
		if (oflag & DFS_O_CREAT) /* create a dir*/
		{	
			result = yaffs_mkdir(file->path, 0x777); /* the second args not supported by rtt */
			if (result < 0)
				return yaffsfs_GetLastError();
		}
		/* open dir */
		dir = yaffs_opendir(file->path);
		if (dir == RT_NULL)
			return yaffsfs_GetLastError();			
		/* save this pointer,will used by dfs_yaffs_getdents*/
		file->data = dir;
		return 0;
	} 
	/* regular file operations */
	fd = yaffs_open(file->path, oflag, S_IREAD | S_IWRITE); 	
	if (fd < 0)
		return yaffsfs_GetLastError();

	 /* save this pointer, it will be used when calling read()£¬write(), 
	 flush(), lessk(), and will be free when calling close()*/
	file->data = (void *)fd;
	file->pos  = yaffs_lseek(fd,0,SEEK_CUR); 
	file->size = yaffs_lseek(fd,0,SEEK_END);
	yaffs_lseek(fd, file->pos, SEEK_SET);

	if (oflag & DFS_O_APPEND)
	{
		file->pos = file->size;	
		file->size = yaffs_lseek(fd,0,SEEK_END);
	}
	return 0; 
}

static int dfs_yaffs_close(struct dfs_fd* file)
{
	int oflag;
	int fd;
	
	oflag = file->flags;
	if (oflag & DFS_O_DIRECTORY) /* operations about dir */
	{
		if (yaffs_closedir((yaffs_DIR *)(file->data)) < 0)
			return yaffsfs_GetLastError();	
		return 0;
	}
	/* regular file operations */
	fd = (int)(file->data);

	if (yaffs_close(fd) == 0)
		return 0;

	/* release memory */
	return yaffsfs_GetLastError();
}

static int dfs_yaffs_ioctl(struct dfs_fd* file, int cmd,	void* args)
{
	return -DFS_STATUS_ENOSYS;
}

static int dfs_yaffs_read(struct dfs_fd* file, void* buf, rt_size_t len)
{
	int fd;
	int char_read;
	
	fd = (int)(file->data);
	char_read = yaffs_read(fd, buf, len);
	if (char_read < 0) 
		return yaffsfs_GetLastError();
	
	/* update position */
	file->pos = yaffs_lseek(fd, 0, SEEK_CUR);
	if (file->size < file->pos)
		file->size = file->pos;
	return char_read;
}

static int dfs_yaffs_write(struct dfs_fd* file,
                    const void* buf, 
					rt_size_t len)
{
	int fd;
	int char_write;
	
	fd = (int)(file->data);
	
	char_write = yaffs_write(fd, buf, len);
	if (char_write < 0) 
		return yaffsfs_GetLastError();
	
	/* update position */
	file->pos = yaffs_lseek(fd, 0, SEEK_CUR);
	if (file->size < file->pos)
		file->size = file->pos;
	return char_write;
}

static int dfs_yaffs_flush(struct dfs_fd* file)
{
	int fd;
	int result;
	
	fd = (int)(file->data);

	result = yaffs_flush(fd);
	if (result < 0)
		return yaffsfs_GetLastError();
	return 0;
}

static int dfs_yaffs_lseek(struct dfs_fd* file,
                    rt_off_t offset)
{
	int fd;
	int result;
	
	fd = (int)(file->data);

	/* set offset as current offset */
	result = yaffs_lseek(fd, offset, SEEK_SET);
	if (result < 0)
		return yaffsfs_GetLastError();
	file->pos = result;
	if (file->size < file->pos)
		file->size = file->pos;
	return result;
}

/* return the size of  struct dirent*/
static int dfs_yaffs_getdents(struct dfs_fd* file,
                       struct dirent* dirp, 
					   rt_uint32_t count)
{
	rt_uint32_t index;
	char * file_path;	
	struct dirent* d;
	yaffs_DIR* dir;
	struct yaffs_dirent * yaffs_d;	
		
	dir = (yaffs_DIR*)(file->data);
	RT_ASSERT(dir != RT_NULL);

	/* make integer count, usually count is 1 */
	count = (count / sizeof(struct dirent)) * sizeof(struct dirent);
	if (count == 0) return -DFS_STATUS_EINVAL;

	/* allocate file name */
	file_path = rt_malloc(YAFFS_FILE_PATH_MAX);
	if (file_path == RT_NULL)
		return -DFS_STATUS_ENOMEM;
	
	index = 0;
	/* usually, the while loop should only be looped only once! */
	while (1)
	{
		struct yaffs_stat s; 
		
		d = dirp + index;

		yaffs_d = yaffs_readdir(dir);
		if (yaffs_d == RT_NULL)
		{
			rt_free(file_path);
			return yaffsfs_GetLastError();
		}
		
		rt_snprintf(file_path, YAFFS_FILE_PATH_MAX, "%s/%s", file->path, yaffs_d->d_name);

		yaffs_lstat(file_path, &s); 	 
		switch(s.st_mode & S_IFMT) 
		{ 
		case S_IFREG: d->d_type = DFS_DT_REG; break; 
		
		case S_IFDIR: d->d_type = DFS_DT_DIR; break; 
		
		case S_IFLNK:
		default: d->d_type = DFS_DT_UNKNOWN; break; 
		} 	

		/* write the rest feilds of struct dirent* dirp  */
		d->d_namlen = rt_strlen(yaffs_d->d_name);
		d->d_reclen = (rt_uint16_t)sizeof(struct dirent);
		rt_strncpy(d->d_name, yaffs_d->d_name, rt_strlen(yaffs_d->d_name) + 1);
		
		index ++;
		if (index * sizeof(struct dirent) >= count)
			break;
	}

	/* free file name buf */
	rt_free(file_path);
	
	if (index == 0)
		return yaffsfs_GetLastError();

	return index * sizeof(struct dirent);
}

static int dfs_yaffs_unlink(struct dfs_filesystem* fs, const char* path)
{
	int result;
	struct yaffs_stat s;

	/* judge file type, dir is to be delete by yaffs_rmdir, others by yaffs_unlink */
	if (yaffs_lstat(path, &s) < 0) 
	{
		return yaffsfs_GetLastError();
	}
	
	switch(s.st_mode & S_IFMT) 
	{ 
	case S_IFREG: 
		result = yaffs_unlink(path); 
		break; 	
	case S_IFDIR: 
		result = yaffs_rmdir(path);		
		break;
	default: 
		/* unknown file type */
		return -1;
	} 
	if (result < 0) 
		return yaffsfs_GetLastError();
	return 0;			
}

static int dfs_yaffs_rename(struct dfs_filesystem* fs,
                     const char* oldpath,
					 const char* newpath)
{
	int result;

	result = yaffs_rename(oldpath, newpath);

	if (result < 0)
		return yaffsfs_GetLastError();
	return 0;
}

static int dfs_yaffs_stat(struct dfs_filesystem* fs, const char *path, struct stat *st)
{
	int result;
	struct yaffs_stat s;
	struct rt_mtd_nand_device * mtd;
	
	result = yaffs_stat(path, &s);
	if (result < 0)
		return yaffsfs_GetLastError(); 

	/* convert to dfs stat structure */
	st->st_dev  = 0;
	st->st_mode = s.st_mode;
	st->st_size = s.st_size;
	st->st_mtime = s.yst_mtime;

	mtd = RT_MTD_NAND_DEVICE(fs->dev_id);
	st->st_blksize = mtd->page_size;

	return 0;
}

static int dfs_yaffs_truncate(struct dfs_fd* file,
                    rt_off_t length)
{
	int fd;
	int result;

	fd = (int)(file->data);

	/* set length as new length */
	result = yaffs_ftruncate(fd, length);
	if (result < 0)
		return yaffsfs_GetLastError();
	/* update position */
	file->pos = yaffs_lseek(fd, 0, SEEK_CUR);
	file->size = length;
	return result;
}

static int dfs_yaffs_lstat(struct dfs_filesystem* fs, const char *path, struct stat *st)
{
	int result;
	struct yaffs_stat s;
	struct rt_mtd_nand_device * mtd;

	result = yaffs_lstat(path, &s);
	if (result < 0)
		return yaffsfs_GetLastError();

	/* convert to dfs stat structure */
	st->st_dev  = 0;
	st->st_mode = s.st_mode;
	st->st_size = s.st_size;
	st->st_mtime = s.yst_mtime;

	mtd = RT_MTD_NAND_DEVICE(fs->dev_id);
	st->st_blksize = mtd->page_size;

	return 0;
}

static int dfs_yaffs_link(struct dfs_filesystem* fs, const char *oldpath, const char *new)
{
	int result;

	result = yaffs_link(oldpath, new);
	if (result < 0)
		return yaffsfs_GetLastError();

	return 0;
}

static int dfs_yaffs_symlink(struct dfs_filesystem* fs, const char *oldpath, const char *new)
{
	int result;

	result = yaffs_symlink(oldpath, new);
	if (result < 0)
		return yaffsfs_GetLastError();

	return 0;
}

static int dfs_yaffs_readlink(struct dfs_filesystem* fs, const char *path, char *buf, rt_size_t bufsiz)
{
	int result;

	result = yaffs_readlink(path, buf, bufsiz);
	if (result < 0)
		return yaffsfs_GetLastError();

	return 0;
}

static const struct dfs_filesystem_operation dfs_yaffs_ops = 
{
	"yaffs2", /* file system type: yaffs2 */
#if RTTHREAD_VERSION >= 10100
	DFS_FS_FLAG_FULLPATH,
#else
#error "yaffs2 can only work with rtthread whose version should >= 1.01\n"
#endif
	dfs_yaffs_mount,
	dfs_yaffs_unmount,
	dfs_yaffs_mkfs,
	dfs_yaffs_statfs,

	dfs_yaffs_open,
	dfs_yaffs_close,
	dfs_yaffs_ioctl,
	dfs_yaffs_read,
	dfs_yaffs_write,
	dfs_yaffs_flush,
	dfs_yaffs_lseek,
	dfs_yaffs_getdents,
	dfs_yaffs_unlink,
	dfs_yaffs_stat,
	dfs_yaffs_rename,
	dfs_yaffs_truncate,
	dfs_yaffs_lstat,
	dfs_yaffs_link,
	dfs_yaffs_symlink,
	dfs_yaffs_readlink,
};

int dfs_yaffs_init(void)
{
    /* register fatfs file system */
    dfs_register(&dfs_yaffs_ops);

	return 0;
}
