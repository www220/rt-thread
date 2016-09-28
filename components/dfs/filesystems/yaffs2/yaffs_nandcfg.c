#include <rtthread.h>
#include <rtdevice.h>
#include <dfs_fs.h>
#include <dfs_def.h>

#include "yaffsfs.h"
#include "yaffs_nandif.h"
#include "yaffs_trace.h"

/*
 * RT-Thread Device Interface for yaffs
 */
static int nand_init(struct yaffs_dev *dev)
{
	return YAFFS_OK;
}

static int nand_deinit(struct yaffs_dev *dev)
{
	return YAFFS_OK;
}

/* if block is good, return YAFFS_OK, else return YAFFS_FAIL */
static int nand_checkblock(struct yaffs_dev *dev, unsigned block)
{
	rt_err_t res;
	res = rt_mtd_nand_check_block(RT_MTD_NAND_DEVICE(dev->os_context), block);
	return res == RT_EOK ? YAFFS_OK : YAFFS_FAIL;
}
static int nand_markbadblk(struct yaffs_dev *dev, unsigned block)
{
	rt_err_t res;
	res = rt_mtd_nand_mark_badblock(RT_MTD_NAND_DEVICE(dev->os_context), block);
	return res == RT_EOK ? YAFFS_OK : YAFFS_FAIL;
}

static int nand_eraseblock(struct yaffs_dev *dev, unsigned block)
{
	int res;
	res = rt_mtd_nand_erase_block(RT_MTD_NAND_DEVICE(dev->os_context), block);
	return 	res == RT_EOK ? YAFFS_OK : YAFFS_FAIL;
}

static int nand_readpage(
	struct yaffs_dev *dev,
    unsigned page,
    unsigned char *data, unsigned data_len, /* page data */
    unsigned char *spare, unsigned spare_len,/* page spare */
    int *ecc_status)
{
	int res;
	unsigned char spare_buf[64];

	res = rt_mtd_nand_read(RT_MTD_NAND_DEVICE(dev->os_context),
	                   	   page, data, data_len, spare_buf, spare_len + 2);
	if (res == -RT_EIO)
        return YAFFS_FAIL;
    
	rt_memcpy(spare, spare_buf + 2, spare_len);
	if (res == RT_EOK)
		*ecc_status = 0;
	else if (res == -RT_ERROR)
		*ecc_status = -1;
	else
		*ecc_status = 1;

	return YAFFS_OK;
}

static int nand_writepage(
	struct yaffs_dev *dev,
	unsigned page,
	const unsigned char *data, unsigned data_len, /* page data */
    const unsigned char *spare, unsigned spare_len) /* page spare */
{
	int res;
	unsigned char spare_buf[64]; //not use malloc, this can be faster
	rt_memset(spare_buf, 0xFF, sizeof(spare_buf));
	rt_memcpy(spare_buf+2, spare, spare_len);

	res = rt_mtd_nand_write(RT_MTD_NAND_DEVICE(dev->os_context),
						   page, data, data_len, spare_buf, spare_len + 2);
	if (res == -RT_EIO)
		return YAFFS_FAIL;

	return YAFFS_OK;
}


/*
 * yaffs2 configuration
 */
#define CONFIG_YAFFS_ECC_MODE     1 //1 use ecc, 0 no ecc
#define CONFIG_YAFFS_INBAND_TAGS  0 //1 use in band tags, 0-no in band tags
#define CONFIG_YAFFS_USE_YAFFS2   1 //1 yaffs2, 0-yaffs1

void yaffs_config(ynandif_Geometry * g, struct rt_mtd_nand_device * dev)
{
	rt_memset(g,0,sizeof(ynandif_Geometry));

	g->start_block   = dev->block_start;
	g->end_block     = dev->block_end-1;
	g->dataSize      = dev->page_size;
	g->spareSize     = dev->oob_size;
	g->pagesPerBlock = dev->pages_per_block;

	g->hasECC        = CONFIG_YAFFS_ECC_MODE;
	g->inband_tags   = CONFIG_YAFFS_INBAND_TAGS;
	g->useYaffs2     = CONFIG_YAFFS_USE_YAFFS2;

	g->privateData   = dev;//will be copy to dev->os_context.

	g->initialise = nand_init;
	g->deinitialise = nand_deinit;
	g->readChunk = nand_readpage;
	g->writeChunk = nand_writepage;
	g->eraseBlock = nand_eraseblock;
	g->checkBlockOk = nand_checkblock;
	g->markBlockBad = nand_markbadblk;
}

/* configuration for yaffs's log */
unsigned yaffs_trace_mask =

	//YAFFS_TRACE_SCAN |
	//YAFFS_TRACE_GC |
	//YAFFS_TRACE_ERASE |
	YAFFS_TRACE_ERROR |
	//YAFFS_TRACE_TRACING |
	//YAFFS_TRACE_ALLOCATE |
	YAFFS_TRACE_BAD_BLOCKS |
	//YAFFS_TRACE_VERIFY |

	0;

void* yaffs_start(const char * mount_point, ynandif_Geometry * g)
{
	// Stuff to configure YAFFS
	// Stuff to initialise anything special (eg lock semaphore).
	yaffsfs_OSInitialisation();
	return yaffs_add_dev_from_geometry(mount_point, g);
}
