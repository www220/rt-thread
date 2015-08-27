#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include "board.h"
#include "nand.h"

/*
 * Standard NAND flash commands
 */
#define NAND_CMD_READ0		0
#define NAND_CMD_READ1		1
#define NAND_CMD_RNDOUT		5
#define NAND_CMD_PAGEPROG	0x10
#define NAND_CMD_READOOB	0x50
#define NAND_CMD_ERASE1		0x60
#define NAND_CMD_STATUS		0x70
#define NAND_CMD_STATUS_MULTI	0x71
#define NAND_CMD_SEQIN		0x80
#define NAND_CMD_RNDIN		0x85
#define NAND_CMD_READID		0x90
#define NAND_CMD_ERASE2		0xd0
#define NAND_CMD_RESET		0xff

#if 1
#define OOB_SIZE        64
#define PAGE_DATA_SIZE  2048
#define PAGE_PER_BLOCK  64
#define ECC_SIZE       ((PAGE_DATA_SIZE) * 3 / 256)
#define BLOCK_NUM       1024
#else
#define OOB_SIZE        16
#define PAGE_DATA_SIZE  512
#define PAGE_PER_BLOCK  32
#define ECC_SIZE       ((PAGE_DATA_SIZE) * 3 / 256)
#define BLOCK_NUM       512
#endif

#define BLOCK_SIZE      (PAGE_SIZE * PAGE_PER_BLOCK)
#define PAGE_SIZE       (PAGE_DATA_SIZE + OOB_SIZE)

#define MIN_PROP_DELAY_IN_NS	(5)
#define MAX_PROP_DELAY_IN_NS	(9)

#define NFC_DMA_DESCRIPTOR_COUNT	(4)

static struct gpmi_nfc_timing  safe_timing = {
	.m_u8DataSetup		= 80,
	.m_u8DataHold		= 60,
	.m_u8AddressSetup		= 25,
	.m_u8HalfPeriods		= 0,
	.m_u8SampleDelay		= 6,
	.m_u8NandTimingState	= 0,
	.m_u8tREA		= -1,
	.m_u8tRLOH		= -1,
	.m_u8tRHOH		= -1,
};

static struct rt_mtd_nand_device _nanddrv_file_device;


/* read chip id */
static rt_err_t nanddrv_file_read_id(struct rt_mtd_nand_device *device)
{
    return 0x00;
}

/* read/write/move page */
static rt_err_t nanddrv_file_read_page(struct rt_mtd_nand_device *device,
                                       rt_off_t page,
                                       rt_uint8_t *data, rt_uint32_t data_len,
                                       rt_uint8_t *spare, rt_uint32_t spare_len)
{
    return RT_EOK;
}

static rt_err_t nanddrv_file_write_page(struct rt_mtd_nand_device *device,
                                        rt_off_t page,
                                        const rt_uint8_t *data, rt_uint32_t data_len,
                                        const rt_uint8_t *oob, rt_uint32_t spare_len)
{
    return RT_EOK;
}

static rt_err_t nanddrv_file_move_page(struct rt_mtd_nand_device *device, rt_off_t from, rt_off_t to)
{
    return RT_EOK;
}

/* erase block */
static rt_err_t nanddrv_file_erase_block(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    if (block > BLOCK_NUM) return -RT_EIO;

    /* add the start blocks */
    block = block + device->block_start;

    return RT_EOK;
}

const static struct rt_mtd_nand_driver_ops _ops =
{
    nanddrv_file_read_id,
    nanddrv_file_read_page,
    nanddrv_file_write_page,
    nanddrv_file_move_page,
    nanddrv_file_erase_block,
    RT_NULL,
    RT_NULL,
};

void rt_hw_mtd_nand_init(void)
{
    rt_uint16_t ecc_size;
    
	/* Reset the GPMI block. */
	gpmi_nfc_reset_block((void *)(CONFIG_GPMI_REG_BASE + HW_GPMI_CTRL0), 1);

	/* Choose NAND mode. */
	REG_CLR(CONFIG_GPMI_REG_BASE, HW_GPMI_CTRL1,
		BM_GPMI_CTRL1_GPMI_MODE);

	/* Set the IRQ polarity. */
	REG_SET(CONFIG_GPMI_REG_BASE, HW_GPMI_CTRL1,
		BM_GPMI_CTRL1_ATA_IRQRDY_POLARITY);

	/* Disable write protection. */
	REG_SET(CONFIG_GPMI_REG_BASE, HW_GPMI_CTRL1,
		BM_GPMI_CTRL1_DEV_RESET);

	/* Select BCH ECC. */
	REG_SET(CONFIG_GPMI_REG_BASE, HW_GPMI_CTRL1,
		BM_GPMI_CTRL1_BCH_MODE);

    ecc_size = (PAGE_DATA_SIZE) * 3 / 256;
    _nanddrv_file_device.plane_num = 2;
    _nanddrv_file_device.oob_size = OOB_SIZE;
    _nanddrv_file_device.oob_free = OOB_SIZE - ecc_size;
    _nanddrv_file_device.page_size = PAGE_DATA_SIZE;
    _nanddrv_file_device.pages_per_block = PAGE_PER_BLOCK;
    _nanddrv_file_device.block_start = 0;
    _nanddrv_file_device.block_end = BLOCK_NUM / 2;
    _nanddrv_file_device.block_total = _nanddrv_file_device.block_end - _nanddrv_file_device.block_start;
    _nanddrv_file_device.ops = &_ops;

    rt_mtd_nand_register_device("nand0", &_nanddrv_file_device);
}

void nand_init(void)
{
}

#if defined(RT_USING_FINSH)
#include <finsh.h>
void nand_eraseall()
{
    int index;
    for (index = 0; index < _nanddrv_file_device.block_total; index ++)
    {
        nanddrv_file_erase_block(&_nanddrv_file_device, index);
    }
}
FINSH_FUNCTION_EXPORT(nand_eraseall, erase all of block in the nand flash);

#endif //RT_USING_FINSH
