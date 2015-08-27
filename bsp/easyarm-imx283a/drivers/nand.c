#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include "board.h"
#include "nand.h"
#include "apbh_dma.h"
#include "bch.h"

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

#define NFC_DMA_DESCRIPTOR_COUNT	(4)
static struct mxs_dma_desc *dma_desc[NFC_DMA_DESCRIPTOR_COUNT];
static struct rt_mtd_nand_device _nanddrv_file_device;

/**
 * clear_bch() - Clears a BCH interrupt.
 *
 * @this:  Per-device data.
 */
static void clear_bch(struct rt_mtd_nand_device *mtd)
{
	REG_CLR(CONFIG_BCH_REG_BASE, HW_BCH_CTRL,
		BM_BCH_CTRL_COMPLETE_IRQ);
}

/**
 * send_command() - Sends a command and associated addresses.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that contains the command bytes.
 * @length:  The number of bytes in the buffer.
 */
static int send_command(struct rt_mtd_nand_device *mtd, unsigned chip,
			dma_addr_t buffer, unsigned int length)
{
	struct mxs_dma_desc **d = dma_desc;
	s32 dma_channel;
	s32 error;
	u32 command_mode;
	u32 address;

	/* Compute the DMA channel. */
	dma_channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + chip;

	/* A DMA descriptor that sends out the command. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_CLE;

	/* reset the cmd bits fieled */
	(*d)->cmd.cmd.data                   = 0;

	(*d)->cmd.cmd.bits.command           = DMA_READ;
#if defined(CONFIG_GPMI_NFC_V2)
	(*d)->cmd.cmd.bits.chain             = 0;
#else
	(*d)->cmd.cmd.bits.chain             = 1;
#endif
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
#if defined(CONFIG_GPMI_NFC_V2)
	(*d)->cmd.cmd.bits.halt_on_terminate = 1;
#else
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
#endif
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 3;
	(*d)->cmd.cmd.bits.bytes             = length;

#ifdef CONFIG_ARCH_MMU
	(*d)->cmd.address = iomem_to_phys(buffer);
#else
	(*d)->cmd.address = buffer;
#endif

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BM_GPMI_CTRL0_ADDRESS_INCREMENT          |
		BF_GPMI_CTRL0_XFER_COUNT(length)         ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */
	error = mxs_dma_go(dma_channel);

	if (error)
		printf("[%s] DMA error\n", __func__);

	/* Return success. */
	return error;
}

/**
 * send_data() - Sends data to the given chip.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that contains the data.
 * @length:  The number of bytes in the buffer.
 */
static int send_data(struct rt_mtd_nand_device *mtd, unsigned chip,
			dma_addr_t buffer, unsigned length)
{
	struct mxs_dma_desc	**d  = dma_desc;
	int			dma_channel;
	int			error = 0;
	u32			command_mode;
	u32			address;

	/* Compute the DMA channel. */
	dma_channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + chip;

	/* A DMA descriptor that writes a buffer out. */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = DMA_READ;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 4;
	(*d)->cmd.cmd.bits.bytes             = length;

#ifdef CONFIG_ARCH_MMU
	(*d)->cmd.address = iomem_to_phys(buffer);
#else
	(*d)->cmd.address = buffer;
#endif

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(length)         ;
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;
	(*d)->cmd.pio_words[3] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */
	error = mxs_dma_go(dma_channel);

	if (error)
		printf("[%s] DMA error\n", __func__);

	/* Return success. */
	return error;

}

/**
 * read_data() - Receives data from the given chip.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that will receive the data.
 * @length:  The number of bytes to read.
 */
static int read_data(struct rt_mtd_nand_device *mtd, unsigned chip,
			dma_addr_t buffer, unsigned int length)
{
	struct mxs_dma_desc  **d        = dma_desc;
	int                  dma_channel;
	int                  error = 0;
	uint32_t             command_mode;
	uint32_t             address;

	/* Compute the DMA channel. */
	dma_channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + chip;

	/* A DMA descriptor that reads the data. */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = DMA_WRITE;
#if !defined(CONFIG_GPMI_NFC_V0)
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
#else
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
#endif
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
#if defined(CONFIG_GPMI_NFC_V2)
	(*d)->cmd.cmd.bits.halt_on_terminate = 1;
#else
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
#endif
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 1;
	(*d)->cmd.cmd.bits.bytes             = length;

#ifdef CONFIG_ARCH_MMU
	(*d)->cmd.address = iomem_to_phys(buffer);
#else
	(*d)->cmd.address = buffer;
#endif

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(length)         ;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

#if !defined(CONFIG_GPMI_NFC_V2)
	/*
	 * A DMA descriptor that waits for the command to end and the chip to
	 * become ready.
	 *
	 * I think we actually should *not* be waiting for the chip to become
	 * ready because, after all, we don't care. I think the original code
	 * did that and no one has re-thought it yet.
	 */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 1;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 4;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(0)              ;
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;
	(*d)->cmd.pio_words[3] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;
#endif
	/* Go! */
	error = mxs_dma_go(dma_channel);

	if (error)
		printf("[%s] DMA error\n", __func__);

#ifdef CONFIG_MTD_DEBUG
	{
		int i;
		dma_addr_t *tmp_buf_ptr = (dma_addr_t *)buffer;

		printf("Buffer:");
		for (i = 0; i < length; ++i)
			printf("0x%08x ", tmp_buf_ptr[i]);
		printf("\n");
	}
#endif

	/* Return success. */
	return error;

}

int wait_for_bch_completion(u32 timeout)
{
	while ((!(REG_RD(CONFIG_BCH_REG_BASE, HW_BCH_CTRL) & 0x1)) &&
			--timeout)
		;

	return (timeout > 0) ? 0 : 1;
}

/**
 * send_page() - Sends a page, using ECC.
 *
 * @this:       Per-device data.
 * @chip:       The chip of interest.
 * @payload:    The physical address of the payload buffer.
 * @auxiliary:  The physical address of the auxiliary buffer.
 */
static int send_page(struct rt_mtd_nand_device *mtd, unsigned chip,
				dma_addr_t payload, dma_addr_t auxiliary)
{
	struct mxs_dma_desc  **d        = dma_desc;
	int                  dma_channel;
	int                  error = 0;
	uint32_t             command_mode;
	uint32_t             address;
	uint32_t             ecc_command;
	uint32_t             buffer_mask;

	/* Compute the DMA channel. */
	dma_channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + chip;

	/* A DMA descriptor that does an ECC page read. */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
#if defined(CONFIG_GPMI_NFC_V0)
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__BCH_ENCODE;
#else
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__ENCODE;
#endif
	buffer_mask  = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
			BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 6;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(0)              ;

	(*d)->cmd.pio_words[1] = 0;

	(*d)->cmd.pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC               |
		BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;

	(*d)->cmd.pio_words[3] = (mtd->page_size + mtd->oob_size);
#ifdef CONFIG_ARCH_MMU
	(*d)->cmd.pio_words[4] = iomem_to_phys(payload);
	(*d)->cmd.pio_words[5] = iomem_to_phys(auxiliary);
#else
	(*d)->cmd.pio_words[4] = payload;
	(*d)->cmd.pio_words[5] = auxiliary;
#endif

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */
	error = mxs_dma_go(dma_channel);

	if (error)
		printf("[%s] DMA error\n", __func__);

	error = wait_for_bch_completion(10000);

	error = (error) ? -ETIMEDOUT : 0;

	if (error)
		printf("[%s] bch timeout!!!\n", __func__);

	clear_bch(NULL);

	/* Return success. */
	return error;
}

/**
 * read_page() - Reads a page, using ECC.
 *
 * @this:       Per-device data.
 * @chip:       The chip of interest.
 * @payload:    The physical address of the payload buffer.
 * @auxiliary:  The physical address of the auxiliary buffer.
 */
static int read_page(struct rt_mtd_nand_device *mtd, unsigned chip,
			dma_addr_t payload, dma_addr_t auxiliary)
{
	struct mxs_dma_desc	**d        = dma_desc;
	s32			dma_channel;
	s32			error = 0;
	u32			command_mode;
	u32			address;
	u32			ecc_command;
	u32			buffer_mask;
	u32			page_size = mtd->page_size + mtd->oob_size;

	dma_channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + chip;

	/* Wait for the chip to report ready. */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 1;
#if !defined(CONFIG_GPMI_NFC_V0)
	(*d)->cmd.cmd.bits.dec_sem           = 0;
#else
	(*d)->cmd.cmd.bits.dec_sem           = 1;
#endif
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 1;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(0)              ;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Enable the BCH block and read. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
#if defined(CONFIG_GPMI_NFC_V0)
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__BCH_DECODE;
#else
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__DECODE;
#endif
	buffer_mask  = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
			BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
#if !defined(CONFIG_GPMI_NFC_V0)
	(*d)->cmd.cmd.bits.dec_sem           = 0;
#else
	(*d)->cmd.cmd.bits.dec_sem           = 1;
#endif
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 6;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode)              |
		BM_GPMI_CTRL0_WORD_LENGTH                             |
		BF_GPMI_CTRL0_CS(chip)                                |
		BF_GPMI_CTRL0_ADDRESS(address)                        |
		BF_GPMI_CTRL0_XFER_COUNT(page_size) ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC	|
		BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;
	(*d)->cmd.pio_words[3] = page_size;
#ifdef CONFIG_ARCH_MMU
	(*d)->cmd.pio_words[4] = iomem_to_phys(payload);
	(*d)->cmd.pio_words[5] = iomem_to_phys(auxiliary);
#else
	(*d)->cmd.pio_words[4] = payload;
	(*d)->cmd.pio_words[5] = auxiliary;
#endif

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Disable the BCH block */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 1;
#if !defined(CONFIG_GPMI_NFC_V0)
	(*d)->cmd.cmd.bits.dec_sem           = 0;
#else
	(*d)->cmd.cmd.bits.dec_sem           = 1;
#endif
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 3;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode)              |
		BM_GPMI_CTRL0_WORD_LENGTH                             |
		BF_GPMI_CTRL0_CS(chip)                                |
		BF_GPMI_CTRL0_ADDRESS(address)                        |
		BF_GPMI_CTRL0_XFER_COUNT(page_size) ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Deassert the NAND lock and interrupt. */

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 0;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 0;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */
	error = mxs_dma_go(dma_channel);

	if (error)
		printf("[%s] DMA error\n", __func__);

	error = wait_for_bch_completion(10000);

	error = (error) ? -ETIMEDOUT : 0;

	if (error)
		printf("[%s] bch timeout!!!\n", __func__);

	clear_bch(NULL);

	/* Return success. */
	return error;
}

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
    int i;    
	for (i = 0; i < NFC_DMA_DESCRIPTOR_COUNT; ++i) {
		dma_desc[i] = mxs_dma_alloc_desc();

		if (NULL == dma_desc[i]) {
			for (i -= 1; i >= 0; --i)
				mxs_dma_free_desc(dma_desc[i]);
			return;
		}
	}

	mxs_dma_init();
    
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

        {
    char buf[4] = {0x90,00};
    send_command(&_nanddrv_file_device,0,buf,2);
    udelay(1000);
    read_data(&_nanddrv_file_device,0,buf,2);
    rt_kprintf("%x %x %x",buf[0],buf[1]);
        }
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
