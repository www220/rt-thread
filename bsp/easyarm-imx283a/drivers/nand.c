#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "nand.h"
#include "apbh_dma.h"
#include "bch.h"

/*
 * Constants for hardware specific CLE/ALE/NCE function
 *
 * These are bits which can be or'ed to set/clear multiple
 * bits in one go.
 */
/* Select the chip by setting nCE to low */
#define NAND_NCE		0x01
/* Select the command latch by setting CLE to high */
#define NAND_CLE		0x02
/* Select the address latch by setting ALE to high */
#define NAND_ALE		0x04

#define NAND_CTRL_CLE		(NAND_NCE | NAND_CLE)
#define NAND_CTRL_ALE		(NAND_NCE | NAND_ALE)
#define NAND_CTRL_CHANGE	0x80

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

#define NAND_CMD_NONE		-1

/* Status bits */
#define NAND_STATUS_FAIL	0x01
#define NAND_STATUS_FAIL_N1	0x02
#define NAND_STATUS_TRUE_READY	0x20
#define NAND_STATUS_READY	0x40
#define NAND_STATUS_WP		0x80

/*
 * CONFIG_SYS_NAND_RESET_CNT is used as a timeout mechanism when resetting
 * a flash.  NAND flash is initialized prior to interrupts so standard timers
 * can't be used.  CONFIG_SYS_NAND_RESET_CNT should be set to a value
 * which is greater than (max NAND reset time / NAND status read time).
 * A conservative default of 200000 (500 us / 25 ns) is used as a default.
 */
#ifndef CONFIG_SYS_NAND_RESET_CNT
#define CONFIG_SYS_NAND_RESET_CNT 200000
#endif

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
 * is_ready() - Returns the ready/busy status of the given chip.
 *
 * @this:  Per-device data.
 * @chip:  The chip of interest.
 */
static int is_ready(struct rt_mtd_nand_device *mtd, unsigned int target_chip)
{
	u32 mask;
	u32 register_image;

	/* Extract and return the status. */
#if defined(CONFIG_GPMI_NFC_V0)
	mask = BM_GPMI_DEBUG_READY0 << target_chip;

	register_image = REG_RD(CONFIG_GPMI_REG_BASE, HW_GPMI_DEBUG);
#else
	mask = BF_GPMI_STAT_READY_BUSY(1 << 0);

	register_image = REG_RD(CONFIG_GPMI_REG_BASE, HW_GPMI_STAT);
#endif

	return register_image & mask;
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

/**
 * cmd_ctrl - MTD Interface cmd_ctrl()
 *
 * This is the function that we install in the cmd_ctrl function pointer of the
 * owning struct nand_chip. The only functions in the reference implementation
 * that use these functions pointers are cmdfunc and select_chip.
 *
 * In this driver, we implement our own select_chip, so this function will only
 * be called by the reference implementation's cmdfunc. For this reason, we can
 * ignore the chip enable bit and concentrate only on sending bytes to the
 * NAND Flash.
 *
 * @mtd:   The owning MTD.
 * @data:  The value to push onto the data signals.
 * @ctrl:  The values to push onto the control signals.
 */
static void cmd_ctrl(struct rt_mtd_nand_device *mtd, int data, unsigned int ctrl)
{
	int error;
	static u8 *cmd_queue = 0;
	static u32 cmd_Q_len = 0;
#if defined(CONFIG_MTD_DEBUG)
	unsigned int          i;
	char                  display[GPMI_NFC_COMMAND_BUFFER_SIZE * 5];
#endif

	if (!cmd_queue) {
#ifdef CONFIG_ARCH_MMU
		cmd_queue =
		(u8 *)ioremap_nocache((u32)iomem_to_phys((ulong)memalign(MXS_DMA_ALIGNMENT,
		GPMI_NFC_COMMAND_BUFFER_SIZE)),
		MXS_DMA_ALIGNMENT);
#else
		cmd_queue =
		memalign(MXS_DMA_ALIGNMENT, GPMI_NFC_COMMAND_BUFFER_SIZE);
#endif
		if (!cmd_queue) {
			printf("%s: failed to allocate command "
				"queuebuffer\n",
				__func__);
			return;
		}
        
        // mmu nocache
        cmd_queue = (u8 *)(0x8000000|((u32)cmd_queue));
        
		memset(cmd_queue, 0, GPMI_NFC_COMMAND_BUFFER_SIZE);
		cmd_Q_len = 0;
	}

	/*
	 * Every operation begins with a command byte and a series of zero or
	 * more address bytes. These are distinguished by either the Address
	 * Latch Enable (ALE) or Command Latch Enable (CLE) signals being
	 * asserted. When MTD is ready to execute the command, it will
	 * deasert both latch enables.
	 *
	 * Rather than run a separate DMA operation for every single byte, we
	 * queue them up and run a single DMA operation for the entire series
	 * of command and data bytes.
	 */

	if ((ctrl & (NAND_ALE | NAND_CLE))) {
		if (data != NAND_CMD_NONE)
			cmd_queue[cmd_Q_len++] = data;
		return;
	}

	/*
	 * If control arrives here, MTD has deasserted both the ALE and CLE,
	 * which means it's ready to run an operation. Check if we have any
	 * bytes to send.
	 */

	if (!cmd_Q_len)
		return;

#if defined(CONFIG_MTD_DEBUG)
	display[0] = 0;
	for (i = 0; i < cmd_Q_len; i++)
		sprintf(display + strlen(display),
			" 0x%02x", cmd_queue[i] & 0xff);
	MTDDEBUG(MTD_DEBUG_LEVEL1, "%s: command: %s\n", __func__, display);
#endif

#ifdef CONFIG_ARCH_MMU
	error = send_command(mtd, 0,
		(dma_addr_t)iomem_to_phys((u32)cmd_queue), cmd_Q_len);
#else
	error = send_command(mtd, 0,
		(dma_addr_t)cmd_queue, cmd_Q_len);
#endif

	if (error)
		printf("Command execute failed!\n");

	/* Reset. */
	cmd_Q_len = 0;
}

/*
 * Wait for the ready pin, after a command
 * The timeout is catched later.
 */
void nand_wait_ready(struct rt_mtd_nand_device *mtd)
{
	/* wait until command is processed or timeout occures */
	u32 timeo = rt_tick_get();
	while (rt_tick_get()-timeo < 1000) {
		if (is_ready(mtd, 0))
			break;
	}
}

/**
 * nand_read_buf() - MTD Interface read_buf().
 *
 * @mtd:  A pointer to the owning MTD.
 * @buf:  The destination buffer.
 * @len:  The number of bytes to read.
 */
static void nand_read_buf(struct rt_mtd_nand_device *mtd, uint8_t *buf, int len)
{
	static u8 *data_buf = 0;

	if (!data_buf) {
#ifdef CONFIG_ARCH_MMU
		data_buf =
		(u8 *)ioremap_nocache((u32)iomem_to_phys((ulong)memalign(MXS_DMA_ALIGNMENT,
		PAGE_SIZE)),
		MXS_DMA_ALIGNMENT);
#else
		data_buf =
		memalign(MXS_DMA_ALIGNMENT, PAGE_SIZE);
#endif
		if (!data_buf) {
			printf("%s: failed to allocate data_buf "
				"queuebuffer\n",
				__func__);
			return;
		}
        
        // mmu nocache
        data_buf = (u8 *)(0x8000000|((u32)data_buf));
	}
    
	if (len > PAGE_SIZE)
		printf("[%s] Inadequate DMA buffer\n", __func__);

	if (!buf)
		printf("[%s] Buffer pointer is NULL\n", __func__);

	/* Ask the NFC. */
#ifdef CONFIG_ARCH_MMU
	read_data(mtd, 0,
			(dma_addr_t)iomem_to_phys((u32)data_buf),
			len);
#else
	read_data(mtd, 0,
			(dma_addr_t)data_buf, len);
#endif

	memcpy(buf, data_buf, len);
}

/**
 * nand_command - [DEFAULT] Send command to NAND device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This function is used for small page
 * devices (256/512 Bytes per page)
 */
static void nand_command(struct rt_mtd_nand_device *mtd, unsigned int command,
			 int column, int page_addr)
{
	int ctrl = NAND_CTRL_CLE | NAND_CTRL_CHANGE;

	/*
	 * Write out the command to the device.
	 */
	if (command == NAND_CMD_SEQIN) {
		int readcmd;

		if (column >= mtd->page_size) {
			/* OOB area */
			column -= mtd->page_size;
			readcmd = NAND_CMD_READOOB;
		} else if (column < 256) {
			/* First 256 bytes --> READ0 */
			readcmd = NAND_CMD_READ0;
		} else {
			column -= 256;
			readcmd = NAND_CMD_READ1;
		}
		cmd_ctrl(mtd, readcmd, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
	}
	cmd_ctrl(mtd, command, ctrl);

	/*
	 * Address cycle, when necessary
	 */
	ctrl = NAND_CTRL_ALE | NAND_CTRL_CHANGE;
	/* Serially input address */
	if (column != -1) {
		cmd_ctrl(mtd, column, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
	}
	if (page_addr != -1) {
		cmd_ctrl(mtd, page_addr, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
		cmd_ctrl(mtd, page_addr >> 8, ctrl);
		cmd_ctrl(mtd, page_addr >> 16, ctrl);
	}
	cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * program and erase have their own busy handlers
	 * status and sequential in needs no delay
	 */
	switch (command) {

	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_STATUS:
		return;

	}
	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	udelay(1);

	nand_wait_ready(mtd);
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

static inline void __enable_gpmi_clk(void)
{
	/* Clear bypass bit*/
	REG_SET(REGS_CLKCTRL_BASE, HW_CLKCTRL_CLKSEQ,
	       BM_CLKCTRL_CLKSEQ_BYPASS_GPMI);
	/* Set gpmi clock to ref_gpmi/12 */
	REG_WR(REGS_CLKCTRL_BASE, HW_CLKCTRL_GPMI,
	      (REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_GPMI) &
	      (~(BM_CLKCTRL_GPMI_DIV)) &
	      (~(BM_CLKCTRL_GPMI_CLKGATE))) |
	      1);
}

void rt_hw_mtd_nand_init(void)
{
    rt_uint16_t ecc_size;

	/* Reset the GPMI block. */
    __enable_gpmi_clk();
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
    uint8_t id[4] = {0};
    int i;
    
	for (i = 0; i < NFC_DMA_DESCRIPTOR_COUNT; ++i) {
		dma_desc[i] = mxs_dma_alloc_desc();

		if (NULL == dma_desc[i]) {
			for (i -= 1; i >= 0; --i)
				mxs_dma_free_desc(dma_desc[i]);
            rt_kprintf("failed to malloc dma desc\n");
			return;
		}
	}

	mxs_dma_init();

	/*
	 * Reset the chip, required by some chips (e.g. Micron MT29FxGxxxxx)
	 * after power-up
	 */
	nand_command(&_nanddrv_file_device, NAND_CMD_RESET, -1, -1);

	/* Send the command for reading device ID */
	nand_command(&_nanddrv_file_device, NAND_CMD_READID, 0x00, -1);

	/* Read manufacturer and device IDs */
	nand_read_buf(&_nanddrv_file_device, id, 4);
    rt_kprintf("NAND ID:%x %x %x MXIC NAND 128Mi\n", id[0], id[1], id[2]);
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
