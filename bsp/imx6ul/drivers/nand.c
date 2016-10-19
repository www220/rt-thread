#include <rthw.h>
#include <board.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#undef ALIGN
#include <common.h>
#include <malloc.h>
#include <asm/io.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/imx-common/regs-bch.h>
#include <asm/imx-common/regs-gpmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/imx-common/dma.h>

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

/* Extended commands for large page devices */
#define NAND_CMD_READSTART	0x30
#define NAND_CMD_RNDOUTSTART	0xE0
#define NAND_CMD_CACHEDPROG	0x15

/* Extended commands for AG-AND device */
/*
 * Note: the command for NAND_CMD_DEPLETE1 is really 0x00 but
 *       there is no way to distinguish that from NAND_CMD_READ0
 *       until the remaining sequence of commands has been completed
 *       so add a high order bit and mask it off in the command.
 */
#define NAND_CMD_DEPLETE1	0x100
#define NAND_CMD_DEPLETE2	0x38
#define NAND_CMD_STATUS_MULTI	0x71
#define NAND_CMD_STATUS_ERROR	0x72
/* multi-bank error status (banks 0-3) */
#define NAND_CMD_STATUS_ERROR0	0x73
#define NAND_CMD_STATUS_ERROR1	0x74
#define NAND_CMD_STATUS_ERROR2	0x75
#define NAND_CMD_STATUS_ERROR3	0x76
#define NAND_CMD_STATUS_RESET	0x7f
#define NAND_CMD_STATUS_CLEAR	0xff

#define NAND_CMD_NONE		-1

/* Status bits */
#define NAND_STATUS_FAIL	0x01
#define NAND_STATUS_FAIL_N1	0x02
#define NAND_STATUS_TRUE_READY	0x20
#define NAND_STATUS_READY	0x40
#define NAND_STATUS_WP		0x80

#define OOB_SIZE        64
#define PAGE_DATA_SIZE  2048
#define PAGE_PER_BLOCK  64
#define BLOCK_NUM       1024

#define BLOCK_SIZE      (PAGE_SIZE * PAGE_PER_BLOCK)
#define PAGE_SIZE       (PAGE_DATA_SIZE + OOB_SIZE)

/* ECC Macros */
#define GPMI_NFC_METADATA_SIZE	(20)
#define GPMI_NFC_CHUNK_DATA_CHUNK_SIZE	(512)
#define GPMI_NFC_CHUNK_DATA_CHUNK_SIZE_IN_BITS	(512 * 6)
#define GPMI_NFC_CHUNK_ECC_SIZE_IN_BITS(ecc_str)	(ecc_str * 13)
#define GPMI_NFC_ECC_CHUNK_CNT(page_data_size)	\
	(page_data_size / GPMI_NFC_CHUNK_DATA_CHUNK_SIZE)

#define GPMI_NFC_AUX_STATUS_OFF	((GPMI_NFC_METADATA_SIZE + 0x3) & ~0x3)
#define GPMI_NFC_AUX_SIZE(page_size)	((GPMI_NFC_AUX_STATUS_OFF) + \
	((GPMI_NFC_ECC_CHUNK_CNT(page_size) + 0x3) & ~0x3))

#define	GPMI_NFC_COMMAND_BUFFER_SIZE (32)
#define NFC_DMA_DESCRIPTOR_COUNT	(4)
static struct mxs_dma_desc *dma_desc[NFC_DMA_DESCRIPTOR_COUNT];
static struct rt_mtd_nand_device _nanddrv_file_device[2];

static u8 *data_buf = 0;
static u8 *oob_buf = 0;
static u32 m_u32BlkMarkBitStart = 0;
static u32 m_u32BlkMarkByteOfs = 0;
static int m_plan_num = 0;

/*
 * Cache management functions
 */
#ifndef	CONFIG_SYS_DCACHE_OFF
static void mxs_nand_flush_data_buf(uint32_t data_buf)
{
	flush_dcache_range(data_buf, data_buf+PAGE_SIZE);
}

static void mxs_nand_inval_data_buf(uint32_t data_buf)
{
	invalidate_dcache_range(data_buf, data_buf+PAGE_SIZE);
}

static void mxs_nand_flush_cmd_buf(uint32_t cmd_buf)
{
	flush_dcache_range(cmd_buf, cmd_buf+GPMI_NFC_COMMAND_BUFFER_SIZE);
}
#else
static inline void mxs_nand_flush_data_buf(uint32_t data_buf) {}
static inline void mxs_nand_inval_data_buf(uint32_t data_buf) {}
static inline void mxs_nand_flush_cmd_buf(uint32_t cmd_buf) {}
#endif

/**
 * is_ready() - Returns the ready/busy status of the given chip.
 *
 * @this:  Per-device data.
 * @chip:  The chip of interest.
 */
static int is_ready(struct rt_mtd_nand_device *mtd, unsigned int target_chip)
{
	struct mxs_gpmi_regs *gpmi_regs = (struct mxs_gpmi_regs *)MXS_GPMI_BASE;
	uint32_t tmp;

	tmp = readl(&gpmi_regs->hw_gpmi_stat);
	tmp >>= (GPMI_STAT_READY_BUSY_OFFSET + target_chip);

	return tmp & 1;
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
	struct mxs_dma_desc *d = dma_desc[0];
	uint32_t dma_channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + chip;
	int error = 0;

	d->cmd.data =
		MXS_DMA_DESC_COMMAND_DMA_READ | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_CHAIN | MXS_DMA_DESC_DEC_SEM |
		MXS_DMA_DESC_WAIT4END | (3 << MXS_DMA_DESC_PIO_WORDS_OFFSET) |
		(length << MXS_DMA_DESC_BYTES_OFFSET);

	d->cmd.address = buffer;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WRITE |
		GPMI_CTRL0_WORD_LENGTH |
		(chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_CLE |
		GPMI_CTRL0_ADDRESS_INCREMENT |
		length;
	d->cmd.pio_words[1] = 0;
	d->cmd.pio_words[2] = 0;

	mxs_dma_desc_append(dma_channel, d);
	mxs_nand_flush_cmd_buf(buffer);

	/* Go! */
	error = mxs_dma_go(dma_channel);
	if (error)
		rt_kprintf("[%s] DMA error\n", __func__);

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
	struct mxs_dma_desc *d = dma_desc[0];
	uint32_t dma_channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + chip;
	int error = 0;

	d->cmd.data =
		MXS_DMA_DESC_COMMAND_DMA_READ | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_DEC_SEM |
		MXS_DMA_DESC_WAIT4END | (1 << MXS_DMA_DESC_PIO_WORDS_OFFSET) |
		(length << MXS_DMA_DESC_BYTES_OFFSET);

	d->cmd.address = buffer;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WRITE |
		GPMI_CTRL0_WORD_LENGTH |
		(chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA |
		length;

	mxs_dma_desc_append(dma_channel, d);
	mxs_nand_flush_data_buf(buffer);

	/* Go! */
	error = mxs_dma_go(dma_channel);

	if (error)
		rt_kprintf("[%s] DMA error\n", __func__);

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
	struct mxs_dma_desc *d = dma_desc[0];
	uint32_t dma_channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + chip;
	int error = 0;

	d->cmd.data =
		MXS_DMA_DESC_COMMAND_DMA_WRITE | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_DEC_SEM |
		MXS_DMA_DESC_WAIT4END | (1 << MXS_DMA_DESC_PIO_WORDS_OFFSET) |
		(length << MXS_DMA_DESC_BYTES_OFFSET);

	d->cmd.address = buffer;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_READ |
		GPMI_CTRL0_WORD_LENGTH |
		(chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA |
		length;

	mxs_dma_desc_append(dma_channel, d);

	d = dma_desc[1];

	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_DEC_SEM | MXS_DMA_DESC_NAND_WAIT_4_READY |
		MXS_DMA_DESC_WAIT4END | (1 << MXS_DMA_DESC_PIO_WORDS_OFFSET);

	d->cmd.address = 0;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WAIT_FOR_READY |
		GPMI_CTRL0_WORD_LENGTH |
		(chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA;

	mxs_dma_desc_append(dma_channel, d);
	mxs_nand_inval_data_buf(buffer);

	/* Go! */
	error = mxs_dma_go(dma_channel);
	mxs_nand_inval_data_buf(buffer);

	if (error)
		rt_kprintf("[%s] DMA error\n", __func__);

#ifdef CONFIG_MTD_DEBUG
	{
		int i;
		dma_addr_t *tmp_buf_ptr = (dma_addr_t *)buffer;

		rt_kprintf("Buffer:");
		for (i = 0; i < length; ++i)
			rt_kprintf("0x%08x ", tmp_buf_ptr[i]);
		rt_kprintf("\n");
	}
#endif

	/* Return success. */
	return error;
}

int wait_for_bch_completion(int timeout)
{
	struct mxs_bch_regs *bch_regs = (struct mxs_bch_regs *)MXS_BCH_BASE;
	int ret;

	ret = mxs_wait_mask_set(&bch_regs->hw_bch_ctrl_reg,
		BCH_CTRL_COMPLETE_IRQ, timeout);

	writel(BCH_CTRL_COMPLETE_IRQ, &bch_regs->hw_bch_ctrl_clr);

	return ret;
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
	struct mxs_dma_desc *d = dma_desc[0];
	uint32_t dma_channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + chip;
	int error = 0;

	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_DEC_SEM |
		MXS_DMA_DESC_WAIT4END | (6 << MXS_DMA_DESC_PIO_WORDS_OFFSET);

	d->cmd.address = 0;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WRITE |
		GPMI_CTRL0_WORD_LENGTH |
		(chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA;
	d->cmd.pio_words[1] = 0;
	d->cmd.pio_words[2] =
		GPMI_ECCCTRL_ENABLE_ECC |
		GPMI_ECCCTRL_ECC_CMD_ENCODE |
		GPMI_ECCCTRL_BUFFER_MASK_BCH_PAGE | GPMI_ECCCTRL_BUFFER_MASK_BCH_AUXONLY;
	d->cmd.pio_words[3] = (mtd->page_size + mtd->oob_size);
	d->cmd.pio_words[4] = payload;
	d->cmd.pio_words[5] = auxiliary;

	mxs_dma_desc_append(dma_channel, d);
	mxs_nand_flush_data_buf(payload);

	/* Go! */
	error = mxs_dma_go(dma_channel);

	if (error)
		rt_kprintf("[%s] DMA error\n", __func__);

	error = wait_for_bch_completion(10000);

	error = (error) ? -ETIMEDOUT : 0;

	if (error)
		rt_kprintf("[%s] bch timeout!!!\n", __func__);

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
			dma_addr_t payload, dma_addr_t auxiliary, unsigned page_size)
{
	struct mxs_dma_desc *d = dma_desc[0];
	uint32_t dma_channel = MXS_DMA_CHANNEL_AHB_APBH_GPMI0 + chip;
	int error = 0;

	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER |
		MXS_DMA_DESC_CHAIN | MXS_DMA_DESC_NAND_WAIT_4_READY |
		MXS_DMA_DESC_WAIT4END | (1 << MXS_DMA_DESC_PIO_WORDS_OFFSET);

	d->cmd.address = 0;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WAIT_FOR_READY |
		GPMI_CTRL0_WORD_LENGTH |
		(chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA;

	mxs_dma_desc_append(dma_channel, d);

	d = dma_desc[1];

	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER |
		MXS_DMA_DESC_CHAIN |
		MXS_DMA_DESC_WAIT4END | (6 << MXS_DMA_DESC_PIO_WORDS_OFFSET);

	d->cmd.address = 0;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_READ |
		GPMI_CTRL0_WORD_LENGTH |
		(chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA |
		page_size;
	d->cmd.pio_words[1] = 0;
	d->cmd.pio_words[2] =
		GPMI_ECCCTRL_ENABLE_ECC |
		GPMI_ECCCTRL_ECC_CMD_DECODE |
		GPMI_ECCCTRL_BUFFER_MASK_BCH_AUXONLY;
	if (page_size == mtd->page_size + mtd->oob_size)
		d->cmd.pio_words[2] |= GPMI_ECCCTRL_BUFFER_MASK_BCH_PAGE;
	d->cmd.pio_words[3] = page_size;
	d->cmd.pio_words[4] = payload;
	d->cmd.pio_words[5] = auxiliary;

	mxs_dma_desc_append(dma_channel, d);

	d = dma_desc[2];

	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER |
		MXS_DMA_DESC_CHAIN | MXS_DMA_DESC_NAND_WAIT_4_READY |
		MXS_DMA_DESC_WAIT4END | (3 << MXS_DMA_DESC_PIO_WORDS_OFFSET);

	d->cmd.address = 0;

	d->cmd.pio_words[0] =
		GPMI_CTRL0_COMMAND_MODE_WAIT_FOR_READY |
		GPMI_CTRL0_WORD_LENGTH |
		(chip << GPMI_CTRL0_CS_OFFSET) |
		GPMI_CTRL0_ADDRESS_NAND_DATA |
		page_size;
	d->cmd.pio_words[1] = 0;
	d->cmd.pio_words[2] = 0;

	mxs_dma_desc_append(dma_channel, d);

	d = dma_desc[3];

	d->cmd.data =
		MXS_DMA_DESC_COMMAND_NO_DMAXFER | MXS_DMA_DESC_IRQ |
		MXS_DMA_DESC_DEC_SEM ;

	d->cmd.address = 0;

	mxs_dma_desc_append(dma_channel, d);
	mxs_nand_inval_data_buf(payload);

	/* Go! */
	error = mxs_dma_go(dma_channel);

	if (error)
		rt_kprintf("[%s] DMA error\n", __func__);

	error = wait_for_bch_completion(10000);
	mxs_nand_inval_data_buf(payload);

	error = (error) ? -ETIMEDOUT : 0;

	if (error)
		rt_kprintf("[%s] bch timeout!!!\n", __func__);

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
		cmd_queue = rt_memalign(MXS_DMA_ALIGNMENT, GPMI_NFC_COMMAND_BUFFER_SIZE);
		if (!cmd_queue) {
			rt_kprintf("%s: failed to allocate command "
				"queuebuffer\n",
				__func__);
			return;
		}

		rt_memset(cmd_queue, 0, GPMI_NFC_COMMAND_BUFFER_SIZE);
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
	rt_kprintf("%s: command: %s\n", __func__, display);
#endif

	error = send_command(mtd, 0, (dma_addr_t)cmd_queue, cmd_Q_len);

	if (error)
		rt_kprintf("Command execute failed!\n");

	/* Reset. */
	cmd_Q_len = 0;
}

/*
 * Wait for the ready pin, after a command
 * The timeout is catched later.
 */
static void nand_wait_ready(struct rt_mtd_nand_device *mtd)
{
	/* wait until command is processed or timeout occures */
	u32 timeo = rt_tick_get();
	while (rt_tick_get()-timeo < 1000) {
		if (is_ready(mtd, 0))
			break;
	}
}

/**
 * nand_write_buf() - MTD Interface write_buf().
 *
 * @mtd:  A pointer to the owning MTD.
 * @buf:  The source buffer.
 * @len:  The number of bytes to read.
 */
static void nand_write_buf(struct rt_mtd_nand_device *mtd,
				const uint8_t *buf, int len)
{
	if (len > PAGE_SIZE)
		rt_kprintf("[%s] Inadequate DMA buffer\n", __func__);

	if (!buf)
		rt_kprintf("[%s] Buffer pointer is NULL\n", __func__);

	rt_memcpy(data_buf, buf, len);

	/* Ask the NFC. */
	send_data(mtd, 0, (dma_addr_t)data_buf, len);
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

	if (len > PAGE_SIZE)
		rt_kprintf("[%s] Inadequate DMA buffer\n", __func__);

	if (!buf)
		rt_kprintf("[%s] Buffer pointer is NULL\n", __func__);

	/* Ask the NFC. */
	read_data(mtd, 0, (dma_addr_t)data_buf, len);

	rt_memcpy(buf, data_buf, len);
}

/**
 * nand_command - [DEFAULT] Send command to NAND large page device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This is the version for the new large page
 * devices We dont have the separate regions as we have in the small page
 * devices.  We must emulate NAND_CMD_READOOB to keep the code compatible.
 */
static void nand_command(struct rt_mtd_nand_device *mtd, unsigned int command,
			    int column, int page_addr)
{
	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += mtd->page_size;
		command = NAND_CMD_READ0;
	}

	/* Command latch cycle */
	cmd_ctrl(mtd, command & 0xff,
		       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	if (column != -1 || page_addr != -1) {
		int ctrl = NAND_CTRL_CHANGE | NAND_NCE | NAND_ALE;

		/* Serially input address */
		if (column != -1) {
			cmd_ctrl(mtd, column, ctrl);
			ctrl &= ~NAND_CTRL_CHANGE;
			cmd_ctrl(mtd, column >> 8, ctrl);
		}
		if (page_addr != -1) {
			cmd_ctrl(mtd, page_addr, ctrl);
			cmd_ctrl(mtd, page_addr >> 8,
				       NAND_NCE | NAND_ALE);
			if (m_plan_num > 1) {
			cmd_ctrl(mtd, page_addr >> 16,
				       NAND_NCE | NAND_ALE);
			}
		}
	}
	cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * program and erase have their own busy handlers
	 * status, sequential in, and deplete1 need no delay
	 */
	switch (command) {

	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
	case NAND_CMD_DEPLETE1:
		return;

		/*
		 * read error status commands require only a short delay
		 */
	case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
	case NAND_CMD_STATUS_ERROR1:
	case NAND_CMD_STATUS_ERROR2:
	case NAND_CMD_STATUS_ERROR3:
		udelay(20);
		return;

	case NAND_CMD_RNDOUT:
		/* No ready / busy check necessary */
		cmd_ctrl(mtd, NAND_CMD_RNDOUTSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		return;

	case NAND_CMD_READ0:
		cmd_ctrl(mtd, NAND_CMD_READSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
	}

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	udelay(1);

	nand_wait_ready(mtd);
}

/**
 * nand_wait - [DEFAULT]  wait until the command is done
 * @mtd:	MTD device structure
 * @chip:	NAND chip structure
 *
 * Wait for command done. This applies to erase and program only
 * Erase can take up to 400ms and program up to 20ms according to
 * general NAND and SmartMedia specs
 */
static int nand_wait(struct rt_mtd_nand_device *mtd)
{
	u8 state = 0;
	u32 timeo = rt_tick_get();

	nand_command(mtd, NAND_CMD_STATUS, -1, -1);
	while (1) {
		if (rt_tick_get()-timeo > 2000)
			return NAND_STATUS_FAIL;
		if (is_ready(mtd, 0))
			break;
	}

	nand_read_buf(mtd, &state, 1);
	return state;
}

/**
 * gpmi_nfc_block_mark_swapping() - Handles block mark swapping.
 *
 * Note that, when this function is called, it doesn't know whether it's
 * swapping the block mark, or swapping it *back* -- but it doesn't matter
 * because the the operation is the same.
 *
 * @this:       Per-device data.
 * @payload:    A pointer to the payload buffer.
 * @auxiliary:  A pointer to the auxiliary buffer.
 */
static void gpmi_nfc_block_mark_swapping(void *data_buf, void *oob_buf)
{
	u8  *p;
	u8  *a;
	u32 bit;
	u8  mask;
	u8  from_data;
	u8  from_oob;

	/*
	 * If control arrives here, we're swapping. Make some convenience
	 * variables.
	 */
	bit = m_u32BlkMarkBitStart;
	p   = ((u8 *)data_buf) + m_u32BlkMarkByteOfs;
	a   = oob_buf;

	/*
	 * Get the byte from the data area that overlays the block mark. Since
	 * the ECC engine applies its own view to the bits in the page, the
	 * physical block mark won't (in general) appear on a byte boundary in
	 * the data.
	 */
	from_data = (p[0] >> bit) | (p[1] << (8 - bit));

	/* Get the byte from the OOB. */
	from_oob = a[0];

	/* Swap them. */
	a[0] = from_data;

	mask = (0x1 << bit) - 1;
	p[0] = (p[0] & mask) | (from_oob << bit);

	mask = ~0 << bit;
	p[1] = (p[1] & mask) | (from_oob >> (8 - bit));
}

/* read chip id */
static rt_err_t nanddrv_file_read_id(struct rt_mtd_nand_device *device)
{
    rt_err_t id = 0;
    
    /* Send the command for reading device ID */
	nand_command(device, NAND_CMD_READID, 0x00, -1);

	/* Read manufacturer and device IDs */
	nand_read_buf(device, (uint8_t *)&id, 4);

    return id;
}

/* read/write/move page */
static rt_err_t nanddrv_file_read_page(struct rt_mtd_nand_device *device,
                                       rt_off_t page,
                                       rt_uint8_t *data, rt_uint32_t data_len,
                                       rt_uint8_t *spare, rt_uint32_t spare_len)
{
	int error = 0,fixnum = 0,i,block;
    u8 *status = 0;
    
    /* Send the command for reading device ID */
	nand_command(device, NAND_CMD_READ0, 0x00, page);
    
    /* Read manufacturer and device IDs */
    if (data && data_len) {
        error = read_page(device, 0, (dma_addr_t)data_buf, (dma_addr_t)oob_buf, PAGE_SIZE);
        block = GPMI_NFC_ECC_CHUNK_CNT(PAGE_DATA_SIZE)+1;
    }
    else {//7 = 4*13/8+1
        error = read_page(device, 0, (dma_addr_t)data_buf, (dma_addr_t)oob_buf, GPMI_NFC_METADATA_SIZE+7);
        block = 1;
    }
    if (error != 0)
        return -RT_EIO;

	/* Handle block mark swapping. */
    if (block > 1)
        gpmi_nfc_block_mark_swapping(data_buf, oob_buf);

    /* Check ECC buf */
    status = oob_buf + GPMI_NFC_AUX_STATUS_OFF;
    for(i=0; i<block; i++,status++)
    {
        if ((*status == 0x00) || (*status == 0xff))
            continue;
        if (*status == 0xfe) {
            error++;
            continue;
        }
        fixnum += *status;
    }
    
    /* Copy Read Buf */
    if (data && data_len)
        rt_memcpy(data,data_buf,data_len);
    if (spare && spare_len)
        rt_memcpy(spare,oob_buf,spare_len);

    if (error > 0)
        return -RT_ERROR;
    if (error == 0 && fixnum > 0)
        return RT_ERROR;
	return RT_EOK;
}

static rt_err_t nanddrv_file_write_page(struct rt_mtd_nand_device *device,
                                        rt_off_t page,
                                        const rt_uint8_t *data, rt_uint32_t data_len,
                                        const rt_uint8_t *oob, rt_uint32_t spare_len)
{
	int status = 0;
    
    /* Send the command for reading device ID */
	nand_command(device, NAND_CMD_SEQIN, 0x00, page);

    if (data && data_len)
        rt_memcpy(data_buf, data, data_len);
    if (oob && spare_len)
        rt_memcpy(oob_buf, oob, spare_len);
    
	/* Handle block mark swapping. */
	gpmi_nfc_block_mark_swapping(data_buf, oob_buf);

    status = send_page(device, 0, (dma_addr_t)data_buf, (dma_addr_t)oob_buf);
    if (status != 0)
        return -RT_EIO;

	nand_command(device, NAND_CMD_PAGEPROG, -1, -1);
	status = nand_wait(device);

    return status & NAND_STATUS_FAIL ? -RT_EIO : RT_EOK;
}

static rt_err_t nanddrv_file_move_page(struct rt_mtd_nand_device *device, rt_off_t from, rt_off_t to)
{
    return -RT_EIO;
}

/* erase block */
static rt_err_t nanddrv_file_mark_block(struct rt_mtd_nand_device *device, rt_uint32_t block);
static rt_err_t nanddrv_file_erase_block(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    int status = 0;

    /* add the start blocks */
    block *= device->pages_per_block;

	/* Send commands to erase a block */
	nand_command(device, NAND_CMD_ERASE1, -1, block);
	nand_command(device, NAND_CMD_ERASE2, -1, -1);

    status = nand_wait(device);
    if (status & NAND_STATUS_FAIL) {
        nanddrv_file_mark_block(device,block/device->pages_per_block);
        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_err_t nanddrv_file_check_block(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    u8 state = 0;

    /* add the start blocks */
    block *= device->pages_per_block;

    /* Send the command for reading device ID */
	nand_command(device, NAND_CMD_READ0, device->page_size, block);
    
    /* Read Bad Block Mark */
	nand_read_buf(device, &state, 1);

    return (state==0xff)?RT_EOK:-RT_EIO;
}

static rt_err_t nanddrv_file_mark_block(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    u8 state = 0;
    int status = 0;

    /* add the start blocks */
    block *= device->pages_per_block;

    /* Send the command for reading device ID */
	nand_command(device, NAND_CMD_SEQIN, device->page_size, block);
    
    /* Read Bad Block Mark */
	nand_write_buf(device, &state, 1);
    
	nand_command(device, NAND_CMD_PAGEPROG, -1, -1);
	status = nand_wait(device);

    return status & NAND_STATUS_FAIL ? -RT_EIO : RT_EOK;
}

const static struct rt_mtd_nand_driver_ops _ops =
{
    nanddrv_file_read_id,
    nanddrv_file_read_page,
    nanddrv_file_write_page,
    nanddrv_file_move_page,
    nanddrv_file_erase_block,
    nanddrv_file_check_block,
    nanddrv_file_mark_block,
};

static inline u32 gpmi_nfc_get_blk_mark_bit_ofs(u32 page_data_size,
						u32 ecc_strength)
{
	u32 chunk_data_size_in_bits;
	u32 chunk_ecc_size_in_bits;
	u32 chunk_total_size_in_bits;
	u32 block_mark_chunk_number;
	u32 block_mark_chunk_bit_offset;
	u32 block_mark_bit_offset;

	/* 4096 bits */
	chunk_data_size_in_bits = GPMI_NFC_CHUNK_DATA_CHUNK_SIZE * 8;
	/* 208 bits */
	chunk_ecc_size_in_bits  = GPMI_NFC_CHUNK_ECC_SIZE_IN_BITS(ecc_strength);

	/* 4304 bits */
	chunk_total_size_in_bits =
			chunk_data_size_in_bits + chunk_ecc_size_in_bits;

	/* Compute the bit offset of the block mark within the physical page. */
	/* 4096 * 8 = 32768 bits */
	block_mark_bit_offset = page_data_size * 8;

	/* Subtract the metadata bits. */
	/* 32688 bits */
	block_mark_bit_offset -= GPMI_NFC_METADATA_SIZE * 8 + GPMI_NFC_CHUNK_ECC_SIZE_IN_BITS(ecc_strength);

	/*
	 * Compute the chunk number (starting at zero) in which the block mark
	 * appears.
	 */
	/* 7 */
	block_mark_chunk_number =
			block_mark_bit_offset / chunk_total_size_in_bits;

	/*
	 * Compute the bit offset of the block mark within its chunk, and
	 * validate it.
	 */
	/* 2560 bits */
	block_mark_chunk_bit_offset =
			block_mark_bit_offset -
			(block_mark_chunk_number * chunk_total_size_in_bits);

	if (block_mark_chunk_bit_offset > chunk_data_size_in_bits)
		return 1;

	/*
	 * Now that we know the chunk number in which the block mark appears,
	 * we can subtract all the ECC bits that appear before it.
	 */
	/* 31232 bits */
	block_mark_bit_offset -=
		block_mark_chunk_number * chunk_ecc_size_in_bits;

	return block_mark_bit_offset;
}

#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | \
			PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)

static iomux_v3_cfg_t const nand_pads[] = {
	MX6_PAD_NAND_DATA00__RAWNAND_DATA00 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_DATA01__RAWNAND_DATA01 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_DATA02__RAWNAND_DATA02 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_DATA03__RAWNAND_DATA03 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_DATA04__RAWNAND_DATA04 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_DATA05__RAWNAND_DATA05 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_DATA06__RAWNAND_DATA06 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_DATA07__RAWNAND_DATA07 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_CLE__RAWNAND_CLE | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_ALE__RAWNAND_ALE | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_CE0_B__RAWNAND_CE0_B | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_RE_B__RAWNAND_RE_B | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_WE_B__RAWNAND_WE_B | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_WP_B__RAWNAND_WP_B | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
	MX6_PAD_NAND_READY_B__RAWNAND_READY_B | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
};
static void setup_gpmi_nand(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* config gpmi nand iomux */
	imx_iomux_v3_setup_multiple_pads(nand_pads, ARRAY_SIZE(nand_pads));

	clrbits_le32(&mxc_ccm->CCGR4,
		     MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_MASK);

	/*
	 * config gpmi and bch clock to 100 MHz
	 * bch/gpmi select PLL2 PFD2 400M
	 * 100M = 400M / 4
	 */
	clrbits_le32(&mxc_ccm->cscmr1,
		     MXC_CCM_CSCMR1_BCH_CLK_SEL |
		     MXC_CCM_CSCMR1_GPMI_CLK_SEL);
	clrsetbits_le32(&mxc_ccm->cscdr1,
			MXC_CCM_CSCDR1_BCH_PODF_MASK |
			MXC_CCM_CSCDR1_GPMI_PODF_MASK,
			(3 << MXC_CCM_CSCDR1_BCH_PODF_OFFSET) |
			(3 << MXC_CCM_CSCDR1_GPMI_PODF_OFFSET));

	/* enable gpmi and bch clock gating */
	setbits_le32(&mxc_ccm->CCGR4,
		     MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_MASK);

	/* enable apbh clock gating */
	setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}

void rt_hw_mtd_nand_init(void)
{
	struct mxs_gpmi_regs *gpmi_regs =
		(struct mxs_gpmi_regs *)MXS_GPMI_BASE;
	struct mxs_bch_regs *bch_regs =
		(struct mxs_bch_regs *)MXS_BCH_BASE;
	u32 block_count;
	u32 block_size;
	u32 metadata_size;
	u32 page_size;
	u32 blk_mark_bit_offs;
	uint32_t tmp;

	/* Set up GPMI pins */
	setup_gpmi_nand();

	/* Reset the GPMI block. */
	mxs_reset_block(&gpmi_regs->hw_gpmi_ctrl0_reg);
	mxs_reset_block(&bch_regs->hw_bch_ctrl_reg);

	/*
	 * Choose NAND mode, set IRQ polarity, disable write protection and
	 * select BCH ECC.
	 */
	clrsetbits_le32(&gpmi_regs->hw_gpmi_ctrl1,
			GPMI_CTRL1_GPMI_MODE,
			GPMI_CTRL1_ATA_IRQRDY_POLARITY | GPMI_CTRL1_DEV_RESET |
			GPMI_CTRL1_BCH_MODE);

	/* Translate the abstract choices into register fields. */
	block_count = GPMI_NFC_ECC_CHUNK_CNT(PAGE_DATA_SIZE);
	block_size = GPMI_NFC_CHUNK_DATA_CHUNK_SIZE/4;
	metadata_size = GPMI_NFC_METADATA_SIZE;
	page_size    = PAGE_DATA_SIZE + OOB_SIZE;

	/* Configure layout 0. */
	tmp = block_count << BCH_FLASHLAYOUT0_NBLOCKS_OFFSET;
	tmp |= metadata_size << BCH_FLASHLAYOUT0_META_SIZE_OFFSET;
	tmp |= 2 << BCH_FLASHLAYOUT0_ECC0_OFFSET;
	tmp |= 0 << BCH_FLASHLAYOUT0_DATA0_SIZE_OFFSET;
	writel(tmp, &bch_regs->hw_bch_flash0layout0);

	tmp = page_size << BCH_FLASHLAYOUT1_PAGE_SIZE_OFFSET;
	tmp |= 2 << BCH_FLASHLAYOUT1_ECCN_OFFSET;
	tmp |= block_size << BCH_FLASHLAYOUT1_DATAN_SIZE_OFFSET;
	writel(tmp, &bch_regs->hw_bch_flash0layout1);

	/* Set *all* chip selects to use layout 0. */
	writel(0, &bch_regs->hw_bch_ctrl_set);

	/* Enable interrupts. */
	writel(BCH_CTRL_COMPLETE_IRQ_EN, &bch_regs->hw_bch_ctrl_set);

	blk_mark_bit_offs = gpmi_nfc_get_blk_mark_bit_ofs(PAGE_DATA_SIZE, 4);

	m_u32BlkMarkByteOfs = blk_mark_bit_offs >> 3;
	m_u32BlkMarkBitStart  = blk_mark_bit_offs & 0x7;

    _nanddrv_file_device[0].plane_num = m_plan_num;
    _nanddrv_file_device[0].oob_size = OOB_SIZE;
    _nanddrv_file_device[0].oob_free = GPMI_NFC_METADATA_SIZE;
    _nanddrv_file_device[0].page_size = PAGE_DATA_SIZE;
    _nanddrv_file_device[0].pages_per_block = PAGE_PER_BLOCK;
    _nanddrv_file_device[0].block_start = 160;
    _nanddrv_file_device[0].block_end = BLOCK_NUM / 2 + 160;
    _nanddrv_file_device[0].block_total = _nanddrv_file_device[0].block_end - _nanddrv_file_device[0].block_start;
    _nanddrv_file_device[0].ops = &_ops;

    _nanddrv_file_device[1].plane_num = m_plan_num;
    _nanddrv_file_device[1].oob_size = OOB_SIZE;
    _nanddrv_file_device[1].oob_free = GPMI_NFC_METADATA_SIZE;
    _nanddrv_file_device[1].page_size = PAGE_DATA_SIZE;
    _nanddrv_file_device[1].pages_per_block = PAGE_PER_BLOCK;
    _nanddrv_file_device[1].block_start = _nanddrv_file_device[0].block_end;
    _nanddrv_file_device[1].block_end = BLOCK_NUM / 2 + 240;
    _nanddrv_file_device[1].block_total = _nanddrv_file_device[1].block_end - _nanddrv_file_device[1].block_start;
    _nanddrv_file_device[1].ops = &_ops;

    rt_mtd_nand_register_device("nand0", &_nanddrv_file_device[0]);
    rt_mtd_nand_register_device("nand1", &_nanddrv_file_device[1]);
}

static int mxs_nand_init(void)
{
	int i = 0, j;

	/* Allocate the Data Buffer. */
	data_buf = rt_memalign(MXS_DMA_ALIGNMENT, PAGE_SIZE);
	if (!data_buf)
		goto err1;
	rt_memset(data_buf, 0, PAGE_SIZE);
	oob_buf = data_buf + PAGE_DATA_SIZE;

	/* Allocate the DMA descriptors. */
	for (i = 0; i < NFC_DMA_DESCRIPTOR_COUNT; i++) {
		dma_desc[i] = mxs_dma_desc_alloc();
		if (!dma_desc[i])
			goto err2;
	}

	/* Init the DMA controller. */
	for (j = MXS_DMA_CHANNEL_AHB_APBH_GPMI0;
		j <= MXS_DMA_CHANNEL_AHB_APBH_GPMI7; j++) {
		if (mxs_dma_init_channel(j))
			goto err3;
	}
	return 0;

err3:
	for (--j; j >= 0; j--)
		mxs_dma_release(j);
err2:
	for (--i; i >= 0; i--)
	{
		mxs_dma_desc_free(dma_desc[i]);
		dma_desc[i] = NULL;
	}
err1:
	if (data_buf)
	{
		rt_freealign(data_buf);
		data_buf = NULL;
	}
	rt_kprintf("MXS NAND: Unable to allocate DMA descriptors\n");
	return -1;
}

void nand_init(void)
{
	uint8_t id[4] = {0};

	if(mxs_nand_init() != 0)
		return;

	/*
	 * Reset the chip, required by some chips (e.g. Micron MT29FxGxxxxx)
	 * after power-up
	 */
	nand_command(&_nanddrv_file_device[0], NAND_CMD_RESET, -1, -1);

	/* Send the command for reading device ID */
	nand_command(&_nanddrv_file_device[0], NAND_CMD_READID, 0x00, -1);

	/* Read manufacturer and device IDs */
	nand_read_buf(&_nanddrv_file_device[0], id, 4);
    rt_kprintf("NAND ID:%02X %02X ", id[0], id[1]);
    if (id[0] == 0xc2 && id[1] == 0xf1)
    {
        rt_kprintf("MXIC NAND 128Mib\n");
        m_plan_num = 1;
    }
    else if (id[0] == 0x01 && id[1] == 0xda)
    {
        rt_kprintf("Spansion NAND 256Mib\n");
        m_plan_num = 2;
    }
    else
    {
        rt_kprintf("Unknow NAND\n");
    }
}

#if defined(RT_USING_FINSH)
#include <finsh.h>
int nand_eraseall(int argc, char **argv)
{
    int index,pos = 0;
    if (argc < 2)
    {
        rt_kprintf("Usage: nand_eraseall 0 or 1\n");
        return -1;
    }
    pos = atol(argv[1]) % 2;
    for (index = _nanddrv_file_device[pos].block_start; index < _nanddrv_file_device[pos].block_end; index ++)
    {
        if (nanddrv_file_erase_block(&_nanddrv_file_device[pos], index) != 0)
            rt_kprintf("Nand Erase %d %d BadBlock\n", pos, index);
    }
    rt_kprintf("Nand Erase %d Ok\n", pos);
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(nand_eraseall, __cmd_nand_eraseall,  erase all of block in the nand flash)

#endif //RT_USING_FINSH
