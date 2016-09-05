#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "tf.h"
#include <dfs_fs.h>

#define SD_LINK
#ifdef SD_LINK
#define SD_LINK_PRINTF         rt_kprintf
#else
#define SD_LINK_PRINTF(...)
#endif

struct at91_mci {
	struct rt_mmcsd_host *host;
	struct rt_mmcsd_io_cfg io_cfg;
	struct imx_ssp_mmc_cfg *cfg;
};

static inline u32 ssp_mmc_is_wp(struct at91_mci *mmc)
{
	return pin_gpio_get(PINID_GPMI_CE1N);
}

static inline int ssp_mmc_read(struct at91_mci *mmc, uint reg)
{
	struct imx_ssp_mmc_cfg *cfg = mmc->cfg;
	return REG_RD(cfg->ssp_mmc_base, reg);
}

static inline void ssp_mmc_write(struct at91_mci *mmc, uint reg, uint val)
{
	struct imx_ssp_mmc_cfg *cfg = mmc->cfg;
	REG_WR(cfg->ssp_mmc_base, reg, val);
}

static void set_bit_clock(struct at91_mci *mmc, u32 clock)
{
	const u32 sspclk = 480000 * 18 / 29 / 1;	/* 297931 KHz */
	u32 divide, rate, tgtclk;

	/*
	 * SSP bit rate = SSPCLK / (CLOCK_DIVIDE * (1 + CLOCK_RATE)),
	 * CLOCK_DIVIDE has to be an even value from 2 to 254, and
	 * CLOCK_RATE could be any integer from 0 to 255.
	 */
	clock /= 1000;		/* KHz */
	for (divide = 2; divide < 254; divide += 2) {
		rate = sspclk / clock / divide;
		if (rate <= 256)
			break;
	}

	tgtclk = sspclk / divide / rate;
	while (tgtclk > clock) {
		rate++;
		tgtclk = sspclk / divide / rate;
	}
	if (rate > 256)
		rate = 256;

	/* Always set timeout the maximum */
	ssp_mmc_write(mmc, HW_SSP_TIMING, BM_SSP_TIMING_TIMEOUT |
		divide << BP_SSP_TIMING_CLOCK_DIVIDE |
		(rate - 1) << BP_SSP_TIMING_CLOCK_RATE);

	printf("MMC: Set clock rate to %d KHz (requested %d KHz)\n",
		tgtclk, clock);
}

static void set_bit_width(struct at91_mci *mmc, u8 width)
{
	int bus_width = 0;
	u32 regval;

	/* Set the bus width */
	regval = ssp_mmc_read(mmc, HW_SSP_CTRL0);
	regval &= ~BM_SSP_CTRL0_BUS_WIDTH;
	switch (width) {
	case MMCSD_BUS_WIDTH_1:
		regval |= (BV_SSP_CTRL0_BUS_WIDTH__ONE_BIT <<
				BP_SSP_CTRL0_BUS_WIDTH);
		bus_width = 1;
		break;
	case MMCSD_BUS_WIDTH_4:
		regval |= (BV_SSP_CTRL0_BUS_WIDTH__FOUR_BIT <<
				BP_SSP_CTRL0_BUS_WIDTH);
		bus_width = 4;
		break;
	case MMCSD_BUS_WIDTH_8:
		regval |= (BV_SSP_CTRL0_BUS_WIDTH__EIGHT_BIT <<
				BP_SSP_CTRL0_BUS_WIDTH);
		bus_width = 8;
	}
	ssp_mmc_write(mmc, HW_SSP_CTRL0, regval);

	printf("MMC: Set %d bits bus width\n",
		bus_width);
}

#define NO_CARD_ERR		-16 /* No SD/MMC card inserted */
#define UNUSABLE_ERR		-17 /* Unusable Card */
#define COMM_ERR		-18 /* Communications Error */
#define TIMEOUT			-19

#define mdelay(x) rt_thread_delay(x)
static volatile int mci_run;
extern int __rt_ffs(int value);
static int ssp_mmc_send_cmd(struct rt_mmcsd_host *host, struct rt_mmcsd_cmd *cmd)
{
	int i;
	struct at91_mci *mmc = (struct at91_mci*)host->private_data;

	mmcsd_dbg("Sending command %2d flag = %08X, arg = %08X, blocks = %d, length = %d\n",
		cmd->cmd_code, cmd->flags, cmd->arg,
		(cmd->data?cmd->data->blks:0),(cmd->data?cmd->data->blksize:0));

	/* Check bus busy */
	i = 0;
	while (ssp_mmc_read(mmc, HW_SSP_STATUS) & (BM_SSP_STATUS_BUSY |
		BM_SSP_STATUS_DATA_BUSY | BM_SSP_STATUS_CMD_BUSY)) {
		udelay(100);
		if (i++ == 10000) {
			printf("MMC: Bus busy timeout!\n");
			return TIMEOUT;
		}
	}

	/* See if card is present */
	if (ssp_mmc_read(mmc, HW_SSP_STATUS) & BM_SSP_STATUS_CARD_DETECT) {
		mci_run = 0;
		return NO_CARD_ERR;
	}

	/* Clear all control bits except bus width */
	ssp_mmc_write(mmc, HW_SSP_CTRL0_CLR, 0xff3fffff);

	/* Set up command */
	if (resp_type(cmd) == RESP_R3 || resp_type(cmd) == RESP_R4)
		ssp_mmc_write(mmc, HW_SSP_CTRL0_SET, BM_SSP_CTRL0_IGNORE_CRC);
	if (resp_type(cmd) != RESP_NONE)	/* Need to get response */
		ssp_mmc_write(mmc, HW_SSP_CTRL0_SET, BM_SSP_CTRL0_GET_RESP);
	if (resp_type(cmd) == RESP_R2)	/* It's a 136 bits response */
		ssp_mmc_write(mmc, HW_SSP_CTRL0_SET, BM_SSP_CTRL0_LONG_RESP);

	/* Command index */
	ssp_mmc_write(mmc, HW_SSP_CMD0,
		(ssp_mmc_read(mmc, HW_SSP_CMD0) & ~BM_SSP_CMD0_CMD) |
		(cmd->cmd_code << BP_SSP_CMD0_CMD));
	/* Command argument */
	ssp_mmc_write(mmc, HW_SSP_CMD1, cmd->arg);

	/* Set up data */
	if (cmd->data) {
		/* READ or WRITE */
		if (cmd->data->flags & DATA_DIR_READ) {
			ssp_mmc_write(mmc, HW_SSP_CTRL0_SET,
				BM_SSP_CTRL0_READ);
		} else if (ssp_mmc_is_wp(mmc)) {
			printf("MMC: Can not write a locked card!\n");
			return UNUSABLE_ERR;
		}
		ssp_mmc_write(mmc, HW_SSP_CTRL0_SET, BM_SSP_CTRL0_DATA_XFER);
		ssp_mmc_write(mmc, HW_SSP_BLOCK_SIZE,
			((cmd->data->blks - 1) <<
				BP_SSP_BLOCK_SIZE_BLOCK_COUNT) |
			((__rt_ffs(cmd->data->blksize) - 1) <<
				BP_SSP_BLOCK_SIZE_BLOCK_SIZE));
		ssp_mmc_write(mmc, HW_SSP_XFER_SIZE,
			cmd->data->blksize * cmd->data->blks);
	}

	/* Kick off the command */
	ssp_mmc_write(mmc, HW_SSP_CTRL0_SET, BM_SSP_CTRL0_WAIT_FOR_IRQ);
	ssp_mmc_write(mmc, HW_SSP_CTRL0_SET, BM_SSP_CTRL0_ENABLE);
	ssp_mmc_write(mmc, HW_SSP_CTRL0_SET, BM_SSP_CTRL0_RUN);

	/* Wait for the command to complete */
	i = 0;
	do {
		udelay(100);
		if (i++ == 10000) {
			printf("MMC: Command %ld busy\n",
				cmd->cmd_code);
			break;
		}
	} while (ssp_mmc_read(mmc, HW_SSP_STATUS) &
		BM_SSP_STATUS_CMD_BUSY);

	/* Check command timeout */
	if (ssp_mmc_read(mmc, HW_SSP_STATUS) &
		BM_SSP_STATUS_RESP_TIMEOUT) {
		printf("MMC: Command %ld timeout\n",
			cmd->cmd_code);
		return TIMEOUT;
	}

	/* Check command errors */
	if (ssp_mmc_read(mmc, HW_SSP_STATUS) &
		(BM_SSP_STATUS_RESP_CRC_ERR | BM_SSP_STATUS_RESP_ERR)) {
		printf("MMC: Command %ld error (status 0x%08x)!\n",
			cmd->cmd_code,
			ssp_mmc_read(mmc, HW_SSP_STATUS));
		return COMM_ERR;
	}

	/* Copy response to response buffer */
	if (resp_type(cmd) == RESP_R2) {
		cmd->resp[3] = ssp_mmc_read(mmc, HW_SSP_SDRESP0);
		cmd->resp[2] = ssp_mmc_read(mmc, HW_SSP_SDRESP1);
		cmd->resp[1] = ssp_mmc_read(mmc, HW_SSP_SDRESP2);
		cmd->resp[0] = ssp_mmc_read(mmc, HW_SSP_SDRESP3);
	} else {
		cmd->resp[0] = ssp_mmc_read(mmc, HW_SSP_SDRESP0);
	}

	/* Return if no data to process */
	if (!cmd->data)
		return 0;

	/* Process the data */
	u32 xfer_cnt = cmd->data->blksize * cmd->data->blks;
	u32 *tmp_ptr;

	if (cmd->data->flags & DATA_DIR_READ) {
		tmp_ptr = (u32 *)cmd->data->buf;
		while (xfer_cnt > 0) {
			if ((ssp_mmc_read(mmc, HW_SSP_STATUS) &
				BM_SSP_STATUS_FIFO_EMPTY) == 0) {
				*tmp_ptr++ = ssp_mmc_read(mmc, HW_SSP_DATA);
				xfer_cnt -= 4;
			}
		}
	} else {
		tmp_ptr = (u32 *)cmd->data->buf;
		while (xfer_cnt > 0) {
			if ((ssp_mmc_read(mmc, HW_SSP_STATUS) &
				BM_SSP_STATUS_FIFO_FULL) == 0) {
				ssp_mmc_write(mmc, HW_SSP_DATA, *tmp_ptr++);
				xfer_cnt -= 4;
			}
		}
	}

	/* Check data errors */
	if (ssp_mmc_read(mmc, HW_SSP_STATUS) &
		(BM_SSP_STATUS_TIMEOUT | BM_SSP_STATUS_DATA_CRC_ERR |
		BM_SSP_STATUS_FIFO_OVRFLW | BM_SSP_STATUS_FIFO_UNDRFLW)) {
		printf("MMC: Data error with command %ld (status 0x%08x)!\n",
			cmd->cmd_code,
			ssp_mmc_read(mmc, HW_SSP_STATUS));
		return COMM_ERR;
	}

	return 0;
}

static struct pin_desc ssp_pins_desc[] = {
	{ PINID_SSP0_DATA0, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_DATA1, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_DATA2, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_DATA3, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_CMD, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_DETECT, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_SCK, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
};
static struct pin_group ssp_pins = {
	.pins		= ssp_pins_desc,
	.nr_pins	= ARRAY_SIZE(ssp_pins_desc)
};

void rt_hw_ssp_init(void)
{
    /* Set up SSP pins */
	pin_set_group(&ssp_pins);

	/* Set up SD0 WP pin */
	pin_set_type(PINID_GPMI_CE1N, PIN_GPIO);
	pin_gpio_direction(PINID_GPMI_CE1N, 0);
}

/*
 * Handle an MMC request
 */
static void at91_mci_request(struct rt_mmcsd_host *host, struct rt_mmcsd_req *req)
{
	if (req->cmd->cmd_code == 5)
	{
		req->cmd->err = COMM_ERR;
		mmcsd_req_complete(host);
	}
	else
	{
		req->cmd->err = ssp_mmc_send_cmd(host,req->cmd);
		if (req->stop)
			req->stop->err = ssp_mmc_send_cmd(host,req->stop);
		mmcsd_req_complete(host);
	}
}

/*
 * Set the IOCFG
 */
static void at91_mci_set_iocfg(struct rt_mmcsd_host *host, struct rt_mmcsd_io_cfg *io_cfg)
{
	rt_uint32_t regval;
	struct at91_mci *mmc = (struct at91_mci*)host->private_data;

	if (io_cfg->power_mode == MMCSD_POWER_UP) {
		mmc->io_cfg = *io_cfg;

		/* Set REF_IO0 at 297.731 MHz */
		regval = REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_FRAC0);
		regval &= ~BM_CLKCTRL_FRAC0_IO0FRAC;
		REG_WR(REGS_CLKCTRL_BASE, HW_CLKCTRL_FRAC0,
			regval | (29 << BP_CLKCTRL_FRAC0_IO0FRAC));
		/* Enable REF_IO0 */
		REG_CLR(REGS_CLKCTRL_BASE, HW_CLKCTRL_FRAC0,
			BM_CLKCTRL_FRAC0_CLKGATEIO0);

		/* Source SSPCLK from REF_IO0 */
		REG_CLR(REGS_CLKCTRL_BASE, HW_CLKCTRL_CLKSEQ,
			mmc->cfg->clkctrl_clkseq_ssp_offset);
		/* Turn on SSPCLK */
		REG_WR(REGS_CLKCTRL_BASE, mmc->cfg->clkctrl_ssp_offset,
			REG_RD(REGS_CLKCTRL_BASE, mmc->cfg->clkctrl_ssp_offset) &
			~BM_CLKCTRL_SSP_CLKGATE);
		/* Set SSPCLK divide 1 */
		regval = REG_RD(REGS_CLKCTRL_BASE, mmc->cfg->clkctrl_ssp_offset);
		regval &= ~(BM_CLKCTRL_SSP_DIV_FRAC_EN | BM_CLKCTRL_SSP_DIV);
		REG_WR(REGS_CLKCTRL_BASE, mmc->cfg->clkctrl_ssp_offset,
			regval | (1 << BP_CLKCTRL_SSP_DIV));
		/* Wait for new divide ready */
		do {
			udelay(10);
		} while (REG_RD(REGS_CLKCTRL_BASE, mmc->cfg->clkctrl_ssp_offset) &
			BM_CLKCTRL_SSP_BUSY);

		/* Prepare for software reset */
		ssp_mmc_write(mmc, HW_SSP_CTRL0_CLR, BM_SSP_CTRL0_SFTRST);
		ssp_mmc_write(mmc, HW_SSP_CTRL0_CLR, BM_SSP_CTRL0_CLKGATE);
		/* Assert reset */
		ssp_mmc_write(mmc, HW_SSP_CTRL0_SET, BM_SSP_CTRL0_SFTRST);
		/* Wait for confirmation */
		while (!(ssp_mmc_read(mmc, HW_SSP_CTRL0) & BM_SSP_CTRL0_CLKGATE))
			;
		/* Done */
		ssp_mmc_write(mmc, HW_SSP_CTRL0_CLR, BM_SSP_CTRL0_SFTRST);
		ssp_mmc_write(mmc, HW_SSP_CTRL0_CLR, BM_SSP_CTRL0_CLKGATE);

		/* 8 bits word length in MMC mode */
		regval = ssp_mmc_read(mmc, HW_SSP_CTRL1);
		regval &= ~(BM_SSP_CTRL1_SSP_MODE | BM_SSP_CTRL1_WORD_LENGTH);
		ssp_mmc_write(mmc, HW_SSP_CTRL1, regval |
			(BV_SSP_CTRL1_SSP_MODE__SD_MMC << BP_SSP_CTRL1_SSP_MODE) |
			(BV_SSP_CTRL1_WORD_LENGTH__EIGHT_BITS <<
				BP_SSP_CTRL1_WORD_LENGTH));

		/* Set initial bit clock 400 KHz */
		set_bit_clock(mmc, 400000);
		mmc->io_cfg.clock = 400000;

		/* Send initial 74 clock cycles (185 us @ 400 KHz)*/
		ssp_mmc_write(mmc, HW_SSP_CMD0_SET, BM_SSP_CMD0_CONT_CLKING_EN);
		udelay(200);
		ssp_mmc_write(mmc, HW_SSP_CMD0_CLR, BM_SSP_CMD0_CONT_CLKING_EN);

		set_bit_width(mmc, 0);
		mmc->io_cfg.bus_width = 0;
		return;
	}

	/* Set up SSPCLK */
	if (io_cfg->clock != mmc->io_cfg.clock)
	{
		if (io_cfg->clock != 0)
			set_bit_clock(mmc, io_cfg->clock);
	}

	/* Set the bus width */
	if (io_cfg->bus_width != mmc->io_cfg.bus_width)
	{
		set_bit_width(mmc, io_cfg->bus_width);
	}

	/* maybe switch power to the card */
	if (io_cfg->power_mode != mmc->io_cfg.power_mode)
	{
		switch (io_cfg->power_mode)
		{
			case MMCSD_POWER_OFF:
				printf("MMC: Power Off!\n");
				mci_run = 0;
				break;
		}
	}

	mmc->io_cfg = *io_cfg;
}

static const struct rt_mmcsd_host_ops ops = {
	at91_mci_request,
	at91_mci_set_iocfg,
	RT_NULL,
	RT_NULL,
};

struct imx_ssp_mmc_cfg ssp_mmc_cfg = {
	REGS_SSP0_BASE, HW_CLKCTRL_SSP0, BM_CLKCTRL_CLKSEQ_BYPASS_SSP0
};
static struct at91_mci mci = {
	.cfg = &ssp_mmc_cfg,
};

static void sd_monitor_thread_entry(void *parameter)
{
    uint32_t card,status;
    card  = (ssp_mmc_read(&mci, HW_SSP_STATUS) & BM_SSP_STATUS_CARD_DETECT);

    while(1)
    {
        status  = (ssp_mmc_read(&mci, HW_SSP_STATUS) & BM_SSP_STATUS_CARD_DETECT);
        if (status != card) {
            if (status) {
        		printf("MMC: No card detected!\n");
        		if (mci.host->card) {
            		mmcsd_change(mci.host);
            		mmcsd_wait_cd_changed(5000);
            		if (mci.host->card == NULL)
            			rt_kprintf("Unmount /mnt/mmc ok!\n");
            		else
            			rt_kprintf("Unmount /mnt/mmc failed!\n");
        		}
        	} else {
        		printf("MMC: Card detected!\n");
        		mci_run = 1;
        		mmcsd_change(mci.host);
        		mmcsd_wait_cd_changed(5000);
        	    if (dfs_mount("sd0", "/mnt/mmc", "elm", 0, 0) == 0)
        	        rt_kprintf("Mount /mnt/mmc ok!\n");
        	    else
        	        rt_kprintf("Mount /mnt/mmc failed!\n");
        	}
            card = status;
        }

        rt_thread_delay(RT_TICK_PER_SECOND);
    } /* while(1) */
}

void tf_init(void)
{
	struct rt_mmcsd_host *host;

	rt_mmcsd_core_init();
	rt_mmcsd_blk_init();

	host = mmcsd_alloc_host();
	if (!host)
		return;

	mci.host = host;

	host->ops = &ops;
	host->freq_min = 400000;
	host->freq_max = 148000000;
	host->valid_ocr = VDD_32_33 | VDD_31_32 | VDD_30_31 | \
				VDD_29_30 | VDD_28_29 | VDD_27_28;
	host->flags = MMCSD_BUSWIDTH_4 | MMCSD_MUTBLKWRITE | \
				MMCSD_SUP_HIGHSPEED;
	host->max_seg_size = 32 * 1024;
	host->max_dma_segs = 0;
	host->max_blk_size = 512;
	host->max_blk_count = 32;
	host->private_data = &mci;

	mci_run = 1;
	mmcsd_change(host);
	mmcsd_wait_cd_changed(5000);
	if (mci_run)
        printf("MMC: Card detected!\n");
	else
        printf("MMC: No card detected!\n");

    /* start sd monitor */
    {
        rt_thread_t tid;
        tid = rt_thread_create("sd_mon",
                               sd_monitor_thread_entry,
                               RT_NULL,
                               2048,
                               RT_THREAD_PRIORITY_MAX - 2,
                               20);
        if (tid != RT_NULL)
            rt_thread_startup(tid);
    }
}
