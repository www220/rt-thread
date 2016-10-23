#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include <dfs_fs.h>

#undef ALIGN
#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/imx-common/iomux-v3.h>
#include "fsl_esdhc.h"

#define SD_LINK

#ifdef SD_LINK
#define SD_LINK_PRINTF         rt_kprintf
#else
#define SD_LINK_PRINTF(...)
#endif

#define USDHC1_WP_GPIO	IMX_GPIO_NR(1, 18)
#define USDHC1_CD_GPIO	IMX_GPIO_NR(1, 19)
#define SDHCI_IRQ_EN_BITS		(IRQSTATEN_CC | IRQSTATEN_TC | \
				IRQSTATEN_CINT | \
				IRQSTATEN_CTOE | IRQSTATEN_CCE | IRQSTATEN_CEBE | \
				IRQSTATEN_CIE | IRQSTATEN_DTOE | IRQSTATEN_DCE | \
				IRQSTATEN_DEBE | IRQSTATEN_BRR | IRQSTATEN_BWR | \
				IRQSTATEN_DINT)

struct fsl_esdhc {
	uint    dsaddr;		/* SDMA system address register */
	uint    blkattr;	/* Block attributes register */
	uint    cmdarg;		/* Command argument register */
	uint    xfertyp;	/* Transfer type register */
	uint    cmdrsp0;	/* Command response 0 register */
	uint    cmdrsp1;	/* Command response 1 register */
	uint    cmdrsp2;	/* Command response 2 register */
	uint    cmdrsp3;	/* Command response 3 register */
	uint    datport;	/* Buffer data port register */
	uint    prsstat;	/* Present state register */
	uint    proctl;		/* Protocol control register */
	uint    sysctl;		/* System Control Register */
	uint    irqstat;	/* Interrupt status register */
	uint    irqstaten;	/* Interrupt status enable register */
	uint    irqsigen;	/* Interrupt signal enable register */
	uint    autoc12err;	/* Auto CMD error status register */
	uint    hostcapblt;	/* Host controller capabilities register */
	uint    wml;		/* Watermark level register */
	uint    mixctrl;	/* For USDHC */
	char    reserved1[4];	/* reserved */
	uint    fevt;		/* Force event register */
	uint    admaes;		/* ADMA error status register */
	uint    adsaddr;	/* ADMA system address register */
	char    reserved2[4];
	uint    dllctrl;
	uint    dllstat;
	uint    clktunectrlstatus;
	char    reserved3[84];
	uint    vendorspec;
	uint    mmcboot;
	uint    vendorspec2;
	char	reserved4[48];
	uint    hostver;	/* Host controller version register */
#ifndef ARCH_MXC
	char    reserved5[4];	/* reserved */
	uint    dmaerraddr;	/* DMA error address register */
	char    reserved6[4];	/* reserved */
	uint    dmaerrattr;	/* DMA error attribute register */
	char    reserved7[4];	/* reserved */
	uint    hostcapblt2;	/* Host controller capabilities register 2 */
	char    reserved8[8];	/* reserved */
	uint    tcr;		/* Tuning control register */
	char    reserved9[28];	/* reserved */
	uint    sddirctl;	/* SD direction control register */
	char    reserved10[712];/* reserved */
	uint    scr;		/* eSDHC control register */
#endif
};

struct imx_ssp_mmc_cfg {
	u32 ssp_mmc_base;
	u32 clkctrl_ssp_offset;
	u32 clkctrl_clkseq_ssp_offset;
};

struct at91_mci {
	struct rt_mmcsd_host *host;
	struct rt_mmcsd_io_cfg io_cfg;
	struct imx_ssp_mmc_cfg *cfg;
	u32 iobase;
};

static inline u32 ssp_mmc_is_wp(struct at91_mci *mmc)
{
	return gpio_get_value(USDHC1_WP_GPIO);
}

static inline int ssp_mmc_read(struct at91_mci *mmc, uint reg)
{
	struct imx_ssp_mmc_cfg *cfg = mmc->cfg;
	return readl(cfg->ssp_mmc_base + reg);
}

static inline void ssp_mmc_write(struct at91_mci *mmc, uint reg, uint val)
{
	struct imx_ssp_mmc_cfg *cfg = mmc->cfg;
	writel(cfg->ssp_mmc_base + reg, val);
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

	rt_kprintf("MMC: Set clock rate to %d KHz (requested %d KHz)\n",
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

	rt_kprintf("MMC: Set %d bits bus width\n",
		bus_width);
}

#define NO_CARD_ERR		-16 /* No SD/MMC card inserted */
#define UNUSABLE_ERR	-17 /* Unusable Card */
#define COMM_ERR		-18 /* Communications Error */
#define TIMEOUT			-19

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
			rt_kprintf("MMC: Bus busy timeout!\n");
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
			rt_kprintf("MMC: Can not write a locked card!\n");
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
			rt_kprintf("MMC: Command %ld busy\n",
				cmd->cmd_code);
			break;
		}
	} while (ssp_mmc_read(mmc, HW_SSP_STATUS) &
		BM_SSP_STATUS_CMD_BUSY);

	/* Check command timeout */
	if (ssp_mmc_read(mmc, HW_SSP_STATUS) &
		BM_SSP_STATUS_RESP_TIMEOUT) {
		rt_kprintf("MMC: Command %ld timeout\n",
			cmd->cmd_code);
		return TIMEOUT;
	}

	/* Check command errors */
	if (ssp_mmc_read(mmc, HW_SSP_STATUS) &
		(BM_SSP_STATUS_RESP_CRC_ERR | BM_SSP_STATUS_RESP_ERR)) {
		rt_kprintf("MMC: Command %ld error (status 0x%08x)!\n",
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
		rt_kprintf("MMC: Data error with command %ld (status 0x%08x)!\n",
			cmd->cmd_code,
			ssp_mmc_read(mmc, HW_SSP_STATUS));
		return COMM_ERR;
	}

	return 0;
}

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)
#define USDHC_DAT3_CD_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_DOWN  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/* CD */
	MX6_PAD_UART1_RTS_B__GPIO1_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* WP */
	MX6_PAD_UART1_CTS_B__USDHC1_WP  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void esdhc_reset(struct fsl_esdhc *regs)
{
	unsigned long timeout = 100; /* wait max 100 ms */

	/* reset the controller */
	esdhc_setbits32(&regs->sysctl, SYSCTL_RSTA);

	/* hardware clears the bit when it is done */
	while ((esdhc_read32(&regs->sysctl) & SYSCTL_RSTA) && --timeout)
		udelay(1000);
	if (!timeout)
		rt_kprintf("MMC/SD: Reset never completed.\n");
}

void rt_hw_ssp_init(void)
{
	struct fsl_esdhc *regs = (struct fsl_esdhc *)USDHC2_BASE_ADDR;
    /* Set up SSP pins */
	imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));

	/* Set up SD0 WP pin */
	gpio_direction_input(USDHC1_WP_GPIO);
	gpio_direction_input(USDHC1_CD_GPIO);

	esdhc_reset(regs);
	esdhc_setbits32(&regs->sysctl, SYSCTL_PEREN | SYSCTL_HCKEN | SYSCTL_IPGEN | SYSCTL_CKEN);
	writel(SDHCI_IRQ_EN_BITS, &regs->irqstaten);
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
	struct fsl_esdhc *regs = mmc->iobase;

	if (io_cfg->power_mode == MMCSD_POWER_UP) {
		mmc->io_cfg = *io_cfg;

		/* RSTA doesn't reset MMC_BOOT register, so manually reset it */
		esdhc_write32(&regs->mmcboot, 0x0);
		/* Reset MIX_CTRL and CLK_TUNE_CTRL_STATUS regs to 0 */
		esdhc_write32(&regs->mixctrl, 0x0);
		esdhc_write32(&regs->clktunectrlstatus, 0x0);

		/* Put VEND_SPEC to default value */
		esdhc_write32(&regs->vendorspec, VENDORSPEC_INIT);
		/* Enable cache snooping */
		esdhc_write32(&regs->scr, 0x00000040);

		esdhc_setbits32(&regs->sysctl, SYSCTL_HCKEN | SYSCTL_IPGEN);

		/* Disable the BRR and BWR bits in IRQSTAT */
		esdhc_clrbits32(&regs->irqstaten, IRQSTATEN_BRR | IRQSTATEN_BWR);

		/* Put the PROCTL reg back to the default */
		esdhc_write32(&regs->proctl, PROCTL_INIT);

		/* Set timout to the maximum value */
		esdhc_clrsetbits32(&regs->sysctl, SYSCTL_TIMEOUT_MASK, 14 << 16);

#ifdef CONFIG_SYS_FSL_ESDHC_FORCE_VSELECT
		esdhc_setbits32(&regs->vendorspec, ESDHC_VENDORSPEC_VSELECT);
#endif
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
				rt_kprintf("MMC: Power Off!\n");
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
	USDHC2_BASE_ADDR, HW_CLKCTRL_SSP0, BM_CLKCTRL_CLKSEQ_BYPASS_SSP0
};
static struct at91_mci mci = {
	.cfg = &ssp_mmc_cfg,
	.iobase = USDHC2_BASE_ADDR,
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
            	SD_LINK_PRINTF("MMC: No card detected!\n");
            	if (mci.host->card) {
            		mmcsd_change(mci.host);
            		mmcsd_wait_cd_changed(5000);
            		if (mci.host->card == NULL)
            			rt_kprintf("Unmount /mnt/mmc ok!\n");
            		else
            			rt_kprintf("Unmount /mnt/mmc failed!\n");
        		}
        	} else {
        		SD_LINK_PRINTF("MMC: Card detected!\n");
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
	host->max_dma_segs = 0;
	host->max_blk_size = 512;
	host->max_blk_count = 32;
	host->private_data = &mci;

	mci_run = 1;
	mmcsd_change(host);
	mmcsd_wait_cd_changed(5000);
	if (mci_run)
		SD_LINK_PRINTF("MMC: Card detected!\n");
	else
		SD_LINK_PRINTF("MMC: No card detected!\n");

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
