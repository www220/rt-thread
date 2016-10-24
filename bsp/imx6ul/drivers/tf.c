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
};

struct mmc_mci {
	struct rt_mmcsd_host *host;
	struct rt_mmcsd_io_cfg io_cfg;
	u32 iobase;
	int sdhc_clk;
};

static inline u32 ssp_mmc_is_wp(struct mmc_mci *mmc)
{
	volatile struct fsl_esdhc *regs = (struct fsl_esdhc *)mmc->iobase;

	return (esdhc_read32(&regs->prsstat) & PRSSTAT_WPSPL) == 0;
}

static int ssp_mmc_is_cd(struct mmc_mci *mmc)
{
	volatile struct fsl_esdhc *regs = (struct fsl_esdhc *)mmc->iobase;
	int timeout = 100;

	while (!(esdhc_read32(&regs->prsstat) & PRSSTAT_CINS) && --timeout)
		udelay(100);

	return timeout <= 0;
}

static void set_bit_clock(struct mmc_mci *mmc, u32 clock)
{
	int div, pre_div;
	volatile struct fsl_esdhc *regs = (struct fsl_esdhc *)mmc->iobase;
	int sdhc_clk = mmc->sdhc_clk;
	uint tgtclk, clk;

	if (sdhc_clk / 16 > clock) {
		for (pre_div = 2; pre_div < 256; pre_div *= 2)
			if ((sdhc_clk / pre_div) <= (clock * 16))
				break;
	} else {
		pre_div = 2;
	}

	tgtclk = sdhc_clk / pre_div;
	for (div = 1; div <= 16; div++) {
		if ((sdhc_clk / (div * pre_div)) <= clock) {
			tgtclk = sdhc_clk / (div * pre_div);
			break;
		}
	}

	pre_div >>= 1;
	div -= 1;

	clk = (pre_div << 8) | (div << 4);

	esdhc_clrbits32(&regs->sysctl, SYSCTL_CKEN);

	esdhc_clrsetbits32(&regs->sysctl, SYSCTL_CLOCK_MASK, clk);

	udelay(10000);

	clk = SYSCTL_PEREN | SYSCTL_CKEN;

	esdhc_setbits32(&regs->sysctl, clk);

	rt_kprintf("MMC: Set clock rate to %d KHz (requested %d KHz)\n",
		tgtclk, clock);
}

static void set_bit_width(struct mmc_mci *mmc, u8 width)
{
	int bus_width = 0;
	volatile struct fsl_esdhc *regs = (struct fsl_esdhc *)mmc->iobase;

	/* Set the bus width */
	esdhc_clrbits32(&regs->proctl, PROCTL_DTW_4 | PROCTL_DTW_8);

	switch (width) {
	case MMCSD_BUS_WIDTH_4:
		esdhc_setbits32(&regs->proctl, PROCTL_DTW_4);
		bus_width = 4;
		break;
	case MMCSD_BUS_WIDTH_8:
		esdhc_setbits32(&regs->proctl, PROCTL_DTW_8);
		bus_width = 8;
	}

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
	int	err = 0;
	uint	xfertyp;
	uint	irqstat;
	struct mmc_mci *mmc = (struct mmc_mci*)host->private_data;
	volatile struct fsl_esdhc *regs = (struct fsl_esdhc *)mmc->iobase;

	mmcsd_dbg("Sending command %2d flag = %08X, arg = %08X, blocks = %d, length = %d\n",
		cmd->cmd_code, cmd->flags, cmd->arg,
		(cmd->data?cmd->data->blks:0),(cmd->data?cmd->data->blksize:0));

	esdhc_write32(&regs->irqstat, -1);
	sync();

	/* Wait for the bus to be idle */
	while ((esdhc_read32(&regs->prsstat) & PRSSTAT_CICHB) ||
			(esdhc_read32(&regs->prsstat) & PRSSTAT_CIDHB));
	while (esdhc_read32(&regs->prsstat) & PRSSTAT_DLA);

	/* See if card is present */
	if (ssp_mmc_is_cd(mmc)) {
		mci_run = 0;
		return NO_CARD_ERR;
	}

	/* Set up command */
	xfertyp = 0;
	if (resp_type(cmd) != RESP_NONE) {
		if (resp_type(cmd) != RESP_R3 && resp_type(cmd) != RESP_R4)
			xfertyp |= XFERTYP_CCCEN;
		if (resp_type(cmd) != RESP_R2 && resp_type(cmd) != RESP_R3 && resp_type(cmd) != RESP_R4)
			xfertyp |= XFERTYP_CICEN;
		if (resp_type(cmd) == RESP_R2)	/* It's a 136 bits response */
			xfertyp |= XFERTYP_RSPTYP_136;
		else if (resp_type(cmd) == RESP_R1B)
			xfertyp |= XFERTYP_RSPTYP_48_BUSY;
		else
			xfertyp |= XFERTYP_RSPTYP_48;
	}

	/* Command index */
	xfertyp |= XFERTYP_CMD(cmd->cmd_code);

	/* Set up data */
	if (cmd->data) {
		int timeout;
		uint wml_value = cmd->data->blksize/4;
		/* READ or WRITE */
		if (cmd->data->flags & DATA_DIR_READ) {
			if (wml_value > WML_RD_WML_MAX)
				wml_value = WML_RD_WML_MAX_VAL;
			esdhc_clrsetbits32(&regs->wml, WML_RD_WML_MASK, wml_value);
			esdhc_write32(&regs->dsaddr, cmd->data->buf);
			invalidate_dcache_range((ulong)cmd->data->buf,
							   (ulong)cmd->data->buf+cmd->data->blks*cmd->data->blksize);
		} else {
			if (wml_value > WML_WR_WML_MAX)
				wml_value = WML_WR_WML_MAX_VAL;
			if (ssp_mmc_is_wp(mmc)) {
				rt_kprintf("MMC: Can not write a locked card!\n");
				return UNUSABLE_ERR;
			}
			esdhc_clrsetbits32(&regs->wml, WML_WR_WML_MASK, wml_value << 16);
			esdhc_write32(&regs->dsaddr, cmd->data->buf);
			flush_dcache_range((ulong)cmd->data->buf,
							   (ulong)cmd->data->buf+cmd->data->blks*cmd->data->blksize);
		}
		esdhc_write32(&regs->blkattr, cmd->data->blks << 16 | cmd->data->blksize);
		/* Calculate the timeout period for data transactions */
		/*
		 * 1)Timeout period = (2^(timeout+13)) SD Clock cycles
		 * 2)Timeout period should be minimum 0.250sec as per SD Card spec
		 *  So, Number of SD Clock cycles for 0.25sec should be minimum
		 *		(SD Clock/sec * 0.25 sec) SD Clock cycles
		 *		= (mmc->clock * 1/4) SD Clock cycles
		 * As 1) >=  2)
		 * => (2^(timeout+13)) >= mmc->clock * 1/4
		 * Taking log2 both the sides
		 * => timeout + 13 >= log2(mmc->clock/4)
		 * Rounding up to next power of 2
		 * => timeout + 13 = log2(mmc->clock/4) + 1
		 * => timeout + 13 = fls(mmc->clock/4)
		 */
		timeout = fls(mmc->sdhc_clk/4);
		timeout -= 13;

		if (timeout > 14)
			timeout = 14;

		if (timeout < 0)
			timeout = 0;
		esdhc_clrsetbits32(&regs->sysctl, SYSCTL_TIMEOUT_MASK, timeout << 16);

		/* READ or WRITE */
		if (cmd->data->flags & DATA_DIR_READ)
			xfertyp |= XFERTYP_DTDSEL;
		xfertyp |= XFERTYP_DPSEL;
		xfertyp |= XFERTYP_DMAEN;
		if (cmd->data->blks > 1) {
			xfertyp |= XFERTYP_MSBSEL;
			xfertyp |= XFERTYP_BCEN;
		}
	}

	/* Mask all irqs */
	esdhc_write32(&regs->irqsigen, 0);

	/* Send the command */
	esdhc_write32(&regs->cmdarg, cmd->arg);
	esdhc_write32(&regs->mixctrl, (esdhc_read32(&regs->mixctrl) & 0xFFFFFF80) | (xfertyp & 0x7F));
	esdhc_write32(&regs->xfertyp, xfertyp & 0xFFFF0000);

	/* Wait for the command to complete */
	while (!(esdhc_read32(&regs->irqstat) & (IRQSTAT_CC | IRQSTAT_CTOE)));
	irqstat = esdhc_read32(&regs->irqstat);

	/* Check command timeout */
	if (irqstat & IRQSTAT_CTOE) {
		rt_kprintf("MMC: Command %ld timeout\n",
			cmd->cmd_code);
		err = TIMEOUT;
		goto out;
	}

	/* Check command errors */
	if (irqstat & CMD_ERR) {
		rt_kprintf("MMC: Command %ld error (status 0x%08x)!\n",
			cmd->cmd_code,
			(irqstat & CMD_ERR));
		err = COMM_ERR;
		goto out;
	}

	/* Workaround for ESDHC errata ENGcm03648 */
	if (!cmd->data && (resp_type(cmd) == RESP_R1B)) {
		int timeout = 2500;

		/* Poll on DATA0 line for cmd with busy signal for 250 ms */
		while (timeout > 0 && !(esdhc_read32(&regs->prsstat) &
					PRSSTAT_DAT0)) {
			udelay(100);
			timeout--;
		}

		if (timeout <= 0) {
			rt_kprintf("MMC: Timeout waiting for DAT0 to go high\n");
			err = TIMEOUT;
			goto out;
		}
	}

	/* Copy response to response buffer */
	if (resp_type(cmd) == RESP_R2) {
		u32 cmdrsp3, cmdrsp2, cmdrsp1, cmdrsp0;
		cmdrsp3 = esdhc_read32(&regs->cmdrsp3);
		cmdrsp2 = esdhc_read32(&regs->cmdrsp2);
		cmdrsp1 = esdhc_read32(&regs->cmdrsp1);
		cmdrsp0 = esdhc_read32(&regs->cmdrsp0);
		cmd->resp[3] = (cmdrsp0 << 8);
		cmd->resp[2] = (cmdrsp1 << 8) | (cmdrsp0 >> 24);
		cmd->resp[1] = (cmdrsp2 << 8) | (cmdrsp1 >> 24);
		cmd->resp[0] = (cmdrsp3 << 8) | (cmdrsp2 >> 24);
	} else {
		cmd->resp[0] = esdhc_read32(&regs->cmdrsp0);
	}

	/* Return if no data to process */
	if (!cmd->data)
		return 0;

	do {
		irqstat = esdhc_read32(&regs->irqstat);

		if (irqstat & IRQSTAT_DTOE) {
			rt_kprintf("MMC: Data timeout with Command %ld\n",
				cmd->cmd_code);
			err = TIMEOUT;
			goto out;
		}

		if (irqstat & DATA_ERR) {
			rt_kprintf("MMC: Data error with command %ld (status 0x%08x)!\n",
				cmd->cmd_code,
				(irqstat & DATA_ERR));
			err = COMM_ERR;
			goto out;
		}
	} while ((irqstat & DATA_COMPLETE) != DATA_COMPLETE);

	if (cmd->data->flags & DATA_DIR_READ) {
		invalidate_dcache_range((ulong)cmd->data->buf,
						   (ulong)cmd->data->buf+cmd->data->blks*cmd->data->blksize);
	}

out:
	/* Reset CMD and DATA portions on error */
	if (err) {
		esdhc_write32(&regs->sysctl, esdhc_read32(&regs->sysctl) | SYSCTL_RSTC);
		while (esdhc_read32(&regs->sysctl) & SYSCTL_RSTC);

		if (cmd->data) {
			esdhc_write32(&regs->sysctl, esdhc_read32(&regs->sysctl) | SYSCTL_RSTD);
			while ((esdhc_read32(&regs->sysctl) & SYSCTL_RSTD));
		}
	}

	esdhc_write32(&regs->irqstat, -1);
	return err;
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

static void esdhc_reset(volatile struct fsl_esdhc *regs)
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
	volatile struct fsl_esdhc *regs = (struct fsl_esdhc *)USDHC2_BASE_ADDR;
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
static void mmc_mci_request(struct rt_mmcsd_host *host, struct rt_mmcsd_req *req)
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
static void mmc_mci_set_iocfg(struct rt_mmcsd_host *host, struct rt_mmcsd_io_cfg *io_cfg)
{
	struct mmc_mci *mmc = (struct mmc_mci*)host->private_data;
	volatile struct fsl_esdhc *regs = (struct fsl_esdhc *)mmc->iobase;
	int timeout = 1000;

	if (io_cfg->power_mode == MMCSD_POWER_UP) {
		mmc->io_cfg = *io_cfg;

		/* Reset the entire host controller */
		esdhc_setbits32(&regs->sysctl, SYSCTL_RSTA);

		/* Wait until the controller is available */
		while ((esdhc_read32(&regs->sysctl) & SYSCTL_RSTA) && --timeout)
			udelay(1000);

		/* RSTA doesn't reset MMC_BOOT register, so manually reset it */
		esdhc_write32(&regs->mmcboot, 0x0);
		/* Reset MIX_CTRL and CLK_TUNE_CTRL_STATUS regs to 0 */
		esdhc_write32(&regs->mixctrl, 0x0);
		esdhc_write32(&regs->clktunectrlstatus, 0x0);

		/* Put VEND_SPEC to default value */
		esdhc_write32(&regs->vendorspec, VENDORSPEC_INIT);

		esdhc_setbits32(&regs->sysctl, SYSCTL_HCKEN | SYSCTL_IPGEN);

		/* Disable the BRR and BWR bits in IRQSTAT */
		esdhc_clrbits32(&regs->irqstaten, IRQSTATEN_BRR | IRQSTATEN_BWR);

		/* Put the PROCTL reg back to the default */
		esdhc_write32(&regs->proctl, PROCTL_INIT);

		/* Set timout to the maximum value */
		esdhc_clrsetbits32(&regs->sysctl, SYSCTL_TIMEOUT_MASK, 14 << 16);

		/* Set initial bit clock 400 KHz */
		set_bit_clock(mmc, 400000);
		mmc->io_cfg.clock = 400000;

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
	mmc_mci_request,
	mmc_mci_set_iocfg,
	RT_NULL,
	RT_NULL,
};

static struct mmc_mci mci = {
	.iobase = USDHC2_BASE_ADDR,
};

static void sd_monitor_thread_entry(void *parameter)
{
    uint32_t card,status;
    struct mmc_mci *mmc = (struct mmc_mci*)parameter;
    card  = ssp_mmc_is_cd(mmc);

    while(1)
    {
        status  = ssp_mmc_is_cd(mmc);
        if (status != card) {
            if (status) {
            	SD_LINK_PRINTF("MMC: No card detected!\n");
            	if (mci.host->card) {
            		mmcsd_change(mci.host);
            		mmcsd_wait_cd_changed(5000);
            		if (mci.host->card == NULL)
            			SD_LINK_PRINTF("Unmount /mnt/mmc ok!\n");
            		else
            			SD_LINK_PRINTF("Unmount /mnt/mmc failed!\n");
        		}
        	} else {
        		SD_LINK_PRINTF("MMC: Card detected!\n");
        		mci_run = 1;
        		mmcsd_change(mci.host);
        		mmcsd_wait_cd_changed(5000);
        	    if (dfs_mount("sd0", "/mnt/mmc", "elm", 0, 0) == 0)
        	    	SD_LINK_PRINTF("Mount /mnt/mmc ok!\n");
        	    else
        	    	SD_LINK_PRINTF("Mount /mnt/mmc failed!\n");
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
	mci.sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
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
                               &mci,
                               2048,
                               RT_THREAD_PRIORITY_MAX - 2,
                               20);
        if (tid != RT_NULL)
            rt_thread_startup(tid);
    }
}
