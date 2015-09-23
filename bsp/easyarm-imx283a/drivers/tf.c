#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "tf.h"

struct mxs_mci {
	struct rt_mmcsd_host *host;
	struct rt_mmcsd_req *req;
	struct rt_mmcsd_cmd *cmd;
	struct rt_timer timer;
	//struct rt_semaphore sem_ack;
	rt_uint32_t *buf;
	rt_uint32_t current_status;
};

void rt_hw_ssp_init(void)
{

}

static const struct rt_mmcsd_host_ops ops = {
	at91_mci_request,
	at91_mci_set_iocfg,
	RT_NULL,
	at91_mci_enable_sdio_irq,
};

static struct mxs_mci mci = {

};

void tf_init(void)
{
	struct rt_mmcsd_host *host;

	rt_mmcsd_core_init();
	rt_mmcsd_blk_init();

	host = mmcsd_alloc_host();
	if (!host)
	{
		rt_kprintf("alloc mmchost failed\n");
		return;
	}

	host->ops = &ops;
	host->freq_min = 375000;
	host->freq_max = 25000000;
	host->valid_ocr = VDD_32_33 | VDD_33_34;
	host->flags = MMCSD_BUSWIDTH_4 | MMCSD_MUTBLKWRITE | \
				MMCSD_SUP_HIGHSPEED | MMCSD_SUP_SDIO_IRQ;
	host->max_seg_size = 65535;
	host->max_dma_segs = 2;
	host->max_blk_size = 512;
	host->max_blk_count = 4096;
	host->private_data = mci;

	mmcsd_change(host);
}
