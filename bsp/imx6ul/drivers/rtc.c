#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include <errno.h>

#undef ALIGN
#include <common.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>

/* These register offsets are relative to LP (Low Power) range */
#define SNVS_HPCR		0x08
#define SNVS_HPSR		0x14
#define SNVS_HPRTCMR	0x24
#define SNVS_HPRTCLR	0x28

#define SNVS_BASE ((void *)SNVS_BASE_ADDR)
#define CNTR_TO_SECS_SH		15
#define SNVS_HPCR_RTC_ENV	(1 << 0)

static u32 rtc_read_hp_counter(void __iomem *ioaddr)
{
	u64 read1, read2;

	do {
		read1 = readl(ioaddr + SNVS_HPRTCMR);
		read1 <<= 32;
		read1 |= readl(ioaddr + SNVS_HPRTCLR);

		read2 = readl(ioaddr + SNVS_HPRTCMR);
		read2 <<= 32;
		read2 |= readl(ioaddr + SNVS_HPRTCLR);
	} while (read1 != read2);

	/* Convert 47-bit counter to 32-bit raw second count */
	return (u32) (read1 >> CNTR_TO_SECS_SH);
}

static int snvs_rtc_enable(void __iomem *ioaddr, bool enable)
{
	unsigned long flags;
	int timeout = 1000;
	u32 lpcr;

	lpcr = readl(ioaddr + SNVS_HPCR);
	if (enable)
		lpcr |= SNVS_HPCR_RTC_ENV;
	else
		lpcr &= ~SNVS_HPCR_RTC_ENV;
	writel(lpcr, ioaddr + SNVS_HPCR);

	while (--timeout) {
		lpcr = readl(ioaddr + SNVS_HPCR);

		if (enable) {
			if (lpcr & SNVS_HPCR_RTC_ENV)
				break;
		} else {
			if (!(lpcr & SNVS_HPCR_RTC_ENV))
				break;
		}
	}

	if (!timeout)
		return -ETIMEDOUT;

	return 0;
}

static int snvs_rtc_read_time(void __iomem *ioaddr, time_t *time)
{
	*time = rtc_read_hp_counter(ioaddr);

	return 0;
}

static int snvs_rtc_set_time(void __iomem *ioaddr, time_t *stime)
{
	unsigned long time = *stime;

	/* Disable RTC first */
	snvs_rtc_enable(ioaddr, false);

	/* Write 32-bit time to 47-bit timer, leaving 15 LSBs blank */
	writel(time << CNTR_TO_SECS_SH, ioaddr + SNVS_HPRTCLR);
	writel(time >> (32 - CNTR_TO_SECS_SH), ioaddr + SNVS_HPRTCMR);

	/* Enable RTC again */
	snvs_rtc_enable(ioaddr, true);

	return 0;
}

static struct rt_mutex rtc_mutex;
static rt_err_t rt_rtc_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_size_t rt_rtc_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    return RT_EOK;
}

static rt_err_t rt_rtc_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t rt_rtc_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    time_t *time = (time_t *)args;

    RT_ASSERT(dev != RT_NULL);
    rt_mutex_take(&rtc_mutex, RT_WAITING_FOREVER);
	
    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        snvs_rtc_read_time(SNVS_BASE, time);
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
        snvs_rtc_set_time(SNVS_BASE, time);
        break;
    }

	rt_mutex_release(&rtc_mutex);
    return RT_EOK;
}

static struct rt_device rtc;
void rt_hw_rtc_init(void)
{
    writel(0xffffffff, SNVS_BASE + SNVS_HPSR);
    snvs_rtc_enable(SNVS_BASE, true);

    /* register rtc device */
    rtc.type	= RT_Device_Class_RTC;
    rtc.init 	= rt_rtc_init;
    rtc.open 	= rt_rtc_open;
    rtc.close	= RT_NULL;
    rtc.read 	= rt_rtc_read;
    rtc.write	= RT_NULL;
    rtc.control = rt_rtc_control;

    /* no private */
    rtc.user_data = RT_NULL;

	rt_mutex_init(&rtc_mutex, "rtc", RT_IPC_FLAG_FIFO);
    rt_device_register(&rtc, "rtc", RT_DEVICE_FLAG_RDWR);
    return;
}

extern void list_date(void);
extern rt_err_t set_time(rt_uint32_t hour, rt_uint32_t minute, rt_uint32_t second);
extern rt_err_t set_date(rt_uint32_t year, rt_uint32_t month, rt_uint32_t day);

int setdate(int argc, char** argv)
{
    if (argc <= 1)
    {
        list_date();
        return 0;
    }
    if (argc < 5 || (rt_strcasecmp(argv[1],"-d") != 0 && rt_strcasecmp(argv[1],"-t") != 0))
    {
      rt_kprintf("Usage: date -d 2004 12 1\n");
      rt_kprintf("Usage: date -t 12 15 36\n");
      return 0;
    }
    if (rt_strcasecmp(argv[1],"-d") == 0)
    {
        set_date(atol(argv[2]), atol(argv[3]), atol(argv[4]));
        return 0;
    }
    if (rt_strcasecmp(argv[1],"-t") == 0)
    {
        set_time(atol(argv[2]), atol(argv[3]), atol(argv[4]));
        return 0;
    }
    return 0;
}

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT_ALIAS(setdate, __cmd_date, show date and time.)
#endif
