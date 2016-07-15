#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"

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
        *time = readl(REGS_RTC_BASE + HW_RTC_SECONDS);
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
        writel(*time, REGS_RTC_BASE + HW_RTC_SECONDS);
        break;
    }

	rt_mutex_release(&rtc_mutex);
    return RT_EOK;
}

static int RTC_Configuration(void)
{
    u32 clk = readl(REGS_RTC_BASE + HW_RTC_PERSISTENT0);
    clk |= BM_RTC_PERSISTENT0_CLOCKSOURCE | BM_RTC_PERSISTENT0_XTAL32KHZ_PWRUP;
    writel(clk, REGS_RTC_BASE + HW_RTC_PERSISTENT0);

    return mxs_reset_clock(REGS_RTC_BASE + HW_RTC_CTRL, 1);
}

static struct rt_device rtc;
void rt_hw_rtc_init(void)
{
    RTC_Configuration();

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
