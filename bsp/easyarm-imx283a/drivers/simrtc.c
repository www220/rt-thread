#include <string.h>
#include <time.h>
#include <rtthread.h>
#include <windows.h>
#include <board.h>
#include "devrtc.h"
#include <stdlib.h>

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
    time_t *time;
    struct tm time_temp;

    RT_ASSERT(dev != RT_NULL);
    memset(&time_temp, 0, sizeof(struct tm));
    rt_mutex_take(&rtc_mutex, RT_WAITING_FOREVER);
	
    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
    {
        SYSTEMTIME st;
        time = (time_t *)args;

        /* Get the current Time */
        GetLocalTime(&st);
        /* Years since 1900 : 0-99 range */
        time_temp.tm_year = st.wYear - 1900;
        /* Months *since* january 0-11 : RTC_Month_Date_Definitions 1 - 12 */
        time_temp.tm_mon = st.wMonth - 1;
        /* Day of the month 1-31 : 1-31 range */
        time_temp.tm_mday = st.wDay;
        /* Hours since midnight 0-23 : 0-23 range */
        time_temp.tm_hour = st.wHour;
        /* Minutes 0-59 : the 0-59 range */
        time_temp.tm_min = st.wMinute;
        /* Seconds 0-59 : the 0-59 range */
        time_temp.tm_sec = st.wSecond;

        *time = mktime(&time_temp);
        break;
    }

    case RT_DEVICE_CTRL_RTC_SET_TIME:
    {
        SYSTEMTIME st;
        time = (time_t *)args;

        time_temp = *localtime(time);

        /* 0-99 range              : Years since 1900 */
        st.wYear = time_temp.tm_year + 1900;
        /* RTC_Month_Date_Definitions 1 - 12 : Months *since* january 0-11 */
        st.wMonth = time_temp.tm_mon + 1;
        /* 1-31 range : Day of the month 1-31 */
        st.wDay = time_temp.tm_mday;
        /* 0-23 range : Hours since midnight 0-23 */
        st.wHour = time_temp.tm_hour;
        /* the 0-59 range : Minutes 0-59 */
        st.wMinute = time_temp.tm_min;
        /* the 0-59 range : Seconds 0-59 */
        st.wSecond = time_temp.tm_sec;

        SetLocalTime(&st);
        break;
    }
    break;
    }

    rt_mutex_release(&rtc_mutex);
    return RT_EOK;
}

static struct rt_device rtc;
void rt_hw_rtc_init(void)
{
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