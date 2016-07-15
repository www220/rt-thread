#include <rtthread.h>
#include <rthw.h>
#include "board.h"

#define CPU_USAGE_CALC_TICK    10
#define CPU_USAGE_LOOP        100

static rt_uint8_t  cpu_usage_major = 0, cpu_usage_minor= 0;
static rt_uint32_t total_count = 0;
static rt_uint32_t update_tick = 0;

void cpu_usage_idle_hook()
{
    rt_tick_t tick;
    rt_uint32_t count;
    volatile rt_uint32_t loop;
    wtdog_count = 0;

    if (total_count == 0)
    {
        /* get total count */
        rt_enter_critical();
        tick = rt_tick_get();
        while(rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
        {
            total_count ++;
            loop = 0;
            while (loop < CPU_USAGE_LOOP) loop ++;
        }
        rt_exit_critical();
    }

    count = 0;
    /* get CPU usage */
    tick = rt_tick_get();
    while (rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
    {
        count ++;
        loop  = 0;
        while (loop < CPU_USAGE_LOOP) loop ++;
    }

    /* calculate major and minor */
    if (count < total_count)
    {
        count = total_count - count;
        cpu_usage_major = (count * 100) / total_count;
        cpu_usage_minor = ((count * 100) % total_count) * 100 / total_count;
        update_tick = rt_tick_get();
    }
    else
    {
        total_count = count;

        /* no CPU usage */
        cpu_usage_major = 0;
        cpu_usage_minor = 0;
        update_tick = rt_tick_get();
    }
}

#if defined(RT_USING_FINSH)
#include <finsh.h>
void cpu_usage()
{
    //long time no update£¬ 100%
    if ((rt_tick_get() - update_tick) > 500)
    {
        cpu_usage_major = 100;
        cpu_usage_minor = 0;
    }
    rt_kprintf("Cpu Usage: %d.%d(%d)\n",cpu_usage_major,cpu_usage_minor,total_count);
}

void rt_usage_info(rt_uint32_t *major, rt_uint32_t *minor)
{
    //long time no update£¬ 100%
    if ((rt_tick_get() - update_tick) > 500)
    {
        cpu_usage_major = 100;
        cpu_usage_minor = 0;
    }
    if (major)
        *major = cpu_usage_major;
    if (minor)
        *minor = cpu_usage_minor;
}
RTM_EXPORT(rt_usage_info);

FINSH_FUNCTION_EXPORT(cpu_usage, cpu usage);
MSH_CMD_EXPORT(cpu_usage, cpu usage);
#endif //RT_USING_FINSH
