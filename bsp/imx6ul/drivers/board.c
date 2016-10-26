/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2016, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-11-20     Bernard    the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include "board.h"
#include <cortex_a.h>

#undef ALIGN
#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/imx-common/iomux-v3.h>

#define CONFIG_WD_PERIOD 2000000
void udelay(unsigned long usec)
{
	ulong kv;

	do {
		kv = usec > CONFIG_WD_PERIOD ? CONFIG_WD_PERIOD : usec;
		__udelay (kv);
		usec -= kv;
	} while(usec);
}
void mdelay(unsigned long msec)
{
	while (msec--)
		udelay(1000);
}

#define EPIT_BASE    EPIT1_BASE_ADDR
#define __REG(x)     (*((volatile unsigned long *)(x)))
static void rt_hw_timer_isr(int vector, void *param)
{
    rt_tick_increase();
    __REG(EPIT_BASE+4) = 0x01;
}

void rt_hw_timer_init(void)
{
    uint32_t clk = mxc_get_clock(MXC_IPG_CLK);

    __REG(EPIT_BASE) = 0x00010000;
    while (__REG(EPIT_BASE) & 0x00010000);

    __REG(EPIT_BASE) = ((clk/1000000-1)<<4)|0x012a000a;

    __REG(EPIT_BASE+8) = 999;
    __REG(EPIT_BASE) = __REG(EPIT_BASE)|0x05;

    rt_hw_interrupt_install(IMX_INT_EPIT1, rt_hw_timer_isr, RT_NULL, "Tick");
    rt_hw_interrupt_umask(IMX_INT_EPIT1);
}

/* None-cached RAM DMA */
unsigned char * dma_align_mem = (unsigned char *)0x00900000;
unsigned char * dma_align_max = (unsigned char *)0x80000000;

/* tlb base mem */
extern char __l1_page_table_start;
u32 *rtt_tlb_table;
/* mmu config */
u32 rtt_bi_dram_start[CONFIG_NR_DRAM_BANKS];
u32 rtt_bi_dram_size[CONFIG_NR_DRAM_BANKS];

#define PWM_PAD_CTRL    (PAD_CTL_HYS | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

static iomux_v3_cfg_t const iox_pads[] = {
	MX6_PAD_NAND_CE1_B__GPIO4_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NAND_DQS__GPIO4_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER0__GPIO5_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER1__GPIO5_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER2__GPIO5_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER3__GPIO5_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER4__GPIO5_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER5__GPIO5_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER6__GPIO5_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER8__GPIO5_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER9__GPIO5_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),

	MX6_PAD_LCD_DATA22__MQS_RIGHT    | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_LCD_DATA23__MQS_LEFT     | MUX_PAD_CTRL(NO_PAD_CTRL),

	MX6_PAD_LCD_DATA18__PWM5_OUT     | MUX_PAD_CTRL(PWM_PAD_CTRL),
	MX6_PAD_LCD_DATA19__PWM6_OUT     | MUX_PAD_CTRL(PWM_PAD_CTRL),
};

/**
 * This function will initialize hardware board
 */
void rt_hw_board_init(void)
{
    imx_iomux_v3_setup_multiple_pads(iox_pads, ARRAY_SIZE(iox_pads));
    gpio_direction_output(PIN_WDT, 0);
    gpio_direction_output(PIN_RUN, 0);
    gpio_direction_output(PIN_ERR, 0);
    gpio_direction_input(PIN_PZ1);
    gpio_direction_input(PIN_PZ2);

    PZ[0] = gpio_get_value(PIN_PZ1);
    PZ[1] = gpio_get_value(PIN_PZ2);

    s_init();
    arch_cpu_init();
    timer_init();
    board_postclk_init();

    rtt_tlb_table = (u32 *)&__l1_page_table_start;
    rt_memset(rtt_tlb_table, 0, 4*4096);
    rtt_bi_dram_start[0] = 0x80100000;
    rtt_bi_dram_size[0]  = 0x10000000;
    enable_caches();

    rt_hw_interrupt_init();
    enable_neon_fpu();
    disable_strict_align_check();

    /* initialize uart */
    rt_hw_uart_init();
    rt_console_set_device(CONSOLE_DEVICE);
    gpio_set_value(PIN_WDT, 1);

    /* initialize timer */
    rt_hw_timer_init();

#ifdef RT_USING_MTD_NAND
    /* initialize gpmi */
    rt_hw_mtd_nand_init();
#endif
    gpio_set_value(PIN_WDT, 0);

#ifdef RT_USING_SDIO
    /* initialize ssp */
    rt_hw_ssp_init();
#endif
    gpio_set_value(PIN_WDT, 1);

#ifdef RT_USING_I2C
    /* initialize ssp */
    rt_hw_i2c_init();
#endif
    gpio_set_value(PIN_WDT, 0);

    rt_kprintf("\n\n");
    print_cpuinfo();
    rt_kprintf("MEM:   256 MiB\n");
    rt_kprintf("NAND:  256 MiB\n");
    gpio_set_value(PIN_WDT, 1);
}

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_VAR_EXPORT(dma_align_mem, finsh_type_int, dma ram pos for finsh)
FINSH_VAR_EXPORT(dma_align_max, finsh_type_int, dma_max pos for finsh)
#endif

/*@}*/
