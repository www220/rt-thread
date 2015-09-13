/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-01-13     weety       first version
 * 2013-07-21     weety       using serial component
 */

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdint.h>
#include "board.h"

#define CONFIG_UART_CLK		24000000
struct at91_uart {
	u32 membase;
	int irq;
	char* name;
	struct pin_group* pin;
	struct rt_serial_device* dev;
};

#if defined(RT_USING_UART1)
static struct pin_desc uart1_pins_desc[] = {
	{ PINID_AUART0_RX, PIN_FUN1, PAD_4MA, PAD_3V3, 1 },
	{ PINID_AUART0_TX, PIN_FUN1, PAD_4MA, PAD_3V3, 1 }
};
static struct pin_group uart1_pins = {
	.pins		= uart1_pins_desc,
	.nr_pins	= ARRAY_SIZE(uart1_pins_desc)
};
#endif

#if defined(RT_USING_UART2)
static struct pin_desc uart2_pins_desc[] = {
	{ PINID_AUART1_RX, PIN_FUN1, PAD_4MA, PAD_3V3, 1 },
	{ PINID_AUART1_TX, PIN_FUN1, PAD_4MA, PAD_3V3, 1 }
};
static struct pin_group uart2_pins = {
	.pins		= uart2_pins_desc,
	.nr_pins	= ARRAY_SIZE(uart2_pins_desc)
};
#endif

#if defined(RT_USING_DBGU)
static struct rt_serial_device serial_dbgu;
struct at91_uart dbgu = {
	REGS_UARTDBG_BASE,
	IRQ_DUART,
	"DbgU",
	0,
	&serial_dbgu
};
#endif

#if defined(RT_USING_UART1)
static struct rt_serial_device serial_uart1;
struct at91_uart uart1 = {
	REGS_UARTAPP0_BASE,
	IRQ_AUART0,
	"Uart1",
	&uart1_pins,
	&serial_uart1
};
#endif

#if defined(RT_USING_UART2)
static struct rt_serial_device serial_uart2;
struct at91_uart uart2 = {
	REGS_UARTAPP1_BASE,
	IRQ_AUART1,
	"Uart2",
	&uart2_pins,
	&serial_uart2
};
#endif

/**
 * This function will handle serial
 */
void rt_at91_usart_handler(int vector, void *param)
{
    struct at91_uart *uart = (struct at91_uart *)param;

#if defined(RT_USING_DBGU)
    if (uart == &dbgu)
    {
    	register rt_uint32_t ir = REG_RD(REGS_UARTDBG_BASE, HW_UARTDBGRIS);
    	if (ir & BM_UARTDBGRIS_RXRIS)
    	{
    		rt_hw_serial_isr((struct rt_serial_device *)uart->dev, RT_SERIAL_EVENT_RX_IND);
    		REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGICR, BM_UARTDBGICR_RXIC);
    	}
    }
    else
#endif
    {
    	register rt_uint32_t ir = readl(uart->membase + HW_UARTAPP_INTR);
    	if (ir & BM_UARTAPP_INTR_TXIS)
    	{
    		rt_hw_serial_isr((struct rt_serial_device *)uart->dev, RT_SERIAL_EVENT_RX_IND);
    		writel(BM_UARTAPP_INTR_TXIS, uart->membase + HW_UARTAPP_INTR_CLR);
    	}
    }
}

/**
* UART device in RT-Thread
*/
static rt_err_t at91_usart_configure(struct rt_serial_device *serial,
                                struct serial_configure *cfg)
{
    struct at91_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct at91_uart *)serial->parent.user_data;

#if defined(RT_USING_DBGU)
    if (uart == &dbgu)
    {
    	u32 quot;

    	/* Calculate and set baudrate */
    	quot = (CONFIG_UART_CLK * 4)	/ cfg->baud_rate;
    	REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGFBRD, quot & 0x3f);
    	REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGIBRD, quot >> 6);

    	/* Set 8n1 mode */
    	REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGLCR_H,
    		BM_UARTDBGLCR_H_WLEN);

    	/* Enable UART */
    	REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGCR,
    		BM_UARTDBGCR_TXE | BM_UARTDBGCR_RXE | BM_UARTDBGCR_UARTEN);
    	/* Enable UART Irq */
    	REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGIMSC, BM_UARTDBGIMSC_RXIM);
    }
    else
#endif
    {
    	u32 bm, ctrl, ctrl2, div;

    	ctrl = BM_UARTAPP_LINECTRL_FEN;
    	ctrl2 = readl(uart->membase + HW_UARTAPP_CTRL2);

		div = CONFIG_UART_CLK * 32 / cfg->baud_rate;
		ctrl |= BF_UARTAPP_LINECTRL_BAUD_DIVFRAC(div & 0x3F);
		ctrl |= BF_UARTAPP_LINECTRL_BAUD_DIVINT(div >> 6);

    	/* byte size */
    	switch (cfg->data_bits) {
    	case DATA_BITS_6:
    		bm = 1;
    		break;
    	case DATA_BITS_7:
    		bm = 2;
    		break;
    	case DATA_BITS_8:
    		bm = 3;
    		break;
    	default:
    		bm = 0;
    		break;
    	}
    	ctrl |= BF_UARTAPP_LINECTRL_WLEN(bm);

    	/* parity */
    	if (cfg->parity != PARITY_NONE) {
    		ctrl |= BM_UARTAPP_LINECTRL_PEN;
    		if (cfg->parity == PARITY_EVEN)
    			ctrl |= BM_UARTAPP_LINECTRL_EPS;
    	}

    	/* figure out the stop bits requested */
    	if (cfg->stop_bits == STOP_BITS_2)
    		ctrl |= BM_UARTAPP_LINECTRL_STP2;

    	/* figure out the hardware flow control settings */
   		ctrl2 &= ~(BM_UARTAPP_CTRL2_CTSEN|BM_UARTAPP_CTRL2_RTSEN);

    	writel(ctrl, uart->membase + HW_UARTAPP_LINECTRL);
    	writel(ctrl2, uart->membase + HW_UARTAPP_CTRL2);

        /* Enable UART */
    	writel(BM_UARTAPP_CTRL2_UARTEN | BM_UARTAPP_CTRL2_TXE | BM_UARTAPP_CTRL2_RXE,
    			     uart->membase + HW_UARTAPP_CTRL2_SET);
    	/* Enable UART Irq */
		writel(BM_UARTAPP_INTR_RXIEN, uart->membase + HW_UARTAPP_INTR_SET);
    }

    return RT_EOK;
}

static rt_err_t at91_usart_control(struct rt_serial_device *serial,
                              int cmd, void *arg)
{
    struct at91_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct at91_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        rt_hw_interrupt_mask(uart->irq);
        /* disable interrupt */
#if defined(RT_USING_DBGU)
        if (uart == &dbgu)
        	REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGIMSC, 0);
        else
#endif
    		writel(BM_UARTAPP_INTR_RXIEN, uart->membase + HW_UARTAPP_INTR_CLR);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        rt_hw_interrupt_umask(uart->irq);
        /* enable interrupt */
#if defined(RT_USING_DBGU)
        if (uart == &dbgu)
        	REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGIMSC, BM_UARTDBGIMSC_RXIM);
        else
#endif
    		writel(BM_UARTAPP_INTR_RXIEN, uart->membase + HW_UARTAPP_INTR_SET);
        break;
    }

    return RT_EOK;
}

static int at91_usart_putc(struct rt_serial_device *serial, char c)
{
    struct at91_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct at91_uart *)serial->parent.user_data;

#if defined(RT_USING_DBGU)
    if (uart == &dbgu)
    {
    	/* Wait for room in TX FIFO */
    	while (REG_RD(REGS_UARTDBG_BASE, HW_UARTDBGFR) & BM_UARTDBGFR_TXFF) ;
    	/* Write the data byte */
    	REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGDR, c);
    }
    else
#endif
    {
    	/* Wait for room in TX FIFO */
    	while (readl(uart->membase + HW_UARTAPP_STAT) & BM_UARTAPP_STAT_TXFF) ;
    	/* Write the data byte */
		writel(c, uart->membase + BP_UARTAPP_DATA_DATA);
    }

    return 1;
}

static int at91_usart_getc(struct rt_serial_device *serial)
{
    struct at91_uart *uart;
    int result = -1;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct at91_uart *)serial->parent.user_data;

#if defined(RT_USING_DBGU)
    if (uart == &dbgu)
    {
    	/* Read data byte */
    	if ((REG_RD(REGS_UARTDBG_BASE, HW_UARTDBGFR) & BM_UARTDBGFR_RXFE) == 0)
    		result = REG_RD(REGS_UARTDBG_BASE, HW_UARTDBGDR) & 0xff;
    }
    else
#endif
    {
    	/* Read data byte */
    	if ((readl(uart->membase + HW_UARTAPP_STAT) & BM_UARTAPP_STAT_TXFE) == 0)
    		result = readl(uart->membase + BP_UARTAPP_DATA_DATA) & 0xff;
    }

    return result;
}

static const struct rt_uart_ops at91_usart_ops =
{
    at91_usart_configure,
    at91_usart_control,
    at91_usart_putc,
    at91_usart_getc,
};

static void mxs_auart_reset(struct at91_uart *uart)
{
	int i;
	unsigned int reg;

	writel(BM_UARTAPP_CTRL0_SFTRST,
		     uart->membase + HW_UARTAPP_CTRL0_CLR);

	for (i = 0; i < 10000; i++) {
		reg = readl(uart->membase + HW_UARTAPP_CTRL0);
		if (!(reg & BM_UARTAPP_CTRL0_SFTRST))
			break;
		udelay(3);
	}

	writel(BM_UARTAPP_CTRL0_CLKGATE,
		     uart->membase + HW_UARTAPP_CTRL0_CLR);
}

static void GPIO_Configuration(struct at91_uart *uart)
{
#if defined(RT_USING_DBGU)
	if (uart == &dbgu)
	{
		/* Disable UART */
		REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGCR, 0);
		/* Mask interrupts */
		REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGIMSC, 0);
	}
	else
#endif
	{
	    /* Set up UART pins */
		pin_set_group(uart->pin);
		/* Reset HLK */
		mxs_auart_reset(uart);
		/* Disable UART */
		writel(BM_UARTAPP_CTRL2_UARTEN, uart->membase + HW_UARTAPP_CTRL2_SET);
		/* Mask interrupts */
		writel(0xfff, uart->membase + HW_UARTAPP_INTR_SET);
	}
}

static void NVIC_Configuration(struct at91_uart *uart)
{
    /* install interrupt handler */
    rt_hw_interrupt_install(uart->irq, rt_at91_usart_handler, uart, uart->name);
    rt_hw_interrupt_umask(uart->irq);
}

/**
 * This function will handle init uart
 */
void rt_hw_usart_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#if defined(RT_USING_DBGU)
    GPIO_Configuration(&dbgu);

	serial_dbgu.ops = &at91_usart_ops;
	serial_dbgu.config = config;

    NVIC_Configuration(&dbgu);

    /* register vcom device */
    rt_hw_serial_register(&serial_dbgu, "dbgu", RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &dbgu);
#endif

#if defined(RT_USING_UART1)
    GPIO_Configuration(&uart1);

	serial_uart1.ops = &at91_usart_ops;
	serial_uart1.config = config;

    NVIC_Configuration(&uart1);

    /* register vcom device */
    rt_hw_serial_register(&serial_uart1, "uart1", RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &uart1);
#endif

#if defined(RT_USING_UART2)
    GPIO_Configuration(&uart2);

    serial_uart2.ops = &at91_usart_ops;
    serial_uart2.config = config;

    NVIC_Configuration(&uart2);

    /* register vcom device */
    rt_hw_serial_register(&serial_uart2, "uart2", RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &uart2);
#endif
}
