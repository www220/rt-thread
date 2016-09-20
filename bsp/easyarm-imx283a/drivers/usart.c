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
#ifdef DFS_USING_SELECT
#include "linux-syscall.h"
#include "linux-usedef.h"
#include <errno.h>
#include <dfs_def.h>
#include <dfs.h>
#ifndef RT_USING_DFS_DEVFS
#error "defined RT_USING_DFS_DEVFS"
#endif
extern int rt_process_kill(rt_process_t module, int pid, int sig);
#endif

#define CONFIG_UART_CLK		24000000
#define CONFIG_INT_MASK		BM_UARTAPP_INTR_RXIEN|BM_UARTAPP_INTR_RTIEN

#define TTY_FILE_SIZE 32
struct tty_device
{
	struct rt_device parent;

	rt_device_t device; /* the actual device */
	struct winsize win;
	struct termios ios;
	int file[TTY_FILE_SIZE];
	pid_t tpid[2];
};
static struct rt_event rx_sem;
static void rt_tty_init(rt_device_t device, struct tty_device* tty, const char* att_name);
extern int __rt_ffs(int value);

struct at91_uart {
	u32 membase;
	int irq;
	u32 id;
	char* name;
	struct pin_group* pin;
	struct rt_serial_device* dev;
	struct tty_device* tty;
};

#if defined(RT_USING_UART1)
static struct pin_desc uart1_pins_desc[] = {
	{ PINID_AUART0_RX, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_AUART0_TX, PIN_FUN1, PAD_4MA, PAD_3V3, 0 }
};
static struct pin_group uart1_pins = {
	.pins		= uart1_pins_desc,
	.nr_pins	= ARRAY_SIZE(uart1_pins_desc)
};
#endif

#if defined(RT_USING_UART2)
static struct pin_desc uart2_pins_desc[] = {
	{ PINID_AUART1_RX, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_AUART1_TX, PIN_FUN1, PAD_4MA, PAD_3V3, 0 }
};
static struct pin_group uart2_pins = {
	.pins		= uart2_pins_desc,
	.nr_pins	= ARRAY_SIZE(uart2_pins_desc)
};
#endif

#if defined(RT_USING_UART3)
static struct pin_desc uart3_pins_desc[] = {
	{ PINID_SSP2_SCK, PIN_FUN2, PAD_4MA, PAD_3V3, 0 },
	{ PINID_SSP2_MOSI, PIN_FUN2, PAD_4MA, PAD_3V3, 0 }
};
static struct pin_group uart3_pins = {
	.pins		= uart3_pins_desc,
	.nr_pins	= ARRAY_SIZE(uart3_pins_desc)
};
#endif

#if defined(RT_USING_UART4)
static struct pin_desc uart4_pins_desc[] = {
	{ PINID_SSP2_MISO, PIN_FUN2, PAD_4MA, PAD_3V3, 0 },
	{ PINID_SSP2_SS0, PIN_FUN2, PAD_4MA, PAD_3V3, 0 }
};
static struct pin_group uart4_pins = {
	.pins		= uart4_pins_desc,
	.nr_pins	= ARRAY_SIZE(uart4_pins_desc)
};
#endif

#if defined(RT_USING_UART5)
static struct pin_desc uart5_pins_desc[] = {
	{ PINID_SAIF0_BITCLK, PIN_FUN3, PAD_4MA, PAD_3V3, 0 },
	{ PINID_SAIF0_SDATA0, PIN_FUN3, PAD_4MA, PAD_3V3, 0 }
};
static struct pin_group uart5_pins = {
	.pins		= uart5_pins_desc,
	.nr_pins	= ARRAY_SIZE(uart5_pins_desc)
};
#endif

#if defined(RT_USING_DBGU)
static struct rt_serial_device serial_dbgu;
static struct tty_device tty_dbgu;
static struct at91_uart dbgu = {
	REGS_UARTDBG_BASE,
	IRQ_DUART,
	0,
	"DbgU",
	0,
	&serial_dbgu,
	&tty_dbgu
};
#endif

#if defined(RT_USING_UART1)
static struct rt_serial_device serial_uart1;
static struct tty_device tty_uart1;
static struct at91_uart uart1 = {
	REGS_UARTAPP0_BASE,
	IRQ_AUART0,
	PIN_UART1,
	"Uart1",
	&uart1_pins,
	&serial_uart1,
	&tty_uart1
};
#endif

#if defined(RT_USING_UART2)
static struct rt_serial_device serial_uart2;
static struct tty_device tty_uart2;
static struct at91_uart uart2 = {
	REGS_UARTAPP1_BASE,
	IRQ_AUART1,
	PIN_UART2,
	"Uart2",
	&uart2_pins,
	&serial_uart2,
	&tty_uart2
};
#endif

#if defined(RT_USING_UART3)
static struct rt_serial_device serial_uart3;
static struct tty_device tty_uart3;
static struct at91_uart uart3 = {
	REGS_UARTAPP2_BASE,
	IRQ_AUART2,
	PIN_UART3,
	"Uart3",
	&uart3_pins,
	&serial_uart3,
	&tty_uart3
};
#endif

#if defined(RT_USING_UART4)
static struct rt_serial_device serial_uart4;
static struct tty_device tty_uart4;
static struct at91_uart uart4 = {
	REGS_UARTAPP3_BASE,
	IRQ_AUART3,
	PIN_UART4,
	"Uart4",
	&uart4_pins,
	&serial_uart4,
	&tty_uart4
};
#endif

#if defined(RT_USING_UART5)
static struct rt_serial_device serial_uart5;
static struct tty_device tty_uart5;
static struct at91_uart uart5 = {
	REGS_UARTAPP4_BASE,
	IRQ_AUART4,
	0,
	"Uart5",
	&uart5_pins,
	&serial_uart5,
	&tty_uart5
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
    	if (ir & BM_UARTAPP_INTR_RXIS)
    	{
    		rt_hw_serial_isr((struct rt_serial_device *)uart->dev, RT_SERIAL_EVENT_RX_IND);
    		writel(BM_UARTAPP_INTR_RXIS, uart->membase + HW_UARTAPP_INTR_CLR);
    	}
    	if (ir & BM_UARTAPP_INTR_RTIS)
    	{
    		if ((readl(uart->membase + HW_UARTAPP_STAT) & BM_UARTAPP_STAT_RXFE) == 0)
    			rt_hw_serial_isr((struct rt_serial_device *)uart->dev, RT_SERIAL_EVENT_RX_IND);
    		writel(BM_UARTAPP_INTR_RTIS, uart->membase + HW_UARTAPP_INTR_CLR);
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
    serial->config.baud_rate = cfg->baud_rate;
    serial->config.data_bits = cfg->data_bits;
    serial->config.parity = cfg->parity;
    serial->config.stop_bits = cfg->stop_bits;

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
    	if (serial->parent.open_flag & RT_DEVICE_FLAG_INT_RX)
    	{
    		/* Enable UART Irq */
    		REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGIMSC, BM_UARTDBGIMSC_RXIM);
    	}
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

		if (uart->id)
			pin_gpio_set(uart->id,0);
        /* Enable UART */
    	writel(BM_UARTAPP_CTRL2_UARTEN | BM_UARTAPP_CTRL2_TXE | BM_UARTAPP_CTRL2_RXE,
    			     uart->membase + HW_UARTAPP_CTRL2_SET);
    	if (serial->parent.open_flag & RT_DEVICE_FLAG_INT_RX)
    	{
    		/* Enable UART Irq */
    		writel(CONFIG_INT_MASK, uart->membase + HW_UARTAPP_INTR_SET);
    	}
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
        {
    		if (uart->id)
    			pin_gpio_set(uart->id,0);
    		writel(CONFIG_INT_MASK, uart->membase + HW_UARTAPP_INTR_CLR);
        }
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
        {
    		if (uart->id)
    			pin_gpio_set(uart->id,0);
    		writel(CONFIG_INT_MASK, uart->membase + HW_UARTAPP_INTR_SET);
        }
        break;
    case RT_DEVICE_CTRL_CHAR_GETREAD: {
        struct rt_serial_rx_fifo* rx_fifo = (struct rt_serial_rx_fifo*) serial->serial_rx;
        *((int *)arg) = (rx_fifo && (rx_fifo->get_index!=rx_fifo->put_index))?1:0;
        break; }
    case RT_DEVICE_CTRL_CHAR_GETWRITE:
        *((int *)arg) = 0;
        break;
    case RT_DEVICE_CTRL_FLSH: {
        rt_base_t level = rt_hw_interrupt_disable();
        struct rt_serial_rx_fifo* rx_fifo = (struct rt_serial_rx_fifo*) serial->serial_rx;
        if (rx_fifo) rx_fifo->get_index = rx_fifo->put_index;
        rt_hw_interrupt_enable(level);
        break; }
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
		writel(c, uart->membase + HW_UARTAPP_DATA);
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
#ifdef DFS_USING_SELECT
    	//控制字符串检索到以后发送控制命令
    	if (result != -1 && uart->tty->tpid[0] != 0)
    	{
    		if (result == uart->tty->ios.c_cc[VINTR])
    			rt_process_kill(RT_NULL, -uart->tty->tpid[0], SIGINT);
    		else if (result == uart->tty->ios.c_cc[VQUIT])
    			rt_process_kill(RT_NULL, -uart->tty->tpid[0], SIGQUIT);
    		else if (result == uart->tty->ios.c_cc[VKILL])
    			rt_process_kill(RT_NULL, -uart->tty->tpid[0], SIGTERM);
    	}
#endif
    }
    else
#endif
    {
    	/* Read data byte */
    	if ((readl(uart->membase + HW_UARTAPP_STAT) & BM_UARTAPP_STAT_RXFE) == 0)
    		result = readl(uart->membase + HW_UARTAPP_DATA) & 0xff;
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
		/* Timer INIT */
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
    rt_tty_init(&serial_dbgu.parent, &tty_dbgu, CONSOLE_DEVICE);
#endif

#if defined(RT_USING_UART1)
    GPIO_Configuration(&uart1);

	serial_uart1.ops = &at91_usart_ops;
	serial_uart1.config = config;

    NVIC_Configuration(&uart1);

    /* register vcom device */
    rt_hw_serial_register(&serial_uart1, "uart1", RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &uart1);
    rt_tty_init(&serial_uart1.parent, &tty_uart1, "ttyS0");
#endif

#if defined(RT_USING_UART2)
    GPIO_Configuration(&uart2);

    serial_uart2.ops = &at91_usart_ops;
    serial_uart2.config = config;

    NVIC_Configuration(&uart2);

    /* register vcom device */
    rt_hw_serial_register(&serial_uart2, "uart2", RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &uart2);
    rt_tty_init(&serial_uart2.parent, &tty_uart2, "ttyS1");
#endif

#if defined(RT_USING_UART3)
    GPIO_Configuration(&uart3);

    serial_uart3.ops = &at91_usart_ops;
    serial_uart3.config = config;

    NVIC_Configuration(&uart3);

    /* register vcom device */
    rt_hw_serial_register(&serial_uart3, "uart3", RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &uart3);
    rt_tty_init(&serial_uart3.parent, &tty_uart3, "ttyS2");
#endif

#if defined(RT_USING_UART4)
    GPIO_Configuration(&uart4);

    serial_uart4.ops = &at91_usart_ops;
    serial_uart4.config = config;

    NVIC_Configuration(&uart4);

    /* register vcom device */
    rt_hw_serial_register(&serial_uart4, "uart4", RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &uart4);
    rt_tty_init(&serial_uart4.parent, &tty_uart4, "ttyS3");
#endif

#if defined(RT_USING_UART5)
    GPIO_Configuration(&uart5);

    serial_uart5.ops = &at91_usart_ops;
    serial_uart5.config = config;

    NVIC_Configuration(&uart5);

    /* register vcom device */
    rt_hw_serial_register(&serial_uart5, "uart5", RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &uart5);
    rt_tty_init(&serial_uart5.parent, &tty_uart5, "ttyS4");
#endif

    rt_event_init(&rx_sem, "uartrx", 0);
}

rt_size_t rt_device_write_485(rt_device_t dev,
                          rt_off_t    pos,
                          const void *buffer,
                          rt_size_t   size)
{
	struct at91_uart *uart = (struct at91_uart *)dev->user_data;
	rt_size_t ret;

	if (uart->id > 0)
		pin_gpio_set(uart->id,1);

	ret = rt_device_write(dev, pos, buffer, size);

	if (uart->id > 0)
	{
    	/* Wait for TX Complete */
    	while (readl(uart->membase + HW_UARTAPP_STAT) & BM_UARTAPP_STAT_BUSY)
    		udelay(10);
		pin_gpio_set(uart->id,0);
	}
	return ret;
}
RTM_EXPORT(rt_device_write_485);

#ifdef DFS_USING_SELECT
#if 0
#define PRESS_DEBUG_DEVFILE rt_kprintf
#else
#define PRESS_DEBUG_DEVFILE(...)
#endif

volatile int tty_rx_inxpz = 1;
static rt_err_t tty_rx_ind(rt_device_t dev, rt_size_t size)
{
    struct at91_uart *uart;
    struct tty_device* device;

    uart = (struct at91_uart *)dev->user_data;
    RT_ASSERT(uart != RT_NULL);
    device = (struct tty_device*)uart->tty;
    RT_ASSERT(device != RT_NULL);

    /* release semaphore to let finsh thread rx data */
    if (device->parent.rx_indicate)
    {
        if (uart->irq != IRQ_DUART || tty_rx_inxpz)
        {
            device->parent.rx_indicate(&device->parent, size);
            return RT_EOK;
        }
    }

    //发出事件信号
    if (uart->irq == IRQ_DUART)
        rt_event_send(&rx_sem, 0x01);
    else
        rt_event_send(&rx_sem, (1<<(uart->irq-IRQ_AUART0+1)));
    //其他串口不支持select，防止频繁中断影响效率
    if (uart->irq != IRQ_DUART)
        return RT_EOK;

    /* lock interrupt */
    {register rt_base_t temp = rt_hw_interrupt_disable();
    struct rt_list_node *n;
    struct dfs_select_info *sinfo;
    /* find a suitable position */
    for (n = dfs_select_list.list.next; n != &dfs_select_list.list; n = n->next)
    {
    	int i,pz = 0;
    	sinfo = rt_list_entry(n, struct dfs_select_info, list);
    	//遍历找到是否关注该句柄
    	for (i=0; i<TTY_FILE_SIZE; i++)
    	{
    		if (device->file[i] == 0)
    			break;
    		if (device->file[i] > sinfo->maxfdp)
    			continue;
    		int fileno = device->file[i]-1;
    		if (sinfo->reqset[0] && (sinfo->reqset[0][fileno/TTY_FILE_SIZE] & (1<<(fileno%TTY_FILE_SIZE))))
    		{
    			sinfo->recvset[0][fileno/TTY_FILE_SIZE] |= (1<<(fileno%TTY_FILE_SIZE));
    			pz = 1;
    		}
    	}
    	//释放对象
    	if (pz)
    	{
    		rt_sem_release(&sinfo->sem);
    		pz = 0;
    	}
    }
    /* unlock interrupt */
    rt_hw_interrupt_enable(temp);}

    return RT_EOK;
}

/* common device interface */
static rt_err_t tty_init(rt_device_t dev)
{
	rt_err_t ret = RT_EOK;
	struct tty_device* device;

	device = (struct tty_device*)dev;
	RT_ASSERT(device != RT_NULL);

	memset(&device->win,0,sizeof(struct winsize));
	device->win.ws_col = 132;
	device->win.ws_row = 43;

    memset(&device->ios,0,sizeof(struct termios));
    device->ios.c_iflag = (BRKINT | ISTRIP | ICRNL | IMAXBEL | IXON | IXANY);
    device->ios.c_oflag = (OPOST | ONLCR);
    device->ios.c_cflag = (B115200 | CS8 | CREAD | HUPCL | CLOCAL);
    device->ios.c_lflag = (ICANON | ISIG | IEXTEN | ECHOE|ECHOKE|ECHOCTL);

    static cc_t def_c_cc[NCCS] = {CINTR,CQUIT,CERASE,CKILL,CEOF,CTIME,CMIN,0,CSTART,CSTOP,CSUSP,
    														 0,CREPRINT,CDISCARD,CWERASE,CLNEXT};
    memcpy(device->ios.c_cc,def_c_cc,sizeof(def_c_cc));
    memset(&device->file,0,sizeof(device->file));
    memset(&device->tpid,0,sizeof(device->tpid));
	return ret;
}

static rt_err_t tty_open(rt_device_t dev, rt_uint16_t oflag)
{
	rt_err_t ret = RT_EOK;
	struct tty_device* device;

	device = (struct tty_device*)dev;
	RT_ASSERT(device != RT_NULL);

	//设备已经打开了，不要重复打开了,系统初始化的时候不能开中断
	if (device->parent.ref_count && (device->device->open_flag&RT_DEVICE_FLAG_INT_RX))
		return RT_EOK;

	if (oflag&RT_DEVICE_FLAG_INT_RX)
		rt_device_set_rx_indicate(device->device, tty_rx_ind);

	/* open this device and set the new device in finsh shell */
	return rt_device_open(device->device, oflag);
}

static rt_err_t tty_close(rt_device_t dev)
{
	rt_err_t ret = RT_EOK;
	struct tty_device* device;

	device = (struct tty_device*)dev;
	RT_ASSERT(device != RT_NULL);

	return rt_device_close(device->device);
}

static rt_size_t tty_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	int ret = 0,set = 0;
	struct tty_device* device;
	struct at91_uart *uart;

	device = (struct tty_device*)dev;
	RT_ASSERT(device != RT_NULL);
	uart = (struct at91_uart *)device->device->user_data;
	RT_ASSERT(uart != RT_NULL);

	//缓存数量不足最小字符
	if (device->ios.c_cc[VMIN] && size<device->ios.c_cc[VMIN])
	{
		errno = EINVAL;
		return -1;
	}

	if (uart->irq == IRQ_DUART)
		set = 0x01;
	else
		set = (1<<(uart->irq-IRQ_AUART0+1));

	//接受前清理事件信号
	rt_event_recv(&rx_sem,set,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,0,0);
	ret = rt_device_read(device->device, 0, buffer, size); if (ret < 0) ret = 0;
	if (ret == 0)
	{
		//msh使用的是异步读取
		if (uart->irq == IRQ_DUART && tty_rx_inxpz)
			return ret;
		//获取读取文件
		int i,fileno = -1;
		for (i=0; i<TTY_FILE_SIZE; i++)
		{
			if (device->file[i] == 0)
				break;
			if (device->file[i] == pos)
			{
				fileno = pos-1;
				break;
			}
		}
		//判断读取文件的打开类型
		if (fileno > 0)
		{
			struct dfs_fd * fd = fd_get(fileno);
			if (fd != RT_NULL)
			{
				fd_put(fd);
				if (fd->flags & O_NONBLOCK)
				{
					//读取不到等待下一次继续
					errno = EAGAIN;
					return 0;
				}
			}
		}
		//等待事件信号
		if (device->ios.c_cc[VMIN] == 0)
		{
			if (device->ios.c_cc[VTIME] == 0)
				return 0;
			rt_event_recv(&rx_sem,set,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,device->ios.c_cc[VTIME]*100,0);
			ret = rt_device_read(device->device, 0, buffer, size);
		}
	}
	if (device->ios.c_cc[VMIN])
	{
		//需要继续等待
		while (ret < device->ios.c_cc[VMIN])
		{
			if (device->ios.c_cc[VTIME] == 0 || ret == 0)
			{
				rt_event_recv(&rx_sem,set,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,0);
				int read = rt_device_read(device->device, 0, buffer+ret, size-ret); if (read < 0) read = 0;
				ret += read;
			}
			else
			{
				rt_event_recv(&rx_sem,set,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,device->ios.c_cc[VTIME]*100,0);
				int read = rt_device_read(device->device, 0, buffer+ret, size-ret); if (read < 0) read = 0;
				ret += read;
				break;
			}
		}
	}

	//其他串口不支持回显模式
	if (uart->irq != IRQ_DUART)
		return ret;

	if (ret>0 && (device->ios.c_lflag & ECHO) && !tty_rx_inxpz)
	{
		//回车换行的转换
		if (device->ios.c_iflag & (ICRNL|INLCR))
		{
			char read = '\r',write = '\n';
			if (device->ios.c_iflag & INLCR)
			{
				read = '\n';
				write = '\r';
			}
			size = ret;
			while (size--)
			{
				if (((char *)buffer)[size] == read)
					((char *)buffer)[size] = write;
			}
		}
		rt_device_write(device->device, 0, buffer, ret);
	}
	return ret;
}

static rt_size_t tty_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	struct tty_device* device;

	device = (struct tty_device*)dev;
	RT_ASSERT(device != RT_NULL);

	//回车换行的转换
	if (device->ios.c_oflag & ONLCR)
		device->device->open_flag |= RT_DEVICE_FLAG_STREAM;
	else
		device->device->open_flag &= ~RT_DEVICE_FLAG_STREAM;

	return rt_device_write(device->device, 0, buffer, size);
}

static rt_err_t tty_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	struct tty_device* device;

	device = (struct tty_device*)dev;
	RT_ASSERT(device != RT_NULL);
	RT_ASSERT(device->device != RT_NULL);

	switch(cmd)
	{
	case RT_DEVICE_CTRL_GETWS:
		rt_memcpy(args,&device->win,sizeof(device->win));
		return RT_EOK;
	case RT_DEVICE_CTRL_SETWS:
		rt_memcpy(&device->win,args,sizeof(device->win));
		return RT_EOK;
	case RT_DEVICE_CTRL_GETS:
		rt_memcpy(args,&device->ios,sizeof(device->ios));
		return RT_EOK;
	case RT_DEVICE_CTRL_SETS:
	case RT_DEVICE_CTRL_SETSW:
	case RT_DEVICE_CTRL_SETSF:
		rt_memcpy(&device->ios,args,sizeof(device->ios));
		return RT_EOK;
	case RT_DEVICE_CTRL_GPGRP:
		rt_memcpy(args,&device->tpid[0],sizeof(pid_t));
		if (device->tpid[0] == 0)
		{
			errno = ENOTTY;
			return -RT_ENOSYS;
		}
		return RT_EOK;
	case RT_DEVICE_CTRL_SPGRP:
		rt_memcpy(&device->tpid[0],args,sizeof(pid_t));
		return RT_EOK;
	case RT_DEVICE_CTRL_GSID:
		rt_memcpy(args,&device->tpid[1],sizeof(pid_t));
		if (device->tpid[1] == 0)
		{
			errno = ENOTTY;
			return -RT_ENOSYS;
		}
		return RT_EOK;
	case RT_DEVICE_CTRL_SSID:
		rt_memcpy(&device->tpid[1],args,sizeof(pid_t));
		return RT_EOK;
	case RT_DEVICE_CTRL_CHAR_SETFILE:
	{
		int i;
		PRESS_DEBUG_DEVFILE("devsetfile ");
		register rt_base_t temp = rt_hw_interrupt_disable();
		for (i=0; i<TTY_FILE_SIZE; i++)
		{
			if (device->file[i] == 0)
			{
				device->file[i] = *((int *)args);
				PRESS_DEBUG_DEVFILE("%2d/%2d ",i,device->file[i]);
				break;
			}
			PRESS_DEBUG_DEVFILE("%2d/%2d ",i,device->file[i]);
		}
		PRESS_DEBUG_DEVFILE("\n");
		RT_ASSERT(i != TTY_FILE_SIZE);
		rt_hw_interrupt_enable(temp);
		return RT_EOK;
	}
	case RT_DEVICE_CTRL_CHAR_GETFILE:
	{
		int i,find = 0;
		register rt_base_t temp = rt_hw_interrupt_disable();
		for (i=TTY_FILE_SIZE-1; i>=0; i--)
		{
			if (device->file[i] != 0)
			{
				*((int *)args) = device->file[i];
				find = 1;
				break;
			}
		}
		rt_hw_interrupt_enable(temp);
		return (find==0)?RT_ERROR:RT_EOK;
	}
	case RT_DEVICE_CTRL_CHAR_CHKFILE:
	{
		int i,find = 0;
		register rt_base_t temp = rt_hw_interrupt_disable();
		for (i=0; i<TTY_FILE_SIZE; i++)
		{
			if (device->file[i] == 0)
				break;
			if (device->file[i] == *((int *)args))
			{
				*((int *)args) = device->file[i]-1;
				find = 1;
				break;
			}
		}
		rt_hw_interrupt_enable(temp);
		return (find==0)?RT_ERROR:RT_EOK;
	}
	case RT_DEVICE_CTRL_CHAR_CLRFILE:
	{
		int i,j;
		PRESS_DEBUG_DEVFILE("devclrfile ");
		register rt_base_t temp = rt_hw_interrupt_disable();
		for (i=0; i<TTY_FILE_SIZE; i++)
		{
			if (device->file[i] == 0)
				break;
			if (device->file[i] == *((int *)args))
			{
				PRESS_DEBUG_DEVFILE("(%2d/%2d) ",i,device->file[i]);
				device->file[i] = 0;
				//保证数据的紧凑，不用遍历到最后节约时间
				for (j=TTY_FILE_SIZE-1; j>i; j--)
				{
					if (device->file[j] != 0)
					{
						device->file[i] = device->file[j];
						device->file[j] = 0;
						break;
					}
				}
			}
			if (device->file[i] == 0)
				break;
			PRESS_DEBUG_DEVFILE("%2d/%2d ",i,device->file[i]);
		}
		PRESS_DEBUG_DEVFILE("\n");
		rt_hw_interrupt_enable(temp);
		return RT_EOK;
	}
	}
	return rt_device_control(device->device, cmd, args);
}

static void rt_tty_init(rt_device_t device, struct tty_device* tty, const char* att_name)
{
	rt_memset(tty, 0, sizeof(struct tty_device));

	/* device initialization */
	tty->parent.type = RT_Device_Class_Char;
	/* set device interface */
	tty->parent.init 	= tty_init;
	tty->parent.open 	= tty_open;
	tty->parent.close   = tty_close;
	tty->parent.read 	= tty_read;
	tty->parent.write   = tty_write;
	tty->parent.control	= tty_control;
	tty->parent.user_data = RT_NULL;

	tty->device = device;
	rt_device_register(&tty->parent, att_name, RT_DEVICE_FLAG_RDWR);
}
#endif
