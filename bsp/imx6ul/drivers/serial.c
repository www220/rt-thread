/*
 *  serial.c UART driver
 *
 * COPYRIGHT (C) 2013, Shanghai Real-Thread Technology Co., Ltd
 *
 *  This file is part of RT-Thread (http://www.rt-thread.org)
 *  Maintainer: bernard.xiong <bernard.xiong at gmail.com>
 *
 *  All rights reserved.
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
 * 2013-03-30     Bernard      the first verion
 */

#include <rthw.h>
#include <registers/regsuart.h>
#include <imx_uart.h>
#include <board.h>
#include <rtdevice.h>

#undef ALIGN
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/imx-common/iomux-v3.h>

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_UART2_TX_DATA__UART2_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART2_RX_DATA__UART2_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart3_pads[] = {
	MX6_PAD_UART3_TX_DATA__UART3_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART3_RX_DATA__UART3_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_UART4_TX_DATA__UART4_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART4_RX_DATA__UART4_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart5_pads[] = {
	MX6_PAD_UART5_TX_DATA__UART5_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART5_RX_DATA__UART5_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart7_pads[] = {
	MX6_PAD_LCD_DATA16__UART7_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_LCD_DATA17__UART7_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart8_pads[] = {
	MX6_PAD_LCD_DATA20__UART8_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_LCD_DATA21__UART8_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

struct hw_uart_device
{
    uint32_t instance;
    char *name;
    int irqno;
};

static void rt_hw_uart_isr(int irqno, void *param)
{
    struct rt_serial_device *serial = (struct rt_serial_device *)param;

    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
}

static rt_err_t uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct hw_uart_device *uart = (struct hw_uart_device *)serial->parent.user_data;
    uint32_t baudrate;
    uint8_t parity, stopbits, datasize, flowcontrol;

    baudrate = cfg->baud_rate;
    switch (cfg->data_bits)
    {
    case DATA_BITS_8:
        datasize = EIGHTBITS;
        break;
    case DATA_BITS_7:
        datasize = SEVENBITS;
        break;
    default:
        datasize = EIGHTBITS;
        break;
    }
    switch (cfg->stop_bits)
    {
    case STOP_BITS_1:
        stopbits = STOPBITS_ONE;
        break;
    case STOP_BITS_2:
        stopbits = STOPBITS_TWO;
        break;
    default:
        stopbits = STOPBITS_ONE;
        break;
    }
    parity = PARITY_NONE;
    flowcontrol = FLOWCTRL_OFF;

    /* Initialize UART */
    uart_init(uart->instance, baudrate, parity, stopbits, datasize, flowcontrol);
    rt_hw_interrupt_install(uart->irqno, rt_hw_uart_isr, serial, uart->name);
    rt_hw_interrupt_mask(uart->irqno);

    /* Set the IRQ mode for the Rx FIFO */
    uart_set_FIFO_mode(uart->instance, RX_FIFO, 1, UIRQ_MODE);
    return RT_EOK;
}

static rt_err_t uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct hw_uart_device *uart = (struct hw_uart_device *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        rt_hw_interrupt_mask(uart->irqno);
        break;

    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        rt_hw_interrupt_umask(uart->irqno);
        break;
    }

    return RT_EOK;
}

static int uart_putc(struct rt_serial_device *serial, char c)
{
    struct hw_uart_device *uart = (struct hw_uart_device *)serial->parent.user_data;

    uart_putchar(uart->instance, (uint8_t*)&c);
    return 1;
}

static int uart_getc(struct rt_serial_device *serial)
{
    struct hw_uart_device *uart = (struct hw_uart_device *)serial->parent.user_data;

    int ch = uart_getchar(uart->instance);
    if (ch == NONE_CHAR) ch = -1;

    return ch;
}

static const struct rt_uart_ops _uart_ops =
{
    uart_configure,
    uart_control,
    uart_putc,
    uart_getc,
};

#ifdef RT_USING_UART1
/* UART1 device driver structure */
static struct hw_uart_device _uart1_device =
{
    HW_UART1,
    "uart1",
    IMX_INT_UART1
};
static struct rt_serial_device _serial1;
#endif
#ifdef RT_USING_UART2
/* UART2 device driver structure */
static struct hw_uart_device _uart2_device =
{
    HW_UART2,
    "uart2",
    IMX_INT_UART2
};
static struct rt_serial_device _serial2;
#endif

void uart_iomux_config(int instance) {}
int rt_hw_uart_init(void)
{
    struct hw_uart_device *uart;
    struct serial_configure config;

    config.baud_rate = BAUD_RATE_115200;
    config.bit_order = BIT_ORDER_LSB;
    config.data_bits = DATA_BITS_8;
    config.parity    = PARITY_NONE;
    config.stop_bits = STOP_BITS_1;
    config.invert    = NRZ_NORMAL;
    config.bufsz     = RT_SERIAL_RB_BUFSZ;

#ifdef RT_USING_UART1
    uart = &_uart1_device;
    _serial1.ops = &_uart_ops;
    _serial1.config = config;

	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
    rt_hw_interrupt_mask(uart->irqno);

    /* register UART1 device */
    rt_hw_serial_register(&_serial1, "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
#endif
#ifdef RT_USING_UART2
    uart = &_uart2_device;
    _serial2.ops = &_uart_ops;
    _serial2.config = config;

    imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart2_pads));
    rt_hw_interrupt_mask(uart->irqno);

    /* register UART1 device */
    rt_hw_serial_register(&_serial2, "uart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
#endif

    return 0;
}
INIT_BOARD_EXPORT(rt_hw_uart_init);