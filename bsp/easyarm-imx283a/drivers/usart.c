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
#include "board.h"

typedef struct uartport
{
	volatile rt_uint32_t CR;
	volatile rt_uint32_t MR;
	volatile rt_uint32_t IER;
	volatile rt_uint32_t IDR;
	volatile rt_uint32_t IMR;
	volatile rt_uint32_t CSR;
	volatile rt_uint32_t RHR;
	volatile rt_uint32_t THR;
	volatile rt_uint32_t BRGR;
	volatile rt_uint32_t RTOR;
	volatile rt_uint32_t TTGR;
	volatile rt_uint32_t reserved0[5];
	volatile rt_uint32_t FIDI;
	volatile rt_uint32_t NER;
	volatile rt_uint32_t reserved1;
	volatile rt_uint32_t IFR;
	volatile rt_uint32_t reserved2[44];
	volatile rt_uint32_t RPR;
	volatile rt_uint32_t RCR;
	volatile rt_uint32_t TPR;
	volatile rt_uint32_t TCR;
	volatile rt_uint32_t RNPR;
	volatile rt_uint32_t RNCR;
	volatile rt_uint32_t TNPR;
	volatile rt_uint32_t TNCR;
	volatile rt_uint32_t PTCR;
	volatile rt_uint32_t PTSR;
}uartport;

#define DBGU	((struct uartport *)1111)

struct at91_uart {
	uartport *port;
	int irq;
};
/**
 * This function will handle serial
 */
void rt_at91_usart_handler(int vector, void *param)
{
    register rt_uint32_t ir = REG_RD(REGS_UARTDBG_BASE, HW_UARTDBGRIS);
    if (ir & BM_UARTDBGRIS_RXRIS)
    {
        rt_device_t dev = (rt_device_t)param;
        rt_hw_serial_isr((struct rt_serial_device *)dev, RT_SERIAL_EVENT_RX_IND);
        REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGICR, BM_UARTDBGICR_RXIC);
    }
}

/**
* UART device in RT-Thread
*/
static rt_err_t at91_usart_configure(struct rt_serial_device *serial,
                                struct serial_configure *cfg)
{
    return RT_EOK;
}

static rt_err_t at91_usart_control(struct rt_serial_device *serial,
                              int cmd, void *arg)
{
    return RT_EOK;
}

static int at91_usart_putc(struct rt_serial_device *serial, char c)
{
	/* Wait for room in TX FIFO */
	while (REG_RD(REGS_UARTDBG_BASE, HW_UARTDBGFR) & BM_UARTDBGFR_TXFF)
		;

	/* Write the data byte */
	REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGDR, c);
    return 1;
}

static int at91_usart_getc(struct rt_serial_device *serial)
{
    int result = -1;

	/* Read data byte */
	if ((REG_RD(REGS_UARTDBG_BASE, HW_UARTDBGFR) & BM_UARTDBGFR_RXFE) == 0)
        result = REG_RD(REGS_UARTDBG_BASE, HW_UARTDBGDR) & 0xff;

    return result;
}

static const struct rt_uart_ops at91_usart_ops =
{
    at91_usart_configure,
    at91_usart_control,
    at91_usart_putc,
    at91_usart_getc,
};

#if defined(RT_USING_DBGU)
static struct rt_serial_device serial_dbgu;
struct at91_uart dbgu = {
	DBGU,
	1
};

#endif

/**
 * This function will handle init uart
 */
void rt_hw_usart_init(void)
{
#if defined(RT_USING_DBGU)
	serial_dbgu.ops = &at91_usart_ops;
	serial_dbgu.config.baud_rate = BAUD_RATE_115200;
    serial_dbgu.config.bit_order = BIT_ORDER_LSB;
    serial_dbgu.config.data_bits = DATA_BITS_8;
    serial_dbgu.config.parity = PARITY_NONE;
    serial_dbgu.config.stop_bits = STOP_BITS_1;
    serial_dbgu.config.invert = NRZ_NORMAL;
	serial_dbgu.config.bufsz = RT_SERIAL_RB_BUFSZ;

    /* register vcom device */
    rt_hw_serial_register(&serial_dbgu, "dbgu", RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &dbgu);
    
    REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGCR, BM_UARTDBGCR_TXE | BM_UARTDBGCR_RXE | BM_UARTDBGCR_UARTEN);
	REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGLCR_H, BM_UARTDBGLCR_H_WLEN);
    REG_WR(REGS_UARTDBG_BASE, HW_UARTDBGIMSC, BM_UARTDBGIMSC_RXIM);
    
    /* install interrupt handler */
    rt_hw_interrupt_install(IRQ_DUART, rt_at91_usart_handler, &serial_dbgu, "DbgU");
    rt_hw_interrupt_umask(IRQ_DUART);
#endif
}
