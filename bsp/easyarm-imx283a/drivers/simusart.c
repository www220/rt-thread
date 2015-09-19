/*
 * File      : serial.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-02-05     Bernard      first version
 * 2009-10-25     Bernard      fix rt_serial_read bug when there is no data
 *                             in the buffer.
 * 2010-03-29     Bernard      cleanup code.
 */

#include <rthw.h>
#include <rtthread.h>
#include <windows.h>
#include "board.h"

#define UART_RX_BUFFER_SIZE 1024
static DWORD WINAPI ThreadforUsartRecv(LPVOID lpParam);

extern void RegisterSimulateInterrupt(rt_uint32_t IntIndex,rt_uint32_t (*IntHandler)(void));
extern void TriggerSimulateInterrupt(rt_uint32_t IntIndex);

struct stm32_serial_int_rx
{
    rt_uint8_t  rx_buffer[UART_RX_BUFFER_SIZE];
    rt_uint32_t read_index, save_index;
};

struct stm32_serial_device
{
    char port_name[20];
    HANDLE uart_device;
    HANDLE uart_thread;
    DCB uart_dcb;
    int interrupt;
    rt_uint32_t (*intfunc)(void);
    struct stm32_serial_int_rx* int_rx;
};

/* RT-Thread Device Interface */
static rt_err_t rt_serial_init (rt_device_t dev)
{
    struct stm32_serial_device* uart = (struct stm32_serial_device*) dev->user_data;

    if (!(dev->flag & RT_DEVICE_FLAG_ACTIVATED))
    {
        if (dev->flag & RT_DEVICE_FLAG_INT_RX)
        {
            rt_memset(uart->int_rx->rx_buffer, 0,
                sizeof(uart->int_rx->rx_buffer));
            uart->int_rx->read_index = 0;
            uart->int_rx->save_index = 0;
        }

        dev->flag |= RT_DEVICE_FLAG_ACTIVATED;
    }

    return RT_EOK;
}

static rt_err_t rt_serial_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct stm32_serial_device* uart = (struct stm32_serial_device*) dev->user_data;

    if (uart->uart_device == INVALID_HANDLE_VALUE)
    {
        uart->uart_device = CreateFile(uart->port_name, 
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            0);
    }
    if (uart->uart_thread == NULL)
    {
        DWORD ThreadID;
        uart->uart_thread = CreateThread(NULL, 
            0,
            ThreadforUsartRecv,
            uart,
            CREATE_SUSPENDED,
            &ThreadID);
        if (uart->uart_thread == NULL)
        {
            CloseHandle(uart->uart_device);
            uart->uart_device = INVALID_HANDLE_VALUE;
        }
        else
        {
            SetThreadPriority(uart->uart_thread, THREAD_PRIORITY_NORMAL);
            SetThreadPriorityBoost(uart->uart_thread, TRUE);
            SetThreadAffinityMask(uart->uart_thread,0x01);

            RegisterSimulateInterrupt(uart->interrupt, uart->intfunc);
            ResumeThread(uart->uart_thread);
        }
    }
    if (uart->uart_device != INVALID_HANDLE_VALUE)
    {
        DCB dcb;
        COMMTIMEOUTS cto;
        GetCommState(uart->uart_device, &dcb);
        GetCommTimeouts(uart->uart_device, &cto);

        dcb.fBinary = uart->uart_dcb.fBinary;
        dcb.BaudRate= uart->uart_dcb.BaudRate;
        dcb.Parity = uart->uart_dcb.Parity;
        dcb.ByteSize = uart->uart_dcb.ByteSize;
        dcb.StopBits= uart->uart_dcb.StopBits;
        SetCommState(uart->uart_device, &dcb);

        cto.ReadIntervalTimeout = MAXDWORD;
        cto.ReadTotalTimeoutMultiplier = 0;
        cto.ReadTotalTimeoutConstant = 0;
        cto.WriteTotalTimeoutMultiplier = 10;
        cto.WriteTotalTimeoutConstant = 0;
        SetCommTimeouts(uart->uart_device, &cto);
    }
    return (uart->uart_device != INVALID_HANDLE_VALUE)?RT_EOK:RT_ERROR;
}

static rt_err_t rt_serial_close(rt_device_t dev)
{
    struct stm32_serial_device* uart = (struct stm32_serial_device*) dev->user_data;

    if (uart->uart_device != INVALID_HANDLE_VALUE)
    {
        CloseHandle(uart->uart_device);
        uart->uart_device = INVALID_HANDLE_VALUE;
    }
    if (uart->uart_thread != NULL)
    {
        WaitForSingleObject(uart->uart_thread, INFINITE);
        CloseHandle(uart->uart_thread);
        uart->uart_thread = NULL;
    }
    return RT_EOK;
}

static rt_size_t rt_serial_read (rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    rt_uint8_t* ptr;
    rt_err_t err_code;
    struct stm32_serial_device* uart;

    ptr = buffer;
    err_code = RT_EOK;
    uart = (struct stm32_serial_device*)dev->user_data;

    if (dev->flag & RT_DEVICE_FLAG_INT_RX)
    {
        /* interrupt mode Rx */
        while (size)
        {
            rt_base_t level;

            /* disable interrupt */
            level = rt_hw_interrupt_disable();

            if (uart->int_rx->read_index != uart->int_rx->save_index)
            {
                /* read a character */
                *ptr++ = uart->int_rx->rx_buffer[uart->int_rx->read_index];
                size--;

                /* move to next position */
                uart->int_rx->read_index ++;
                if (uart->int_rx->read_index >= UART_RX_BUFFER_SIZE)
                    uart->int_rx->read_index = 0;
            }
            else
            {
                /* set error code */
                err_code = -RT_EEMPTY;

                /* enable interrupt */
                rt_hw_interrupt_enable(level);
                break;
            }

            /* enable interrupt */
            rt_hw_interrupt_enable(level);
        }
    }

    /* set error code */
    rt_set_errno(err_code);
    return (rt_uint32_t)ptr - (rt_uint32_t)buffer;
}

static rt_size_t rt_serial_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    rt_uint8_t* ptr;
    rt_err_t err_code;
    rt_uint32_t code;
    struct stm32_serial_device* uart;

    err_code = RT_EOK;
    ptr = (rt_uint8_t*)buffer;
    uart = (struct stm32_serial_device*)dev->user_data;

    if (dev->flag & RT_DEVICE_FLAG_INT_TX)
    {
        /* interrupt mode Tx, does not support */
        RT_ASSERT(0);
    }
    else
    {
        /* polling mode */
        if (dev->open_flag & RT_DEVICE_FLAG_STREAM)
        {
            /* stream mode */
            while (size)
            {
                if (*ptr == '\n')
                    WriteFile(uart->uart_device, "\r", 1, &code, 0);

                WriteFile(uart->uart_device, ptr, 1, &code, 0);

                ++ptr; --size;
            }
        }
        else
        {
            /* write data directly */
            while (size)
            {
                WriteFile(uart->uart_device, ptr, 1, &code, 0);

                ++ptr; --size;
            }
        }
    }

    /* set error code */
    rt_set_errno(err_code);

    return (rt_uint32_t)ptr - (rt_uint32_t)buffer;
}

static rt_err_t rt_serial_control (rt_device_t dev, rt_uint8_t cmd, void *args)
{
    struct stm32_serial_device* uart;

    RT_ASSERT(dev != RT_NULL);

    uart = (struct stm32_serial_device*)dev->user_data;
    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        /* suspend device */
        dev->flag |= RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_RESUME:
        /* resume device */
        dev->flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        break;
    }

    return RT_EOK;
}

rt_err_t rt_hw_serial_register(rt_device_t device, const char* name, rt_uint32_t flag, struct stm32_serial_device *serial)
{
    RT_ASSERT(device != RT_NULL);

    if ((flag & RT_DEVICE_FLAG_DMA_RX) ||
        (flag & RT_DEVICE_FLAG_INT_TX))
    {
        RT_ASSERT(0);
    }

    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->init        = rt_serial_init;
    device->open        = rt_serial_open;
    device->close       = rt_serial_close;
    device->read        = rt_serial_read;
    device->write       = rt_serial_write;
    device->control     = rt_serial_control;
    device->user_data   = serial;

    /* register a character device */
    return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR | flag);
}

static DWORD WINAPI ThreadforUsartRecv(LPVOID lpParam)
{
    struct stm32_serial_device* uart = (struct stm32_serial_device* )lpParam;

    while (uart->uart_device != INVALID_HANDLE_VALUE)
    {
        DWORD dwEvent;
        COMSTAT comStat;
        if (ClearCommError(uart->uart_device, &dwEvent, &comStat))
        {
            if (comStat.cbInQue > 0)
                TriggerSimulateInterrupt(uart->interrupt);
        }
        Sleep(10);
    }

    return 0;
}

/* ISR for serial interrupt */
static void rt_hw_serial_isr(rt_device_t device)
{
    struct stm32_serial_device* uart = (struct stm32_serial_device* )device->user_data;
    rt_uint8_t rxbuf;
    rt_uint32_t code;

    /* save on rx buffer */
    while (ReadFile(uart->uart_device, &rxbuf, 1, &code, NULL) && code)
    {
        rt_base_t level;

        /* disable interrupt */
        level = rt_hw_interrupt_disable();

        /* save character */
        uart->int_rx->rx_buffer[uart->int_rx->save_index] = rxbuf;
        uart->int_rx->save_index ++;
        if (uart->int_rx->save_index >= UART_RX_BUFFER_SIZE)
            uart->int_rx->save_index = 0;

        /* if the next position is read index, discard this 'read char' */
        if (uart->int_rx->save_index == uart->int_rx->read_index)
        {
            uart->int_rx->read_index ++;
            if (uart->int_rx->read_index >= UART_RX_BUFFER_SIZE)
                uart->int_rx->read_index = 0;
        }

        /* enable interrupt */
        rt_hw_interrupt_enable(level);
    }

    /* invoke callback */
    if (device->rx_indicate != RT_NULL)
    {
        rt_size_t rx_length;

        /* get rx length */
        rx_length = uart->int_rx->read_index > uart->int_rx->save_index ?
            UART_RX_BUFFER_SIZE - uart->int_rx->read_index + uart->int_rx->save_index :
        uart->int_rx->save_index - uart->int_rx->read_index;

        device->rx_indicate(device, rx_length);
    }
}


#ifdef RT_USING_UART1
static struct rt_device uart1_device;
static rt_int32_t uart1_interrupt(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&uart1_device);

    /* leave interrupt */
    rt_interrupt_leave();
    return 0; 
}
static struct stm32_serial_int_rx uart1_int_rx;
static struct stm32_serial_device uart1 =
{
    "COM1",
    INVALID_HANDLE_VALUE,
    NULL,
    {0},
    0x02,
    uart1_interrupt,
    &uart1_int_rx
};
#endif

void rt_hw_usart_init()
{
    /* uart init */
#ifdef RT_USING_UART1
    uart1.uart_dcb.fBinary = 1;
    uart1.uart_dcb.BaudRate= 115200;
    uart1.uart_dcb.Parity = 0;
    uart1.uart_dcb.ByteSize = 8;
    uart1.uart_dcb.StopBits= 0;

    /* register uart1 */
    rt_hw_serial_register(&uart1_device, "uart1",
        RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_INT_RX,
        &uart1);
#endif
}
