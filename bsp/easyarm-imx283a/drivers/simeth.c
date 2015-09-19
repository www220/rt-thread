/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
/*
 * STM32 Eth Driver for RT-Thread
 * Change Logs:
 * Date           Author       Notes
 * 2009-10-05     Bernard      eth interface driver for STM32F107 CL
 */
#include <stdint.h>
#include <rtthread.h>
#include <netif/ethernetif.h>
#include <netif/etharp.h>
#include <lwip/icmp.h>
#include "lwipopts.h"
#include "win32/pktdrv.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#ifdef ETH_DEBUG
#define STM32_ETH_PRINTF          rt_kprintf
#else
#define STM32_ETH_PRINTF(...)
#endif

#define MAX_ADDR_LEN 6
struct rt_stm32_eth
{
    /* inherit from ethernet device */
    struct eth_device parent;
    /* interface address info. */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];
    /* thread */
    HANDLE eth_thread;
    HANDLE eth_event;
    /* pcap info */
    void *packinfo;
    /* buf offset */
    int bufoffset;
    /* recv pbuf info */
    struct pbuf* recvbuf;
};
static struct rt_stm32_eth stm32_eth_device;
static DWORD WINAPI ThreadforETHRecv(LPVOID lpParam);

extern void RegisterSimulateInterrupt(rt_uint32_t IntIndex,rt_uint32_t (*IntHandler)(void));
extern void TriggerSimulateInterrupt(rt_uint32_t IntIndex);

/* initialize the interface */
static rt_err_t rt_stm32_eth_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t rt_stm32_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_stm32_eth_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t rt_stm32_eth_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t rt_stm32_eth_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t rt_stm32_eth_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch(cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if(args) rt_memcpy(args, stm32_eth_device.dev_addr, 6);
        else return -RT_ERROR;
        break;

    default :
        break;
    }

    return RT_EOK;
}

/* transmit packet. */
rt_err_t rt_stm32_eth_tx( rt_device_t dev, struct pbuf* p)
{
    struct pbuf* q;
    rt_uint32_t offset,buflen;

    buflen = 0;
    for (q = p; q != NULL; q = q->next)
        buflen += q->len;

    offset = 0;
    for (q = p; q != NULL; offset += q->len, q = q->next)
        packet_send(stm32_eth_device.packinfo, buflen, offset, q->payload, q->len);

    /* Return SUCCESS */
    return RT_EOK;
}

/* reception packet. */
extern volatile int eth_wtdog;
extern volatile int eth_linkstatus;
extern unsigned char NET_MAC[6];
int rt_stm32_pack_rx(void *arg, void *packet, int len)
{
    struct pbuf* p;
    rt_uint32_t offset = 0, framelength = 0;

    /* wtdog eth status */
    eth_wtdog = 0;
    eth_linkstatus = 1;

    if ((rt_memcmp(packet, NET_MAC, 6) != 0)
        && (((uint32_t *)packet)[0] != 0xffffffff))
        return 0;

    p = pbuf_alloc(PBUF_LINK, len, PBUF_RAM);
    if (p != RT_NULL)
    {
        struct pbuf* q;
        for (q = p; q != RT_NULL; q= q->next)
        {
            /* Copy the received frame into buffer from memory pointed by the current ETHERNET DMA Rx descriptor */
            memcpy(q->payload, (uint8_t *)(packet) + offset, q->len);
            offset += q->len;
        }
        stm32_eth_device.recvbuf = p;
    }
    return p != NULL;
}

struct pbuf *rt_stm32_eth_rx(rt_device_t dev)
{
    /* init p pointer */
    stm32_eth_device.recvbuf = NULL;
    update_adapter(stm32_eth_device.packinfo, &stm32_eth_device.bufoffset);
    return stm32_eth_device.recvbuf;
}

/* PHY: DP83848 */
static enum link_adapter_event phy_speed = LINKEVENT_UNCHANGED;
static void phy_monitor_thread_entry(void *parameter)
{
    enum link_adapter_event phy_speed_new = LINKEVENT_UNCHANGED;
    while(1)
    {
        phy_speed_new = link_adapter(stm32_eth_device.packinfo);
        if (phy_speed_new != phy_speed)
        {
            if (phy_speed_new == LINKEVENT_UP)
            {
                /* send link up. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_TRUE);
                /* wtdog eth status */
                eth_wtdog = 0;
                eth_linkstatus = 1;
            }
            else if (phy_speed_new == LINKEVENT_DOWN)
            {
                /* send link down. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_FALSE);
                /* wtdog eth status */
                eth_wtdog = 0;
                eth_linkstatus = 0;
            }
            phy_speed = phy_speed_new;
        }

        eth_device_ready(&(stm32_eth_device.parent));
        rt_thread_delay(RT_TICK_PER_SECOND);
    } /* while(1) */
}

static DWORD WINAPI ThreadforETHRecv(LPVOID lpParam)
{
    HANDLE hEvent = (HANDLE)lpParam;

    while (stm32_eth_device.eth_thread != NULL)
    {
        if (WaitForSingleObject(hEvent, INFINITE) == WAIT_OBJECT_0)
        {
            TriggerSimulateInterrupt(0x0a);
            Sleep(10);
        }
        /* close event */
        else
        {
            break;
        }
    }

    return 0;
}

static rt_int32_t eth_interrupt(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    eth_device_ready(&(stm32_eth_device.parent));

    /* leave interrupt */
    rt_interrupt_leave();
    return 0; 
}

void rt_hw_eth_init(void)
{
    enum link_adapter_event status = LINKEVENT_UNCHANGED;
    stm32_eth_device.bufoffset = 0;
    stm32_eth_device.recvbuf = NULL;
    stm32_eth_device.packinfo = init_adapter(-1,NET_MAC,rt_stm32_pack_rx,&stm32_eth_device,&status);
    if (stm32_eth_device.packinfo == NULL)
    {
        rt_kprintf("netif init failed!\n");
        return;
    }
    /* modify MAC Addr */
    NET_MAC[5] += 0x01;

    {
        DWORD ThreadID;
        stm32_eth_device.eth_event = get_adapter_readevent(stm32_eth_device.packinfo);
        stm32_eth_device.eth_thread = CreateThread(NULL, 
            0,
            ThreadforETHRecv,
            stm32_eth_device.eth_event,
            CREATE_SUSPENDED,
            &ThreadID);
        if (stm32_eth_device.eth_thread != NULL)
        {
            SetThreadPriority(stm32_eth_device.eth_thread, THREAD_PRIORITY_NORMAL);
            SetThreadPriorityBoost(stm32_eth_device.eth_thread, TRUE);
            SetThreadAffinityMask(stm32_eth_device.eth_thread,0x01);

            RegisterSimulateInterrupt(0x0a, eth_interrupt);
            ResumeThread(stm32_eth_device.eth_thread);
        }
    }

    /* OUI 00-80-E1 STMICROELECTRONICS. */
    stm32_eth_device.dev_addr[0] = 0x00;
    stm32_eth_device.dev_addr[1] = 0x80;
    stm32_eth_device.dev_addr[2] = 0xE1;
    /* generate MAC addr from 96bit unique ID (only for test). */
    stm32_eth_device.dev_addr[3] = 0x02;
    stm32_eth_device.dev_addr[4] = 0x03;
    stm32_eth_device.dev_addr[5] = 0x04;
    /* load or save net mac addr */
    if (NET_MAC[0] == 0 && NET_MAC[1] == 0 && NET_MAC[2] == 0)
    {
        NET_MAC[0] = stm32_eth_device.dev_addr[0];
        NET_MAC[1] = stm32_eth_device.dev_addr[1];
        NET_MAC[2] = stm32_eth_device.dev_addr[2];
        NET_MAC[3] = stm32_eth_device.dev_addr[3];
        NET_MAC[4] = stm32_eth_device.dev_addr[4];
        NET_MAC[5] = stm32_eth_device.dev_addr[5];        
    }
    else
    {
        stm32_eth_device.dev_addr[0] = NET_MAC[0];
        stm32_eth_device.dev_addr[1] = NET_MAC[1];
        stm32_eth_device.dev_addr[2] = NET_MAC[2];
        stm32_eth_device.dev_addr[3] = NET_MAC[3];
        stm32_eth_device.dev_addr[4] = NET_MAC[4];
        stm32_eth_device.dev_addr[5] = NET_MAC[5];
    }
    stm32_eth_device.parent.parent.init       = rt_stm32_eth_init;
    stm32_eth_device.parent.parent.open       = rt_stm32_eth_open;
    stm32_eth_device.parent.parent.close      = rt_stm32_eth_close;
    stm32_eth_device.parent.parent.read       = rt_stm32_eth_read;
    stm32_eth_device.parent.parent.write      = rt_stm32_eth_write;
    stm32_eth_device.parent.parent.control    = rt_stm32_eth_control;
    stm32_eth_device.parent.parent.user_data  = RT_NULL;

    stm32_eth_device.parent.eth_rx     = rt_stm32_eth_rx;
    stm32_eth_device.parent.eth_tx     = rt_stm32_eth_tx;

    /* register eth device */
    eth_device_init(&(stm32_eth_device.parent), "e0");

    /* start phy monitor */
    {
        rt_thread_t tid;
        tid = rt_thread_create("phy",
                               phy_monitor_thread_entry,
                               RT_NULL,
                               512,
                               RT_THREAD_PRIORITY_MAX - 2,
                               20);
        if (tid != RT_NULL)
            rt_thread_startup(tid);
    }
}
