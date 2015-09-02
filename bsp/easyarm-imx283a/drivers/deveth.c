#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "fec.h"

#include <netif/ethernetif.h>
#include <netif/ethernetif.h>
#include <netif/etharp.h>
#include <lwip/icmp.h>
#include "lwipopts.h"

/* debug option */
#define ETH_DEBUG
//#define ETH_RX_DUMP
//#define ETH_TX_DUMP

#ifdef ETH_DEBUG
#define STM32_ETH_PRINTF          rt_kprintf
#else
#define STM32_ETH_PRINTF(...)
#endif

struct fec_info_s fec_info[] = {
	{
	 0,			/* index */
	 CONFIG_FEC0_IOBASE,	/* io base */
	 0,			/* RX BD */
	 0,			/* TX BD */
	 0,			/* rx Index */
	 0,			/* tx Index */
	 0,			/* tx buffer */
	 { 0 },		/* rx buffer */
	 0,			/* initialized flag */
	 },
};

#define MAX_ADDR_LEN 6
struct rt_stm32_eth
{
	/* inherit from ethernet device */
	struct eth_device parent;

	/* interface address info. */
	rt_uint8_t  dev_addr[MAX_ADDR_LEN];			/* hw address	*/
    struct fec_info_s *priv;

	uint32_t    ETH_Speed; /*!< @ref ETH_Speed */
	uint32_t    ETH_Mode;  /*!< @ref ETH_Duplex_Mode */
};

static struct rt_stm32_eth stm32_eth_device;
static struct rt_semaphore tx_wait;
static rt_bool_t tx_is_waiting = RT_FALSE;

/* Ethernet Transmit and Receive Buffers */
#define DBUF_LENGTH		1520
#define TX_BUF_CNT		2
#define PKT_MAXBUF_SIZE		1518
#define PKT_MINBUF_SIZE		64
#define PKT_MAXBLR_SIZE		1520
#define LAST_PKTBUFSRX		(PKTBUFSRX - 1)
#define BD_ENET_RX_W_E		(BD_ENET_RX_WRAP | BD_ENET_RX_EMPTY)
#define BD_ENET_TX_RDY_LST	(BD_ENET_TX_READY | BD_ENET_TX_LAST)

/* the defins of MII operation */
#define FEC_MII_ST      0x40000000
#define FEC_MII_OP_OFF  28
#define FEC_MII_OP_MASK 0x03
#define FEC_MII_OP_RD   0x02
#define FEC_MII_OP_WR   0x01
#define FEC_MII_PA_OFF  23
#define FEC_MII_PA_MASK 0xFF
#define FEC_MII_RA_OFF  18
#define FEC_MII_RA_MASK 0xFF
#define FEC_MII_TA      0x00020000
#define FEC_MII_DATA_OFF 0
#define FEC_MII_DATA_MASK 0x0000FFFF

#define FEC_MII_FRAME   (FEC_MII_ST | FEC_MII_TA)
#define FEC_MII_OP(x)   (((x) & FEC_MII_OP_MASK) << FEC_MII_OP_OFF)
#define FEC_MII_PA(pa)  (((pa) & FEC_MII_PA_MASK) << FEC_MII_PA_OFF)
#define FEC_MII_RA(ra)  (((ra) & FEC_MII_RA_MASK) << FEC_MII_RA_OFF)
#define FEC_MII_SET_DATA(v) (((v) & FEC_MII_DATA_MASK) << FEC_MII_DATA_OFF)
#define FEC_MII_GET_DATA(v) (((v) >> FEC_MII_DATA_OFF) & FEC_MII_DATA_MASK)
#define FEC_MII_READ(pa, ra) ((FEC_MII_FRAME | FEC_MII_OP(FEC_MII_OP_RD)) |\
					FEC_MII_PA(pa) | FEC_MII_RA(ra))
#define FEC_MII_WRITE(pa, ra, v) (FEC_MII_FRAME | FEC_MII_OP(FEC_MII_OP_WR)|\
			FEC_MII_PA(pa) | FEC_MII_RA(ra) | FEC_MII_SET_DATA(v))

#define FEC_MII_TIMEOUT		5000000
#define FEC_MII_TICK         	2

static inline int __fec_mii_read(volatile fec_t *fecp, unsigned char addr,
				 unsigned char reg, unsigned short *value)
{
	int waiting = FEC_MII_TIMEOUT;
	if (fecp->eir & FEC_EIR_MII)
		fecp->eir = FEC_EIR_MII;

	fecp->mmfr = FEC_MII_READ(addr, reg);
	while (1) {
		if (fecp->eir & FEC_EIR_MII) {
			fecp->eir = FEC_EIR_MII;
			break;
		}
		if ((waiting--) <= 0)
			return -1;
		udelay(FEC_MII_TICK);
	}
	*value = FEC_MII_GET_DATA(fecp->mmfr);
	return 0;
}

static inline int __fec_mii_write(volatile fec_t *fecp, unsigned char addr,
				  unsigned char reg, unsigned short value)
{
	int waiting = FEC_MII_TIMEOUT;
	if (fecp->eir & FEC_EIR_MII)
		fecp->eir = FEC_EIR_MII;

	fecp->mmfr = FEC_MII_WRITE(addr, reg, value);
	while (1) {
		if (fecp->eir & FEC_EIR_MII) {
			fecp->eir = FEC_EIR_MII;
			break;
		}
		if ((waiting--) <= 0)
			return -1;
		udelay(FEC_MII_TICK);
	}
	return 0;
}

extern u32 mx28_get_hclk(void);
static void mxc_fec_mii_init(volatile fec_t *fecp)
{
	u32 clk = mx28_get_hclk() * 1000000;
	fecp->mscr = (fecp->mscr & (~0x7E)) | (((clk + 499999) / 5000000) << 1);
}

static void fec_reset(struct rt_stm32_eth *dev)
{
	struct fec_info_s *info = dev->priv;
	volatile fec_t *fecp = (fec_t *)(info->iobase);
	int i;

	fecp->ecr = FEC_ECR_RESET;
	for (i = 0; (fecp->ecr & FEC_ECR_RESET) && (i < FEC_RESET_DELAY); ++i)
		udelay(1);

	if (i == FEC_RESET_DELAY)
		printf("FEC_RESET_DELAY timeout\n");
}

/* initialize the interface */
static rt_err_t rt_stm32_eth_init(rt_device_t dev)
{
    struct rt_stm32_eth * stm32_eth = (struct rt_stm32_eth *)dev;
	struct fec_info_s *info = stm32_eth->priv;
	volatile fec_t *fecp = (fec_t *) (info->iobase);
	int i;
	u8 *ea = NULL;

	fec_reset(stm32_eth);

	/* We use strictly polling mode only */
	fecp->eimr = 0;

	/* Clear any pending interrupt */
	fecp->eir = 0xffffffff;

	/* Set station address   */
	ea = stm32_eth->dev_addr;
	fecp->palr = (ea[0] << 24) | (ea[1] << 16) | (ea[2] << 8) | (ea[3]);
	fecp->paur = (ea[4] << 24) | (ea[5] << 16);

	/* Clear unicast address hash table */
	fecp->iaur = 0;
	fecp->ialr = 0;

	/* Clear multicast address hash table */
	fecp->gaur = 0;
	fecp->galr = 0;

	/* Set maximum receive buffer size. */
	fecp->emrbr = PKT_MAXBLR_SIZE;

	/*
	 * Setup Buffers and Buffer Desriptors
	 */
	info->rxIdx = 0;
	info->txIdx = 0;

	/*
	 * Setup Receiver Buffer Descriptors (13.14.24.18)
	 * Settings:
	 *     Empty, Wrap
	 */
	for (i = 0; i < PKTBUFSRX; i++) {
		info->rxbd[i].cbd_sc = BD_ENET_RX_EMPTY;
		info->rxbd[i].cbd_datlen = 0;	/* Reset */
#ifdef CONFIG_ARCH_MMU
		info->rxbd[i].cbd_bufaddr =
			(uint)iomem_to_phys((ulong)info->rxbuf[i]);
#else
		info->rxbd[i].cbd_bufaddr = 
			(uint)info->rxbuf[i];
#endif
	}
	info->rxbd[PKTBUFSRX - 1].cbd_sc |= BD_ENET_RX_WRAP;

	/*
	 * Setup Ethernet Transmitter Buffer Descriptors (13.14.24.19)
	 * Settings:
	 *    Last, Tx CRC
	 */
	for (i = 0; i < TX_BUF_CNT; i++) {
		info->txbd[i].cbd_sc = BD_ENET_TX_LAST | BD_ENET_TX_TC;
		info->txbd[i].cbd_datlen = 0;	/* Reset */
#ifdef CONFIG_ARCH_MMU
		info->txbd[i].cbd_bufaddr =
			(uint)iomem_to_phys((ulong)&info->txbuf[0]);
#else
		info->txbd[i].cbd_bufaddr = (uint)&info->txbuf[0];
#endif
	}
	info->txbd[TX_BUF_CNT - 1].cbd_sc |= BD_ENET_TX_WRAP;

	/* Set receive and transmit descriptor base */
#ifdef CONFIG_ARCH_MMU
	fecp->erdsr = (uint)iomem_to_phys((ulong)&info->rxbd[0]);
	fecp->etdsr = (uint)iomem_to_phys((ulong)&info->txbd[0]);
#else
	fecp->erdsr = (uint)(&info->rxbd[0]);
	fecp->etdsr = (uint)(&info->txbd[0]);
#endif

	/* Now enable the transmit and receive processing */
	fecp->ecr |= FEC_ECR_ETHER_EN;

	/* And last, try to fill Rx Buffer Descriptors */
	fecp->rdar = 0x01000000;	/* Descriptor polling active    */
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

/* ethernet device interface */
/* transmit packet. */
rt_err_t rt_stm32_eth_tx( rt_device_t dev, struct pbuf* p)
{
    struct rt_stm32_eth * stm32_eth = (struct rt_stm32_eth *)dev;
	struct fec_info_s *info = stm32_eth->priv;
	volatile fec_t *fecp = (fec_t *) (info->iobase);
	int j, rc;
    void *packet;
    int length;
    
    packet = p->payload;
    length = p->len;
    
	j = 0;
	while ((info->txbd[info->txIdx].cbd_sc & BD_ENET_TX_READY) &&
	       (j < FEC_MAX_TIMEOUT)) {
		udelay(FEC_TIMEOUT_TICKET);
		j++;
	}
	if (j >= FEC_MAX_TIMEOUT)
		printf("TX not ready\n");

#ifdef CONFIG_MX28
	swap_packet((void *)packet, length);
#endif

#ifdef CONFIG_ARCH_MMU
	memcpy(ioremap_nocache((ulong)info->txbd[info->txIdx].cbd_bufaddr,
			length),
			(void *)packet, length);
#else
	info->txbd[info->txIdx].cbd_bufaddr = (uint)packet;
#endif
	info->txbd[info->txIdx].cbd_datlen = length;
	info->txbd[info->txIdx].cbd_sc =
	    (info->txbd[info->txIdx].cbd_sc & BD_ENET_TX_WRAP) |
	    BD_ENET_TX_TC | BD_ENET_TX_RDY_LST;

	/* Activate transmit Buffer Descriptor polling */
	fecp->tdar = 0x01000000;	/* Descriptor polling active    */

	/* FEC fix for MCF5275, FEC unable to initial transmit data packet.
	 * A nop will ensure the descriptor polling active completed.
	 */
	__asm__("nop");

#ifdef CONFIG_SYS_UNIFY_CACHE
	icache_invalid();
#endif
	j = 0;

	while ((info->txbd[info->txIdx].cbd_sc & BD_ENET_TX_READY) &&
	       (j < FEC_MAX_TIMEOUT)) {
		udelay(FEC_TIMEOUT_TICKET);
		j++;
	}
	if (j >= FEC_MAX_TIMEOUT)
		printf("TX timeout packet at %p\n", packet);

#ifdef ET_DEBUG
	printf("%s[%d] %s: cycles: %d    status: %x  retry cnt: %d\n",
	       __FILE__, __LINE__, __func__, j,
	       info->txbd[info->txIdx].cbd_sc,
	       (info->txbd[info->txIdx].cbd_sc & 0x003C) >> 2);
#endif
	/* return only status bits */
	rc = (info->txbd[info->txIdx].cbd_sc & BD_ENET_TX_STATS);
	info->txIdx = (info->txIdx + 1) % TX_BUF_CNT;

	return rc;
}

/* reception packet. */
struct pbuf *rt_stm32_eth_rx(rt_device_t dev)
{
    struct pbuf* p;

    /* init p pointer */
    p = RT_NULL;

    return p;
}


/** @defgroup PHY_Register_address
  * @{
  */
#define PHY_BCR                          0          /*!< Tranceiver Basic Control Register */
#define PHY_BSR                          1          /*!< Tranceiver Basic Status Register */

/** @defgroup PHY_basic_Control_register
  * @{
  */
#define PHY_Reset                       ((uint16_t)0x8000)      /*!< PHY Reset */
#define PHY_Loopback                    ((uint16_t)0x4000)      /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100)      /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000)      /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100)      /*!< Set the full-duplex mode at 10 Mb/s */
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000)      /*!< Set the half-duplex mode at 10 Mb/s */
#define PHY_AutoNegotiation             ((uint16_t)0x1000)      /*!< Enable auto-negotiation function */
#define PHY_Restart_AutoNegotiation     ((uint16_t)0x0200)      /*!< Restart auto-negotiation function */
#define PHY_Powerdown                   ((uint16_t)0x0800)      /*!< Select the power down mode */
#define PHY_Isolate                     ((uint16_t)0x0400)      /*!< Isolate PHY from MII */

/** @defgroup PHY_basic_status_register
  * @{
  */
#define PHY_AutoNego_Complete           ((uint16_t)0x0020)      /*!< Auto-Negotioation process completed */
#define PHY_Linked_Status               ((uint16_t)0x0004)      /*!< Valid link established */
#define PHY_Jabber_detection            ((uint16_t)0x0002)      /*!< Jabber condition detected */

/** @defgroup ETH_Speed
  * @{
  */
#define ETH_Speed_10M        ((uint32_t)0x00000000)
#define ETH_Speed_100M       ((uint32_t)0x00004000)
#define IS_ETH_SPEED(SPEED) (((SPEED) == ETH_Speed_10M) || \
                             ((SPEED) == ETH_Speed_100M))

/** @defgroup ETH_Duplex_Mode
  * @{
  */
#define ETH_Mode_FullDuplex       ((uint32_t)0x00000800)
#define ETH_Mode_HalfDuplex       ((uint32_t)0x00000000)
#define IS_ETH_DUPLEX_MODE(MODE) (((MODE) == ETH_Mode_FullDuplex) || \
                                  ((MODE) == ETH_Mode_HalfDuplex))

uint32_t ETH_WritePHYRegister(uint8_t PHYAddress, uint8_t PHYReg, uint16_t PHYValue)
{
	return __fec_mii_write((fec_t *)fec_info[0].iobase, PHYAddress, PHYReg, PHYValue);
}

uint16_t ETH_ReadPHYRegister(uint8_t PHYAddress, uint8_t PHYReg)
{
    uint16_t PHYValue = 0;
    __fec_mii_read((fec_t *)fec_info[0].iobase, PHYAddress, PHYReg, &PHYValue);
	return PHYValue;
}

#define RT_USING_DP83848
#ifdef RT_USING_LAN8720
/* PHY: LAN8720 */
static uint8_t phy_speed = 0;
#define PHY_LINK_MASK       (1<<0)
#define PHY_100M_MASK       (1<<1)
#define PHY_DUPLEX_MASK     (1<<2)
static void phy_monitor_thread_entry(void *parameter)
{
    uint8_t phy_addr = 0xFF;
    uint8_t phy_speed_new = 0;
	uint16_t phy_errcount = 0;

    /* phy search */
    {
        rt_uint32_t i;
        rt_uint16_t temp;

        for(i=0; i<=0x1F; i++)
        {
            temp = ETH_ReadPHYRegister(i, 0x02);

            if( temp != 0xFFFF )
            {
                phy_addr = i;
                break;
            }
        }
    } /* phy search */

    if(phy_addr == 0xFF)
    {
        STM32_ETH_PRINTF("phy not probe!\r\n");
        return;
    }
    else
    {
        STM32_ETH_PRINTF("found a phy, address:0x%02X\r\n", phy_addr);
    }

    /* RESET PHY */
    STM32_ETH_PRINTF("RESET PHY!\r\n");
    ETH_WritePHYRegister(phy_addr, PHY_BCR, PHY_Reset);
    rt_thread_delay(RT_TICK_PER_SECOND * 2);
    ETH_WritePHYRegister(phy_addr, PHY_BCR, PHY_AutoNegotiation);

    while(1)
    {
        uint16_t status  = ETH_ReadPHYRegister(phy_addr, PHY_BSR);
        STM32_ETH_PRINTF("LAN8720 status:0x%04X\r\n", status);

        phy_speed_new = 0;

        if(status & (PHY_AutoNego_Complete | PHY_Linked_Status))
        {
            uint16_t SR;

            SR = ETH_ReadPHYRegister(phy_addr, 31);
            STM32_ETH_PRINTF("LAN8720 REG 31:0x%04X\r\n", SR);

            SR = (SR >> 2) & 0x07; /* LAN8720, REG31[4:2], Speed Indication. */
            phy_speed_new = PHY_LINK_MASK;

            if((SR & 0x03) == 2)
            {
                phy_speed_new |= PHY_100M_MASK;
            }

            if(SR & 0x04)
            {
                phy_speed_new |= PHY_DUPLEX_MASK;
            }

            SR = ETH_ReadPHYRegister(phy_addr, 26);
            STM32_ETH_PRINTF("LAN8720 REG 26:0x%04X\r\n", SR);
            /* LAN8720, REG26[15:0], Error Counter. */
            if (SR-phy_errcount > 10)
                eth_wtdog++;
            phy_errcount = SR;
        }

        /* linkchange */
        if(phy_speed_new != phy_speed)
        {
            if(phy_speed_new & PHY_LINK_MASK)
            {
                STM32_ETH_PRINTF("link up ");

                if(phy_speed_new & PHY_100M_MASK)
                {
                    STM32_ETH_PRINTF("100Mbps");
                    stm32_eth_device.ETH_Speed = ETH_Speed_100M;
                }
                else
                {
                    stm32_eth_device.ETH_Speed = ETH_Speed_10M;
                    STM32_ETH_PRINTF("10Mbps");
                }

                if(phy_speed_new & PHY_DUPLEX_MASK)
                {
                    STM32_ETH_PRINTF(" full-duplex\r\n");
                    stm32_eth_device.ETH_Mode = ETH_Mode_FullDuplex;
                }
                else
                {
                    STM32_ETH_PRINTF(" half-duplex\r\n");
                    stm32_eth_device.ETH_Mode = ETH_Mode_HalfDuplex;
                }
                rt_stm32_eth_init((rt_device_t)&stm32_eth_device);

                /* send link up. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_TRUE);
				/* wtdog eth status */
				eth_wtdog = 0;
 				eth_linkstatus = 1;
           } /* link up. */
            else
            {
                STM32_ETH_PRINTF("link down\r\n");
                /* send link down. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_FALSE);
				/* wtdog eth status */
				eth_wtdog = 0;
				eth_linkstatus = 0;
            } /* link down. */

            phy_speed = phy_speed_new;
        } /* linkchange */

        rt_thread_delay(RT_TICK_PER_SECOND);
    } /* while(1) */
}
#endif
#ifdef RT_USING_DP83848
/* PHY: DP83848 */
static uint8_t phy_speed = 0;
#define PHY_LINK_MASK       (1<<0)
#define PHY_100M_MASK       (1<<1)
#define PHY_DUPLEX_MASK     (1<<2)
static void phy_monitor_thread_entry(void *parameter)
{
    uint8_t phy_addr = 0xFF;
    uint8_t phy_speed_new = 0;

    mxc_fec_mii_init((fec_t *)fec_info[0].iobase);

    /* phy search */
    {
        rt_uint32_t i;
        rt_uint16_t temp;

        for(i=0; i<=0x1F; i++)
        {
            temp = ETH_ReadPHYRegister(i, 0x02);

            if( temp != 0xFFFF )
            {
                phy_addr = i;
                break;
            }
        }
    } /* phy search */

    if(phy_addr == 0xFF)
    {
        STM32_ETH_PRINTF("phy not probe!\r\n");
        return;
    }
    else
    {
        STM32_ETH_PRINTF("found a phy, address:0x%02X\r\n", phy_addr);
    }

    /* RESET PHY */
    STM32_ETH_PRINTF("RESET PHY!\r\n");
    ETH_WritePHYRegister(phy_addr, PHY_BCR, PHY_Reset);
    rt_thread_delay(RT_TICK_PER_SECOND * 2);
    ETH_WritePHYRegister(phy_addr, PHY_BCR, PHY_AutoNegotiation);

    while(1)
    {
        uint16_t status  = ETH_ReadPHYRegister(phy_addr, PHY_BSR);
        STM32_ETH_PRINTF("DP83848 status:0x%04X\r\n", status);

        phy_speed_new = 0;

        if(status & (PHY_AutoNego_Complete | PHY_Linked_Status))
        {
            uint16_t SR;

            SR = ETH_ReadPHYRegister(phy_addr, 16);
            STM32_ETH_PRINTF("DP83848 REG 16:0x%04X\r\n", SR);

            SR = SR & 0x07; /* DP83848, REG16[2:0], Speed Indication. */
            phy_speed_new = PHY_LINK_MASK;

            if((SR & 0x02) == 0)
            {
                phy_speed_new |= PHY_100M_MASK;
            }

            if(SR & 0x04)
            {
                phy_speed_new |= PHY_DUPLEX_MASK;
            }

            SR = ETH_ReadPHYRegister(phy_addr, 21);
            STM32_ETH_PRINTF("DP83848 REG 21:0x%04X\r\n", SR);
            /* DP83848, REG21[7:0], Error Counter.[[COR]] */
            if (SR > 10)
                eth_wtdog++;
        }

        /* linkchange */
        if(phy_speed_new != phy_speed)
        {
            if(phy_speed_new & PHY_LINK_MASK)
            {
                STM32_ETH_PRINTF("link up ");

                if(phy_speed_new & PHY_100M_MASK)
                {
                    STM32_ETH_PRINTF("100Mbps");
                    stm32_eth_device.ETH_Speed = ETH_Speed_100M;
                }
                else
                {
                    stm32_eth_device.ETH_Speed = ETH_Speed_10M;
                    STM32_ETH_PRINTF("10Mbps");
                }

                if(phy_speed_new & PHY_DUPLEX_MASK)
                {
                    STM32_ETH_PRINTF(" full-duplex\r\n");
                    stm32_eth_device.ETH_Mode = ETH_Mode_FullDuplex;
                }
                else
                {
                    STM32_ETH_PRINTF(" half-duplex\r\n");
                    stm32_eth_device.ETH_Mode = ETH_Mode_HalfDuplex;
                }
                rt_stm32_eth_init((rt_device_t)&stm32_eth_device);

                /* send link up. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_TRUE);
				/* wtdog eth status */
				eth_wtdog = 0;
 				eth_linkstatus = 1;
           } /* link up. */
            else
            {
                STM32_ETH_PRINTF("link down\r\n");
                /* send link down. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_FALSE);
				/* wtdog eth status */
				eth_wtdog = 0;
				eth_linkstatus = 0;
            } /* link down. */

            phy_speed = phy_speed_new;
        } /* linkchange */

        rt_thread_delay(RT_TICK_PER_SECOND);
    } /* while(1) */
}
#endif

extern unsigned char NET_MAC[6];
static struct pin_desc enet_pins_desc[] = {
	{ PINID_ENET0_MDC, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_ENET0_MDIO, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_ENET0_RX_EN, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_ENET0_RXD0, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_ENET0_RXD1, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_ENET0_TX_EN, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_ENET0_TXD0, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_ENET0_TXD1, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_ENET_CLK, PIN_FUN1, PAD_8MA, PAD_3V3, 1 }
};
static struct pin_group enet_pins = {
	.pins		= enet_pins_desc,
	.nr_pins	= ARRAY_SIZE(enet_pins_desc)
};

void rt_hw_eth_init(void)
{
    int j;
    
	/* Turn on ENET clocks */
	REG_WR(REGS_CLKCTRL_BASE, HW_CLKCTRL_ENET,
		REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_ENET) &
		~(BM_CLKCTRL_ENET_SLEEP | BM_CLKCTRL_ENET_DISABLE));

	/* Set up ENET PLL for 50 MHz */
	REG_SET(REGS_CLKCTRL_BASE, HW_CLKCTRL_PLL2CTRL0,
		BM_CLKCTRL_PLL2CTRL0_POWER);    /* Power on ENET PLL */
	udelay(10);                             /* Wait 10 us */
	REG_CLR(REGS_CLKCTRL_BASE, HW_CLKCTRL_PLL2CTRL0,
		BM_CLKCTRL_PLL2CTRL0_CLKGATE);  /* Gate on ENET PLL */
	REG_WR(REGS_CLKCTRL_BASE, HW_CLKCTRL_ENET,
		REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_ENET) |
		BM_CLKCTRL_ENET_CLK_OUT_EN);    /* Enable pad output */

    pin_set_group(&enet_pins);
    
	pin_gpio_direction(PINID_LCD_D16, 1);
	pin_gpio_set(PINID_LCD_D16, 0);
	udelay(200);
	pin_gpio_set(PINID_LCD_D16, 1);

    fec_info[0].rxbd =
        (cbd_t *)memalign(CONFIG_SYS_CACHELINE_SIZE,
                    (PKTBUFSRX * sizeof(cbd_t)));
    fec_info[0].txbd =
        (cbd_t *)memalign(CONFIG_SYS_CACHELINE_SIZE,
                   (TX_BUF_CNT * sizeof(cbd_t)));
    fec_info[0].txbuf =
        (char *)memalign(CONFIG_SYS_CACHELINE_SIZE, DBUF_LENGTH);
    for (j = 0; j < PKTBUFSRX; ++j) {
        fec_info[0].rxbuf[j] =
            (char *)memalign(PKTSIZE_ALIGN, PKTSIZE);
    }

    stm32_eth_device.priv = &fec_info[0];
    stm32_eth_device.ETH_Speed = ETH_Speed_100M;
    stm32_eth_device.ETH_Mode  = ETH_Mode_FullDuplex;

    /* OUI 00-80-E1 STMICROELECTRONICS. */
    stm32_eth_device.dev_addr[0] = 0x00;
    stm32_eth_device.dev_addr[1] = 0x80;
    stm32_eth_device.dev_addr[2] = 0xE1;
    /* generate MAC addr from 96bit unique ID (only for test). */
    stm32_eth_device.dev_addr[3] = *(rt_uint8_t*)(0x10000+4);
    stm32_eth_device.dev_addr[4] = *(rt_uint8_t*)(0x10000+2);
    stm32_eth_device.dev_addr[5] = *(rt_uint8_t*)(0x10000+0);
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

    /* init tx semaphore */
    rt_sem_init(&tx_wait, "tx_wait", 0, RT_IPC_FLAG_FIFO);

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
