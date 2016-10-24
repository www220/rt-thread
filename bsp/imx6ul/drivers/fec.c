#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"

#include <netif/ethernetif.h>
#include <netif/ethernetif.h>
#include <netif/etharp.h>
#include <lwip/icmp.h>
#include "lwipopts.h"

#undef ALIGN
#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/imx-common/iomux-v3.h>
#include "fec.h"

/* debug option */
//#define ETH_DEBUG
//#define ETH_RX_DUMP
//#define ETH_TX_DUMP
#define ETH_LINK

#ifdef ETH_LINK
#define STM32_LINK_PRINTF         rt_kprintf
#else
#define STM32_LINK_PRINTF(...)
#endif

#ifdef ETH_DEBUG
#define STM32_ETH_PRINTF          rt_kprintf
#else
#define STM32_ETH_PRINTF(...)
#endif

#if defined(ETH_RX_DUMP) || defined(ETH_TX_DUMP)
#define STM32_DUMP_PRINTF          rt_kprintf
#else
#define STM32_DUMP_PRINTF(...)
#endif

struct fec_info_s fec_info[] = {
	{
	 0,			/* index */
	 ENET_BASE_ADDR,	/* io base */
	 0,			/* RX BD */
	 0,			/* TX BD */
	 0,			/* rx Index */
	 0,			/* tx Index */
	 0,			/* tx buffer */
	 { 0 },		/* rx buffer */
	 },
	{
	 1,			/* index */
	 ENET2_BASE_ADDR,	/* io base */
	 0,			/* RX BD */
	 0,			/* TX BD */
	 0,			/* rx Index */
	 0,			/* tx Index */
	 0,			/* tx buffer */
	 { 0 },		/* rx buffer */
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

    char *devname;
	char *irqname;
	int irq;
	char phy_addr;
	volatile int init;
};

static struct rt_stm32_eth stm32_eth_device[2] = {
	{
	.devname = "e0",
	.irqname = "EMac0",
	.irq = IMX_INT_ENET1,
	.phy_addr = 1,
	},
	{
	.devname = "e1",
	.irqname = "EMac1",
	.irq = IMX_INT_ENET2,
	.phy_addr = 5,
	},
};

/* Ethernet Transmit and Receive Buffers */
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

#define FEC_MII_TIMEOUT		5000
#define FEC_MII_TICK		1
#define	FEC_RESET_DELAY		10000
#define FEC_MAX_TIMEOUT		10000
#define FEC_MAX_TICKET		1

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

static void mxc_fec_mii_init(volatile fec_t *fecp)
{
	register u32 speed = DIV_ROUND_UP(imx_get_fecclk(), 5000000);
	register u32 holdtime = DIV_ROUND_UP(imx_get_fecclk(), 100000000) - 1;
#ifdef FEC_QUIRK_ENET_MAC
	speed--;
#endif
	speed <<= 1;
	holdtime <<= 8;
	fecp->mscr = speed | holdtime;
}

static void fec_reset(struct rt_stm32_eth *dev)
{
	struct fec_info_s *info = dev->priv;
	volatile fec_t *fecp = (fec_t *)(info->iobase);
	int i;

	fecp->ecr |= FEC_ECR_RESET;
	for (i = 0; (fecp->ecr & FEC_ECR_RESET) && (i < FEC_RESET_DELAY); ++i)
		udelay(1000);

	if (i == FEC_RESET_DELAY)
		rt_kprintf("PHY %02X FEC_ECR_RESET timeout\n", dev->phy_addr);
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

	/* We use interrupt mode only */
	fecp->eimr = FEC_EIR_RXF|FEC_EIR_EBERR;
	/* Clear any pending interrupt */
	fecp->eir = 0xffffffff;

	fecp->rcr = FEC_RCR_MAX_FL(PKTSIZE) | FEC_RCR_RMII_MODE | FEC_RCR_MII_MODE | FEC_RCR_FCE;
	fecp->tcr = FEC_TCR_FDEN;
	mxc_fec_mii_init(fecp);
	stm32_eth->init = 1;

	fecp->opd = 0x00010020;
	fecp->tfwr = FEC_TFWR_X_WMRK_128;

	/* Clear unicast address hash table */
	fecp->iaur = 0;
	fecp->ialr = 0;
	/* Clear multicast address hash table */
	fecp->gaur = 0;
	fecp->galr = 0;

	/* Set station address   */
	ea = stm32_eth->dev_addr;
	fecp->palr = (ea[0] << 24) | (ea[1] << 16) | (ea[2] << 8) | (ea[3]);
	fecp->paur = (ea[4] << 24) | (ea[5] << 16);

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
		info->rxbd[i].cbd_bufaddr = (uint)info->rxbuf[i];
	}
	info->rxbd[PKTBUFSRX - 1].cbd_sc |= BD_ENET_RX_WRAP;

	/*
	 * Setup Ethernet Transmitter Buffer Descriptors (13.14.24.19)
	 * Settings:
	 *    Last, Tx CRC
	 */
	for (i = 0; i < TX_BUF_CNT; i++) {
		info->txbd[i].cbd_sc = 0;
		info->txbd[i].cbd_datlen = 0;	/* Reset */
		info->txbd[i].cbd_bufaddr = (uint)&info->txbuf[0];
	}
	info->txbd[TX_BUF_CNT - 1].cbd_sc |= BD_ENET_TX_WRAP;

	/* Set receive and transmit descriptor base */
	fecp->erdsr = (uint)(&info->rxbd[0]);
	fecp->etdsr = (uint)(&info->txbd[0]);

#ifdef FEC_QUIRK_ENET_MAC
	fecp->ecr |= FEC_ECR_DBSWAP;
	fecp->tfwr |= FEC_TFWR_X_WMRK_STRFWD;
#endif

	/* Now enable the transmit and receive processing */
	fecp->ecr |= FEC_ECR_ETHER_EN;

#ifdef FEC_QUIRK_ENET_MAC
	fecp->ecr &= ~FEC_ECR_SPEED;
	fecp->rcr &= ~FEC_RCR_RMII_10T;
#endif

	/* And last, try to fill Rx Buffer Descriptors */
	fecp->rdar = FEC_RDAR_R_DES_ACTIVE;	/* Descriptor polling active    */
	udelay(100000);
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
	struct rt_stm32_eth * stm32_eth = (struct rt_stm32_eth *)dev;
	switch(cmd)
	{
	case NIOCTL_GADDR:
		/* get mac address */
		if(args) rt_memcpy(args, stm32_eth->dev_addr, MAX_ADDR_LEN);
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
    
    struct pbuf* q;
    rt_uint32_t offset,j,rc;
    
    offset = 0;
    for (q = p; q != NULL; q = q->next)
    {
        /* Copy the frame to be sent into memory pointed by the current ETHERNET DMA Tx descriptor */
        rt_memcpy((uint8_t*)info->txbd[info->txIdx].cbd_bufaddr+offset,q->payload,q->len);
        offset += q->len;
    }
#ifdef ETH_TX_DUMP
    {
        rt_uint32_t i;
        rt_uint8_t *ptr = (rt_uint8_t*)(info->txbd[info->txIdx].cbd_bufaddr);

        STM32_DUMP_PRINTF("tx_dump, len:%d\r\n", p->tot_len);
        for(i=0; i<p->tot_len; i++)
        {
            STM32_DUMP_PRINTF("%02x ",*ptr);
            ptr++;

            if(((i+1)%8) == 0)
            {
                STM32_DUMP_PRINTF("  ");
            }
            if(((i+1)%16) == 0)
            {
                STM32_DUMP_PRINTF("\r\n");
            }
        }
        STM32_DUMP_PRINTF("\r\ndump done!\r\n");
    }
#endif
	info->txbd[info->txIdx].cbd_datlen = offset;
	info->txbd[info->txIdx].cbd_sc =
	    (info->txbd[info->txIdx].cbd_sc & BD_ENET_TX_WRAP) |
	    BD_ENET_TX_TC | BD_ENET_TX_RDY_LST;

	/* Activate transmit Buffer Descriptor polling */
	fecp->tdar = FEC_TDAR_X_DES_ACTIVE;	/* Descriptor polling active    */

	j = 0;
	while ((info->txbd[info->txIdx].cbd_sc & BD_ENET_TX_READY) &&
	       (j < FEC_MAX_TIMEOUT)) {
		udelay(FEC_MAX_TICKET);
		j++;
	}
	if (j >= FEC_MAX_TIMEOUT)
		rt_kprintf("PHY %02X TX timeout packet at %p\n", stm32_eth->phy_addr, p);

	rc = (info->txbd[info->txIdx].cbd_sc & BD_ENET_TX_READY);
	info->txIdx = (info->txIdx + 1) % TX_BUF_CNT;

	return ((rc & BD_ENET_TX_READY)==0) ? RT_EOK : -RT_ERROR;
}

/* reception packet. */
struct pbuf *rt_stm32_eth_rx(rt_device_t dev)
{
    struct rt_stm32_eth * stm32_eth = (struct rt_stm32_eth *)dev;
	struct fec_info_s *info = stm32_eth->priv;
	volatile fec_t *fecp = (fec_t *) (info->iobase);

    struct pbuf *p,*q;
    rt_uint32_t offset,framelength;

    /* init p pointer */
    p = RT_NULL;

	do {
		/* section 16.9.23.2 */
		if (info->rxbd[info->rxIdx].cbd_sc & BD_ENET_RX_EMPTY) {
			return NULL;	/* nothing received - leave for() loop */
		}

		if (info->rxbd[info->rxIdx].cbd_sc & 0x003f) {
			break;	/* error received - leave for() loop */
		}

        /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
        framelength = info->rxbd[info->rxIdx].cbd_datlen - 4;
        p = pbuf_alloc(PBUF_LINK, framelength, PBUF_RAM);
        if (p == RT_NULL) {
            break;
        }

        offset = 0;
        for (q = p; q != RT_NULL; q= q->next)
        {
            /* Copy the received frame into buffer from memory pointed by the current ETHERNET DMA Rx descriptor */
            rt_memcpy(q->payload,(uint8_t*)info->rxbd[info->rxIdx].cbd_bufaddr+offset,q->len);
            offset += q->len;
        }
#ifdef ETH_RX_DUMP
        {
            rt_uint32_t i;
            rt_uint8_t *ptr = (rt_uint8_t*)(info->rxbd[info->rxIdx].cbd_bufaddr);

            STM32_DUMP_PRINTF("rx_dump, len:%d\r\n", p->tot_len);
            for(i=0; i<p->tot_len; i++)
            {
                STM32_DUMP_PRINTF("%02x ", *ptr);
                ptr++;

                if(((i+1)%8) == 0)
                {
                    STM32_DUMP_PRINTF("  ");
                }
                if(((i+1)%16) == 0)
                {
                    STM32_DUMP_PRINTF("\r\n");
                }
            }
            STM32_DUMP_PRINTF("\r\ndump done!\r\n");
        }
#endif
	} while (0);

    /* Give the buffer back to the FEC. */
	info->rxbd[info->rxIdx].cbd_datlen = 0;
	/* wrap around buffer index when necessary */
	if (info->rxIdx == LAST_PKTBUFSRX) {
		info->rxbd[PKTBUFSRX - 1].cbd_sc = BD_ENET_RX_W_E;
		info->rxIdx = 0;
	} else {
		info->rxbd[info->rxIdx].cbd_sc = BD_ENET_RX_EMPTY;
		info->rxIdx++;
	}

    /* Try to fill Buffer Descriptors */
	fecp->rdar = FEC_RDAR_R_DES_ACTIVE; /* Descriptor polling active    */
        
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

uint32_t ETH_WritePHYRegister(u32 iobase, uint8_t PHYAddress, uint8_t PHYReg, uint16_t PHYValue)
{
	volatile fec_t *fecp = (fec_t *)iobase;

	return __fec_mii_write(fecp, PHYAddress, PHYReg, PHYValue);
}

uint16_t ETH_ReadPHYRegister(u32 iobase, uint8_t PHYAddress, uint8_t PHYReg)
{
    uint16_t PHYValue = 0;
	volatile fec_t *fecp = (fec_t *)iobase;

	__fec_mii_read(fecp, PHYAddress, PHYReg, &PHYValue);
	return PHYValue;
}

#define RT_USING_DP83848
#ifdef RT_USING_DP83848
/* PHY: DP83848 */
#define PHY_LINK_MASK       (1<<0)
#define PHY_100M_MASK       (1<<1)
#define PHY_DUPLEX_MASK     (1<<2)
static void phy_monitor_thread_entry(void *parameter)
{
    struct rt_stm32_eth * stm32_eth = (struct rt_stm32_eth *)parameter;
    struct fec_info_s *info = stm32_eth->priv;
    uint32_t mem_addr = info->iobase;

    uint8_t phy_speed[2] = {0};
    uint8_t phy_speed_new[2] = {0};
    int i = 0,reset_count[2] = {0};

    while (1)
    {
        if (stm32_eth->init == 0)
        {
            rt_thread_delay(RT_TICK_PER_SECOND/2);
            continue;
        }
        else if (stm32_eth->init == 1)
        {
            /* RESET PHY */
            STM32_LINK_PRINTF("reset PHY address:%02X\r\n", 0x01);
            ETH_WritePHYRegister(mem_addr, 0x01, PHY_BCR, PHY_Reset);
            STM32_LINK_PRINTF("reset PHY address:%02X\r\n", 0x05);
            ETH_WritePHYRegister(mem_addr, 0x05, PHY_BCR, PHY_Reset);
            rt_thread_delay(RT_TICK_PER_SECOND);
            ETH_WritePHYRegister(mem_addr, 0x01, PHY_BCR, PHY_AutoNegotiation);
            ETH_WritePHYRegister(mem_addr, 0x05, PHY_BCR, PHY_AutoNegotiation);
            stm32_eth->init = 2;
            continue;
        }
        uint8_t phy_addr = stm32_eth_device[i].phy_addr;
        uint16_t status  = ETH_ReadPHYRegister(mem_addr, phy_addr, PHY_BSR);
        STM32_ETH_PRINTF("PHY %02X DP83848 status:0x%04X\r\n", phy_addr, status);

        phy_speed_new[i] = 0;

        if(status & (PHY_AutoNego_Complete | PHY_Linked_Status))
        {
            uint16_t SR;

            SR = ETH_ReadPHYRegister(mem_addr, phy_addr, 16);
            STM32_ETH_PRINTF("PHY %02X DP83848 REG 16:0x%04X\r\n", phy_addr, SR);

            SR = SR & 0x07; /* DP83848, REG16[2:0], Speed Indication. */
            phy_speed_new[i] = PHY_LINK_MASK;

            if((SR & 0x02) == 0)
            {
                phy_speed_new[i] |= PHY_100M_MASK;
            }

            if(SR & 0x04)
            {
                phy_speed_new[i] |= PHY_DUPLEX_MASK;
            }

            SR = ETH_ReadPHYRegister(mem_addr, phy_addr, 21);
            STM32_ETH_PRINTF("PHY %02X DP83848 REG 21:0x%04X\r\n", phy_addr, SR);
            /* DP83848, REG21[7:0], Error Counter.[[COR]] */
        }

        /* linkchange */
        if(phy_speed_new[i] != phy_speed[i] && reset_count[i] <= 0)
        {
            if(phy_speed_new[i] & PHY_LINK_MASK)
            {
                STM32_LINK_PRINTF("PHY %02X link up ", phy_addr);

                if(phy_speed_new[i] & PHY_100M_MASK)
                {
                    STM32_LINK_PRINTF("100Mbps");
                    stm32_eth_device[i].ETH_Speed = ETH_Speed_100M;
                }
                else
                {
                	stm32_eth_device[i].ETH_Speed = ETH_Speed_10M;
                    STM32_LINK_PRINTF("10Mbps");
                }

                if(phy_speed_new[i] & PHY_DUPLEX_MASK)
                {
                    STM32_LINK_PRINTF(" full-duplex\r\n");
                    stm32_eth_device[i].ETH_Mode = ETH_Mode_FullDuplex;
                }
                else
                {
                    STM32_LINK_PRINTF(" half-duplex\r\n");
                    stm32_eth_device[i].ETH_Mode = ETH_Mode_HalfDuplex;
                }

                /* send link up. */
                eth_device_linkchange(&stm32_eth_device[i].parent, RT_TRUE);
            } /* link up. */
            else
            {
                STM32_LINK_PRINTF("PHY %02X link down\r\n", phy_addr);
                /* send link down. */
                eth_device_linkchange(&stm32_eth_device[i].parent, RT_FALSE);
            } /* link down. */

            phy_speed[i] = phy_speed_new[i];
            reset_count[i] = 3;
        } /* linkchange */
        else if (phy_speed_new[i] != phy_speed[i])
        {
            reset_count[i]--;
        }
        else
        {
            reset_count[i] = 3;
        }

        rt_thread_delay(RT_TICK_PER_SECOND/2);
        if (i==0) i=1; else i=0;
    } /* while(1) */
}
#endif

extern unsigned char NET_MAC[2][6];
extern int NET_DHCP[2];

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH   |                                   \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)
#define MDIO_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST | PAD_CTL_ODE)
#define ENET_CLK_PAD_CTRL  (PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)
#define ENET_RX_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |          \
	PAD_CTL_SPEED_HIGH   | PAD_CTL_SRE_FAST)

static iomux_v3_cfg_t const fec1_pads[] = {
	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__ENET1_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
};
static iomux_v3_cfg_t const fec2_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET2_MDIO | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET2_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA0__ENET2_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA1__ENET2_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_EN__ENET2_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_CLK__ENET2_REF_CLK2 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA0__ENET2_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA1__ENET2_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_EN__ENET2_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_ER__ENET2_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

void rt_hw_enetmac_handler(int vector, void *param)
{
    struct rt_stm32_eth * stm32_eth = (struct rt_stm32_eth *)param;
	struct fec_info_s *info = stm32_eth->priv;
	volatile fec_t *fecp = (fec_t *) (info->iobase);

    rt_uint32_t ievent = fecp->eir;
    STM32_ETH_PRINTF("ETH_ITR %x\r\n",ievent);

    if (ievent & FEC_EIR_EBERR)
    {
        /* bus dma err stoped */
        stm32_eth->init = 0;
        rt_stm32_eth_init(&stm32_eth->parent.parent);
    }
    else if (ievent & FEC_EIR_RXF)
    {
        /* a frame has been received */
        eth_device_ready(&stm32_eth->parent);
    }

    fecp->eir = FEC_EIR_RXF|FEC_EIR_EBERR;
}

void rt_hw_eth_init(void)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;
    int i,j;
    rt_uint16_t flags;

	imx_iomux_v3_setup_multiple_pads(fec1_pads, ARRAY_SIZE(fec1_pads));
	imx_iomux_v3_setup_multiple_pads(fec2_pads, ARRAY_SIZE(fec2_pads));

	/* Use 50M anatop loopback REF_CLK1 for ENET1, clear gpr1[13], set gpr1[17]*/
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
			IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);
	/* Use 50M anatop loopback REF_CLK2 for ENET2, clear gpr1[14], set gpr1[18]*/
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
			IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);

	enable_fec_anatop_clock(-1, ENET_50MHZ);
	enable_enet_clk(true);

	gpio_direction_output(PIN_NET0, 0);
	udelay(1000);
	gpio_set_value(PIN_NET0, 1);
	gpio_direction_output(PIN_NET1, 0);
	udelay(1000);
	gpio_set_value(PIN_NET1, 1);

    for (i=0; i<2; i++) {
    fec_info[i].rxbd = (cbd_t *)rt_memalign(ARCH_DMA_MINALIGN, (PKTBUFSRX * sizeof(cbd_t)));
    fec_info[i].txbd = (cbd_t *)rt_memalign(ARCH_DMA_MINALIGN, (TX_BUF_CNT * sizeof(cbd_t)));
    fec_info[i].txbuf = (char *)rt_memalign(ARCH_DMA_MINALIGN, PKT_MAXBLR_SIZE);
    for (j = 0; j < PKTBUFSRX; ++j) {
        fec_info[i].rxbuf[j] = (char *)rt_memalign(ARCH_DMA_MINALIGN, PKT_MAXBLR_SIZE);
    }

    stm32_eth_device[i].priv = &fec_info[i];
    stm32_eth_device[i].ETH_Speed = ETH_Speed_100M;
    stm32_eth_device[i].ETH_Mode  = ETH_Mode_FullDuplex;

	/* load or save net mac addr */
	if (NET_MAC[i][0] == 0 && NET_MAC[i][1] == 0 && NET_MAC[i][2] == 0)
	{
		NET_MAC[i][0] = stm32_eth_device[i].dev_addr[0];
		NET_MAC[i][1] = stm32_eth_device[i].dev_addr[1];
		NET_MAC[i][2] = stm32_eth_device[i].dev_addr[2];
		NET_MAC[i][3] = stm32_eth_device[i].dev_addr[3];
		NET_MAC[i][4] = stm32_eth_device[i].dev_addr[4];
		NET_MAC[i][5] = stm32_eth_device[i].dev_addr[5];
	}
	else
	{
		stm32_eth_device[i].dev_addr[0] = NET_MAC[i][0];
		stm32_eth_device[i].dev_addr[1] = NET_MAC[i][1];
		stm32_eth_device[i].dev_addr[2] = NET_MAC[i][2];
		stm32_eth_device[i].dev_addr[3] = NET_MAC[i][3];
		stm32_eth_device[i].dev_addr[4] = NET_MAC[i][4];
		stm32_eth_device[i].dev_addr[5] = NET_MAC[i][5];
	}
    stm32_eth_device[i].parent.parent.init       = rt_stm32_eth_init;
    stm32_eth_device[i].parent.parent.open       = rt_stm32_eth_open;
    stm32_eth_device[i].parent.parent.close      = rt_stm32_eth_close;
    stm32_eth_device[i].parent.parent.read       = rt_stm32_eth_read;
    stm32_eth_device[i].parent.parent.write      = rt_stm32_eth_write;
    stm32_eth_device[i].parent.parent.control    = rt_stm32_eth_control;
    stm32_eth_device[i].parent.parent.user_data  = RT_NULL;

    stm32_eth_device[i].parent.eth_rx     = rt_stm32_eth_rx;
    stm32_eth_device[i].parent.eth_tx     = rt_stm32_eth_tx;

    /* register eth device */
    flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | ETHIF_LINK_PHYUP;
    /* DHCP support */
    if (NET_DHCP[i])
        flags |= NETIF_FLAG_DHCP;
    eth_device_init_with_flag(&stm32_eth_device[i].parent, stm32_eth_device[i].devname, flags);

    /* start phy monitor */
    if (i==1)
    {
        rt_thread_t tid;
        tid = rt_thread_create("phy",
                               phy_monitor_thread_entry,
                               &stm32_eth_device[i],
                               512,
                               RT_THREAD_PRIORITY_MAX - 4,
                               20);
        if (tid != RT_NULL)
            rt_thread_startup(tid);
    }

    rt_hw_interrupt_install(stm32_eth_device[i].irq, rt_hw_enetmac_handler, &stm32_eth_device[i], stm32_eth_device[i].irqname);
    rt_hw_interrupt_umask(stm32_eth_device[i].irq);}
}
