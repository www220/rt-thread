#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "fec.h"

#include <netif/ethernetif.h>
#include <netif/ethernetif.h>
#include <netif/etharp.h>
#include <lwip/icmp.h>
#include "lwipopts.h"

/* debug option */
//#define ETH_DEBUG
//#define ETH_RX_DUMP
//#define ETH_TX_DUMP
#define STM32_LINK_PRINTF         rt_kprintf

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

#define FEC_MII_TIMEOUT		50000
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
	fecp->mscr <<= 2;
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

static void fec_get_mac_addr(unsigned char *mac)
{
	u32 val;
	/*set this bit to open the OTP banks for reading*/
	REG_WR(REGS_OCOTP_BASE, HW_OCOTP_CTRL_SET,
		BM_OCOTP_CTRL_RD_BANK_OPEN);

	/*wait until OTP contents are readable*/
	while (BM_OCOTP_CTRL_BUSY & REG_RD(REGS_OCOTP_BASE, HW_OCOTP_CTRL))
		udelay(100);

	mac[0] = 0x00;
	mac[1] = 0x04;
	val = REG_RD(REGS_OCOTP_BASE, HW_OCOTP_CUSTn(0));
	mac[2] = (val >> 24) & 0xFF;
	mac[3] = (val >> 16) & 0xFF;
	mac[4] = (val >> 8) & 0xFF;
	mac[5] = (val >> 0) & 0xFF;
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
    
    mxc_fec_mii_init(fecp);
	fecp->rcr = FEC_RCR_MAX_FL(PKT_MAXBUF_SIZE) | FEC_RCR_RMII_MODE;
	fecp->tcr = FEC_TCR_FDEN;

	/* We use strictly polling mode only */
	fecp->eimr = BM_ENET_MAC0_EIR_RXF;

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
		info->rxbd[i].cbd_bufaddr = (uint)info->rxbuf[i];
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
		info->txbd[i].cbd_bufaddr = (uint)&info->txbuf[0];
	}
	info->txbd[TX_BUF_CNT - 1].cbd_sc |= BD_ENET_TX_WRAP;

	/* Set receive and transmit descriptor base */
	fecp->erdsr = (uint)(&info->rxbd[0]);
	fecp->etdsr = (uint)(&info->txbd[0]);

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

#define ___swab32(x) \
	((u32)( \
		(((u32)(x) & (u32)0x000000ffUL) << 24) | \
		(((u32)(x) & (u32)0x0000ff00UL) <<  8) | \
		(((u32)(x) & (u32)0x00ff0000UL) >>  8) | \
		(((u32)(x) & (u32)0xff000000UL) >> 24) ))
static void swap_packet(void *packet, int length)
{
	int i;
	unsigned int *buf = packet;
	for (i = 0; i < (length + 3) / 4; i++, buf++)
		*buf = ___swab32(*buf);
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

        STM32_ETH_PRINTF("tx_dump, len:%d\r\n", p->tot_len);
        for(i=0; i<p->tot_len; i++)
        {
            STM32_ETH_PRINTF("%02x ",*ptr);
            ptr++;

            if(((i+1)%8) == 0)
            {
                STM32_ETH_PRINTF("  ");
            }
            if(((i+1)%16) == 0)
            {
                STM32_ETH_PRINTF("\r\n");
            }
        }
        STM32_ETH_PRINTF("\r\ndump done!\r\n");
    }
#endif
    swap_packet((uint8_t*)info->txbd[info->txIdx].cbd_bufaddr, offset);
	info->txbd[info->txIdx].cbd_datlen = offset;
	info->txbd[info->txIdx].cbd_sc =
	    (info->txbd[info->txIdx].cbd_sc & BD_ENET_TX_WRAP) |
	    BD_ENET_TX_TC | BD_ENET_TX_RDY_LST;

	/* Activate transmit Buffer Descriptor polling */
	fecp->tdar = 0x01000000;	/* Descriptor polling active    */

	/* FEC fix for MCF5275, FEC unable to initial transmit data packet.
	 * A nop will ensure the descriptor polling active completed.
	 */
	__asm__("nop");

	j = 0;
	while ((info->txbd[info->txIdx].cbd_sc & BD_ENET_TX_READY) &&
	       (j < FEC_MAX_TIMEOUT)) {
		udelay(FEC_TIMEOUT_TICKET);
		j++;
	}
	if (j >= FEC_MAX_TIMEOUT)
		printf("TX timeout packet at %p\n", p);

	rc = (info->txbd[info->txIdx].cbd_sc & BD_ENET_TX_READY);
	info->txIdx = (info->txIdx + 1) % TX_BUF_CNT;

	return (rc & BD_ENET_TX_READY) ? RT_EOK : -RT_ERROR;
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
        
        /* wtdog eth status */
        eth_wtdog = 0;
        framelength = info->rxbd[info->rxIdx].cbd_datlen - 4;
        swap_packet((uint8_t*)info->rxbd[info->rxIdx].cbd_bufaddr, framelength);

        /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
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

            STM32_ETH_PRINTF("rx_dump, len:%d\r\n", p->tot_len);
            for(i=0; i<p->tot_len; i++)
            {
                STM32_ETH_PRINTF("%02x ", *ptr);
                ptr++;

                if(((i+1)%8) == 0)
                {
                    STM32_ETH_PRINTF("  ");
                }
                if(((i+1)%16) == 0)
                {
                    STM32_ETH_PRINTF("\r\n");
                }
            }
            STM32_ETH_PRINTF("\r\ndump done!\r\n");
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
	fecp->rdar = 0x01000000; /* Descriptor polling active    */
        
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
        STM32_LINK_PRINTF("phy not probe!\r\n");
        return;
    }
    else
    {
        STM32_LINK_PRINTF("found a phy, address:0x%02X\r\n", phy_addr);
    }

    /* RESET PHY */
    STM32_LINK_PRINTF("RESET PHY!\r\n");
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
                STM32_LINK_PRINTF("link up ");

                if(phy_speed_new & PHY_100M_MASK)
                {
                    STM32_LINK_PRINTF("100Mbps");
                    stm32_eth_device.ETH_Speed = ETH_Speed_100M;
                }
                else
                {
                    stm32_eth_device.ETH_Speed = ETH_Speed_10M;
                    STM32_LINK_PRINTF("10Mbps");
                }

                if(phy_speed_new & PHY_DUPLEX_MASK)
                {
                    STM32_LINK_PRINTF(" full-duplex\r\n");
                    stm32_eth_device.ETH_Mode = ETH_Mode_FullDuplex;
                }
                else
                {
                    STM32_LINK_PRINTF(" half-duplex\r\n");
                    stm32_eth_device.ETH_Mode = ETH_Mode_HalfDuplex;
                }

                /* send link up. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_TRUE);
				/* wtdog eth status */
				eth_wtdog = 0;
 				eth_linkstatus = 1;
           } /* link up. */
            else
            {
                STM32_LINK_PRINTF("link down\r\n");
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

void rt_hw_enetmac_handler(int vector, void *param)
{
    register rt_uint32_t ir = REG_RD(0x800F0004, HW_ENET_MAC0_EIR);
    if (ir & BM_ENET_MAC0_EIR_RXF)
    {
        STM32_ETH_PRINTF("ETH_DMA_IT_R\r\n");
        /* a frame has been received */
        eth_device_ready(&(stm32_eth_device.parent));
    }
    REG_WR(0x800F0004, HW_ENET_MAC0_EIR, ir);
}

void rt_hw_eth_init(void)
{
    int j;

    /* Set up EMAC pins */
    pin_set_group(&enet_pins);

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

    /* generate MAC addr from 96bit unique ID (only for test). */
    fec_get_mac_addr(stm32_eth_device.dev_addr);
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

    rt_hw_interrupt_install(IRQ_ENET_MAC0, rt_hw_enetmac_handler, RT_NULL, "EMac");
    rt_hw_interrupt_umask(IRQ_ENET_MAC0);
}
