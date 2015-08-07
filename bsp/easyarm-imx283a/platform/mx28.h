/*
 * Copyright (C) 2008 Embedded Alley Solutions Inc.
 *
 * (C) Copyright 2009-2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __MX28_H
#define __MX28_H

/*
 * Most of i.MX28 SoC registers are associated with four addresses
 * used for different operations - read/write, set, clear and toggle bits.
 *
 * Some of registers do not implement such feature and, thus, should be
 * accessed/manipulated via single address in common way.
 */
#define REG_RD(base, reg) \
	(*(volatile unsigned int *)((base) + (reg)))
#define REG_WR(base, reg, value) \
	((*(volatile unsigned int *)((base) + (reg))) = (value))
#define REG_SET(base, reg, value) \
	((*(volatile unsigned int *)((base) + (reg ## _SET))) = (value))
#define REG_CLR(base, reg, value) \
	((*(volatile unsigned int *)((base) + (reg ## _CLR))) = (value))
#define REG_TOG(base, reg, value) \
	((*(volatile unsigned int *)((base) + (reg ## _TOG))) = (value))

#define REG_RD_ADDR(addr) \
	(*(volatile unsigned int *)((addr)))
#define REG_WR_ADDR(addr, value) \
	((*(volatile unsigned int *)((addr))) = (value))
#define REG_SET_ADDR(addr, value) \
	((*(volatile unsigned int *)((addr) + 0x4)) = (value))
#define REG_CLR_ADDR(addr, value) \
	((*(volatile unsigned int *)((addr) + 0x8)) = (value))
#define REG_TOG_ADDR(addr, value) \
	((*(volatile unsigned int *)((addr) + 0xc)) = (value))

/*
 * Register base address
 */
#define REGS_ICOL_BASE		(0x80000000)
#define REGS_HSADC_BASE		(0x80002000)
#define REGS_APBH_BASE		(0x80004000)
#define REGS_PERFMON_BASE	(0x80006000)
#define REGS_BCH_BASE		(0x8000A000)
#define REGS_GPMI_BASE		(0x8000C000)
#define REGS_SSP0_BASE		(0x80010000)
#define REGS_SSP1_BASE		(0x80012000)
#define REGS_SSP2_BASE		(0x80014000)
#define REGS_SSP3_BASE		(0x80016000)
#define REGS_PINCTRL_BASE	(0x80018000)
#define REGS_DIGCTL_BASE	(0x8001C000)
#define REGS_ETM_BASE		(0x80022000)
#define REGS_APBX_BASE		(0x80024000)
#define REGS_DCP_BASE		(0x80028000)
#define REGS_PXP_BASE		(0x8002A000)
#define REGS_OCOTP_BASE		(0x8002C000)
#define REGS_AXI_AHB0_BASE	(0x8002E000)
#define REGS_LCDIF_BASE		(0x80030000)
#define REGS_CAN0_BASE		(0x80032000)
#define REGS_CAN1_BASE		(0x80034000)
#define REGS_SIMDBG_BASE	(0x8003C000)
#define REGS_SIMGPMISEL_BASE	(0x8003C200)
#define REGS_SIMSSPSEL_BASE	(0x8003C300)
#define REGS_SIMMEMSEL_BASE	(0x8003C400)
#define REGS_GPIOMON_BASE	(0x8003C500)
#define REGS_SIMENET_BASE	(0x8003C700)
#define REGS_ARMJTAG_BASE	(0x8003C800)
#define REGS_CLKCTRL_BASE	(0x80040000)
#define REGS_SAIF0_BASE		(0x80042000)
#define REGS_POWER_BASE		(0x80044000)
#define REGS_SAIF1_BASE		(0x80046000)
#define REGS_LRADC_BASE		(0x80050000)
#define REGS_SPDIF_BASE		(0x80054000)
#define REGS_RTC_BASE		(0x80056000)
#define REGS_I2C0_BASE		(0x80058000)
#define REGS_I2C1_BASE		(0x8005A000)
#define REGS_PWM_BASE		(0x80064000)
#define REGS_TIMROT_BASE	(0x80068000)
#define REGS_UARTAPP0_BASE	(0x8006A000)
#define REGS_UARTAPP1_BASE	(0x8006C000)
#define REGS_UARTAPP2_BASE	(0x8006E000)
#define REGS_UARTAPP3_BASE	(0x80070000)
#define REGS_UARTAPP4_BASE	(0x80072000)
#define REGS_UARTDBG_BASE	(0x80074000)
#define REGS_USBPHY0_BASE	(0x8007C000)
#define REGS_USBPHY1_BASE	(0x8007E000)
#define REGS_USBCTRL0_BASE	(0x80080000)
#define REGS_USBCTRL1_BASE	(0x80090000)
#define REGS_DFLPT_BASE		(0x800C0000)
#define REGS_DRAM_BASE		(0x800E0000)
#define REGS_ENET_BASE		(0x800F0000)

#define BCH_BASE_ADDR REGS_BCH_BASE
#define GPMI_BASE_ADDR REGS_GPMI_BASE
#define ABPHDMA_BASE_ADDR  REGS_APBH_BASE


/* IRQ Definitions */
#define IRQ_BATT_BRNOUT			0
#define IRQ_VDDD_BRNOUT			1
#define IRQ_VDDIO_BRNOUT		2
#define IRQ_VDDA_BRNOUT			3
#define IRQ_VDD5V_DROOP			4
#define IRQ_DCDC4P2_BRNOUT		5
#define IRQ_VDD5V			6
#define IRQ_RESV7			7
#define IRQ_CAN0			8
#define IRQ_CAN1			9
#define IRQ_LRADC_TOUCH			10
#define IRQ_RESV11			11
#define IRQ_RESV12			12
#define IRQ_HSADC			13
#define IRQ_IRADC_THRESH0		14
#define IRQ_IRADC_THRESH1		15
#define IRQ_LRADC_CH0			16
#define IRQ_LRADC_CH1			17
#define IRQ_LRADC_CH2			18
#define IRQ_LRADC_CH3			19
#define IRQ_LRADC_CH4			20
#define IRQ_LRADC_CH5			21
#define IRQ_LRADC_CH6			22
#define IRQ_LRADC_CH7			23
#define IRQ_LRADC_BUTTON0		24
#define IRQ_LRADC_BUTTON1		25
#define IRQ_RESV26			26
#define IRQ_PERFMON			27
#define IRQ_RTC_1MSEC			28
#define IRQ_RTC_ALARM			29
#define IRQ_RESV30			30
#define IRQ_COMMS			31
#define IRQ_EMI_ERR			32
#define IRQ_RESV33			33
#define IRQ_RESV34			34
#define IRQ_RESV35			35
#define IRQ_RESV36			36
#define IRQ_RESV37			37
#define IRQ_LCDIF			38
#define IRQ_PXP				39
#define IRQ_RESV40			40
#define IRQ_BCH				41
#define IRQ_GPMI			42
#define IRQ_RESV43			43
#define IRQ_RESV44			44
#define IRQ_SPDIF_ERROR			45
#define IRQ_RESV46			46
#define IRQ_DUART			47
#define IRQ_TIMER0			48
#define IRQ_TIMER1			49
#define IRQ_TIMER2			50
#define IRQ_TIMER3			51
#define IRQ_DCP_VMI			52
#define IRQ_DCP				53
#define IRQ_DCP_SECURE			54
#define IRQ_RESV55			55
#define IRQ_RESV56			56
#define IRQ_RESV57			57
#define IRQ_SAIF1			58
#define IRQ_SAIF0			59
#define IRQ_RESV60			60
#define IRQ_RESV61			61
#define IRQ_RESV62			62
#define IRQ_RESV63			63
#define IRQ_RESV64			64
#define IRQ_RESV65			65
#define IRQ_SPDIF_DMA			66
#define IRQ_RESV67			67
#define IRQ_I2C0_DMA			68
#define IRQ_I2C1_DMA			69
#define IRQ_AUART0_RX_DMA		70
#define IRQ_AUART0_TX_DMA		71
#define IRQ_AUART1_RX_DMA		72
#define IRQ_AUART1_TX_DMA		73
#define IRQ_AUART2_RX_DMA		74
#define IRQ_AUART2_TX_DMA		75
#define IRQ_AUART3_RX_DMA		76
#define IRQ_AUART3_TX_DMA		77
#define IRQ_AUART4_RX_DMA		78
#define IRQ_AUART4_TX_DMA		79
#define IRQ_SAIF0_DMA			80
#define IRQ_SAIF1_DMA			81
#define IRQ_SSP0_DMA			82
#define IRQ_SSP1_DMA			83
#define IRQ_SSP2_DMA			84
#define IRQ_SSP3_DMA			85
#define IRQ_LCDIF_DMA			86
#define IRQ_HSADC_DMA			87
#define IRQ_GPMI_DMA			88
#define IRQ_DIGCTL_DEBUG_TRAP		89
#define IRQ_RESV90			90
#define IRQ_RESV91			91
#define IRQ_USB1			92
#define IRQ_USB0			93
#define IRQ_USB1_WAKEUP			94
#define IRQ_USB0_WAKEUP			95
#define IRQ_SSP0			96
#define IRQ_SSP1			97
#define IRQ_SSP2			98
#define IRQ_SSP3			99
#define IRQ_ENET_SWI			100
#define IRQ_ENET_MAC0			101
#define IRQ_ENET_MAC1			102
#define IRQ_ENET_MAC0_1588		103
#define IRQ_ENET_MAC1_1588		104
#define IRQ_RESV105			105
#define IRQ_RESV106			106
#define IRQ_RESV107			107
#define IRQ_RESV108			108
#define IRQ_RESV109			109
#define IRQ_I2C1_ERROR			110
#define IRQ_I2C0_ERROR			111
#define IRQ_AUART0			112
#define IRQ_AUART1			113
#define IRQ_AUART2			114
#define IRQ_AUART3			115
#define IRQ_AUART4			116
#define IRQ_RESV117			117
#define IRQ_RESV118			118
#define IRQ_RESV119			119
#define IRQ_RESV120			120
#define IRQ_RESV121			121
#define IRQ_RESV122			122
#define IRQ_GPIO4			123
#define IRQ_GPIO3			124
#define IRQ_GPIO2			125
#define IRQ_GPIO1			126
#define IRQ_GPIO0			127

#define ARCH_NR_IRQS		128

#endif /* __MX28_H */
