#ifndef __COMMON_H_
#define __COMMON_H_	1

#include "mx28.h"

#define CONFIG_GPMI_NFC_SWAP_BLOCK_MARK
#define CONFIG_GPMI_NFC_V1
#define CONFIG_GPMI_REG_BASE	GPMI_BASE_ADDR
#define CONFIG_BCH_REG_BASE		BCH_BASE_ADDR

#define CONFIG_APBH_DMA_V1
#define CONFIG_MXS_DMA_REG_BASE ABPHDMA_BASE_ADDR

#define CONFIG_FEC0_IOBASE		REGS_ENET_BASE
#define CONFIG_FEC0_PHY_ADDR	-1

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

extern unsigned char *dma_align_mem;
#define memalign(x,y) dma_align_mem; dma_align_mem += (u32)(((u32)(y)+31)&(u32)(~31))
#define freealign(x)

extern unsigned char *dma_align_max;
#define memalign_max(x,y) dma_align_max; dma_align_max += (u32)(((u32)(y)+31)&(u32)(~31))
#define freealign_max(x)

/* Dma addresses are 32-bits wide.  */
typedef u32 dma_addr_t;

typedef unsigned long phys_addr_t;
typedef unsigned long phys_size_t;

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/*
 * Generic virtual read/write.  Note that we don't support half-word
 * read/writes.  We define __arch_*[bl] here, and leave __arch_*w
 * to the architecture specific code.
 */
#define __arch_getb(a)			(*(volatile unsigned char *)(a))
#define __arch_getw(a)			(*(volatile unsigned short *)(a))
#define __arch_getl(a)			(*(volatile unsigned int *)(a))

#define __arch_putb(v,a)		(*(volatile unsigned char *)(a) = (v))
#define __arch_putw(v,a)		(*(volatile unsigned short *)(a) = (v))
#define __arch_putl(v,a)		(*(volatile unsigned int *)(a) = (v))

#define writeb(v,a)			__arch_putb(v,a)
#define writew(v,a)			__arch_putw(v,a)
#define writel(v,a)			__arch_putl(v,a)

#define readb(a)			__arch_getb(a)
#define readw(a)			__arch_getw(a)
#define readl(a)			__arch_getl(a)

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})

#endif	/* __COMMON_H_ */