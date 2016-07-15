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

#define CONFIG_USB_EHCI
#define CONFIG_EHCI_IS_TDI
//#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_CMD_USB
#define CONFIG_DOS_PARTITION

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

#define __u32 u32
#define __u16 u16
#define __u8 u8

typedef struct {
    unsigned short indx; /* index into big table */
    unsigned short used; /* bitmask of used entries */
} Summary16;

typedef unsigned int ucs4_t;
typedef int conv_t;

/* Return code if invalid. (xxx_wctomb) */
#define RET_ILUNI      -1
/* Return code if output buffer is too small. (xxx_wctomb, xxx_reset) */
#define RET_TOOSMALL   -2

/* Return code if invalid input after a shift sequence of n bytes was read.
   (xxx_mbtowc) */
#define RET_SHIFT_ILSEQ(n)  (-1-2*(n))
/* Return code if invalid. (xxx_mbtowc) */
#define RET_ILSEQ           RET_SHIFT_ILSEQ(0)
/* Return code if only a shift sequence of n bytes was read. (xxx_mbtowc) */
#define RET_TOOFEW(n)       (-2-2*(n))

int sym_conv_unicode_asc( const char* src, int srclen, char* desc, int desclen );
int sym_conv_asc_unicode( const char* src, int srclen, char *desc, int desclen );
int sym_conv_asc_utf8( const char* src, int srclen, char *desc, int desclen );
int sym_conv_utf8_asc( const char* src, int srclen, char* desc, int desclen );

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

#define uswap_16(x) \
	((((x) & 0xff00) >> 8) | \
	 (((x) & 0x00ff) << 8))
#define uswap_32(x) \
	((((x) & 0xff000000) >> 24) | \
	 (((x) & 0x00ff0000) >>  8) | \
	 (((x) & 0x0000ff00) <<  8) | \
	 (((x) & 0x000000ff) << 24))
#define _uswap_64(x, sfx) \
	((((x) & 0xff00000000000000##sfx) >> 56) | \
	 (((x) & 0x00ff000000000000##sfx) >> 40) | \
	 (((x) & 0x0000ff0000000000##sfx) >> 24) | \
	 (((x) & 0x000000ff00000000##sfx) >>  8) | \
	 (((x) & 0x00000000ff000000##sfx) <<  8) | \
	 (((x) & 0x0000000000ff0000##sfx) << 24) | \
	 (((x) & 0x000000000000ff00##sfx) << 40) | \
	 (((x) & 0x00000000000000ff##sfx) << 56))
#if defined(__GNUC__)
# define uswap_64(x) _uswap_64(x, ull)
#else
# define uswap_64(x) _uswap_64(x, )
#endif

# define cpu_to_le16(x)		(x)
# define cpu_to_le32(x)		(x)
# define cpu_to_le64(x)		(x)
# define le16_to_cpu(x)		(x)
# define le32_to_cpu(x)		(x)
# define le64_to_cpu(x)		(x)
# define cpu_to_be16(x)		uswap_16(x)
# define cpu_to_be32(x)		uswap_32(x)
# define cpu_to_be64(x)		uswap_64(x)
# define be16_to_cpu(x)		uswap_16(x)
# define be32_to_cpu(x)		uswap_32(x)
# define be64_to_cpu(x)		uswap_64(x)

#define le16_to_cpus(x) do {} while (0)
#define le32_to_cpus(x) do {} while (0)
#define le64_to_cpus(x) do {} while (0)

#endif	/* __COMMON_H_ */
