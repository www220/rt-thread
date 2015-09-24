#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include <dfs_fs.h>

#define USBH_LINK
#ifdef USBH_LINK
#define USBH_LINK_PRINTF         rt_kprintf
#else
#define USBH_LINK_PRINTF(...)
#endif

static struct pin_desc usbh_pins_desc[] = {
	{ PINID_SSP0_DATA0, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_DATA1, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_DATA2, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_DATA3, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_CMD, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_DETECT, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_SSP0_SCK, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
};
static struct pin_group usbh_pins = {
	.pins		= usbh_pins_desc,
	.nr_pins	= ARRAY_SIZE(usbh_pins_desc)
};

void rt_hw_usbh_init(void)
{
    /* Set up USBH pins */
	pin_set_group(&usbh_pins);
}

void usbh_init(void)
{

}
