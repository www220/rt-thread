#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include "board.h"
#include <errno.h>

#undef ALIGN
#include <common.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/mx6-pins.h>
#include <asm/imx-common/mxc_i2c.h>

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode =  MX6_PAD_UART4_TX_DATA__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART4_TX_DATA__GPIO1_IO28 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 28),
	},
	.sda = {
		.i2c_mode = MX6_PAD_UART4_RX_DATA__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART4_RX_DATA__GPIO1_IO29 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 29),
	},
};

struct imx_i2c_bus
{
    struct rt_i2c_bus_device parent;
    u32 index;
    u32 iobase;
    struct i2c_pads_info *pad;
};

static rt_size_t imx_i2c_xfer(struct rt_i2c_bus_device *bus,
                              struct rt_i2c_msg msgs[], rt_uint32_t num)
{
   struct rt_i2c_msg *msg;
    rt_uint32_t i;
    rt_err_t ret = RT_ERROR;
    rt_uint32_t stat = 0;
    struct imx_i2c_bus *imx_i2c = (struct imx_i2c_bus *)bus;

    /*start the i2c bus*/
    stat = imx_i2c_start(imx_i2c->I2C);
    if ((I2C_I2STAT_M_TX_RESTART != stat) && (I2C_I2STAT_M_TX_START != stat))
    {
        i2c_dbg("start the i2c bus failed,i2c bus stop!\n");
        goto out;
    }
    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];
        if (!(msg->flags & RT_I2C_NO_START))
        {
            if (i)
            {
                stat = imx_i2c_start(lpc_i2c->I2C);
                if ((I2C_I2STAT_M_TX_RESTART != stat) && (I2C_I2STAT_M_TX_START != stat))
                {
                    i2c_dbg("restart the i2c bus failed,i2c bus stop!\n");
                    goto out;
                }
            }
            stat = i2c_send_addr(lpc_i2c->I2C, msg);
            if (I2C_I2STAT_M_TX_SLAW_ACK != stat && I2C_I2STAT_M_RX_SLAR_ACK != stat)
            {
                i2c_dbg("send i2c address but no ack,i2c stop!");
                goto out;
            }
        }
        if (msg->flags & RT_I2C_RD)
        {
            ret = imx_i2c_recv_bytes(lpc_i2c->I2C, msg);
            if (ret >= 1)
                i2c_dbg("read %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_EIO;
                goto out;
            }
        }
        else
        {
            ret = imx_i2c_send_bytes(lpc_i2c->I2C, msg);
            if (ret >= 1)
                i2c_dbg("write %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_ERROR;
                goto out;
            }
        }
    }
    ret = i;

out:
    i2c_dbg("send stop condition\n");
    imx_i2c_stop(lpc_i2c->I2C);

    return ret;
}

static const struct rt_i2c_bus_device_ops i2c_ops =
{
    imx_i2c_xfer,
    RT_NULL,
    RT_NULL
};
static struct imx_i2c_bus i2c0 = {
	.index = 0,
	.iobase = I2C1_BASE_ADDR,
	.pad = &i2c_pad_info0,
};

static int force_idle_bus(void *priv)
{
	int i;
	int sda, scl;
	ulong elapsed, start_time;
	struct i2c_pads_info *p = (struct i2c_pads_info *)priv;
	int ret = 0;

	gpio_direction_input(p->sda.gp);
	gpio_direction_input(p->scl.gp);

	imx_iomux_v3_setup_pad(p->sda.gpio_mode);
	imx_iomux_v3_setup_pad(p->scl.gpio_mode);

	sda = gpio_get_value(p->sda.gp);
	scl = gpio_get_value(p->scl.gp);
	if ((sda & scl) == 1)
		goto exit;		/* Bus is idle already */

	gpio_direction_output(p->scl.gp, 1);
	udelay(1000);
	/* Send high and low on the SCL line */
	for (i = 0; i < 9; i++) {
		gpio_direction_output(p->scl.gp, 1);
		udelay(50);
		gpio_direction_output(p->scl.gp, 0);
		udelay(50);
	}

	/* Simulate the NACK */
	gpio_direction_output(p->sda.gp, 1);
	udelay(50);
	gpio_direction_output(p->scl.gp, 1);
	udelay(50);
	gpio_direction_output(p->scl.gp, 0);
	udelay(50);

	/* Simulate the STOP signal */
	gpio_direction_output(p->sda.gp, 0);
	udelay(50);
	gpio_direction_output(p->scl.gp, 1);
	udelay(50);
	gpio_direction_output(p->sda.gp, 1);
	udelay(50);

	/* Get the bus status */
	gpio_direction_input(p->sda.gp);
	gpio_direction_input(p->scl.gp);

	start_time = get_timer(0);
	for (;;) {
		sda = gpio_get_value(p->sda.gp);
		scl = gpio_get_value(p->scl.gp);
		if ((sda & scl) == 1)
			break;
		elapsed = get_timer(start_time);
		if (elapsed > (CONFIG_SYS_HZ / 5)) {	/* .2 seconds */
			ret = -EBUSY;
			rt_kprintf("%s: failed to clear bus, sda=%d scl=%d\n",
					__func__, sda, scl);
			break;
		}
	}
exit:
	imx_iomux_v3_setup_pad(p->sda.i2c_mode);
	imx_iomux_v3_setup_pad(p->scl.i2c_mode);
	return ret;
}

void rt_hw_i2c_init(void)
{
	enable_i2c_clk(true, i2c0.index);
	force_idle_bus(i2c0.pad);

	i2c0.parent.ops = &i2c_ops;
	rt_i2c_bus_device_register(&i2c0.parent, "i2c0");
}

