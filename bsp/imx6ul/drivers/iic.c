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

#define I2CR_IIEN	(1 << 6)
#define I2CR_MSTA	(1 << 5)
#define I2CR_MTX	(1 << 4)
#define I2CR_TX_NO_AK	(1 << 3)
#define I2CR_RSTA	(1 << 2)

#define I2SR_ICF	(1 << 7)
#define I2SR_IBB	(1 << 5)
#define I2SR_IAL	(1 << 4)
#define I2SR_IIF	(1 << 1)
#define I2SR_RX_NO_AK	(1 << 0)

#ifdef I2C_QUIRK_REG
#define I2CR_IEN	(0 << 7)
#define I2CR_IDIS	(1 << 7)
#define I2SR_IIF_CLEAR	(1 << 1)
#else
#define I2CR_IEN	(1 << 7)
#define I2CR_IDIS	(0 << 7)
#define I2SR_IIF_CLEAR	(0 << 1)
#endif

#ifdef I2C_QUIRK_REG
struct mxc_i2c_regs {
	uint8_t		iadr;
	uint8_t		ifdr;
	uint8_t		i2cr;
	uint8_t		i2sr;
	uint8_t		i2dr;
};
#else
struct mxc_i2c_regs {
	uint32_t	iadr;
	uint32_t	ifdr;
	uint32_t	i2cr;
	uint32_t	i2sr;
	uint32_t	i2dr;
};
#endif

#ifdef I2C_QUIRK_REG
static u16 i2c_clk_div[60][2] = {
	{ 20,	0x00 }, { 22,	0x01 }, { 24,	0x02 }, { 26,	0x03 },
	{ 28,	0x04 },	{ 30,	0x05 },	{ 32,	0x09 }, { 34,	0x06 },
	{ 36,	0x0A }, { 40,	0x07 }, { 44,	0x0C }, { 48,	0x0D },
	{ 52,	0x43 },	{ 56,	0x0E }, { 60,	0x45 }, { 64,	0x12 },
	{ 68,	0x0F },	{ 72,	0x13 },	{ 80,	0x14 },	{ 88,	0x15 },
	{ 96,	0x19 },	{ 104,	0x16 },	{ 112,	0x1A },	{ 128,	0x17 },
	{ 136,	0x4F }, { 144,	0x1C },	{ 160,	0x1D }, { 176,	0x55 },
	{ 192,	0x1E }, { 208,	0x56 },	{ 224,	0x22 }, { 228,	0x24 },
	{ 240,	0x1F },	{ 256,	0x23 }, { 288,	0x5C },	{ 320,	0x25 },
	{ 384,	0x26 }, { 448,	0x2A },	{ 480,	0x27 }, { 512,	0x2B },
	{ 576,	0x2C },	{ 640,	0x2D },	{ 768,	0x31 }, { 896,	0x32 },
	{ 960,	0x2F },	{ 1024,	0x33 },	{ 1152,	0x34 }, { 1280,	0x35 },
	{ 1536,	0x36 }, { 1792,	0x3A },	{ 1920,	0x37 },	{ 2048,	0x3B },
	{ 2304,	0x3C },	{ 2560,	0x3D },	{ 3072,	0x3E }, { 3584,	0x7A },
	{ 3840,	0x3F }, { 4096,	0x7B }, { 5120,	0x7D },	{ 6144,	0x7E },
};
#else
static u16 i2c_clk_div[50][2] = {
	{ 22,	0x20 }, { 24,	0x21 }, { 26,	0x22 }, { 28,	0x23 },
	{ 30,	0x00 }, { 32,	0x24 }, { 36,	0x25 }, { 40,	0x26 },
	{ 42,	0x03 }, { 44,	0x27 }, { 48,	0x28 }, { 52,	0x05 },
	{ 56,	0x29 }, { 60,	0x06 }, { 64,	0x2A }, { 72,	0x2B },
	{ 80,	0x2C }, { 88,	0x09 }, { 96,	0x2D }, { 104,	0x0A },
	{ 112,	0x2E }, { 128,	0x2F }, { 144,	0x0C }, { 160,	0x30 },
	{ 192,	0x31 }, { 224,	0x32 }, { 240,	0x0F }, { 256,	0x33 },
	{ 288,	0x10 }, { 320,	0x34 }, { 384,	0x35 }, { 448,	0x36 },
	{ 480,	0x13 }, { 512,	0x37 }, { 576,	0x14 }, { 640,	0x38 },
	{ 768,	0x39 }, { 896,	0x3A }, { 960,	0x17 }, { 1024,	0x3B },
	{ 1152,	0x18 }, { 1280,	0x3C }, { 1536,	0x3D }, { 1792,	0x3E },
	{ 1920,	0x1B }, { 2048,	0x3F }, { 2304,	0x1C }, { 2560,	0x1D },
	{ 3072,	0x1E }, { 3840,	0x1F }
};
#endif

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
    u32 index;
    u32 iobase;
    struct i2c_pads_info *pad;
};

/*
 * Calculate and set proper clock divider
 */
static uint8_t i2c_imx_get_clk(unsigned int rate)
{
	unsigned int i2c_clk_rate;
	unsigned int div;
	u8 clk_div;

#if defined(CONFIG_MX31)
	struct clock_control_regs *sc_regs =
		(struct clock_control_regs *)CCM_BASE;

	/* start the required I2C clock */
	writel(readl(&sc_regs->cgr0) | (3 << CONFIG_SYS_I2C_CLK_OFFSET),
		&sc_regs->cgr0);
#endif

	/* Divider value calculation */
	i2c_clk_rate = mxc_get_clock(MXC_I2C_CLK);
	div = (i2c_clk_rate + rate - 1) / rate;
	if (div < i2c_clk_div[0][0])
		clk_div = 0;
	else if (div > i2c_clk_div[ARRAY_SIZE(i2c_clk_div) - 1][0])
		clk_div = ARRAY_SIZE(i2c_clk_div) - 1;
	else
		for (clk_div = 0; i2c_clk_div[clk_div][0] < div; clk_div++)
			;

	/* Store divider value */
	return clk_div;
}

/*
 * Set I2C Bus speed
 */
static int bus_i2c_set_bus_speed(u32 base, int speed)
{
	volatile struct mxc_i2c_regs *i2c_regs = (struct mxc_i2c_regs *)base;
	u8 clk_idx = i2c_imx_get_clk(speed);
	u8 idx = i2c_clk_div[clk_idx][1];

	/* Store divider value */
	writeb(idx, &i2c_regs->ifdr);

	/* Reset module */
	writeb(I2CR_IDIS, &i2c_regs->i2cr);
	writeb(0, &i2c_regs->i2sr);
	return 0;
}

#define ST_BUS_IDLE (0 | (I2SR_IBB << 8))
#define ST_BUS_BUSY (I2SR_IBB | (I2SR_IBB << 8))
#define ST_IIF (I2SR_IIF | (I2SR_IIF << 8))

static int wait_for_sr_state(volatile struct mxc_i2c_regs *i2c_regs, unsigned state)
{
	unsigned sr;
	ulong elapsed;
	ulong start_time = get_timer(0);
	for (;;) {
		sr = readb(&i2c_regs->i2sr);
		if (sr & I2SR_IAL) {
#ifdef I2C_QUIRK_REG
			writeb(sr | I2SR_IAL, &i2c_regs->i2sr);
#else
			writeb(sr & ~I2SR_IAL, &i2c_regs->i2sr);
#endif
			rt_kprintf("%s: Arbitration lost sr=%x cr=%x state=%x\n",
				__func__, sr, readb(&i2c_regs->i2cr), state);
			return -EBADMSG;
		}
		if ((sr & (state >> 8)) == (unsigned char)state)
			return sr;
		elapsed = get_timer(start_time);
		if (elapsed > (CONFIG_SYS_HZ / 10))	/* .1 seconds */
			break;
	}
	rt_kprintf("%s: failed sr=%x cr=%x state=%x\n", __func__,
			sr, readb(&i2c_regs->i2cr), state);
	return -ETIMEDOUT;
}

static int tx_byte(volatile struct mxc_i2c_regs *i2c_regs, u8 byte)
{
	int ret;

	writeb(I2SR_IIF_CLEAR, &i2c_regs->i2sr);
	writeb(byte, &i2c_regs->i2dr);
	ret = wait_for_sr_state(i2c_regs, ST_IIF);
	if (ret < 0)
		return ret;
	if (ret & I2SR_RX_NO_AK)
		return -ENODEV;
	return 0;
}

/*
 * Stop I2C transaction
 */
static void i2c_imx_stop(volatile struct mxc_i2c_regs *i2c_regs)
{
	int ret;
	unsigned int temp = readb(&i2c_regs->i2cr);

	temp &= ~(I2CR_MSTA | I2CR_MTX);
	writeb(temp, &i2c_regs->i2cr);
	ret = wait_for_sr_state(i2c_regs, ST_BUS_IDLE);
	if (ret < 0)
		rt_kprintf("%s:trigger stop failed\n", __func__);
}

/*
 * Stub implementations for outer i2c slave operations
 * Any board has special requirement (i.mx6slevk) can
 * overwrite the function
 */
void __i2c_force_reset_slave(void)
{
}
void i2c_force_reset_slave(void)
	__attribute__((weak, alias("__i2c_force_reset_slave")));

/*
 * Send start signal, chip address and
 * write register address
 */
static int i2c_init_transfer_(volatile struct mxc_i2c_regs *i2c_regs,
		uchar chip, uint addr, int alen)
{
	unsigned int temp;
	int ret;

	/* Reset i2c slave */
	i2c_force_reset_slave();

	/* Enable I2C controller */
#ifdef I2C_QUIRK_REG
	if (readb(&i2c_regs->i2cr) & I2CR_IDIS) {
#else
	if (!(readb(&i2c_regs->i2cr) & I2CR_IEN)) {
#endif
		writeb(I2CR_IEN, &i2c_regs->i2cr);
		/* Wait for controller to be stable */
		udelay(50);
	}
	if (readb(&i2c_regs->iadr) == (chip << 1))
		writeb((chip << 1) ^ 2, &i2c_regs->iadr);
	writeb(I2SR_IIF_CLEAR, &i2c_regs->i2sr);
	ret = wait_for_sr_state(i2c_regs, ST_BUS_IDLE);
	if (ret < 0)
		return ret;

	/* Start I2C transaction */
	temp = readb(&i2c_regs->i2cr);
	temp |= I2CR_MSTA;
	writeb(temp, &i2c_regs->i2cr);

	ret = wait_for_sr_state(i2c_regs, ST_BUS_BUSY);
	if (ret < 0)
		return ret;

	temp |= I2CR_MTX | I2CR_TX_NO_AK;
	writeb(temp, &i2c_regs->i2cr);

	/* write slave address */
	ret = tx_byte(i2c_regs, chip << 1);
	if (ret < 0)
		return ret;

	while (alen--) {
		ret = tx_byte(i2c_regs, (addr >> (alen * 8)) & 0xff);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int i2c_idle_bus(u32 iobase);
static int i2c_init_transfer(volatile struct mxc_i2c_regs *i2c_regs,
		uchar chip, uint addr, int alen)
{
	int retry;
	int ret;
	for (retry = 0; retry < 3; retry++) {
		ret = i2c_init_transfer_(i2c_regs, chip, addr, alen);
		if (ret >= 0)
			return 0;
		i2c_imx_stop(i2c_regs);
		if (ret == -ENODEV)
			return ret;

		rt_kprintf("%s: failed for chip 0x%x retry=%d\n", __func__, chip,
				retry);
		if (ret != -EBADMSG)
			/* Disable controller */
			writeb(I2CR_IDIS, &i2c_regs->i2cr);
		udelay(100);
		if (i2c_idle_bus((u32)i2c_regs) < 0)
			break;
	}
	rt_kprintf("%s: give up i2c_regs=%p\n", __func__, i2c_regs);
	return ret;
}

/*
 * Read data from I2C device
 */
int bus_i2c_read(void *base, uchar chip, uint addr, int alen, uchar *buf,
		int len)
{
	int ret;
	unsigned int temp;
	int i;
	volatile struct mxc_i2c_regs *i2c_regs = (struct mxc_i2c_regs *)base;

	ret = i2c_init_transfer(i2c_regs, chip, addr, alen);
	if (ret < 0)
		return ret;

	temp = readb(&i2c_regs->i2cr);
	temp |= I2CR_RSTA;
	writeb(temp, &i2c_regs->i2cr);

	ret = tx_byte(i2c_regs, (chip << 1) | 1);
	if (ret < 0) {
		i2c_imx_stop(i2c_regs);
		return ret;
	}

	/* setup bus to read data */
	temp = readb(&i2c_regs->i2cr);
	temp &= ~(I2CR_MTX | I2CR_TX_NO_AK);
	if (len == 1)
		temp |= I2CR_TX_NO_AK;
	writeb(temp, &i2c_regs->i2cr);
	writeb(I2SR_IIF_CLEAR, &i2c_regs->i2sr);
	readb(&i2c_regs->i2dr);		/* dummy read to clear ICF */

	/* read data */
	for (i = 0; i < len; i++) {
		ret = wait_for_sr_state(i2c_regs, ST_IIF);
		if (ret < 0) {
			i2c_imx_stop(i2c_regs);
			return ret;
		}

		/*
		 * It must generate STOP before read I2DR to prevent
		 * controller from generating another clock cycle
		 */
		if (i == (len - 1)) {
			i2c_imx_stop(i2c_regs);
		} else if (i == (len - 2)) {
			temp = readb(&i2c_regs->i2cr);
			temp |= I2CR_TX_NO_AK;
			writeb(temp, &i2c_regs->i2cr);
		}
		writeb(I2SR_IIF_CLEAR, &i2c_regs->i2sr);
		buf[i] = readb(&i2c_regs->i2dr);
	}
	i2c_imx_stop(i2c_regs);
	return len;
}

/*
 * Write data to I2C device
 */
int bus_i2c_write(void *base, uchar chip, uint addr, int alen,
		const uchar *buf, int len)
{
	int ret;
	int i;
	volatile struct mxc_i2c_regs *i2c_regs = (struct mxc_i2c_regs *)base;

	ret = i2c_init_transfer(i2c_regs, chip, addr, alen);
	if (ret < 0)
		return ret;

	for (i = 0; i < len; i++) {
		ret = tx_byte(i2c_regs, buf[i]);
		if (ret < 0)
			break;
	}
	i2c_imx_stop(i2c_regs);
	return (ret < 0)?ret:i;
}

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

static int i2c_idle_bus(u32 iobase)
{
	if (i2c0.iobase == iobase)
		return force_idle_bus(i2c0.pad);
	return 0;
}

static void gpio_set_sda(void *data, rt_int32_t state)
{
    gpio_direction_output(I2C_SDA, state);
}
static void gpio_set_scl(void *data, rt_int32_t state)
{
    gpio_direction_output(I2C_SDL, state);
}
static rt_int32_t gpio_get_sda(void *data)
{
    gpio_direction_input(I2C_SDA);
    return gpio_get_value(I2C_SDA);
}
static rt_int32_t gpio_get_scl(void *data)
{
    gpio_direction_input(I2C_SDL);
    return gpio_get_value(I2C_SDL);
}
static void gpio_udelay(rt_uint32_t us)
{
    udelay(us);
}
static const struct rt_i2c_bit_ops _i2c_bit_ops =
{
    NULL,
    gpio_set_sda,
    gpio_set_scl,
    gpio_get_sda,
    gpio_get_scl,
    gpio_udelay,
    5,
    100
};
struct rt_i2c_bus_device bit_i2c;

void rt_hw_i2c_init(void)
{
#if 0
	enable_i2c_clk(true, i2c0.index);
	force_idle_bus(i2c0.pad);
	bus_i2c_set_bus_speed(i2c0.iobase, CONFIG_SYS_I2C_SPEED);
#else
	gpio_direction_input(I2C_SDA);
	gpio_direction_input(I2C_SDL);
	imx_iomux_v3_setup_pad(MX6_PAD_SNVS_TAMPER8__GPIO5_IO08 | MUX_PAD_CTRL(I2C_PAD_CTRL));
	imx_iomux_v3_setup_pad(MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(I2C_PAD_CTRL));

	rt_memset(&bit_i2c, 0, sizeof(struct rt_i2c_bus_device));
	bit_i2c.priv = (void *)&_i2c_bit_ops;
	rt_i2c_bit_add_bus(&bit_i2c, "i2c");
#endif
}

rt_uint8_t i2c_reg_read(rt_uint8_t index, rt_uint8_t addr, rt_uint8_t reg)
{
	rt_uint8_t val = 0;
	if (index == 0)
	{
#if 0
		if (bus_i2c_read((void *)i2c0.iobase, addr, reg, 1, &val, 1) != 1)
			return 0;
#else
		if (rt_i2c_master_send(&bit_i2c, addr, 0, &reg, 1) != 1)
			return 0;
		if (rt_i2c_master_recv(&bit_i2c, addr, 0, &val, 1) != 1)
			return 0;
#endif
	}
	return val;
}
void i2c_reg_write(rt_uint8_t index, rt_uint8_t addr, rt_uint8_t reg, rt_uint8_t val)
{
	if (index == 0)
	{
#if 0
		if (bus_i2c_write((void *)i2c0.iobase, addr, reg, 1, &val, 1) != 1)
			return;
#else
		if (rt_i2c_master_send(&bit_i2c, addr, 0, &reg, 1) != 1)
			return;
		if (rt_i2c_master_send(&bit_i2c, addr, 0, &val, 1) != 1)
			return;
#endif
	}
}

