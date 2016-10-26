#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <rtgui/rtgui.h>
#include <rtgui/event.h>
#include <rtgui/touch.h>
#include <rtgui/calibration.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include "board.h"

#undef ALIGN
#include <common.h>
#include <malloc.h>
#include <asm/io.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>

/* ADC configuration registers field define */
#define ADC_AIEN		(0x1 << 7)
#define ADC_CONV_DISABLE	0x1F
#define ADC_CAL			(0x1 << 7)
#define ADC_CALF		0x2
#define ADC_12BIT_MODE		(0x2 << 2)
#define ADC_IPG_CLK		0x00
#define ADC_CLK_DIV_8		(0x03 << 5)
#define ADC_SHORT_SAMPLE_MODE	(0x0 << 4)
#define ADC_HARDWARE_TRIGGER	(0x1 << 13)
#define SELECT_CHANNEL_4	0x04
#define SELECT_CHANNEL_1	0x01
#define DISABLE_CONVERSION_INT	(0x0 << 7)

/* ADC registers */
#define REG_ADC_HC0		0x00
#define REG_ADC_HC1		0x04
#define REG_ADC_HC2		0x08
#define REG_ADC_HC3		0x0C
#define REG_ADC_HC4		0x10
#define REG_ADC_HS		0x14
#define REG_ADC_R0		0x18
#define REG_ADC_CFG		0x2C
#define REG_ADC_GC		0x30
#define REG_ADC_GS		0x34

#define ADC_TIMEOUT		100

/* TSC registers */
#define REG_TSC_BASIC_SETING	0x00
#define REG_TSC_PRE_CHARGE_TIME	0x10
#define REG_TSC_FLOW_CONTROL	0x20
#define REG_TSC_MEASURE_VALUE	0x30
#define REG_TSC_INT_EN		0x40
#define REG_TSC_INT_SIG_EN	0x50
#define REG_TSC_INT_STATUS	0x60
#define REG_TSC_DEBUG_MODE	0x70
#define REG_TSC_DEBUG_MODE2	0x80

/* TSC configuration registers field define */
#define DETECT_4_WIRE_MODE	(0x0 << 4)
#define AUTO_MEASURE		0x1
#define MEASURE_SIGNAL		0x1
#define DETECT_SIGNAL		(0x1 << 4)
#define VALID_SIGNAL		(0x1 << 8)
#define MEASURE_INT_EN		0x1
#define MEASURE_SIG_EN		0x1
#define VALID_SIG_EN		(0x1 << 8)
#define DE_GLITCH_2		(0x2 << 29)
#define START_SENSE		(0x1 << 12)
#define TSC_DISABLE		(0x1 << 16)
#define DETECT_MODE		0x2

struct mxs_ts_info {
    struct rt_device parent;

	u32 tsc_regs;
	u32 adc_regs;
	int value;
	iomux_v3_cfg_t xnur_gpio;
	struct rt_event tsc_evt;
	int measure_delay_time;
	int pre_charge_time;
};

static struct mxs_ts_info imx6ul_ts_info = {
	.tsc_regs = 0x02040000,
	.adc_regs = 0x0219c000,
	.measure_delay_time = 0xffff,
	.pre_charge_time = 0xfff,
	.xnur_gpio = IMX_GPIO_NR(1, 3),
};

#define ABS_X 0
#define ABS_Y 1
#define BTN_TOUCH 2
static rt_tick_t tsc_last;
static int touch_data[4];
static inline void input_report_key(int type, int data) { touch_data[type] = data; }
static inline void input_report_abs(int type, int data) { touch_data[type] = data; }
static void input_sync()
{
    int x,y;
    //计算坐标位置
    x = touch_data[ABS_X];
    y = touch_data[ABS_Y];
    //按压状态发生了变化，只在变化后发送，
    if (touch_data[BTN_TOUCH] != touch_data[BTN_TOUCH+1])
    {
        rtgui_touch_post((touch_data[BTN_TOUCH]?RTGUI_TOUCH_DOWN:RTGUI_TOUCH_UP), x, y);
        touch_data[BTN_TOUCH+1] = touch_data[BTN_TOUCH];
        tsc_last = rt_tick_get();
    }
    //按压状态才发送移动消息，不是鼠标
    else if (touch_data[BTN_TOUCH])
    {
        if (rt_tick_get() - tsc_last > 100)
        {
            rtgui_touch_post(RTGUI_TOUCH_MOTION, x, y);
            tsc_last = rt_tick_get();
        }
    }
}

static volatile int tsc_status;
static void rt_hw_touch_handler(void *param)
{
	struct mxs_ts_info *tsc = param;
	int x, y;
	int xnur;
	int debug_mode2;
	int state_machine;
	int start;
	unsigned long timeout;

	while (1) {
	rt_event_recv(&tsc->tsc_evt,1 ,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR ,RT_WAITING_FOREVER, 0);

	/* It's a HW self-clean bit. Set this bit and start sense detection */
	start = readl(tsc->tsc_regs + REG_TSC_FLOW_CONTROL);
	start |= START_SENSE;
	writel(start, tsc->tsc_regs + REG_TSC_FLOW_CONTROL);

	if (tsc_status & MEASURE_SIGNAL) {
		tsc->value = readl(tsc->tsc_regs + REG_TSC_MEASURE_VALUE);
		x = (tsc->value >> 16) & 0x0fff;
		y = tsc->value & 0x0fff;

		/*
		 * Delay some time(max 2ms), wait the pre-charge done.
		 * After the pre-change mode, TSC go into detect mode.
		 * And in detect mode, we can get the xnur gpio value.
		 * If xnur is low, this means the touch screen still
		 * be touched. If xnur is high, this means finger leave
		 * the touch screen.
		 */
		timeout = 0;
		do {
			if (++timeout >= 10) {
				xnur = 0;
				goto touch_event;
			}
			udelay(200);
			debug_mode2 = readl(tsc->tsc_regs + REG_TSC_DEBUG_MODE2);
			state_machine = (debug_mode2 >> 20) & 0x7;
		} while (state_machine != DETECT_MODE);
		udelay(200);

		xnur = gpio_get_value(tsc->xnur_gpio);
touch_event:
		if (xnur == 0) {
			input_report_key(BTN_TOUCH, 1);
			input_report_abs(ABS_X, x);
			input_report_abs(ABS_Y, y);
		} else {
			input_report_key(BTN_TOUCH, 0);
		}
		input_sync();
	}
	rt_thread_delay(10);}
}
static void rt_hw_adc_handler(int irq, void *param)
{
	struct mxs_ts_info *tsc = param;
	int coco;
	int value;

	if (irq == IMX_INT_TSC)
	{
		tsc_status = readl(tsc->tsc_regs + REG_TSC_INT_STATUS);

		/* write 1 to clear the bit measure-signal */
		writel(MEASURE_SIGNAL | DETECT_SIGNAL,
			tsc->tsc_regs + REG_TSC_INT_STATUS);

		rt_event_send(&tsc->tsc_evt, 1);
		return;
	}
	coco = readl(tsc->adc_regs + REG_ADC_HS);
	if (coco & 0x01)
		value = readl(tsc->adc_regs + REG_ADC_R0);
}

static rt_err_t touch_device_init(rt_device_t dev)
{
    return RT_EOK;
}
static rt_bool_t calibration_after(calculate_data_t*cal)
{
    FILE *file;
    if ((file = fopen("/etc/pointercal", "w")) != NULL)
    {
        fprintf(file,"%d %d %d %d %d %d %d",cal->x_coord[0],cal->x_coord[1],cal->x_coord[2],
                cal->y_coord[0],cal->y_coord[1],cal->y_coord[2],cal->scaling);
        fclose(file);
        return RT_TRUE;
    }
    return RT_FALSE;
}

/*
 * TSC module need ADC to get the measure value. So
 * before config TSC, we should initialize ADC module.
 */
static void imx6ul_adc_init(struct mxs_ts_info *tsc)
{
	int adc_hc = 0;
	int adc_gc;
	int adc_gs;
	int adc_cfg;
	int timeout = ADC_TIMEOUT;

	adc_cfg = readl(tsc->adc_regs + REG_ADC_CFG);
	adc_cfg |= ADC_12BIT_MODE | ADC_IPG_CLK;
	adc_cfg |= ADC_CLK_DIV_8 | ADC_SHORT_SAMPLE_MODE;
	adc_cfg &= ~ADC_HARDWARE_TRIGGER;
	writel(adc_cfg, tsc->adc_regs + REG_ADC_CFG);

	/* enable calibration interrupt */
	adc_hc |= ADC_AIEN;
	adc_hc |= ADC_CONV_DISABLE;
	writel(adc_hc, tsc->adc_regs + REG_ADC_HC0);

	/* start ADC calibration */
	adc_gc = readl(tsc->adc_regs + REG_ADC_GC);
	adc_gc |= ADC_CAL;
	writel(adc_gc, tsc->adc_regs + REG_ADC_GC);

	while (!(readl(tsc->adc_regs + REG_ADC_HS)&0x01) && --timeout)
		udelay(10);
	if (timeout)
		readl(tsc->adc_regs + REG_ADC_R0);
	else
		rt_kprintf("Timeout for adc calibration\n");

	adc_gs = readl(tsc->adc_regs + REG_ADC_GS);
	if (adc_gs & ADC_CALF)
		rt_kprintf("ADC calibration failed\n");

	/* TSC need the ADC work in hardware trigger */
	adc_cfg = readl(tsc->adc_regs + REG_ADC_CFG);
	adc_cfg |= ADC_HARDWARE_TRIGGER;
	writel(adc_cfg, tsc->adc_regs + REG_ADC_CFG);
}

/*
 * This is a TSC workaround. Currently TSC misconnect two
 * ADC channels, this function remap channel configure for
 * hardware trigger.
 */
static void imx6ul_tsc_channel_config(struct mxs_ts_info *tsc)
{
	int adc_hc0, adc_hc1, adc_hc2, adc_hc3, adc_hc4;

	adc_hc0 = DISABLE_CONVERSION_INT;
	writel(adc_hc0, tsc->adc_regs + REG_ADC_HC0);

	adc_hc1 = DISABLE_CONVERSION_INT | SELECT_CHANNEL_4;
	writel(adc_hc1, tsc->adc_regs + REG_ADC_HC1);

	adc_hc2 = DISABLE_CONVERSION_INT;
	writel(adc_hc2, tsc->adc_regs + REG_ADC_HC2);

	adc_hc3 = DISABLE_CONVERSION_INT | SELECT_CHANNEL_1;
	writel(adc_hc3, tsc->adc_regs + REG_ADC_HC3);

	adc_hc4 = DISABLE_CONVERSION_INT;
	writel(adc_hc4, tsc->adc_regs + REG_ADC_HC4);
}

/*
 * TSC setting, confige the pre-charge time and measure delay time.
 * different touch screen may need different pre-charge time and
 * measure delay time.
 */
static void imx6ul_tsc_set(struct mxs_ts_info *tsc)
{
	int basic_setting = 0;
	int start;

	basic_setting |= tsc->measure_delay_time << 8;
	basic_setting |= DETECT_4_WIRE_MODE | AUTO_MEASURE;
	writel(basic_setting, tsc->tsc_regs + REG_TSC_BASIC_SETING);

	writel(DE_GLITCH_2, tsc->tsc_regs + REG_TSC_DEBUG_MODE2);

	writel(tsc->pre_charge_time, tsc->tsc_regs + REG_TSC_PRE_CHARGE_TIME);
	writel(MEASURE_INT_EN, tsc->tsc_regs + REG_TSC_INT_EN);
	writel(MEASURE_SIG_EN | VALID_SIG_EN,
		tsc->tsc_regs + REG_TSC_INT_SIG_EN);

	/* start sense detection */
	start = readl(tsc->tsc_regs + REG_TSC_FLOW_CONTROL);
	start |= START_SENSE;
	start &= ~TSC_DISABLE;
	writel(start, tsc->tsc_regs + REG_TSC_FLOW_CONTROL);
}

#define TS_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

iomux_v3_cfg_t const ts_pads[] = {
	MX6_PAD_GPIO1_IO01__GPIO1_IO01 | MUX_PAD_CTRL(TS_PAD_CTRL),
	MX6_PAD_GPIO1_IO02__GPIO1_IO02 | MUX_PAD_CTRL(TS_PAD_CTRL),
	MX6_PAD_GPIO1_IO03__GPIO1_IO03 | MUX_PAD_CTRL(TS_PAD_CTRL),
	MX6_PAD_GPIO1_IO04__GPIO1_IO04 | MUX_PAD_CTRL(TS_PAD_CTRL),
};

int touch_init(void)
{
    struct mxs_ts_info *info = &imx6ul_ts_info;

	rt_event_init(&imx6ul_ts_info.tsc_evt,"tsc",0);
	imx_iomux_v3_setup_multiple_pads(ts_pads, ARRAY_SIZE(ts_pads));

	imx6ul_adc_init(info);
	imx6ul_tsc_channel_config(info);
	imx6ul_tsc_set(info);

    info->parent.type = RT_Device_Class_Miscellaneous;
    info->parent.init = touch_device_init;
    info->parent.control = RT_NULL;
    info->parent.user_data = RT_NULL;

    /* register touch device to RT-Thread */
    rt_device_register(&(info->parent), "touch", RT_DEVICE_FLAG_RDWR);
    rtgui_touch_init(calibration_get_ops());
    calibration_set_after(calibration_after);
    //读取校验数据
    FILE *file;
    if ((file = fopen("/etc/pointercal", "r")) != NULL)
    {
        calculate_data_t data;
        if (fscanf(file,"%d %d %d %d %d %d %d",&data.x_coord[0],&data.x_coord[1],&data.x_coord[2],
            &data.y_coord[0],&data.y_coord[1],&data.y_coord[2],&data.scaling) == 7)
        {
            calibration_set_data(&data);
            rt_kprintf("Load pointercal Ok\n");
        }
        else
        {
            calibration_init(RT_NULL);
            rt_kprintf("Load pointercal Fail\n");
        }
        fclose(file);
    }
    else
    {
        calibration_init(RT_NULL);
        rt_kprintf("Can't Open pointercal\n");
    }
    //启动检测线程
    {
        rt_thread_t tid;
        tid = rt_thread_create("tsc_mon",
                               rt_hw_touch_handler,
                               info,
                               512,
                               10,
                               20);
        if (tid != RT_NULL)
            rt_thread_startup(tid);
    }
    rt_hw_interrupt_install(IMX_INT_TSC, rt_hw_adc_handler, info, "Touch");
    rt_hw_interrupt_umask(IMX_INT_TSC);
    rt_hw_interrupt_install(IMX_INT_ADC2, rt_hw_adc_handler, info, "Adc2");
    rt_hw_interrupt_umask(IMX_INT_ADC2);

    return RT_EOK;
}
