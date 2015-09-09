#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <rtgui/rtgui.h>
#include <rtgui/event.h>
#include <rtgui/touch.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include "board.h"
#include <lradc.h>

struct mxs_ts_info {
	unsigned int base;
	u8 x_plus_chan;
	u8 x_minus_chan;
	u8 y_plus_chan;
	u8 y_minus_chan;

	unsigned int x_plus_val;
	unsigned int x_minus_val;
	unsigned int y_plus_val;
	unsigned int y_minus_val;
	unsigned int x_plus_mask;
	unsigned int x_minus_mask;
	unsigned int y_plus_mask;
	unsigned int y_minus_mask;

	enum {
		TS_STATE_DISABLED,
		TS_STATE_TOUCH_DETECT,
		TS_STATE_TOUCH_VERIFY,
		TS_STATE_X_PLANE,
		TS_STATE_Y_PLANE,
	} state;
	u16 x;
	u16 y;
	int sample_count;
};

static struct mxs_ts_info mx28_ts_info = {
    .base = REGS_LRADC_BASE,
	.x_plus_chan = LRADC_TOUCH_X_PLUS,
	.x_minus_chan = LRADC_TOUCH_X_MINUS,
	.y_plus_chan = LRADC_TOUCH_Y_PLUS,
	.y_minus_chan = LRADC_TOUCH_Y_MINUS,
	.x_plus_val = BM_LRADC_CTRL0_XPULSW,
	.x_minus_val = BF_LRADC_CTRL0_XNURSW(2),
	.y_plus_val = BF_LRADC_CTRL0_YPLLSW(1),
	.y_minus_val = BM_LRADC_CTRL0_YNLRSW,
	.x_plus_mask = BM_LRADC_CTRL0_XPULSW,
	.x_minus_mask = BM_LRADC_CTRL0_XNURSW,
	.y_plus_mask = BM_LRADC_CTRL0_YPLLSW,
	.y_minus_mask = BM_LRADC_CTRL0_YNLRSW,
};

static inline void enter_state_touch_detect(struct mxs_ts_info *info)
{
	writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->x_plus_chan));
	writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->y_plus_chan));
	writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->x_minus_chan));
	writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->y_minus_chan));

	writel(BM_LRADC_CTRL1_LRADC0_IRQ << info->y_minus_chan,
		     info->base + HW_LRADC_CTRL1_CLR);
	writel(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ,
		     info->base + HW_LRADC_CTRL1_CLR);
	/*
	 * turn off the yplus and yminus pullup and pulldown, and turn off touch
	 * detect (enables yminus, and xplus through a resistor.On a press,
	 * xplus is pulled down)
	 */
	writel(info->y_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->y_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->x_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->x_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE,//touch detect enable
		     info->base + HW_LRADC_CTRL0_SET);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 0);
	info->state = TS_STATE_TOUCH_DETECT;
	info->sample_count = 0;
}

static inline void enter_state_disabled(struct mxs_ts_info *info)
{
	writel(info->y_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->y_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->x_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->x_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE,
		     info->base + HW_LRADC_CTRL0_CLR);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 0);
	info->state = TS_STATE_DISABLED;
	info->sample_count = 0;
}

static inline void enter_state_x_plane(struct mxs_ts_info *info)
{
	writel(info->y_plus_val, info->base + HW_LRADC_CTRL0_SET);
	writel(info->y_minus_val, info->base + HW_LRADC_CTRL0_SET);
	writel(info->x_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->x_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE,
		     info->base + HW_LRADC_CTRL0_CLR);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);

	info->state = TS_STATE_X_PLANE;
	info->sample_count = 0;
}

static inline void enter_state_y_plane(struct mxs_ts_info *info)
{
	writel(info->y_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->y_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->x_plus_val, info->base + HW_LRADC_CTRL0_SET);
	writel(info->x_minus_val, info->base + HW_LRADC_CTRL0_SET);
	writel(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE,
		     info->base + HW_LRADC_CTRL0_CLR);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
	info->state = TS_STATE_Y_PLANE;
	info->sample_count = 0;
}

static inline void enter_state_touch_verify(struct mxs_ts_info *info)
{
	writel(info->y_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->y_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->x_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(info->x_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	writel(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE,
		     info->base + HW_LRADC_CTRL0_SET);
	info->state = TS_STATE_TOUCH_VERIFY;
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
	info->sample_count = 0;
}

#define TOUCH_DEBOUNCE_TOLERANCE	100

static void process_lradc(struct mxs_ts_info *info, u16 x, u16 y,
			int pressure)
{
	switch (info->state) {
	case TS_STATE_X_PLANE:
		if (info->sample_count < 2) {
			info->x = x;
			info->sample_count++;
		} else {
			if (abs(info->x - x) > TOUCH_DEBOUNCE_TOLERANCE)
				info->sample_count = 1;
			else {
				u16 x_c = info->x * (info->sample_count - 1);
				info->x = (x_c + x) / info->sample_count;
				info->sample_count++;
			}
		}
		if (info->sample_count > 4){
			enter_state_y_plane(info);
		}
		else
			hw_lradc_set_delay_trigger_kick(
					LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
		break;

	case TS_STATE_Y_PLANE:
		if (info->sample_count < 2) {
			info->y = y;
			info->sample_count++;
		} else {
			if (abs(info->y - y) > TOUCH_DEBOUNCE_TOLERANCE)
				info->sample_count = 1;
			else {
				u16 y_c = info->y * (info->sample_count - 1);
				info->y = (y_c + y) / info->sample_count;
				info->sample_count++;
			}
		}
		if (info->sample_count > 4) {
			enter_state_touch_verify(info);
		}
		else
			hw_lradc_set_delay_trigger_kick(
					LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
		break;

	case TS_STATE_TOUCH_VERIFY:
	case TS_STATE_TOUCH_DETECT:
		if (pressure) {
			enter_state_x_plane(info);
			hw_lradc_set_delay_trigger_kick(
					LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
		} else
			enter_state_touch_detect(info);
		break;
        
    default:
        break;
	}
}

static void rt_hw_touch_handler(int irq, void *param)
{
	struct mxs_ts_info *info = param;
	u16 x_plus, y_plus;
	int pressure = 0;

	if (irq == IRQ_LRADC_TOUCH){
		writel(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ,
			     info->base + HW_LRADC_CTRL1_CLR);
	}

	else if (irq == IRQ_LRADC_CH5){ 
		writel(BM_LRADC_CTRL1_LRADC0_IRQ << info->y_minus_chan,
			     info->base + HW_LRADC_CTRL1_CLR);
	}
	/* get x, y values */
	x_plus = readl(info->base + HW_LRADC_CHn(info->x_plus_chan)) &
		BM_LRADC_CHn_VALUE;
	y_plus = readl(info->base + HW_LRADC_CHn(info->y_plus_chan)) &
		BM_LRADC_CHn_VALUE;

	if (readl(info->base + HW_LRADC_STATUS) &
	    BM_LRADC_STATUS_TOUCH_DETECT_RAW)
		pressure = 1;

	process_lradc(info, x_plus, y_plus, pressure);//y_plus is x zuobiao , x_plus is y zuobiao
}

#define IS_TOUCH_UP() 0
#define touch_int_enable(x)

struct rt_event touch_event;
static void touch_thread_entry(void *parameter)
{
    rt_bool_t touch_down = RT_FALSE;
    rt_uint32_t event_value;

    while (1)
    {
        if (rt_event_recv(&touch_event,
                          1,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER,
                          &event_value)
                == RT_EOK)
        {
            while (1)
            {
                if (IS_TOUCH_UP())
                {
                    if (touch_down != RT_TRUE)
                    {
                        touch_int_enable(RT_TRUE);
                        break;
                    }

                    //rtgui_touch_post(RTGUI_TOUCH_UP, touch->x, touch->y);

                    /* clean */
                    touch_down = RT_FALSE;
                    touch_int_enable(RT_TRUE);
                    break;
                } /* touch up */
                else /* touch down or move */
                {
                    int type = RTGUI_TOUCH_DOWN;
                    if (touch_down == RT_FALSE)
                    {
                        rt_thread_delay(RT_TICK_PER_SECOND / 10);
                    }
                    else
                    {
                        rt_thread_delay(RT_TICK_PER_SECOND / 20);
                        /* touch motion event */
                        type = RTGUI_TOUCH_MOTION;
                    }

                    /* check it again */
                    if (IS_TOUCH_UP()) continue;

                    /* calculation */
                    //rtgui_touch_post(type, touch->x, touch->y);

                    touch_down = RT_TRUE;
                } /* touch down or move */
            } /* read touch */
        } /* event recv */
    } /* thread while(1) */
}

extern void hw_lradc_reinit(int enable_ground_ref, unsigned freq);
int touch_init(void)
{
    struct rt_thread *touch_thread;
    struct mxs_ts_info *info = &mx28_ts_info;
    
    hw_lradc_reinit(0, LRADC_CLOCK_6MHZ);
	enter_state_touch_detect(info);
	
	hw_lradc_use_channel(info->x_plus_chan);
	hw_lradc_use_channel(info->x_minus_chan);
	hw_lradc_use_channel(info->y_plus_chan);
	hw_lradc_use_channel(info->y_minus_chan);
	hw_lradc_configure_channel(info->x_plus_chan, 0, 0, 0);
	hw_lradc_configure_channel(info->x_minus_chan, 0, 0, 0);
	hw_lradc_configure_channel(info->y_plus_chan, 0, 0, 0);
	hw_lradc_configure_channel(info->y_minus_chan, 0, 0, 0);

	/* Clear the accumulator & NUM_SAMPLES for the channels */
	writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->x_plus_chan));
	writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->x_minus_chan));
	writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->y_plus_chan));
	writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->y_minus_chan));

	hw_lradc_set_delay_trigger(LRADC_DELAY_TRIGGER_TOUCHSCREEN,
			0x3c, 0, 0,8); //8/2 ms trigger

	writel(BM_LRADC_CTRL1_LRADC0_IRQ << info->y_minus_chan,
		     info->base + HW_LRADC_CTRL1_CLR); // clear ch5  iqr pending bit
	writel(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ,  // clear touch detect  iqr pending bit
		     info->base + HW_LRADC_CTRL1_CLR);

	writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << info->y_minus_chan,
		     info->base + HW_LRADC_CTRL1_SET);  //ch5 irq enable
	writel(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ_EN,//touch detect irq enbale
		     info->base + HW_LRADC_CTRL1_SET);
    
    rt_event_init(&touch_event, "touch", RT_IPC_FLAG_FIFO);
    touch_thread = rt_thread_create("touch",
                                    touch_thread_entry, RT_NULL,
                                    1024, 20, 10);
    if (touch_thread != RT_NULL) rt_thread_startup(touch_thread);
    
    rt_hw_interrupt_install(IRQ_LRADC_TOUCH, rt_hw_touch_handler, info, "Touch");
    rt_hw_interrupt_umask(IRQ_LRADC_TOUCH);
    rt_hw_interrupt_install(IRQ_LRADC_CH5, rt_hw_touch_handler, info, "ADC5");
    rt_hw_interrupt_umask(IRQ_LRADC_CH5);

    return RT_EOK;
}