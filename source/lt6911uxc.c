#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/camera_common.h>
#include <media/v4l2-common.h>
#ifdef CONFIG_V4L2_FWNODE
#include <media/v4l2-fwnode.h>
#else
#include <media/v4l2-of.h>
#endif
#include "lt6911uxe_regs.h"
#include <linux/delay.h>

/* v4l2 debug level */
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

/* custom v4l2 controls */
#define V4L2_CID_USER_LT6911UXE_BASE (V4L2_CID_USER_BASE + 0x1090)
#define LT6911UXE_CID_AUDIO_SAMPLING_RATE (V4L2_CID_USER_LT6911UXE_BASE + 1)
#define LT6911UXE_CID_AUDIO_PRESENT       (V4L2_CID_USER_LT6911UXE_BASE + 2)

/* v4l2 dv timings */
static struct v4l2_dv_timings default_timing = V4L2_DV_BT_CEA_1920X1080P60;

static const struct v4l2_dv_timings_cap lt6911uxe_timings_cap_4kp30 = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },
	/* Pixel clock from REF_01 p. 20. Min/max height/width are unknown */
	V4L2_INIT_BT_TIMINGS(
		160, 3840,				/* min/max width */
		120, 2160,				/* min/max height */
		25000000, 297000000,			/* min/max pixelclock */
		V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
		V4L2_DV_BT_STD_CVT,
		V4L2_DV_BT_CAP_PROGRESSIVE | V4L2_DV_BT_CAP_CUSTOM |
		V4L2_DV_BT_CAP_REDUCED_BLANKING)
};

/* Timings for LT6911UXE */
struct lt6911_timings {
	__u32 width;
	__u32 height;
	__u32 interlaced;
	__u32 htot;
	__u32 vtot;
	__u32 fps;
	__u64 pixelclock;
	__u32 standards;
	__u32 flags;
	bool change;
};

struct lt6911_timings lt6911uxe_timings;
struct lt6911_timings lt6911uxe_timings_old;

bool compare_timings(struct lt6911_timings *a, struct lt6911_timings *b) {
	if (a->width != b->width) return false;
	if (a->height != b->height) return false;
	if (a->htot != b->htot) return false;
	if (a->vtot != b->vtot) return false;
	if (a->fps != b->fps) return false;
	//if (a->pixelclock != b->pixelclock) return false;
	
	return true;
}

struct lt6911uxe_platform_data {
	/* GPIOs */
	int reset_gpio;
};

struct lt6911uxe_state {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct lt6911uxe_platform_data *pdata;
	struct media_pad pad[1];
	struct mutex lock;
	struct v4l2_ctrl_handler ctrl_handler;
	/* controls */
	u8 bank;		/* active reg-bank for I2C */
	bool enable_i2c;
	bool signal_present;
	/* expose audio capabilties */
	struct v4l2_ctrl *audio_sampling_rate_ctrl;
	struct v4l2_ctrl *audio_present_ctrl;
	/* timing / media format */
	struct v4l2_dv_timings timings;
	struct v4l2_dv_timings detected_timings;/* timings detected from phy */
	u32 mbus_fmt_code;			/* current media bus format */
};

static const struct camera_common_colorfmt lt6911uxe_color_fmts[] = {
	{ MEDIA_BUS_FMT_UYVY8_1X16,  V4L2_COLORSPACE_SRGB,  V4L2_PIX_FMT_UYVY},
};

static const struct v4l2_event lt6911uxe_ev_source_change = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static inline struct lt6911uxe_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lt6911uxe_state, sd);
}

/* ------ I2C --------------------------------------------------------------- */

static void lt6911uxe_reg_bank(struct v4l2_subdev *sd, u8 bank)
{
	struct lt6911uxe_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	struct i2c_msg msg;
	u8 data[2];
	u8 address;

	//dev_dbg(&client->dev, "bank!!");

	if(state->bank == bank) {
		//dev_dbg(&client->dev, "i2c: bank is 0x%02X\n",
		//bank);
		return;
	}
		
	dev_dbg(&client->dev, "i2c: change register bank to 0x%02X\n",
		bank);

	address = 0xFF;
	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2;
	msg.flags = 0;

	data[0] = address;
	data[1] = bank;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		dev_err(&client->dev, "%s: switch to bank 0x%x from 0x%x failed\n",
			__func__, bank, client->addr);
		return;
	}
	state->bank = bank;
}

static void lt6911uxe_i2c_wr8(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct lt6911uxe_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	struct i2c_msg msg;
	u8 data[2];
	u8 address;
	
	/* write register bank offset */
	//u8 bank = (reg >> 8) & 0xFF;
	//lt6911uxe_reg_bank(sd, bank);

	u8 bank = 0xE0;
	//lt6911uxe_reg_bank(sd, bank);

	//address = reg & 0xFF;
	address = reg;
	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2;
	msg.flags = 0;

	data[0] = address;
	data[1] = val;

	err = i2c_transfer(client->adapter, &msg, 1);

	if (err != 1) {
		dev_err(&client->dev, "%s: write register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
		return;
	}
	dev_dbg(&client->dev, "i2c: write register: 0x%02X = 0x%02X\n",
		reg, val);
}

static void lt6911uxe_i2c_rd(struct v4l2_subdev *sd, u8 reg, u8 *values, u32 n)
{
	struct lt6911uxe_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	//u8 reg_addr[1] = { (u8)(reg & 0xff) };  EE  80EE 
	//u8 bank_addr   = (u8)((reg >> 8) & 0xFF); 80

	u8 reg_addr[1] = { (u8)(reg) };
	u8 bank_addr   = { (u8)(0xff) }; /*no use?*/


	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,		/* write */
			.len = 1,
			.buf = reg_addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,	/* read n bytes */
			.len = n,
			.buf = values,
		},
	};	

	/* write register bank offset */
	//lt6911uxe_reg_bank(sd, bank_addr);

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: read register 0x%02X from 0x%x failed\n",
			__func__, reg, client->addr);
	}
}

static u8 lt6911uxe_i2c_rd8(struct v4l2_subdev *sd, u8 reg)
{
	u8 val;

	lt6911uxe_i2c_rd(sd, reg, &val, 1);

	//dev_dbg(sd->dev, "i2c: read 0x%02X = 0x%02X\n", reg, val);
	return val;
}

static u16 lt6911uxe_i2c_rd16(struct v4l2_subdev *sd, u16 reg)
{
	u16 val;
	u8 tmp;

	lt6911uxe_i2c_rd(sd, reg, (u8 *)&val, 2);
	/* high byte always at lower address -> swap */
	tmp = val & 0xFF;
	val = (val >> 8) | tmp << 8;

	//dev_dbg(sd->dev, "i2c: read 0x%04X = 0x%04X\n", reg, val);
	return val;
}

/* ------ STATUS / CTRL ----------------------------------------------------- */

static inline bool no_signal(struct v4l2_subdev *sd)
{
	struct lt6911uxe_state *state = to_state(sd);
	return !state->signal_present;
}

static void lt6911uxe_ext_control(struct v4l2_subdev *sd, bool enable)
{
	struct lt6911uxe_state *state = to_state(sd);

	if(state->enable_i2c == enable)
		return;

	state->enable_i2c = enable;
	if (enable) {
		dev_dbg(sd->dev, "%s(): enable external i2c control\n", __func__);
		//lt6911uxe_i2c_wr8(sd, SW_BANK, 0xE0);
		lt6911uxe_i2c_wr8(sd, ENABLE_I2C, 0x01);
		//lt6911uxe_i2c_wr8(sd, DISABLE_WD, 0x00);
	} else {
		dev_dbg(sd->dev, "%s(): disable external i2c control\n", __func__);
		//lt6911uxe_i2c_wr8(sd, SW_BANK, 0xE0);
		lt6911uxe_i2c_wr8(sd, ENABLE_I2C, 0x00);
	}
}

static int lt6911uxe_csi_enable(struct v4l2_subdev *sd, bool enable)
{
	//dev_dbg(sd->dev, "%s(): %d\n", __func__, enable);

	//lt6911uxe_ext_control(sd, true);
	if (enable) {
		dev_dbg(sd->dev, "%s(): enable csi\n", __func__);
		lt6911uxe_i2c_wr8(sd, MIPI_TX_CTRL, 0x01);
	} else {
		dev_dbg(sd->dev, "%s(): disable csi\n", __func__);
		lt6911uxe_i2c_wr8(sd, MIPI_TX_CTRL, 0x00);
	}
	//lt6911uxe_ext_control(sd, false);

	return 0;
}

static int lt6911uxe_get_audio_sampling_rate(struct lt6911uxe_state* state)
{
	int audio_fs, idx;
	static const int eps = 1500;
	static const int rates_default [] = {
		32000, 44100, 48000, 88200, 96000, 176400, 192000
	};
	int fs1, fs2;
	fs1 = lt6911uxe_i2c_rd8(&state->sd, Audio_FS_Value1);
	fs2 = lt6911uxe_i2c_rd8(&state->sd, Audio_FS_Value0);
	audio_fs = (fs1 << 8 | fs2) * 1000 ;
	//dev_dbg(&state->i2c_client->dev, "%s: Audio sample rate_H %d [Hz]\n",
	//	__func__, fs1);
	//dev_dbg(&state->i2c_client->dev, "%s: Audio sample rate_L %d [Hz]\n",
	//	__func__, fs2);

	//audio_fs = lt6911uxe_i2c_rd8(&state->sd, AUDIO_SR) * 1000;
	dev_dbg(&state->i2c_client->dev, "%s: Audio sample rate %d [Hz]\n",
	__func__, audio_fs);

	/* audio_fs is an approximation of sample rate - search nearest */
	for(idx = 0; idx < ARRAY_SIZE(rates_default); ++idx)
	{
		if ((rates_default[idx] - eps < audio_fs) &&
		    (rates_default[idx] + eps > audio_fs))
			return rates_default[idx];
	}
	dev_err(&state->i2c_client->dev, "%s: unhandled sampling rate_H %d [Hz]",
		__func__, audio_fs);

	return 0;
}

/* ------ TIMINGS ----------------------------------------------------------- */

 static int lt6911uxe_detect_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
 {
	struct v4l2_bt_timings *bt = &timings->bt;
	u8 width1, width0, height1, height0, fm2, fm1, fm0, h_htotal1, h_htotal0, vtotal1, vtotal0;
	u32 width, height, half_pixel_clk, htot, vtot, frame_interval, fps;

	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	memset(&lt6911uxe_timings, 0, sizeof(lt6911uxe_timings));

	if (no_signal(sd)) {
		v4l2_err(sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}

	timings->type  = V4L2_DV_BT_656_1120;
	bt->interlaced = V4L2_DV_PROGRESSIVE;

	/* video frame size */

	//width  = lt6911uxe_i2c_rd8(sd, H_ACTIVE_0P5) * 2;
	//height = lt6911uxe_i2c_rd8(sd, V_ACTIVE);
	//v4l2_dbg(1, debug, sd, "frame active - width %u height %u\n",
	//	width, height);
	//bt->width = width;
	//bt->height = height;

	/* front/back porch, sync pulse */
	//bt->hfrontporch	= lt6911uxe_i2c_rd8(sd, H_FP_0P5) * 2;
	//bt->hbackporch	= lt6911uxe_i2c_rd8(sd, H_BP_0P5) * 2;
	//bt->hsync	= lt6911uxe_i2c_rd8(sd, H_SW_0P5) * 2;

	//bt->vfrontporch	= lt6911uxe_i2c_rd8(sd, V_FP);
	//bt->vbackporch	= lt6911uxe_i2c_rd8(sd, V_BP);
	//bt->vsync	= lt6911uxe_i2c_rd8(sd, V_SW);

	//pol = lt6911uxe_i2c_rd8(sd, SYNC_POL);
	//if (pol & MASK_HSYNC_POL)
	//	bt->polarities |= V4L2_DV_HSYNC_POS_POL;
	//if (pol & MASK_VSYNC_POL)
	//	bt->polarities |= V4L2_DV_VSYNC_POS_POL;

	/* ------  pixelclock ------ */

	/* set frequency meter to half pixel clock */
	//lt6911uxe_i2c_wr8(sd, AD_HALF_PCLK, 0x21);
	//usleep_range(10000,10100);	/* needed by manufacturer */

	fm2 = lt6911uxe_i2c_rd8(sd, half_PixelClock2);
	fm1 = lt6911uxe_i2c_rd8(sd, half_PixelClock1);
	fm0 = lt6911uxe_i2c_rd8(sd, half_PixelClock0);

	half_pixel_clk = fm2 << 16 | fm1 << 8 | fm0;
	v4l2_dbg(1, debug, sd, "%s(): Pixelclock: %d\n", __func__, half_pixel_clk * 2 * 1000);

	h_htotal1 = lt6911uxe_i2c_rd8(sd, half_Htotal1);
	h_htotal0 = lt6911uxe_i2c_rd8(sd, half_Htotal0);
	htot = (h_htotal1 << 8 | h_htotal0) * 2;
	v4l2_dbg(1, debug, sd, "%s(): Htotal: %d\n", __func__, htot);

	vtotal1 = lt6911uxe_i2c_rd8(sd, Vtotal1);
	vtotal0 = lt6911uxe_i2c_rd8(sd, Vtotal0);
	vtot = (vtotal1 << 8 | vtotal0);
	v4l2_dbg(1, debug, sd, "%s(): Vtotal: %d\n", __func__, vtot);

	width1  = lt6911uxe_i2c_rd8(sd, half_Hactive1);
	width0  = lt6911uxe_i2c_rd8(sd, half_Hactive0);
	width = (width1 << 8 | width0) * 2;
	v4l2_dbg(1, debug, sd, "%s(): Width: %d\n", __func__, width);
	
	height1  = lt6911uxe_i2c_rd8(sd, Vactive1);
	height0  = lt6911uxe_i2c_rd8(sd, Vactive0);
	height = height1 << 8 | height0;
	v4l2_dbg(1, debug, sd, "%s(): Height: %d\n", __func__, height);

	//htot = V4L2_DV_BT_FRAME_WIDTH(bt);
	//vtot = V4L2_DV_BT_FRAME_HEIGHT(bt);

	/* frameinterval in ms */
	
	
	frame_interval = DIV_ROUND_CLOSEST((htot * vtot), (half_pixel_clk * 2));
	//fps = DIV_ROUND_CLOSEST((half_pixel_clk * 2 * 1000), (htot * vtot));
	//v4l2_dbg(1, debug, sd, "%s(): frame_interval %u ms\n", __func__, frame_interval);
	
	fps = (half_pixel_clk * 2 * 1000) / (htot * vtot);
	v4l2_dbg(1, debug, sd, "%s(): fps: %u\n", __func__, fps);


	lt6911uxe_timings.interlaced = V4L2_DV_PROGRESSIVE;
	lt6911uxe_timings.width = width;
	lt6911uxe_timings.height = height;
	lt6911uxe_timings.htot = htot;
	lt6911uxe_timings.vtot = vtot;
	lt6911uxe_timings.fps = fps;
	lt6911uxe_timings.pixelclock = half_pixel_clk * 2 * 1000;
	
	/*
	v4l2_dbg(1, debug, sd, "detected format: %ux%u%s%u.%02u (%ux%u)\n", 
	width, height, lt6911uxe_timings.interlaced ? "i" : "p", fps, fps % 100, 
	htot, vtot);
	*/

	if(compare_timings(&lt6911uxe_timings_old, &lt6911uxe_timings)) {
		v4l2_info(sd, "%s(): timings no change\n", __func__);
		lt6911uxe_timings.change = 0;
	} else {
		v4l2_info(sd, "%s(): timings change\n", __func__);
		lt6911uxe_timings.change = 1;
	}

	lt6911uxe_timings_old = lt6911uxe_timings;


	/* sanity check */
	//if ((bt->width < 640) || (bt->height < 480) ||
	//    (htot <= width) || (vtot <= height)) {
	//		memset(timings, 0, sizeof(struct v4l2_dv_timings));
	//		v4l2_dbg(1, debug, sd, "sanity check failed\n");
	//		v4l2_dbg(1, debug, sd, "bt->width: %d, bt->height: %d, htot: %d, width: %d, vtot: %d, height: %d\n",
	//				bt->width, bt->height, htot, width, vtot, height);
	//		return -ENOLCK;
	//}
	return 0;
}

/* ------ CORE OPS ---------------------------------------------------------- */

static int lt6911uxe_log_status(struct v4l2_subdev *sd)
{
	struct lt6911uxe_state *state = to_state(sd);

	v4l2_info(sd, "----- Timings -----\n");
	//if (!&state->detected_timings.bt.width) {
	if (!&lt6911uxe_timings.width) {
		v4l2_info(sd, "no video detected\n");
	} else {
		//v4l2_print_dv_timings(sd->name, "detected format: ",
		//		&state->detected_timings, true);
		v4l2_info(sd, "detected format: %ux%u%s%u.%02u (%ux%u)\n", lt6911uxe_timings.width, lt6911uxe_timings.height, lt6911uxe_timings.interlaced ? "i" : "p",
		lt6911uxe_timings.fps, lt6911uxe_timings.fps % 100, lt6911uxe_timings.htot, lt6911uxe_timings.vtot);
		v4l2_info(sd, "pixelclock: %u\n", lt6911uxe_timings.pixelclock);
		v4l2_info(sd, "---\n");
	}
	v4l2_print_dv_timings(sd->name, "configured format: ", &state->timings,
			true);

	return 0;
}

static int lt6911uxe_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
		struct v4l2_event_subscription *sub)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

/* ------ IRQ --------------------------------------------------------------- */

static void lt6911uxe_hdmi_int_handler(struct lt6911uxe_state *state,
		bool *handled)
{
	u8 int_event;
	struct v4l2_dv_timings timings = {};
	u8 fm2, fm1, fm0, lanes, mipi_format;
	int byte_clock;
	struct device *dev = &state->i2c_client->dev;

	/* Read interrupt event */
	int_event = lt6911uxe_i2c_rd8(&state->sd, INT_VIDEO);
	dev_dbg(dev, "%s: Interrupt type = 0x%02X\n", __func__, int_event);

	switch(int_event) {
	case INT_VIDEO_DISAPPEAR:
		/* stop MIPI output */
		dev_info(dev,"Video disappear\n");
		//lt6911uxe_csi_enable(&state->sd, false);

		if (state->signal_present) {
			state->signal_present = false;
			v4l2_subdev_notify_event(&state->sd,
						&lt6911uxe_ev_source_change);
			dev_dbg(dev,"event: no signal\n");

			//memset(&state->timings, 0, sizeof(state->timings));
			memset(&state->detected_timings, 0,
			       sizeof(state->detected_timings));
			dev_dbg(dev, "%s: clear lt6911uxe_timings\n", __func__);
			memset(&lt6911uxe_timings, 0, sizeof(lt6911uxe_timings));
		}
		if (handled)
			*handled = true;
		break;

	case INT_VIDEO_READY:
		dev_info(dev, "Video ready\n");

		/* at each HDMI-stable event renew timings */
		state->signal_present = true; 
		lt6911uxe_detect_timings(&state->sd, &timings);	
		
		/* byte clock / MIPI clock */
		//lt6911uxe_i2c_wr8(&state->sd, AD_HALF_PCLK, 0x1B);
		//usleep_range(10000,10100);
		fm2 = lt6911uxe_i2c_rd8(&state->sd, ByteClock2);
		fm1 = lt6911uxe_i2c_rd8(&state->sd, ByteClock1);
		fm0 = lt6911uxe_i2c_rd8(&state->sd, ByteClock0);

		byte_clock = fm2 << 16 | fm1 << 8 | fm0;
		dev_dbg(dev, "Byte clock: %d kHz, MIPI clock: %d kHz\n",
			byte_clock, byte_clock*4);

		/* MIPI */ 
		lanes = lt6911uxe_i2c_rd8(&state->sd, MIPI_LANES);
		dev_dbg(dev, "MIPI lanes: %d\n", lanes);

		//lt6911uxe_csi_enable(&state->sd, true);

		/* store newly detected timings (if any) if those are
		 * detected for the first time */
		/*
		if (!state->detected_timings.bt.width) {
			state->detected_timings = timings;
			dev_dbg(dev,"store new timings");
		} else if (v4l2_match_dv_timings(&timings, 
				&state->detected_timings, 250000, false)) {
			dev_dbg(dev,"ignore timings change");
		} else {
			state->detected_timings = timings;
			dev_dbg(dev,"detected timings updated");
		}
		*/
		if (lt6911uxe_timings.change) {
			dev_dbg(dev,"timings change, store new timings");
		} else {
			dev_dbg(dev,"no timings change");
		}

		if (handled)
			*handled = true;
		break;
	default:
		dev_err(dev, "%s: unhandled = 0x%02X\n", __func__, int_event);
		return;
	}
}

static void lt6911uxe_audio_int_handler(struct lt6911uxe_state *state, 
		bool *handled)
{
	u8 int_event;
	int audio_fs = 0;
	struct device *dev = &state->i2c_client->dev;
	
	/* read interrupt event */
	int_event = lt6911uxe_i2c_rd8(&state->sd, INT_AUDIO);
	dev_dbg(dev,"%s: Interrupt type =  0x%02X\n", __func__, int_event);

	switch(int_event) {
	case INT_AUDIO_DISAPPEAR:
		dev_info(dev,"Audio disappear\n");
		audio_fs = 0;
		break;
	//case INT_AUDIO_SR_HIGH:
	case INT_AUDIO_READY:
		/*
		if (state->signal_present) {
			dev_info(dev,"Audio ready\n");
			audio_fs = lt6911uxe_get_audio_sampling_rate(state);
		}
		else
			audio_fs = 0;
		*/
		dev_info(dev,"Audio ready\n");
		audio_fs = lt6911uxe_get_audio_sampling_rate(state);
		break;
	default:
		dev_err(dev,"%s: unhandled = 0x%02X\n", __func__, int_event);
		return;
	}

	v4l2_ctrl_s_ctrl(state->audio_present_ctrl, (audio_fs != 0));
	v4l2_ctrl_s_ctrl(state->audio_sampling_rate_ctrl, audio_fs);

	if (handled)
		*handled = true;
	return;
}

static int lt6911uxe_isr(struct v4l2_subdev *sd, bool *handled)
{
	struct lt6911uxe_state *state = to_state(sd);

	mutex_lock(&state->lock);
	dev_dbg(sd->dev, "%s in kthread %d\n", __func__, current->pid);

	lt6911uxe_ext_control(sd, true);

	/* Retrieve interrupt event */
	lt6911uxe_hdmi_int_handler(state, handled);

	lt6911uxe_audio_int_handler(state, handled);
	
	lt6911uxe_log_status(sd);

	//msleep(2300);

	lt6911uxe_ext_control(sd, false);

	mutex_unlock(&state->lock);
	return 0;
}

static irqreturn_t lt6911uxe_irq_handler(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = dev_id;
	bool handled = false;

	dev_err(sd->dev, "%s(): -----------------------------------------------IRQ-----------------------------------------------\n", __func__);
	lt6911uxe_isr(sd, &handled);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

/* ------ VIDEO OPS --------------------------------------------------------- */

static const struct v4l2_dv_timings_cap* lt6911uxe_g_timings_cap(
		struct lt6911uxe_state *state)
{
	return &lt6911uxe_timings_cap_4kp30;
}

static int lt6911uxe_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	*status = 0;
	*status |= no_signal(sd) ? V4L2_IN_ST_NO_SIGNAL : 0;

	v4l2_dbg(1, debug, sd, "%s: status = 0x%x\n", __func__, *status);
	return 0;
}

static int lt6911uxe_s_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct lt6911uxe_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (!v4l2_valid_dv_timings(timings, lt6911uxe_g_timings_cap(state),
				   NULL, NULL)) {
		v4l2_err(sd, "%s: timings out of range\n", __func__);
		return -EINVAL;
	}

	/* Fill optional fields .standards and .flags if format is part of
	 * CEA-861 / VESA-DMT timings */
	v4l2_find_dv_timings_cap(timings, lt6911uxe_g_timings_cap(state), 0,
				 NULL, NULL);

	/* Verify if new timings match current timings */
	if (v4l2_match_dv_timings(timings, &state->timings, 0, false)) {
		v4l2_info(sd, "%s: no change\n", __func__);
		return 0;
	}

	memset(timings->bt.reserved, 0, sizeof(timings->bt.reserved));
	state->timings = *timings;

	if (debug)
		v4l2_print_dv_timings(sd->name, "s_dv_timings: ",
				&state->timings, true);
	return 0;
}

static int lt6911uxe_g_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct lt6911uxe_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	*timings = state->timings;
	return 0;
}

static int lt6911uxe_query_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct lt6911uxe_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (no_signal(sd)) {
		v4l2_warn(sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}

	if (!v4l2_valid_dv_timings(&state->detected_timings,
				lt6911uxe_g_timings_cap(state), NULL, NULL)) {
		v4l2_warn(sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	*timings = state->detected_timings;
	if (debug)
		v4l2_print_dv_timings(sd->name, "query_dv_timings: ",
				timings, true);
	return 0;
}

static int lt6911uxe_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct lt6911uxe_state *state = container_of(sd, struct lt6911uxe_state, sd);
	dev_dbg(sd->dev, "%s(): enable %d \n", __func__, enable);
	//lt6911uxe_ext_control(sd, true);
	
	//lt6911uxe_csi_enable(sd, enable);
	//usleep_range(1000000, 1005000);
	//msleep(2100);

	//lt6911uxe_ext_control(sd, false);

	/* handled by ISR */
	
	if (enable) {
		dev_dbg(sd->dev, "%s(): reset LT6911UXE to stream on \n", __func__);
		gpio_set_value(state->pdata->reset_gpio, 0);
		msleep(120);
		gpio_set_value(state->pdata->reset_gpio, 1);
	}
	
	return 0;
}

/* ------ PAD OPS ----------------------------------------------------------- */

static int lt6911uxe_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_format *format)
{
	struct lt6911uxe_state *state = to_state(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int i = 0;

	v4l2_dbg(3, debug, sd,"%s():\n", __func__);

	if (format->pad != 0)
		return -EINVAL;

	/* retrieve mbus pixelcode and active video frame size */
	fmt->code = state->mbus_fmt_code;
	fmt->width  = state->timings.bt.width;
	fmt->height = state->timings.bt.height;
	fmt->field  = V4L2_FIELD_NONE;

	for (i = 0; i < ARRAY_SIZE(lt6911uxe_color_fmts); i++) {
		if (lt6911uxe_color_fmts[i].code == fmt->code) {
			fmt->colorspace = lt6911uxe_color_fmts[i].colorspace;
			break;
		}
	}

	switch (fmt->code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
	default:
		fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
		fmt->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		break;
	}
	return 0;
}

static int lt6911uxe_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_format *format)
{
	struct lt6911uxe_state *state = to_state(sd);
	u32 code = format->format.code; /* is overwritten by get_fmt */
	int ret;

	v4l2_dbg(2, debug, sd,
		"%s(): query format - width=%d, height=%d, code=0x%08X\n",
		__func__, format->format.width, format->format.height, code);

	/* adjust requested format based on current DV timings */
	ret = lt6911uxe_get_fmt(sd, sd_state, format);
	format->format.code = code;

	if (ret)
		return ret;

	switch (code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
		break;
	default:
		return -EINVAL;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	state->mbus_fmt_code = format->format.code;
	v4l2_dbg(2, debug, sd,
		"%s(): current format - width=%d, height=%d, code=0x%08X\n",
		__func__, format->format.width, format->format.height,
		state->mbus_fmt_code);
	return 0;
}

static int lt6911uxe_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_mbus_code_enum *code)
{
	v4l2_dbg(3, debug, sd, "%s()\n", __func__);

	if (code->index >= ARRAY_SIZE(lt6911uxe_color_fmts))
		return -EINVAL;

	code->code = lt6911uxe_color_fmts[code->index].code;
	v4l2_dbg(2, debug, sd, "%s(): fmt-code 0x%04X\n", __func__, code->code);

	return 0;
}

static int lt6911uxe_dv_timings_cap(struct v4l2_subdev *sd,
		struct v4l2_dv_timings_cap *cap)
{
	struct lt6911uxe_state *state = to_state(sd);
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (cap->pad != 0)
		return -EINVAL;

	*cap = *lt6911uxe_g_timings_cap(state);
	return 0;
}

static int lt6911uxe_enum_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_enum_dv_timings *timings)
{
	struct lt6911uxe_state *state = to_state(sd);
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (timings->pad != 0)
		return -EINVAL;

	/* filter non supported DV timings */
	return v4l2_enum_dv_timings_cap(timings,
				lt6911uxe_g_timings_cap(state), NULL, NULL);
}

/* ------ Register OPS ------------------------------------------------------ */

static int lt6911uxe_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);
	return 0;
}

static const struct v4l2_subdev_internal_ops lt6911uxe_subdev_internal_ops = {
	.open = lt6911uxe_open,
};

static struct v4l2_subdev_core_ops lt6911uxe_subdev_core_ops = {
	.log_status 		= lt6911uxe_log_status,
	.subscribe_event 	= lt6911uxe_subscribe_event,
	.unsubscribe_event 	= v4l2_event_subdev_unsubscribe,
};

static struct v4l2_subdev_video_ops lt6911uxe_subdev_video_ops = {
	.g_input_status		= lt6911uxe_g_input_status,	
	.s_dv_timings		= lt6911uxe_s_dv_timings,
	.g_dv_timings		= lt6911uxe_g_dv_timings,
	.query_dv_timings	= lt6911uxe_query_dv_timings,
	.s_stream		= lt6911uxe_s_stream,
};

static const struct v4l2_subdev_pad_ops lt6911uxe_pad_ops = {
	.set_fmt		= lt6911uxe_set_fmt,
	.get_fmt		= lt6911uxe_get_fmt,
	.enum_mbus_code		= lt6911uxe_enum_mbus_code,
	.dv_timings_cap		= lt6911uxe_dv_timings_cap,
	.enum_dv_timings	= lt6911uxe_enum_dv_timings,
};

static struct v4l2_subdev_ops lt6911uxe_ops = {
	.core 	= &lt6911uxe_subdev_core_ops,
	.video 	= &lt6911uxe_subdev_video_ops,
	.pad 	= &lt6911uxe_pad_ops,
};

#ifdef CONFIG_MEDIA_CONTROLLER
static const struct media_entity_operations lt6911uxe_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};
#endif

/* ------ CUSTOM CTRLS ------------------------------------------------------ */

static const struct v4l2_ctrl_config lt6911uxe_ctrl_audio_sampling_rate = {
	.id = LT6911UXE_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio Sampling Rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 192000,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config lt6911uxe_ctrl_audio_present = {
	.id = LT6911UXE_CID_AUDIO_PRESENT,
	.name = "Audio Present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

/* ------ Driver setup ------------------------------------------------------ */

static void lt6911uxe_initial_setup(struct lt6911uxe_state *state)
{
	state->mbus_fmt_code = MEDIA_BUS_FMT_UYVY8_1X16;
	state->signal_present = false;
	state->enable_i2c = false;
	mutex_init(&state->lock);

	/* Init Timings */
	lt6911uxe_s_dv_timings(&state->sd, &default_timing);
}

#ifdef CONFIG_OF

static const struct of_device_id lt6911uxe_of_match[] = {
	{ .compatible = "lontium,lt6911uxe" },
	{ }
};
MODULE_DEVICE_TABLE(of, lt6911uxe_of_match);

static struct lt6911uxe_platform_data* lt6911uxe_parse_dt(
						struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct lt6911uxe_platform_data *pdata;
	const struct of_device_id *match;
	int gpio;

	match = of_match_device(lt6911uxe_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev,
			"Driver has not been loaded from an of_match\n");
		return NULL;
	}
	pdata = devm_kzalloc(&client->dev, 
			     sizeof(struct lt6911uxe_platform_data), GFP_KERNEL);

	gpio = of_get_named_gpio(node, "reset-gpio", 0);
	if(gpio < 0) {
		if(gpio == -EPROBE_DEFER) {
			dev_err(&client->dev, "reset-gpio read failed: (%d)\n",
				gpio);
			goto prop_err;
		}
		dev_info(&client->dev, "reset-gpio not found, ignoring\n");
	}
	pdata->reset_gpio = gpio;
	return pdata;

prop_err:
	dev_err(&client->dev, "Could not parse DT parameters\n");
	devm_kfree(&client->dev, pdata);
	return NULL;
}
#else
static struct lt6911uxe_platform_data* lt6911uxe_parse_dt(
						struct i2c_client *client)
{
	return NULL;
}
#endif

static int lt6911uxe_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lt6911uxe_state *state;
	struct v4l2_subdev *sd;
	int err = 0;

	dev_info(&client->dev, "Probing lt6911uxe\n");
	state = devm_kzalloc(&client->dev, sizeof(struct lt6911uxe_state),
			     GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->pdata = lt6911uxe_parse_dt(client);
	if (!state->pdata)
		return -ENODEV;

	state->i2c_client = client;
	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &lt6911uxe_ops);

	dev_info(&client->dev, "Chip found @ 7h%02X (%s)\n", client->addr,
		 client->adapter->name);

	/* initial setup */
	lt6911uxe_initial_setup(state);
	dev_err(&client->dev, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX: start setup!\n");

	/* get interrupt */
	if (client->irq) {
		err = devm_request_threaded_irq(&state->i2c_client->dev,
						client->irq, NULL,
						lt6911uxe_irq_handler,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT, sd->name,
						(void *)sd);
		if (err) {
			dev_err(&client->dev,"Could not request interrupt %d!\n",
				client->irq);
			return err;
		}
	}

	/* custom v4l2 controls */
	v4l2_ctrl_handler_init(&state->ctrl_handler, 2);
	state->audio_sampling_rate_ctrl = v4l2_ctrl_new_custom(
		&state->ctrl_handler, &lt6911uxe_ctrl_audio_sampling_rate, NULL);
	state->audio_present_ctrl = v4l2_ctrl_new_custom(&state->ctrl_handler,
		&lt6911uxe_ctrl_audio_present, NULL);

	v4l2_ctrl_handler_setup(sd->ctrl_handler);
	if (state->ctrl_handler.error) {
		err = state->ctrl_handler.error;
		goto err_ctrl_handler;
	}
	sd->ctrl_handler = &state->ctrl_handler;

	/* media entitiy: define pad as output -> origins of link */
	if (IS_ENABLED(CONFIG_MEDIA_CONTROLLER)) {
		state->pad[0].flags = MEDIA_PAD_FL_SOURCE;
		sd->entity.ops = &lt6911uxe_media_ops;

		err = media_entity_pads_init(&sd->entity, 1,
					state->pad);
		if (err < 0) {
			dev_err(&client->dev, "unable to init media entity\n");
			goto err_ctrl_handler;
		}
	}

	/* register v4l2_subdev device */
	sd->dev	= &client->dev;
	sd->internal_ops = &lt6911uxe_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	err = v4l2_async_register_subdev(sd);
	if (err) {
		dev_err(&client->dev, "lt6911uxe subdev registration failed\n");
		goto err_ctrl_handler;
	}

	/*reset*/
	if (gpio_is_valid(state->pdata->reset_gpio)) {
        err = devm_gpio_request_one(&client->dev, state->pdata->reset_gpio,
                                    GPIOF_OUT_INIT_LOW, "lt6911uxe_reset");
        if (err) {
            dev_err(&client->dev, "Failed to request reset GPIO %d\n", state->pdata->reset_gpio);
            return err;
        }
        gpio_set_value(state->pdata->reset_gpio, 0);
        msleep(120);
        gpio_set_value(state->pdata->reset_gpio, 1);
		dev_info(&client->dev, "reset GPIO %d\n", state->pdata->reset_gpio);
    } else {
        dev_info(&client->dev, "reset_gpio is not valid\n");
    }

	dev_err(&client->dev, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX: probe finish!\n");
	return 0;

 err_ctrl_handler:
	v4l2_ctrl_handler_free(&state->ctrl_handler);
	return err;
}

static int lt6911uxe_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s \n", __func__);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	if (IS_ENABLED(CONFIG_MEDIA_CONTROLLER)) {
		media_entity_cleanup(&sd->entity);
	}

	dev_info(&client->dev, "removed lt6911uxe instance \n");
	return 0;
}

static const struct i2c_device_id lt6911uxe_id[] = {
	{ "lt6911uxe", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lt6911uxe_id);

static struct i2c_driver lt6911uxe_driver = {
	.driver = {
		.of_match_table = of_match_ptr(lt6911uxe_of_match),
		.name = "lt6911uxe",
		.owner = THIS_MODULE,
	},
	.id_table = lt6911uxe_id,
	.probe 	  = lt6911uxe_probe,
	.remove   = lt6911uxe_remove,
};
module_i2c_driver(lt6911uxe_driver);

MODULE_DESCRIPTION("Driver for Lontium lt6911uxe HDMI to CSI-2 Bridge");
MODULE_AUTHOR("Lukas Neuner <neur@zhaw.ch>");
MODULE_AUTHOR("Alexey Gromov <groo@zhaw.ch>");
MODULE_LICENSE("GPL v2");
