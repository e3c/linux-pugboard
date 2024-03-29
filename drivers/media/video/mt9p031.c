/*
 * drivers/media/video/mt9p031.c
 *
 * Aptina mt9p031 sensor driver
 *
 *
 * Copyright (C) 2010 Aptina Imaging
 * 
 * 
 * Leverage mt9p012.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */


#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/videodev2.h>
#include <linux/sysfs.h>

#include <media/mt9p031.h>
#include <media/v4l2-int-device.h>
#include <media/v4l2-chip-ident.h>

#define MT9P031_DEBUG

#ifdef MT9P031_DEBUG
#define DPRINTK_DRIVER(format, ...)				\
	printk(KERN_INFO "_MT9P031_DRIVER: " format, ## __VA_ARGS__)
#else
#define DPRINTK_DRIVER(format, ...)
#endif
/************************************************************************
			macro
************************************************************************/
// Macro to configure I2c level shifter. Use only for MT9P031 Headboards from Aptina; not required for Leopard Imaging or elsewise. 
#undef MT9P031_HEADBOARD

#define MT9P031_CHIP_ID			0x1801
#define MT9P031_MAX_HEIGHT		1944
#define MT9P031_MAX_WIDTH		2592
#define MT9P031_MIN_HEIGHT		2
#define MT9P031_MIN_WIDTH		2

#define VGA_HEIGHT		480
#define VGA_WIDTH		640

#define MT9P031_NORMAL_OPERATION_MODE		(0x1F82) //write
#define MT9P031_OUTPUT_CTRL_CHIP_UNSELECT	(0x1F80)
#define MT9P031_OUTPUT_CTRL_HALT		(0x1F83)

/* FPS Capabilities */
#define MT9P031_MIN_FPS		10
#define MT9P031_DEF_FPS		30
#define MT9P031_MAX_FPS		50

#define MT9P031_XCLK_NOM_1 12000000
#define MT9P031_XCLK_NOM_2 24000000

/* Analog gain values */
#define MT9P031_EV_MIN_GAIN		0
#define MT9P031_EV_MAX_GAIN		47
#define MT9P031_EV_DEF_GAIN		24
#define MT9P031_EV_GAIN_STEP		1

/* Exposure time values */
#define MT9P031_MIN_EXPOSURE		15000
#define MT9P031_MAX_EXPOSURE		128000
#define MT9P031_DEF_EXPOSURE		33000
#define MT9P031_EXPOSURE_STEP		100
#define Q12		4096
/************************************************************************
			Register Address
************************************************************************/

#define REG_MT9P031_CHIP_VERSION		0x00
#define REG_MT9P031_ROWSTART			0x01
#define REG_MT9P031_COLSTART			0x02
#define REG_MT9P031_HEIGHT			0x03
#define REG_MT9P031_WIDTH			0x04
#define REG_MT9P031_HBLANK			0x05
#define REG_MT9P031_VBLANK			0x06
#define REG_MT9P031_OUT_CTRL			0x07
#define REG_MT9P031_SHUTTER_WIDTH_U		0x08
#define REG_MT9P031_SHUTTER_WIDTH_L		0x09
#define REG_MT9P031_PCLK_CTRL			0x0a
#define REG_MT9P031_RESTART			0x0b
#define REG_MT9P031_SHUTTER_DELAY		0x0c
#define REG_MT9P031_RESET			0x0d

#define REG_MT9P031_PLL_CTRL			0x10
#define REG_MT9P031_PLL_CONF1			0x11
#define REG_MT9P031_PLL_CONF2			0x12

#define REG_MT9P031_READ_MODE1			0x1e
#define REG_MT9P031_READ_MODE2			0x20
#define REG_MT9P031_ROW_ADDR_MODE		0x22
#define REG_MT9P031_COL_ADDR_MODE		0x23
#define REG_MT9P031_GREEN_1_GAIN		0x2b
#define REG_MT9P031_BLUE_GAIN			0x2c
#define REG_MT9P031_RED_GAIN			0x2d
#define REG_MT9P031_GREEN_2_GAIN		0x2e
#define REG_MT9P031_GLOBAL_GAIN			0x35
#define REG_MT9P031_CHIP_VERSION_ALT	        0x0FF

/************************************************************************
			struct
************************************************************************/
struct mt9p031_frame_size {
	u16 width;
	u16 height;
};

struct mt9p031_priv {
	struct mt9p031_platform_data  *pdata;
	struct v4l2_int_device  *v4l2_int_device;
	struct i2c_client  *client;
	struct v4l2_pix_format  pix;
	struct v4l2_fract timeperframe;
	unsigned long xclk_current;
	int fps;
	int scaler;
	int ver;
	int  model;
	u32  flags;
/* for flags */
#define INIT_DONE  (1<<0)
};

struct mt9p031_priv sysPriv;

static const struct v4l2_fmtdesc mt9p031_formats[] = {
	{
		.description = "Bayer (sRGB) 10 bit",
		.pixelformat = V4L2_PIX_FMT_SRGGB10,
	},
};

static const unsigned int mt9p031_num_formats = ARRAY_SIZE(mt9p031_formats);

/***********************Minimum Horizontal blanking*********************/
int hb_min[4][4] = { 
	{ 450, 430, 0, 420 },
	{ 796, 776, 0, 766 },
	{ 0, 0, 0, 0 },
	{ 1488, 1468, 0, 1458 }, 
};

/**************************supported sizes******************************/
const static struct mt9p031_frame_size mt9p031_sizes[] = {
	{  640, 480 },
	{ 1280, 720 },
	{ 1920, 1080 },
	{ 2048, 1536 },	//3MP
	{ 2592, 1944 },	//5MP
};


struct mt9p031_format_params {
	int width;
	int height;
	int row_start;
	int col_start;
	int row_size;
	int col_size;
	int hblank;
	int vblank;
	int integ_time;
	int row_addr_mode;
	int col_addr_mode;
	int read_mode_2_config;
	int shutter_width_hi;
	int shutter_delay;
	int row_bin;
	int col_bin;		
};

enum mt9p031_image_size {
	VGA_BIN_30FPS,
	HDV_720P_30FPS,
	//HDV_720P_60FPS,
	//HDV_720P_60FPS_LVB,
	HDV_1080P_30FPS,
	MT9P031_THREE_MP,
	MT9P031_FIVE_MP,
};

enum mt9p031_image_size mt9p031_current_format;

const struct mt9p031_format_params mt9p031_supported_formats[] = {
	{ 640, 480, 64, 24, 1919, 2559, 0, 0, 0x0296,  0x0033, 0x0033, 0x0060, 0, 0, 3, 3 },  // VGA_BIN_30FPS
	{ 1280, 720, 64, 24, 1439, 2559, 0, 0, 0x0296, 0x0011, 0x0011, 0x0060, 0, 0, 1, 1 },  // 720P_HD_30FPS
	//{ 1280, 720, 0x0040, 0x0018, 0x059F, 0x09FF, 0, 0, 0x0296, 0x0011, 0x0011, 0x0060, 0, 0, 1, 1 },  // 720P_HD_60FPS
	//{ 1280, 720, 0x0040, 0x0018, 0x059F, 0x09FF, 0, 0x02D0, 0x0296, 0x0011, 0x0011, 0x0060, 0, 0, 1, 1 },  // 720P_HD_60FPS_LVB
	{ 1920, 1080, 431, 335, 1079, 1919, 0, 0x0037, 0x01AC, 0, 0, 0x0040, 0, 0, 0, 0 },	// 1080P_30FPS
	{ 2048, 1536, 431, 335, 1535, 2047, 0, 0x0037, 0x01AC, 0, 0, 0x0040, 0, 0, 0, 0 },	// 3MP CAPTURE		
	{ 2592, 1944, 431, 335, 1943, 2591, 0, 0x0037, 0x01AC, 0, 0, 0x0040, 0, 0, 0, 0 },	// 5MP CAPTURE	
};


const struct v4l2_fract mt9p031_frameintervals[] = {
	{ .numerator = 1, .denominator = 10 },
	{ .numerator = 1, .denominator = 20 },
	{ .numerator = 1, .denominator = 30 },	
	{ .numerator = 1, .denominator = 40 },	
	{ .numerator = 1, .denominator = 50 },		
};


const u16 MT9P031_EV_GAIN_TBL[48] = {
	/* Gain x1 */
	8, 9, 10, 11, 12, 13, 14, 15, 
	/* Gain x2 */
	16, 17, 18, 19, 20, 21, 22, 23,
	/* Gain x3 */
	24, 25, 26, 27, 28, 29, 30, 31, 
	/* Gain x4 */
	32, 33, 34, 35,
	/* Gain x5 */
	81, 82, 83,
	/* Gain x6 */
	84, 85, 86, 87, 88, 89, 90, 91,
	/* Gain x7 */
	92, 93, 94, 95, 96, 97, 98, 99,
	/* Gain x8 */
	100,
};

#ifdef MT9P031_HEADBOARD
/**
 * mt9p031_config_PCA9543A - configure on-board I2c level-shifter PCA9543A of MT9P031 Headboards from Aptina
 * @client: pointer to i2c client
 * Configures the level shifter to enable channel 0 
 */
static int mt9p031_config_PCA9543A(const struct i2c_client *client)
{
	struct i2c_msg msg;
	int ret;
	u8 buf;
	buf = 0x21;
	
	msg.addr  = (0xE6 >> 1);	//slave address of PCA9543A
	msg.flags = 0;
	msg.len   = 1;
	msg.buf   = &buf;
	
	ret = i2c_transfer(client->adapter, &msg, 1);

	return 0;
		
}
#endif		//MT9P031_HEADBOARD

/**
 * mt9p031_reg_read - read resgiter value
 * @client: pointer to i2c client
 * @command: register address
 */
static int mt9p031_reg_read(const struct i2c_client *client, u16 command, u16 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	// 8-bit/ byte addressable register
	buf[0] = command & 0xff;

	msg[0].addr  = client->addr;
	msg[0].flags = 0;
	msg[0].len   = 1;
	msg[0].buf   = buf ;
	ret = i2c_transfer(client->adapter, &msg[0], 1);
	
	if(ret >= 0) {
		msg[1].addr  = client->addr;
		msg[1].flags = I2C_M_RD; //1
		msg[1].len   = 2;
		msg[1].buf   = buf;
		ret = i2c_transfer(client->adapter, &msg[1], 1);
	}
	/*
	 * if return value of this function is < 0,
	 * it mean error.
	 * else, under 16bit is valid data.
	 */
	if(ret >= 0) {
		*val = 0;
		*val = buf[1] + (buf[0] << 8);
		return 0;
	}
	
	v4l_err(client, "read from offset 0x%x error %d", command, ret);
	return ret;
}

/**
 * mt9p031_reg_write - read resgiter value
 * @client: pointer to i2c client
 * @command: register address
 * @data: value to be written 
 */ 
static int mt9p031_reg_write(const struct i2c_client *client,
			       u16 command, u16 data)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	// 8-bit/ byte addressable register
		
	buf[0] = command & 0xff;
	data = swab16(data);
	memcpy(buf + 1, &data,    2);

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 3;
	msg.buf   = buf;
		
	/*
	 * i2c_transfer return message length,
	 * but this function should return 0 if correct case
	 */
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		ret = 0;

	return ret;
}

/**
 * struct vcontrol - Video controls
 * @v4l2_queryctrl: V4L2 VIDIOC_QUERYCTRL ioctl structure
 * @current_value: current value of this control
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} mt9p031_video_control[] = {
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = MT9P031_MIN_EXPOSURE,
			.maximum = MT9P031_MAX_EXPOSURE,
			.step = MT9P031_EXPOSURE_STEP,
			.default_value = MT9P031_DEF_EXPOSURE,
		},
		.current_value = MT9P031_DEF_EXPOSURE,
	},
	{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Analog Gain",
			.minimum = MT9P031_EV_MIN_GAIN,
			.maximum = MT9P031_EV_MAX_GAIN,
			.step = MT9P031_EV_GAIN_STEP,
			.default_value = MT9P031_EV_DEF_GAIN,
		},
		.current_value = MT9P031_EV_DEF_GAIN,
	},
};

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(mt9p031_video_control) - 1); i >= 0; i--)
		if (mt9p031_video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/**
 * mt9p031_calc_size - Find the best match for a requested image capture size
 * @width: requested image width in pixels
 * @height: requested image height in pixels
 *
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum mt9p031_image_size mt9p031_calc_size(unsigned int width,
						 unsigned int height)
{
	enum mt9p031_image_size isize;
	unsigned long pixels = width * height;

	for (isize = VGA_BIN_30FPS; isize <= MT9P031_FIVE_MP; isize++) {
		if (mt9p031_sizes[isize].height *
					mt9p031_sizes[isize].width >= pixels) {
			
			return isize;
		}
	}

	return MT9P031_FIVE_MP;
}

/**
 * mt9p031_find_isize - Find the best match for a requested image capture size
 * @width: requested image width in pixels
 * @height: requested image height in pixels
 *
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum mt9p031_image_size mt9p031_find_isize(unsigned int width)
{
	enum mt9p031_image_size isize;

	for (isize = VGA_BIN_30FPS; isize <= MT9P031_FIVE_MP; isize++) {
		if (mt9p031_sizes[isize].width >= width)
			break;
	}

	return isize;
}

/**
 * mt9p031_calc_xclk - Calculate the required xclk frequency
 * @c: i2c client driver structure
 *
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate and return the required xclk frequency
 */
static unsigned long mt9p031_calc_xclk(struct i2c_client *c)
{
	struct mt9p031_priv *priv = i2c_get_clientdata(c);
	struct v4l2_fract *timeperframe = &priv->timeperframe;

	if (timeperframe->numerator == 0 ||
	    timeperframe->denominator == 0) {
		/* supply a default nominal_timeperframe */
		timeperframe->numerator = 1;
		timeperframe->denominator = MT9P031_DEF_FPS;
	}

	priv->fps = timeperframe->denominator / timeperframe->numerator;
	if (priv->fps < MT9P031_MIN_FPS)
		priv->fps = MT9P031_MIN_FPS;
	else if (priv->fps > MT9P031_MAX_FPS)
		priv->fps = MT9P031_MAX_FPS;

	timeperframe->numerator = 1;
	timeperframe->denominator = priv->fps;

	return MT9P031_XCLK_NOM_1;
}

/**
 * mt9p031_set_params - sets register settings according to resolution
 * @client: pointer to standard i2c client
 * @width: width as queried by ioctl
 * @height: height as queried by ioctl
 */
static int mt9p031_set_params(struct i2c_client *client, u32 width, u32 height)
{
	struct mt9p031_priv *priv = i2c_get_clientdata(client);
	struct v4l2_pix_format *pix = &priv->pix;
	int ret;
	enum mt9p031_image_size i;

	i = mt9p031_find_isize(pix->width);
	priv->pix.width = mt9p031_supported_formats[i].width;
	priv->pix.height = mt9p031_supported_formats[i].height;
	
	ret = mt9p031_reg_write(client, REG_MT9P031_ROWSTART, mt9p031_supported_formats[i].row_start);		//ROW_WINDOW_START_REG
	ret |= mt9p031_reg_write(client, REG_MT9P031_COLSTART, mt9p031_supported_formats[i].col_start);		//COL_WINDOW_START_REG
	ret |= mt9p031_reg_write(client, REG_MT9P031_HEIGHT, mt9p031_supported_formats[i].row_size);		//ROW_WINDOW_SIZE_REG=1439
	ret |= mt9p031_reg_write(client, REG_MT9P031_WIDTH, mt9p031_supported_formats[i].col_size);		//COL_WINDOW_SIZE_REG=2559
	ret |= mt9p031_reg_write(client, REG_MT9P031_HBLANK, mt9p031_supported_formats[i].hblank);		//HORZ_BLANK=0
	ret |= mt9p031_reg_write(client, REG_MT9P031_VBLANK, mt9p031_supported_formats[i].vblank);		//VERT_BLANK_REG=720
	ret |= mt9p031_reg_write(client, REG_MT9P031_SHUTTER_WIDTH_L, 0x0400);		//SHUTTER_WIDTH_LOW (INTEG_TIME_REG = 1024)
	ret |= mt9p031_reg_write(client, REG_MT9P031_ROW_ADDR_MODE, mt9p031_supported_formats[i].row_addr_mode);		//ROW_MODE, ROW_SKIP=1, ROW_BIN=1	
	ret |= mt9p031_reg_write(client, REG_MT9P031_COL_ADDR_MODE, mt9p031_supported_formats[i].col_addr_mode);		//COL_MODE, COL_SKIP=1, COL_BIN=1
	ret |= mt9p031_reg_write(client, REG_MT9P031_READ_MODE2, mt9p031_supported_formats[i].read_mode_2_config);		//READ_MODE_2, COL_SUM
	ret |= mt9p031_reg_write(client, REG_MT9P031_SHUTTER_WIDTH_U, mt9p031_supported_formats[i].shutter_width_hi);		//SHUTTER_WIDTH_HI
	ret |= mt9p031_reg_write(client, REG_MT9P031_SHUTTER_WIDTH_L, mt9p031_supported_formats[i].integ_time);		//SHUTTER_WIDTH_LOW (INTEG_TIME_REG)
	ret |= mt9p031_reg_write(client, REG_MT9P031_SHUTTER_DELAY, mt9p031_supported_formats[i].shutter_delay);		//SHUTTER_DELAY_REG
		
	return ret;
}

/**
 * mt9p031_init_camera - initialize camera settings
 * @client: pointer to i2c client
 * Initialize camera settings 
 */ 
static int mt9p031_init_camera(const struct i2c_client *client)
{
	int ret;
	struct mt9p031_priv *priv = i2c_get_clientdata(client);
	struct v4l2_pix_format *pix = &priv->pix;

	ret = mt9p031_reg_write(client, REG_MT9P031_PLL_CTRL, 0x0051);  	//PLL_CTRL; power up pll
	ret |= mt9p031_reg_write(client, REG_MT9P031_PLL_CONF1, 0x1801);		//PLL_CONFIG_1: m=24, n=1
	ret |= mt9p031_reg_write(client, REG_MT9P031_PLL_CONF2, 0x0002);		//PLL_CONFIG_2: p1=2, p2=0
	mdelay(10);  										//wait 10 ms for VCO to lock
	ret |= mt9p031_reg_write(client, REG_MT9P031_PLL_CTRL, 0x0053);		//PLL_CONTROL; use PLL
	mdelay(200);

	ret |= mt9p031_set_params(priv->client, pix->width, pix->height);
	
	ret |= mt9p031_reg_write(client, REG_MT9P031_RESET, 0x0001);	//High
	ret |= mt9p031_reg_write(client, REG_MT9P031_RESET, 0x0000);	//Low
	mdelay(100);
	
	ret |= mt9p031_reg_write(client, REG_MT9P031_GREEN_1_GAIN, 0x0051);  	//Green1_gain_reg
	ret |= mt9p031_reg_write(client, REG_MT9P031_BLUE_GAIN, 0x0051);  	//Blue_gain_reg
	ret |= mt9p031_reg_write(client, REG_MT9P031_RED_GAIN, 0x0051);  	//Red_gain_reg
	ret |= mt9p031_reg_write(client, REG_MT9P031_GREEN_2_GAIN, 0x0051);  	//Green2_gain_reg
	ret |= mt9p031_reg_write(client, REG_MT9P031_GLOBAL_GAIN, 0x0008);		//Analog Gain
	ret |= mt9p031_reg_write(client, REG_MT9P031_READ_MODE1, 0x0006);  	//Read_mode_1 //disable AB
	ret |= mt9p031_reg_write(client, REG_MT9P031_OUT_CTRL, 0x1F8E);		//Enable parll fifo data
	
	return ret>= 0 ? 0 : -EIO;
}

/************************************************************************
			i2c driver
************************************************************************/
/**
 * mt9p031_detect - Detect if an mt9p031 is present, and if so which revision
 * @client: pointer to the i2c client driver structure
 *
 * Returns a negative error number if no device is detected
 */
static int mt9p031_detect(struct i2c_client *client)
{
	struct mt9p031_priv *priv = i2c_get_clientdata(client);
	const char	*devname;
	u16 chipid;

	if (!client)
		return -ENODEV;
	/*
	 * Set Normal Mode
	 */
	if(mt9p031_reg_write(client, REG_MT9P031_OUT_CTRL, MT9P031_NORMAL_OPERATION_MODE))
		return -ENODEV;
	/*
	 * check and show chip ID
	 */
	if(mt9p031_reg_read(client, REG_MT9P031_CHIP_VERSION, &chipid)) 
		return -ENODEV;
		
	if(chipid == MT9P031_CHIP_ID) {
		devname = "mt9p031";
		priv->model = V4L2_IDENT_MT9P031;
		dev_info(&client->dev, "%s chip ID %04x\n", devname, chipid);
		return 0;
	}
			
	dev_err(&client->dev, "Product ID error %04x\n", chipid);
		return -ENODEV;
}

/**
 * mt9p031_set_exposure_time - sets exposure time per input value
 * @exp_time: exposure time to be set on device
 * @client: pointer to standard i2c client
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 *
 * If the requested exposure time is within the allowed limits, the HW
 * is configured to use the new exposure time, and the video_controls
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
static int mt9p031_set_exposure_time(u32 exp_time, struct i2c_client *client,
								struct vcontrol *lvc)
{
	int ret = 0, i, shutter_width, so_p, t_pix_clk, sd_p, shutter_delay;
	int sw_l ,sw_u ,W ,h_blanking, t_row;
	
	if(exp_time < MT9P031_MIN_EXPOSURE)
			exp_time = MT9P031_MIN_EXPOSURE;
	else if(exp_time > MT9P031_MAX_EXPOSURE)
			exp_time = MT9P031_MAX_EXPOSURE;
	
	shutter_delay = mt9p031_supported_formats[mt9p031_current_format].shutter_delay;
	sd_p = min(shutter_delay + 1, 1504);
	so_p = 208 * (mt9p031_supported_formats[mt9p031_current_format].row_bin + 1) + 98 + sd_p - 94;
	t_pix_clk = (Q12/96 );	
	h_blanking = mt9p031_supported_formats[mt9p031_current_format].hblank + 1;
	W = 2 * (int)((mt9p031_supported_formats[mt9p031_current_format].row_size + 1) / (2 * (mt9p031_supported_formats[mt9p031_current_format].row_bin + 1)) + 1);		
	t_row = 2 * t_pix_clk * max(W/2 + max(h_blanking, hb_min[mt9p031_supported_formats[mt9p031_current_format].row_bin][mt9p031_supported_formats[mt9p031_current_format].col_bin]),
							  (41 + 346 * (mt9p031_supported_formats[mt9p031_current_format].row_bin + 1) + 99))/Q12;
							  
	shutter_width = (exp_time + 2*so_p*t_pix_clk) / t_row;
	
	if (shutter_width<  3) {
		sd_p = 1232 >  shutter_delay ? 1232 : shutter_delay;
		so_p = 208 * (mt9p031_supported_formats[mt9p031_current_format].row_bin + 1) + 98 + sd_p - 94;
		shutter_width = ((exp_time*Q12 + 2*so_p*t_pix_clk) / (t_row * Q12));	
	}
	
	if (shutter_width <  1)
		shutter_width = 1;
	sw_l = shutter_width&  0xffff;
	sw_u = (shutter_width)>>  16;
	ret = mt9p031_reg_write(client, REG_MT9P031_SHUTTER_WIDTH_L,sw_l);
	mdelay(1);
	ret = mt9p031_reg_write(client, REG_MT9P031_SHUTTER_WIDTH_U,sw_u);
	
	if (ret)
		dev_err(&client->dev, "Error setting exposure time %d\n",
									ret);
	else{										
		i = find_vctrl(V4L2_CID_EXPOSURE);
		if (i >= 0) {
			lvc = &mt9p031_video_control[i];
			lvc->current_value = exp_time;
		}
	}
	
	return ret;	
}

/**
 * mt9p031_set_gain - sets sensor analog gain per input value
 * @lineargain: analog gain value index to be set on device
 * @client: pointer to standard i2c client
 * @lvc: pointer to V4L2 analog gain entry in video_controls array
 *
 * If the requested analog gain is within the allowed limits, the HW
 * is configured to use the new gain value, and the video_controls
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int mt9p031_set_gain(u16 lineargain, struct i2c_client *client,
							struct vcontrol *lvc)
{
	int ret= 0, i;
	u16 reg_gain = 0;
		
	if (lineargain < MT9P031_EV_MIN_GAIN) {
		lineargain = MT9P031_EV_MIN_GAIN;
		v4l_err(client, "Gain out of legal range.");
	}
	if (lineargain > MT9P031_EV_MAX_GAIN) {
		lineargain = MT9P031_EV_MAX_GAIN;
		v4l_err(client, "Gain out of legal range.");
	}

	reg_gain = MT9P031_EV_GAIN_TBL[lineargain];
	ret = mt9p031_reg_write(client, REG_MT9P031_GLOBAL_GAIN,
					reg_gain);
	
	if (ret) {
		dev_err(&client->dev, "Error setting gain.%d", ret);
		return ret;	
	}
	else {
		i = find_vctrl(V4L2_CID_GAIN);
		if (i >= 0) {
			lvc = &mt9p031_video_control[i];
			lvc->current_value = lineargain;
		}
	}

	return ret;
}

/************************************************************************
			v4l2_ioctls
************************************************************************/

/**
 * mt9p031_v4l2_int_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int mt9p031_v4l2_int_s_power(struct v4l2_int_device *s,
				    enum v4l2_power power)
{
	struct mt9p031_priv *priv = s->priv;
	struct i2c_client *client = priv->client;
	
	int ret;

	switch (power) {
	case V4L2_POWER_STANDBY:
		/* FALLTHROUGH */
	case V4L2_POWER_OFF:
		ret = priv->pdata->power_set(s, power);
		if (ret < 0) {
			dev_err(&client->dev, "Unable to set target board power "
					 "state (OFF/STANDBY)\n");
			return ret;
		}
		break;
	case V4L2_POWER_ON:
		ret = priv->pdata->power_set(s, power);

		if (ret < 0) {
			dev_err(&client->dev, "Unable to set target board power "
					 "state (ON)\n");
			return ret;
		}
		if (!(priv->flags & INIT_DONE)) {
			ret = mt9p031_detect(client);
			if (ret < 0) {
				dev_err(&client->dev, "Unable to detect sensor\n");
				return ret;
			}
			priv->flags |= INIT_DONE;
		}

		ret = mt9p031_init_camera(client);
		if (ret < 0) {
				dev_err(&client->dev, "Unable to initialize sensor\n");
				return ret;
		}
	}
	
	return 0;
}

/**
 * mt9p031_v4l2_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int mt9p031_v4l2_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	int retval = -EINVAL;
	int i;
	struct vcontrol *lvc;
	struct mt9p031_priv *priv = s->priv;
	struct i2c_client *client = priv->client;
	
	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &mt9p031_video_control[i];

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		retval = mt9p031_set_exposure_time(vc->value, client, lvc);
		break;
	case V4L2_CID_GAIN:
		retval = mt9p031_set_gain(vc->value, client, lvc);
		break;
	}

	return retval;
}

/**
 * mt9p031_v4l2_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int mt9p031_v4l2_g_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	struct vcontrol *lvc;
	int i;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &mt9p031_video_control[i];

	switch (vc->id) {
	case  V4L2_CID_EXPOSURE:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_GAIN:
		vc->value = lvc->current_value;
		break;
	}

	return 0;
}

/**
 * mt9p031_v4l2_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int mt9p031_v4l2_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = mt9p031_video_control[i].qc;
	return 0;
}


/**
 * mt9p031_v4l2_int_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
static int mt9p031_v4l2_int_enum_fmt_cap(struct v4l2_int_device *s,
					 struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= ARRAY_SIZE(mt9p031_formats))
			return -EINVAL;
	        break;
	default:
		return -EINVAL;
	}

	strlcpy(fmt->description, mt9p031_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = mt9p031_formats[index].pixelformat;

	return 0;
}

/**
 * mt9p031_v4l2_int_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int mt9p031_v4l2_int_try_fmt_cap(struct v4l2_int_device *s,
					struct v4l2_format *f)
{
	enum mt9p031_image_size isize;
	int ifmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct mt9p031_priv *priv = s->priv;
	struct v4l2_pix_format *pix2 = &priv->pix;

	isize = mt9p031_calc_size(pix->width, pix->height);
	mt9p031_current_format = isize;
	
	pix->width = mt9p031_sizes[isize].width;
	pix->height = mt9p031_sizes[isize].height;
	for (ifmt = 0; ifmt < mt9p031_num_formats; ifmt++) {
		if (pix->pixelformat == mt9p031_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == mt9p031_num_formats)
		ifmt = 0;
	pix->pixelformat = mt9p031_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	
	*pix2 = *pix;

	return 0;
}

/**
 * mt9p031_v4l2_int_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int mt9p031_v4l2_int_s_fmt_cap(struct v4l2_int_device *s,
				      struct v4l2_format *f)
{
	struct mt9p031_priv *priv = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;
	
	rval = mt9p031_v4l2_int_try_fmt_cap(s, f);
	if (!rval)
		priv->pix = *pix;
		
	return rval;
}

/**
 * mt9p031_v4l2_int_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int mt9p031_v4l2_int_g_fmt_cap(struct v4l2_int_device *s,
				      struct v4l2_format *f)
{
	struct mt9p031_priv *priv = s->priv;
	
	f->fmt.pix.width	= priv->pix.width;
	f->fmt.pix.height	= priv->pix.height;
	f->fmt.pix.pixelformat	= V4L2_COLORSPACE_SRGB;
	f->fmt.pix.pixelformat	= priv->pix.pixelformat;
	f->fmt.pix.field	= V4L2_FIELD_NONE;

	return 0;
}

/**
 * mt9p031_v4l2_int_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */


static int mt9p031_v4l2_int_s_parm(struct v4l2_int_device *s,
				   struct v4l2_streamparm *a)
{
	struct mt9p031_priv *priv = s->priv;
	struct i2c_client *client = priv->client;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;

	priv->timeperframe = *timeperframe;
	priv->xclk_current = mt9p031_calc_xclk(client);
	*timeperframe = priv->timeperframe;

	return 0;
}

/**
 * mt9p031_v4l2_int_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int mt9p031_v4l2_int_g_parm(struct v4l2_int_device *s,
				   struct v4l2_streamparm *a)
{
	struct mt9p031_priv *priv = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe.numerator = 1;
	cparm->timeperframe = priv->timeperframe;

	return 0;
}

/**
 * mt9p031_v4l2_int_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int mt9p031_v4l2_int_g_priv(struct v4l2_int_device *s, void *p)
{
	struct mt9p031_priv *priv = s->priv;

	return priv->pdata->priv_data_set(p);
}

/**
 * mt9p031_v4l2_int_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's ifparm
 *
 * Returns device's (sensor's) ifparm in p parameter
 */
static int mt9p031_v4l2_int_g_ifparm(struct v4l2_int_device *s,
				     struct v4l2_ifparm *p)
{
	struct mt9p031_priv *priv = s->priv;
	int rval;

	if (p == NULL)
		return -EINVAL;

	if (!priv->pdata->ifparm)
		return -EINVAL;

	rval = priv->pdata->ifparm(p);
	if (rval) {
		v4l_err(priv->client, "g_ifparm.Err[%d]\n", rval);
		return rval;
	}

	return 0;
}

/**
 * mt9p031_v4l2_int_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 */
static int mt9p031_v4l2_int_enum_framesizes(struct v4l2_int_device *s,
					    struct v4l2_frmsizeenum *frms)
{
	int ifmt;

	for (ifmt = 0; ifmt < ARRAY_SIZE(mt9p031_formats); ifmt++)
		if (mt9p031_formats[ifmt].pixelformat == frms->pixel_format)
			break;

	if (ifmt == ARRAY_SIZE(mt9p031_formats))
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= ARRAY_SIZE(mt9p031_sizes))
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = mt9p031_sizes[frms->index].width;
	frms->discrete.height = mt9p031_sizes[frms->index].height;

	return 0;
}

static int mt9p031_v4l2_int_enum_frameintervals(struct v4l2_int_device *s,
						struct v4l2_frmivalenum *frmi)
{
	int ifmt;
	int max_size;

	for (ifmt = 0; ifmt < ARRAY_SIZE(mt9p031_formats); ifmt++)
		if (mt9p031_formats[ifmt].pixelformat == frmi->pixel_format)
			break;

	if (ifmt == ARRAY_SIZE(mt9p031_formats))
		return -EINVAL;

	max_size = ARRAY_SIZE(mt9p031_sizes);
	
	for(ifmt = 0; ifmt < max_size; ifmt++) {
		if(frmi->width <= mt9p031_sizes[ifmt].width) {
			frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
			frmi->discrete.numerator =
				mt9p031_frameintervals[frmi->index].numerator;
			frmi->discrete.denominator =
				mt9p031_frameintervals[frmi->index].denominator;

			if(frmi->discrete.denominator <= mt9p031_frameintervals[max_size - ifmt - 1].denominator)
				return 0;
			else
				return -EINVAL;
		}
	}

	return 0;
}

static struct v4l2_int_ioctl_desc mt9p031_ioctl_desc[] = {
	{ .num = vidioc_int_enum_framesizes_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_enum_framesizes },
	{ .num = vidioc_int_enum_frameintervals_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_enum_frameintervals },
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_g_priv },
	{ .num = vidioc_int_g_ifparm_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_g_ifparm },
	{ .num = vidioc_int_enum_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_enum_fmt_cap },
	{ .num = vidioc_int_try_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_try_fmt_cap },
	{ .num = vidioc_int_g_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_g_fmt_cap }, 
	{ .num = vidioc_int_s_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_s_fmt_cap },
	{ .num = vidioc_int_g_parm_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_g_parm },
	{ .num = vidioc_int_s_parm_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_int_s_parm },
	{ .num = vidioc_int_g_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_g_ctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_s_ctrl },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)mt9p031_v4l2_queryctrl },
};

#ifdef MT9P031_DEBUG
/**
 * ---------------------------------------------------------------------------------
 * Sysfs
 * ---------------------------------------------------------------------------------
 */

/* Basic register read write support */
static u16 mt9p031_attr_basic_addr  = 0x0000;

static ssize_t
mt9p031_basic_reg_addr_show( struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", mt9p031_attr_basic_addr);
}

static ssize_t
mt9p031_basic_reg_addr_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	u16 val;
	sscanf(buf, "%hx", &val);
	mt9p031_attr_basic_addr = (u16) val;
	return n;
}

static DEVICE_ATTR( basic_reg_addr, S_IRUGO|S_IWUSR, mt9p031_basic_reg_addr_show, mt9p031_basic_reg_addr_store);


static ssize_t
mt9p031_basic_reg_val_show( struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 val;
	int ret;
	ret = mt9p031_reg_read(sysPriv.client, mt9p031_attr_basic_addr, &val);
	if(ret < 0){        
		printk(KERN_INFO "mt9p031: Basic register read failed");
		return 1; // nothing processed
	} else {
		return sprintf(buf, "0x%x\n", val);
	}
}

static ssize_t
mt9p031_basic_reg_val_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	u32 val;
	sscanf(buf, "%x", &val);

	if (mt9p031_reg_write(sysPriv.client, mt9p031_attr_basic_addr, (u16)val)) {
		printk(KERN_INFO "mt9p031: Basic regiser write failed");
		return n; // nothing processed
	} else {
		return n;
	}
}
static DEVICE_ATTR( basic_reg_val, S_IRUGO|S_IWUSR, mt9p031_basic_reg_val_show, mt9p031_basic_reg_val_store);


/* Exposure time access support */
static ssize_t
mt9p031_exposure_val_show( struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 val;
	struct vcontrol *lvc;
	int i = find_vctrl(V4L2_CID_EXPOSURE);
	if (i < 0)
		return -EINVAL;
	lvc = &mt9p031_video_control[i];
	val = lvc->current_value;
	
	if(val < 0){        
		printk(KERN_INFO "mt9p031: Exposure value read failed");
		return 1; // nothing processed
	} else {
		return sprintf(buf, "%d\n", val);
	}
}


static ssize_t
mt9p031_exposure_val_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	u32 val;
	struct i2c_client *client;
	struct vcontrol *lvc;
	
	sscanf(buf, "%d", &val);
	client = sysPriv.client;
		
	lvc = &mt9p031_video_control[V4L2_CID_EXPOSURE];	

	if (mt9p031_set_exposure_time((u32)val, client, lvc)) {
		printk(KERN_INFO "mt9p031: Exposure write failed");
		return n; // nothing processed
	} else {
		return n;
    }
}

static DEVICE_ATTR( exposure_val, S_IRUGO|S_IWUSR, mt9p031_exposure_val_show, mt9p031_exposure_val_store);


/* Global Gain access support */
static ssize_t
mt9p031_gain_val_show( struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 val;
	struct vcontrol *lvc;
    
	int i = find_vctrl(V4L2_CID_GAIN);
	if (i < 0)
		return -EINVAL;
	lvc = &mt9p031_video_control[i];
	val = lvc->current_value;
      
	if(val < 0){        
		printk(KERN_INFO "mt9p031: Global Gain value read failed");
		return 1; // nothing processed
	} else {
		return sprintf(buf, "%d\n", val);
    }
}

static ssize_t
mt9p031_gain_val_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	u16 val;
	struct i2c_client *client;
	struct vcontrol *lvc;
	
	sscanf(buf, "%hd", &val);
	client = sysPriv.client;
		
	lvc = &mt9p031_video_control[V4L2_CID_GAIN];	
		
	if (mt9p031_set_gain(val, client, lvc)) {
		printk(KERN_INFO "mt9p031: Global gain write failed");
		return n; // nothing processed
	} else {
		return n;
	}
}

static DEVICE_ATTR( gain_val, S_IRUGO|S_IWUSR, mt9p031_gain_val_show, mt9p031_gain_val_store);


static struct attribute *mt9p031_sysfs_attr[] = {
	&dev_attr_basic_reg_addr.attr,
	&dev_attr_basic_reg_val.attr,
	&dev_attr_exposure_val.attr,
	&dev_attr_gain_val.attr,
};

static int mt9p031_sysfs_add(struct kobject *kobj)
{
	int i = ARRAY_SIZE(mt9p031_sysfs_attr);
	int rval = 0;
	
	do {
		rval = sysfs_create_file(kobj, mt9p031_sysfs_attr[--i]);
	} while((i > 0) && (rval == 0));
	return rval;
}

static int mt9p031_sysfs_rm(struct kobject *kobj)
{
	int i = ARRAY_SIZE(mt9p031_sysfs_attr);
	int rval = 0;

	do {
		sysfs_remove_file(kobj, mt9p031_sysfs_attr[--i]);
	} while(i > 0);
	return rval;
}
#endif	//MT9P031_DEBUG

static struct v4l2_int_slave mt9p031_slave = {
	.ioctls = mt9p031_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9p031_ioctl_desc),
};

static int mt9p031_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct mt9p031_priv *priv;
	struct v4l2_int_device *v4l2_int_device;
	int ret;
	if (!client->dev.platform_data) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_int_device = kzalloc(sizeof(*v4l2_int_device), GFP_KERNEL);
	if (!v4l2_int_device) {
		kfree(priv);
		return -ENOMEM;
	}

#ifdef MT9P031_HEADBOARD
	mt9p031_config_PCA9543A(client);		//configure i2c level shifter on mt9p031 head-board, no need for Leopard module
	mdelay(10);
#endif	//MT9P031_HEADBOARD
	
	v4l2_int_device->module = THIS_MODULE;
	strncpy(v4l2_int_device->name, "mt9p031", sizeof(v4l2_int_device->name));
	
	v4l2_int_device->type = v4l2_int_type_slave;
	v4l2_int_device->u.slave = &mt9p031_slave;

	v4l2_int_device->priv = priv;

	priv->v4l2_int_device = v4l2_int_device;
	priv->client = client;
	priv->pdata = client->dev.platform_data;
	
	priv->pdata->flags = MT9P031_FLAG_PCLK_RISING_EDGE;
	
	/* Setting Pixel Values */
	priv->pix.width       = mt9p031_sizes[0].width;
	priv->pix.height      = mt9p031_sizes[0].height;
	priv->pix.pixelformat = mt9p031_formats[0].pixelformat;
	
	i2c_set_clientdata(client, priv);
	
	sysPriv.client = priv->client;

	ret = v4l2_int_device_register(priv->v4l2_int_device);
	if (ret) {
		i2c_set_clientdata(client, NULL);
		kfree(v4l2_int_device);
		kfree(priv);
	}
	
#ifdef MT9P031_DEBUG
	mt9p031_sysfs_add(&client->dev.kobj);
#endif	//MT9P031_DEBUG	
	return ret;
}

static int mt9p031_remove(struct i2c_client *client)
{
	struct mt9p031_priv *priv = i2c_get_clientdata(client);

	v4l2_int_device_unregister(priv->v4l2_int_device);
	i2c_set_clientdata(client, NULL);
	mt9p031_sysfs_rm(&client->dev.kobj);
	
	kfree(priv->v4l2_int_device);
	kfree(priv);
	return 0;
}

static const struct i2c_device_id mt9p031_id[] = {
	{ "mt9p031", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9p031_id);

static struct i2c_driver mt9p031_i2c_driver = {
	.driver = {
		.name = "mt9p031",
	},
	.probe    = mt9p031_probe,
	.remove   = mt9p031_remove,
	.id_table = mt9p031_id,
};

/************************************************************************
			module function
************************************************************************/
static int __init mt9p031_module_init(void)
{
	return i2c_add_driver(&mt9p031_i2c_driver);
}

static void __exit mt9p031_module_exit(void)
{
	i2c_del_driver(&mt9p031_i2c_driver);
}

module_init(mt9p031_module_init);
module_exit(mt9p031_module_exit);

MODULE_DESCRIPTION("mt9p031 sensor driver");
MODULE_AUTHOR("Aptina");
MODULE_LICENSE("GPL v2");

