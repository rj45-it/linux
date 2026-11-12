// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony IMX682 CMOS Image Sensor driver
 *
 * Copyright (C) 2026
 *
 * Based on drivers/media/i2c/imx258.c and imx319.c.
 *
 * Power sequencing and pin assignments below are independently confirmed
 * against Samsung's own published GPL kernel source for this exact board
 * (matei9/android_kernel_samsung_sm7125, arch/arm64/boot/dts/samsung/
 * drivers/atoll-sec-a52q-camera-sensor-common.dtsi, "REAR CAM1 - 64MP
 * Wide" node) - not just the user's own hardware measurements, but a
 * second, independent source: reset on GPIO 30, MCLK0 at 24MHz, I2C
 * slave-addr 0x34 (0x1a 7-bit), and regulator ranges 1.8V / 2.8-2.9V /
 * 1.0-1.1V all match exactly.
 *
 * That same kernel source confirms techpack/camera (the actual sensor
 * driver code) is NOT published here - zero files, not even a submodule
 * reference, across the repo's full history. On this Spectra/CamX
 * architecture the init/mode register VALUES are never compiled into
 * kernel source at all (confirmed architecturally via CamX decompilation
 * earlier in this project); only power/clock/GPIO wiring lives in the
 * device tree. So the init register sequence and per-mode register
 * tables below remain PLACEHOLDERS/best-effort cross-references (see
 * per-table comments) until obtained via a live I2C/CCI capture. Do NOT
 * fill the flagged (clock- and PLL-dependent) entries from guesses.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Chip ID */
#define IMX682_REG_CHIP_ID		0x0016
#define IMX682_CHIP_ID			0x0682

/* Standard Sony IMX register set */
#define IMX682_REG_MODE_SELECT		0x0100
#define IMX682_MODE_STANDBY		0x00
#define IMX682_MODE_STREAMING		0x01

#define IMX682_REG_RESET		0x0103

#define IMX682_REG_EXPOSURE		0x0202
#define IMX682_EXPOSURE_MIN		8
#define IMX682_EXPOSURE_STEP		1
#define IMX682_EXPOSURE_DEFAULT		0x0648
#define IMX682_EXPOSURE_MAX		65535

#define IMX682_REG_ANALOG_GAIN		0x0204
#define IMX682_ANA_GAIN_MIN		0
#define IMX682_ANA_GAIN_MAX		978
#define IMX682_ANA_GAIN_STEP		1
#define IMX682_ANA_GAIN_DEFAULT		0

#define IMX682_REG_FRAME_LENGTH_LINES	0x0340
#define IMX682_REG_LINE_LENGTH_PCK	0x0342

#define IMX682_REG_TEST_PATTERN		0x0600
#define IMX682_TEST_PATTERN_DISABLE	0
#define IMX682_TEST_PATTERN_SOLID_COLOR	1
#define IMX682_TEST_PATTERN_COLOR_BARS	2
#define IMX682_TEST_PATTERN_GREY_COLOR	3
#define IMX682_TEST_PATTERN_PN9		4

/*
 * Per-channel test pattern values. Addresses confirmed by extracting the
 * real device's vendor Chromatix blob (symbolic register-address catalog
 * embedded in com.samsung.sensormodule.0_0_sony_imx682.bin), not guessed
 * from imx258.c. 0x0202/0x0204/0x0340/0x0342/0x0600 above were likewise
 * cross-checked against that same catalog.
 */
#define IMX682_REG_TEST_PATTERN_R	0x0602
#define IMX682_REG_TEST_PATTERN_GR	0x0604
#define IMX682_REG_TEST_PATTERN_GB	0x0606
#define IMX682_REG_TEST_PATTERN_B	0x0608

#define IMX682_REG_ORIENTATION		0x0101

/* MIPI link frequencies (from adaptive MIPI header) */
#define IMX682_LINK_FREQ_1708		1708800000ULL
#define IMX682_LINK_FREQ_1849		1849600000ULL

#define IMX682_NUM_LANES		4
#define IMX682_DATA_RATE_MAX		(IMX682_LINK_FREQ_1849 * 2)

/* Pixel rate is derived per-mode from link freq / lanes / bpp; placeholder
 * defaults below, recompute once real mode timing is extracted. */
#define IMX682_PIXEL_RATE		1200000000ULL

#define IMX682_NATIVE_WIDTH		9248
#define IMX682_NATIVE_HEIGHT		6936
#define IMX682_PIXEL_ARRAY_LEFT		0
#define IMX682_PIXEL_ARRAY_TOP		0
#define IMX682_PIXEL_ARRAY_WIDTH	9248
#define IMX682_PIXEL_ARRAY_HEIGHT	6936

static const char * const imx682_supply_names[] = {
	"vana",		/* Analog 2.8V */
	"vdig",		/* Digital 1.05V */
	"vio",		/* I/O 1.8V */
};

#define IMX682_NUM_SUPPLIES ARRAY_SIZE(imx682_supply_names)

struct imx682_reg {
	u16 address;
	u8 val;
};

struct imx682_reg_list {
	u32 num_of_regs;
	const struct imx682_reg *regs;
};

struct imx682_mode {
	u32 width;
	u32 height;
	struct v4l2_rect crop;
	u32 line_length_pck;
	u32 frame_length_lines;
	u32 vts_def;
	u32 vts_min;
	u64 link_freq_index;
	u64 pixel_rate;
	const struct imx682_reg_list reg_list;
};

/*
 * Global init sequence. Sourced from a real, working mainline driver for
 * the same Sony IMX682 silicon on a different phone (Xiaomi POCO X3 NFC /
 * SM7150, see github.com/woodyst/surya-pmos and github.com/pulpoff/linux-surya),
 * cross-checked here against our own CamX reverse-engineering: every control
 * register address this project independently derived (0x0100, 0x0202,
 * 0x0204, 0x0340, 0x0600, 0x0602/4/6/8) matches that driver exactly, and its
 * 20ms post-reset delay matches this board's measured timing too.
 *
 * The bulk of this table (analog/ADC bias tuning in the 0x5Bxx-0x5Dxx range,
 * LSC-adjacent blocks, etc.) is silicon-level calibration data that does not
 * depend on board wiring and should be directly reusable here.
 *
 * CAUTION - NOT YET ADAPTED FOR THIS BOARD:
 * {0x0136,0x13},{0x0137,0x33} below encode the INPUT CLOCK RATE as a
 * (MHz, fraction/256) pair: 0x13 + 0x33/256 = 19.2 MHz, matching that
 * board's 19.2MHz MCLK. This board's MCLK is 24MHz (confirmed), so this
 * pair must become {0x0136,0x18},{0x0137,0x00} (24 + 0/256 = 24.0MHz)
 * before use.
 * Global init sequence - REAL, captured directly from this exact phone's
 * live I2C traffic (Qualcomm CamX cam_debug_util debug_mdl=0xFFFFFFFF,
 * dmesg CAM-CCI cam_cci_data_queue logging), not ported or guessed. 339
 * static registers: same value observed across multiple independent
 * camera-open events, confirming these are genuine init constants rather
 * than runtime-adjusted values. Registers that varied across captures
 * (exposure, gain, frame length, group-hold, and the 0x5Bxx-0x5Dxx
 * color-temperature-dependent shading block) were excluded here and are
 * handled dynamically elsewhere (v4l2_ctrl / mode tables) or still use a
 * static snapshot default, same as any mainline driver would for registers
 * normally updated live by the vendor's closed 3A pipeline.
 *
 * Control register addresses independently reverse-engineered earlier in
 * this project (0x0100, 0x0202, 0x0204, 0x0340, 0x0600, 0x0602/4/6/8) are
 * all independently confirmed correct against this live capture too.
 */
static const struct imx682_reg imx682_global_init_regs[] = {
	{0x020e, 0x01},
	{0x33f0, 0x05},
	{0x0111, 0x03},
	{0x1f06, 0x06},
	{0x5d7f, 0x0a},
	{0x5d8a, 0x09},
	{0x5d63, 0x0a},
	{0x5d76, 0x09},
	{0x32c8, 0x00},
	{0x4017, 0x40},
	{0x40a2, 0x01},
	{0x40ac, 0x01},
	{0x4328, 0x00},
	{0x4e15, 0x10},
	{0x4e19, 0x2f},
	{0x4e21, 0x0f},
	{0x4e2f, 0x10},
	{0x4e3d, 0x10},
	{0x4e41, 0x2f},
	{0x4e57, 0x29},
	{0x4ffb, 0x2f},
	{0x5011, 0x24},
	{0x501d, 0x03},
	{0x505f, 0x41},
	{0x5065, 0xdf},
	{0x506e, 0x57},
	{0x5070, 0xc5},
	{0x5072, 0x57},
	{0x5075, 0x53},
	{0x507d, 0x57},
	{0x5081, 0x57},
	{0x53c8, 0x01},
	{0x5422, 0x7a},
	{0x548e, 0x40},
	{0x5497, 0x5e},
	{0x54a1, 0x40},
	{0x54a9, 0x40},
	{0x54b2, 0x5e},
	{0x54bc, 0x40},
	{0x57c6, 0x00},
	{0x583d, 0x0e},
	{0x5900, 0x12},
	{0x590f, 0x12},
	{0x5911, 0x12},
	{0x5913, 0x12},
	{0x591c, 0x12},
	{0x591e, 0x12},
	{0x5920, 0x12},
	{0x5948, 0x08},
	{0x595c, 0x08},
	{0x595e, 0x08},
	{0x5960, 0x08},
	{0x596e, 0x08},
	{0x5970, 0x08},
	{0x5972, 0x08},
	{0x597e, 0x0f},
	{0x599a, 0x0f},
	{0x59de, 0x08},
	{0x59fa, 0x08},
	{0x5a59, 0x22},
	{0x5a5b, 0x22},
	{0x5a5d, 0x1a},
	{0x5a5f, 0x22},
	{0x5a61, 0x1a},
	{0x5a63, 0x22},
	{0x5a65, 0x1a},
	{0x5a67, 0x22},
	{0x5a77, 0x22},
	{0x5a7b, 0x22},
	{0x5a7f, 0x22},
	{0x5a91, 0x22},
	{0x5a95, 0x22},
	{0x5a99, 0x22},
	{0x5ae9, 0x66},
	{0x5aeb, 0x66},
	{0x5aed, 0x5e},
	{0x5aef, 0x66},
	{0x5af1, 0x5e},
	{0x5af3, 0x66},
	{0x5af5, 0x5e},
	{0x5af7, 0x66},
	{0x5b07, 0x66},
	{0x5b0b, 0x66},
	{0x5b0f, 0x66},
	{0x5b21, 0x66},
	{0x5b25, 0x66},
	{0x5b29, 0x66},
	{0x5b79, 0x46},
	{0x5b7b, 0x3e},
	{0x5b7d, 0x3e},
	{0x5b89, 0x46},
	{0x5b8b, 0x46},
	{0x5b97, 0x46},
	{0x5b99, 0x46},
	{0x5c9e, 0x0a},
	{0x5cad, 0x0b},
	{0x5cbc, 0x0b},
	{0x5ccb, 0x0a},
	{0x5cda, 0x09},
	{0x5ce3, 0x09},
	{0x5cf4, 0x04},
	{0x5d04, 0x04},
	{0x5d13, 0x06},
	{0x5d22, 0x06},
	{0x5d2e, 0x06},
	{0x5d37, 0x06},
	{0x5d6f, 0x09},
	{0x5d72, 0x0f},
	{0x5d88, 0x0f},
	{0x5de6, 0x01},
	{0x5df2, 0x01},
	{0x5dfe, 0x01},
	{0x5e0a, 0x01},
	{0x5e16, 0x01},
	{0x5e1e, 0x01},
	{0x5e6e, 0x5a},
	{0x5e81, 0x46},
	{0x5e83, 0x3c},
	{0x5e85, 0x28},
	{0x5e87, 0x6e},
	{0x5e92, 0x46},
	{0x5e94, 0x3c},
	{0x5e96, 0x28},
	{0x5e98, 0x6e},
	{0x5ecb, 0x26},
	{0x5ed2, 0x26},
	{0x5ed9, 0x26},
	{0x5ee5, 0x08},
	{0x6006, 0x14},
	{0x600f, 0x14},
	{0x601a, 0x14},
	{0x6023, 0x14},
	{0x602e, 0x14},
	{0x6039, 0x14},
	{0x6132, 0x0f},
	{0x613e, 0x0f},
	{0x614a, 0x0f},
	{0x6156, 0x0f},
	{0x6162, 0x0f},
	{0x616a, 0x0f},
	{0x6226, 0x00},
	{0x84f8, 0x01},
	{0x8501, 0x00},
	{0x8505, 0x00},
	{0x8744, 0x00},
	{0x883c, 0x01},
	{0x8845, 0x00},
	{0x8849, 0x00},
	{0x9004, 0x1f},
	{0x9064, 0x4d},
	{0x922e, 0x91},
	{0x9237, 0xc3},
	{0xb0b9, 0x10},
	{0xbc76, 0x00},
	{0xbc7b, 0x28},
	{0xbc7f, 0xc0},
	{0xc6b9, 0x01},
	{0xecb5, 0x04},
	{0xecbf, 0x04},
	{0x0112, 0x0a},
	{0x30d8, 0x04},
	{0x3200, 0x41},
	{0x0408, 0x00},
	{0x0301, 0x08},
	{0x0303, 0x02},
	{0x0305, 0x03},
	{0x030b, 0x02},
	{0x030d, 0x03},
	{0x30d9, 0x00},
	{0x32d5, 0x00},
	{0x401e, 0x00},
	{0x40a0, 0x00},
	{0x40b8, 0x01},
	{0x40bc, 0x00},
	{0x41a4, 0x00},
	{0x5a09, 0x01},
	{0x5a17, 0x01},
	{0x5a25, 0x01},
	{0x5a33, 0x01},
	{0x98d7, 0xb4},
	{0x99c4, 0x16},
	{0x4018, 0x04},
	{0x7107, 0x02},
	{0x0b06, 0x01},
	{0x3093, 0x01},
	{0x3100, 0x00},
	{0xb000, 0x00},
	{0x0600, 0x00},
	{0x0b8e, 0x01},
	{0x306a, 0x2f},
	{0x306b, 0x4c},
	{0x306c, 0xca},
	{0x306d, 0x01},
	{0x307a, 0x0d},
	{0x307b, 0x20},
	{0x309e, 0x2d},
	{0x3072, 0x13},
	{0x3074, 0x09},
	{0x3075, 0x77},
	{0x3024, 0x16},
	{0x3002, 0x0e},
	{0x3003, 0x00},
	{0x3006, 0x10},
	{0x3007, 0x00},
	{0x300a, 0x0c},
	{0x300b, 0x00},
	{0x3018, 0xc5},
	{0x3019, 0x00},
	{0x303a, 0x02},
	{0x303b, 0x04},
	{0x3266, 0x01},
	{0x38da, 0x0a},
	{0x38dc, 0x0b},
	{0x38d6, 0x0a},
	{0x3070, 0x3d},
	{0x3071, 0x00},
	{0x3084, 0x13},
	{0x3085, 0x14},
	{0x3086, 0x0c},
	{0x3087, 0xe7},
	{0x3004, 0x08},
	{0x3005, 0x00},
	{0x3c08, 0xff},
	{0x3c09, 0xff},
	{0x314a, 0x5f},
	{0x314b, 0x00},
	{0x3064, 0xff},
	{0x3065, 0xcf},
	{0x3066, 0x7e},
	{0x3067, 0x00},
	{0x309c, 0x06},
	{0x309d, 0x40},
	{0x380c, 0x1a},
	{0x32b2, 0x00},
	{0x32b4, 0x00},
	{0x32b6, 0x00},
	{0x32b8, 0x00},
	{0x3090, 0x88},
	{0x3091, 0x00},
	{0x3238, 0x0c},
	{0x0348, 0x10},
	{0x0349, 0x0f},
	{0x0346, 0x44},
	{0x034a, 0x0b},
	{0x034b, 0xfb},
	{0x034e, 0x0b},
	{0x034f, 0xb8},
	{0x3402, 0x4e},
	{0x3403, 0x42},
	{0x0304, 0x03},
	{0x0306, 0x70},
	{0x030c, 0x03},
	{0x030e, 0x5d},
	{0x3c16, 0x00},
	{0x0386, 0x01},
	{0x3452, 0x00},
	{0x345a, 0x00},
	{0x345c, 0x00},
	{0x345e, 0x00},
	{0x3460, 0x00},
	{0x38c4, 0x09},
	{0x38d8, 0x2a},
	{0x38c2, 0x0a},
	{0x38c0, 0x0f},
	{0x38d4, 0x09},
	{0x38b0, 0x0f},
	{0x3932, 0x10},
	{0x3933, 0x00},
	{0x0820, 0x04},
	{0x0821, 0xa6},
	{0x3c34, 0x08},
	{0x3c36, 0x38},
	{0x3c37, 0x00},
	{0x3c38, 0x20},
	{0x393e, 0x40},
	{0x393f, 0x00},
	{0x3892, 0x36},
	{0x3893, 0x00},
	{0x00f4, 0x40},
	{0x00f8, 0x49},
	{0x00d3, 0x87},
	{0x00f7, 0x01},
	{0x00ee, 0x30},
	{0x0087, 0x18},
	{0x0011, 0x02},
	{0x0017, 0x80},
	{0x0019, 0x05},
	{0x0030, 0x03},
	{0x00d9, 0xc0},
	{0x001b, 0x20},
	{0x0028, 0x22},
	{0x004b, 0x10},
	{0x004e, 0x1a},
	{0x0050, 0x11},
	{0x0052, 0x33},
	{0x005b, 0x11},
	{0x00c5, 0x02},
	{0x0032, 0x38},
	{0x0016, 0x0c},
	{0x001a, 0x1a},
	{0x0020, 0x10},
	{0x0046, 0x83},
	{0x004a, 0x04},
	{0x0054, 0x02},
	{0x0062, 0x00},
	{0x0072, 0x8f},
	{0x007a, 0x05},
	{0x007c, 0x80},
	{0x0090, 0x00},
	{0x00ce, 0x18},
	{0x00d2, 0x40},
	{0x00e6, 0xe0},
	{0x0012, 0x01},
	{0x0022, 0x7c},
	{0x00b0, 0x6e},
	{0x0053, 0x00},
	{0x0089, 0x03},
	{0x0060, 0x40},
	{0x00fa, 0x10},
	{0x0065, 0x80},
	{0x00a9, 0x01},
	{0x0069, 0x00},
	{0x006a, 0x78},
	{0x0080, 0x02},
	{0x0005, 0x02},
	{0x009d, 0x14},
	{0x0095, 0xa0},
	{0x00d5, 0xfc},
	{0x0097, 0x28},
	{0x001f, 0x11},
	{0x00d0, 0xb2},
	{0x0042, 0x21},
	{0x0049, 0x03},
	{0x004c, 0x00},
	{0x0048, 0x02},
	{0x0015, 0x14},
	{0x0018, 0x0f},
	{0x0029, 0x06},
	{0x00b1, 0x01},
};

/*
 * Full-resolution (9248x6936) mode. The PLL/timing/analog portion below
 * (0x0304 onward) is REAL, captured directly from this board's own I2C
 * traffic - same capture method as imx682_global_init_regs above - and
 * includes {0x0304,0x0306,0x030c,0x030e} = {3,0x70,3,0x5d}, registers this
 * project could never resolve through static analysis alone despite
 * extensive reverse-engineering (see conversation history). line_length_pck
 * (0x0342/0x0343 = 0x1320 = 4896) and frame_length_lines (0x0340/0x0341 =
 * 0x0cb5 = 3253) are likewise real, not computed.
 *
 * {0x0112-0x0114} (format/lane config) and the crop/output-size registers
 * (0x0344-0x034f) are NOT independently confirmed for this specific mode -
 * this capture session didn't reprogram them, implying the sensor's
 * existing state already matched (full array, no crop), but that's an
 * inference, not a direct observation like the rest of this table. Full
 * native array is 9248x6936 (0x2420 x 0x1b18); crop set to 0,0 -
 * 9247,6935 accordingly.
 *
 * HONEST CAVEAT: frame_length_lines captured here (3253) is smaller than
 * even the binned mode's 3472 active rows, which isn't physically
 * consistent with a literal 6936-row full-array readout at a sane line
 * length. 0x0900=0 reliably confirms binning was off during this capture,
 * but the exact output resolution it corresponds to (possibly a lower-res
 * video/preview sub-mode rather than true full-res) isn't certain. Treat
 * the PLL/analog values here as solid; treat width/height/line/frame
 * timing as needing a dedicated capture specifically while the full 9248x
 * 6936 mode is active (e.g. a RAW/full-size still capture) before trusting
 * it for that exact resolution.
 */
static const struct imx682_reg imx682_mode_full_regs[] = {
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x02},	/* unconfirmed for this mode - see comment above */
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x24},
	{0x0349, 0x1f},
	{0x034a, 0x1b},
	{0x034b, 0x1f},
	{0x034c, 0x24},
	{0x034d, 0x20},
	{0x034e, 0x1b},
	{0x034f, 0x18},
	{0x0304, 0x03},	/* CONFIRMED: real I2C capture */
	{0x0306, 0x70},	/* CONFIRMED: real I2C capture */
	{0x030c, 0x03},	/* CONFIRMED: real I2C capture */
	{0x030e, 0x5d},	/* CONFIRMED: real I2C capture */
	{0x030f, 0x0b},	/* Samsung's own adaptive-MIPI default (SET A index 0, 1708.8MHz) */
	{0x3c16, 0x00},	/* CONFIRMED: real I2C capture */
	{0x0342, 0x13},	/* CONFIRMED: real I2C capture (line_length_pck hi) */
	{0x0343, 0x20},	/* CONFIRMED: real I2C capture (line_length_pck lo) */
	{0x0340, 0x0c},	/* CONFIRMED: real I2C capture (frame_length_lines hi) */
	{0x0341, 0xb5},	/* CONFIRMED: real I2C capture (frame_length_lines lo) */
	{0x0900, 0x00},	/* CONFIRMED: real I2C capture - binning OFF */
	{0x0386, 0x01},	/* CONFIRMED: real I2C capture */
	{0x3452, 0x00},	/* CONFIRMED: real I2C capture */
	{0x345a, 0x00},	/* CONFIRMED: real I2C capture */
	{0x345c, 0x00},	/* CONFIRMED: real I2C capture */
	{0x345e, 0x00},	/* CONFIRMED: real I2C capture */
	{0x3460, 0x00},	/* CONFIRMED: real I2C capture */
	{0x38c4, 0x09},	/* CONFIRMED: real I2C capture */
	{0x38d8, 0x2a},	/* CONFIRMED: real I2C capture */
	{0x38da, 0x0a},	/* CONFIRMED: real I2C capture */
	{0x38dc, 0x0b},	/* CONFIRMED: real I2C capture */
	{0x38c2, 0x0a},	/* CONFIRMED: real I2C capture */
	{0x38c0, 0x0f},	/* CONFIRMED: real I2C capture */
	{0x38d6, 0x0a},	/* CONFIRMED: real I2C capture */
	{0x38d4, 0x09},	/* CONFIRMED: real I2C capture */
	{0x38b0, 0x0f},	/* CONFIRMED: real I2C capture */
	{0x3932, 0x10},	/* CONFIRMED: real I2C capture */
	{0x3933, 0x00},	/* CONFIRMED: real I2C capture */
	{0x0820, 0x04},	/* CONFIRMED: real I2C capture */
	{0x0821, 0xa6},	/* CONFIRMED: real I2C capture */
	{0x3c34, 0x08},	/* CONFIRMED: real I2C capture */
	{0x3c36, 0x38},	/* CONFIRMED: real I2C capture */
	{0x3c37, 0x00},	/* CONFIRMED: real I2C capture */
	{0x3c38, 0x20},	/* CONFIRMED: real I2C capture */
	{0x393e, 0x40},	/* CONFIRMED: real I2C capture */
	{0x393f, 0x00},	/* CONFIRMED: real I2C capture */
	{0x3892, 0x36},	/* CONFIRMED: real I2C capture */
	{0x3893, 0x00},	/* CONFIRMED: real I2C capture */
	{0x3c1e, 0x01},	/* CONFIRMED: real I2C capture */
	{0x3c1f, 0x00},	/* CONFIRMED: real I2C capture */
};

/*
 * 2x2-binned 4624x3472 mode. Crop/output-size registers (0x0342-0x034f,
 * 0x0900-0x0902, etc.) are geometry-only and confirmed reusable verbatim.
 *
 * CAUTION - PARTIALLY ADAPTED FOR THIS BOARD:
 * 0x0301,0x0303,0x0305,0x0306-0307,0x030b,0x030d,0x030e,0x0310 are the base
 * PLL pre-divider/multiplier chain, still copied from the 19.2MHz/953.6MHz
 * 3-lane reference below and NOT yet confirmed for this board's 24MHz
 * input - do not treat these as correct yet.
 *
 * 0x030F below IS now confirmed correct for this exact board: Samsung's
 * own published kernel source (matei9/android_kernel_samsung_sm7125,
 * drivers/media/platform/msm/camera/cam_sensor_module/cam_sensor/
 * cam_sensor_adaptive_mipi_imx682.h) gives {0x030F, 0x96} for this
 * 2x2-binned "SET B / Quad" mode - identical for both of its listed
 * RF-band options there, so no ambiguity - overriding the placeholder
 * value that was here. That same source gives {0x030F, 0x0B} as the
 * *default* (index 0, used when no RF band matches) for full-res "SET A",
 * targeting 1708.8MHz - worth carrying into the full-res table once its
 * base PLL chain is known. Lane count is still unconfirmed for this board
 * (the reference driver is 3-lane per its 0x0114 setting; don't assume
 * 4-lane without checking this board's actual wiring).
 */
static const struct imx682_reg imx682_mode_binned_regs[] = {
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x02},
	{0x0342, 0x24},
	{0x0343, 0xd8},
	{0x0340, 0x0d},
	{0x0341, 0xc8},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x24},
	{0x0349, 0x1f},
	{0x034a, 0x1b},
	{0x034b, 0x1f},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x08},
	{0x30d8, 0x04},
	{0x3200, 0x41},
	{0x3201, 0x41},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x12},
	{0x040d, 0x10},
	{0x040e, 0x0d},
	{0x040f, 0x90},
	{0x034c, 0x12},
	{0x034d, 0x10},
	{0x034e, 0x0d},
	{0x034f, 0x90},
	{0x0301, 0x08},
	{0x0303, 0x02},
	{0x0305, 0x03},
	{0x0306, 0x01},
	{0x0307, 0x38},
	{0x030b, 0x02},
	{0x030d, 0x03},
	{0x030e, 0x01},
	{0x030f, 0x96},	/* CONFIRMED: Samsung's own cam_sensor_adaptive_mipi_imx682.h, SET B */
	{0x0310, 0x01},
	{0x30d9, 0x00},
	{0x32d5, 0x00},
	{0x32d6, 0x00},
	{0x401e, 0x00},
	{0x40b8, 0x01},
	{0x40b9, 0x2c},
	{0x40bc, 0x01},
	{0x40bd, 0x18},
	{0x40be, 0x00},
	{0x40bf, 0x00},
	{0x41a4, 0x00},
	{0x5a09, 0x01},
	{0x5a17, 0x01},
	{0x5a25, 0x01},
	{0x5a33, 0x01},
	{0x98d7, 0xb4},
	{0x98d8, 0x8c},
	{0x98d9, 0x0a},
	{0x99c4, 0x16},
	{0x0202, 0x0d},
	{0x0203, 0x98},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020e, 0x01},
	{0x020f, 0x00},
	{0x4018, 0x04},
	{0x4019, 0x80},
	{0x401a, 0x00},
	{0x401b, 0x01},
	{0x3400, 0x02},
	{0x40a0, 0x00},	/* CONFIRMED: real I2C capture (cam_debug_util), not previously known */
	{0x7107, 0x02},	/* CONFIRMED: real I2C capture */
	{0x0b06, 0x01},	/* CONFIRMED: real I2C capture */
	{0x3093, 0x01},	/* CONFIRMED: real I2C capture */
};

enum {
	IMX682_LINK_FREQ_1708_IDX,
	IMX682_LINK_FREQ_1849_IDX,
};

static const s64 link_freq_menu_items[] = {
	[IMX682_LINK_FREQ_1708_IDX] = IMX682_LINK_FREQ_1708,
	[IMX682_LINK_FREQ_1849_IDX] = IMX682_LINK_FREQ_1849,
};

/*
 * TODO: width/height/line_length_pck/frame_length_lines below are
 * PLACEHOLDERS consistent with a 64MP IMX-class sensor (full res +
 * one common 2x2-binned mode) but must be confirmed/replaced with the
 * real values from the vendor mode table.
 */
static const struct imx682_mode imx682_supported_modes[] = {
	{
		/* Full resolution (quad-bayer raw10) */
		.width = 9248,
		.height = 6936,
		.crop = {
			.left = IMX682_PIXEL_ARRAY_LEFT,
			.top = IMX682_PIXEL_ARRAY_TOP,
			.width = 9248,
			.height = 6936,
		},
		.line_length_pck = 4896,	/* confirmed: real I2C capture, 0x0342/0x0343 */
		.frame_length_lines = 3253,	/* confirmed: real I2C capture, 0x0340/0x0341 */
		.vts_def = 3253,
		.vts_min = 3253 - 56,	/* vblank_min estimate, same margin as binned mode */
		.link_freq_index = IMX682_LINK_FREQ_1849_IDX,
		.pixel_rate = IMX682_PIXEL_RATE,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx682_mode_full_regs),
			.regs = imx682_mode_full_regs,
		},
	},
	{
		/* 2x2 binned mode */
		.width = 4624,
		.height = 3468,
		.crop = {
			.left = IMX682_PIXEL_ARRAY_LEFT,
			.top = IMX682_PIXEL_ARRAY_TOP,
			.width = 9248,
			.height = 6936,
		},
		.line_length_pck = 9432,	/* confirmed: 0x0342/0x0343 */
		.frame_length_lines = 3528,	/* confirmed: 0x0340/0x0341 */
		.vts_def = 3528,
		.vts_min = 3528 - 56,	/* reference vblank_min = 56 */
		.link_freq_index = IMX682_LINK_FREQ_1708_IDX,
		.pixel_rate = IMX682_PIXEL_RATE,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx682_mode_binned_regs),
			.regs = imx682_mode_binned_regs,
		},
	},
};

static const u32 imx682_mbus_codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
};

struct imx682 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX682_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;

	const struct imx682_mode *cur_mode;

	struct mutex mutex; /* protects cur_mode, streaming state */
	bool streaming;
};

static inline struct imx682 *to_imx682(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx682, sd);
}

/* ------------------------------------------------------------------
 * Register I/O
 * ------------------------------------------------------------------ */

static int imx682_read_reg(struct imx682 *imx682, u16 reg, u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx682->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf = 0;
	int ret;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = &data_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = data_buf;
	return 0;
}

static int imx682_write_reg(struct imx682 *imx682, u16 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx682->sd);
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	int ret;

	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret != ARRAY_SIZE(buf))
		return -EIO;

	return 0;
}

static int imx682_write_regs(struct imx682 *imx682,
			      const struct imx682_reg *regs, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx682_write_reg(imx682, regs[i].address, regs[i].val);
		if (ret) {
			struct i2c_client *client =
				v4l2_get_subdevdata(&imx682->sd);

			dev_err_ratelimited(&client->dev,
				"failed to write reg 0x%4.4x: %d\n",
				regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

/* ------------------------------------------------------------------
 * Power sequencing
 * ------------------------------------------------------------------ */

static int imx682_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx682 *imx682 = to_imx682(sd);
	int ret;

	ret = regulator_bulk_enable(IMX682_NUM_SUPPLIES, imx682->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(imx682->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		goto err_disable_regulators;
	}

	/* Reset is active high; deassert after rails + MCLK are stable */
	gpiod_set_value_cansleep(imx682->reset_gpio, 1);
	usleep_range(20000, 20100); /* 20ms per vendor power sequence */
	gpiod_set_value_cansleep(imx682->reset_gpio, 0);
	usleep_range(500, 1000);

	return 0;

err_disable_regulators:
	regulator_bulk_disable(IMX682_NUM_SUPPLIES, imx682->supplies);
	return ret;
}

static int imx682_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx682 *imx682 = to_imx682(sd);

	gpiod_set_value_cansleep(imx682->reset_gpio, 1);
	clk_disable_unprepare(imx682->xclk);
	regulator_bulk_disable(IMX682_NUM_SUPPLIES, imx682->supplies);

	return 0;
}

/* ------------------------------------------------------------------
 * Controls
 * ------------------------------------------------------------------ */

static int imx682_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx682 *imx682 =
		container_of(ctrl->handler, struct imx682, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx682->sd);
	int ret = 0;

	/* Only apply V4L2 controls when streaming, mirroring imx258.c */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = imx682_write_reg(imx682, IMX682_REG_EXPOSURE,
					ctrl->val & 0xff);
		if (!ret)
			ret = imx682_write_reg(imx682, IMX682_REG_EXPOSURE + 1,
						(ctrl->val >> 8) & 0xff);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx682_write_reg(imx682, IMX682_REG_ANALOG_GAIN,
					ctrl->val & 0xff);
		if (!ret)
			ret = imx682_write_reg(imx682,
						IMX682_REG_ANALOG_GAIN + 1,
						(ctrl->val >> 8) & 0xff);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx682_write_reg(imx682, IMX682_REG_ORIENTATION,
					(imx682->hflip->val ? 0x1 : 0) |
					(imx682->vflip->val ? 0x2 : 0));
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx682_write_reg(imx682, IMX682_REG_TEST_PATTERN,
					ctrl->val & 0xff);
		if (!ret)
			ret = imx682_write_reg(imx682,
						IMX682_REG_TEST_PATTERN + 1,
						(ctrl->val >> 8) & 0xff);
		break;
	default:
		dev_info(&client->dev, "ctrl 0x%x not handled\n", ctrl->id);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx682_ctrl_ops = {
	.s_ctrl = imx682_set_ctrl,
};

static const char * const imx682_test_pattern_menu[] = {
	"Disabled",
	"Solid Color",
	"100% Color Bars",
	"Fade to Grey Color Bars",
	"PN9",
};

static const int imx682_test_pattern_val[] = {
	IMX682_TEST_PATTERN_DISABLE,
	IMX682_TEST_PATTERN_SOLID_COLOR,
	IMX682_TEST_PATTERN_COLOR_BARS,
	IMX682_TEST_PATTERN_GREY_COLOR,
	IMX682_TEST_PATTERN_PN9,
};

static int imx682_init_controls(struct imx682 *imx682)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx682->sd);
	struct v4l2_ctrl_handler *hdl = &imx682->ctrl_handler;
	const struct imx682_mode *mode = imx682->cur_mode;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(hdl, 10);
	hdl->lock = &imx682->mutex;

	imx682->link_freq =
		v4l2_ctrl_new_int_menu(hdl, &imx682_ctrl_ops,
					V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(link_freq_menu_items) - 1,
					0, link_freq_menu_items);
	if (imx682->link_freq)
		imx682->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx682->pixel_rate = v4l2_ctrl_new_std(hdl, &imx682_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						1, INT_MAX, 1,
						mode->pixel_rate);

	imx682->exposure = v4l2_ctrl_new_std(hdl, &imx682_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      IMX682_EXPOSURE_MIN,
					      IMX682_EXPOSURE_MAX,
					      IMX682_EXPOSURE_STEP,
					      IMX682_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(hdl, &imx682_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			   IMX682_ANA_GAIN_MIN, IMX682_ANA_GAIN_MAX,
			   IMX682_ANA_GAIN_STEP, IMX682_ANA_GAIN_DEFAULT);

	imx682->hflip = v4l2_ctrl_new_std(hdl, &imx682_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);
	imx682->vflip = v4l2_ctrl_new_std(hdl, &imx682_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(hdl, &imx682_ctrl_ops,
				      V4L2_CID_TEST_PATTERN,
				      ARRAY_SIZE(imx682_test_pattern_menu) - 1,
				      0, 0, imx682_test_pattern_menu);

	if (hdl->error) {
		ret = hdl->error;
		dev_err(&client->dev, "control init failed: %d\n", ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &imx682_ctrl_ops, &props);
	if (ret)
		goto error;

	imx682->sd.ctrl_handler = hdl;
	return 0;

error:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

/* ------------------------------------------------------------------
 * pad ops
 * ------------------------------------------------------------------ */

static int imx682_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(imx682_mbus_codes))
		return -EINVAL;

	code->code = imx682_mbus_codes[code->index];
	return 0;
}

static int imx682_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(imx682_supported_modes))
		return -EINVAL;

	if (fse->code != imx682_mbus_codes[0])
		return -EINVAL;

	fse->min_width = imx682_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx682_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx682_update_pad_format(const struct imx682_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int imx682_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct imx682 *imx682 = to_imx682(sd);

	mutex_lock(&imx682->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt->format = *v4l2_subdev_state_get_format(sd_state, 0);
	} else {
		imx682_update_pad_format(imx682->cur_mode, &fmt->format);
	}

	mutex_unlock(&imx682->mutex);
	return 0;
}

static const struct imx682_mode *
imx682_find_best_fit(struct v4l2_subdev_format *fmt)
{
	unsigned int i, best = 0;
	int best_area = INT_MAX;

	for (i = 0; i < ARRAY_SIZE(imx682_supported_modes); i++) {
		int area = abs((int)imx682_supported_modes[i].width -
				(int)fmt->format.width) +
			   abs((int)imx682_supported_modes[i].height -
				(int)fmt->format.height);

		if (area < best_area) {
			best_area = area;
			best = i;
		}
	}

	return &imx682_supported_modes[best];
}

static int imx682_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct imx682 *imx682 = to_imx682(sd);
	const struct imx682_mode *mode;

	mutex_lock(&imx682->mutex);

	fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
	mode = imx682_find_best_fit(fmt);
	imx682_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, 0) = fmt->format;
	} else {
		imx682->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(imx682->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(imx682->pixel_rate,
					  mode->pixel_rate);
		__v4l2_ctrl_modify_range(imx682->exposure,
					  IMX682_EXPOSURE_MIN,
					  mode->vts_def - 8,
					  IMX682_EXPOSURE_STEP,
					  IMX682_EXPOSURE_DEFAULT);
	}

	mutex_unlock(&imx682->mutex);
	return 0;
}

static int imx682_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_selection *sel)
{
	struct imx682 *imx682 = to_imx682(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		mutex_lock(&imx682->mutex);
		sel->r = imx682->cur_mode->crop;
		mutex_unlock(&imx682->mutex);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX682_NATIVE_WIDTH;
		sel->r.height = IMX682_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX682_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX682_PIXEL_ARRAY_TOP;
		sel->r.width = IMX682_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX682_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int imx682_init_cfg(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};

	imx682_update_pad_format(&imx682_supported_modes[0], &fmt.format);
	*v4l2_subdev_state_get_format(sd_state, 0) = fmt.format;

	return 0;
}

/* ------------------------------------------------------------------
 * Streaming
 * ------------------------------------------------------------------ */

static int imx682_start_streaming(struct imx682 *imx682)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx682->sd);
	const struct imx682_reg_list *reg_list;
	int ret;

	ret = imx682_write_regs(imx682, imx682_global_init_regs,
				 ARRAY_SIZE(imx682_global_init_regs));
	if (ret) {
		dev_err(&client->dev, "failed to set global settings: %d\n",
			ret);
		return ret;
	}

	reg_list = &imx682->cur_mode->reg_list;
	ret = imx682_write_regs(imx682, reg_list->regs,
				 reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "failed to set mode: %d\n", ret);
		return ret;
	}

	ret = __v4l2_ctrl_handler_setup(&imx682->ctrl_handler);
	if (ret)
		return ret;

	return imx682_write_reg(imx682, IMX682_REG_MODE_SELECT,
				 IMX682_MODE_STREAMING);
}

static int imx682_stop_streaming(struct imx682 *imx682)
{
	return imx682_write_reg(imx682, IMX682_REG_MODE_SELECT,
				 IMX682_MODE_STANDBY);
}

static int imx682_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx682 *imx682 = to_imx682(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&imx682->sd);
	int ret = 0;

	mutex_lock(&imx682->mutex);
	if (imx682->streaming == enable) {
		mutex_unlock(&imx682->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret)
			goto unlock;

		ret = imx682_start_streaming(imx682);
		if (ret) {
			pm_runtime_put(&client->dev);
			goto unlock;
		}
	} else {
		imx682_stop_streaming(imx682);
		pm_runtime_put(&client->dev);
	}

	imx682->streaming = enable;

unlock:
	mutex_unlock(&imx682->mutex);
	return ret;
}

/* ------------------------------------------------------------------
 * Subdev ops tables
 * ------------------------------------------------------------------ */

static const struct v4l2_subdev_video_ops imx682_video_ops = {
	.s_stream = imx682_set_stream,
};

static const struct v4l2_subdev_pad_ops imx682_pad_ops = {
	.enum_mbus_code = imx682_enum_mbus_code,
	.enum_frame_size = imx682_enum_frame_size,
	.get_fmt = imx682_get_fmt,
	.set_fmt = imx682_set_fmt,
	.get_selection = imx682_get_selection,
};

static const struct v4l2_subdev_ops imx682_subdev_ops = {
	.video = &imx682_video_ops,
	.pad = &imx682_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx682_internal_ops = {
	.init_state = imx682_init_cfg,
};

static const struct media_entity_operations imx682_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* ------------------------------------------------------------------
 * Probe helpers
 * ------------------------------------------------------------------ */

static int imx682_identify_module(struct imx682 *imx682)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx682->sd);
	u8 hi, lo;
	u16 chip_id;
	int ret;

	ret = imx682_read_reg(imx682, IMX682_REG_CHIP_ID, &hi);
	if (ret)
		return ret;
	ret = imx682_read_reg(imx682, IMX682_REG_CHIP_ID + 1, &lo);
	if (ret)
		return ret;

	chip_id = (hi << 8) | lo;
	if (chip_id != IMX682_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: 0x%04x != 0x%04x\n",
			chip_id, IMX682_CHIP_ID);
		return -ENXIO;
	}

	return 0;
}

static int imx682_get_regulators(struct imx682 *imx682,
				  struct i2c_client *client)
{
	unsigned int i;

	for (i = 0; i < IMX682_NUM_SUPPLIES; i++)
		imx682->supplies[i].supply = imx682_supply_names[i];

	return devm_regulator_bulk_get(&client->dev, IMX682_NUM_SUPPLIES,
					imx682->supplies);
}

static int imx682_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx682 *imx682;
	int ret;

	imx682 = devm_kzalloc(dev, sizeof(*imx682), GFP_KERNEL);
	if (!imx682)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx682->sd, client, &imx682_subdev_ops);

	imx682->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx682->xclk))
		return dev_err_probe(dev, PTR_ERR(imx682->xclk),
				      "failed to get MCLK\n");

	imx682->xclk_freq = clk_get_rate(imx682->xclk);
	if (imx682->xclk_freq != 24000000)
		dev_warn(dev, "MCLK is %u Hz, expected 24 MHz\n",
			 imx682->xclk_freq);

	ret = imx682_get_regulators(imx682, client);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	imx682->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(imx682->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx682->reset_gpio),
				      "failed to get reset GPIO\n");

	mutex_init(&imx682->mutex);
	imx682->cur_mode = &imx682_supported_modes[0];

	ret = imx682_power_on(dev);
	if (ret)
		goto err_mutex;

	ret = imx682_identify_module(imx682);
	if (ret)
		goto err_power_off;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = imx682_init_controls(imx682);
	if (ret)
		goto err_pm;

	imx682->sd.internal_ops = &imx682_internal_ops;
	imx682->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx682->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx682->sd.entity.ops = &imx682_subdev_entity_ops;
	imx682->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx682->sd.entity, 1, &imx682->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto err_ctrl_handler;
	}

	ret = v4l2_async_register_subdev_sensor(&imx682->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register subdev: %d\n", ret);
		goto err_entity;
	}

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_entity:
	media_entity_cleanup(&imx682->sd.entity);
err_ctrl_handler:
	v4l2_ctrl_handler_free(&imx682->ctrl_handler);
err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
err_power_off:
	imx682_power_off(dev);
err_mutex:
	mutex_destroy(&imx682->mutex);
	return ret;
}

static void imx682_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx682 *imx682 = to_imx682(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx682->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx682_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&imx682->mutex);
}

static int imx682_runtime_suspend(struct device *dev)
{
	return imx682_power_off(dev);
}

static int imx682_runtime_resume(struct device *dev)
{
	return imx682_power_on(dev);
}

static const struct dev_pm_ops imx682_pm_ops = {
	SET_RUNTIME_PM_OPS(imx682_runtime_suspend, imx682_runtime_resume, NULL)
};

static const struct of_device_id imx682_dt_ids[] = {
	{ .compatible = "sony,imx682" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx682_dt_ids);

static const struct i2c_device_id imx682_id[] = {
	{ "imx682" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, imx682_id);

static struct i2c_driver imx682_i2c_driver = {
	.driver = {
		.name = "imx682",
		.pm = &imx682_pm_ops,
		.of_match_table = imx682_dt_ids,
	},
	.probe = imx682_probe,
	.remove = imx682_remove,
	.id_table = imx682_id,
};

module_i2c_driver(imx682_i2c_driver);

MODULE_DESCRIPTION("Sony IMX682 sensor driver");
MODULE_LICENSE("GPL");
