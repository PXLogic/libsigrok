/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
 * Copyright (C) 2013 DreamSourceLab <support@dreamsourcelab.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_DREAMSOURCELAB_DSLOGIC_PROTOCOL_H
#define LIBSIGROK_HARDWARE_DREAMSOURCELAB_DSLOGIC_PROTOCOL_H

#include <glib.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include <sys/stat.h>

#include "command.h"

#define LOG_PREFIX "dreamsourcelab-dslogic"

#define USB_INTERFACE		0
#define USB_CONFIGURATION	1
#define MAX_RENUM_DELAY_MS	3000
#define NUM_SIMUL_TRANSFERS	64
#define MAX_EMPTY_POLL		16

#define NUM_CHANNELS		16
#define NUM_TRIGGER_STAGES	16

#define DSL_REQUIRED_VERSION_MAJOR	2
#define DSL_REQUIRED_VERSION_MINOR	0
#define DSL_HDL_VERSION		0x0E

#define DS_VENDOR_ID		0x2A0E

/* --- Fork-compatible macros (not in upstream libsigrok 0.6.0) ---
 * Upstream has SR_HZ/KHZ/MHZ/GHZ, SR_mV/V, SR_NS/US.
 * Fork-only: SR_KB/MB/GB (memory size), SR_Kn/Mn/Gn (kilo/mega/giga-nano). */
#ifndef SR_KB
#define SR_KB(n) ((n) * (uint64_t)(1024ULL))
#endif
#ifndef SR_MB
#define SR_MB(n) ((n) * (uint64_t)(1048576ULL))
#endif
#ifndef SR_GB
#define SR_GB(n) ((n) * (uint64_t)(1073741824ULL))
#endif
#ifndef SR_Kn
#define SR_Kn(n) ((n) * (uint64_t)(1000ULL))
#endif
#ifndef SR_Mn
#define SR_Mn(n) ((n) * (uint64_t)(1000000ULL))
#endif
#ifndef SR_Gn
#define SR_Gn(n) ((n) * (uint64_t)(1000000000ULL))
#endif

/* --- Capability feature bits --- */
#define CAPS_FEATURE_NONE		0
#define CAPS_FEATURE_VTH		(1 << 0)	/* voltage threshold */
#define CAPS_FEATURE_BUF		(1 << 1)	/* with external buffer */
#define CAPS_FEATURE_PREOFF		(1 << 2)	/* pre offset control */
#define CAPS_FEATURE_SEEP		(1 << 3)	/* small startup eeprom */
#define CAPS_FEATURE_ZERO		(1 << 4)	/* zero calibration ability */
#define CAPS_FEATURE_HMCAD1511		(1 << 5)	/* use HMCAD1511 adc chip */
#define CAPS_FEATURE_USB30		(1 << 6)	/* usb 3.0 */
#define CAPS_FEATURE_POGOPIN		(1 << 7)	/* pogopin panel */
#define CAPS_FEATURE_ADF4360		(1 << 8)	/* use ADF4360-7 vco chip */
#define CAPS_FEATURE_20M		(1 << 9)	/* 20M bandwidth limitation */
#define CAPS_FEATURE_FLASH		(1 << 10)	/* use startup flash (fx3) */
#define CAPS_FEATURE_LA_CH32		(1 << 11)	/* 32 channels */
#define CAPS_FEATURE_AUTO_VGAIN		(1 << 12)	/* auto tunning vgain */
#define CAPS_FEATURE_MAX25_VTH		(1 << 13)	/* max 2.5v fpga threshold */
#define CAPS_FEATURE_SECURITY		(1 << 14)	/* security check */

/* --- Atomic sample constants --- */
#define DSLOGIC_ATOMIC_BITS		6
#define DSLOGIC_ATOMIC_SAMPLES		(1 << DSLOGIC_ATOMIC_BITS)
#define DSLOGIC_ATOMIC_SIZE		(1 << (DSLOGIC_ATOMIC_BITS - 3))
#define DSLOGIC_ATOMIC_MASK		(0xFFFF << DSLOGIC_ATOMIC_BITS)

/* --- Basic configuration bits --- */
#define TRIG_EN_BIT		0
#define CLK_TYPE_BIT		1
#define CLK_EDGE_BIT		2
#define RLE_MODE_BIT		3
#define DSO_MODE_BIT		4
#define HALF_MODE_BIT		5
#define QUAR_MODE_BIT		6
#define ANALOG_MODE_BIT		7
#define FILTER_BIT		8
#define INSTANT_BIT		9
#define SLOW_ACQ_BIT		10
#define STRIG_MODE_BIT		11
#define STREAM_MODE_BIT		12
#define LPB_TEST_BIT		13
#define EXT_TEST_BIT		14
#define INT_TEST_BIT		15

/* --- Bit masks --- */
#define bmNONE			0
#define bmEEWP			(1 << 0)
#define bmFORCE_RDY		(1 << 1)
#define bmFORCE_STOP		(1 << 2)
#define bmSCOPE_SET		(1 << 3)
#define bmSCOPE_CLR		(1 << 4)
#define bmBW20M_SET		(1 << 5)
#define bmBW20M_CLR		(1 << 6)

/* --- Packet content check --- */
#define TRIG_CHECKID		0x55555555
#define DSO_PKTID		0xa500

/* --- Zero configuration --- */
#define DSO_ZERO_PAGE		8
#define MAX_ACC_VARIANCE	0.0005

/* --- DSCope offset constants --- */
#define DSCOPE_CONSTANT_BIAS	160
#define DSCOPE_TRANS_CMULTI	10
#define DSCOPE_TRANS_FMULTI	100.0
#define CALI_VGAIN_RANGE	200

/* --- Operation modes (fork enum OPERATION_MODE / DSLOGIC_OPERATION_MODE) --- */
enum dsl_operation_mode {
	DSL_MODE_LOGIC		= 0,
	DSL_MODE_DSO		= 1,
	DSL_MODE_ANALOG		= 2,
	DSL_MODE_UNKNOWN	= 99,
};

enum dsl_op_mode {
	LO_OP_BUFFER		= 0,
	LO_OP_STREAM		= 1,
	LO_OP_INTEST		= 2,
	LO_OP_EXTEST		= 3,
	LO_OP_LPTEST		= 4,
};

/* --- DSL_CHANNEL_ID enum (fork libsigrok.h:1223) --- */
enum DSL_CHANNEL_ID {
	DSL_STREAM20x16		= 0,
	DSL_STREAM25x12		= 1,
	DSL_STREAM50x6		= 2,
	DSL_STREAM100x3		= 3,

	DSL_STREAM20x16_3DN2	= 4,
	DSL_STREAM25x12_3DN2	= 5,
	DSL_STREAM50x6_3DN2	= 6,
	DSL_STREAM100x3_3DN2	= 7,

	DSL_STREAM10x32_32_3DN2	= 8,
	DSL_STREAM20x16_32_3DN2	= 9,
	DSL_STREAM25x12_32_3DN2	= 10,
	DSL_STREAM50x6_32_3DN2	= 11,
	DSL_STREAM100x3_32_3DN2	= 12,

	DSL_STREAM50x32		= 13,
	DSL_STREAM100x30	= 14,
	DSL_STREAM250x12	= 15,
	DSL_STREAM125x16_16	= 16,
	DSL_STREAM250x12_16	= 17,
	DSL_STREAM500x6		= 18,
	DSL_STREAM1000x3		= 19,

	DSL_BUFFER100x16	= 20,
	DSL_BUFFER200x8		= 21,
	DSL_BUFFER400x4		= 22,

	DSL_BUFFER250x32	= 23,
	DSL_BUFFER500x16	= 24,
	DSL_BUFFER1000x8	= 25,

	DSL_ANALOG10x2		= 26,
	DSL_ANALOG10x2_500	= 27,

	DSL_DSO200x2		= 28,
	DSL_DSO1000x2		= 29,
};

/* --- DSL stop status enum --- */
enum {
	DSL_ERROR	= -1,
	DSL_INIT	= 0,
	DSL_START	= 1,
	DSL_READY	= 2,
	DSL_TRIGGERED	= 3,
	DSL_DATA	= 4,
	DSL_STOP	= 5,
	DSL_FINISH	= 7,
	DSL_ABORT	= 8,
};

/* --- Trigger position struct (fork struct ds_trigger_pos) --- */
struct dsl_trigger_pos {
	uint32_t check_id;
	uint32_t real_pos;
	uint32_t ram_saddr;
	uint32_t remain_cnt_l;
	uint32_t remain_cnt_h;
	uint32_t status;
	uint8_t first_block[488];
};

/* --- Status struct (fork struct sr_status, ported as dsl_status) --- */
struct dsl_status {
	uint8_t trig_hit;
	uint8_t captured_cnt0;
	uint8_t captured_cnt1;
	uint8_t captured_cnt2;
	uint8_t captured_cnt3;
	uint8_t stream_mode;
	uint8_t sample_divider_tog;
	uint8_t trig_flag;
	uint8_t trig_ch;
	uint16_t pkt_id;
	uint32_t vlen;
	uint32_t trig_offset;
	uint32_t sample_divider_tog2;
	double ch0_acc_mean;
	double ch0_acc_mean_p1;
	double ch0_acc_mean_p2;
	double ch0_acc_mean_p3;
	double ch1_acc_mean;
	double ch1_acc_mean_p1;
	double ch1_acc_mean_p2;
	double ch1_acc_mean_p3;
};

/* --- Device capability/profile structs --- */
struct DSL_caps {
	uint64_t mode_caps;
	uint64_t feature_caps;
	uint64_t channels;
	uint64_t total_ch_num;
	uint64_t hw_depth;
	uint64_t dso_depth;
	uint8_t intest_channel;
	const uint64_t *vdivs;
	const uint64_t *samplerates;
	uint8_t vga_id;
	uint16_t default_channelmode;
	uint64_t default_samplerate;
	uint64_t default_samplelimit;
	uint16_t default_pwmtrans;
	uint16_t default_pwmmargin;
	uint32_t ref_min;
	uint32_t ref_max;
	uint16_t default_comb_comp;
	uint64_t half_samplerate;
	uint64_t quarter_samplerate;
};

struct DSL_profile {
	uint16_t vid;
	uint16_t pid;
	enum libusb_speed usb_speed;

	const char *vendor;
	const char *model;
	const char *model_version;

	const char *firmware;

	const char *fpga_bit33;
	const char *fpga_bit50;

	struct DSL_caps dev_caps;
};

/* --- Mode capability bits --- */
#define CAPS_MODE_LOGIC		(1 << 0)
#define CAPS_MODE_ANALOG	(1 << 1)
#define CAPS_MODE_DSO		(1 << 2)

/* --- Voltage divider presets (mV) --- */
static const uint64_t vdivs10to2000[] = {
	SR_mV(10),
	SR_mV(20),
	SR_mV(50),
	SR_mV(100),
	SR_mV(200),
	SR_mV(500),
	SR_V(1),
	SR_V(2),
	0,
};

/* --- Samplerate presets --- */
static const uint64_t samplerates100[] = {
	SR_HZ(10),   SR_HZ(20),   SR_HZ(50),   SR_HZ(100),
	SR_HZ(200),  SR_HZ(500),  SR_KHZ(1),   SR_KHZ(2),
	SR_KHZ(5),   SR_KHZ(10),  SR_KHZ(20),  SR_KHZ(40),
	SR_KHZ(50),  SR_KHZ(100), SR_KHZ(200), SR_KHZ(400),
	SR_KHZ(500), SR_MHZ(1),  SR_MHZ(2),   SR_MHZ(4),
	SR_MHZ(5),   SR_MHZ(10), SR_MHZ(20),  SR_MHZ(25),
	SR_MHZ(50),  SR_MHZ(100),
	0,
};

static const uint64_t samplerates400[] = {
	SR_HZ(10),   SR_HZ(20),   SR_HZ(50),   SR_HZ(100),
	SR_HZ(200),  SR_HZ(500),  SR_KHZ(1),   SR_KHZ(2),
	SR_KHZ(5),   SR_KHZ(10),  SR_KHZ(20),  SR_KHZ(40),
	SR_KHZ(50),  SR_KHZ(100), SR_KHZ(200), SR_KHZ(400),
	SR_KHZ(500), SR_MHZ(1),   SR_MHZ(2),   SR_MHZ(4),
	SR_MHZ(5),   SR_MHZ(10),  SR_MHZ(20),  SR_MHZ(25),
	SR_MHZ(50),  SR_MHZ(100), SR_MHZ(200), SR_MHZ(400),
	0,
};

static const uint64_t samplerates1000[] = {
	SR_HZ(10),   SR_HZ(20),   SR_HZ(50),   SR_HZ(100),
	SR_HZ(200),  SR_HZ(500),  SR_KHZ(1),   SR_KHZ(2),
	SR_KHZ(5),   SR_KHZ(10),  SR_KHZ(20),  SR_KHZ(40),
	SR_KHZ(50),  SR_KHZ(100), SR_KHZ(200), SR_KHZ(400),
	SR_KHZ(500), SR_MHZ(1),   SR_MHZ(2),   SR_MHZ(4),
	SR_MHZ(5),   SR_MHZ(10),  SR_MHZ(20),  SR_MHZ(25),
	SR_MHZ(50),  SR_MHZ(100), SR_MHZ(125), SR_MHZ(250),
	SR_MHZ(500), SR_GHZ(1),
	0,
};

/* --- VGA defaults --- */
struct DSL_vga {
	uint8_t id;
	uint64_t key;
	uint64_t vgain;
	uint16_t preoff;
	uint16_t preoff_comp;
};

static const struct DSL_vga vga_defaults[] = {
	{1, 10,   0x162400, (32<<10)+558, (32<<10)+558},
	{1, 20,   0x14C000, (32<<10)+558, (32<<10)+558},
	{1, 50,   0x12E800, (32<<10)+558, (32<<10)+558},
	{1, 100,  0x118000, (32<<10)+558, (32<<10)+558},
	{1, 200,  0x102400, (32<<10)+558, (32<<10)+558},
	{1, 500,  0x2E800,  (32<<10)+558, (32<<10)+558},
	{1, 1000, 0x18000,  (32<<10)+558, (32<<10)+558},
	{1, 2000, 0x02400,  (32<<10)+558, (32<<10)+558},

	{2, 10,   0x1DA800, 45, 1024-920-45},
	{2, 20,   0x1A7200, 45, 1024-920-45},
	{2, 50,   0x164200, 45, 1024-920-45},
	{2, 100,  0x131800, 45, 1024-920-45},
	{2, 200,  0xBD000,  45, 1024-920-45},
	{2, 500,  0x7AD00,  45, 1024-920-45},
	{2, 1000, 0x48800,  45, 1024-920-45},
	{2, 2000, 0x12000,  45, 1024-920-45},

	{3, 10,   0x1C5C00, 45, 1024-920-45},
	{3, 20,   0x19EB00, 45, 1024-920-45},
	{3, 50,   0x16AE00, 45, 1024-920-45},
	{3, 100,  0x143D00, 45, 1024-920-45},
	{3, 200,  0xB1000,  45, 1024-920-45},
	{3, 500,  0x7F000,  45, 1024-920-45},
	{3, 1000, 0x57200,  45, 1024-920-45},
	{3, 2000, 0x2DD00,  45, 1024-920-45},

	{4, 10,   0x1C6C00, 60, 1024-900-60},
	{4, 20,   0x19E000, 60, 1024-900-60},
	{4, 50,   0x16A800, 60, 1024-900-60},
	{4, 100,  0x142800, 60, 1024-900-60},
	{4, 200,  0xC7F00,  60, 1024-900-60},
	{4, 500,  0x94000,  60, 1024-900-60},
	{4, 1000, 0x6CF00,  60, 1024-900-60},
	{4, 2000, 0x44F00,  60, 1024-900-60},

	{5, 10,   0x1C3400, 60, 1024-900-60},
	{5, 20,   0x19BD00, 60, 1024-900-60},
	{5, 50,   0x167400, 60, 1024-900-60},
	{5, 100,  0x13F300, 60, 1024-900-60},
	{5, 200,  0xC4F00,  60, 1024-900-60},
	{5, 500,  0x91B00,  60, 1024-900-60},
	{5, 1000, 0x69D00,  60, 1024-900-60},
	{5, 2000, 0x41D00,  60, 1024-900-60},

	{0, 0, 0, 0, 0}
};

/* --- Channel modes --- */
struct DSL_channels {
	enum DSL_CHANNEL_ID id;
	int mode;		/* enum dsl_operation_mode */
	int type;		/* SR_CHANNEL_LOGIC / SR_CHANNEL_DSO / SR_CHANNEL_ANALOG */
	gboolean stream;
	uint16_t num;
	uint16_t vld_num;
	uint8_t unit_bits;
	uint64_t min_samplerate;
	uint64_t max_samplerate;
	uint64_t hw_min_samplerate;
	uint64_t hw_max_samplerate;
	uint8_t pre_div;
	const char *descr;
};

static const struct DSL_channels channel_modes[] = {
	/* LA Stream */
	{DSL_STREAM20x16,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 16, 16, 1, SR_KHZ(50), SR_MHZ(20),
		SR_KHZ(10), SR_MHZ(100), 1, "Use 16 Channels (Max 20MHz)"},
	{DSL_STREAM25x12,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 16, 12, 1, SR_KHZ(50), SR_MHZ(25),
		SR_KHZ(10), SR_MHZ(100), 1, "Use 12 Channels (Max 25MHz)"},
	{DSL_STREAM50x6,   DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 16, 6,  1, SR_KHZ(50), SR_MHZ(50),
		SR_KHZ(10), SR_MHZ(100), 1, "Use 6 Channels (Max 50MHz)"},
	{DSL_STREAM100x3,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 16, 3,  1, SR_KHZ(50), SR_MHZ(100),
		SR_KHZ(10), SR_MHZ(100), 1, "Use 3 Channels (Max 100MHz)"},

	{DSL_STREAM20x16_3DN2,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 16, 16, 1, SR_KHZ(100), SR_MHZ(20),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 16 Channels (Max 20MHz)"},
	{DSL_STREAM25x12_3DN2,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 16, 12, 1, SR_KHZ(100), SR_MHZ(25),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 12 Channels (Max 25MHz)"},
	{DSL_STREAM50x6_3DN2,   DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 16, 6,  1, SR_KHZ(100), SR_MHZ(50),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 6 Channels (Max 50MHz)"},
	{DSL_STREAM100x3_3DN2,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 16, 3,  1, SR_KHZ(100), SR_MHZ(100),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 3 Channels (Max 100MHz)"},

	{DSL_STREAM10x32_32_3DN2,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 32, 32, 1, SR_KHZ(100), SR_MHZ(10),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 32 Channels (Max 10MHz)"},
	{DSL_STREAM20x16_32_3DN2,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 32, 16, 1, SR_KHZ(100), SR_MHZ(20),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 16 Channels (Max 20MHz)"},
	{DSL_STREAM25x12_32_3DN2,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 32, 12, 1, SR_KHZ(100), SR_MHZ(25),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 12 Channels (Max 25MHz)"},
	{DSL_STREAM50x6_32_3DN2,   DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 32, 6,  1, SR_KHZ(100), SR_MHZ(50),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 6 Channels (Max 50MHz)"},
	{DSL_STREAM100x3_32_3DN2,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 32, 3,  1, SR_KHZ(100), SR_MHZ(100),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 3 Channels (Max 100MHz)"},

	{DSL_STREAM50x32,   DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 32, 32, 1, SR_MHZ(1), SR_MHZ(50),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 32 Channels (Max 50MHz)"},
	{DSL_STREAM100x30,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 32, 30, 1, SR_MHZ(1), SR_MHZ(100),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 30 Channels (Max 100MHz)"},
	{DSL_STREAM250x12,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 32, 12, 1, SR_MHZ(1), SR_MHZ(250),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 12 Channels (Max 250MHz)"},
	{DSL_STREAM125x16_16, DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 16, 16, 1, SR_MHZ(1), SR_MHZ(125),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 16 Channels (Max 125MHz)"},
	{DSL_STREAM250x12_16, DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 16, 12, 1, SR_MHZ(1), SR_MHZ(250),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 12 Channels (Max 250MHz)"},
	{DSL_STREAM500x6,   DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 16, 6,  1, SR_MHZ(1), SR_MHZ(500),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 6 Channels (Max 500MHz)"},
	{DSL_STREAM1000x3,   DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, TRUE, 8, 3,  1, SR_MHZ(1), SR_GHZ(1),
		SR_KHZ(10), SR_MHZ(500), 5, "Use 3 Channels (Max 1GHz)"},

	/* LA Buffer */
	{DSL_BUFFER100x16, DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, FALSE, 16, 16, 1, SR_KHZ(50), SR_MHZ(100),
		SR_KHZ(10), SR_MHZ(100), 1, "Use Channels 0~15 (Max 100MHz)"},
	{DSL_BUFFER200x8,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, FALSE, 8, 8,  1, SR_KHZ(50), SR_MHZ(200),
		SR_KHZ(10), SR_MHZ(100), 1, "Use Channels 0~7 (Max 200MHz)"},
	{DSL_BUFFER400x4,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, FALSE, 4, 4,  1, SR_KHZ(50), SR_MHZ(400),
		SR_KHZ(10), SR_MHZ(100), 1, "Use Channels 0~3 (Max 400MHz)"},

	{DSL_BUFFER250x32,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, FALSE, 32, 32, 1, SR_MHZ(1), SR_MHZ(250),
		SR_KHZ(10), SR_MHZ(500), 5, "Use Channels 0~31 (Max 250MHz)"},
	{DSL_BUFFER500x16,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, FALSE, 16, 16, 1, SR_MHZ(1), SR_MHZ(500),
		SR_KHZ(10), SR_MHZ(500), 5, "Use Channels 0~15 (Max 500MHz)"},
	{DSL_BUFFER1000x8,  DSL_MODE_LOGIC, SR_CHANNEL_LOGIC, FALSE, 8, 8,  1, SR_MHZ(1), SR_GHZ(1),
		SR_KHZ(10), SR_MHZ(500), 5, "Use Channels 0~7 (Max 1GHz)"},

	/* DAQ */
	{DSL_ANALOG10x2,   DSL_MODE_ANALOG, SR_CHANNEL_ANALOG, TRUE, 2, 2, 8, SR_HZ(10), SR_MHZ(10),
		SR_KHZ(10), SR_MHZ(100), 1, "Use Channels 0~1 (Max 10MHz)"},
	{DSL_ANALOG10x2_500, DSL_MODE_ANALOG, SR_CHANNEL_ANALOG, TRUE, 2, 2, 8, SR_HZ(10), SR_MHZ(10),
		SR_KHZ(10), SR_MHZ(500), 1, "Use Channels 0~1 (Max 10MHz)"},

	/* OSC */
	{DSL_DSO200x2,    DSL_MODE_DSO, SR_CHANNEL_DSO, FALSE, 2, 2, 8, SR_KHZ(10), SR_MHZ(200),
		SR_KHZ(10), SR_MHZ(100), 1, "Use Channels 0~1 (Max 200MHz)"},
	{DSL_DSO1000x2,   DSL_MODE_DSO, SR_CHANNEL_DSO, FALSE, 2, 2, 8, SR_KHZ(10), SR_GHZ(1),
		SR_KHZ(10), SR_MHZ(500), 1, "Use Channels 0~1 (Max 1GHz)"}
};

/* --- Hardware setting for each capture --- */
struct DSL_setting {
	uint32_t sync;

	uint16_t mode_header;
	uint16_t mode;
	uint16_t divider_header;
	uint16_t div_l;
	uint16_t div_h;
	uint16_t count_header;
	uint16_t cnt_l;
	uint16_t cnt_h;
	uint16_t trig_pos_header;
	uint16_t tpos_l;
	uint16_t tpos_h;
	uint16_t trig_glb_header;
	uint16_t trig_glb;
	uint16_t dso_count_header;
	uint16_t dso_cnt_l;
	uint16_t dso_cnt_h;
	uint16_t ch_en_header;
	uint16_t ch_en_l;
	uint16_t ch_en_h;
	uint16_t fgain_header;
	uint16_t fgain;

	uint16_t trig_header;
	uint16_t trig_mask0[NUM_TRIGGER_STAGES];
	uint16_t trig_mask1[NUM_TRIGGER_STAGES];
	uint16_t trig_value0[NUM_TRIGGER_STAGES];
	uint16_t trig_value1[NUM_TRIGGER_STAGES];
	uint16_t trig_edge0[NUM_TRIGGER_STAGES];
	uint16_t trig_edge1[NUM_TRIGGER_STAGES];
	uint16_t trig_logic0[NUM_TRIGGER_STAGES];
	uint16_t trig_logic1[NUM_TRIGGER_STAGES];
	uint32_t trig_count[NUM_TRIGGER_STAGES];

	uint32_t end_sync;
};

struct DSL_setting_ext32 {
	uint32_t sync;

	uint16_t trig_header;
	uint16_t trig_mask0[NUM_TRIGGER_STAGES];
	uint16_t trig_mask1[NUM_TRIGGER_STAGES];
	uint16_t trig_value0[NUM_TRIGGER_STAGES];
	uint16_t trig_value1[NUM_TRIGGER_STAGES];
	uint16_t trig_edge0[NUM_TRIGGER_STAGES];
	uint16_t trig_edge1[NUM_TRIGGER_STAGES];

	uint16_t align_bytes;
	uint32_t end_sync;
};

/* --- ADC config (HMCAD1511) --- */
struct DSL_adc_config {
	uint8_t dest;
	uint8_t cnt;
	uint8_t delay;
	uint8_t byte[4];
};

static const struct DSL_adc_config adc_single_ch0[] = {
	{ADCC_ADDR+1, 3,   0,   {0x03, 0x01, 0x00, 0x00}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x01, 0x00, 0x31}},
	{ADCC_ADDR+1, 1,   0,   {0x01, 0x00, 0x00, 0x00}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x02, 0x02, 0x3A}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x02, 0x02, 0x3B}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x00, 0x00, 0x42}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x34, 0x00, 0x50}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x22, 0x02, 0x11}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x7F, 0x00, 0x24}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x00, 0x00, 0x55}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x02, 0x00, 0x33}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x00, 0x03, 0x2B}},
	{0, 0, 0, {0, 0, 0, 0}}
};

static const struct DSL_adc_config adc_single_ch3[] = {
	{ADCC_ADDR+1, 3,   0,   {0x03, 0x01, 0x00, 0x00}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x01, 0x00, 0x31}},
	{ADCC_ADDR+1, 1,   0,   {0x01, 0x00, 0x00, 0x00}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x10, 0x10, 0x3A}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x10, 0x10, 0x3B}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x00, 0x00, 0x42}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x34, 0x00, 0x50}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x22, 0x02, 0x11}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x00, 0x00, 0x24}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x00, 0x00, 0x55}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x02, 0x00, 0x33}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x00, 0x03, 0x2B}},
	{0, 0, 0, {0, 0, 0, 0}}
};

static const struct DSL_adc_config adc_dual_ch03[] = {
	{ADCC_ADDR+1, 3,   0,   {0x03, 0x01, 0x00, 0x00}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x02, 0x01, 0x31}},
	{ADCC_ADDR+1, 1,   0,   {0x01, 0x00, 0x00, 0x00}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x02, 0x02, 0x3A}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x10, 0x10, 0x3B}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x00, 0x00, 0x42}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x34, 0x00, 0x50}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x22, 0x02, 0x11}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x11, 0x00, 0x24}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x00, 0x00, 0x55}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x02, 0x00, 0x33}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x33, 0x00, 0x2B}},
	{0, 0, 0, {0, 0, 0, 0}}
};

static const struct DSL_adc_config adc_init_fix[] = {
	{ADCC_ADDR+1, 3,   0,   {0x03, 0x01, 0x00, 0x00}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x02, 0x01, 0x31}},
	{ADCC_ADDR+1, 1,   0,   {0x01, 0x00, 0x00, 0x00}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x02, 0x02, 0x3A}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x10, 0x10, 0x3B}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x00, 0x00, 0x42}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x34, 0x00, 0x50}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x22, 0x02, 0x11}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x10, 0x00, 0x25}},
	{ADCC_ADDR,   4,   0,   {0x00, 0x00, 0x55, 0x26}},
	{0, 0, 0, {0, 0, 0, 0}}
};

static const struct DSL_adc_config adc_clk_init_1g[] = {
	{ADCC_ADDR+2, 1,   0,   {0x01, 0x00, 0x00, 0x00}},
	{ADCC_ADDR,   4,   0,   {0x01, 0x61, 0x00, 0x30}},
	{ADCC_ADDR,   4,   0,   {0x01, 0x40, 0xF1, 0x46}},
	{ADCC_ADDR,   4,   10,  {0x01, 0x62, 0x3D, 0x00}},
	{0, 0, 0, {0, 0, 0, 0}}
};

static const struct DSL_adc_config adc_clk_init_500m[] = {
	{ADCC_ADDR+2, 1,   0,   {0x01, 0x00, 0x00, 0x00}},
	{ADCC_ADDR,   4,   0,   {0x01, 0x61, 0x00, 0x30}},
	{ADCC_ADDR,   4,   0,   {0x01, 0x40, 0xF1, 0x46}},
	{ADCC_ADDR,   4,   10,  {0x01, 0x62, 0x3D, 0x40}},
	{0, 0, 0, {0, 0, 0, 0}}
};

static const struct DSL_adc_config adc_power_down[] = {
	{ADCC_ADDR+1, 1,   0,   {0x00, 0x00, 0x00, 0x00}},
	{0, 0, 0, {0, 0, 0, 0}}
};

static const struct DSL_adc_config adc_power_up[] = {
	{ADCC_ADDR+1, 1,   0,   {0x01, 0x00, 0x00, 0x00}},
	{0, 0, 0, {0, 0, 0, 0}}
};

/* --- Channel private data (replaces fork sr_channel fork-only fields) ---
 * Upstream sr_channel only has index/type/enabled/name/priv.
 * All DSO/ANALOG-specific fields go here, accessed via ch->priv. */
struct dsl_channel_priv {
	uint8_t bits;
	uint64_t vdiv;
	uint64_t vfactor;
	uint16_t offset;
	uint16_t zero_offset;
	uint16_t hw_offset;
	uint16_t vpos_trans;
	uint8_t coupling;
	uint8_t trig_value;
	int8_t comb_diff_top;
	int8_t comb_diff_bom;
	int8_t comb_comp;
	uint16_t digi_fgain;

	double cali_fgain0;
	double cali_fgain1;
	double cali_fgain2;
	double cali_fgain3;
	double cali_comb_fgain0;
	double cali_comb_fgain1;
	double cali_comb_fgain2;
	double cali_comb_fgain3;

	gboolean map_default;
	int map_unit;
	double map_min;
	double map_max;

	gboolean vgain_change;
	gboolean offset_change;
	const struct DSL_vga *vga_ptr;
};

#define DSL_CH_PRIV(ch) ((struct dsl_channel_priv *)(ch->priv))

/* --- Device context (expanded from fork DSL_context) --- */
struct dev_context {
	const struct DSL_profile *profile;

	int64_t fw_updated;

	/* Device/capture settings */
	uint64_t cur_samplerate;
	uint64_t limit_samples;
	uint64_t actual_samples;
	uint64_t actual_bytes;

	/* Operational settings */
	gboolean clock_type;
	gboolean clock_edge;
	gboolean rle_mode;
	gboolean rle_support;
	gboolean instant;
	uint16_t op_mode;
	gboolean stream;
	uint8_t test_mode;
	uint16_t buf_options;
	enum DSL_CHANNEL_ID ch_mode;
	uint16_t samplerates_min_index;
	uint16_t samplerates_max_index;
	uint16_t th_level;
	double vth;
	uint16_t filter;
	uint16_t trigger_mask[NUM_TRIGGER_STAGES];
	uint16_t trigger_value[NUM_TRIGGER_STAGES];
	int trigger_stage;
	uint16_t trigger_buffer[NUM_TRIGGER_STAGES];
	uint64_t trigger_pos;
	uint64_t timebase;
	uint8_t max_height;
	uint8_t trigger_channel;
	uint8_t trigger_slope;
	uint8_t trigger_source;
	uint8_t trigger_hrate;
	uint32_t trigger_hpos;
	uint64_t trigger_holdoff;
	uint8_t trigger_margin;
	gboolean zero;
	gboolean cali;
	gboolean tune;
	int16_t tune_index;
	int zero_stage;
	int zero_pcnt;
	gboolean zero_branch;
	gboolean zero_comb_fgain;
	gboolean zero_comb;
	int tune_stage;
	int tune_pcnt;
	struct sr_channel *tune_probe;
	gboolean roll;
	gboolean data_lock;
	uint16_t unit_pitch;

	uint64_t num_samples;
	uint64_t num_bytes;
	int submitted_transfers;
	int empty_transfer_count;
	int instant_tail_bytes;

	void *cb_data;
	unsigned int num_transfers;
	struct libusb_transfer **transfers;
	int *usbfd;

	int pipe_fds[2];
	GIOChannel *channel;

	int status;
	int trf_completed;
	gboolean mstatus_valid;
	struct dsl_status mstatus;
	gboolean abort;
	gboolean overflow;
	int bw_limit;
	int empty_poll_count;

	int is_loop;

	/* Port-added fields (replace fork sr_dev_inst fields not in upstream) */
	int mode;			/* replaces fork sdi->mode (LOGIC/DSO/ANALOG) */
	uint64_t capture_ratio;
	uint64_t sent_samples;
	gboolean acq_aborted;
	gboolean external_clock;
	gboolean continuous_mode;
	struct sr_context *ctx;
	uint16_t *deinterleave_buffer;
	/* Pattern mode for DSO/ANALOG waveform selection (GUI compat — stored
	 * as a string index but the actual waveform comes from the hardware,
	 * so this is a no-op for real devices. Mirrors demo driver's enum. */
	uint8_t pattern_mode;
};

/* --- Function declarations --- */
SR_PRIV int dsl_adjust_probes(struct sr_dev_inst *sdi, int num_probes);
SR_PRIV int dsl_setup_probes(struct sr_dev_inst *sdi, int num_probes);
SR_PRIV const GSList *dsl_mode_list(const struct sr_dev_inst *sdi);
SR_PRIV void dsl_adjust_samplerate(struct dev_context *devc);

/* --- "has the FX2 firmware already been uploaded?" identity ---------------
 * A device that has been flashed re-enumerates as manufacturer
 * "DreamSourceLab" plus one of these product strings. Both must be accepted
 * and BOTH are defined here rather than inline, because the same fact is
 * needed in two places (scan()'s has_firmware probe and
 * dsl_check_conf_profile()) and drifting apart is precisely what broke this
 * driver once already.
 *
 *   DSL_PROD_STRING_DSVIEW -- declared by every DreamSourceLab FX2 blob this
 *       project ships. Verified by decoding the UTF-16LE string descriptors
 *       inside DSLogic.fw / DSLogicPro.fw / DSCope.fw / DSCope20.fw.
 *   DSL_PROD_STRING_UPSTREAM -- declared by the older upstream sigrok
 *       fx2 firmware (fx2lafw lineage), accepted for compatibility.
 *
 * Comparisons against these are exact (strcmp semantics), so the shorter
 * upstream string does NOT match the longer DreamSourceLab one. */
#define DSL_MANUF_STRING		"DreamSourceLab"
#define DSL_PROD_STRING_DSVIEW		"USB-based DSL Instrument v2"
#define DSL_PROD_STRING_UPSTREAM	"USB-based Instrument"

SR_PRIV int dsl_en_ch_num(const struct sr_dev_inst *sdi);
SR_PRIV gboolean dsl_check_conf_profile(libusb_device *dev);
SR_PRIV int dsl_configure_probes(const struct sr_dev_inst *sdi);
SR_PRIV uint64_t dsl_channel_depth(const struct sr_dev_inst *sdi);

SR_PRIV int dsl_wr_reg(const struct sr_dev_inst *sdi, uint8_t addr, uint8_t value);
SR_PRIV int dsl_rd_reg(const struct sr_dev_inst *sdi, uint8_t addr, uint8_t *value);
SR_PRIV int dsl_wr_ext(const struct sr_dev_inst *sdi, uint8_t addr, uint8_t value);
SR_PRIV int dsl_rd_ext(const struct sr_dev_inst *sdi, unsigned char *ctx, uint16_t addr, uint8_t len);

SR_PRIV int dsl_wr_dso(const struct sr_dev_inst *sdi, uint64_t cmd);
SR_PRIV int dsl_wr_nvm(const struct sr_dev_inst *sdi, unsigned char *ctx, uint16_t addr, uint8_t len);
SR_PRIV int dsl_rd_nvm(const struct sr_dev_inst *sdi, unsigned char *ctx, uint16_t addr, uint8_t len);
SR_PRIV int dsl_rd_probe(const struct sr_dev_inst *sdi, unsigned char *ctx, uint16_t addr, uint8_t len);

SR_PRIV int dsl_config_adc(const struct sr_dev_inst *sdi, const struct DSL_adc_config *config);
SR_PRIV double dsl_adc_code2fgain(uint8_t code);
SR_PRIV uint8_t dsl_adc_fgain2code(double gain);
SR_PRIV int dsl_config_adc_fgain(const struct sr_dev_inst *sdi, uint8_t branch, double gain0, double gain1);
SR_PRIV int dsl_config_fpga_fgain(const struct sr_dev_inst *sdi);
SR_PRIV int dsl_skew_fpga_fgain(const struct sr_dev_inst *sdi, gboolean comb, double skew[]);
SR_PRIV int dsl_probe_cali_fgain(struct dev_context *devc, struct sr_channel *probe, double mean, gboolean comb, gboolean reset);
SR_PRIV gboolean dsl_probe_fgain_inrange(struct sr_channel *probe, gboolean comb, double skew[]);

SR_PRIV int dsl_fpga_arm(const struct sr_dev_inst *sdi);
SR_PRIV int dsl_fpga_config(struct libusb_device_handle *hdl, const char *filename);

/* DSO command IDs for dslogic_dso_cmd_gen() (fork-only, not upstream SR_CONF_*). */
#define DSO_CMD_PROBE_VDIV		0x1001
#define DSO_CMD_PROBE_EN		0x1002
#define DSO_CMD_TIMEBASE		0x1003
#define DSO_CMD_PROBE_COUPLING		0x1004
#define DSO_CMD_SAMPLERATE		0x1005
#define DSO_CMD_HORIZ_TRIGGERPOS	0x1006
#define DSO_CMD_TRIGGER_SLOPE		0x1007
#define DSO_CMD_TRIGGER_SOURCE		0x1008
#define DSO_CMD_TRIGGER_VALUE		0x1009
#define DSO_CMD_TRIGGER_MARGIN		0x100A
#define DSO_CMD_TRIGGER_HOLDOFF		0x100B
#define DSO_CMD_DSO_SYNC			0x100C

/* Coupling values (match coupling_strs[] in api.c: GND=0, DC=1, AC=2). */
#define DSL_COUPLING_GND	0
#define DSL_COUPLING_DC		1
#define DSL_COUPLING_AC		2

/* DSO command generation and initialization (ported from fork dslogic.c). */
SR_PRIV uint64_t dslogic_dso_cmd_gen(const struct sr_dev_inst *sdi,
	struct sr_channel *ch, int cmd_id);
SR_PRIV int dslogic_dso_init(const struct sr_dev_inst *sdi);

SR_PRIV int dsl_config_get(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi,
			const struct sr_channel_group *cg);
SR_PRIV int dsl_config_set(uint32_t key, GVariant *data, struct sr_dev_inst *sdi,
			const struct sr_channel_group *cg);
SR_PRIV int dsl_config_list(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi,
			const struct sr_channel_group *cg);

SR_PRIV int dsl_dev_open(struct sr_dev_driver *di, struct sr_dev_inst *sdi, gboolean *fpga_done);
SR_PRIV int dsl_dev_close(struct sr_dev_inst *sdi);
SR_PRIV int dsl_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data);
SR_PRIV int dsl_dev_status_get(const struct sr_dev_inst *sdi, struct dsl_status *status, gboolean prg);

SR_PRIV unsigned int dsl_get_timeout(const struct sr_dev_inst *sdi);
SR_PRIV int dsl_start_transfers(const struct sr_dev_inst *sdi);
SR_PRIV int dsl_header_size(const struct dev_context *devc);

SR_PRIV int dsl_destroy_device(struct sr_dev_inst *sdi);
SR_PRIV int dsl_secuCheck(const struct sr_dev_inst *sdi, uint16_t *encryption, int steps);

SR_PRIV struct dev_context *dslogic_dev_new(void);
SR_PRIV int dslogic_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di);
SR_PRIV int dslogic_fpga_firmware_upload(const struct sr_dev_inst *sdi);
SR_PRIV int dslogic_set_voltage_threshold(const struct sr_dev_inst *sdi, double threshold);
SR_PRIV int dslogic_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV int dslogic_acquisition_stop(struct sr_dev_inst *sdi);

/* --- Firmware presence diagnostics -----------------------------------------
 * A missing FX2 .fw makes the device never re-enumerate, but libusb has
 * already opened its handle inside scan(), so the user-visible symptom is the
 * misleading "device busy / driver problem" pair instead of the real cause.
 * These helpers detect the gap up-front and report it actionably.
 */
SR_PRIV gboolean dslogic_firmware_exists(struct sr_context *sr_ctx, const char *name);
SR_PRIV void dslogic_report_missing_firmware(struct sr_context *sr_ctx,
		const struct DSL_profile *prof, const char *name);

#endif
