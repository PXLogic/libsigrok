/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2010 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Olivier Fauchon <olivier@aixmarseille.com>
 * Copyright (C) 2012 Alexandru Gagniuc <mr.nuke.me@gmail.com>
 * Copyright (C) 2015 Bartosz Golaszewski <bgolaszewski@baylibre.com>
 * Copyright (C) 2019 Frank Stettner <frank-stettner@gmx.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_DEMO_PROTOCOL_H
#define LIBSIGROK_HARDWARE_DEMO_PROTOCOL_H

#include <stdint.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include <minizip/unzip.h>
#include <glib.h>

#define LOG_PREFIX "demo"

/* The size in bytes of chunks to send through the session bus.
 *
 * 1 MB: matches pxlogic's approach of sending large packets (up to 4MB
 * per USB transfer). Each sr_session_send call has significant overhead
 * (vector heap-alloc + copy, mutex lock/unlock, condition variable
 * notify, 2 Qt async events). With 64KB, a 2M-sample tick at 32 channels
 * produced 122 calls — now it produces only 8.
 *
 * The dev_context struct embeds two arrays of this size (logic_data +
 * cross_data_buf = 2 MB total). This is fine for a heap-allocated struct. */
#define LOGIC_BUFSIZE			(1024 * 1024)
/* Size of the analog pattern space per channel. */
#define ANALOG_BUFSIZE			4096
/* This is a development feature: it starts a new frame every n samples. */
#define SAMPLES_PER_FRAME		1000UL
#define DEFAULT_LIMIT_FRAMES		0

#define DEFAULT_ANALOG_ENCODING_DIGITS	4
#define DEFAULT_ANALOG_SPEC_DIGITS		4
#define DEFAULT_ANALOG_AMPLITUDE		10
#define DEFAULT_ANALOG_OFFSET			0.

/* Protocol waveform generators (PATTERN_I2C / PATTERN_MIXED):
 * independent samples-per-bit (SPB) per protocol so each bus keeps its own
 * bit timing (they previously shared I2C_BITTIME=10, which corrupted SPI/UART
 * sampling). At 1 MHz sample rate:
 *   I2C  50 → 20 kHz bus, 4-phase structure (start/stop/data/hold) fits
 *   SPI  40 → 25 kHz SCLK (20 samples high / 20 low)
 *   UART 80 → 12.5 kHz, midpoint sampling with comfortable tolerance
 *   CAN/I2S/SWD/MIPI 40 → generic default bit time
 * The MIPI/DSI generator ignores bittime; these are still passed for
 * uniformity. */
#define I2C_SPB				50
#define SPI_SPB				40
#define UART_SPB			80
#define CAN_SPB				40
#define I2S_SPB				40
#define SWD_SPB				40
#define MIPI_SPB			40

/* Cyclic random buffer length for PATTERN_ANALOG_RANDOM streaming. The buffer
 * is pre-filled once by init_analog_random_data() and read cyclically by
 * send_analog_packet(), mirroring old fork demo's analog random streaming. */
#define ANALOG_RANDOM_BUF_LEN			4096

/* DSO defaults (ported from PXView fork demo). */
#define DEFAULT_NUM_DSO_CHANNELS		2
#define DSO_PACKET_LEN			20000
#define DSO_SAMPLE_BITS			8
#define DSO_DEFAULT_VDIV			1000
#define DSO_DEFAULT_HW_OFFSET		128
#define DSO_DEFAULT_OFFSET			128
#define DSO_DEFAULT_TRIG_VAL		128
#define DSO_DEFAULT_COUPLING		1  /* SR_DC_COUPLING */
#define DSO_MAX_CHANNELS			16

/* Logic patterns we can generate. */
enum logic_pattern_type {
	/**
	 * Spells "sigrok" across 8 channels using '0's (with '1's as
	 * "background") when displayed using the 'bits' output format.
	 * The pattern is repeated every 8 channels, shifted to the right
	 * in time by one bit.
	 */
	PATTERN_SIGROK,

	/** Pseudo-random values on all channels. */
	PATTERN_RANDOM,

	/**
	 * Incrementing number across 8 channels. The pattern is repeated
	 * every 8 channels, shifted to the right in time by one bit.
	 */
	PATTERN_INC,

	/**
	 * Single bit "walking" across all logic channels by being
	 * shifted across data lines, restarting after the last line
	 * was used. An all-zero (all-one) state is inserted to prevent
	 * repetitive patterns (e.g. with 8 data lines, every 8th state
	 * would show the same line state)
	 */
	PATTERN_WALKING_ONE,
	PATTERN_WALKING_ZERO,

	/** All channels have a low logic state. */
	PATTERN_ALL_LOW,

	/** All channels have a high logic state. */
	PATTERN_ALL_HIGH,

	/**
	 * Mimics a cable squid. Derived from the "works with" logo
	 * to occupy a larger number of channels yet "painting"
	 * something that can get recognized.
	 */
	PATTERN_SQUID,

	/** Gray encoded data, like rotary encoder signals. */
	PATTERN_GRAYCODE,

	/**
	 * I2C bus traffic on ch0(SCL)/ch1(SDA). Generates START,
	 * 7-bit address + R/W, ACK, data bytes, STOP, and repeated
	 * START. Address increments each transaction; data bytes
	 * increment within each transaction. This produces decodable
	 * I2C traffic for protocol decoder testing.
	 */
	PATTERN_I2C,

	/**
	 * Mixed protocol pattern: 8 independent protocol buses packed into
	 * ch0-15, mirrored to ch16-31 so all 32 logic channels carry data.
	 * mixed_get_sample() in protocol.c is the authoritative layout.
	 *
	 * Channel layout (bit N of the sample = chN):
	 *   ch0-1:   I2C      (SCL, SDA)
	 *   ch2-5:   SPI      (CS, SCLK, MOSI, MISO)
	 *   ch6:     UART     (RX)
	 *   ch7:     CAN      (RX)
	 *   ch8:     PWM      (DATA)
	 *   ch9-11:  I2S      (SCK, WS, SD)
	 *   ch12-13: MIPI DSI (D0N, D0P)
	 *   ch14-15: SWD      (SWDIO, SWCLK)
	 *   ch16-31: mirror of ch0-15
	 *
	 * Each bus uses its own samples-per-bit timebase (I2C_SPB ...
	 * MIPI_SPB above) and a per-bus phase offset, so bit boundaries
	 * do not align across buses. Every generator must return a single
	 * level bit (0/1): mixed_get_sample() masks it with & 1u before
	 * shifting, so a byte-wide value cannot leak into neighbouring
	 * channels.
	 */
	PATTERN_MIXED,
};

/* Analog patterns we can generate. */
enum analog_pattern_type {
	PATTERN_SQUARE,
	PATTERN_SINE,
	PATTERN_TRIANGLE,
	PATTERN_SAWTOOTH,
	PATTERN_ANALOG_RANDOM,
};

/* DSO patterns we can generate. Mirrors old fork demo's DSO pattern set so the
 * demo device can produce non-random DSO waveforms (sine/square/saw/tri).
 * Selected via SR_CONF_PATTERN_MODE on the DSO channel group. */
enum demo_dso_pattern {
	DEMO_DSO_PATTERN_RANDOM = 0,
	DEMO_DSO_PATTERN_SINE,
	DEMO_DSO_PATTERN_SQUARE,
	DEMO_DSO_PATTERN_SAWTOOTH,
	DEMO_DSO_PATTERN_TRIANGLE,
};

/* Logic channel-mode identifiers. Mirrors pxlogic's buffer-mode channel
 * modes so the demo device can simulate real hardware channel-count /
 * samplerate tradeoffs. The numeric ID is stored in devc->logic_ch_mode;
 * the index is the 0-based position in logic_channel_modes[]. */
enum demo_logic_channel_id {
	DEMO_LOGIC250x32 = 32,
	DEMO_LOGIC250x16 = 33,
	DEMO_LOGIC500x16 = 34,
	DEMO_LOGIC1000x8 = 35,
};

enum demo_logic_channel_index {
	LOGIC250x32 = 0,
	LOGIC250x16 = 1,
	LOGIC500x16 = 2,
	LOGIC1000x8 = 3,
};

/* Operation mode — mirrors pxlogic's OP_BUFFER/OP_STREAM so the same
 * DeviceAgent::is_stream_mode() / samplingbar code path works for demo.
 * - OP_BUFFER: trigger-unhit buffers pre-trigger data inside soft_trigger_logic,
 *   waits for trigger to fire, then sends post-trigger data until limit_samples.
 * - OP_STREAM: continuously sends data regardless of trigger state; trigger
 *   (if any) just inserts a SR_DF_TRIGGER marker into the stream. limit_samples
 *   still acts as the stop condition (user-chosen capture depth). */
enum demo_op_mode {
	DEMO_OP_BUFFER = 0,
	DEMO_OP_STREAM = 1,
};

/* Device working mode (mirrors old fork OPERATION_MODE enum). Controls which
 * channel type is active. SR_CONF_DEVICE_MODE get/set reads/writes this field.
 * Unlike old fork, scan() creates all channels upfront — mode switching does
 * NOT rebuild sdi->channels; instead it toggles ch->enabled flags so
 * demo_prepare_data routes to the correct data path. */
enum demo_device_mode {
	DEMO_MODE_LOGIC = 0,
	DEMO_MODE_DSO = 1,
	DEMO_MODE_ANALOG = 2,
	DEMO_MODE_UNKNOWN = 99,
};

/* DSO measurement values (mirrors old fork struct sr_status ch0/ch1 fields).
 * This is a driver-private structure — NOT placed in dsvdef.h, to avoid
 * conflict with the project's architectural decision to remove sr_status.
 * Filled by dso_status_update() after each SR_DF_DSO packet; exposed to GUI
 * via SR_CONF_STATUS_GET config key (returns a GVariant with the fields). */
struct demo_dso_measure {
	uint8_t ch0_max, ch0_min;
	uint8_t ch0_high_level, ch0_low_level;
	uint32_t ch0_cyc_cnt, ch0_cyc_tlen, ch0_cyc_plen, ch0_cyc_llen;
	uint32_t ch0_cyc_rlen, ch0_cyc_flen;
	gboolean ch0_level_valid, ch0_plevel;
	uint64_t ch0_acc_square;
	uint32_t ch0_acc_mean;
	/* ch1 mirrors ch0 for 2-channel DSO. */
	uint8_t ch1_max, ch1_min;
	uint8_t ch1_high_level, ch1_low_level;
	uint32_t ch1_cyc_cnt, ch1_cyc_tlen, ch1_cyc_plen, ch1_cyc_llen;
	uint32_t ch1_cyc_rlen, ch1_cyc_flen;
	gboolean ch1_level_valid, ch1_plevel;
	uint64_t ch1_acc_square;
	uint32_t ch1_acc_mean;
	gboolean measure_valid;
};

/* .demo file replay support (ported from old fork demo driver).
 * Files are loaded from <user_config_dir>/demo/<mode>/<name>.demo as zip
 * archives containing a 'header' INI file plus data files:
 *   LOGIC: L-<ch>/<block>     — per-channel per-block byte-interleaved
 *   DSO:   O-<ch>/0           — per-channel single-block
 *   ANALOG: A-0/0             — single channel single-block */
#define DSO_MID_VAL			128
#define DSO_MAX_VAL			0
#define DSO_MIN_VAL			255
#define DSO_LIMIT			255
#define DSO_WAVE_PERIOD_LEN		200
#define DSO_WAVE_PERIOD_LEN_PER_PROBE	100
#define ANALOG_DEFAULT_VDIV		1000
#define ANALOG_MID_VAL			128
#define ANALOG_MAX_VAL			0
#define ANALOG_MIN_VAL			255
#define ANALOG_CYCLE_RATIO		((gdouble)(103) / (gdouble)(2048))
#define ANALOG_DATA_LEN_PER_CYCLE	206
/* Sample-generator mode index meaning "random math-generated" (no .demo file
 * replay). Any non-zero value in devc->sample_generator is a 1-based index
 * into demo_pattern_array[]. Renamed from old fork's PATTERN_RANDOM to avoid
 * colliding with enum logic_pattern_type's PATTERN_RANDOM (=1) which would
 * otherwise be silently redefined to 0 by this macro. */
#define DEMO_GEN_RANDOM		0
#define PATTERN_COUNT			50
#define DEFAULT_LOGIC_FILE		"protocol"
#define DEFAULT_DSO_FILE		"sine"
/* "random" = DEMO_GEN_RANDOM: use the math generator (per-channel patterns)
 * instead of loading a .demo file. This lets each analog channel show a
 * different waveform (sine/square/triangle/...) by default. Loading a
 * .demo file replays the same data for all channels. */
#define DEFAULT_ANALOG_FILE		"random"
#define MAX_PROBE_NUM			32

/* Pattern mode descriptor: a list of .demo file names (without .demo suffix).
 * Index 0 is always "random" (DEMO_GEN_RANDOM); subsequent entries are files
 * scanned from <user_config_dir>/demo/<mode>/. */
struct demo_mode_pattern {
	char *patterns[PATTERN_COUNT];
	int count;
};

/* Block read buffer for .demo zip data replay. Each channel gets one
 * block_buf; bytes are interleaved into post_buf for output. */
struct demo_packet_buffer {
	void *post_buf;
	uint64_t post_buf_len;
	uint64_t post_len;
	uint64_t block_data_len;
	uint64_t block_chan_read_pos;
	void *block_bufs[MAX_PROBE_NUM];
	uint64_t block_read_positions[MAX_PROBE_NUM];
};

static const char *analog_pattern_str[] = {
	"square",
	"sine",
	"triangle",
	"sawtooth",
	"random",
};

struct analog_pattern {
	float data[ANALOG_BUFSIZE];
	unsigned int num_samples;
};

struct dev_context {
	uint64_t cur_samplerate;
	uint64_t limit_samples;
	uint64_t limit_msec;
	uint64_t limit_frames;
	/* Operation mode: DEMO_OP_BUFFER (default) or DEMO_OP_STREAM.
	 * Set via SR_CONF_OPERATION_MODE config_set. Controls whether
	 * demo_prepare_data sends data continuously (stream) or waits
	 * for trigger (buffer). */
	enum demo_op_mode op_mode;
	uint64_t sent_samples;
	uint64_t sent_frame_samples; /* Number of samples that were sent for current frame. */
	int64_t start_us;
	int64_t spent_us;
	uint64_t step;
	/* Logic */
	int32_t num_logic_channels;
	size_t logic_unitsize;
	uint64_t all_logic_channels_mask;
	/* There is only ever one logic channel group, so its pattern goes here. */
	enum logic_pattern_type logic_pattern;
	uint8_t logic_data[LOGIC_BUFSIZE];
	/* LA_CROSS_DATA conversion buffer. After logic_generator produces
	 * sample-interleaved data in logic_data, convert_to_cross_data()
	 * rearranges it into channel-block format (64 samples per channel
	 * per block) matching pxlogic's hardware DMA layout. This exercises
	 * the frontend's append_cross_payload + bit-align code path, which
	 * would otherwise never be tested with the demo driver. */
	uint8_t cross_data_buf[LOGIC_BUFSIZE];
	/* Analog */
	struct analog_pattern *analog_patterns[ARRAY_SIZE(analog_pattern_str)];
	int32_t num_analog_channels;
	GHashTable *ch_ag;
	/* DAQ per-channel probe config (vdiv/coupling for ANALOG channels). */
	uint64_t analog_vdiv[DSO_MAX_CHANNELS];
	uint8_t analog_coupling[DSO_MAX_CHANNELS];
	/* Per-channel "map default" toggle. When TRUE (default), map unit/min/max
	 * use driver defaults and are disabled in the UI; when user unchecks the
	 * "map default" checkbox, SET stores FALSE so GET returns FALSE and the
	 * UI enables the map fields for editing. Old code ignored SET and GET
	 * always returned TRUE → map unit/min/max permanently disabled. */
	gboolean analog_map_default[DSO_MAX_CHANNELS];
	gboolean avg; /* True if averaging is enabled */
	uint64_t avg_samples;
	size_t enabled_logic_channels;
	size_t enabled_analog_channels;
	size_t first_partial_logic_index;
	uint8_t first_partial_logic_mask;
	/* Triggers */
	uint64_t capture_ratio;
	gboolean trigger_fired;
	struct soft_trigger_logic *stl;
	/* Advanced trigger configuration (PXView-local extension). */
	uint8_t trig_adv_mode;       /* 0=Simple, 1=Adv, 2=Serial */
	gboolean trig_adv_enable;    /* advanced trigger enabled */
	uint8_t trig_adv_stages;     /* stage count */
	char *trig_adv_config;       /* JSON string with full advanced trigger config */
	/* DSO */
	int32_t num_dso_channels;
	size_t enabled_dso_channels;
	uint64_t dso_timebase;
	uint64_t dso_max_timebase;
	uint64_t dso_min_timebase;
	uint8_t dso_unit_bits;
	uint32_t dso_ref_min;
	uint32_t dso_ref_max;
	uint8_t dso_trig_hrate;   /* horizontal trigger position percentage */
	uint8_t dso_trig_source;
	uint8_t dso_trig_slope;
	/* per-channel DSO config (indexed by DSO channel 0..n) */
	uint64_t dso_vdiv[DSO_MAX_CHANNELS];
	uint64_t dso_vfactor[DSO_MAX_CHANNELS];
	uint16_t dso_offset[DSO_MAX_CHANNELS];
	uint16_t dso_hw_offset[DSO_MAX_CHANNELS];
	uint8_t dso_coupling[DSO_MAX_CHANNELS];
	uint8_t dso_trig_value[DSO_MAX_CHANNELS];
	gboolean dso_enabled[DSO_MAX_CHANNELS];
	/* DSO AC 耦合高通滤波器状态 (per-channel, 跨包连续)。
	 * 一阶 RC 高通: y[n] = a*(y[n-1] + x[n] - x[n-1])。
	 * 仅在 dso_coupling[ch]==AC 时使用; GND/DC 不需要。
	 * devc 由 g_malloc0 分配, 初始为 0 (等价于 prev_in=prev_out=0)。 */
	float dso_ac_prev_input[DSO_MAX_CHANNELS];
	float dso_ac_prev_output[DSO_MAX_CHANNELS];
	/* DSO data buffer */
	uint8_t *dso_buf;
	uint64_t dso_sent_samples;
	/* DSO pattern selection (sine/square/sawtooth/triangle/random).
	 * Per-channel: each DSO channel can have its own waveform shape.
	 * Driven by SR_CONF_PATTERN_MODE on the DSO channel group. */
	enum demo_dso_pattern dso_pattern[DSO_MAX_CHANNELS];
	/* Config-change flags: set in config_set when vdiv/offset/timebase change,
	 * checked by dso_wavelength_updata() to trigger waveform regeneration
	 * in demo_send_dso_packet. Mirrors old fork demo's vdiv_change etc. */
	gboolean dso_vdiv_change;
	gboolean dso_offset_change;
	gboolean dso_timebase_change;
	/* Instant mode: when TRUE, demo_send_dso_packet sends progressive
	 * packets based on elapsed time rather than one full frame per tick.
	 * Set via SR_CONF_INSTANT (GUI binds as bool in deviceoptions.cpp). */
	gboolean instant;
	/* Loop mode: when TRUE, dev_acquisition_stop re-arms acquisition
	 * instead of finalizing, so capture repeats until user stops.
	 * Set via SR_CONF_LOOP_MODE (capturemanager.cpp:197). */
	gboolean loop_mode;
	/* Logic channel-mode selection (16ch@125M / 12ch@250M / 6ch@500M / 3ch@1G).
	 * Set via SR_CONF_CHANNEL_MODE (string). logic_adjust_samplerate() clamps
	 * cur_samplerate to the mode's max. */
	enum demo_logic_channel_id logic_ch_mode;
	enum demo_logic_channel_index logic_ch_mode_index;
	/* Analog random cyclic buffer for PATTERN_ANALOG_RANDOM streaming.
	 * Pre-filled once by init_analog_random_data(); read cyclically by
	 * send_analog_packet() so random analog data streams without re-seeding. */
	uint8_t *analog_random_buf;
	size_t analog_random_buf_len;
	size_t analog_random_read_pos;
	/* --- Ported from old fork demo session_vdev (below) --- */
	/* Device working mode (LOGIC/DSO/ANALOG). Mirrors fork sdi->mode.
	 * Set via SR_CONF_DEVICE_MODE; controls channel enabled flags. */
	enum demo_device_mode device_mode;
	/* DSO measurement state. Filled by dso_status_update() after each
	 * SR_DF_DSO packet; read by config_get(SR_CONF_STATUS_GET). */
	struct demo_dso_measure mstatus;
	/* .demo file replay state. archive is the open zip handle (NULL for
	 * math-generated patterns). sample_generator==DEMO_GEN_RANDOM means
	 * use math generation; otherwise index into demo_pattern_array[mode]. */
	unzFile archive;
	uint8_t sample_generator;
	int cur_block;
	int num_blocks;
	uint64_t total_samples;
	uint64_t trig_pos;
	void *data_buf;
	uint64_t data_buf_len;
	gboolean load_data;
	gboolean vdiv_change;
	gboolean offset_change;
	gboolean timebase_change;
	/* Pattern file scan cache. demo_pattern_array[mode] holds the list of
	 * .demo files found under <user_config_dir>/demo/<mode>/. */
	struct demo_mode_pattern demo_pattern_array[3];
	gboolean b_load_directory;
	/* Packet buffer for zip data replay (byte-interleaved output). */
	struct demo_packet_buffer *packet_buffer;
	/* .demo file path (g_strdup'd). Empty string for math-generated mode. */
	char *demo_file_path;
	/* Number of enabled probes (set by SR_CONF_CAPTURE_NUM_PROBES). */
	int num_probes;

	/* --- PXLogic-compatible test keys (for UI testing without hardware) --- */
	/* Voltage threshold (adjustable). */
	double vth;
	/* Glitch filter: 0=None, 1=1 Sample Clock. */
	int filter;
	/* Clock edge: 0=rising, 1=falling. */
	int clock_edge;
	/* External clock enable. */
	gboolean clock_type;
	/* Trigger output enable. */
	gboolean trig_out_en;
	/* RLE compression enable. */
	gboolean rle;
	/* External trigger match mode (index into extern_trigger_match_strs). */
	int ext_trig_mode;
	/* Threshold select (index into threshold_strs). */
	int threshold_sel;
	/* Buffer options (index into buffer_option_strs). */
	int buffer_options;
	/* Bandwidth limit string (stored as int index). */
	int bw_limit;

	/* PWM0 output: when enabled, overrides channel 6's logic data with a
	 * square wave at the specified frequency and duty cycle. */
	gboolean pwm0_en;
	double pwm0_freq;
	double pwm0_duty;
	/* PWM1 output: when enabled, overrides channel 7's logic data with a
	 * square wave at the specified frequency and duty cycle. */
	gboolean pwm1_en;
	double pwm1_freq;
	double pwm1_duty;

	/* Simulated hardware memory depth (total samples across all channels).
	 * Mirrors pxlogic's hw_depth (SR_Gn(4) = 4 billion samples). Used by
	 * SR_CONF_HW_DEPTH to return hw_depth / ch_num, bounding the sample
	 * depth dropdown the same way real hardware does. In Stream mode the
	 * app-layer SR_CONF_STREAM_MEM_BUFF / SR_CONF_STREAM_BUFF (16GB default)
	 * is used instead — matching pxlogic's stream mode behavior. */
	uint64_t simulated_hw_depth;

	/* Fast PRNG state (xorshift32). rand() on Windows is ~150 ns/call;
	 * at 1 GHz with 500K samples/tick and unitsize=2, that's 1 M calls =
	 * 150 ms — 6x the tick budget. xorshift32 is ~3 ns/call (50x faster),
	 * bringing the same workload to ~3 ms. */
	uint32_t prng_state;
};

struct analog_gen {
	struct sr_channel *ch;
	enum sr_mq mq;
	enum sr_mqflag mq_flags;
	enum sr_unit unit;
	enum analog_pattern_type pattern;
	float amplitude;
	float offset;
	struct sr_datafeed_analog packet;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	float avg_val; /* Average value */
	unsigned int num_avgs; /* Number of samples averaged */
	/* AC coupling 高通滤波器状态 (模拟真实 AC 耦合电容)。
	 * 一阶 RC 高通: y[n] = a*(y[n-1] + x[n] - x[n-1]), a = RC/(RC+dt)。
	 * 保持每通道独立状态, 跨包连续。demo 的 analog_coupling==AC 时启用。 */
	float ac_prev_input;
	float ac_prev_output;
	/* 耦合处理临时缓冲区。fast-path (amplitude/offset 未变) 原本直接指向
	 * pattern->data, 但 GND/AC 耦合需修改样本, 不能污染共享的 pattern。
	 * 仅在 coupling != DC 时用作拷贝目标。 */
	float coupling_buf[ANALOG_BUFSIZE];
};

SR_PRIV void demo_generate_analog_pattern(struct dev_context *devc);
SR_PRIV void demo_free_analog_pattern(struct dev_context *devc);
SR_PRIV int demo_prepare_data(int fd, int revents, void *cb_data);
SR_PRIV int demo_send_dso_packet(const struct sr_dev_inst *sdi);

/* New helpers (Tier A+B scope). Implemented in protocol.c; declared here so
 * api.c config_set can call them on config-change. */
SR_PRIV void dso_wavelength_updata(struct dev_context *devc);
SR_PRIV int init_analog_random_data(struct dev_context *devc);
SR_PRIV void logic_adjust_samplerate(struct dev_context *devc);

/* --- .demo file replay (ported from old fork demo) --- */
SR_PRIV void demo_scan_dsl_file(struct sr_dev_inst *sdi);
SR_PRIV int demo_reset_dsl_path(struct sr_dev_inst *sdi, uint8_t pattern_mode);
SR_PRIV int demo_load_virtual_device_session(struct sr_dev_inst *sdi);
SR_PRIV int demo_get_pattern_mode_from_file(const char *sub_dir,
	struct demo_mode_pattern *info, int max_count);
/* NOTE: takes devc so it can lookup devc->demo_pattern_array[mode]. The
 * api.c config_set(SR_CONF_DEVICE_MODE) call site passes devc explicitly. */
SR_PRIV int demo_get_pattern_mode_index_by_string(struct dev_context *devc,
	int mode, const char *str);
SR_PRIV void demo_close_archive(struct dev_context *devc);

/* DSO measurement (ported from old fork dso_status_update). Called after
 * each SR_DF_DSO packet to fill devc->mstatus. */
SR_PRIV void demo_dso_status_update(struct dev_context *devc,
	const uint8_t *buf, uint32_t len);

/* DSO vdiv scaling (ported from old fork receive_data_dso). Called by
 * demo_send_dso_packet to apply per-channel vdiv/offset transform. */
SR_PRIV void demo_dso_vdiv_scale(struct dev_context *devc,
	uint8_t *buf, uint32_t len);

/* ANALOG vdiv scaling (ported from old fork receive_data_analog). Called
 * by send_analog_packet to apply per-channel vdiv transform. */
SR_PRIV void demo_analog_vdiv_scale(struct dev_context *devc,
	uint8_t *buf, uint32_t len, int ch_idx);

/* .demo zip data replay callbacks. Called by demo_prepare_data when
 * sample_generator != DEMO_GEN_RANDOM to read data from zip archive. */
SR_PRIV int demo_receive_data_logic_decoder(struct sr_dev_inst *sdi);
SR_PRIV int demo_receive_data_dso_file(struct sr_dev_inst *sdi);
SR_PRIV int demo_receive_data_analog_file(struct sr_dev_inst *sdi);

/* Helper: get demo file root directory (<user_config_dir>/demo/). */
SR_PRIV char *demo_get_root_dir(void);

#endif
