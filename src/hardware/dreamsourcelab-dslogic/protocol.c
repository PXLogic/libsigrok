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

#include <config.h>
#include <math.h>
#include <stdbool.h>
#include <glib.h>
#include <glib/gstdio.h>
#include "protocol.h"

/* --- Local USB wire-protocol command constants (internal to protocol.c) ---
 * These are the raw bRequest values sent via libusb_control_transfer.
 * They do not conflict with command.h (which defines CMD_CTL_WR/RD_PRE/RD
 * and the DSL_CTL_* enum used by the newer command_ctl_wr/rd wrappers). */
#define DS_CMD_GET_FW_VERSION		0xb0
#define DS_CMD_GET_REVID_VERSION	0xb1
#define DS_CMD_START			0xb2
#define DS_CMD_CONFIG			0xb3
#define DS_CMD_SETTING			0xb4
#define DS_CMD_CONTROL			0xb5
#define DS_CMD_STATUS			0xb6
#define DS_CMD_STATUS_INFO		0xb7
#define DS_CMD_WR_REG			0xb8
#define DS_CMD_WR_NVM			0xb9
#define DS_CMD_RD_NVM			0xba
#define DS_CMD_RD_NVM_PRE		0xbb
#define DS_CMD_GET_HW_INFO		0xbc

#define DS_START_FLAGS_STOP		(1 << 7)
#define DS_START_FLAGS_CLK_48MHZ	(1 << 6)
#define DS_START_FLAGS_SAMPLE_WIDE	(1 << 5)
#define DS_START_FLAGS_MODE_LA		(1 << 4)

/* Legacy register addresses (used by dslogic_set_voltage_threshold raw path). */
#define DS_ADDR_COMB			0x68
#define DS_ADDR_EEWP			0x70
#define DS_ADDR_VTH			0x78

#define DS_MAX_LOGIC_DEPTH		SR_MHZ(16)
#define DS_MAX_LOGIC_SAMPLERATE		SR_MHZ(100)
#define DS_MAX_TRIG_PERCENT		90

/* Sample-alignment mask: align to DSLOGIC_ATOMIC_SAMPLES boundary. */
#define SAMPLES_ALIGN			(DSLOGIC_ATOMIC_SAMPLES - 1)

/* Mode bit masks (built from the TRIG_EN_BIT/CLK_TYPE_BIT/... positions
 * defined in protocol.h). These are the actual bit values written into
 * the FPGA mode word. */
#define DS_MODE_TRIG_EN			(1u << TRIG_EN_BIT)
#define DS_MODE_CLK_TYPE		(1u << CLK_TYPE_BIT)
#define DS_MODE_CLK_EDGE		(1u << CLK_EDGE_BIT)
#define DS_MODE_RLE_MODE		(1u << RLE_MODE_BIT)
#define DS_MODE_DSO_MODE		(1u << DSO_MODE_BIT)
#define DS_MODE_HALF_MODE		(1u << HALF_MODE_BIT)
#define DS_MODE_QUAR_MODE		(1u << QUAR_MODE_BIT)
#define DS_MODE_ANALOG_MODE		(1u << ANALOG_MODE_BIT)
#define DS_MODE_FILTER			(1u << FILTER_BIT)
#define DS_MODE_INSTANT			(1u << INSTANT_BIT)
#define DS_MODE_STRIG_MODE		(1u << STRIG_MODE_BIT)
#define DS_MODE_STREAM_MODE		(1u << STREAM_MODE_BIT)
#define DS_MODE_LPB_TEST		(1u << LPB_TEST_BIT)
#define DS_MODE_EXT_TEST		(1u << EXT_TEST_BIT)
#define DS_MODE_INT_TEST		(1u << INT_TEST_BIT)

/* FPGA TLV configuration tags (legacy fpga_config path). */
#define _DS_CFG(variable, wordcnt) ((variable << 8) | wordcnt)
#define DS_CFG_START			0xf5a5f5a5
#define DS_CFG_MODE			_DS_CFG(0, 1)
#define DS_CFG_DIVIDER			_DS_CFG(1, 2)
#define DS_CFG_COUNT			_DS_CFG(3, 2)
#define DS_CFG_TRIG_POS			_DS_CFG(5, 2)
#define DS_CFG_TRIG_GLB			_DS_CFG(7, 1)
#define DS_CFG_CH_EN			_DS_CFG(8, 1)
#define DS_CFG_TRIG			_DS_CFG(64, 160)
#define DS_CFG_END			0xfa5afa5a

#pragma pack(push, 1)

/* Legacy FPGA configuration struct (used by the LOGIC receive path). */
struct fpga_config {
	uint32_t sync;

	uint16_t mode_header;
	uint16_t mode;
	uint16_t divider_header;
	uint32_t divider;
	uint16_t count_header;
	uint32_t count;
	uint16_t trig_pos_header;
	uint32_t trig_pos;
	uint16_t trig_glb_header;
	uint16_t trig_glb;
	uint16_t ch_en_header;
	uint16_t ch_en;

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

/* Start/stop acquisition command (sent via DS_CMD_START). */
struct cmd_start_acquisition {
	uint8_t flags;
	uint8_t sample_delay_h;
	uint8_t sample_delay_l;
};

#pragma pack(pop)

/*
 * This should be larger than the FPGA bitstream image so that it'll get
 * uploaded in one big operation. There seem to be issues when uploading
 * it in chunks.
 */
#define FW_BUFSIZE (1024 * 1024)

#define FPGA_UPLOAD_DELAY (10 * 1000)

#define USB_TIMEOUT (3 * 1000)

/* ===========================================================================
 * Static helper functions (legacy USB control-transfer wrappers)
 * =========================================================================== */

static int command_get_fw_version(libusb_device_handle *devhdl,
				  struct version_info *vi)
{
	int ret;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, DS_CMD_GET_FW_VERSION, 0x0000, 0x0000,
		(unsigned char *)vi, sizeof(struct version_info), USB_TIMEOUT);

	if (ret < 0) {
		sr_err("Unable to get version info: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_get_revid_version(struct sr_dev_inst *sdi, uint8_t *revid)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	libusb_device_handle *devhdl = usb->devhdl;
	int ret;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, DS_CMD_GET_REVID_VERSION, 0x0000, 0x0000,
		revid, 1, USB_TIMEOUT);

	if (ret < 0) {
		sr_err("Unable to get REVID: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_start_acquisition(const struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;
	struct cmd_start_acquisition mode;
	int ret;

	mode.flags = DS_START_FLAGS_MODE_LA | DS_START_FLAGS_SAMPLE_WIDE;
	mode.sample_delay_h = mode.sample_delay_l = 0;

	usb = sdi->conn;
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_START, 0x0000, 0x0000,
			(unsigned char *)&mode, sizeof(mode), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Failed to send start command: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_stop_acquisition(const struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;
	struct cmd_start_acquisition mode;
	int ret;

	mode.flags = DS_START_FLAGS_STOP;
	mode.sample_delay_h = mode.sample_delay_l = 0;

	usb = sdi->conn;
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_START, 0x0000, 0x0000,
			(unsigned char *)&mode, sizeof(mode), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Failed to send stop command: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

/* ===========================================================================
 * Channel helpers
 * =========================================================================== */

static unsigned int enabled_channel_count(const struct sr_dev_inst *sdi)
{
	unsigned int count = 0;
	for (const GSList *l = sdi->channels; l; l = l->next) {
		const struct sr_channel *const probe = (struct sr_channel *)l->data;
		if (probe->enabled)
			count++;
	}
	return count;
}

static uint16_t enabled_channel_mask(const struct sr_dev_inst *sdi)
{
	unsigned int mask = 0;
	for (const GSList *l = sdi->channels; l; l = l->next) {
		const struct sr_channel *const probe = (struct sr_channel *)l->data;
		if (probe->enabled)
			mask |= 1 << probe->index;
	}
	return mask;
}

/*
 * Get the session trigger and configure the legacy FPGA structure
 * accordingly.
 * @return @c true if any triggers are enabled, @c false otherwise.
 */
static bool set_trigger(const struct sr_dev_inst *sdi, struct fpga_config *cfg)
{
	struct sr_trigger *trigger;
	struct sr_trigger_stage *stage;
	struct sr_trigger_match *match;
	struct dev_context *devc;
	const GSList *l, *m;
	const unsigned int num_enabled_channels = enabled_channel_count(sdi);
	int num_trigger_stages = 0;

	int channelbit, i = 0;
	uint32_t trigger_point;

	devc = sdi->priv;

	cfg->ch_en = enabled_channel_mask(sdi);

	for (i = 0; i < NUM_TRIGGER_STAGES; i++) {
		cfg->trig_mask0[i] = 0xffff;
		cfg->trig_mask1[i] = 0xffff;
		cfg->trig_value0[i] = 0;
		cfg->trig_value1[i] = 0;
		cfg->trig_edge0[i] = 0;
		cfg->trig_edge1[i] = 0;
		cfg->trig_logic0[i] = 2;
		cfg->trig_logic1[i] = 2;
		cfg->trig_count[i] = 0;
	}

	trigger_point = (devc->capture_ratio * devc->limit_samples) / 100;
	if (trigger_point < DSLOGIC_ATOMIC_SAMPLES)
		trigger_point = DSLOGIC_ATOMIC_SAMPLES;
	const uint64_t mem_depth = devc->profile->dev_caps.hw_depth;
	const uint32_t max_trigger_point = devc->continuous_mode ?
		((uint32_t)((mem_depth * 10) / 100)) :
		((uint32_t)((mem_depth * DS_MAX_TRIG_PERCENT) / 100));
	if (trigger_point > max_trigger_point)
		trigger_point = max_trigger_point;
	cfg->trig_pos = trigger_point & ~(uint32_t)(DSLOGIC_ATOMIC_SAMPLES - 1);

	if (!(trigger = sr_session_trigger_get(sdi->session))) {
		sr_dbg("No session trigger found");
		return false;
	}

	for (l = trigger->stages; l; l = l->next) {
		stage = l->data;
		num_trigger_stages++;
		for (m = stage->matches; m; m = m->next) {
			match = m->data;
			if (!match->channel->enabled)
				/* Ignore disabled channels with a trigger. */
				continue;
			channelbit = 1 << (match->channel->index);
			/* Simple trigger support (event). */
			if (match->match == SR_TRIGGER_ONE) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
				cfg->trig_value0[0] |= channelbit;
				cfg->trig_value1[0] |= channelbit;
			} else if (match->match == SR_TRIGGER_ZERO) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
			} else if (match->match == SR_TRIGGER_FALLING) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
				cfg->trig_edge0[0] |= channelbit;
				cfg->trig_edge1[0] |= channelbit;
			} else if (match->match == SR_TRIGGER_RISING) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
				cfg->trig_value0[0] |= channelbit;
				cfg->trig_value1[0] |= channelbit;
				cfg->trig_edge0[0] |= channelbit;
				cfg->trig_edge1[0] |= channelbit;
			} else if (match->match == SR_TRIGGER_EDGE) {
				cfg->trig_edge0[0] |= channelbit;
				cfg->trig_edge1[0] |= channelbit;
			}
		}
	}

	cfg->trig_glb = (num_enabled_channels << 4) | (num_trigger_stages - 1);

	return num_trigger_stages != 0;
}

static int fpga_configure(const struct sr_dev_inst *sdi)
{
	const struct dev_context *const devc = sdi->priv;
	const struct sr_usb_dev_inst *const usb = sdi->conn;
	uint8_t c[3];
	struct fpga_config cfg;
	uint16_t mode = 0;
	uint32_t divider;
	int transferred, len, ret;

	sr_dbg("Configuring FPGA.");

	WL32(&cfg.sync, DS_CFG_START);
	WL16(&cfg.mode_header, DS_CFG_MODE);
	WL16(&cfg.divider_header, DS_CFG_DIVIDER);
	WL16(&cfg.count_header, DS_CFG_COUNT);
	WL16(&cfg.trig_pos_header, DS_CFG_TRIG_POS);
	WL16(&cfg.trig_glb_header, DS_CFG_TRIG_GLB);
	WL16(&cfg.ch_en_header, DS_CFG_CH_EN);
	WL16(&cfg.trig_header, DS_CFG_TRIG);
	WL32(&cfg.end_sync, DS_CFG_END);

	/* Pass in the length of a fixed-size struct. Really. */
	len = sizeof(struct fpga_config) / 2;
	c[0] = len & 0xff;
	c[1] = (len >> 8) & 0xff;
	c[2] = (len >> 16) & 0xff;

	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_SETTING, 0x0000, 0x0000,
			c, sizeof(c), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Failed to send FPGA configure command: %s.",
			libusb_error_name(ret));
		return SR_ERR;
	}

	if (set_trigger(sdi, &cfg))
		mode |= DS_MODE_TRIG_EN;

	/* Operation-mode test bits (compare op_mode, NOT mode). */
	if (devc->op_mode == LO_OP_INTEST)
		mode |= DS_MODE_INT_TEST;
	else if (devc->op_mode == LO_OP_EXTEST)
		mode |= DS_MODE_EXT_TEST;
	else if (devc->op_mode == LO_OP_LPTEST)
		mode |= DS_MODE_LPB_TEST;

	if (devc->cur_samplerate == DS_MAX_LOGIC_SAMPLERATE * 2)
		mode |= DS_MODE_HALF_MODE;
	else if (devc->cur_samplerate == DS_MAX_LOGIC_SAMPLERATE * 4)
		mode |= DS_MODE_QUAR_MODE;

	if (devc->continuous_mode)
		mode |= DS_MODE_STREAM_MODE;
	if (devc->external_clock) {
		mode |= DS_MODE_CLK_TYPE;
		if (devc->clock_edge)
			mode |= DS_MODE_CLK_EDGE;
	}
	if (devc->limit_samples > DS_MAX_LOGIC_DEPTH *
		ceil(devc->cur_samplerate * 1.0 / DS_MAX_LOGIC_SAMPLERATE)
		&& !devc->continuous_mode) {
		/* Enable RLE for long captures.
		 * Without this, captured data present errors.
		 */
		mode |= DS_MODE_RLE_MODE;
	}

	WL16(&cfg.mode, mode);
	divider = ceil(DS_MAX_LOGIC_SAMPLERATE * 1.0 / devc->cur_samplerate);
	WL32(&cfg.divider, divider);

	/* Number of 16-sample units. */
	WL32(&cfg.count, devc->limit_samples / 16);

	len = sizeof(struct fpga_config);
	ret = libusb_bulk_transfer(usb->devhdl, 2 | LIBUSB_ENDPOINT_OUT,
			(unsigned char *)&cfg, len, &transferred, USB_TIMEOUT);
	if (ret < 0 || transferred != len) {
		sr_err("Failed to send FPGA configuration: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

/* ===========================================================================
 * Acquisition transfer management (LOGIC receive path)
 * =========================================================================== */

static void abort_acquisition(struct dev_context *devc)
{
	int i;

	devc->acq_aborted = TRUE;

	for (i = devc->num_transfers - 1; i >= 0; i--) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;

	std_session_send_df_end(sdi);

	usb_source_remove(sdi->session, devc->ctx);

	devc->num_transfers = 0;
	g_free(devc->transfers);
	g_free(devc->deinterleave_buffer);
	devc->deinterleave_buffer = NULL;
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	unsigned int i;

	sdi = transfer->user_data;
	devc = sdi->priv;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}

	devc->submitted_transfers--;
	if (devc->submitted_transfers == 0)
		finish_acquisition(sdi);
}

static void resubmit_transfer(struct libusb_transfer *transfer)
{
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	sr_err("%s: %s", __func__, libusb_error_name(ret));
	free_transfer(transfer);
}

static void deinterleave_buffer(const uint8_t *src, size_t length,
	uint16_t *dst_ptr, size_t channel_count, uint16_t channel_mask)
{
	uint16_t sample;

	for (const uint64_t *src_ptr = (uint64_t*)src;
		src_ptr < (uint64_t*)(src + length);
		src_ptr += channel_count) {
		for (int bit = 0; bit != 64; bit++) {
			const uint64_t *word_ptr = src_ptr;
			sample = 0;
			for (unsigned int channel = 0; channel != 16;
				channel++) {
				const uint16_t m = channel_mask >> channel;
				if (!m)
					break;
				if ((m & 1) && ((*word_ptr++ >> bit) & UINT64_C(1)))
					sample |= 1 << channel;
			}
			*dst_ptr++ = sample;
		}
	}
}

static void send_data(struct sr_dev_inst *sdi,
	uint16_t *data, size_t sample_count)
{
	const struct sr_datafeed_logic logic = {
		.length = sample_count * sizeof(uint16_t),
		.unitsize = sizeof(uint16_t),
		.data = data
	};

	const struct sr_datafeed_packet packet = {
		.type = SR_DF_LOGIC,
		.payload = &logic
	};

	sr_session_send(sdi, &packet);
}

/* Forward declarations: receive_transfer() (DSO path) needs get_buffer_size
 * which is defined later in the transfer-management section. */
static size_t to_bytes_per_ms(const struct sr_dev_inst *sdi);
static size_t get_buffer_size(const struct sr_dev_inst *sdi);

/* ===========================================================================
 * get_measure() — parse DSO status header from USB transfer buffer.
 * Ported from old dsl.c::get_measure(), simplified for struct dsl_status
 * (fewer fields than old fork struct sr_status). Populates devc->mstatus
 * and sets devc->mstatus_valid based on pkt_id / vlen sanity checks.
 * =========================================================================== */
static void get_measure(const struct sr_dev_inst *sdi, uint8_t *buf, uint32_t offset)
{
	struct dev_context *devc = sdi->priv;
	const struct DSL_channels *cm = &channel_modes[devc->ch_mode];
	int en_ch_num = dsl_en_ch_num(sdi);

	devc->mstatus.pkt_id = *((const uint16_t *)buf + offset);
	devc->mstatus.vlen = *((const uint32_t *)buf + offset / 2 + 2 / 2) & 0x0fffffff;
	devc->mstatus.stream_mode =
		(*((const uint32_t *)buf + offset / 2 + 2 / 2) & 0x80000000) != 0;
	devc->mstatus.sample_divider_tog =
		(*((const uint32_t *)buf + offset / 2 + 4 / 2) & 0x80000000) != 0;
	devc->mstatus.trig_flag =
		(*((const uint32_t *)buf + offset / 2 + 4 / 2) & 0x40000000) != 0;
	devc->mstatus.trig_ch =
		(*((const uint8_t *)buf + offset * 2 + 5 * 2 + 1) & 0x38) >> 3;
	devc->mstatus.trig_offset =
		*((const uint8_t *)buf + offset * 2 + 5 * 2 + 1) & 0x07;

	/* ch0 accumulated mean values (4 phases) */
	devc->mstatus.ch0_acc_mean =
		*((const uint32_t *)buf + offset / 2 + 52 / 2);
	devc->mstatus.ch0_acc_mean_p1 =
		*((const uint32_t *)buf + offset / 2 + 54 / 2);
	devc->mstatus.ch0_acc_mean_p2 =
		*((const uint32_t *)buf + offset / 2 + 56 / 2);
	devc->mstatus.ch0_acc_mean_p3 =
		*((const uint32_t *)buf + offset / 2 + 58 / 2);

	/* ch1 accumulated mean values (4 phases) */
	devc->mstatus.ch1_acc_mean =
		*((const uint32_t *)buf + offset / 2 + 84 / 2);
	devc->mstatus.ch1_acc_mean_p1 =
		*((const uint32_t *)buf + offset / 2 + 86 / 2);
	devc->mstatus.ch1_acc_mean_p2 =
		*((const uint32_t *)buf + offset / 2 + 88 / 2);
	devc->mstatus.ch1_acc_mean_p3 =
		*((const uint32_t *)buf + offset / 2 + 90 / 2);

	if (!devc->zero_branch) {
		devc->mstatus.ch0_acc_mean += devc->mstatus.ch0_acc_mean_p1;
		devc->mstatus.ch0_acc_mean += devc->mstatus.ch0_acc_mean_p2;
		devc->mstatus.ch0_acc_mean += devc->mstatus.ch0_acc_mean_p3;

		devc->mstatus.ch1_acc_mean += devc->mstatus.ch1_acc_mean_p1;
		devc->mstatus.ch1_acc_mean += devc->mstatus.ch1_acc_mean_p2;
		devc->mstatus.ch1_acc_mean += devc->mstatus.ch1_acc_mean_p3;

		if (en_ch_num == 1) {
			uint64_t tmp;
			tmp = devc->mstatus.ch0_acc_mean + devc->mstatus.ch1_acc_mean;
			devc->mstatus.ch0_acc_mean = tmp;
			devc->mstatus.ch1_acc_mean = tmp;
		}
	}

	/* Validate mstatus: pkt_id must match DSO_PKTID and vlen must be nonzero.
	 * The old fork also checked sample_divider == divider, but the new
	 * struct dsl_status doesn't carry the full sample_divider value (only
	 * the toggle bit), so we relax to pkt_id + vlen check. */
	devc->mstatus_valid = FALSE;
	if (devc->instant) {
		devc->mstatus_valid = (devc->mstatus.pkt_id == DSO_PKTID);
	} else if (devc->mstatus.pkt_id == DSO_PKTID && devc->mstatus.vlen != 0) {
		devc->mstatus_valid = TRUE;
	}

	if (devc->mstatus_valid) {
		devc->mstatus.trig_hit = devc->mstatus.trig_flag ? 1 : 0;
		/* Update hw_offset for each channel from the status header */
		for (const GSList *l = sdi->channels; l; l = l->next) {
			struct sr_channel *probe = (struct sr_channel *)l->data;
			if (probe->priv) {
				DSL_CH_PRIV(probe)->hw_offset =
					*((const uint8_t *)buf + offset * 2 +
						(51 + 32 * probe->index) * 2);
			}
		}
		devc->roll = (devc->mstatus.stream_mode != 0);
	}
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *const sdi = transfer->user_data;
	struct dev_context *const devc = sdi->priv;
	uint8_t *const cur_buf = transfer->buffer;

	gboolean packet_has_error = FALSE;

	/*
	 * If acquisition has already ended, just free any queued up
	 * transfer that come in.
	 */
	if (devc->acq_aborted) {
		free_transfer(transfer);
		return;
	}

	sr_dbg("receive_transfer(): status %s received %d bytes.",
		libusb_error_name(transfer->status), transfer->actual_length);

	/* Save incoming transfer before reusing the transfer struct. */

	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		abort_acquisition(devc);
		free_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
		break;
	default:
		packet_has_error = TRUE;
		break;
	}

	if (transfer->actual_length == 0 || packet_has_error) {
		devc->empty_transfer_count++;
		if (devc->empty_transfer_count > MAX_EMPTY_POLL) {
			/*
			 * The FX2 gave up. End the acquisition, the frontend
			 * will work out that the samplecount is short.
			 */
			abort_acquisition(devc);
			free_transfer(transfer);
		} else {
			resubmit_transfer(transfer);
		}
		return;
	} else {
		devc->empty_transfer_count = 0;
	}

	/* ===================================================================
	 * Dispatch by acquisition mode (LOGIC / DSO / ANALOG).
	 *
	 * LOGIC:  deinterleave channel-block USB data into bit-interleaved
	 *         samples and forward via SR_DF_LOGIC. Trigger position is
	 *         delivered inline via std_session_send_df_trigger().
	 *
	 * DSO:    parse the DSO status header embedded at the start of the
	 *         transfer buffer (get_measure), then forward the waveform
	 *         via SR_DF_DSO. Non-instant mode delivers one frame per
	 *         acquisition; instant mode streams progressive slices until
	 *         actual_samples + instant_tail_bytes are collected.
	 *
	 * ANALOG: forward the raw analog sample buffer via SR_DF_ANALOG
	 *         (constructed with sr_analog_init). One frame per acquisition.
	 * =================================================================== */
	if (devc->mode == DSL_MODE_DSO) {
		/* --- DSO receive path (ported from old dsl.c::receive_transfer) --- */
		struct sr_datafeed_packet packet;
		struct sr_datafeed_dso dso;
		uint64_t cur_sample_count = 0;

		memset(&packet, 0, sizeof(packet));
		memset(&dso, 0, sizeof(dso));
		packet.type = SR_DF_DSO;
		packet.payload = &dso;

		if (!devc->instant) {
			/* Locate the DSO status header offset within the buffer.
			 * For non-instant mode the header sits at an offset that
			 * depends on actual_samples and the channel-mode split. */
			const uint32_t offset = (uint32_t)(devc->actual_samples /
				(channel_modes[devc->ch_mode].num /
				 dsl_en_ch_num(sdi)));
			get_measure(sdi, cur_buf, offset);
		} else {
			/* Instant mode: no embedded status header; synthesize
			 * defaults so the packet is marked valid and the full
			 * buffer is treated as progressive waveform data. */
			devc->mstatus.vlen = (uint32_t)(get_buffer_size(sdi) /
				channel_modes[devc->ch_mode].num);
			devc->mstatus.trig_offset = 0;
			devc->mstatus.sample_divider_tog = FALSE;
			devc->mstatus_valid = TRUE;
		}

		if (devc->mstatus_valid) {
			devc->roll = (devc->mstatus.stream_mode != 0);
			cur_sample_count = MIN(
				(uint64_t)channel_modes[devc->ch_mode].num *
				devc->mstatus.vlen / dsl_en_ch_num(sdi),
				devc->limit_samples);

			dso.data = cur_buf + (devc->zero ? 0 :
				2 * devc->mstatus.trig_offset);
			dso.num_samples = (uint32_t)cur_sample_count;
			dso.trig_flag = (devc->mstatus.trig_flag != 0);
			dso.trig_ch = devc->mstatus.trig_ch;
			dso.en_ch_num = (uint8_t)dsl_en_ch_num(sdi);
			dso.sample_bits = channel_modes[devc->ch_mode].unit_bits;
			dso.trig_offset = (int16_t)devc->mstatus.trig_offset;
			dso.packet_len = transfer->actual_length;
			dso.samplerate_tog = (devc->mstatus.sample_divider_tog != 0);

			sr_session_send(sdi, &packet);
		} else {
			sr_err("DSO mstatus invalid, skipping packet.");
		}

		devc->num_samples += cur_sample_count;

		/* DSO instant mode stop condition: keep streaming progressive
		 * slices until we've collected actual_samples worth of data
		 * PLUS instant_tail_bytes of overshoot (so the final status
		 * header can be parsed from the tail). */
		if (devc->instant && devc->limit_samples &&
			devc->num_samples >= devc->actual_samples) {
			int over_bytes = (int)((devc->num_samples -
				devc->actual_samples) * dsl_en_ch_num(sdi));
			if (over_bytes >= devc->instant_tail_bytes) {
				const uint32_t offset =
					(transfer->actual_length - over_bytes) / 2;
				get_measure(sdi, cur_buf, offset);
				abort_acquisition(devc);
				free_transfer(transfer);
				return;
			}
		}

		/* DSO non-instant: device delivers exactly one frame per
		 * acquisition; stop after the first valid packet is sent. */
		if (!devc->instant && devc->mstatus_valid) {
			abort_acquisition(devc);
			free_transfer(transfer);
			return;
		}

		resubmit_transfer(transfer);
	}
	else if (devc->mode == DSL_MODE_ANALOG) {
		/* --- ANALOG receive path (ported from old dsl.c::receive_transfer) --- */
		struct sr_datafeed_packet packet;
		struct sr_datafeed_analog analog;
		struct sr_analog_encoding encoding;
		struct sr_analog_meaning meaning;
		struct sr_analog_spec spec;
		GSList *channels = NULL;
		uint64_t cur_sample_count;
		unsigned int ch_list_len;

		memset(&packet, 0, sizeof(packet));
		packet.type = SR_DF_ANALOG;
		packet.payload = &analog;

		/* sr_analog_init zeroes analog/encoding/meaning/spec and wires
		 * the encoding/meaning/spec pointers into the analog struct.
		 * Default encoding is float, native endianness. */
		sr_analog_init(&analog, &encoding, &meaning, &spec, 0);

		/* Build the enabled-channel GSList for analog meaning. The
		 * old fork used analog.probes = sdi->channels directly, but
		 * upstream carries channels inside sr_analog_meaning. */
		for (const GSList *l = sdi->channels; l; l = l->next) {
			struct sr_channel *ch = (struct sr_channel *)l->data;
			if (ch->enabled)
				channels = g_slist_append(channels, ch);
		}
		ch_list_len = g_slist_length(channels);
		meaning.channels = channels;
		meaning.mq = SR_MQ_VOLTAGE;
		meaning.unit = SR_UNIT_VOLT;
		meaning.mqflags = SR_MQFLAG_AC;

		/* Sample count = total_bytes / (bytes_per_sample * num_channels).
		 * bytes_per_sample = ceil(unit_bits / 8). */
		cur_sample_count = transfer->actual_length /
			(((channel_modes[devc->ch_mode].unit_bits + 7) / 8) *
			 (ch_list_len ? ch_list_len : 1));

		analog.data = cur_buf;
		analog.num_samples = (uint32_t)cur_sample_count;

		sr_session_send(sdi, &packet);
		g_slist_free(channels);

		devc->num_samples += cur_sample_count;

		/* ANALOG: one frame per acquisition (single-shot), stop after
		 * the first frame is forwarded. The device stops sending on
		 * its own; aborting here avoids waiting for MAX_EMPTY_POLL
		 * empty transfers. */
		abort_acquisition(devc);
		free_transfer(transfer);
	}
	else {
		/* --- LOGIC receive path (existing implementation) ---
		 *
		 * The DSLogic emits sample data as sequences of 64-bit sample
		 * words in a round-robin i.e. 64-bits from channel 0, 64-bits
		 * from channel 1 etc. for each of the enabled channels, then
		 * looping back to the channel.
		 *
		 * Because sigrok's internal representation is bit-interleaved
		 * channels we must recast the data. */
		const size_t channel_count = enabled_channel_count(sdi);
		const uint16_t channel_mask = enabled_channel_mask(sdi);
		const unsigned int cur_sample_count = DSLOGIC_ATOMIC_SAMPLES *
			transfer->actual_length /
			(DSLOGIC_ATOMIC_SIZE * channel_count);
		unsigned int num_samples;
		int trigger_offset;

		/* Loop mode: skip limit_samples clipping — data flows continuously
		 * until user stops, matching demo/pxlogic's loop_mode behavior. */
		if (devc->is_loop || !devc->limit_samples || devc->sent_samples < devc->limit_samples) {
			if (!devc->is_loop && devc->limit_samples && devc->sent_samples + cur_sample_count > devc->limit_samples)
				num_samples = devc->limit_samples - devc->sent_samples;
			else
				num_samples = cur_sample_count;

			if (transfer->actual_length % (DSLOGIC_ATOMIC_SIZE * channel_count) != 0)
				sr_err("Invalid transfer length!");
			deinterleave_buffer(transfer->buffer, transfer->actual_length,
				devc->deinterleave_buffer, channel_count, channel_mask);

			/* Send the incoming transfer to the session bus. */
			if (devc->trigger_pos > devc->sent_samples
				&& devc->trigger_pos <= devc->sent_samples + num_samples) {
				/* DSLogic trigger in this block. Send trigger position. */
				trigger_offset = (int)(devc->trigger_pos - devc->sent_samples);
				/* Pre-trigger samples. */
				send_data(sdi, devc->deinterleave_buffer, trigger_offset);
				devc->sent_samples += trigger_offset;
				/* Trigger position. */
				devc->trigger_pos = 0;
				std_session_send_df_trigger(sdi);
				/* Post trigger samples. */
				num_samples -= trigger_offset;
				send_data(sdi, devc->deinterleave_buffer
					+ trigger_offset, num_samples);
				devc->sent_samples += num_samples;
			} else {
				send_data(sdi, devc->deinterleave_buffer, num_samples);
				devc->sent_samples += num_samples;
			}
		}

		/* Stop condition: in loop mode, data flows forever until user stops
		 * (matching demo/pxlogic loop_mode behavior). Without this, DSL would
		 * abort at limit_samples even when is_loop is set. */
		if (!devc->is_loop && devc->limit_samples && devc->sent_samples >= devc->limit_samples) {
			abort_acquisition(devc);
			free_transfer(transfer);
		} else {
			resubmit_transfer(transfer);
		}
	}
}

static int receive_data(int fd, int revents, void *cb_data)
{
	struct timeval tv;
	struct drv_context *drvc;

	(void)fd;
	(void)revents;

	drvc = (struct drv_context *)cb_data;

	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);

	return TRUE;
}

static size_t to_bytes_per_ms(const struct sr_dev_inst *sdi)
{
	const struct dev_context *const devc = sdi->priv;
	const size_t ch_count = enabled_channel_count(sdi);

	if (devc->continuous_mode)
		return (devc->cur_samplerate * ch_count) / (1000 * 8);

	/* If we're in buffered mode, the transfer rate is not so important,
	 * but we expect to get at least 10% of the high-speed USB bandwidth.
	 */
	return 35000000 / (1000 * 10);
}

static size_t get_buffer_size(const struct sr_dev_inst *sdi)
{
	/*
	 * The buffer should be large enough to hold 10ms of data and
	 * a multiple of the size of a data atom.
	 */
	const size_t block_size = enabled_channel_count(sdi) * 512;
	const size_t s = 10 * to_bytes_per_ms(sdi);
	if (!block_size)
		return s;
	return ((s + block_size - 1) / block_size) * block_size;
}

static unsigned int get_number_of_transfers(const struct sr_dev_inst *sdi)
{
	/* Total buffer size should be able to hold about 100ms of data. */
	const unsigned int s = get_buffer_size(sdi);
	const unsigned int n = (100 * to_bytes_per_ms(sdi) + s - 1) / s;
	return (n > NUM_SIMUL_TRANSFERS) ? NUM_SIMUL_TRANSFERS : n;
}

static unsigned int get_timeout(const struct sr_dev_inst *sdi)
{
	const size_t total_size = get_buffer_size(sdi) *
		get_number_of_transfers(sdi);
	const unsigned int timeout = total_size / to_bytes_per_ms(sdi);
	return timeout + timeout / 4; /* Leave a headroom of 25% percent. */
}

static int start_transfers(const struct sr_dev_inst *sdi)
{
	const size_t channel_count = enabled_channel_count(sdi);
	const size_t size = get_buffer_size(sdi);
	const unsigned int num_transfers = get_number_of_transfers(sdi);
	const unsigned int timeout = get_timeout(sdi);

	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct libusb_transfer *transfer;
	unsigned int i;
	int ret;
	unsigned char *buf;

	devc = sdi->priv;
	usb = sdi->conn;

	devc->sent_samples = 0;
	devc->acq_aborted = FALSE;
	devc->empty_transfer_count = 0;
	devc->submitted_transfers = 0;

	g_free(devc->transfers);
	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * num_transfers);
	if (!devc->transfers) {
		sr_err("USB transfers malloc failed.");
		return SR_ERR_MALLOC;
	}

	devc->deinterleave_buffer = g_try_malloc(DSLOGIC_ATOMIC_SAMPLES *
		(size / (channel_count * DSLOGIC_ATOMIC_SIZE)) * sizeof(uint16_t));
	if (!devc->deinterleave_buffer) {
		sr_err("Deinterleave buffer malloc failed.");
		g_free(devc->deinterleave_buffer);
		devc->deinterleave_buffer = NULL;
		return SR_ERR_MALLOC;
	}

	devc->num_transfers = num_transfers;
	for (i = 0; i < num_transfers; i++) {
		if (!(buf = g_try_malloc(size))) {
			sr_err("USB transfer buffer malloc failed.");
			return SR_ERR_MALLOC;
		}
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, usb->devhdl,
				6 | LIBUSB_ENDPOINT_IN, buf, size,
				receive_transfer, (void *)sdi, timeout);
		sr_info("submitting transfer: %d", i);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("Failed to submit transfer: %s.",
			       libusb_error_name(ret));
			libusb_free_transfer(transfer);
			g_free(buf);
			abort_acquisition(devc);
			return SR_ERR;
		}
		devc->transfers[i] = transfer;
		devc->submitted_transfers++;
	}

	std_session_send_df_header(sdi);

	return SR_OK;
}

static void LIBUSB_CALL trigger_receive(struct libusb_transfer *transfer)
{
	const struct sr_dev_inst *sdi;
	struct dsl_trigger_pos *tpos;
	struct dev_context *devc;

	sdi = transfer->user_data;
	devc = sdi->priv;
	if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
		sr_dbg("Trigger transfer canceled.");
		/* Terminate session. */
		std_session_send_df_end(sdi);
		usb_source_remove(sdi->session, devc->ctx);
		devc->num_transfers = 0;
		g_free(devc->transfers);
		devc->transfers = NULL;
	} else if (transfer->status == LIBUSB_TRANSFER_COMPLETED
			&& transfer->actual_length == sizeof(struct dsl_trigger_pos)) {
		tpos = (struct dsl_trigger_pos *)transfer->buffer;
		sr_info("tpos real_pos %d ram_saddr %d cnt_h %d cnt_l %d", tpos->real_pos,
			tpos->ram_saddr, tpos->remain_cnt_h, tpos->remain_cnt_l);
		devc->trigger_pos = tpos->real_pos;
		start_transfers(sdi);
	}
	libusb_free_transfer(transfer);
}

/* ===========================================================================
 * Firmware presence diagnostics
 *
 * A missing FX2 .fw is the single most misleading failure mode of this driver:
 * scan() calls ezusb_upload_firmware() expecting the device to re-enumerate as
 * "DreamSourceLab USB-based Instrument", but when the file is absent libusb has
 * already opened the handle and never releases it. The user then sees
 * "Unable to claim USB interface. Another program or driver has already claimed
 * it." plus a driver error -- neither of which names the real cause.
 *
 * These helpers let scan()/dev_open() detect the gap BEFORE touching the
 * device, and report the filename plus every directory that was searched.
 * =========================================================================== */

/* Look the firmware up through the very same search path libsigrok's
 * resource_open_default() uses, so a "exists" answer here is authoritative. */
SR_PRIV gboolean dslogic_firmware_exists(struct sr_context *sr_ctx, const char *name)
{
	struct sr_resource res;
	int ret;

	if (!name || !*name)
		return FALSE;

	/* sr_resource_open() dereferences ctx->resource_open_cb without a NULL
	 * check, so guard here rather than crash on an uninitialised context. */
	if (!sr_ctx)
		return FALSE;

	ret = sr_resource_open(sr_ctx, &res, SR_RESOURCE_FIRMWARE, name);
	if (ret != SR_OK)
		return FALSE;

	sr_resource_close(sr_ctx, &res);
	return TRUE;
}

/* Print every directory libsigrok searches for firmware, in order, so the user
 * knows exactly where to drop the file. */
static void dslogic_log_firmware_search_paths(void)
{
	GSList *paths, *p;

	paths = sr_resourcepaths_get(SR_RESOURCE_FIRMWARE);
	if (!paths) {
		sr_err("    (no firmware search path configured)");
		return;
	}

	for (p = paths; p; p = p->next)
		sr_err("      %s", (const char *)p->data);

	g_slist_free_full(paths, g_free);
}

/* Report a missing firmware file with an actionable message. Kept separate from
 * the boolean check so the caller can decide whether to skip the device. */
SR_PRIV void dslogic_report_missing_firmware(struct sr_context *sr_ctx,
		const struct DSL_profile *prof, const char *name)
{
	(void)sr_ctx;

	sr_err("========================================================");
	sr_err("Firmware file is missing -- cannot initialize device.");
	sr_err("  Device : %s (VID:PID %04x:%04x)",
	       prof->model, prof->vid, prof->pid);
	sr_err("  Missing: %s", name);
	sr_err("");
	sr_err("The device will not be recognized until this file is");
	sr_err("installed. Without it the FX2 never re-enumerates and the");
	sr_err("USB handle stays claimed, which surfaces as a bogus");
	sr_err("\"interface already claimed\" / \"driver problem\" error.");
	sr_err("");
	sr_err("Place the file in any of these directories:");
	dslogic_log_firmware_search_paths();
	sr_err("");
	sr_err("Or set the SIGROK_FIRMWARE_PATH environment variable to the");
	sr_err("directory holding '%s'.", name);
	sr_err("========================================================");
}

/* ===========================================================================
 * dslogic_* functions (called by api.c)
 * =========================================================================== */

SR_PRIV int dslogic_fpga_firmware_upload(const struct sr_dev_inst *sdi)
{
	const char *name = NULL;
	uint64_t sum;
	struct sr_resource bitstream;
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	unsigned char *buf;
	ssize_t chunksize;
	int transferred;
	int result, ret;
	const uint8_t cmd[3] = {0, 0, 0};

	drvc = sdi->driver->context;
	devc = sdi->priv;
	usb = sdi->conn;

	/* Prefer the per-profile fpga_bit33/fpga_bit50 names when available,
	 * selecting based on the current voltage threshold. Fall back to the
	 * legacy model-name lookup otherwise. */
	if (devc->profile->fpga_bit33 && devc->profile->fpga_bit50) {
		if (devc->vth < 1.40)
			name = devc->profile->fpga_bit33;
		else
			name = devc->profile->fpga_bit50;
	} else if (!strcmp(devc->profile->model, "DSLogic")) {
		if (devc->vth < 1.40)
			name = "DSLogic33.bin";
		else
			name = "DSLogic50.bin";
	} else if (!strcmp(devc->profile->model, "DSLogic Pro")) {
		name = "DSLogicPro.bin";
	} else if (!strcmp(devc->profile->model, "DSLogic Plus") ||
		   !strcmp(devc->profile->model, "DSLogic PLus")) {
		name = "DSLogicPlus.bin";
	} else if (!strcmp(devc->profile->model, "DSLogic Basic")) {
		name = "DSLogicBasic.bin";
	} else if (!strcmp(devc->profile->model, "DSCope")) {
		name = "DSCope.bin";
	} else {
		sr_err("Failed to select FPGA firmware.");
		return SR_ERR;
	}

	sr_dbg("Uploading FPGA firmware '%s'.", name);

	/* Check presence first: sr_resource_open() failure below would report the
	 * generic "Firmware not found." without naming the file or the searched
	 * directories. */
	if (!dslogic_firmware_exists(drvc->sr_ctx, name)) {
		dslogic_report_missing_firmware(drvc->sr_ctx, devc->profile, name);
		return SR_ERR;
	}

	result = sr_resource_open(drvc->sr_ctx, &bitstream,
			SR_RESOURCE_FIRMWARE, name);
	if (result != SR_OK)
		return result;

	/* Tell the device firmware is coming. */
	if ((ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_CONFIG, 0x0000, 0x0000,
			(unsigned char *)&cmd, sizeof(cmd), USB_TIMEOUT)) < 0) {
		sr_err("Failed to upload FPGA firmware: %s.", libusb_error_name(ret));
		sr_resource_close(drvc->sr_ctx, &bitstream);
		return SR_ERR;
	}

	/* Give the FX2 time to get ready for FPGA firmware upload. */
	g_usleep(FPGA_UPLOAD_DELAY);

	buf = g_malloc(FW_BUFSIZE);
	sum = 0;
	result = SR_OK;
	while (1) {
		chunksize = sr_resource_read(drvc->sr_ctx, &bitstream,
				buf, FW_BUFSIZE);
		if (chunksize < 0)
			result = SR_ERR;
		if (chunksize <= 0)
			break;

		if ((ret = libusb_bulk_transfer(usb->devhdl, 2 | LIBUSB_ENDPOINT_OUT,
				buf, chunksize, &transferred, USB_TIMEOUT)) < 0) {
			sr_err("Unable to configure FPGA firmware: %s.",
					libusb_error_name(ret));
			result = SR_ERR;
			break;
		}
		sum += transferred;
		sr_spew("Uploaded %" PRIu64 "/%" PRIu64 " bytes.",
			sum, bitstream.size);

		if (transferred != chunksize) {
			sr_err("Short transfer while uploading FPGA firmware.");
			result = SR_ERR;
			break;
		}
	}
	g_free(buf);
	sr_resource_close(drvc->sr_ctx, &bitstream);

	if (result == SR_OK)
		sr_dbg("FPGA firmware upload done.");

	return result;
}

SR_PRIV int dslogic_set_voltage_threshold(const struct sr_dev_inst *sdi, double threshold)
{
	int ret;
	struct dev_context *const devc = sdi->priv;
	const struct sr_usb_dev_inst *const usb = sdi->conn;
	const uint8_t value = (threshold / 5.0) * 255;
	const uint16_t cmd = value | (DS_ADDR_VTH << 8);

	/* Send the control command. */
	ret = libusb_control_transfer(usb->devhdl,
			LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
			DS_CMD_WR_REG, 0x0000, 0x0000,
			(unsigned char *)&cmd, sizeof(cmd), 3000);
	if (ret < 0) {
		sr_err("Unable to set voltage-threshold register: %s.",
		libusb_error_name(ret));
		return SR_ERR;
	}

	devc->vth = threshold;

	return SR_OK;
}

SR_PRIV int dslogic_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di)
{
	libusb_device **devlist;
	struct sr_usb_dev_inst *usb;
	struct libusb_device_descriptor des;
	struct dev_context *devc;
	struct drv_context *drvc;
	struct version_info vi;
	int ret = SR_ERR, i, device_count;
	uint8_t revid;
	char connection_id[64];

	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
		       libusb_error_name(device_count));
		return SR_ERR;
	}

	for (i = 0; i < device_count; i++) {
		libusb_get_device_descriptor(devlist[i], &des);

		if (des.idVendor != devc->profile->vid
		    || des.idProduct != devc->profile->pid)
			continue;

		if ((sdi->status == SR_ST_INITIALIZING) ||
				(sdi->status == SR_ST_INACTIVE)) {
			/* Check device by its physical USB bus/port address. */
			if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
				continue;

			if (strcmp(sdi->connection_id, connection_id))
				/* This is not the one. */
				continue;
		}

		if (!(ret = libusb_open(devlist[i], &usb->devhdl))) {
			if (usb->address == 0xff)
				/*
				 * First time we touch this device after FW
				 * upload, so we don't know the address yet.
				 */
				usb->address = libusb_get_device_address(devlist[i]);
		} else {
			sr_err("Failed to open device: %s.",
			       libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}

		if (libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER)) {
			if (libusb_kernel_driver_active(usb->devhdl, USB_INTERFACE) == 1) {
				if ((ret = libusb_detach_kernel_driver(usb->devhdl, USB_INTERFACE)) < 0) {
					sr_err("Failed to detach kernel driver: %s.",
						libusb_error_name(ret));
					ret = SR_ERR;
					break;
				}
			}
		}

		ret = command_get_fw_version(usb->devhdl, &vi);
		if (ret != SR_OK) {
			sr_err("Failed to get firmware version.");
			break;
		}

		ret = command_get_revid_version(sdi, &revid);
		if (ret != SR_OK) {
			sr_err("Failed to get REVID.");
			break;
		}

		/*
		 * Changes in major version mean incompatible/API changes, so
		 * bail out if we encounter an incompatible version.
		 * Different minor versions are OK, they should be compatible.
		 */
		if (vi.major != DSL_REQUIRED_VERSION_MAJOR) {
			sr_err("Expected firmware version %d.x, "
			       "got %d.%d.", DSL_REQUIRED_VERSION_MAJOR,
			       vi.major, vi.minor);
			ret = SR_ERR;
			break;
		}

		sr_info("Opened device on %d.%d (logical) / %s (physical), "
			"interface %d, firmware %d.%d.",
			usb->bus, usb->address, connection_id,
			USB_INTERFACE, vi.major, vi.minor);

		sr_info("Detected REVID=%d, it's a Cypress CY7C68013%s.",
			revid, (revid != 1) ? " (FX2)" : "A (FX2LP)");

		ret = SR_OK;

		break;
	}

	libusb_free_device_list(devlist, 1);

	return ret;
}

SR_PRIV struct dev_context *dslogic_dev_new(void)
{
	struct dev_context *devc;

	devc = g_malloc0(sizeof(struct dev_context));
	devc->profile = NULL;
	devc->fw_updated = 0;
	devc->cur_samplerate = 0;
	devc->limit_samples = 0;
	devc->actual_samples = 0;
	devc->actual_bytes = 0;
	devc->capture_ratio = 0;
	devc->clock_type = FALSE;
	devc->clock_edge = FALSE;
	devc->rle_mode = FALSE;
	devc->rle_support = TRUE;
	devc->instant = FALSE;
	devc->continuous_mode = FALSE;
	devc->external_clock = FALSE;
	devc->op_mode = LO_OP_STREAM;
	devc->stream = TRUE;
	devc->test_mode = 0;
	devc->buf_options = 0;
	devc->ch_mode = 0;
	devc->samplerates_min_index = 0;
	devc->samplerates_max_index = 0;
	devc->th_level = 0;
	devc->vth = 1.0;
	devc->filter = 0;
	devc->trigger_stage = 0;
	devc->trigger_pos = 0;
	devc->timebase = 10000;
	devc->max_height = 0;
	devc->trigger_channel = 0;
	devc->trigger_slope = 0;
	devc->trigger_source = 0;
	devc->trigger_hrate = 0;
	devc->trigger_hpos = 0;
	devc->trigger_holdoff = 0;
	devc->trigger_margin = 8;
	devc->zero = FALSE;
	devc->cali = FALSE;
	devc->tune = FALSE;
	devc->tune_index = -1;
	devc->zero_branch = FALSE;
	devc->zero_comb_fgain = FALSE;
	devc->zero_comb = FALSE;
	devc->roll = FALSE;
	devc->data_lock = FALSE;
	devc->unit_pitch = 0;
	devc->status = DSL_FINISH;
	devc->mstatus_valid = FALSE;
	devc->abort = FALSE;
	devc->overflow = FALSE;
	devc->bw_limit = 0;
	devc->empty_poll_count = 0;
	devc->is_loop = 0;
	devc->mode = DSL_MODE_LOGIC;
	devc->sent_samples = 0;
	devc->acq_aborted = FALSE;
	devc->ctx = NULL;
	devc->deinterleave_buffer = NULL;

	return devc;
}

SR_PRIV int dslogic_acquisition_start(const struct sr_dev_inst *sdi)
{
	const unsigned int timeout = get_timeout(sdi);

	struct sr_dev_driver *di;
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct dsl_trigger_pos *tpos;
	struct libusb_transfer *transfer;
	int ret;

	di = sdi->driver;
	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	devc->ctx = drvc->sr_ctx;
	devc->sent_samples = 0;
	devc->empty_transfer_count = 0;
	devc->acq_aborted = FALSE;
	devc->trigger_pos = 0;

	/* Loop mode: keep limit_samples non-zero so the app ring buffer
	 * (get_ring_sample_count() -> limit_samples) sizes to the selected
	 * duration window. The receive_transfer stop condition guards on
	 * !is_loop, so loop keeps streaming forever. Aligns with PXView-1.5.8. */

	usb_source_add(sdi->session, devc->ctx, timeout, receive_data, drvc);

	if ((ret = command_stop_acquisition(sdi)) != SR_OK)
		return ret;

	if (devc->mode == DSL_MODE_LOGIC) {
		if ((ret = fpga_configure(sdi)) != SR_OK)
			return ret;
	} else {
		/* DSO / ANALOG mode: send DSO sync, update trigger position,
		 * then arm the FPGA with full DSL_setting via dsl_fpga_arm(). */
		devc->actual_samples = devc->limit_samples;
		ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_DSO_SYNC));
		if (ret != SR_OK)
			return ret;
		ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_HORIZ_TRIGGERPOS));
		if (ret != SR_OK)
			return ret;
		if ((ret = dsl_fpga_arm(sdi)) != SR_OK)
			return ret;
	}

	if ((ret = command_start_acquisition(sdi)) != SR_OK)
		return ret;

	sr_dbg("Getting trigger.");
	tpos = g_malloc(sizeof(struct dsl_trigger_pos));
	transfer = libusb_alloc_transfer(0);
	libusb_fill_bulk_transfer(transfer, usb->devhdl, 6 | LIBUSB_ENDPOINT_IN,
			(unsigned char *)tpos, sizeof(struct dsl_trigger_pos),
			trigger_receive, (void *)sdi, 0);
	if ((ret = libusb_submit_transfer(transfer)) < 0) {
		sr_err("Failed to request trigger: %s.", libusb_error_name(ret));
		libusb_free_transfer(transfer);
		g_free(tpos);
		return SR_ERR;
	}

	devc->transfers = g_try_malloc0(sizeof(*devc->transfers));
	if (!devc->transfers) {
		sr_err("USB trigger_pos transfer malloc failed.");
		return SR_ERR_MALLOC;
	}
	devc->num_transfers = 1;
	devc->submitted_transfers++;
	devc->transfers[0] = transfer;

	return ret;
}

SR_PRIV int dslogic_acquisition_stop(struct sr_dev_inst *sdi)
{
	command_stop_acquisition(sdi);
	abort_acquisition(sdi->priv);
	return SR_OK;
}

/* ===========================================================================
 * dsl_* register / DSO / NVM / probe access functions
 * (ported from old dsl.c, use command_ctl_wr/rd from command.c)
 * =========================================================================== */

SR_PRIV int dsl_wr_reg(const struct sr_dev_inst *sdi, uint8_t addr, uint8_t value)
{
	struct sr_usb_dev_inst *usb;
	struct libusb_device_handle *hdl;
	struct ctl_wr_cmd wr_cmd;
	int ret;

	usb = sdi->conn;
	hdl = usb->devhdl;

	wr_cmd.header.dest = DSL_CTL_I2C_REG;
	wr_cmd.header.offset = addr;
	wr_cmd.header.size = 1;
	wr_cmd.data[0] = value;
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
		sr_err("Sent DSL_CTL_I2C_REG command failed.");
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dsl_rd_reg(const struct sr_dev_inst *sdi, uint8_t addr, uint8_t *value)
{
	struct sr_usb_dev_inst *usb;
	struct ctl_rd_cmd rd_cmd;
	int ret;

	usb = sdi->conn;

	rd_cmd.header.dest = DSL_CTL_I2C_STATUS;
	rd_cmd.header.offset = addr;
	rd_cmd.header.size = 1;
	rd_cmd.data = value;
	if ((ret = command_ctl_rd(usb->devhdl, rd_cmd)) != SR_OK) {
		sr_err("Sent DSL_CTL_I2C_STATUS read command failed.");
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dsl_wr_ext(const struct sr_dev_inst *sdi, uint8_t addr, uint8_t value)
{
	struct sr_usb_dev_inst *usb;
	struct libusb_device_handle *hdl;
	struct ctl_wr_cmd wr_cmd;
	struct dev_context *devc = sdi->priv;
	uint8_t rdata;
	int ret;

	if (devc->profile->dev_caps.feature_caps & CAPS_FEATURE_POGOPIN) {
		usb = sdi->conn;
		hdl = usb->devhdl;

		wr_cmd.header.dest = DSL_CTL_I2C_EXT;
		wr_cmd.header.offset = addr;
		wr_cmd.header.size = 1;
		wr_cmd.data[0] = value;
		if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
			sr_err("Sent DSL_CTL_I2C_EXT command failed.");
			return SR_ERR;
		}
		return SR_OK;
	}

	/* Non-pogopin path: bit-bang through the EI2C controller. */
	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_TXR_OFF, EI2C_AWR);
	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_STA | bmEI2C_WR);
	ret = dsl_rd_reg(sdi, EI2C_ADDR + EI2C_SR_OFF, &rdata);
	if (rdata & bmEI2C_RXNACK) {
		dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_STO | bmEI2C_WR);
		return SR_ERR;
	}

	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_TXR_OFF, addr);
	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_WR);
	ret = dsl_rd_reg(sdi, EI2C_ADDR + EI2C_SR_OFF, &rdata);
	if (rdata & bmEI2C_RXNACK) {
		dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_STO | bmEI2C_WR);
		return SR_ERR;
	}

	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_TXR_OFF, value);
	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_STO | bmEI2C_WR);
	ret = dsl_rd_reg(sdi, EI2C_ADDR + EI2C_SR_OFF, &rdata);
	if (rdata & bmEI2C_RXNACK) {
		dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_STO | bmEI2C_WR);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dsl_rd_ext(const struct sr_dev_inst *sdi, unsigned char *ctx, uint16_t addr, uint8_t len)
{
	struct sr_usb_dev_inst *usb;
	struct libusb_device_handle *hdl;
	struct ctl_rd_cmd rd_cmd;
	struct dev_context *devc = sdi->priv;
	uint8_t rdata;
	int ret;

	if (devc->profile->dev_caps.feature_caps & CAPS_FEATURE_POGOPIN) {
		usb = sdi->conn;
		hdl = usb->devhdl;

		rd_cmd.header.dest = DSL_CTL_I2C_EXT;
		rd_cmd.header.size = len;
		rd_cmd.header.offset = addr;
		rd_cmd.data = ctx;
		if ((ret = command_ctl_rd(hdl, rd_cmd)) != SR_OK) {
			sr_err("Sent DSL_CTL_I2C_EXT read command failed.");
			return SR_ERR;
		}
		return SR_OK;
	}

	/* Non-pogopin path: bit-bang through the EI2C controller. */
	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_TXR_OFF, EI2C_AWR);
	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_STA | bmEI2C_WR);
	ret = dsl_rd_reg(sdi, EI2C_ADDR + EI2C_SR_OFF, &rdata);
	if (rdata & bmEI2C_RXNACK) {
		dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_STO | bmEI2C_WR);
		return SR_ERR;
	}

	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_TXR_OFF, (uint8_t)addr);
	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_WR);
	ret = dsl_rd_reg(sdi, EI2C_ADDR + EI2C_SR_OFF, &rdata);
	if (rdata & bmEI2C_RXNACK) {
		dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_STO | bmEI2C_WR);
		return SR_ERR;
	}

	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_TXR_OFF, EI2C_ARD);
	ret = dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_STA | bmEI2C_WR);
	ret = dsl_rd_reg(sdi, EI2C_ADDR + EI2C_SR_OFF, &rdata);
	if (rdata & bmEI2C_RXNACK) {
		dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_STO | bmEI2C_WR);
		return SR_ERR;
	}

	while (--len) {
		dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_RD);
		dsl_rd_reg(sdi, EI2C_ADDR + EI2C_RXR_OFF, ctx);
		ctx++;
	}
	dsl_wr_reg(sdi, EI2C_ADDR + EI2C_CR_OFF, bmEI2C_STO | bmEI2C_RD | bmEI2C_NACK);
	dsl_rd_reg(sdi, EI2C_ADDR + EI2C_RXR_OFF, ctx);

	return SR_OK;
}

SR_PRIV int dsl_wr_dso(const struct sr_dev_inst *sdi, uint64_t cmd)
{
	struct sr_usb_dev_inst *usb;
	struct libusb_device_handle *hdl;
	struct ctl_wr_cmd wr_cmd;
	int ret;

	usb = sdi->conn;
	hdl = usb->devhdl;

	wr_cmd.header.dest = DSL_CTL_I2C_DSO;
	wr_cmd.header.offset = 0;
	wr_cmd.header.size = 8;
	wr_cmd.data[0] = (uint8_t)cmd;
	wr_cmd.data[1] = (uint8_t)(cmd >> 8);
	wr_cmd.data[2] = (uint8_t)(cmd >> 16);
	wr_cmd.data[3] = (uint8_t)(cmd >> 24);
	wr_cmd.data[4] = (uint8_t)(cmd >> 32);
	wr_cmd.data[5] = (uint8_t)(cmd >> 40);
	wr_cmd.data[6] = (uint8_t)(cmd >> 48);
	wr_cmd.data[7] = (uint8_t)(cmd >> 56);
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
		sr_err("Sent DSL_CTL_I2C_DSO command failed.");
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dsl_wr_nvm(const struct sr_dev_inst *sdi, unsigned char *ctx, uint16_t addr, uint8_t len)
{
	struct sr_usb_dev_inst *usb;
	struct libusb_device_handle *hdl;
	struct ctl_wr_cmd wr_cmd;
	int ret;
	int i;

	usb = sdi->conn;
	hdl = usb->devhdl;

	wr_cmd.header.dest = DSL_CTL_NVM;
	wr_cmd.header.offset = addr;
	wr_cmd.header.size = len;
	for (i = 0; i < len; i++)
		wr_cmd.data[i] = *(ctx + i);
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
		sr_err("Sent DSL_CTL_NVM write command failed.");
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dsl_rd_nvm(const struct sr_dev_inst *sdi, unsigned char *ctx, uint16_t addr, uint8_t len)
{
	struct sr_usb_dev_inst *usb;
	struct libusb_device_handle *hdl;
	struct ctl_rd_cmd rd_cmd;
	int ret;

	usb = sdi->conn;
	hdl = usb->devhdl;

	rd_cmd.header.dest = DSL_CTL_NVM;
	rd_cmd.header.size = len;
	rd_cmd.header.offset = addr;
	rd_cmd.data = ctx;
	if ((ret = command_ctl_rd(hdl, rd_cmd)) != SR_OK) {
		sr_err("Sent DSL_CTL_NVM read command failed.");
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dsl_rd_probe(const struct sr_dev_inst *sdi, unsigned char *ctx, uint16_t addr, uint8_t len)
{
	struct sr_usb_dev_inst *usb;
	struct libusb_device_handle *hdl;
	struct ctl_rd_cmd rd_cmd;
	int ret;

	usb = sdi->conn;
	hdl = usb->devhdl;

	rd_cmd.header.dest = DSL_CTL_I2C_PROBE;
	rd_cmd.header.size = len;
	rd_cmd.header.offset = addr;
	rd_cmd.data = ctx;
	if ((ret = command_ctl_rd(hdl, rd_cmd)) != SR_OK) {
		sr_err("Sent DSL_CTL_I2C_PROBE read command failed.");
		return SR_ERR;
	}

	return SR_OK;
}

/* ===========================================================================
 * dsl_* ADC / calibration functions
 * =========================================================================== */

SR_PRIV int dsl_config_adc(const struct sr_dev_inst *sdi, const struct DSL_adc_config *config)
{
	const struct DSL_adc_config *p = config;
	while (p->dest) {
		assert((p->cnt > 0) && (p->cnt <= 4));
		if (p->delay > 0)
			g_usleep(p->delay * 1000);
		for (int i = 0; i < p->cnt; i++) {
			dsl_wr_reg(sdi, p->dest, p->byte[i]);
		}
		p++;
	}
	return SR_OK;
}

SR_PRIV double dsl_adc_code2fgain(uint8_t code)
{
	double xcode = code & 0x40 ? -(~code & 0x3F) : code & 0x3F;
	return (1 + xcode / (1 << 13));
}

SR_PRIV uint8_t dsl_adc_fgain2code(double gain)
{
	int xratio = (int)((gain - 1) * (1 << 13));
	uint8_t code = xratio > 63 ? 63 :
	               xratio > 0 ? (uint8_t)xratio :
	               xratio < -63 ? 64 : (uint8_t)(~(-xratio) & 0x7F);
	return code;
}

SR_PRIV int dsl_config_adc_fgain(const struct sr_dev_inst *sdi, uint8_t branch, double gain0, double gain1)
{
	dsl_wr_reg(sdi, ADCC_ADDR, 0x00);
	dsl_wr_reg(sdi, ADCC_ADDR, dsl_adc_fgain2code(gain0));
	dsl_wr_reg(sdi, ADCC_ADDR, dsl_adc_fgain2code(gain1));
	dsl_wr_reg(sdi, ADCC_ADDR, 0x34 + branch);
	return SR_OK;
}

SR_PRIV int dsl_config_fpga_fgain(const struct sr_dev_inst *sdi)
{
	GSList *l;
	int ret = SR_OK;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *probe = (struct sr_channel *)l->data;
		if (probe->index == 0) {
			ret = dsl_wr_reg(sdi, ADCC_ADDR + 3, (DSL_CH_PRIV(probe)->digi_fgain & 0x00FF));
			ret = dsl_wr_reg(sdi, ADCC_ADDR + 4, (DSL_CH_PRIV(probe)->digi_fgain >> 8));
		} else if (probe->index == 1) {
			ret = dsl_wr_reg(sdi, ADCC_ADDR + 5, (DSL_CH_PRIV(probe)->digi_fgain & 0x00FF));
			ret = dsl_wr_reg(sdi, ADCC_ADDR + 6, (DSL_CH_PRIV(probe)->digi_fgain >> 8));
		}
	}

	return ret;
}

SR_PRIV int dsl_skew_fpga_fgain(const struct sr_dev_inst *sdi, gboolean comb, double skew[])
{
	uint8_t fgain_up = 0;
	uint8_t fgain_dn = 0;
	GSList *l;
	gboolean tmp;
	int ret = SR_OK;

	for (int i = 0; i <= 7; i++) {
		tmp = (-skew[i] > 1.6 * MAX_ACC_VARIANCE);
		fgain_up += (tmp << i);
	}

	if (comb) {
		for (l = sdi->channels; l; l = l->next) {
			struct sr_channel *probe = (struct sr_channel *)l->data;
			if (probe->index == 0) {
				DSL_CH_PRIV(probe)->digi_fgain |= (DSL_CH_PRIV(probe)->digi_fgain & 0xFF00) + fgain_up;
				fgain_up = (DSL_CH_PRIV(probe)->digi_fgain & 0x00FF);
				break;
			}
		}
		ret = dsl_wr_reg(sdi, ADCC_ADDR + 3, fgain_up);
	} else {
		for (l = sdi->channels; l; l = l->next) {
			struct sr_channel *probe = (struct sr_channel *)l->data;
			if (probe->index == 0) {
				DSL_CH_PRIV(probe)->digi_fgain |= (fgain_up << 8) + (DSL_CH_PRIV(probe)->digi_fgain & 0x00FF);
				fgain_up = (DSL_CH_PRIV(probe)->digi_fgain >> 8);
				break;
			}
		}
		ret = dsl_wr_reg(sdi, ADCC_ADDR + 4, fgain_up);
	}

	for (int i = 0; i <= 7; i++) {
		tmp = (skew[i] > 1.6 * MAX_ACC_VARIANCE);
		fgain_dn += (tmp << i);
	}

	if (comb) {
		for (l = sdi->channels; l; l = l->next) {
			struct sr_channel *probe = (struct sr_channel *)l->data;
			if (probe->index == 1) {
				DSL_CH_PRIV(probe)->digi_fgain |= (DSL_CH_PRIV(probe)->digi_fgain & 0xFF00) + fgain_dn;
				fgain_dn = (DSL_CH_PRIV(probe)->digi_fgain & 0x00FF);
				break;
			}
		}
		ret = dsl_wr_reg(sdi, ADCC_ADDR + 5, fgain_dn);
	} else {
		for (l = sdi->channels; l; l = l->next) {
			struct sr_channel *probe = (struct sr_channel *)l->data;
			if (probe->index == 1) {
				DSL_CH_PRIV(probe)->digi_fgain |= (fgain_dn << 8) + (DSL_CH_PRIV(probe)->digi_fgain & 0x00FF);
				fgain_dn = (DSL_CH_PRIV(probe)->digi_fgain >> 8);
				break;
			}
		}
		ret = dsl_wr_reg(sdi, ADCC_ADDR + 6, fgain_dn);
	}

	return ret;
}

SR_PRIV int dsl_probe_cali_fgain(struct dev_context *devc, struct sr_channel *probe,
                                 double mean, gboolean comb, gboolean reset)
{
	const double UPGAIN = 1.0077;
	const double DNGAIN = 0.9923;
	const double MDGAIN = 1;
	const double ignore_ratio = 2.0;
	const double ratio = 2.0;
	double drift;
	struct dsl_channel_priv *p = DSL_CH_PRIV(probe);

	if (reset) {
		if (comb) {
			p->cali_comb_fgain0 = MDGAIN;
			p->cali_comb_fgain1 = MDGAIN;
			p->cali_comb_fgain2 = MDGAIN;
			p->cali_comb_fgain3 = MDGAIN;
		} else {
			p->cali_fgain0 = MDGAIN;
			p->cali_fgain1 = MDGAIN;
			p->cali_fgain2 = MDGAIN;
			p->cali_fgain3 = MDGAIN;
		}
	} else {
		if (comb) {
			if (probe->index == 0) {
				drift = (devc->mstatus.ch0_acc_mean / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_comb_fgain0 /= (1 + drift);
				drift = (devc->mstatus.ch0_acc_mean_p1 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_comb_fgain1 /= (1 + drift);
				drift = (devc->mstatus.ch1_acc_mean_p2 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_comb_fgain2 /= (1 + drift);
				drift = (devc->mstatus.ch1_acc_mean_p3 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_comb_fgain3 /= (1 + drift);
			} else {
				drift = (devc->mstatus.ch1_acc_mean_p1 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_comb_fgain0 /= (1 + drift);
				drift = (devc->mstatus.ch1_acc_mean / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_comb_fgain1 /= (1 + drift);
				drift = (devc->mstatus.ch0_acc_mean_p3 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_comb_fgain2 /= (1 + drift);
				drift = (devc->mstatus.ch0_acc_mean_p2 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_comb_fgain3 /= (1 + drift);
			}

			p->cali_comb_fgain0 = MAX(MIN(p->cali_comb_fgain0, UPGAIN), DNGAIN);
			p->cali_comb_fgain1 = MAX(MIN(p->cali_comb_fgain1, UPGAIN), DNGAIN);
			p->cali_comb_fgain2 = MAX(MIN(p->cali_comb_fgain2, UPGAIN), DNGAIN);
			p->cali_comb_fgain3 = MAX(MIN(p->cali_comb_fgain3, UPGAIN), DNGAIN);
		} else {
			if (probe->index == 0) {
				drift = (devc->mstatus.ch0_acc_mean / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_fgain0 /= (1 + drift);
				drift = (devc->mstatus.ch0_acc_mean_p2 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_fgain1 /= (1 + drift);
				drift = (devc->mstatus.ch0_acc_mean_p1 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_fgain2 /= (1 + drift);
				drift = (devc->mstatus.ch0_acc_mean_p3 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_fgain3 /= (1 + drift);
			} else {
				drift = (devc->mstatus.ch1_acc_mean / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_fgain0 /= (1 + drift);
				drift = (devc->mstatus.ch1_acc_mean_p2 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_fgain1 /= (1 + drift);
				drift = (devc->mstatus.ch1_acc_mean_p1 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_fgain2 /= (1 + drift);
				drift = (devc->mstatus.ch1_acc_mean_p3 / mean - 1) / ratio;
				if (fabs(drift) > MAX_ACC_VARIANCE / ignore_ratio)
					p->cali_fgain3 /= (1 + drift);
			}

			p->cali_fgain0 = MAX(MIN(p->cali_fgain0, UPGAIN), DNGAIN);
			p->cali_fgain1 = MAX(MIN(p->cali_fgain1, UPGAIN), DNGAIN);
			p->cali_fgain2 = MAX(MIN(p->cali_fgain2, UPGAIN), DNGAIN);
			p->cali_fgain3 = MAX(MIN(p->cali_fgain3, UPGAIN), DNGAIN);
		}
	}

	return SR_OK;
}

SR_PRIV gboolean dsl_probe_fgain_inrange(struct sr_channel *probe, gboolean comb, double skew[])
{
	const double UPGAIN = 1.0077;
	const double DNGAIN = 0.9923;
	struct dsl_channel_priv *p = DSL_CH_PRIV(probe);

	if (comb) {
		if (probe->index == 0) {
			if (fabs(skew[0]) > MAX_ACC_VARIANCE && p->cali_comb_fgain0 > DNGAIN && p->cali_comb_fgain0 < UPGAIN)
				return TRUE;
			if (fabs(skew[1]) > MAX_ACC_VARIANCE && p->cali_comb_fgain1 > DNGAIN && p->cali_comb_fgain1 < UPGAIN)
				return TRUE;
			if (fabs(skew[6]) > MAX_ACC_VARIANCE && p->cali_comb_fgain2 > DNGAIN && p->cali_comb_fgain2 < UPGAIN)
				return TRUE;
			if (fabs(skew[7]) > MAX_ACC_VARIANCE && p->cali_comb_fgain3 > DNGAIN && p->cali_comb_fgain3 < UPGAIN)
				return TRUE;
		} else {
			if (fabs(skew[5]) > MAX_ACC_VARIANCE && p->cali_comb_fgain0 > DNGAIN && p->cali_comb_fgain0 < UPGAIN)
				return TRUE;
			if (fabs(skew[4]) > MAX_ACC_VARIANCE && p->cali_comb_fgain1 > DNGAIN && p->cali_comb_fgain1 < UPGAIN)
				return TRUE;
			if (fabs(skew[3]) > MAX_ACC_VARIANCE && p->cali_comb_fgain2 > DNGAIN && p->cali_comb_fgain2 < UPGAIN)
				return TRUE;
			if (fabs(skew[2]) > MAX_ACC_VARIANCE && p->cali_comb_fgain3 > DNGAIN && p->cali_comb_fgain3 < UPGAIN)
				return TRUE;
		}
	} else {
		if (probe->index == 0) {
			if (fabs(skew[0]) > MAX_ACC_VARIANCE && p->cali_fgain0 > DNGAIN && p->cali_fgain0 < UPGAIN)
				return TRUE;
			if (fabs(skew[2]) > MAX_ACC_VARIANCE && p->cali_fgain1 > DNGAIN && p->cali_fgain1 < UPGAIN)
				return TRUE;
			if (fabs(skew[1]) > MAX_ACC_VARIANCE && p->cali_fgain2 > DNGAIN && p->cali_fgain2 < UPGAIN)
				return TRUE;
			if (fabs(skew[3]) > MAX_ACC_VARIANCE && p->cali_fgain3 > DNGAIN && p->cali_fgain3 < UPGAIN)
				return TRUE;
		} else {
			if (fabs(skew[4]) > MAX_ACC_VARIANCE && p->cali_fgain0 > DNGAIN && p->cali_fgain0 < UPGAIN)
				return TRUE;
			if (fabs(skew[6]) > MAX_ACC_VARIANCE && p->cali_fgain1 > DNGAIN && p->cali_fgain1 < UPGAIN)
				return TRUE;
			if (fabs(skew[5]) > MAX_ACC_VARIANCE && p->cali_fgain2 > DNGAIN && p->cali_fgain2 < UPGAIN)
				return TRUE;
			if (fabs(skew[7]) > MAX_ACC_VARIANCE && p->cali_fgain3 > DNGAIN && p->cali_fgain3 < UPGAIN)
				return TRUE;
		}
	}

	return FALSE;
}

/* ===========================================================================
 * dslogic_dso_cmd_gen — generate DSO hardware command word
 * (ported from old dslogic.c, adapted for upstream libsigrok API)
 *
 * Builds a 64-bit command word for DSO settings (VDIV, coupling, samplerate,
 * trigger position/slope/source/value/margin/holdoff, DSO sync).
 * The command is sent to hardware via dsl_wr_dso().
 *
 * Config key IDs used here mirror the fork SR_CONF_* values; they are only
 * used locally as switch-case labels, not passed to the upstream config API.
 * =========================================================================== */

SR_PRIV uint64_t dslogic_dso_cmd_gen(const struct sr_dev_inst *sdi,
	struct sr_channel *ch, int cmd_id)
{
	struct dev_context *devc;
	uint64_t cmd = 0;
	int channel_cnt = 0;
	GSList *l;
	struct sr_channel *en_probe = ch;

	devc = sdi->priv;

	switch (cmd_id) {
	case DSO_CMD_PROBE_VDIV:
	case DSO_CMD_PROBE_EN:
	case DSO_CMD_TIMEBASE:
	case DSO_CMD_PROBE_COUPLING:
		for (l = sdi->channels; l; l = l->next) {
			struct sr_channel *probe = (struct sr_channel *)l->data;
			if (probe->enabled) {
				channel_cnt += probe->index + 0x1;
				en_probe = probe;
			}
		}
		if (channel_cnt == 0)
			return 0x0;

		/* --VDBS */
		if (channel_cnt != 1)
			en_probe = ch;
		if (en_probe && en_probe->priv) {
			switch (DSL_CH_PRIV(en_probe)->vdiv) {
			case 5:     cmd += 0x247000; break;
			case 10:    cmd += 0x23D000; break;
			case 20:    cmd += 0x22F000; break;
			case 50:    cmd += 0x21C800; break;
			case 100:   cmd += 0x20E800; break;
			case 200:   cmd += 0x200800; break;
			case 500:   cmd += 0x2F000; break;
			case 1000:  cmd += 0x21100; break;
			case 2000:  cmd += 0x13000; break;
			case 5000:  cmd += 0x00800; break;
			default: cmd += 0x21100; break;
			}
		}
		/* --DC/AC */
		if (channel_cnt == 1) {
			for (l = sdi->channels; l; l = l->next) {
				struct sr_channel *probe = (struct sr_channel *)l->data;
				if (probe->priv && DSL_CH_PRIV(probe)->coupling == DSL_COUPLING_AC)
					cmd += 0x100000;
				break;
			}
		} else {
			if (ch && ch->priv && DSL_CH_PRIV(ch)->coupling == DSL_COUPLING_AC)
				cmd += 0x100000;
		}

		/* --Channel */
		if (devc->mode != DSL_MODE_LOGIC) {
			if (channel_cnt == 1)
				cmd += 0xC00000;
			else if (ch && ch->index == 0)
				cmd += 0x400000;
			else if (ch && ch->index == 1)
				cmd += 0x800000;
			else
				cmd += 0x000000;
		}

		/* --Header */
		cmd += 0x55000000;
		break;
	case DSO_CMD_SAMPLERATE:
		for (l = sdi->channels; l; l = l->next) {
			struct sr_channel *probe = (struct sr_channel *)l->data;
			channel_cnt += probe->enabled;
		}
		cmd += 0x18;
		{
			uint32_t divider = (uint32_t)ceil(channel_modes[devc->ch_mode].max_samplerate * 1.0
				/ devc->cur_samplerate / channel_cnt);
			cmd += (uint64_t)divider << 8;
		}
		break;
	case DSO_CMD_HORIZ_TRIGGERPOS:
		cmd += 0x20;
		cmd += (uint64_t)devc->trigger_hpos << 8;
		break;
	case DSO_CMD_TRIGGER_SLOPE:
		cmd += 0x28;
		cmd += (uint64_t)devc->trigger_slope << 8;
		break;
	case DSO_CMD_TRIGGER_SOURCE:
		cmd += 0x30;
		cmd += (uint64_t)devc->trigger_source << 8;
		break;
	case DSO_CMD_TRIGGER_VALUE:
		cmd += 0x38;
		for (l = sdi->channels; l; l = l->next) {
			struct sr_channel *probe = (struct sr_channel *)l->data;
			if (probe->priv)
				cmd += (uint64_t)DSL_CH_PRIV(probe)->trig_value << (8 * (probe->index + 1));
		}
		break;
	case DSO_CMD_TRIGGER_MARGIN:
		cmd += 0x40;
		cmd += ((uint64_t)devc->trigger_margin << 8);
		break;
	case DSO_CMD_TRIGGER_HOLDOFF:
		cmd += 0x58;
		cmd += (uint64_t)devc->trigger_holdoff << 8;
		break;
	case DSO_CMD_DSO_SYNC:
		cmd = 0xa5a5a500;
		break;
	default:
		cmd = 0xFFFFFFFF;
	}

	return cmd;
}

SR_PRIV int dslogic_dso_init(const struct sr_dev_inst *sdi)
{
	int ret;
	GSList *l;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *probe = (struct sr_channel *)l->data;
		ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, probe, DSO_CMD_PROBE_COUPLING));
		if (ret != SR_OK) {
			sr_err("DSO set coupling of channel %d command failed!", probe->index);
			return ret;
		}
		ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, probe, DSO_CMD_PROBE_VDIV));
		if (ret != SR_OK) {
			sr_err("Set VDIV of channel %d command failed!", probe->index);
			return ret;
		}
	}

	ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_SAMPLERATE));
	if (ret != SR_OK) {
		sr_err("Set Sample Rate command failed!");
		return ret;
	}
	ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_HORIZ_TRIGGERPOS));
	if (ret != SR_OK) {
		sr_err("Set Horiz Trigger Position command failed!");
		return ret;
	}
	ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_TRIGGER_HOLDOFF));
	if (ret != SR_OK) {
		sr_err("Set Trigger Holdoff Time command failed!");
		return ret;
	}
	ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_TRIGGER_SLOPE));
	if (ret != SR_OK) {
		sr_err("Set Trigger Slope command failed!");
		return ret;
	}
	ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_TRIGGER_SOURCE));
	if (ret != SR_OK) {
		sr_err("Set Trigger Source command failed!");
		return ret;
	}
	ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_TRIGGER_VALUE));
	if (ret != SR_OK) {
		sr_err("Set Trigger Value command failed!");
		return ret;
	}
	ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_TRIGGER_MARGIN));
	if (ret != SR_OK) {
		sr_err("Set Trigger Margin command failed!");
		return ret;
	}
	return ret;
}

/* ===========================================================================
 * dsl_fpga_arm — build DSL_setting struct and ARM the FPGA
 * (ported from old dsl.c, simplified to use upstream sr_session_trigger_get)
 * =========================================================================== */

static unsigned int to_bytes_per_ms_devc(struct dev_context *devc, const struct sr_dev_inst *sdi)
{
	const size_t ch_count = dsl_en_ch_num(sdi);
	if (devc->mode == DSL_MODE_LOGIC) {
		return (unsigned int)ceil(devc->cur_samplerate / 1000.0 * ch_count / 8.0);
	} else {
		if (devc->cur_samplerate > SR_MHZ(100))
			return (unsigned int)(SR_MHZ(100) / 1000.0 * ch_count);
		else
			return (unsigned int)ceil(MAX(devc->cur_samplerate,
				channel_modes[devc->ch_mode].hw_min_samplerate) / 1000.0 * ch_count);
	}
}

SR_PRIV int dsl_fpga_arm(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct libusb_device_handle *hdl;
	struct DSL_setting setting;
	struct sr_trigger *trigger;
	struct sr_trigger_stage *stage;
	struct sr_trigger_match *match;
	const GSList *l, *m;
	struct ctl_wr_cmd wr_cmd;
	struct ctl_rd_cmd rd_cmd;
	uint8_t rd_cmd_data;
	int ret, transferred;
	int i, num_trigger_stages = 0;
	uint32_t tmp_u32;
	uint64_t tmp_u64;
	const int ch_num = dsl_en_ch_num(sdi);
	uint32_t arm_size;
	uint16_t mode = 0;
	uint16_t trig_pos;

	devc = sdi->priv;
	usb = sdi->conn;
	hdl = usb->devhdl;

	memset(&setting, 0, sizeof(setting));
	setting.sync = 0xf5a5f5a5;
	setting.mode_header = 0x0001;
	setting.divider_header = 0x0102;
	setting.count_header = 0x0302;
	setting.trig_pos_header = 0x0502;
	setting.trig_glb_header = 0x0701;
	setting.dso_count_header = 0x0802;
	setting.ch_en_header = 0x0a02;
	setting.fgain_header = 0x0c01;
	setting.trig_header = 0x40a0;
	setting.end_sync = 0xfa5afa5a;

	/* Default trigger arrays: mask all channels (no trigger) on every stage. */
	for (i = 0; i < NUM_TRIGGER_STAGES; i++) {
		setting.trig_mask0[i] = 0xffff;
		setting.trig_mask1[i] = 0xffff;
		setting.trig_value0[i] = 0;
		setting.trig_value1[i] = 0;
		setting.trig_edge0[i] = 0;
		setting.trig_edge1[i] = 0;
		setting.trig_logic0[i] = 2;
		setting.trig_logic1[i] = 2;
		setting.trig_count[i] = 0;
	}

	/* Parse the upstream session trigger (simple trigger on stage 0). */
	trigger = sr_session_trigger_get(sdi->session);
	if (trigger) {
		for (l = trigger->stages; l; l = l->next) {
			stage = l->data;
			num_trigger_stages++;
			for (m = stage->matches; m; m = m->next) {
				int channelbit;
				match = m->data;
				if (!match->channel->enabled)
					continue;
				channelbit = 1 << (match->channel->index);
				setting.trig_mask0[0] &= ~channelbit;
				setting.trig_mask1[0] &= ~channelbit;
				if (match->match == SR_TRIGGER_ONE) {
					setting.trig_value0[0] |= channelbit;
					setting.trig_value1[0] |= channelbit;
				} else if (match->match == SR_TRIGGER_FALLING) {
					setting.trig_edge0[0] |= channelbit;
					setting.trig_edge1[0] |= channelbit;
				} else if (match->match == SR_TRIGGER_RISING) {
					setting.trig_value0[0] |= channelbit;
					setting.trig_value1[0] |= channelbit;
					setting.trig_edge0[0] |= channelbit;
					setting.trig_edge1[0] |= channelbit;
				} else if (match->match == SR_TRIGGER_EDGE) {
					setting.trig_edge0[0] |= channelbit;
					setting.trig_edge1[0] |= channelbit;
				}
			}
		}
	}

	if (num_trigger_stages > 0)
		mode |= DS_MODE_TRIG_EN;

	/* Basic mode bits. */
	mode |= (uint16_t)(devc->clock_type << CLK_TYPE_BIT);
	mode |= (uint16_t)(devc->clock_edge << CLK_EDGE_BIT);
	mode |= (uint16_t)(devc->rle_mode << RLE_MODE_BIT);
	mode |= (uint16_t)((devc->mode == DSL_MODE_DSO) << DSO_MODE_BIT);
	mode |= (uint16_t)((devc->cur_samplerate == devc->profile->dev_caps.half_samplerate) << HALF_MODE_BIT);
	mode |= (uint16_t)((devc->cur_samplerate == devc->profile->dev_caps.quarter_samplerate) << QUAR_MODE_BIT);
	mode |= (uint16_t)(((devc->mode == DSL_MODE_ANALOG) || devc->is_loop) << ANALOG_MODE_BIT);
	mode |= (uint16_t)((devc->filter == 1) << FILTER_BIT);
	mode |= (uint16_t)(devc->instant << INSTANT_BIT);
	mode |= (uint16_t)((to_bytes_per_ms_devc(devc, sdi) < 1024) << SLOW_ACQ_BIT);
	mode |= (uint16_t)(devc->stream << STREAM_MODE_BIT);
	mode |= (uint16_t)((devc->test_mode == 1) << INT_TEST_BIT);   /* DSL_TEST_INTERNAL */
	mode |= (uint16_t)((devc->op_mode == LO_OP_EXTEST) << EXT_TEST_BIT);
	mode |= (uint16_t)((devc->op_mode == LO_OP_LPTEST) << LPB_TEST_BIT);

	setting.mode = mode;

	/* Sample-rate divider. */
	tmp_u32 = (devc->mode == DSL_MODE_DSO) ?
		(uint32_t)ceil(channel_modes[devc->ch_mode].max_samplerate * 1.0 / devc->cur_samplerate / ch_num) :
		(uint32_t)ceil(channel_modes[devc->ch_mode].hw_max_samplerate * 1.0 / devc->cur_samplerate);
	devc->unit_pitch = (uint16_t)ceil(channel_modes[devc->ch_mode].hw_min_samplerate * 1.0 / devc->cur_samplerate);
	setting.div_h = ((tmp_u32 >= channel_modes[devc->ch_mode].pre_div) ?
		(uint16_t)((channel_modes[devc->ch_mode].pre_div - 1U) << 8) :
		(uint16_t)((tmp_u32 - 1U) << 8));
	tmp_u32 = (uint32_t)ceil(tmp_u32 * 1.0 / channel_modes[devc->ch_mode].pre_div);
	setting.div_l = (uint16_t)(tmp_u32 & 0x0000ffff);
	setting.div_h += (uint16_t)(tmp_u32 >> 16);

	/* Capture counter. */
	tmp_u64 = devc->actual_samples;
	tmp_u64 >>= 4; /* hardware minimum unit 64 */
	setting.cnt_l = (uint16_t)(tmp_u64 & 0x0000ffff);
	setting.cnt_h = (uint16_t)(tmp_u64 >> 16);
	tmp_u64 = (devc->mode == DSL_MODE_DSO) ? devc->limit_samples : devc->actual_samples;
	setting.dso_cnt_l = (uint16_t)(tmp_u64 & 0x0000ffff);
	setting.dso_cnt_h = (uint16_t)(tmp_u64 >> 16);

	/* Trigger position. */
	tmp_u32 = (uint32_t)((devc->capture_ratio * devc->limit_samples) / 100);
	if (tmp_u32 < DSLOGIC_ATOMIC_SAMPLES)
		tmp_u32 = DSLOGIC_ATOMIC_SAMPLES;
	if (devc->stream)
		tmp_u32 = MIN(tmp_u32, (uint32_t)(dsl_channel_depth(sdi) * 10 / 100));
	else
		tmp_u32 = MIN(tmp_u32, (uint32_t)(dsl_channel_depth(sdi) * DS_MAX_TRIG_PERCENT / 100));
	setting.tpos_l = (uint16_t)(tmp_u32 & 0x0000ffff);
	setting.tpos_h = (uint16_t)(tmp_u32 >> 16);
	trig_pos = tmp_u32;

	/* Trigger global settings. */
	setting.trig_glb = (uint16_t)(((ch_num & 0x1f) << 8) + (num_trigger_stages & 0x00ff));

	/* Channel enable mapping. */
	setting.ch_en_l = 0;
	setting.ch_en_h = 0;
	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *probe = (struct sr_channel *)l->data;
		if (probe->index < 16)
			setting.ch_en_l += (uint16_t)(probe->enabled << probe->index);
		else
			setting.ch_en_h += (uint16_t)(probe->enabled << (probe->index - 16));
	}

	/* Digital fgain (from first channel). */
	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *probe = (struct sr_channel *)l->data;
		setting.fgain = DSL_CH_PRIV(probe)->digi_fgain;
		break;
	}

	/* Set GPIF to be wordwide (USB 2.0 only). */
	if (!(devc->profile->usb_speed == LIBUSB_SPEED_SUPER)) {
		wr_cmd.header.dest = DSL_CTL_WORDWIDE;
		wr_cmd.header.offset = 0;
		wr_cmd.header.size = 1;
		wr_cmd.data[0] = bmWR_WORDWIDE;
		if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
			sr_err("Sent DSL_CTL_WORDWIDE command failed.");
			return SR_ERR;
		}
	}

	/* Send bulk write control command. */
	arm_size = sizeof(struct DSL_setting) / sizeof(uint16_t);
	wr_cmd.header.dest = DSL_CTL_BULK_WR;
	wr_cmd.header.offset = 0;
	wr_cmd.header.size = 3;
	wr_cmd.data[0] = (uint8_t)arm_size;
	wr_cmd.data[1] = (uint8_t)(arm_size >> 8);
	wr_cmd.data[2] = (uint8_t)(arm_size >> 16);
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
		sr_err("Sent bulk write command of arm FPGA failed.");
		return SR_ERR;
	}

	/* Wait for sys_clr to deassert. */
	rd_cmd.header.dest = DSL_CTL_HW_STATUS;
	rd_cmd.header.offset = 0;
	rd_cmd.header.size = 1;
	rd_cmd_data = 0;
	rd_cmd.data = &rd_cmd_data;
	while (1) {
		if ((ret = command_ctl_rd(hdl, rd_cmd)) != SR_OK)
			return SR_ERR;
		if (rd_cmd_data & bmSYS_CLR)
			break;
	}

	/* Send the DSL_setting via bulk transfer. */
	ret = libusb_bulk_transfer(hdl, 2 | LIBUSB_ENDPOINT_OUT,
	                           (unsigned char *)&setting,
	                           sizeof(struct DSL_setting),
	                           &transferred, 1000);
	if (ret < 0) {
		sr_err("Unable to arm FPGA of dsl device: %s.",
		        libusb_error_name(ret));
		return SR_ERR;
	} else if (transferred != (int)sizeof(struct DSL_setting)) {
		sr_err("Arm FPGA error: expected transfer size %d; actually %d",
		        (int)sizeof(struct DSL_setting), transferred);
		return SR_ERR;
	}

	/* Assert INTRDY high (indicate data end). */
	wr_cmd.header.dest = DSL_CTL_INTRDY;
	wr_cmd.header.offset = 0;
	wr_cmd.header.size = 1;
	wr_cmd.data[0] = bmWR_INTRDY;
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK)
		return SR_ERR;

	/* Check FPGA_DONE bit. */
	rd_cmd.header.dest = DSL_CTL_HW_STATUS;
	rd_cmd.header.offset = 0;
	rd_cmd.header.size = 1;
	rd_cmd_data = 0;
	rd_cmd.data = &rd_cmd_data;
	if ((ret = command_ctl_rd(hdl, rd_cmd)) != SR_OK)
		return SR_ERR;
	if (rd_cmd_data & bmGPIF_DONE) {
		sr_info("Arm FPGA done");
		return SR_OK;
	} else {
		return SR_ERR;
	}
}

SR_PRIV int dsl_fpga_config(struct libusb_device_handle *hdl, const char *filename)
{
	FILE *fw;
	int chunksize, ret;
	unsigned char *buf;
	int transferred;
	uint64_t filesize;
	struct ctl_wr_cmd wr_cmd;
	struct ctl_rd_cmd rd_cmd;
	uint8_t rd_cmd_data;
	struct stat f_stat;

	sr_info("Configure FPGA using \"%s\"", filename);
	if ((fw = fopen(filename, "rb")) == NULL) {
		sr_err("Unable to open FPGA bit file %s for reading: %s",
		       filename, strerror(errno));
		return SR_ERR;
	}

	if (stat(filename, &f_stat) == -1) {
		fclose(fw);
		return SR_ERR;
	}

	filesize = (uint64_t)f_stat.st_size;

	if ((buf = malloc(filesize)) == NULL) {
		sr_err("FPGA configure buf malloc failed.");
		fclose(fw);
		return SR_ERR;
	}

	/* step0: assert PROG_B low */
	wr_cmd.header.dest = DSL_CTL_PROG_B;
	wr_cmd.header.offset = 0;
	wr_cmd.header.size = 1;
	wr_cmd.data[0] = (uint8_t)~bmWR_PROG_B;
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
		fclose(fw);
		free(buf);
		return SR_ERR;
	}

	/* step1: turn off GREEN/RED led */
	wr_cmd.header.dest = DSL_CTL_LED;
	wr_cmd.header.offset = 0;
	wr_cmd.header.size = 1;
	wr_cmd.data[0] = (uint8_t)(~bmLED_GREEN & ~bmLED_RED);
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
		fclose(fw);
		free(buf);
		return SR_ERR;
	}

	/* step2: assert PROG_B high */
	wr_cmd.header.dest = DSL_CTL_PROG_B;
	wr_cmd.header.offset = 0;
	wr_cmd.header.size = 1;
	wr_cmd.data[0] = bmWR_PROG_B;
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
		fclose(fw);
		free(buf);
		return SR_ERR;
	}

	/* step3: wait INIT_B go high */
	rd_cmd.header.dest = DSL_CTL_HW_STATUS;
	rd_cmd.header.offset = 0;
	rd_cmd.header.size = 1;
	rd_cmd_data = 0;
	rd_cmd.data = &rd_cmd_data;
	while (1) {
		if ((ret = command_ctl_rd(hdl, rd_cmd)) != SR_OK) {
			fclose(fw);
			free(buf);
			return SR_ERR;
		}
		if (rd_cmd_data & bmFPGA_INIT_B)
			break;
	}

	/* step4: send config ctl command */
	wr_cmd.header.dest = DSL_CTL_INTRDY;
	wr_cmd.header.offset = 0;
	wr_cmd.header.size = 1;
	wr_cmd.data[0] = (uint8_t)~bmWR_INTRDY;
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
		fclose(fw);
		free(buf);
		return SR_ERR;
	}

	wr_cmd.header.dest = DSL_CTL_BULK_WR;
	wr_cmd.header.offset = 0;
	wr_cmd.header.size = 3;
	wr_cmd.data[0] = (uint8_t)filesize;
	wr_cmd.data[1] = (uint8_t)(filesize >> 8);
	wr_cmd.data[2] = (uint8_t)(filesize >> 16);
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
		sr_err("Configure FPGA error: send command fpga_config failed.");
		fclose(fw);
		free(buf);
		return SR_ERR;
	}

	/* step5: send config data */
	chunksize = (int)fread(buf, 1, filesize, fw);
	fclose(fw);

	if (chunksize <= 0) {
		free(buf);
		return SR_ERR;
	}

	ret = libusb_bulk_transfer(hdl, 2 | LIBUSB_ENDPOINT_OUT,
	                           buf, chunksize,
	                           &transferred, 1000);
	free(buf);

	if (ret < 0) {
		sr_err("Unable to configure FPGA of dsl device: %s.",
		        libusb_error_name(ret));
		return SR_ERR;
	} else if (transferred != chunksize) {
		sr_err("Configure FPGA error: expected transfer size %d; actually %d.",
		        chunksize, transferred);
		return SR_ERR;
	}

	/* step6: assert INTRDY high */
	wr_cmd.header.dest = DSL_CTL_INTRDY;
	wr_cmd.header.offset = 0;
	wr_cmd.header.size = 1;
	wr_cmd.data[0] = bmWR_INTRDY;
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK)
		return SR_ERR;

	/* step7: check GPIF_DONE */
	rd_cmd.header.dest = DSL_CTL_HW_STATUS;
	rd_cmd.header.offset = 0;
	rd_cmd.header.size = 1;
	rd_cmd_data = 0;
	rd_cmd.data = &rd_cmd_data;
	while ((ret = command_ctl_rd(hdl, rd_cmd)) == SR_OK) {
		if (rd_cmd_data & bmGPIF_DONE)
			break;
	}

	/* step8: assert INTRDY low */
	wr_cmd.header.dest = DSL_CTL_INTRDY;
	wr_cmd.header.offset = 0;
	wr_cmd.header.size = 1;
	wr_cmd.data[0] = (uint8_t)~bmWR_INTRDY;
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK)
		return SR_ERR;

	/* step9: check FPGA_DONE bit */
	rd_cmd.header.dest = DSL_CTL_HW_STATUS;
	rd_cmd.header.offset = 0;
	rd_cmd.header.size = 1;
	rd_cmd_data = 0;
	rd_cmd.data = &rd_cmd_data;
	while ((ret = command_ctl_rd(hdl, rd_cmd)) == SR_OK) {
		if (rd_cmd_data & bmFPGA_DONE) {
			/* step10: turn on GREEN led */
			wr_cmd.header.dest = DSL_CTL_LED;
			wr_cmd.header.offset = 0;
			wr_cmd.header.size = 1;
			wr_cmd.data[0] = bmLED_GREEN;
			if ((ret = command_ctl_wr(hdl, wr_cmd)) == SR_OK)
				break;
		}
	}

	/* recover GPIF to be wordwide */
	wr_cmd.header.dest = DSL_CTL_WORDWIDE;
	wr_cmd.header.offset = 0;
	wr_cmd.header.size = 1;
	wr_cmd.data[0] = bmWR_WORDWIDE;
	if ((ret = command_ctl_wr(hdl, wr_cmd)) != SR_OK) {
		sr_err("Sent DSL_CTL_WORDWIDE command failed.");
		return SR_ERR;
	}

	sr_info("FPGA configure done: %d bytes.", chunksize);
	return SR_OK;
}

/* ===========================================================================
 * dsl_* channel/probe/samplerate management functions
 * =========================================================================== */

static const char *const dsl_probe_names[] = {
	"0",  "1",  "2",  "3",  "4",  "5",  "6",  "7",
	"8",  "9",  "10", "11", "12", "13", "14", "15",
	"16", "17", "18", "19", "20", "21", "22", "23",
	"24", "25", "26", "27", "28", "29", "30", "31",
	NULL,
};

static void dsl_probe_init(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	GSList *l;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *probe = (struct sr_channel *)l->data;
		struct dsl_channel_priv *p = DSL_CH_PRIV(probe);
		p->bits = channel_modes[devc->ch_mode].unit_bits;
		p->vdiv = (devc->profile->dev_caps.vdivs) ? devc->profile->dev_caps.vdivs[0] : 1000;
		p->vfactor = 1;
		p->cali_fgain0 = 1;
		p->cali_fgain1 = 1;
		p->cali_fgain2 = 1;
		p->cali_fgain3 = 1;
		p->cali_comb_fgain0 = 1;
		p->cali_comb_fgain1 = 1;
		p->cali_comb_fgain2 = 1;
		p->cali_comb_fgain3 = 1;
		p->offset = (1 << (p->bits - 1));
		p->coupling = 1; /* SR_DC_COUPLING */
		p->trig_value = (1 << (p->bits - 1));
		p->vpos_trans = devc->profile->dev_caps.default_pwmtrans;
		p->comb_comp = (int8_t)devc->profile->dev_caps.default_comb_comp;
		p->digi_fgain = 0;
		p->map_default = TRUE;
		p->map_unit = 0;
		p->map_min = -(p->vdiv * p->vfactor * 8 / 2000.0);
		p->map_max = p->vdiv * p->vfactor * 8 / 2000.0;
		p->vga_ptr = NULL;
	}
}

SR_PRIV int dsl_setup_probes(struct sr_dev_inst *sdi, int num_probes)
{
	uint16_t j;
	struct sr_channel *probe;
	struct dev_context *devc = sdi->priv;
	struct dsl_channel_priv *ch_priv;

	for (j = 0; j < num_probes; j++) {
		probe = sr_channel_new(sdi, j, channel_modes[devc->ch_mode].type,
		                       TRUE, dsl_probe_names[j]);
		if (!probe)
			return SR_ERR;
		ch_priv = g_malloc0(sizeof(struct dsl_channel_priv));
		probe->priv = ch_priv;
		sdi->channels = g_slist_append(sdi->channels, probe);
	}
	dsl_probe_init(sdi);
	return SR_OK;
}

SR_PRIV int dsl_adjust_probes(struct sr_dev_inst *sdi, int num_probes)
{
	uint16_t j;
	struct sr_channel *probe;
	struct dev_context *devc = sdi->priv;
	GSList *l;

	assert(num_probes > 0);

	j = (uint16_t)g_slist_length(sdi->channels);
	while (j < num_probes) {
		probe = sr_channel_new(sdi, j, channel_modes[devc->ch_mode].type,
		                       TRUE, dsl_probe_names[j]);
		if (!probe)
			return SR_ERR;
		/* Allocate channel private data for new channels. */
		probe->priv = g_malloc0(sizeof(struct dsl_channel_priv));
		sdi->channels = g_slist_append(sdi->channels, probe);
		j++;
	}

	while (j > num_probes) {
		GSList *last = g_slist_last(sdi->channels);
		struct sr_channel *ch = last->data;
		if (ch && ch->priv) {
			g_free(ch->priv);
			ch->priv = NULL;
		}
		sdi->channels = g_slist_delete_link(sdi->channels, last);
		j--;
	}

	for (l = sdi->channels; l; l = l->next) {
		probe = (struct sr_channel *)l->data;
		probe->enabled = TRUE;
		probe->type = channel_modes[devc->ch_mode].type;
	}

	return SR_OK;
}

SR_PRIV const GSList *dsl_mode_list(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	GSList *l = NULL;
	unsigned int i;

	devc = sdi->priv;
	for (i = 0; i < ARRAY_SIZE(channel_modes); i++) {
		if (devc->profile->dev_caps.channels & (1ULL << i)) {
			l = g_slist_append(l, (gpointer)&channel_modes[i]);
		}
	}

	return l;
}

SR_PRIV void dsl_adjust_samplerate(struct dev_context *devc)
{
	int i;
	for (i = 0; devc->profile->dev_caps.samplerates[i]; i++) {
		if (devc->profile->dev_caps.samplerates[i] >
				channel_modes[devc->ch_mode].max_samplerate)
			break;
	}
	devc->samplerates_max_index = (uint16_t)(i - 1);

	for (i = 0; devc->profile->dev_caps.samplerates[i]; i++) {
		if (devc->profile->dev_caps.samplerates[i] >=
				channel_modes[devc->ch_mode].min_samplerate)
			break;
	}
	devc->samplerates_min_index = (uint16_t)i;

	if (devc->samplerates_max_index < devc->samplerates_min_index) {
		sr_err("samplerates_max_index < samplerates_min_index");
		return;
	}

	if (devc->cur_samplerate > devc->profile->dev_caps.samplerates[devc->samplerates_max_index])
		devc->cur_samplerate = devc->profile->dev_caps.samplerates[devc->samplerates_max_index];

	if (devc->cur_samplerate < devc->profile->dev_caps.samplerates[devc->samplerates_min_index])
		devc->cur_samplerate = devc->profile->dev_caps.samplerates[devc->samplerates_min_index];
}

SR_PRIV int dsl_en_ch_num(const struct sr_dev_inst *sdi)
{
	GSList *l;
	int channel_en_cnt = 0;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *probe = (struct sr_channel *)l->data;
		channel_en_cnt += probe->enabled;
	}
	channel_en_cnt += (channel_en_cnt == 0);

	return channel_en_cnt;
}

SR_PRIV gboolean dsl_check_conf_profile(libusb_device *dev)
{
	struct libusb_device_descriptor des;
	struct libusb_device_handle *hdl;
	int ret;
	unsigned char strdesc[64];

	hdl = NULL;

	if ((ret = libusb_get_device_descriptor(dev, &des)) < 0) {
		sr_err("Failed to get device descriptor: %s", libusb_error_name(ret));
		return FALSE;
	}

	if ((ret = libusb_open(dev, &hdl)) < 0) {
		sr_err("Failed to open device: %s", libusb_error_name(ret));
		/* Maybe the device is busy; assume it matches. */
		return TRUE;
	}

	if ((ret = libusb_get_string_descriptor_ascii(hdl,
			des.iManufacturer, strdesc, sizeof(strdesc))) < 0) {
		sr_err("Failed to get manufacturer descriptor: %s", libusb_error_name(ret));
		libusb_close(hdl);
		return FALSE;
	}

	if (strncmp((const char *)strdesc, DSL_MANUF_STRING,
			strlen(DSL_MANUF_STRING))) {
		libusb_close(hdl);
		return FALSE;
	}

	if ((ret = libusb_get_string_descriptor_ascii(hdl,
			des.iProduct, strdesc, sizeof(strdesc))) < 0) {
		sr_err("Failed to get product descriptor: %s", libusb_error_name(ret));
		libusb_close(hdl);
		return FALSE;
	}

	if (strncmp((const char *)strdesc, DSL_PROD_STRING_DSVIEW,
			strlen(DSL_PROD_STRING_DSVIEW))) {
		libusb_close(hdl);
		return FALSE;
	}

	libusb_close(hdl);
	return TRUE;
}

SR_PRIV int dsl_configure_probes(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	int i;

	devc = sdi->priv;
	for (i = 0; i < NUM_TRIGGER_STAGES; i++) {
		devc->trigger_mask[i] = 0;
		devc->trigger_value[i] = 0;
	}
	devc->trigger_stage = 0;

	/* The detailed trigger parsing is performed in set_trigger()/dsl_fpga_arm()
	 * using the upstream sr_session_trigger_get() API. This stub is retained
	 * for API compatibility. */

	return SR_OK;
}

SR_PRIV uint64_t dsl_channel_depth(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	int ch_num = dsl_en_ch_num(sdi);
	return (devc->profile->dev_caps.hw_depth / (ch_num ? ch_num : 1)) & ~(uint64_t)SAMPLES_ALIGN;
}

/* ===========================================================================
 * dsl_config_get/set/list — stubs (api.c handles config dispatch)
 * =========================================================================== */

SR_PRIV int dsl_config_get(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi,
			const struct sr_channel_group *cg)
{
	(void)key;
	(void)data;
	(void)sdi;
	(void)cg;
	return SR_ERR_NA;
}

SR_PRIV int dsl_config_set(uint32_t key, GVariant *data, struct sr_dev_inst *sdi,
			const struct sr_channel_group *cg)
{
	(void)key;
	(void)data;
	(void)sdi;
	(void)cg;
	return SR_ERR_NA;
}

SR_PRIV int dsl_config_list(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi,
			const struct sr_channel_group *cg)
{
	(void)key;
	(void)data;
	(void)sdi;
	(void)cg;
	return SR_ERR_NA;
}

/* ===========================================================================
 * dsl_dev_open / dsl_dev_close / dsl_dev_acquisition_stop / status
 * =========================================================================== */

SR_PRIV int dsl_dev_open(struct sr_dev_driver *di, struct sr_dev_inst *sdi, gboolean *fpga_done)
{
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;
	struct sr_dev_driver *driver;
	int ret;
	uint8_t hw_info;
	struct ctl_rd_cmd rd_cmd;

	devc = sdi->priv;
	usb = sdi->conn;
	(void)di;

	if (!usb) {
		sr_err("dsl_dev_open: usb is null.");
		return SR_ERR;
	}

	driver = sdi->driver;
	ret = dslogic_dev_open(sdi, driver);
	if (ret != SR_OK) {
		sr_err("dsl_dev_open: Unable to open device.");
		return SR_ERR;
	}

	ret = libusb_claim_interface(usb->devhdl, USB_INTERFACE);
	if (ret != LIBUSB_SUCCESS) {
		sr_err("Unable to claim interface: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	/* Read HW status to determine if FPGA is configured. */
	rd_cmd.header.dest = DSL_CTL_HW_STATUS;
	rd_cmd.header.offset = 0;
	rd_cmd.header.size = 1;
	hw_info = 0;
	rd_cmd.data = &hw_info;
	if ((ret = command_ctl_rd(usb->devhdl, rd_cmd)) != SR_OK) {
		sr_err("Failed to get hardware information.");
		return SR_ERR;
	}
	if (fpga_done)
		*fpga_done = (hw_info & bmFPGA_DONE) != 0;

	/* Upload FPGA bitstream if not configured. */
	if (fpga_done && !(*fpga_done)) {
		ret = dslogic_fpga_firmware_upload(sdi);
		if (ret != SR_OK) {
			sr_err("Configure FPGA failed.");
			return SR_ERR;
		}
	}

	/* Security check for CAPS_FEATURE_SECURITY devices. */
	if (devc->profile->dev_caps.feature_caps & CAPS_FEATURE_SECURITY) {
		uint16_t encryption[SECU_STEPS];
		dsl_wr_reg(sdi, CTR0_ADDR, bmNONE);
		if (dsl_rd_nvm(sdi, (unsigned char *)encryption, SECU_EEP_ADDR, SECU_STEPS * 2) != SR_OK) {
			sr_err("Read EEPROM content failed.");
			return SR_ERR;
		}
		ret = dsl_secuCheck(sdi, encryption, SECU_STEPS);
		if (ret != SR_OK)
			sr_err("Security check failed!");
		else
			sr_info("Security check pass!");
	}

	return SR_OK;
}

SR_PRIV int dsl_dev_close(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;

	if (!sdi)
		return SR_ERR;

	usb = sdi->conn;
	if (!usb) {
		return SR_OK;
	}

	if (!usb->devhdl) {
		return SR_ERR;
	}

	sr_info("Closing device on %d.%d interface %d.",
		usb->bus, usb->address, USB_INTERFACE);

	libusb_release_interface(usb->devhdl, USB_INTERFACE);
	libusb_close(usb->devhdl);
	usb->devhdl = NULL;

	return SR_OK;
}

SR_PRIV int dsl_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
	(void)cb_data;

	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	int ret;
	struct ctl_wr_cmd wr_cmd;

	devc = sdi->priv;
	usb = sdi->conn;

	if (!devc->abort) {
		devc->abort = TRUE;
		dsl_wr_reg(sdi, CTR0_ADDR, bmFORCE_RDY);
		sr_info("Send command:\"bmFORCE_RDY\"");
	} else if (devc->status == DSL_FINISH) {
		/* Stop GPIF acquisition */
		wr_cmd.header.dest = DSL_CTL_STOP;
		wr_cmd.header.offset = 0;
		wr_cmd.header.size = 0;
		if ((ret = command_ctl_wr(usb->devhdl, wr_cmd)) != SR_OK)
			sr_err("Sent acquisition stop command failed!");
		else
			sr_info("Sent acquisition stop command!");

		/* Check FPGA status */
		uint8_t hw_status = 1;
		dsl_rd_reg(sdi, HW_STATUS_ADDR, &hw_status);

		/* ADC power down */
		if (devc->profile->dev_caps.feature_caps & CAPS_FEATURE_HMCAD1511) {
			dsl_config_adc(sdi, adc_power_down);
		}
	}

	return SR_OK;
}

SR_PRIV int dsl_dev_status_get(const struct sr_dev_inst *sdi, struct dsl_status *status, gboolean prg)
{
	int ret = SR_ERR;

	if (sdi) {
		struct dev_context *devc;
		devc = sdi->priv;
		if (prg || devc->mstatus_valid) {
			if (status)
				*status = devc->mstatus;
			ret = SR_OK;
		}
	}

	return ret;
}

SR_PRIV int dsl_header_size(const struct dev_context *devc)
{
	int size;

	if (devc->profile->dev_caps.feature_caps & CAPS_FEATURE_USB30)
		size = SR_KB(1);
	else
		size = 512;
	return size;
}

SR_PRIV unsigned int dsl_get_timeout(const struct sr_dev_inst *sdi)
{
	return get_timeout(sdi);
}

SR_PRIV int dsl_start_transfers(const struct sr_dev_inst *sdi)
{
	/* Delegates to start_transfers() which submits data transfers with
	 * receive_transfer() as the callback. receive_transfer() dispatches
	 * by devc->mode (LOGIC/DSO/ANALOG), so this works for all modes. */
	return start_transfers(sdi);
}

SR_PRIV int dsl_destroy_device(struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *driver;

	if (!sdi)
		return SR_ERR_ARG;

	driver = sdi->driver;
	if (driver && driver->dev_close)
		driver->dev_close(sdi);

	if (sdi->conn)
		sr_usb_dev_inst_free(sdi->conn);

	if (sdi->priv) {
		g_free(sdi->priv);
		sdi->priv = NULL;
	}

	sr_dev_inst_free(sdi);

	return SR_OK;
}

/* ===========================================================================
 * dsl_secuCheck — security check for CAPS_FEATURE_SECURITY devices
 * =========================================================================== */

static int dsl_secu_reset(const struct sr_dev_inst *sdi)
{
	if (dsl_wr_reg(sdi, SEC_CTRL_ADDR, 0) != SR_OK) goto Err;
	if (dsl_wr_reg(sdi, SEC_CTRL_ADDR + 1, 0) != SR_OK) goto Err;
	g_usleep(10 * 1000);
	if (dsl_wr_reg(sdi, SEC_CTRL_ADDR, 1) != SR_OK) goto Err;
	if (dsl_wr_reg(sdi, SEC_CTRL_ADDR + 1, 0) != SR_OK) goto Err;
	return SR_OK;
Err:
	sr_err("dsl_secu_reset: dsl_wr_reg(SEC_XXX_ADDR) failed.");
	return SR_ERR;
}

static int dsl_secu_write(const struct sr_dev_inst *sdi, uint16_t cmd, uint16_t din)
{
	if (dsl_wr_reg(sdi, SEC_DATA_ADDR, (uint8_t)din) != SR_OK) goto Err;
	if (dsl_wr_reg(sdi, SEC_DATA_ADDR + 1, (uint8_t)(din >> 8)) != SR_OK) goto Err;
	if (dsl_wr_reg(sdi, SEC_CTRL_ADDR, (uint8_t)cmd) != SR_OK) goto Err;
	if (dsl_wr_reg(sdi, SEC_CTRL_ADDR + 1, (uint8_t)(cmd >> 8)) != SR_OK) goto Err;
	return SR_OK;
Err:
	sr_err("dsl_secu_write: dsl_wr_reg(SEC_XXX_ADDR) failed.");
	return SR_ERR;
}

static gboolean dsl_is_secu_ready(const struct sr_dev_inst *sdi)
{
	uint8_t temp;
	if (dsl_rd_reg(sdi, SEC_CTRL_ADDR, &temp) != SR_OK) {
		sr_err("dsl_is_secu_ready: dsl_rd_reg failed.");
		return FALSE;
	}
	return (temp & bmSECU_READY) ? TRUE : FALSE;
}

static gboolean dsl_is_secu_pass(const struct sr_dev_inst *sdi)
{
	uint8_t temp;
	if (dsl_rd_reg(sdi, SEC_CTRL_ADDR, &temp) != SR_OK) {
		sr_err("dsl_is_secu_pass: dsl_rd_reg failed.");
		return FALSE;
	}
	return (temp & bmSECU_PASS) ? TRUE : FALSE;
}

static uint16_t dsl_secu_read(const struct sr_dev_inst *sdi)
{
	uint16_t sec = 0;
	uint8_t low = 0, high = 0;
	if (dsl_rd_reg(sdi, SEC_DATA_ADDR + 1, &high) != SR_OK) goto Err;
	if (dsl_rd_reg(sdi, SEC_DATA_ADDR, &low) != SR_OK) goto Err;
	sec = (uint16_t)((high << 8) | low);
	return sec;
Err:
	sr_err("dsl_secu_read: dsl_rd_reg failed.");
	return 0;
}

SR_PRIV int dsl_secuCheck(const struct sr_dev_inst *sdi, uint16_t *encryption, int steps)
{
	int tryCnt;

	if (!encryption)
		return SR_ERR_ARG;

	tryCnt = SECU_TRY_CNT;

	dsl_secu_reset(sdi);
	if (dsl_is_secu_pass(sdi))
		return SR_ERR;
	dsl_secu_write(sdi, SECU_START, 0);
	while (steps--) {
		if (dsl_is_secu_pass(sdi))
			return SR_ERR;
		while (!dsl_is_secu_ready(sdi)) {
			if (tryCnt-- == 0) {
				sr_err("Get security ready failed.");
				return SR_ERR;
			}
		}
		if (dsl_secu_read(sdi) != 0)
			return SR_ERR;
		dsl_secu_write(sdi, SECU_CHECK, encryption[steps]);
	}

	return SR_OK;
}
