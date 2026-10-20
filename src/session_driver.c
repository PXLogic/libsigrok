/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <zip.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "virtual-session"

/* size of payloads sent across the session bus */
/** @cond PRIVATE */
#define CHUNKSIZE (4 * 1024 * 1024)
/** @endcond */

SR_PRIV struct sr_dev_driver session_driver_info;

struct session_vdev {
	char *sessionfile;
	char *capturefile;
	struct zip *archive;
	struct zip_file *capfile;
	int bytes_read;
	uint64_t samplerate;
	uint64_t total_samples;
	int unitsize;
	int num_logic_channels;
	int num_analog_channels;
	int cur_analog_channel;
	GArray *analog_channels;
	int cur_chunk;
	gboolean finished;
	/* PXView v3 format: per-channel chunked data (L-<ch>/<block>) */
	gboolean pxv_format;
	int pxv_cur_block;
	/* MSO 架构修复：analog 数据流式读取支持。
	 * pxv_phase: 0=logic, 1=analog (logic 块读完后切换到 analog) */
	int pxv_phase;
	int pxv_cur_analog_block;
	/* analog 数据格式：从 header 的 "analog bytes"/"analog float" 键解析 */
	int analog_unit_bytes;
	gboolean analog_is_float;
	/* PXView device mode (LOGIC=0, DSO=1, ANALOG=2, MSO=3).
	 * Parsed from header "device mode" key by session_file.c.
	 * Exposed via SR_CONF_DEVICE_MODE so PXView can restore the
	 * correct work mode when opening a .pxl file. */
	int pxv_device_mode;
	/* Device-level settings parsed from header by session_file.c.
	 * These are written by StoreSession::meta_gen() and must be
	 * restored on load so the frontend can display correct trigger
	 * position, DSO timebase, reference ranges, etc. */
	uint64_t trig_pos;       /* "trigger pos" — sample offset of trigger */
	uint64_t timebase;       /* "hDiv" — DSO horizontal timebase */
	uint8_t unit_bits;       /* "bits" — ADC resolution (DSO/ANALOG) */
	uint32_t ref_min;        /* "ref min" — ADC reference minimum */
	uint32_t ref_max;        /* "ref max" — ADC reference maximum */
	uint64_t num_blocks;     /* "total blocks" — data block count */
	int64_t session_time;    /* "trigger time" — ms since epoch (capture timestamp) */
};

static const uint32_t devopts[] = {
	SR_CONF_CAPTUREFILE | SR_CONF_SET,
	SR_CONF_CAPTURE_UNITSIZE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_NUM_LOGIC_CHANNELS | SR_CONF_SET,
	SR_CONF_NUM_ANALOG_CHANNELS | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SESSIONFILE | SR_CONF_SET,
	SR_CONF_DEVICE_MODE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_POS | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TIMEBASE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_UNIT_BITS | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_REF_MIN | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_REF_MAX | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_NUM_BLOCKS | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SESSION_TIME | SR_CONF_GET | SR_CONF_SET,
};

/*
 * Stream PXView v3 logic data: per-channel chunked (L-<ch>/<block>).
 * Each chunk contains a single channel's bitmap (1 bit per sample, LSB-first
 * within each byte). This function reads all channels' chunks for a given
 * block number, interleaves them into upstream unitsize-packed format, and
 * sends SR_DF_LOGIC packets.
 */
static gboolean stream_pxv_logic_data(struct sr_dev_inst *sdi)
{
	struct session_vdev *vdev;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	struct zip_stat zs;
	char chunkname[32];
	struct zip_file *zf;
	struct sr_channel *ch_struct;
	GSList *ch_list;
	int ch_count, ch_ordinal;
	int unitsize;
	uint8_t **channel_bufs;
	int *channel_indices;
	int block_size = 0;
	int samples_in_block;
	uint8_t *out_buf;
	int out_size;
	int got_data = FALSE;

	vdev = sdi->priv;
	unitsize = vdev->unitsize;

	if (unitsize <= 0)
		return FALSE;

	/* Count enabled logic channels and collect their actual indices */
	ch_count = 0;
	for (ch_list = sdi->channels; ch_list; ch_list = ch_list->next) {
		ch_struct = ch_list->data;
		if (ch_struct->type == SR_CHANNEL_LOGIC && ch_struct->enabled)
			ch_count++;
	}
	/* FIX: the v3 per-channel chunked format reconstructs a sample-interleaved
	 * SPLIT stream whose unitsize must equal the number of logic channels'
	 * byte width, NOT the hardware driver's unitsize recorded in the file
	 * (which can be larger, e.g. 2 for 8 channels). Using the wrong unitsize
	 * shifts every byte and corrupts the save→load→save round-trip. */
	int calc_unitsize = (ch_count + 7) / 8;
	if (calc_unitsize > 0)
		unitsize = calc_unitsize;
	if (ch_count <= 0)
		return FALSE;

	/* Check if the first enabled channel has a chunk for current block */
	ch_ordinal = 0;
	for (ch_list = sdi->channels; ch_list; ch_list = ch_list->next) {
		ch_struct = ch_list->data;
		if (ch_struct->type == SR_CHANNEL_LOGIC && ch_struct->enabled) {
			snprintf(chunkname, sizeof(chunkname), "L-%d/%d",
					ch_struct->index, vdev->pxv_cur_block);
			if (zip_stat(vdev->archive, chunkname, 0, &zs) < 0)
				return FALSE;
			block_size = zs.size;
			break;
		}
	}

	/* Allocate arrays for channel data and indices */
	channel_bufs = g_malloc0(sizeof(uint8_t *) * ch_count);
	channel_indices = g_malloc0(sizeof(int) * ch_count);

	/* Read all enabled logic channels' data for this block */
	ch_ordinal = 0;
	for (ch_list = sdi->channels; ch_list; ch_list = ch_list->next) {
		ch_struct = ch_list->data;
		if (ch_struct->type != SR_CHANNEL_LOGIC || !ch_struct->enabled)
			continue;
		int ch_idx = ch_struct->index;
		channel_indices[ch_ordinal] = ch_idx;
		snprintf(chunkname, sizeof(chunkname), "L-%d/%d",
				ch_idx, vdev->pxv_cur_block);
		if (zip_stat(vdev->archive, chunkname, 0, &zs) >= 0) {
			zf = zip_fopen(vdev->archive, chunkname, 0);
			if (zf) {
				channel_bufs[ch_ordinal] = g_malloc(zs.size);
				if (zip_fread(zf, channel_bufs[ch_ordinal], zs.size)
						== (int)zs.size) {
					if (block_size == 0)
						block_size = zs.size;
				} else {
					sr_warn("Short read on %s.", chunkname);
				}
				zip_fclose(zf);
			}
		}
		ch_ordinal++;
	}

	if (block_size == 0) {
		g_free(channel_bufs);
		g_free(channel_indices);
		vdev->pxv_cur_block++;
		return TRUE;
	}

	/* 性能飞跃改造：改用 PXLogic 原生 LA_CROSS_DATA 通道直通格式！
	 * PXL 内部是以 L-<ch>/<block> 单通道位图存储的 (每个字节 8 个样本，LSB-first)。
	 * 硬件 DMA/CROSS 规范要求：以 64 个 sample (8 字节) 为一组，
	 * 布局为: [ch0 8B][ch1 8B]...[chN-1 8B][ch0 下个8B]...
	 * 之前在此处进行逐 sample (s=0..samples) 的逐 bit 循环重排交织，需要数十亿次移位指令；
	 * 改用 uint64_t 块搬移拼成 LA_CROSS_DATA 后，LogicSnapshot::append_cross_payload
	 * 可以直接走 64-bit blocked-transpose 纯内存写入，彻底避开两端的位转置瓶颈！
	 */
	samples_in_block = block_size * 8;
	const int num_groups = block_size / 8; // 每组 64 samples (8 字节)
	out_size = num_groups * ch_count * 8;
	out_buf = g_malloc0(out_size);

	uint64_t *dst_u64 = (uint64_t *)out_buf;
	for (int g = 0; g < num_groups; g++) {
		for (ch_ordinal = 0; ch_ordinal < ch_count; ch_ordinal++) {
			if (channel_bufs[ch_ordinal]) {
				const uint64_t *src_u64 = (const uint64_t *)(channel_bufs[ch_ordinal] + g * 8);
				*dst_u64++ = *src_u64;
			} else {
				*dst_u64++ = 0;
			}
		}
	}

	got_data = TRUE;
	packet.type = SR_DF_LOGIC;
	packet.payload = &logic;
	logic.length = out_size;
	logic.unitsize = 1;
	logic.format = LA_CROSS_DATA;
	logic.data = out_buf;
	vdev->bytes_read += out_size;
	sr_session_send(sdi, &packet);

	g_free(out_buf);
	for (ch_ordinal = 0; ch_ordinal < ch_count; ch_ordinal++)
		g_free(channel_bufs[ch_ordinal]);
	g_free(channel_bufs);
	g_free(channel_indices);

	vdev->pxv_cur_block++;
	return got_data;
}

/*
 * MSO 架构修复：Stream PXView v3 analog data: interleaved multi-channel
 * chunks (A-0/<block>). 每个块包含所有 enabled analog 通道的 interleaved
 * 数据：[s0_ch0][s0_ch1]...[s1_ch0][s1_ch1]...
 * 读取后构造 SR_DF_ANALOG 包，包含所有 analog 通道。
 */
static gboolean stream_pxv_analog_data(struct sr_dev_inst *sdi)
{
	struct session_vdev *vdev;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct zip_stat zs;
	char chunkname[32];
	struct zip_file *zf;
	struct sr_channel *ch_struct;
	GSList *ch_list;
	GSList *analog_ch_list = NULL;
	int analog_count = 0;
	int unit_bytes;
	uint8_t *buf;

	vdev = sdi->priv;

	/* 无 analog 通道或未配置格式 → 结束 */
	if (vdev->num_analog_channels <= 0)
		return FALSE;

	unit_bytes = vdev->analog_unit_bytes;
	if (unit_bytes <= 0) {
		/* 默认 float (4 bytes)，匹配上游 sigrok 约定 */
		unit_bytes = 4;
		sr_warn("analog_unit_bytes not set, assuming float (4 bytes).");
	}

	/* 收集所有 enabled analog 通道 */
	for (ch_list = sdi->channels; ch_list; ch_list = ch_list->next) {
		ch_struct = ch_list->data;
		if (ch_struct->type == SR_CHANNEL_ANALOG && ch_struct->enabled) {
			analog_ch_list = g_slist_append(analog_ch_list, ch_struct);
			analog_count++;
		}
	}
	if (analog_count <= 0) {
		return FALSE;
	}

	/* 读取 A-0/<block> 块（interleaved 多通道数据） */
	snprintf(chunkname, sizeof(chunkname), "A-0/%d",
			vdev->pxv_cur_analog_block);
	if (zip_stat(vdev->archive, chunkname, 0, &zs) < 0)
		return FALSE;  /* 无更多 analog 块 → 完成 */

	buf = g_malloc(zs.size);
	zf = zip_fopen(vdev->archive, chunkname, 0);
	if (!zf) {
		g_free(buf);
		g_slist_free(analog_ch_list);
		return FALSE;
	}
	if (zip_fread(zf, buf, zs.size) != (int)zs.size) {
		sr_warn("Short read on %s.", chunkname);
		g_free(buf);
		g_slist_free(analog_ch_list);
		zip_fclose(zf);
		vdev->pxv_cur_analog_block++;
		return TRUE;
	}
	zip_fclose(zf);

	/* 构造 SR_DF_ANALOG 包 */
	int bytes_per_sample = unit_bytes * analog_count;
	uint64_t num_samples = zs.size / bytes_per_sample;

	packet.type = SR_DF_ANALOG;
	packet.payload = &analog;
	sr_analog_init(&analog, &encoding, &meaning, &spec, 2);
	/* sr_analog_init 默认 unitsize=sizeof(float), is_float=FALSE。
	 * 需根据实际数据格式设置，否则 AnalogSnapshot 会错误解读数据。 */
	encoding.unitsize = unit_bytes;
	encoding.is_float = vdev->analog_is_float;
	encoding.is_signed = TRUE;
	encoding.is_bigendian = FALSE;
	analog.meaning->channels = analog_ch_list;
	analog.num_samples = num_samples;
	analog.meaning->mq = SR_MQ_VOLTAGE;
	analog.meaning->unit = SR_UNIT_VOLT;
	analog.meaning->mqflags = SR_MQFLAG_DC;
	analog.data = buf;

	vdev->bytes_read += zs.size;
	sr_session_send(sdi, &packet);

	/* analog_ch_list 被 sr_session_send 消费后释放（归 sr_datafeed_analog 所有权） */
	/* buf 归 sr_datafeed_analog 所有权，由接收方释放 */
	/* 但实际 libsigrok 的 packet 发送是同步拷贝，需自行释放 */
	g_free(buf);
	g_slist_free(analog_ch_list);

	vdev->pxv_cur_analog_block++;
	return TRUE;
}

/*
 * Stream PXView v3 DSO data: per-channel chunks (O-<ch>/0).
 * Each chunk contains one channel's raw sample bytes (1 byte per sample).
 * We read all enabled DSO channels' chunks, interleave them into
 * sr_datafeed_dso format, and send SR_DF_DSO packets.
 */
static gboolean stream_pxv_dso_data(struct sr_dev_inst *sdi)
{
	struct session_vdev *vdev;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_dso dso;
	struct zip_stat zs;
	char chunkname[32];
	struct zip_file *zf;
	struct sr_channel *ch_struct;
	GSList *ch_list;
	GSList *dso_ch_list = NULL;
	int dso_count = 0;
	int unit_bytes = 1; /* DSO samples are 1 byte each */
	uint8_t *buf = NULL;
	uint8_t *interleaved = NULL;
	uint64_t total_samples = 0;
	int ch_idx;

	vdev = sdi->priv;

	/* Collect all enabled DSO channels */
	for (ch_list = sdi->channels; ch_list; ch_list = ch_list->next) {
		ch_struct = ch_list->data;
		if (ch_struct->type == SR_CHANNEL_DSO && ch_struct->enabled) {
			dso_ch_list = g_slist_append(dso_ch_list, ch_struct);
			dso_count++;
		}
	}
	if (dso_count <= 0) {
		g_slist_free(dso_ch_list);
		return FALSE;
	}

	/* Read first channel's chunk to determine total sample count */
	ch_struct = dso_ch_list->data;
	ch_idx = ch_struct->index;
	snprintf(chunkname, sizeof(chunkname), "O-%d/0", ch_idx);
	if (zip_stat(vdev->archive, chunkname, 0, &zs) < 0) {
		g_slist_free(dso_ch_list);
		return FALSE;  /* No DSO data */
	}
	total_samples = zs.size / unit_bytes;

	if (total_samples == 0) {
		g_slist_free(dso_ch_list);
		return FALSE;
	}

	/* Allocate interleaved buffer: total_samples * dso_count bytes */
	interleaved = g_malloc0(total_samples * dso_count);
	if (!interleaved) {
		g_slist_free(dso_ch_list);
		return FALSE;
	}

	/* Read each channel's data and interleave */
	int ch_ordinal = 0;
	for (GSList *l = dso_ch_list; l; l = l->next) {
		ch_struct = l->data;
		ch_idx = ch_struct->index;
		snprintf(chunkname, sizeof(chunkname), "O-%d/0", ch_idx);
		if (zip_stat(vdev->archive, chunkname, 0, &zs) >= 0) {
			zf = zip_fopen(vdev->archive, chunkname, 0);
			if (zf) {
				buf = g_malloc(zs.size);
				if (zip_fread(zf, buf, zs.size) == (int)zs.size) {
					/* Interleave: [s0_ch0][s0_ch1]...[s1_ch0]... */
					for (uint64_t s = 0; s < total_samples; s++) {
						interleaved[s * dso_count + ch_ordinal] = buf[s];
					}
				} else {
					sr_warn("Short read on %s.", chunkname);
				}
				zip_fclose(zf);
				g_free(buf);
			}
		}
		ch_ordinal++;
	}

	/* Send as a single SR_DF_DSO packet */
	packet.type = SR_DF_DSO;
	packet.payload = &dso;
	memset(&dso, 0, sizeof(dso));
	dso.data = interleaved;
	dso.num_samples = (uint32_t)total_samples;
	dso.en_ch_num = (uint8_t)dso_count;
	dso.sample_bits = vdev->unit_bits ? vdev->unit_bits : 8;
	dso.trig_flag = 0;
	dso.trig_ch = 0;
	dso.trig_offset = 0;

	vdev->bytes_read += total_samples * dso_count;
	sr_session_send(sdi, &packet);

	g_free(interleaved);
	g_slist_free(dso_ch_list);

	return TRUE;
}

/*
 * PXView v3 流式读取分派器：先读 logic 块（L-<ch>/<block>），
 * logic 块读完后切换到 analog 块（A-0/<block>），
 * analog 块读完后切换到 DSO 块（O-<ch>/0）。
 */
static gboolean stream_pxv_session_data(struct sr_dev_inst *sdi)
{
	struct session_vdev *vdev = sdi->priv;

	/* Phase 0: 读取 logic 块 */
	if (vdev->pxv_phase == 0) {
		gboolean got = stream_pxv_logic_data(sdi);
		if (got)
			return TRUE;
		/* logic 块读完，切换到 analog 阶段 */
		vdev->pxv_phase = 1;
		vdev->pxv_cur_analog_block = 0;
	}

	/* Phase 1: 读取 analog 块 */
	if (vdev->pxv_phase == 1) {
		gboolean got = stream_pxv_analog_data(sdi);
		if (got)
			return TRUE;
		/* analog 块读完，切换到 DSO 阶段 */
		vdev->pxv_phase = 2;
	}

	/* Phase 2: 读取 DSO 块 */
	if (vdev->pxv_phase == 2) {
		return stream_pxv_dso_data(sdi);
	}

	return FALSE;
}

static gboolean stream_session_data(struct sr_dev_inst *sdi)
{
	struct session_vdev *vdev;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct zip_stat zs;
	int ret, got_data;
	char capturefile[128];
	void *buf;

	got_data = FALSE;
	vdev = sdi->priv;

	/* PXView v3 format: route to per-channel interleave reader */
	if (vdev->pxv_format)
		return stream_pxv_session_data(sdi);

	if (!vdev->capfile) {
		/* No capture file opened yet, or finished with the last
		 * chunked one. */
		if (vdev->capturefile && (vdev->cur_chunk == 0)) {
			/* capturefile is always the unchunked base name. */
			if (zip_stat(vdev->archive, vdev->capturefile, 0, &zs) != -1) {
				/* No chunks, just a single capture file. */
				vdev->cur_chunk = 0;
				if (!(vdev->capfile = zip_fopen(vdev->archive,
						vdev->capturefile, 0)))
					return FALSE;
				sr_dbg("Opened %s.", vdev->capturefile);
			} else {
				/* Try as first chunk filename. */
				snprintf(capturefile, sizeof(capturefile) - 1, "%s-1", vdev->capturefile);
				if (zip_stat(vdev->archive, capturefile, 0, &zs) != -1) {
					vdev->cur_chunk = 1;
					if (!(vdev->capfile = zip_fopen(vdev->archive,
							capturefile, 0)))
						return FALSE;
					sr_dbg("Opened %s.", capturefile);
				} else {
					sr_err("No capture file '%s' in " "session file '%s'.",
							vdev->capturefile, vdev->sessionfile);
					return FALSE;
				}
			}
		} else {
			/* Capture data is chunked, advance to the next chunk. */
			vdev->cur_chunk++;
			snprintf(capturefile, sizeof(capturefile) - 1, "%s-%d", vdev->capturefile,
					vdev->cur_chunk);
			if (zip_stat(vdev->archive, capturefile, 0, &zs) != -1) {
				if (!(vdev->capfile = zip_fopen(vdev->archive,
						capturefile, 0)))
					return FALSE;
				sr_dbg("Opened %s.", capturefile);
			} else if (vdev->cur_analog_channel < vdev->num_analog_channels) {
				vdev->capturefile = g_strdup_printf("analog-1-%d",
						vdev->num_logic_channels + vdev->cur_analog_channel + 1);
				vdev->cur_analog_channel++;
				vdev->cur_chunk = 0;
				return TRUE;
			} else {
				/* We got all the chunks, finish up. */
				g_free(vdev->capturefile);

				/* If the file has logic channels, the initial value for
				 * capturefile is set by stream_session_data() - however only
				 * once. In order to not mess this mechanism up, we simulate
				 * this here if needed. For purely analog files, capturefile
				 * is not set.
				 */
				if (vdev->num_logic_channels)
					vdev->capturefile = g_strdup("logic-1");
				else
					vdev->capturefile = NULL;
				return FALSE;
			}
		}
	}

	buf = g_malloc(CHUNKSIZE);

	/* unitsize is not defined for purely analog session files. */
	if (vdev->unitsize)
		ret = zip_fread(vdev->capfile, buf,
				CHUNKSIZE / vdev->unitsize * vdev->unitsize);
	else
		ret = zip_fread(vdev->capfile, buf, CHUNKSIZE);

	if (ret > 0) {
		if (vdev->cur_analog_channel != 0) {
			got_data = TRUE;
			packet.type = SR_DF_ANALOG;
			packet.payload = &analog;
			/* TODO: Use proper 'digits' value for this device (and its modes). */
			sr_analog_init(&analog, &encoding, &meaning, &spec, 2);
			analog.meaning->channels = g_slist_prepend(NULL,
					g_array_index(vdev->analog_channels,
						struct sr_channel *, vdev->cur_analog_channel - 1));
			analog.num_samples = ret / sizeof(float);
			analog.meaning->mq = SR_MQ_VOLTAGE;
			analog.meaning->unit = SR_UNIT_VOLT;
			analog.meaning->mqflags = SR_MQFLAG_DC;
			analog.data = (float *) buf;
		} else if (vdev->unitsize) {
			got_data = TRUE;
			if (ret % vdev->unitsize != 0)
				sr_warn("Read size %d not a multiple of the"
					" unit size %d.", ret, vdev->unitsize);
			packet.type = SR_DF_LOGIC;
			packet.payload = &logic;
			logic.length = ret;
			logic.unitsize = vdev->unitsize;
			logic.data = buf;
		} else {
			/*
			 * Neither analog data, nor logic which has
			 * unitsize, must be an unexpected API use.
			 */
			sr_warn("Neither analog nor logic data. Ignoring.");
		}
		if (got_data) {
			vdev->bytes_read += ret;
			sr_session_send(sdi, &packet);
		}
	} else {
		/* done with this capture file */
		zip_fclose(vdev->capfile);
		vdev->capfile = NULL;
		if (vdev->cur_chunk != 0) {
			/* There might be more chunks, so don't fall through
			 * to the SR_DF_END here. */
			got_data = TRUE;
		}
	}
	g_free(buf);

	return got_data;
}

static int receive_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct session_vdev *vdev;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	vdev = sdi->priv;

	if (!vdev->finished && !stream_session_data(sdi))
		vdev->finished = TRUE;
	if (!vdev->finished)
		return G_SOURCE_CONTINUE;

	if (vdev->capfile) {
		zip_fclose(vdev->capfile);
		vdev->capfile = NULL;
	}
	if (vdev->archive) {
		zip_discard(vdev->archive);
		vdev->archive = NULL;
	}

	std_session_send_df_end(sdi);

	return G_SOURCE_REMOVE;
}

/* driver callbacks */

static int dev_open(struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di;
	struct drv_context *drvc;
	struct session_vdev *vdev;

	di = sdi->driver;
	drvc = di->context;
	vdev = g_malloc0(sizeof(struct session_vdev));
	sdi->priv = vdev;
	drvc->instances = g_slist_append(drvc->instances, sdi);

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	const struct session_vdev *const vdev = sdi->priv;
	g_free(vdev->sessionfile);
	g_free(vdev->capturefile);

	g_free(sdi->priv);
	sdi->priv = NULL;

	return SR_OK;
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct session_vdev *vdev;

	(void)cg;

	if (!sdi)
		return SR_ERR;

	vdev = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(vdev->samplerate);
		break;
	case SR_CONF_CAPTURE_UNITSIZE:
		*data = g_variant_new_uint64(vdev->unitsize);
		break;
	case SR_CONF_DEVICE_MODE:
		*data = g_variant_new_int16((int16_t)vdev->pxv_device_mode);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(vdev->total_samples);
		break;
	case SR_CONF_TRIGGER_POS:
		*data = g_variant_new_uint64(vdev->trig_pos);
		break;
	case SR_CONF_TIMEBASE:
		*data = g_variant_new_uint64(vdev->timebase);
		break;
	case SR_CONF_UNIT_BITS:
		*data = g_variant_new_byte(vdev->unit_bits);
		break;
	case SR_CONF_REF_MIN:
		*data = g_variant_new_uint32(vdev->ref_min);
		break;
	case SR_CONF_REF_MAX:
		*data = g_variant_new_uint32(vdev->ref_max);
		break;
	case SR_CONF_NUM_BLOCKS:
		*data = g_variant_new_uint64(vdev->num_blocks);
		break;
	case SR_CONF_SESSION_TIME:
		*data = g_variant_new_int64(vdev->session_time);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct session_vdev *vdev;

	(void)cg;

	vdev = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		vdev->samplerate = g_variant_get_uint64(data);
		sr_info("Setting samplerate to %" PRIu64 ".", vdev->samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		vdev->total_samples = g_variant_get_uint64(data);
		sr_info("Setting total samples to %" PRIu64 ".", vdev->total_samples);
		break;
	case SR_CONF_SESSIONFILE:
		g_free(vdev->sessionfile);
		vdev->sessionfile = g_strdup(g_variant_get_string(data, NULL));
		sr_info("Setting sessionfile to '%s'.", vdev->sessionfile);
		break;
	case SR_CONF_CAPTUREFILE:
		g_free(vdev->capturefile);
		vdev->capturefile = g_strdup(g_variant_get_string(data, NULL));
		sr_info("Setting capturefile to '%s'.", vdev->capturefile);
		break;
	case SR_CONF_CAPTURE_UNITSIZE:
		vdev->unitsize = g_variant_get_uint64(data);
		break;
	case SR_CONF_NUM_LOGIC_CHANNELS:
		vdev->num_logic_channels = g_variant_get_int32(data);
		break;
	case SR_CONF_NUM_ANALOG_CHANNELS:
		vdev->num_analog_channels = g_variant_get_int32(data);
		break;
	case SR_CONF_DEVICE_MODE:
		/* Store PXView device mode (LOGIC=0, DSO=1, ANALOG=2, MSO=3).
		 * Set by session_file.c when parsing the header "device mode" key.
		 * Uses int16 to match the SR_T_INT16 type registered in hwdriver.c
		 * and the int16 variant passed by session_file.c / DeviceAgent. */
		vdev->pxv_device_mode = g_variant_get_int16(data);
		sr_info("Setting PXView device mode to %d.", vdev->pxv_device_mode);
		break;
	case SR_CONF_TRIGGER_POS:
		vdev->trig_pos = g_variant_get_uint64(data);
		break;
	case SR_CONF_TIMEBASE:
		vdev->timebase = g_variant_get_uint64(data);
		break;
	case SR_CONF_UNIT_BITS:
		vdev->unit_bits = g_variant_get_byte(data);
		break;
	case SR_CONF_REF_MIN:
		vdev->ref_min = g_variant_get_uint32(data);
		break;
	case SR_CONF_REF_MAX:
		vdev->ref_max = g_variant_get_uint32(data);
		break;
	case SR_CONF_NUM_BLOCKS:
		vdev->num_blocks = g_variant_get_uint64(data);
		break;
	case SR_CONF_SESSION_TIME:
		vdev->session_time = g_variant_get_int64(data);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	return STD_CONFIG_LIST(key, data, sdi, cg, NO_OPTS, NO_OPTS, devopts);
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct session_vdev *vdev;
	int ret;
	GSList *l;
	struct sr_channel *ch;

	vdev = sdi->priv;
	vdev->bytes_read = 0;
	vdev->cur_analog_channel = 0;
	vdev->analog_channels = g_array_sized_new(FALSE, FALSE,
			sizeof(struct sr_channel *), vdev->num_analog_channels);
	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (ch->type == SR_CHANNEL_ANALOG)
			g_array_append_val(vdev->analog_channels, ch);
	}
	vdev->cur_chunk = 0;
	vdev->finished = FALSE;

	sr_info("Opening archive %s file %s", vdev->sessionfile,
		vdev->capturefile);

	if (!(vdev->archive = zip_open(vdev->sessionfile, 0, &ret))) {
		sr_err("Failed to open session file '%s': "
		       "zip error %d.", vdev->sessionfile, ret);
		return SR_ERR;
	}

	/* Detect PXView v3 format: per-channel chunks L-<ch>/<block> or A-<ch>/<block> */
	{
		struct zip_stat zs;
		struct sr_channel *ch_tmp;
		GSList *l;
		vdev->pxv_format = FALSE;
		vdev->pxv_cur_block = 0;
		vdev->pxv_phase = 0;
		vdev->pxv_cur_analog_block = 0;
		vdev->analog_unit_bytes = 0;
		vdev->analog_is_float = FALSE;

		/* Check for logic chunks (L-<ch>/0) */
		for (l = sdi->channels; l; l = l->next) {
			ch_tmp = l->data;
			if (ch_tmp->type == SR_CHANNEL_LOGIC && ch_tmp->enabled) {
				char tmp_name[32];
				snprintf(tmp_name, sizeof(tmp_name),
					"L-%d/0", ch_tmp->index);
				if (zip_stat(vdev->archive, tmp_name, 0, &zs) >= 0) {
					vdev->pxv_format = TRUE;
					sr_info("Detected PXView v3 per-channel "
						"chunked format (logic).");
				}
				break;
			}
		}

		/* MSO 架构修复：也检测 analog 块 (A-0/0)。
		 * 纯 analog 文件（无 logic 块）也走 v3 路径。 */
		if (!vdev->pxv_format) {
			struct zip_stat azs;
			if (zip_stat(vdev->archive, "A-0/0", 0, &azs) >= 0) {
				vdev->pxv_format = TRUE;
				sr_info("Detected PXView v3 per-channel "
					"chunked format (analog).");
			}
		}

		/* DSO 数据检测：检查 O-<ch>/0 块。
		 * 纯 DSO 文件（无 logic/analog 块）也走 v3 路径。 */
		if (!vdev->pxv_format) {
			for (l = sdi->channels; l; l = l->next) {
				ch_tmp = l->data;
				if (ch_tmp->type == SR_CHANNEL_DSO && ch_tmp->enabled) {
					char tmp_name[32];
					snprintf(tmp_name, sizeof(tmp_name),
						"O-%d/0", ch_tmp->index);
					if (zip_stat(vdev->archive, tmp_name, 0, &zs) >= 0) {
						vdev->pxv_format = TRUE;
						sr_info("Detected PXView v3 per-channel "
							"chunked format (DSO).");
					}
					break;
				}
			}
		}

		/* MSO 架构修复：从 header 内文件解析 analog 数据格式。
		 * 读取 "analog bytes" 和 "analog float" 键。 */
		if (vdev->pxv_format && vdev->num_analog_channels > 0) {
			struct zip_stat hzs;
			if (zip_stat(vdev->archive, "header", 0, &hzs) >= 0) {
				struct zip_file *hf = zip_fopen(vdev->archive, "header", 0);
				if (hf) {
					char *hbuf = g_malloc(hzs.size + 1);
					gint64 nread = zip_fread(hf, hbuf, hzs.size);
					zip_fclose(hf);
					if (nread > 0) {
						hbuf[nread] = '\0';
						GKeyFile *kf = g_key_file_new();
						if (g_key_file_load_from_data(kf, hbuf, nread,
									G_KEY_FILE_NONE, NULL)) {
							gint abytes = g_key_file_get_integer(kf,
								"header", "analog bytes", NULL);
						if (abytes > 0 && abytes <= 8) {
							vdev->analog_unit_bytes = abytes;
							/* 显式读取 analog float 键 */
							gint afloat = g_key_file_get_integer(kf,
									"header", "analog float", NULL);
							vdev->analog_is_float = (afloat == 1);
							sr_info("PXView v3 analog format: "
								"%d bytes/sample, float=%d",
								abytes, vdev->analog_is_float);
						}
							g_key_file_free(kf);
						}
					}
					g_free(hbuf);
				}
			}
		}
	}

	std_session_send_df_header(sdi);

	/* freewheeling source */
	sr_session_source_add(sdi->session, -1, 0, 0, receive_data, (void *)sdi);

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct session_vdev *vdev;

	vdev = sdi->priv;

	vdev->finished = TRUE;

	return SR_OK;
}

/** @private */
SR_PRIV struct sr_dev_driver session_driver = {
	.name = "virtual-session",
	.longname = "Session-emulating driver",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = NULL,
	.dev_list = NULL,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
