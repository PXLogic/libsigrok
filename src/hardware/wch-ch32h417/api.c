/*
 * This file is part of the LogicAnalyzer project.
 * LogicAnaylzer is based on libsigrok.
 *
 * Copyright (C) 2026 Q2H2
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
#include "protocol.h"
#include <math.h>
#include <ctype.h>

/* 支持的设备列表 */
static const struct ch32h417_profile supported_devices[] = {
	/* CH32H417逻辑分析仪 */
	{USB_VENDOR_ID, USB_PRODUCT_ID, "CH32H417", "CH32H417 Logic Analyzer", "01", 
	 DEV_CAPS_16BIT_LOGIC | DEV_CAPS_ADC},
	{USB_VENDOR_ID, USB_PRODUCT_ID, "CH32H417", "WeAct Studio Logic Analyzer V2", "b5",
	 DEV_CAPS_16BIT_LOGIC},
	/* 同一板子接 USB3 口时以 0x5538 枚举（固件 USB2/USB3 用了不同 PID，
	 * 见 APP/Common/usb_desc.h: DEF_USB_PID 0x5537 / DEF_USB30_PID 0x5538）。
	 * 能力与对应的 USB2 表项一致；不列在这里则 USB3 设备完全认不到。 */
	{USB_VENDOR_ID, USB_PRODUCT_ID_USB3, "CH32H417", "CH32H417 Logic Analyzer", "01",
	 DEV_CAPS_16BIT_LOGIC | DEV_CAPS_ADC},
	{USB_VENDOR_ID, USB_PRODUCT_ID_USB3, "CH32H417", "WeAct Studio Logic Analyzer V2", "b5",
	 DEV_CAPS_16BIT_LOGIC},
	ALL_ZERO};

/* 扫描选项 */
static const uint32_t scanopts[] = {
	SR_CONF_CONN,
};

/* 驱动选项 */
static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
};

/* 设备选项 */
static const uint32_t devopts[] = {
	/* 仅保留 PXView 实际消费的键（DeviceOptions 绑定 / SamplingBar /
	 * TriggerDock）。STREAM / VLD_CH_NUM / USB_SPEED / *_SUPPORT 一律不声明。 */
	SR_CONF_CONTINUOUS,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_CONN | SR_CONF_GET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_NUM_LOGIC_CHANNELS | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PATTERN_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_THRESHOLD_VALUE_1 | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_HARDWARE_VERSION | SR_CONF_GET,
	SR_CONF_USB_VERSION | SR_CONF_GET,
};

/* ADC通道组设备选项 */
static const uint32_t devopts_adc[] = {
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_ADC_PRECISION | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_ADC_CHANNEL | SR_CONF_GET | SR_CONF_SET,
};

/* 触发类型 */
static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
	SR_TRIGGER_EDGE,
};

/* 逻辑分析仪采样率 - USB 3.0 (最高200MHz) */
static const uint64_t logic_samplerates_usb3[] = {
	SR_MHZ(200),
	SR_MHZ(187.5),
	SR_MHZ(175),
	SR_MHZ(162.5),
	SR_MHZ(156.25),
	SR_MHZ(150),
	SR_MHZ(137.5),
	SR_MHZ(125),
	SR_MHZ(112.5),
	SR_MHZ(100),
	SR_MHZ(93.75),
	SR_MHZ(87.5),
	SR_MHZ(81.5),
	SR_MHZ(75.125),
	SR_MHZ(75),	
	SR_MHZ(68.75),
	SR_MHZ(50),
	SR_MHZ(40),
	SR_MHZ(30),
	SR_MHZ(20),
	SR_MHZ(16),
	SR_MHZ(12),
	SR_MHZ(8),
	SR_MHZ(6),
	SR_MHZ(4),
};

/* 逻辑分析仪采样率 - USB 2.0 16通道 (最高20MHz) */
static const uint64_t logic_samplerates_usb2_16ch[] = {
	SR_MHZ(20),
	SR_MHZ(16),
	SR_MHZ(12),
	SR_MHZ(8),
	SR_MHZ(6),
	SR_MHZ(4),
};

/* 逻辑分析仪采样率 - USB 2.0 8通道 (最高40MHz) */
static const uint64_t logic_samplerates_usb2_8ch[] = {
	SR_MHZ(40),
	SR_MHZ(30),
	SR_MHZ(20),
	SR_MHZ(16),
	SR_MHZ(12),
	SR_MHZ(8),
	SR_MHZ(6),
	SR_MHZ(4),
};

/* ADC采样率 */
static const uint64_t adc_samplerates[] = {
	SR_MHZ(20),
	SR_MHZ(16),
	SR_MHZ(13.33),
	SR_MHZ(11.43),
	SR_MHZ(10),
};

/* 工作模式名称 */
static const char *mode_names[] = {
	[MODE_LOGIC_ANALYZER] = "Logic Analyzer",
	[MODE_ADC] = "ADC",
};

/*
 * 注：原 CH375 版本的 ch32h417_flush_cmd_endpoint() / ch375_init_comm_params()
 * 已移除 —— 通信参数（超时/IO 模式/缓冲上传）是 CH375DLL 专有概念，在 libusb
 * 路径下由 ch32_usb_open()（claim + RAW_IO）与 ch32_usb_stream_start()（异步
 * transfer 环）取代；命令端点残留由 ch32_usb_cmd_flush() 清理。
 */

/* 检查设备是否匹配 */
static gboolean is_plausible(uint16_t vid, uint16_t pid)
{
	int i;

	for (i = 0; supported_devices[i].vid; i++)
	{
		if (vid == supported_devices[i].vid &&
			pid == supported_devices[i].pid)
			return TRUE;
	}

	return FALSE;
}

/**
 * @brief 解析USB设备路径字符串，提取实例序列号，输出小写
 * @param dev_path 输入字符串，例如 "\\\\?\\usb#vid_1a86&pid_5537#b5e339e33906e19c7f6416064c#{5e7f6bdf‑1ce5‑4d78‑bbcf‑d20c44329f7d}"
 * @param out_sn 输出缓冲区，存放小写序列号
 * @param buf_sz 输出缓冲区大小
 * @return 0成功，‑1失败
 */
/*
 * 注：原 extract_usb_serial()（解析 Windows CH375 设备路径提取实例 SN）已删除。
 * libusb 路径下序列号由 libusb_get_string_descriptor_ascii() 直接读取，见 scan()。
 */

/**
 * @brief 判断 str 是否以前缀 prefix 开头
 * @return 1:是前缀；0:不是
 */
int str_startswith(const char *str, const char *prefix)
{
    if (!str || !prefix)
        return 0;
    size_t pre_len = strlen(prefix);
    size_t str_len = strlen(str);
    if (str_len < pre_len)
        return 0;
    return (memcmp(str, prefix, pre_len) == 0);
}


/* 扫描设备 */
static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_channel *ch;
	struct sr_channel_group *cg;
	struct sr_config *src;
	const struct ch32h417_profile *prof;
	GSList *l, *devices;
	int i, j;
	int num_logic_channels, num_analog_channels;
	const char *conn;
	char connection_id[64];
	char channel_name[16];
	uint16_t vid, pid;
	struct libusb_device **devlist = NULL;
	struct sr_usb_dev_inst *usb;
	int num_devices = 0;
	gboolean want_conn;

	drvc = di->context;

	conn = NULL;
	for (l = options; l; l = l->next)
	{
		src = l->data;
		switch (src->key)
		{
		case SR_CONF_CONN:
			conn = g_variant_get_string(src->data, NULL);
			break;
		}
	}

	devices = NULL;

	/*
	 * 枚举 USB 设备（取代原 CH375DLL 的 0..CH375_MAX_NUMBER 索引模型）。
	 * conn 为 "bus.address" 时只匹配该设备。
	 */
	num_devices = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (num_devices < 0) {
		sr_err("Failed to enumerate USB devices: %s",
				libusb_error_name(num_devices));
		return NULL;
	}

	want_conn = (conn && *conn);
	if (want_conn && conn[0] == '#')
		conn++;

	for (i = 0; i < num_devices; i++) {
		struct libusb_device_descriptor des;
		uint8_t bus, addr;
		char sn[256] = {0};
		libusb_device_handle *hdl = NULL;
		gboolean serial_ok = FALSE;

		if (libusb_get_device_descriptor(devlist[i], &des) != 0)
			continue;

		vid = des.idVendor;
		pid = des.idProduct;

		/* 只处理本驱动支持的 VID/PID。
		 * 注意：固件里 USB2 与 USB3 使用同一个 PID(0x5537)，
		 * 速度必须靠 libusb_get_device_speed() 判定。 */
		prof = NULL;
		for (j = 0; supported_devices[j].vid; j++) {
			if (vid == supported_devices[j].vid &&
					pid == supported_devices[j].pid) {
				prof = &supported_devices[j];
				break;
			}
		}
		if (!prof)
			continue;

		bus = libusb_get_bus_number(devlist[i]);
		addr = libusb_get_device_address(devlist[i]);

		/* conn 过滤（"bus.address"）。 */
		if (want_conn) {
			char cid[64];
			snprintf(cid, sizeof(cid), "%u.%u", bus, addr);
			if (strcmp(cid, conn) != 0)
				continue;
		}

		/* 读序列号以区分硬件版本（原为 Windows 设备路径实例 ID）。
		 * 需要临时打开设备句柄；只为读字符串，不 claim 接口。 */
		if (des.iSerialNumber && libusb_open(devlist[i], &hdl) == 0) {
			if (libusb_get_string_descriptor_ascii(hdl, des.iSerialNumber,
					(unsigned char *)sn, sizeof(sn) - 1) > 0)
				serial_ok = TRUE;
			libusb_close(hdl);
			hdl = NULL;
		}
		if (serial_ok) {
			for (gchar *p = sn; *p; p++)
				*p = (gchar)tolower((unsigned char)*p);
		}

		/* 按 SN 前缀细分 profile（"01" / "b5" 两种硬件版本）。 */
		prof = NULL;
		for (j = 0; supported_devices[j].vid; j++) {
			if (vid == supported_devices[j].vid &&
					pid == supported_devices[j].pid &&
					(!serial_ok ||
					 str_startswith(sn, supported_devices[j].SN_start))) {
				prof = &supported_devices[j];
				break;
			}
		}
		if (!prof)
			continue;

		snprintf(connection_id, sizeof(connection_id), "%u.%u", bus, addr);

		sdi = g_malloc0(sizeof(struct sr_dev_inst));
		sdi->status = SR_ST_INITIALIZING;
		sdi->vendor = g_strdup(prof->vendor);
		sdi->model = g_strdup(prof->model);
		sdi->connection_id = g_strdup(connection_id);
		sdi->serial_num = g_strdup(serial_ok ? sn : "");

		/* CH32H417支持8通道或16通道模式 */
		num_logic_channels = 16; /* 默认16通道，可通过配置切换为8通道 */
		num_analog_channels = (prof->dev_caps & DEV_CAPS_ADC) ? 2 : 0;

		/* 创建逻辑通道组 */
		cg = g_malloc0(sizeof(struct sr_channel_group));
		cg->name = g_strdup("Logic");
		for (j = 0; j < num_logic_channels; j++)
		{
			sprintf(channel_name, "D%d", j);
			ch = sr_channel_new(sdi, j, SR_CHANNEL_LOGIC,
								TRUE, channel_name);
			cg->channels = g_slist_append(cg->channels, ch);
		}
		sdi->channel_groups = g_slist_append(NULL, cg);

		/* 创建模拟通道组 */
		for (j = 0; j < num_analog_channels; j++)
		{
			snprintf(channel_name, sizeof(channel_name), "A%d", j);
			ch = sr_channel_new(sdi, j + num_logic_channels,
								SR_CHANNEL_ANALOG, TRUE, channel_name);

			cg = g_malloc0(sizeof(struct sr_channel_group));
			cg->name = g_strdup(channel_name);
			cg->channels = g_slist_append(NULL, ch);
			sdi->channel_groups = g_slist_append(sdi->channel_groups, cg);
		}

		devc = ch32h417_dev_new();
		devc->profile = prof;
		devc->sr_ctx = drvc->sr_ctx;
		/* libusb_ref_device：libusb_free_device_list(unref=1) 之后仍要能
		 * 在 dev_open 中使用该设备；dev_close 负责 unref。 */
		devc->usb_dev = libusb_ref_device(devlist[i]);
		sdi->priv = devc;

		sdi->inst_type = SR_INST_USB;
		usb = sr_usb_dev_inst_new(bus, addr, NULL);
		sdi->conn = usb;

		sdi->status = SR_ST_INACTIVE;
		devices = g_slist_append(devices, sdi);

		sr_info("Found CH32H417 device at %s (VID=0x%04x PID=0x%04x, sn=%s)",
				connection_id, vid, pid, serial_ok ? sn : "(none)");
	}

	libusb_free_device_list(devlist, 1);

	return std_scan_complete(di, devices);
}

/* 清理设备 */
static int dev_clear(const struct sr_dev_driver *di)
{
	return std_dev_clear(di);
}

/* 打开设备 */
static int dev_open(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	gboolean already_open;

	devc = sdi->priv;
	usb = sdi->conn;
	already_open = (usb && usb->devhdl != NULL);

	if (already_open)
	{
		sr_info("Device already open (%s)", sdi->connection_id);
	}
	else
	{
		/* libusb_open + set_auto_detach + 关闭 RAW_IO 默认 + claim，
		 * 全部在 ch32_usb_open() 内完成（用法对齐 pxlogic.c）。 */
		if (ch32_usb_open(sdi) != SR_OK) {
			sr_err("Failed to open device %s", sdi->connection_id);
			return SR_ERR;
		}

		/* 清空命令端点残留（WeActStudio 的 flush 补丁）。 */
		ch32_usb_cmd_flush(sdi);
	}

	if (devc->mode == MODE_LOGIC_ANALYZER)
	{
		devc->is_usb2 = ch32h417_detect_usb_speed(sdi);
		if (devc->is_usb2)
		{
			if (devc->num_channels == 16) {
				devc->samplerates = logic_samplerates_usb2_16ch;
				if(str_startswith(devc->profile->SN_start, "b5"))
					devc->num_samplerates = ARRAY_SIZE(logic_samplerates_usb2_16ch);
				else
					devc->num_samplerates = 1;
			} else {
				devc->samplerates = logic_samplerates_usb2_8ch;
				if(str_startswith(devc->profile->SN_start, "b5"))
					devc->num_samplerates = ARRAY_SIZE(logic_samplerates_usb2_8ch);
				else
					devc->num_samplerates = 1;
			}
		}
		else
		{
			devc->samplerates = logic_samplerates_usb3;
			if(str_startswith(devc->profile->SN_start, "b5"))
				devc->num_samplerates = ARRAY_SIZE(logic_samplerates_usb3);
			else
				devc->num_samplerates = ARRAY_SIZE(logic_samplerates_usb3) - 11;
		}
		devc->cur_samplerate = devc->samplerates[0];
	}

	/* 设置默认采样率 */
	if (devc->cur_samplerate == 0)
	{
		devc->cur_samplerate = devc->samplerates[0];
	}

	sr_info("Opened device %s", sdi->connection_id);

	return SR_OK;
}

/* 关闭设备 */
static int dev_close(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	devc = sdi->priv;
	usb = sdi->conn;

	if (!usb || !usb->devhdl)
		return SR_ERR_BUG;

	sr_info("Closing device %s", sdi->connection_id);

	ch32_usb_stream_stop(sdi);
	ch32_usb_close(sdi);

	/* 释放 scan() 中 libusb_ref_device() 的引用。 */
	if (devc->usb_dev) {
		libusb_unref_device(devc->usb_dev);
		devc->usb_dev = NULL;
	}

	return SR_OK;
}

/* 获取配置 */
static int config_get(uint32_t key, GVariant **data,
					  const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;

	(void)cg;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	switch (key)
	{
	case SR_CONF_CONN:
		if (!sdi->connection_id)
			return SR_ERR_ARG;
		*data = g_variant_new_string(sdi->connection_id);
		break;
	case SR_CONF_LIMIT_FRAMES:
		*data = g_variant_new_uint64(devc->limit_frames);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_CAPTURE_RATIO:
		*data = g_variant_new_uint64(devc->capture_ratio);
		break;
	case SR_CONF_NUM_LOGIC_CHANNELS:
		*data = g_variant_new_int32(devc->num_channels);
		break;
	case SR_CONF_PATTERN_MODE:
		*data = g_variant_new_string(mode_names[devc->mode]);
		break;
	case SR_CONF_HARDWARE_VERSION:
		ch32h417_get_fw_version(sdi);
		*data = g_variant_new_uint64(devc->fw_version);
		break;
	case SR_CONF_USB_VERSION:
		/* 2表示USB2.0，3表示USB3.0 */
		*data = g_variant_new_uint64(devc->is_usb2 ? 2 : 3);
		break;
	case SR_CONF_ADC_PRECISION:
		/* 返回ADC精度：8或10 */
		*data = g_variant_new_uint64((devc->adc_width == ADC_WIDTH_8BIT) ? 8 : 10);
		break;
	case SR_CONF_ADC_CHANNEL:
		/* 返回ADC通道：0或1 */
		*data = g_variant_new_uint64((devc->adc_channel == ADC_CHANNEL_1) ? 0 : 1);
		break;
	case SR_CONF_THRESHOLD_VALUE_1:
		*data = g_variant_new_uint64(devc->threshold_value_1);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

/* 设置配置 */
static int config_set(uint32_t key, GVariant *data,
					  const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	int idx;

	(void)cg;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	switch (key)
	{
	case SR_CONF_SAMPLERATE:
		if ((idx = std_u64_idx(data, devc->samplerates,
							   devc->num_samplerates)) < 0)
			return SR_ERR_ARG;
		devc->cur_samplerate = devc->samplerates[idx];
		sr_info("Samplerate set to %" PRIu64 " Hz", devc->cur_samplerate);
		break;
	case SR_CONF_LIMIT_FRAMES:
		devc->limit_frames = g_variant_get_uint64(data);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_CAPTURE_RATIO:
		devc->capture_ratio = g_variant_get_uint64(data);
		break;
	case SR_CONF_NUM_LOGIC_CHANNELS:
	{
		int32_t num = g_variant_get_int32(data);
		if (num != 8 && num != 16)
		{
			sr_err("Invalid channel count: %" PRId32 " (must be 8 or 16)", num);
			return SR_ERR_ARG;
		}
		devc->num_channels = num;
		devc->sample_width = (num == 16) ? SAMPLE_WIDTH_16BIT : SAMPLE_WIDTH_8BIT;
		sr_info("Channel count set to %d", devc->num_channels);

		/* USB 2.0时，通道数变化需要更新采样率表 */
		if (devc->is_usb2 && devc->mode == MODE_LOGIC_ANALYZER)
		{
			if (devc->num_channels == 16) {
				devc->samplerates = logic_samplerates_usb2_16ch;
				devc->num_samplerates = ARRAY_SIZE(logic_samplerates_usb2_16ch);
			} else {
				devc->samplerates = logic_samplerates_usb2_8ch;
				devc->num_samplerates = ARRAY_SIZE(logic_samplerates_usb2_8ch);
			}
			devc->cur_samplerate = devc->samplerates[0];
		}
	}
	break;
	case SR_CONF_PATTERN_MODE:
	{
		int mode_idx = std_str_idx(data, ARRAY_AND_SIZE(mode_names));
		if (mode_idx < 0)
			return SR_ERR_ARG;

		if (mode_idx == devc->mode)
			break;

		devc->mode = (enum ch32h417_mode)mode_idx;

		/* 切换模式时更新采样率列表 */
		if (devc->mode == MODE_LOGIC_ANALYZER)
		{
			if (devc->is_usb2)
			{
				if (devc->num_channels == 16) {
					devc->samplerates = logic_samplerates_usb2_16ch;
					if(str_startswith(devc->profile->SN_start, "b5"))
						devc->num_samplerates = ARRAY_SIZE(logic_samplerates_usb2_16ch);
					else
						devc->num_samplerates = 1;
				} else {
					devc->samplerates = logic_samplerates_usb2_8ch;
					if(str_startswith(devc->profile->SN_start, "b5"))
						devc->num_samplerates = ARRAY_SIZE(logic_samplerates_usb2_8ch);
					else
						devc->num_samplerates = 1;
				}
			}
			else
			{
				devc->samplerates = logic_samplerates_usb3;
				if(str_startswith(devc->profile->SN_start, "b5"))
					devc->num_samplerates = ARRAY_SIZE(logic_samplerates_usb3);
				else
					devc->num_samplerates = ARRAY_SIZE(logic_samplerates_usb3) - 11;
			}
		}
		else
		{
			devc->samplerates = adc_samplerates;
			devc->num_samplerates = ARRAY_SIZE(adc_samplerates);
		}

		devc->cur_samplerate = devc->samplerates[0];

		sr_info("Device mode set to %s", mode_names[devc->mode]);
	}
	break;
	case SR_CONF_ADC_PRECISION:
	{
		uint64_t precision = g_variant_get_uint64(data);
		if (precision == 8) {
			devc->adc_width = ADC_WIDTH_8BIT;
			sr_info("ADC precision set to 8-bit");
		} else if (precision == 10) {
			devc->adc_width = ADC_WIDTH_10BIT;
			sr_info("ADC precision set to 10-bit");
		} else {
			sr_err("Invalid ADC precision: %" PRIu64 " (must be 8 or 10)", precision);
			return SR_ERR_ARG;
		}
	}
	break;
	case SR_CONF_ADC_CHANNEL:
	{
		uint64_t channel = g_variant_get_uint64(data);
		if (channel == 0) {
			devc->adc_channel = ADC_CHANNEL_1;
			sr_info("ADC channel set to Channel 0");
		} else if (channel == 1) {
			devc->adc_channel = ADC_CHANNEL_2;
			sr_info("ADC channel set to Channel 1");
		} else {
			sr_err("Invalid ADC channel: %" PRIu64 " (must be 0 or 1)", channel);
			return SR_ERR_ARG;
		}
	}
	break;
	case SR_CONF_THRESHOLD_VALUE_1:
	{
		uint64_t value = g_variant_get_uint64(data);
		if(((value>>8) & 0x80) == 0x80)
		{
			if((value & 0xff) > 33)
			{
				sr_err("threshold_value_1 Input Level value out of range: %" PRIu64 " (must be 10-33)", value & 0xff);
				return SR_ERR_ARG;
			}
			devc->threshold_value_1 = (uint16_t)value;
			sr_info("Setting threshold_value_1 to %x", devc->threshold_value_1);
		}
		else
		{
			if (value > 1024) {
				sr_err("DAC value out of range: %" PRIu64 " (must be 0-1024)", value);
				return SR_ERR_ARG;
			}
			devc->threshold_value_1 = (uint16_t)value;
			sr_info("Setting threshold_value_1 to %d", devc->threshold_value_1);
		}
		return ch32h417_set_threshold_value(sdi, devc->threshold_value_1);
	}
	break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

/* 配置列表 */
static int config_list(uint32_t key, GVariant **data,
					   const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;

	devc = (sdi) ? sdi->priv : NULL;

	switch (key)
	{
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		if (cg)
		{
			/* ADC通道组 */
			return STD_CONFIG_LIST(key, data, sdi, cg,
								   scanopts, drvopts, devopts_adc);
		}
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
	case SR_CONF_SAMPLERATE:
		if (!devc)
			return SR_ERR_NA;
		*data = std_gvar_samplerates(devc->samplerates, devc->num_samplerates);
		break;
	case SR_CONF_TRIGGER_MATCH:
		*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
		break;
	case SR_CONF_PATTERN_MODE:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(mode_names));
		break;
	case SR_CONF_THRESHOLD_VALUE_1:
		if (!devc)
			return SR_ERR_NA;
		if(str_startswith(devc->profile->SN_start, "b5"))
		{
			*data = std_gvar_min_max_step_thresholds(1.0, 3.3, 0.1);
		}
		else
		{
			*data = std_gvar_min_max_step_thresholds(0.8, 2.3, 0.1);
		}
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

/* 停止采集 */
static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	return ch32h417_acquisition_stop(sdi);
}

/* 驱动注册信息 */
static struct sr_dev_driver ch32h417_driver_info = {
	.name = "wch-ch32h417",
	.longname = "WCH CH32H417 Logic Analyzer",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = ch32h417_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(ch32h417_driver_info);
