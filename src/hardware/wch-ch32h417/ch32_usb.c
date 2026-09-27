/*
 * CH32H417 USB 传输核实现。
 *
 * 全部使用本工程的魔改 libusb；用法对齐 pxlogic.c / usb_ctrl.c。
 * 不依赖 CH375DLL。
 */

#include <config.h>
#include "protocol.h"
#include "ch32_usb.h"

#include <stdio.h>
#include <string.h>
#include <libusb.h>

/* ------------------------------------------------------------------ */
/* 打开 / 关闭                                                        */
/* ------------------------------------------------------------------ */

SR_PRIV int ch32_usb_open(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	int ret;

	devc = sdi->priv;
	usb = sdi->conn;

	if (!devc->usb_dev) {
		sr_err("usb_dev is NULL, cannot open.");
		return SR_ERR;
	}

	if ((ret = libusb_open(devc->usb_dev, &usb->devhdl)) != 0) {
		sr_err("Failed to open device: %s", libusb_error_name(ret));
		return SR_ERR;
	}

	libusb_set_auto_detach_kernel_driver(usb->devhdl, 1);

	/* 关闭 WinUSB RAW_IO 默认策略（16B 命令 bulk 握手需要非对齐传输）。
	 * 必须在 claim 之前设置，避免 claim 后 TRUE→FALSE 切换不可靠。
	 * 仅 Windows/内部 libusb 子模块存在此 fork API。 */
#ifdef HAVE_LIBUSB_OS_HANDLE
	libusb_set_raw_io_default(usb->devhdl, 0);
#endif

	if ((ret = libusb_claim_interface(usb->devhdl, CH32_USB_INTERFACE)) < 0) {
		sr_err("Failed to claim interface: %s", libusb_error_name(ret));
		libusb_close(usb->devhdl);
		usb->devhdl = NULL;
		return SR_ERR;
	}

	/* 记录 USB 速度（用于采样率/位宽判定） */
	devc->is_usb2 = !ch32_usb_is_usb3(sdi);

	return SR_OK;
}

SR_PRIV int ch32_usb_close(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;

	usb = sdi->conn;
	if (usb && usb->devhdl) {
		libusb_release_interface(usb->devhdl, CH32_USB_INTERFACE);
		libusb_close(usb->devhdl);
		usb->devhdl = NULL;
	}
	return SR_OK;
}

/* ------------------------------------------------------------------ */
/* 命令通道（阻塞 bulk，EP1）                                          */
/* ------------------------------------------------------------------ */

SR_PRIV int ch32_usb_cmd_write(struct sr_dev_inst *sdi, uint8_t *buf, int len)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	int transferred = 0;
	int ret;

	if (!usb || !usb->devhdl)
		return SR_ERR;

	ret = libusb_bulk_transfer(usb->devhdl, CH32_EP_CMD_OUT,
			buf, len, &transferred, CH32_CMD_TIMEOUT_MS);
	if (ret != 0) {
		sr_err("cmd write failed: %s", libusb_error_name(ret));
		libusb_clear_halt(usb->devhdl, CH32_EP_CMD_OUT);
		return SR_ERR;
	}
	return SR_OK;
}

SR_PRIV int ch32_usb_cmd_read(struct sr_dev_inst *sdi, uint8_t *buf, int len, int *got)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	int transferred = 0;
	int ret;

	if (!usb || !usb->devhdl)
		return SR_ERR;

	ret = libusb_bulk_transfer(usb->devhdl, CH32_EP_CMD_IN,
			buf, len, &transferred, CH32_CMD_TIMEOUT_MS);
	if (ret != 0) {
		sr_err("cmd read failed: %s", libusb_error_name(ret));
		libusb_clear_halt(usb->devhdl, CH32_EP_CMD_IN);
		return SR_ERR;
	}
	if (got)
		*got = transferred;
	return SR_OK;
}

SR_PRIV void ch32_usb_cmd_flush(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	uint8_t buffer[PACKET_SIZE_IN];
	int transferred, ret, retry = 0;

	if (!usb || !usb->devhdl)
		return;

	/* 短超时循环读，丢弃设备端未取走的命令响应残留 */
	while (retry++ < 10) {
		transferred = 0;
		ret = libusb_bulk_transfer(usb->devhdl, CH32_EP_CMD_IN,
				buffer, sizeof(buffer), &transferred, 1);
		if (ret != 0 || transferred == 0)
			break;
		sr_warn("stale cmd data flushed (0x%02x)", buffer[0]);
	}
}

/* ------------------------------------------------------------------ */
/* 流式采集（异步 bulk，EP2/EP3）                                     */
/* ------------------------------------------------------------------ */

static void LIBUSB_CALL ch32_receive_transfer(struct libusb_transfer *xfer)
{
	struct sr_dev_inst *sdi = xfer->user_data;
	struct dev_context *devc;

	devc = sdi->priv;

	if (xfer->status == LIBUSB_TRANSFER_COMPLETED && xfer->actual_length > 0)
		ch32h417_handle_stream_data(sdi, xfer->buffer, xfer->actual_length);

	if (devc->stream.running && !devc->acq_aborted) {
		if (libusb_submit_transfer(xfer) != 0)
			sr_err("resubmit transfer failed");
	}
}

SR_PRIV int ch32_usb_stream_start(struct sr_dev_inst *sdi, uint8_t ep)
{
	struct dev_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	unsigned int i;

	devc->stream.ep = ep;
	devc->stream.num = CH32_NUM_TRANSFERS;
	devc->stream.running = TRUE;
	devc->stream.xfer = g_try_malloc0(sizeof(*devc->stream.xfer) *
			devc->stream.num);
	if (!devc->stream.xfer) {
		devc->stream.running = FALSE;
		return SR_ERR_MALLOC;
	}

	for (i = 0; i < devc->stream.num; i++) {
		uint8_t *buf;
		struct libusb_transfer *xfer;

		buf = g_try_malloc(CH32_TRANSFER_SIZE);
		if (!buf)
			return SR_ERR_MALLOC;
		xfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(xfer, usb->devhdl, ep, buf,
				CH32_TRANSFER_SIZE, ch32_receive_transfer,
				sdi, CH32_STREAM_TIMEOUT_MS);
		if (libusb_submit_transfer(xfer) != 0) {
			sr_err("submit transfer failed (ep=0x%02x)", ep);
			libusb_free_transfer(xfer);
			g_free(buf);
			return SR_ERR;
		}
		devc->stream.xfer[i] = xfer;
	}
	return SR_OK;
}

SR_PRIV int ch32_usb_stream_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	unsigned int i;

	if (!devc->stream.running)
		return SR_OK;

	devc->stream.running = FALSE;

	for (i = 0; i < devc->stream.num; i++) {
		struct libusb_transfer *xfer = devc->stream.xfer[i];
		if (!xfer)
			continue;
		libusb_cancel_transfer(xfer);
	}
	/* 取消是异步的，这里先释放本端资源；transfer 缓冲在回调结束前
	 * 由 libusb 持有，交由事件循环收尾。 */
	for (i = 0; i < devc->stream.num; i++) {
		struct libusb_transfer *xfer = devc->stream.xfer[i];
		if (!xfer)
			continue;
		g_free(xfer->buffer);
		libusb_free_transfer(xfer);
		devc->stream.xfer[i] = NULL;
	}
	g_free(devc->stream.xfer);
	devc->stream.xfer = NULL;
	devc->stream.num = 0;

	return SR_OK;
}

/* ------------------------------------------------------------------ */
/* 事件泵（usb_source_add 回调）                                      */
/* ------------------------------------------------------------------ */

SR_PRIV int ch32_usb_event(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi = cb_data;
	struct dev_context *devc = sdi->priv;
	struct timeval tv;
	int completed = 0;

	(void)fd;
	(void)revents;

	tv.tv_sec = 0;
	tv.tv_usec = 0;
	if (devc->sr_ctx)
		libusb_handle_events_timeout_completed(
				devc->sr_ctx->libusb_ctx, &tv, &completed);

	return TRUE;
}

/* ------------------------------------------------------------------ */
/* USB2 / USB3 判定                                                    */
/* ------------------------------------------------------------------ */

SR_PRIV int ch32_usb_is_usb3(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	enum libusb_speed sp;

	if (!devc->usb_dev)
		return FALSE;

	sp = libusb_get_device_speed(devc->usb_dev);
	return (sp == LIBUSB_SPEED_SUPER);
}

/* ------------------------------------------------------------------ */
/* 进入 IAP 模式（供固件升级界面调用，不依赖 sr_dev_inst）             */
/* ------------------------------------------------------------------ */

/* 最近一次 ch32_usb_enter_iap() 的失败原因（供固件升级界面显示给用户）。
 * 非线程安全：升级流程在单个工作线程里串行调用。 */
static char g_iap_err[256] = {0};

SR_PRIV const char *ch32_usb_iap_last_error(void)
{
	return g_iap_err[0] ? g_iap_err : "unknown";
}

SR_PRIV int ch32_usb_enter_iap(uint16_t vid, uint16_t pid)
{
	libusb_context *ctx = NULL;
	libusb_device **list = NULL;
	libusb_device_handle *hdl = NULL;
	uint8_t cmd[64] = {0};
	int num, i, transferred = 0;
	int ret = SR_ERR;
	int r = 0;
	gboolean saw = FALSE;   /* 是否见到过目标 VID/PID（区分"不存在"和"打不开"） */

	/* 命令包：Buf[0]=CMD_ENTER_IAP(0xAE)，其余清零（与原 CH375 版一致）。 */
	cmd[0] = 0xAE;
	g_iap_err[0] = '\0';

	if (libusb_init(&ctx) != 0) {
		snprintf(g_iap_err, sizeof(g_iap_err), "libusb_init failed");
		sr_err("IAP: libusb_init failed.");
		return SR_ERR;
	}

	num = libusb_get_device_list(ctx, &list);
	if (num < 0) {
		snprintf(g_iap_err, sizeof(g_iap_err),
				"enumerate USB devices failed");
		sr_err("IAP: failed to enumerate USB devices.");
		goto out;
	}

	for (i = 0; i < num; i++) {
		struct libusb_device_descriptor des;

		if (libusb_get_device_descriptor(list[i], &des) != 0)
			continue;
		if (des.idVendor != vid || des.idProduct != pid)
			continue;
		saw = TRUE;
		if (libusb_open(list[i], &hdl) != 0) {
			hdl = NULL;
			continue;
		}
		break;
	}

	if (!hdl) {
		if (saw)
			snprintf(g_iap_err, sizeof(g_iap_err),
					"%04x:%04x present but open failed", vid, pid);
		else
			snprintf(g_iap_err, sizeof(g_iap_err),
					"%04x:%04x not present", vid, pid);
		sr_err("IAP: device %04x:%04x not found.", vid, pid);
		goto out;
	}

	libusb_set_auto_detach_kernel_driver(hdl, 1);

	/* 与正常打开一致：claim 之前关闭 WinUSB RAW_IO 默认策略，
	 * 否则 64 字节命令包在 WinUSB 上会因对齐要求失败。 */
#ifdef HAVE_LIBUSB_OS_HANDLE
	libusb_set_raw_io_default(hdl, 0);
#endif

	r = libusb_claim_interface(hdl, CH32_USB_INTERFACE);
	if (r < 0) {
		/* 最常见的原因：设备已被本程序（或其它上位机）打开并占用。 */
		snprintf(g_iap_err, sizeof(g_iap_err),
				"claim interface %d failed (%s)", CH32_USB_INTERFACE,
				libusb_error_name(r));
		sr_err("IAP: failed to claim interface %d.", CH32_USB_INTERFACE);
		goto out;
	}

	/* 发送 0xAE；设备收到后进入 IAP 并重枚举，无需等待响应。 */
	r = libusb_bulk_transfer(hdl, CH32_EP_CMD_OUT, cmd, sizeof(cmd),
			&transferred, CH32_CMD_TIMEOUT_MS);
	if (r == 0) {
		sr_info("IAP: enter-IAP command sent (%d bytes).", transferred);
		ret = SR_OK;
	} else {
		snprintf(g_iap_err, sizeof(g_iap_err),
				"bulk transfer EP1 failed (%s)", libusb_error_name(r));
		sr_err("IAP: failed to send enter-IAP command.");
	}

	libusb_release_interface(hdl, CH32_USB_INTERFACE);

out:
	if (hdl)
		libusb_close(hdl);
	if (list)
		libusb_free_device_list(list, 1);
	if (ctx)
		libusb_exit(ctx);

	return ret;
}
