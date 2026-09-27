/*
 * CH32H417 USB 传输核（基于本工程的魔改 libusb）
 *
 * 用法对齐 pxlogic.c / usb_ctrl.c：
 *   - 打开：libusb_open + set_auto_detach_kernel_driver(1)
 *           + libusb_set_raw_io_default(hdl, 0)（claim 之前）
 *           + libusb_claim_interface(hdl, 0)
 *   - 命令：EP1 阻塞式 bulk（16B / 32B）
 *   - 采集：EP2/EP3 异步 bulk + 会话事件源 usb_source_add
 *
 * 不依赖 CH375DLL。
 */

#ifndef LIBSIGROK_HARDWARE_WCH_CH32H417_CH32_USB_H
#define LIBSIGROK_HARDWARE_WCH_CH32H417_CH32_USB_H

#include <glib.h>
#include <stdint.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "wch-ch32h417"

/* 端点（与固件 usb_desc.c 描述符一致）：
 *   EP1 OUT/IN : 命令通道
 *   EP2 IN     : 逻辑数据流
 *   EP3 IN     : ADC 数据流
 */
#define CH32_EP_CMD_OUT        0x01
#define CH32_EP_CMD_IN         0x81
#define CH32_EP_LOGIC_IN       0x82
#define CH32_EP_ADC_IN         0x83

#define CH32_USB_INTERFACE     0
#define CH32_CMD_TIMEOUT_MS    500
#define CH32_STREAM_TIMEOUT_MS 200
#define CH32_NUM_TRANSFERS     8
#define CH32_TRANSFER_SIZE     (64 * 1024)   /* 4K 对齐 */

/* 流式采集状态（内嵌于 dev_context） */
struct ch32_usb_stream {
	uint8_t ep;                          /* CH32_EP_LOGIC_IN / CH32_EP_ADC_IN */
	unsigned int num;                    /* 已提交的 transfer 数 */
	struct libusb_transfer **xfer;       /* transfer 数组 */
	gboolean running;
};

/* 打开/关闭设备（libusb open + claim + RAW_IO 处理） */
SR_PRIV int ch32_usb_open(struct sr_dev_inst *sdi);
SR_PRIV int ch32_usb_close(struct sr_dev_inst *sdi);

/* 命令通道：阻塞 bulk（EP1）。
 * write 发 {Cmd,Len,Buf}，read 收 {Cmd|0x10,Len,Status,Buf}。 */
SR_PRIV int ch32_usb_cmd_write(struct sr_dev_inst *sdi, uint8_t *buf, int len);
SR_PRIV int ch32_usb_cmd_read(struct sr_dev_inst *sdi, uint8_t *buf, int len, int *got);

/* 清理命令端点（EP1 IN）残留数据，采集前调用 */
SR_PRIV void ch32_usb_cmd_flush(struct sr_dev_inst *sdi);

/* 流式采集：异步 bulk（EP2/EP3），收到数据回调 ch32h417_handle_stream_data() */
SR_PRIV int ch32_usb_stream_start(struct sr_dev_inst *sdi, uint8_t ep);
SR_PRIV int ch32_usb_stream_stop(struct sr_dev_inst *sdi);

/* 事件泵：作为 usb_source_add 的回调注册 */
SR_PRIV int ch32_usb_event(int fd, int revents, void *cb_data);

/* USB2/USB3 判定 */
SR_PRIV int ch32_usb_is_usb3(struct sr_dev_inst *sdi);

/* 进入 IAP 模式：按 VID/PID 找到设备后经 EP1 发送 0xAE（无需 sr_dev_inst）。
 * 成功后设备会重枚举为 USB CDC 串口(1A86:5539)，烧写由 PXView 的
 * pv/dialogs/iapdialog.* 经 Qt6::SerialPort 完成（不再用 hidapi）。
 * 取代原 CH375 版本中 ch375_write_endpoint(index, 1, cmd[0]=0xAE, ...) 的用法。 */
SR_PRIV int ch32_usb_enter_iap(uint16_t vid, uint16_t pid);

/* 最近一次 ch32_usb_enter_iap() 的失败原因（诊断用，供升级界面显示）。
 * 返回静态字符串，非线程安全；仅在失败返回后立即读取才有意义。 */
SR_PRIV const char *ch32_usb_iap_last_error(void);

#endif /* LIBSIGROK_HARDWARE_WCH_CH32H417_CH32_USB_H */
