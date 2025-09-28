#include "channel_select.h"
#include <stddef.h>

/* Simple sequential channel iterator using PLC_CHANNEL_FREQS_HZ/PLC_CHANNEL_NAMES
 * These arrays are defined in channel_select.h (generated from gateway JSON)
 */

/* Internal index for next-channel iterator */
static size_t plc_channel_index = 0;

/* Last TX index used (for pairing RX). -1 means none yet. */
static int32_t plc_last_tx_index = -1;

/* 返回下一个发送通道（TX），并记录 last_tx_index。
 * out_freq_hz/out_name 必须非 NULL。
 * 返回 0 成功，-1 参数错误。
 */
int plc_get_tx_channel(uint32_t *out_freq_hz, const char **out_name)
{
	if (out_freq_hz == NULL || out_name == NULL) {
		return -1;
	}

	/* Return current channel and advance index (wrap-around) */
	*out_freq_hz = PLC_CHANNEL_FREQS_HZ[plc_channel_index];
	*out_name = PLC_CHANNEL_NAMES[plc_channel_index];

	/* 记录为 last tx，用于后续获取对应的 RX 通道 */
    plc_last_tx_index = (int32_t)plc_channel_index;

	plc_channel_index = (plc_channel_index + 1) % PLC_NUM_CHANNELS;
	if (plc_channel_index == 8)	plc_reset_channel_index();

	return 0;
}

/* 获取与最近一次 plc_get_tx_channel 返回匹配的接收通道（RX）。
 * 若尚未调用 plc_get_channel，则返回第 0 个接收通道。
 * out_freq_hz/out_name 必须非 NULL。
 * 返回 0 成功，-1 参数错误。
 */
int plc_get_rx_channel(uint32_t *out_freq_hz, const char **out_name)
{
    if (out_freq_hz == NULL || out_name == NULL) {
        return -1;
    }

    /* 选择要返回的索引：优先使用最近一次 TX 的索引，否则使用当前 plc_channel_index（作为默认） */
    int32_t idx = plc_last_tx_index;
    if (idx < 0) {
        idx = (int32_t)(plc_channel_index % PLC_RX_NUM_CHANNELS);
    }

    /* 保持索引与 RX 数量对齐（防止数量不一致） */
    idx = idx % (int32_t)PLC_RX_NUM_CHANNELS;
    if (idx < 0) idx += PLC_RX_NUM_CHANNELS;

    *out_freq_hz = PLC_RX_CHANNEL_FREQS_HZ[idx];
    *out_name = PLC_RX_CHANNEL_NAMES[idx];

    return 0;
}


/* 重置发送/接收索引 */
void plc_reset_channel_index(void)
{
    plc_channel_index = 0;
    plc_last_tx_index = -1;
}
