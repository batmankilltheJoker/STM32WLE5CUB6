/* Auto-generated channel list based on gateway global_conf.json
 * radio_0.freq = 471400000
 * radio_1.freq = 475000000
 * offsets: -300000, -100000, 100000, 300000 (for each radio)
 * This file defines absolute receive frequencies (Hz) for the 10 channels.
 */

#ifndef PLC_CHANNEL_SELECT_H
#define PLC_CHANNEL_SELECT_H

#include <stdint.h>
/* ================================================== Tx ================================================================= */
/* Radio base frequencies */
#define PLC_RADIO0_FREQ_HZ 471700000u
#define PLC_RADIO1_FREQ_HZ 475000000u

/* Offsets (Hz) from JSON: chan_multiSF_0..3 and chan_multiSF_4..7 */
#define PLC_CH_IF_0 (-300000)
#define PLC_CH_IF_1 (-100000)
#define PLC_CH_IF_2 (100000)
#define PLC_CH_IF_3 (300000)

/* Computed absolute RX frequencies (Hz) */
#define PLC_CH0_FREQ_HZ (PLC_RADIO0_FREQ_HZ + PLC_CH_IF_0) /* 471400000 */
#define PLC_CH1_FREQ_HZ (PLC_RADIO0_FREQ_HZ + PLC_CH_IF_1) /* 471600000 */
#define PLC_CH2_FREQ_HZ (PLC_RADIO0_FREQ_HZ + PLC_CH_IF_2) /* 471800000 */
#define PLC_CH3_FREQ_HZ (PLC_RADIO0_FREQ_HZ + PLC_CH_IF_3) /* 472000000 */

#define PLC_CH4_FREQ_HZ (PLC_RADIO1_FREQ_HZ + PLC_CH_IF_0) /* 474700000 */
#define PLC_CH5_FREQ_HZ (PLC_RADIO1_FREQ_HZ + PLC_CH_IF_1) /* 474900000 */
#define PLC_CH6_FREQ_HZ (PLC_RADIO1_FREQ_HZ + PLC_CH_IF_2) /* 475100000 */
#define PLC_CH7_FREQ_HZ (PLC_RADIO1_FREQ_HZ + PLC_CH_IF_3) /* 475300000 */

/* Additional explicit channels from JSON */
#define PLC_CH_LORA_STD_FREQ_HZ (PLC_RADIO1_FREQ_HZ - 200000) /* 474800000 */
#define PLC_CH_FSK_FREQ_HZ      (PLC_RADIO1_FREQ_HZ + 300000) /* 475300000 (same as CH7) */

/* Array of the 10 channel frequencies (order matches chan_multiSF_0..3, chan_multiSF_4..7, chan_Lora_std, chan_FSK) */
static const uint32_t PLC_CHANNEL_FREQS_HZ[] = {
    PLC_CH0_FREQ_HZ,
    PLC_CH1_FREQ_HZ,
    PLC_CH2_FREQ_HZ,
    PLC_CH3_FREQ_HZ,
    PLC_CH4_FREQ_HZ,
    PLC_CH5_FREQ_HZ,
    PLC_CH6_FREQ_HZ,
    PLC_CH7_FREQ_HZ,
    /* PLC_CH_LORA_STD_FREQ_HZ, */
    /* PLC_CH_FSK_FREQ_HZ */
};

#define PLC_NUM_CHANNELS (sizeof(PLC_CHANNEL_FREQS_HZ)/sizeof(PLC_CHANNEL_FREQS_HZ[0]))

/* Optional: human-readable names */
static const char* PLC_CHANNEL_NAMES[10] = {
    "chan_multiSF_0",
    "chan_multiSF_1",
    "chan_multiSF_2",
    "chan_multiSF_3",
    "chan_multiSF_4",
    "chan_multiSF_5",
    "chan_multiSF_6",
    "chan_multiSF_7",
    /* "chan_Lora_std", */
    /* "chan_FSK" */
};

/* ================================================== Rx ================================================================= */
/* RX channel frequencies provided (converted from 5014..5053 style -> Hz) */
#define PLC_RX_CH0_FREQ_HZ 501400000u /* 501.4 MHz */
#define PLC_RX_CH1_FREQ_HZ 501600000u /* 501.6 MHz */
#define PLC_RX_CH2_FREQ_HZ 501800000u /* 501.8 MHz */
#define PLC_RX_CH3_FREQ_HZ 502000000u /* 502.0 MHz */
#define PLC_RX_CH4_FREQ_HZ 504700000u /* 504.7 MHz */
#define PLC_RX_CH5_FREQ_HZ 504900000u /* 504.9 MHz */
#define PLC_RX_CH6_FREQ_HZ 505100000u /* 505.1 MHz */
#define PLC_RX_CH7_FREQ_HZ 505300000u /* 505.3 MHz */

/* RX channel arrays */
static const uint32_t PLC_RX_CHANNEL_FREQS_HZ[] = {
    PLC_RX_CH0_FREQ_HZ,
    PLC_RX_CH1_FREQ_HZ,
    PLC_RX_CH2_FREQ_HZ,
    PLC_RX_CH3_FREQ_HZ,
    PLC_RX_CH4_FREQ_HZ,
    PLC_RX_CH5_FREQ_HZ,
    PLC_RX_CH6_FREQ_HZ,
    PLC_RX_CH7_FREQ_HZ
};

#define PLC_RX_NUM_CHANNELS (sizeof(PLC_RX_CHANNEL_FREQS_HZ)/sizeof(PLC_RX_CHANNEL_FREQS_HZ[0]))

static const char* PLC_RX_CHANNEL_NAMES[] = {
    "rx_ch_5014",
    "rx_ch_5016",
    "rx_ch_5018",
    "rx_ch_5020",
    "rx_ch_5047",
    "rx_ch_5049",
    "rx_ch_5051",
    "rx_ch_5053"
};

/* ================================================================================================================== */
/* API: 当发送使用第 N 个通道时，接收使用对应的第 N 个接收通道（索引对应）。
 * 逻辑：
 *   - plc_get_tx_channel() 返回用于发送的下一个 TX 频率，并记录 last_tx_index。
 *   - plc_get_rx_channel() 返回与 last_tx_index 对应的 RX 频率（用于设置接收/下行监听）。
 *   - plc_reset_channel_index() 重置计数器与 last index。
 */


/*
 * 获取下一个通道（轮询顺序）。
 * 返回 0 成功并把频率与名称写入 out 参数；返回 -1 表示参数错误。
 */
int plc_get_tx_channel(uint32_t *out_freq_hz, const char **out_name);

/* 获取与最近一次 plc_get_channel 返回匹配的接收通道（不前进发送索引） */
int plc_get_rx_channel(uint32_t *out_freq_hz, const char **out_name);

/* 重置轮询索引，使下次调用从第 0 个通道开始 */
void plc_reset_channel_index(void);

#endif /* PLC_CHANNEL_SELECT_H */
