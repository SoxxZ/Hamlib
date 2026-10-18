/*
 * system600.c - Yaesu System 600 CAT codec + Hamlib backend.
 *
 * system600.c - (C) soxxz 2026
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Ported strictly from ft600rig/protocol.py (the source of truth for this
 * backend).  Agent-reference section numbers in comments refer to
 * YAESU_SYSTEM600_CAT_PROTOCOL_AGENT_REFERENCE.txt.
 *
 * Two sections: (1) the pure codec - no serial I/O, no Hamlib callbacks,
 * no registration, <stdint.h>/<string.h> only - and (2) below the codec,
 * the Hamlib glue (open/close/freq/mode/VFO/PTT/split/RIT + levels,
 * memory channels and system600_caps).
 * Pure-codec consumers (sys600_veccheck.c) define SYSTEM600_CODEC_ONLY
 * before including this file to compile section 1 alone.
 *
 * Return convention: 0 on success, negative on invalid input - mirroring
 * Hamlib's RIG_OK / negative-error convention without including rig.h.
 */

#include <stdint.h>
#include <string.h>

#ifndef SYSTEM600_CODEC_ONLY
#include <stdlib.h>     /* calloc/free for the priv data (ft817 idiom) */

#include "hamlib/rig.h"
#include "hamlib/rig_state.h"  /* STATE(rig)->priv (ft817_init idiom) */
#include "iofunc.h"
#include "misc.h"
#include "cal.h"        /* rig_raw2val for the provisional STRENGTH mapping */
#endif

#include "system600.h"

/*
 * Some codec helpers are reachable only from the Hamlib glue in section 2
 * below (flag/meter decoding and the command->status mode translator), so
 * they are unused when the vector harness compiles section 1 alone.  Mark
 * them to silence -Wunused-function.  Helpers used only by the harness are
 * guarded with SYSTEM600_CODEC_ONLY instead.
 */
#if defined(__GNUC__) || defined(__clang__)
#define SYS600_UNUSED_FN __attribute__((unused))
#else
#define SYS600_UNUSED_FN
#endif

/*
 * Build the 5-byte wire frame [P4][P3][P2][P1][OPCODE]
 * (protocol.py:132-139 build_command; agent ref sec 4.2).  All arguments
 * are bytes by construction, so no range check is possible or needed.
 */
static void sys600_frame(uint8_t op, uint8_t p1, uint8_t p2, uint8_t p3,
                         uint8_t p4, uint8_t out[SYS600_FRAME_LEN])
{
    out[0] = p4;
    out[1] = p3;
    out[2] = p2;
    out[3] = p1;
    out[4] = op;
}

/*
 * Convert Hz to the four BCD parameter bytes F1..F4 for SET Freq 0Ah/8Ah
 * (protocol.py:150-160 freq_to_bcd_params; agent ref sec 5).  8 BCD digits
 * at 10 Hz resolution, zero padded on the left: F1 = (100s of MHz)(10s of
 * MHz) ... F4 = (100s of Hz)(10s of Hz).  Rounding matches Python's
 * round(freq / 10.0) (half up).  Returns 0, or -1 if the frequency needs
 * more than 8 BCD digits (protocol.py:157-158 ProtocolError).
 */
static int sys600_freq_to_bcd(uint32_t hz, uint8_t out[4])
{
    uint32_t units = hz / 10u;
    uint8_t d[8];
    int i;

    if (hz % 10u >= 5u) {
        units++;
    }
    if (units > 99999999u) {
        return -1;
    }
    for (i = 7; i >= 0; i--) {
        d[i] = (uint8_t)(units % 10u);
        units /= 10u;
    }
    out[0] = (uint8_t)((d[0] << 4) | d[1]);
    out[1] = (uint8_t)((d[2] << 4) | d[3]);
    out[2] = (uint8_t)((d[4] << 4) | d[5]);
    out[3] = (uint8_t)((d[6] << 4) | d[7]);
    return 0;
}

#ifdef SYSTEM600_CODEC_ONLY
/*
 * Inverse of sys600_freq_to_bcd: F1..F4 -> Hz
 * (protocol.py:175-183 bcd_params_to_freq).  Exercised by the vector
 * harness only; the backend never needs to decode a BCD frequency.
 * Returns 0, or -1 if a nibble is not a decimal digit.
 */
static int sys600_bcd_to_freq(const uint8_t in[4], uint32_t *hz)
{
    uint32_t units = 0;
    int i;

    for (i = 0; i < 4; i++) {
        unsigned hi = (in[i] >> 4) & 0x0Fu;
        unsigned lo = in[i] & 0x0Fu;

        if (hi > 9u || lo > 9u) {
            return -1;
        }
        units = units * 100u + hi * 10u + lo;
    }
    *hz = units * 10u;
    return 0;
}
#endif

/*
 * Decode the frequency of a 9-byte VFO/memory sub-record: offsets 1..3 are
 * a 24-bit big-endian BINARY value in 10 Hz units (protocol.py:198-210
 * decode_vfo_record; agent ref sec 12, sec 17.4).  TRAP: BCD is used for
 * SET commands, binary for status records.  The caller guarantees 9 bytes.
 */
static void sys600_vfo_freq(const uint8_t *rec9, uint32_t *hz)
{
    uint32_t units10hz = ((uint32_t)rec9[SYS600_VFO_FREQ] << 16) |
                         ((uint32_t)rec9[SYS600_VFO_FREQ + 1] << 8) |
                         (uint32_t)rec9[SYS600_VFO_FREQ + 2];

    *hz = units10hz * 10u;
}

/*
 * Decode the 19-byte operating/memory record [flag0(1)][Rx(9)][Tx(9)]
 * (agent ref sec 11/17.6).  Yields the Rx and Tx frequencies; d[0] is a
 * status flag (see SYS600_REC_FLAG0) and the Rx mode byte is read directly
 * from SYS600_REC_RX_MODE by the callers that need it.  The caller
 * guarantees 19 bytes.
 *
 * The manual's diagram puts Tx first, but real hardware puts Rx first (8Ah
 * changes only the second sub-record, 0Ah the first).  See system600.h.
 */
static void sys600_decode_record(const uint8_t rec[SYS600_REC_LEN],
                                 uint32_t *rx_hz, uint32_t *tx_hz)
{
    sys600_vfo_freq(rec + SYS600_REC_TX, tx_hz);
    sys600_vfo_freq(rec + SYS600_REC_RX, rx_hz);
}

/*
 * Flag bits of the 3 status flag bytes (protocol.py:253-267; agent ref
 * sec 8.2, sec 10).  Flag1: LOCK / memory-vs-VFO operation.  Flag2: PTT by
 * CAT, scan state, filter narrow bits, J2B sideband.  Flag3: TX state.
 * NOTE: F2 bit5 is labelled "J2B operation selected" in the manual, but on
 * hardware it is the J2B *sideband*: 1 = USB, 0 = LSB (agent ref sec 10/23).
 */
#define SYS600_F1_LOCK         0x01
#define SYS600_F1_MEM          0x20
#define SYS600_F1_VFO          0x80
#define SYS600_F2_PTT_CAT      0x01
#define SYS600_F2_SCAN_PAUSED  0x02
#define SYS600_F2_SCAN         0x04
#define SYS600_F2_J2B_NARROW   0x08
#define SYS600_F2_A1A_NARROW   0x10
#define SYS600_F2_J2B          0x20    /* J2B USB sideband: 1 = USB, 0 = LSB */
#define SYS600_F3_TX_10W       0x02
#define SYS600_F3_TUNER        0x20
#define SYS600_F3_TX           0x80

/*
 * Decoded Read Flags reply (protocol.py:269-304 Flags/decode_flags).
 * Bit fields are 0 or 1; f1/f2/f3 keep the raw bytes for diagnostics.
 */
struct sys600_flags {
    uint8_t lock;
    uint8_t mem;
    uint8_t vfo;
    uint8_t ptt_by_cat;
    uint8_t scan_paused;
    uint8_t scan;
    uint8_t j2b_narrow;
    uint8_t a1a_narrow;
    uint8_t j2b;
    uint8_t tx_10w;
    uint8_t tuner;
    uint8_t transmitting;
    uint8_t f1;
    uint8_t f2;
    uint8_t f3;
};

/*
 * Decode the 5-byte Read Flags (FAh) reply [F1][F2][F3][0x06][0x04]
 * (protocol.py:286-304 decode_flags; radio.py:135-137; agent ref sec 8.2).
 * Returns 0, or -1 if the fixed 06 04 tail does not match (protocol.py
 * only checks the length; the tail check is a deliberate addition).
 */
static SYS600_UNUSED_FN int sys600_decode_flags(const uint8_t reply[SYS600_FLAGS_REPLY_LEN],
                               struct sys600_flags *out)
{
    if (reply[3] != SYS600_FLAGS_REPLY_TAIL0 ||
        reply[4] != SYS600_FLAGS_REPLY_TAIL1) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    out->f1 = reply[0];
    out->f2 = reply[1];
    out->f3 = reply[2];
    out->lock = (reply[0] & SYS600_F1_LOCK) ? 1u : 0u;
    out->mem = (reply[0] & SYS600_F1_MEM) ? 1u : 0u;
    out->vfo = (reply[0] & SYS600_F1_VFO) ? 1u : 0u;
    out->ptt_by_cat = (reply[1] & SYS600_F2_PTT_CAT) ? 1u : 0u;
    out->scan_paused = (reply[1] & SYS600_F2_SCAN_PAUSED) ? 1u : 0u;
    out->scan = (reply[1] & SYS600_F2_SCAN) ? 1u : 0u;
    out->j2b_narrow = (reply[1] & SYS600_F2_J2B_NARROW) ? 1u : 0u;
    out->a1a_narrow = (reply[1] & SYS600_F2_A1A_NARROW) ? 1u : 0u;
    out->j2b = (reply[1] & SYS600_F2_J2B) ? 1u : 0u;
    out->tx_10w = (reply[2] & SYS600_F3_TX_10W) ? 1u : 0u;
    out->tuner = (reply[2] & SYS600_F3_TUNER) ? 1u : 0u;
    out->transmitting = (reply[2] & SYS600_F3_TX) ? 1u : 0u;
    return 0;
}

/*
 * Decode the 5-byte Read Meter (F7h) reply [M][M][M][M][0xF7]: returns the
 * meter byte M (0..255), or -1 if the fixed F7 tail does not match
 * (protocol.py:315-318 decode_meter; radio.py:139-141; agent ref sec 8.3).
 */
static SYS600_UNUSED_FN int sys600_decode_meter(const uint8_t reply[SYS600_METER_REPLY_LEN])
{
    if (reply[4] != SYS600_METER_REPLY_TAIL) {
        return -1;
    }
    return (int)reply[0];
}

/*
 * MODE (0Ch) command code -> status-data mode byte (protocol.py:66-84).
 * TRAP: two different tables - the four J2B command codes (08/09/0E/0F)
 * collapse to the single status code 05.  Index = command code; 0xFF =
 * invalid/reserved.
 */
static const uint8_t sys600_mode_cmd_to_status_tbl[16] = {
    SYS600_MODE_ST_LSB,     /* 0x0: A3J-LSB */
    SYS600_MODE_ST_USB,     /* 0x1: A3J-USB */
    SYS600_MODE_ST_CW_W,    /* 0x2: A1A-W */
    SYS600_MODE_ST_CW_N,    /* 0x3: A1A-N */
    SYS600_MODE_ST_AM,      /* 0x4: A3E-W */
    0xFF,                   /* 0x5: invalid */
    0xFF,                   /* 0x6: invalid */
    0xFF,                   /* 0x7: invalid */
    SYS600_MODE_ST_J2B,     /* 0x8: J2B-LSB-W */
    SYS600_MODE_ST_J2B,     /* 0x9: J2B-USB-W */
    0xFF,                   /* 0xA: invalid */
    0xFF,                   /* 0xB: invalid */
    0xFF,                   /* 0xC: invalid */
    0xFF,                   /* 0xD: invalid */
    SYS600_MODE_ST_J2B,     /* 0xE: J2B-LSB-N */
    SYS600_MODE_ST_J2B,     /* 0xF: J2B-USB-N */
};

#ifdef SYSTEM600_CODEC_ONLY
/*
 * Status-data mode byte -> MODE (0Ch) command code.  The status byte cannot
 * distinguish J2B sideband (it lives in Flag2), so J2B decodes to the
 * J2B-USB-W default, matching protocol.py:115-122 STATUS_MODE_TO_TOKEN
 * (status J2B -> PKTUSB).  Index = status code.
 */
static const uint8_t sys600_mode_status_to_cmd_tbl[6] = {
    SYS600_MODE_CMD_LSB,        /* 0: LSB */
    SYS600_MODE_CMD_USB,        /* 1: USB */
    SYS600_MODE_CMD_CW_W,       /* 2: CW wide */
    SYS600_MODE_CMD_CW_N,       /* 3: CW narrow */
    SYS600_MODE_CMD_AM,         /* 4: AM wide */
    SYS600_MODE_CMD_J2B_USB_W,  /* 5: J2B -> J2B-USB-W default */
};
#endif

/*
 * Translate a MODE (0Ch) command code to the status-data mode byte.
 * Returns 0, or -1 for an invalid/reserved command code.
 */
static SYS600_UNUSED_FN int sys600_mode_cmd_to_status(uint8_t cmd, uint8_t *status)
{
    if ((unsigned)cmd >= sizeof(sys600_mode_cmd_to_status_tbl) ||
        sys600_mode_cmd_to_status_tbl[cmd] == 0xFF) {
        return -1;
    }
    *status = sys600_mode_cmd_to_status_tbl[cmd];
    return 0;
}

#ifdef SYSTEM600_CODEC_ONLY
/*
 * Translate a status-data mode byte to a MODE (0Ch) command code.
 * Exercised by the vector harness only; the backend's get_mode uses the
 * status byte directly.  Returns 0, or -1 for an out-of-table status code.
 */
static int sys600_mode_status_to_cmd(uint8_t status, uint8_t *cmd)
{
    if ((unsigned)status >= sizeof(sys600_mode_status_to_cmd_tbl)) {
        return -1;
    }
    *cmd = sys600_mode_status_to_cmd_tbl[status];
    return 0;
}
#endif


/*
 * =====================================================================
 * Hamlib glue: open/close, set/get freq, minimal registration.
 * Port semantics come from ft600rig/radio.py; call idioms follow the
 * ft817 backend (rigs/yaesu/ft817.c).
 * =====================================================================
 */

#ifndef SYSTEM600_CODEC_ONLY

/*
 * Mode set advertised in the range/tuning-step lists.  CAT has no CWR, so
 * CW only (wide/narrow are filter variants of the same RIG_MODE_CW).
 */
#define SYSTEM600_MODES (RIG_MODE_LSB | RIG_MODE_USB | RIG_MODE_CW | \
                         RIG_MODE_AM | RIG_MODE_PKTLSB | RIG_MODE_PKTUSB)

/* Memory channels are numbered sequentially 1..100 (agent ref sec 6.1). */
#define SYS600_MEM_CH_MIN 1
#define SYS600_MAX_MEM_CH 100

static int sys600_read_status(RIG *rig, uint8_t rec[SYS600_REC_LEN]);
static int sys600_read_flags(RIG *rig, struct sys600_flags *flags);
static int system600_init(RIG *rig);
static int system600_cleanup(RIG *rig);
static int system600_open(RIG *rig);
static int system600_close(RIG *rig);
static int system600_set_freq(RIG *rig, vfo_t vfo, freq_t freq);
static int system600_get_freq(RIG *rig, vfo_t vfo, freq_t *freq);
static int system600_set_mode(RIG *rig, vfo_t vfo, rmode_t mode, pbwidth_t width);
static int system600_get_mode(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width);
static int system600_set_vfo(RIG *rig, vfo_t vfo);
static int system600_get_vfo(RIG *rig, vfo_t *vfo);
static int system600_set_ptt(RIG *rig, vfo_t vfo, ptt_t ptt);
static int system600_get_ptt(RIG *rig, vfo_t vfo, ptt_t *ptt);
static int system600_set_split_vfo(RIG *rig, vfo_t vfo, split_t split, vfo_t tx_vfo);
static int system600_get_split_vfo(RIG *rig, vfo_t vfo, split_t *split, vfo_t *tx_vfo);
static int system600_set_split_freq(RIG *rig, vfo_t vfo, freq_t freq);
static int system600_get_split_freq(RIG *rig, vfo_t vfo, freq_t *freq);
static int system600_set_rit(RIG *rig, vfo_t vfo, shortfreq_t rit);
static int system600_get_rit(RIG *rig, vfo_t vfo, shortfreq_t *rit);
static int system600_vfo_op(RIG *rig, vfo_t vfo, vfo_op_t op);
static int system600_get_func(RIG *rig, vfo_t vfo, setting_t func, int *status);
static int system600_get_level(RIG *rig, vfo_t vfo, setting_t level, value_t *val);
static int system600_set_mem(RIG *rig, vfo_t vfo, int ch);
static int system600_get_mem(RIG *rig, vfo_t vfo, int *ch);
static int system600_set_channel(RIG *rig, vfo_t vfo, const channel_t *chan);
static int system600_get_channel(RIG *rig, vfo_t vfo, channel_t *chan, int read_only);
static const char *system600_get_info(RIG *rig);

/*
 * Per-instance private data, allocated in system600_init and released in
 * system600_cleanup (ft817_init/cleanup idiom: rigs/yaesu/ft817.c:622-646,
 * STATE(rig)->priv).  The J2B sideband is NOT cached - it is read back from
 * Flag2 bit5 on every get_mode (hardware-verified; agent ref sec 10/23).
 * rit_* caches the last RIT offset (the carrier offset is not echoed back).
 */
struct sys600_priv {
    int rit_hz;                 /* cached RIT offset, Hz */
    int rit_valid;              /* rit_hz holds a value */
};

/*
 * Generic Status Update (10h) request/response: flush stale input, write the
 * 5-byte block with the U selector in P1 and CH in P4 (significant only for
 * U=4 - agent ref sec 6, sec 14), then read exactly `expect` bytes.  Mirrors
 * radio.py:115-132 (_query).
 *
 * The reply is read with read_block, not read_string: read_string is a
 * text-line reader and its ASCII post-processing strips a leading ';'
 * (0x3B) from the buffer (iofunc.c:1492-1502), which would corrupt the
 * record whenever byte 0 - the current memory channel - is 59, and it
 * mutates the process-global static minlen (iofunc.c:1432).  radio.py
 * uses a raw read(19); read_block is the matching primitive.
 *
 * Returns RIG_OK, -RIG_ETIMEOUT if the radio did not answer within the
 * port timeout (read_block's return on timeout/short), -RIG_EPROTO for
 * any short/wrong-length reply (defense in depth), or the write_block
 * error verbatim.
 */
static int sys600_status_query(RIG *rig, uint8_t u, uint8_t ch,
                               uint8_t *buf, int expect)
{
    uint8_t frame[SYS600_FRAME_LEN];
    int ret;

    /* P1 = U selector, P4 = CH (U=4), P2/P3 dummy (protocol.py:355-362). */
    sys600_frame(SYS600_OP_STATUS, u, 0, 0, ch, frame);

    /* Drop stale bytes so we stay aligned (radio.py:117 reset_input_buffer). */
    rig_flush(RIGPORT(rig));

    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    if (ret != RIG_OK)
    {
        return ret;
    }

    ret = read_block(RIGPORT(rig), buf, expect);
    if (ret < 0)
    {
        return ret; /* -RIG_ETIMEOUT (a timeout is a timeout, even partial) */
    }

    if (ret != expect)
    {
        rig_debug(RIG_DEBUG_ERR,
                  "%s: short status reply (U=%u): expected %d bytes, got %d\n",
                  __func__, (unsigned)u, expect, ret);
        return -RIG_EPROTO;
    }

    return RIG_OK;
}

/*
 * Status Update (10h) with U=2: request, then read exactly the 19-byte
 * operating record.  Never U=0 (1941 bytes).
 */
static int sys600_read_status(RIG *rig, uint8_t rec[SYS600_REC_LEN])
{
    uint8_t buf[SYS600_STATUS_SIZE_U2];
    int ret;

    ret = sys600_status_query(rig, 2, 0, buf, SYS600_STATUS_SIZE_U2);
    if (ret != RIG_OK)
    {
        return ret;
    }

    memcpy(rec, buf, SYS600_REC_LEN);
    return RIG_OK;
}

/*
 * Status Update (10h) with U=4 CH: the 19-byte record of one memory channel
 * (agent ref sec 14: U=4, CH = 1..100).  Non-invasive: reading a channel does
 * not recall it or otherwise touch the radio's current state.
 */
static int sys600_read_mem_record(RIG *rig, int ch, uint8_t rec[SYS600_REC_LEN])
{
    uint8_t buf[SYS600_STATUS_SIZE_U4];
    int ret;

    ret = sys600_status_query(rig, 4, (uint8_t)ch, buf, SYS600_STATUS_SIZE_U4);
    if (ret != RIG_OK)
    {
        return ret;
    }

    memcpy(rec, buf, SYS600_REC_LEN);
    return RIG_OK;
}

/*
 * Read the current memory number: Status Update U=1 (1 byte).
 *
 * Hardware (System 600, 2026-10): this byte is the 0-based memory channel
 * index (channel 42 -> 0x29), i.e. channel = byte + 1.  The byte at d[0] of
 * the U=2 record, which the manual calls the "memory number", is in fact a
 * status flag (0x20 in VFO mode, 0xA0 in memory mode) and never carries the
 * channel; the manual itself warns not to rely on it (agent ref sec 17.5).
 * Returns RIG_OK, -RIG_ETIMEOUT, -RIG_EPROTO on a short reply, or the
 * write_block error verbatim.
 */
static int sys600_read_memory_number(RIG *rig, uint8_t *num)
{
    uint8_t buf[SYS600_STATUS_SIZE_U1];
    int ret;

    ret = sys600_status_query(rig, 1, 0, buf, SYS600_STATUS_SIZE_U1);
    if (ret != RIG_OK)
    {
        return ret;
    }

    *num = buf[0];
    return RIG_OK;
}

/*
 * Read Flags (FAh): flush, write the 5-byte request, read exactly the
 * 5-byte reply, decode with the codec's tail-checked decoder.  Mirrors
 * radio.py:135-137 (read_flags via _query) with the read_block primitive
 * (see sys600_read_status for why read_string is never used on binary).
 * Returns RIG_OK, -RIG_ETIMEOUT on timeout, -RIG_EPROTO on a short reply
 * or a bad 06 04 tail, or the write_block error verbatim.
 */
static int sys600_read_flags(RIG *rig, struct sys600_flags *flags)
{
    uint8_t frame[SYS600_FRAME_LEN];
    uint8_t buf[SYS600_FLAGS_REPLY_LEN];
    int ret;

    sys600_frame(SYS600_OP_READ_FLAGS, 0, 0, 0, 0, frame);

    /* Drop stale bytes so we stay aligned (radio.py:117 reset_input_buffer). */
    rig_flush(RIGPORT(rig));

    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    if (ret != RIG_OK)
    {
        return ret;
    }

    ret = read_block(RIGPORT(rig), buf, SYS600_FLAGS_REPLY_LEN);
    if (ret < 0)
    {
        return ret;
    }

    if (ret != SYS600_FLAGS_REPLY_LEN)
    {
        rig_debug(RIG_DEBUG_ERR,
                  "%s: short flags reply: expected %d bytes, got %d\n",
                  __func__, SYS600_FLAGS_REPLY_LEN, ret);
        return -RIG_EPROTO;
    }

    if (sys600_decode_flags(buf, flags) != 0)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: bad flags reply tail\n", __func__);
        return -RIG_EPROTO;
    }

    return RIG_OK;
}

/*
 * Read Meter (F7h): flush, write the 5-byte request, read exactly the 5-byte
 * reply [M][M][M][M][0xF7] and decode byte 0 (agent ref sec 8.3; radio.py:139-141
 * read_meter via _query).  The reply is binary, so read_block is used, never
 * read_string (see sys600_read_status).  Returns RIG_OK with *meter in 0..255,
 * -RIG_ETIMEOUT on timeout, -RIG_EPROTO on a short reply or a bad F7 tail, or
 * the write_block error verbatim.
 */
static int sys600_read_meter(RIG *rig, int *meter)
{
    uint8_t frame[SYS600_FRAME_LEN];
    uint8_t buf[SYS600_METER_REPLY_LEN];
    int ret;

    sys600_frame(SYS600_OP_READ_METER, 0, 0, 0, 0, frame);

    rig_flush(RIGPORT(rig));

    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    if (ret != RIG_OK)
    {
        return ret;
    }

    ret = read_block(RIGPORT(rig), buf, SYS600_METER_REPLY_LEN);
    if (ret < 0)
    {
        return ret;
    }

    if (ret != SYS600_METER_REPLY_LEN)
    {
        rig_debug(RIG_DEBUG_ERR,
                  "%s: short meter reply: expected %d bytes, got %d\n",
                  __func__, SYS600_METER_REPLY_LEN, ret);
        return -RIG_EPROTO;
    }

    ret = sys600_decode_meter(buf);
    if (ret < 0)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: bad meter reply tail\n", __func__);
        return -RIG_EPROTO;
    }

    *meter = ret;
    return RIG_OK;
}

/*
 * Init: allocate the per-instance priv data (ft817_init idiom,
 * rigs/yaesu/ft817.c:622-636).  calloc zeroes everything: j2b_sideband
 * starts "never set" so get_mode reports PKTUSB until a PKTLSB/PKTUSB
 * set_mode teaches us the sideband.
 */
static int system600_init(RIG *rig)
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s: called, version %s\n", __func__,
              rig->caps->version);

    if ((STATE(rig)->priv = calloc(1, sizeof(struct sys600_priv))) == NULL)
    {
        return -RIG_ENOMEM;
    }

    return RIG_OK;
}

/*
 * Cleanup: release the priv data (ft817_cleanup idiom,
 * rigs/yaesu/ft817.c:638-646).  rig_cleanup's return code is ignored by
 * the frontend.
 */
static int system600_cleanup(RIG *rig)
{
    rig_debug(RIG_DEBUG_VERBOSE, "%s: called\n", __func__);

    free(STATE(rig)->priv);
    STATE(rig)->priv = NULL;

    return RIG_OK;
}

/*
 * Open: the serial framing (4800 8N2) is applied by the frontend from the
 * model-default caps fields (notes sec 1: serial_* caps, frg9600.c:55-70),
 * not here - backends do not poke port params in rig_open.  Sync means:
 * flush whatever a previous session left behind (radio.py:82
 * reset_input_buffer) and exchange exactly one U=2 status request/response.
 */
static int system600_open(RIG *rig)
{
    uint8_t rec[SYS600_REC_LEN];
    int ret;

    /* radio.py:82 - drop anything left over so reads start aligned. */
    rig_flush(RIGPORT(rig));

    ret = sys600_read_status(rig, rec);
    if (ret != RIG_OK)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: status sync failed: %d\n", __func__, ret);
        return ret;
    }

    return RIG_OK;
}

/*
 * Close: best effort - never leave the transmitter keyed on exit
 * (radio.py:85-93 close always sends PTT-off and swallows errors).
 * The frontend ignores the rig_close return code (notes sec 4).
 */
static int system600_close(RIG *rig)
{
    uint8_t frame[SYS600_FRAME_LEN];
    int ret;

    sys600_frame(SYS600_OP_PTT, 0, 0, 0, 0, frame); /* T=0: transmitter off */
    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    if (ret != RIG_OK)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: PTT-off safety frame failed: %d\n",
                  __func__, ret);
    }

    return RIG_OK;
}

/*
 * Set Rx/operating frequency (0Ah, BCD) with read-back verification
 * (radio.py:144-155): this radio silently ignores out-of-range commands,
 * so a set that does not stick is reported as -RIG_EPROTO instead of
 * failing silently.
 */
static int system600_set_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
    uint8_t frame[SYS600_FRAME_LEN];
    uint8_t bcd[4];
    uint8_t rec[SYS600_REC_LEN];
    uint32_t rx_hz, tx_hz;
    int ret;

    (void)vfo; /* 0Ah always targets the current operating VFO */

    /* 100 kHz .. 30 MHz (radio.py:146-150, protocol span). */
    if (!(freq >= (freq_t)SYS600_FREQ_MIN_HZ &&
            freq <= (freq_t)SYS600_FREQ_MAX_HZ))
    {
        return -RIG_EINVAL;
    }

    if (sys600_freq_to_bcd((uint32_t)freq, bcd) != 0)
    {
        return -RIG_EINVAL;
    }

    /* Wire order F4,F3,F2,F1,opcode (protocol.py:163-166 set_freq_frame). */
    sys600_frame(SYS600_OP_SET_RX_FREQ, bcd[0], bcd[1], bcd[2], bcd[3], frame);
    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    if (ret != RIG_OK)
    {
        return ret;
    }

    /* Read-back confirm: the Rx sub-record must now match (radio.py verify). */
    ret = sys600_read_status(rig, rec);
    if (ret != RIG_OK)
    {
        return ret;
    }

    sys600_decode_record(rec, &rx_hz, &tx_hz);
    if (rx_hz != (uint32_t)freq)
    {
        rig_debug(RIG_DEBUG_ERR,
                  "%s: set did not stick: asked %.0f Hz, radio reports %u Hz\n",
                  __func__, (double)freq, (unsigned)rx_hz);
        return -RIG_EPROTO;
    }

    return RIG_OK;
}

/*
 * Get Rx/operating frequency: Status Update U=2 (19 bytes), frequency of
 * the Rx sub-record d[1..9] (binary 10 Hz units - see codec).  Never U=0.
 */
static int system600_get_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
    uint8_t rec[SYS600_REC_LEN];
    uint32_t hz;
    int ret;

    (void)vfo; /* single operating frequency, VFO-agnostic */

    ret = sys600_read_status(rig, rec);
    if (ret != RIG_OK)
    {
        return ret;
    }

    sys600_vfo_freq(rec + SYS600_REC_RX, &hz);
    *freq = (freq_t)hz;
    return RIG_OK;
}

/*
 * =====================================================================
 * Mode / VFO / PTT callbacks.  Semantics from ft600rig/radio.py
 * (set/get_mode 189-222, set/get_ptt 224-231, set/get_vfo 233-249).
 * =====================================================================
 */

/*
 * Choose between the radio's two filter positions for a mode.  The panel has
 * exactly two: a wide and a narrow passband (CW 500/250 Hz, J2B 2400/500 Hz -
 * hardware-verified 2026-10-17, agent ref sec 23b).  A request below the
 * midpoint picks the narrow position; width <= 0 (RIG_PASSBAND_NORMAL) keeps
 * the radio default, which is the wide position.
 */
static int sys600_narrow(pbwidth_t width, pbwidth_t wide_hz, pbwidth_t narrow_hz)
{
    return width > 0 && width < (wide_hz + narrow_hz) / 2;
}

/*
 * Map a Hamlib mode + width to a MODE (0Ch) command byte
 * (radio.py set_mode).  The radio has no CWR/RTTY/... modes: any rmode
 * outside the CAT table is rejected with -RIG_EINVAL (frg9600.c:189
 * idiom per notes sec 4).
 */
static int sys600_rmode_to_cmd(rmode_t mode, pbwidth_t width, uint8_t *cmd)
{
    /* The CAT MODE command carries a *filter position*, not a passband: CW is
     * 500 Hz (A1A-W) or 250 Hz (A1A-N), J2B is 2400 Hz (wide) or 500 Hz
     * (narrow).  Pick the closer of the two so that asking for 500 Hz CW
     * selects the 500 Hz filter instead of silently landing on 250 Hz.
     * width <= 0 (RIG_PASSBAND_NORMAL) keeps the radio default = wide. */
    switch (mode)
    {
    case RIG_MODE_LSB:
        *cmd = SYS600_MODE_CMD_LSB;
        return 0;

    case RIG_MODE_USB:
        *cmd = SYS600_MODE_CMD_USB;
        return 0;

    case RIG_MODE_CW:
        *cmd = sys600_narrow(width, SYS600_PASSBAND_CW_W_HZ,
                             SYS600_PASSBAND_CW_N_HZ) ?
               SYS600_MODE_CMD_CW_N : SYS600_MODE_CMD_CW_W;
        return 0;

    case RIG_MODE_AM:
        *cmd = SYS600_MODE_CMD_AM;
        return 0;

    case RIG_MODE_PKTLSB:
        *cmd = sys600_narrow(width, SYS600_PASSBAND_J2B_W_HZ,
                             SYS600_PASSBAND_J2B_N_HZ) ?
               SYS600_MODE_CMD_J2B_LSB_N : SYS600_MODE_CMD_J2B_LSB_W;
        return 0;

    case RIG_MODE_PKTUSB:
        *cmd = sys600_narrow(width, SYS600_PASSBAND_J2B_W_HZ,
                             SYS600_PASSBAND_J2B_N_HZ) ?
               SYS600_MODE_CMD_J2B_USB_N : SYS600_MODE_CMD_J2B_USB_W;
        return 0;

    default:
        return -1;
    }
}

/*
 * Set operating mode (0Ch) with read-back verification
 * (radio.py:193-196: re-read the status mode byte; the radio silently
 * ignores out-of-range commands, so a set that does not stick is
 * -RIG_EPROTO).  The status byte cannot encode the J2B sideband, so the
 * verify compares mode-family status codes (the J2B wide/narrow/sideband
 * command codes all collapse to status 05 - codec trap, protocol.py:66-84).
 * On a successful PKTLSB/PKTUSB set the sideband is cached in priv data
 * for get_mode (status 5 ambiguity, agent ref sec 17.10).
 */
static int system600_set_mode(RIG *rig, vfo_t vfo, rmode_t mode, pbwidth_t width)
{
    uint8_t frame[SYS600_FRAME_LEN];
    uint8_t cmd, want_status, rec[SYS600_REC_LEN];
    struct sys600_flags flags;
    int ret;

    (void)vfo; /* 0Ch always targets the current operating mode */

    if (sys600_rmode_to_cmd(mode, width, &cmd) != 0)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: unsupported mode 0x%x\n", __func__,
                  (unsigned)mode);
        return -RIG_EINVAL;
    }

    sys600_frame(SYS600_OP_MODE, cmd, 0, 0, 0, frame);
    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    if (ret != RIG_OK)
    {
        return ret;
    }

    ret = sys600_read_status(rig, rec);
    if (ret != RIG_OK)
    {
        return ret;
    }

    /* cmd is always table-valid here; belt and braces. */
    if (sys600_mode_cmd_to_status(cmd, &want_status) != 0)
    {
        return -RIG_EINVAL;
    }

    /* The status byte must match the requested mode GROUP.  Hardware
     * (2026-10-17, agent ref sec 23b): the radio answers 02h for BOTH CW
     * widths - A1A-N switches the narrow filter but leaves the mode byte at
     * 02h - so both CW bytes are accepted here and the width is confirmed
     * from Flag2 below.  The manual's separate "03h = A1A-N" status code was
     * never observed on this radio.  All four J2B command codes report the
     * single status byte 05h. */
    if (rec[SYS600_REC_RX_MODE] != want_status &&
            !(want_status == SYS600_MODE_ST_CW_W &&
              rec[SYS600_REC_RX_MODE] == SYS600_MODE_ST_CW_N))
    {
        rig_debug(RIG_DEBUG_ERR,
                  "%s: set did not stick: asked status mode 0x%x, radio reports 0x%x\n",
                  __func__, (unsigned)want_status,
                  (unsigned)rec[SYS600_REC_RX_MODE]);
        return -RIG_EPROTO;
    }

    /* Width/sideband are NOT in the status byte for the modes that have
     * variants: CW wide/narrow is Flag2 bit4, J2B width is bit3 and J2B
     * sideband is bit5.  Confirm them, otherwise a request the radio ignores
     * would be reported as success (the failure mode that hid the J2B
     * sideband bug). */
    switch (cmd)
    {
    case SYS600_MODE_CMD_CW_W:
    case SYS600_MODE_CMD_CW_N:
    case SYS600_MODE_CMD_J2B_LSB_W:
    case SYS600_MODE_CMD_J2B_USB_W:
    case SYS600_MODE_CMD_J2B_LSB_N:
    case SYS600_MODE_CMD_J2B_USB_N:
        if (sys600_read_flags(rig, &flags) != RIG_OK)
        {
            /* Cannot confirm the variant; the mode byte already matched. */
            return RIG_OK;
        }

        if (cmd == SYS600_MODE_CMD_CW_W || cmd == SYS600_MODE_CMD_CW_N)
        {
            uint8_t want_narrow = (cmd == SYS600_MODE_CMD_CW_N);

            if (flags.a1a_narrow != want_narrow)
            {
                rig_debug(RIG_DEBUG_ERR,
                          "%s: CW filter did not stick: asked %s, Flag2 bit4=%u\n",
                          __func__, want_narrow ? "A1A-N" : "A1A-W",
                          (unsigned)flags.a1a_narrow);
                return -RIG_EPROTO;
            }
        }
        else
        {
            uint8_t want_usb = (cmd == SYS600_MODE_CMD_J2B_USB_W ||
                                cmd == SYS600_MODE_CMD_J2B_USB_N);
            uint8_t want_narrow = (cmd == SYS600_MODE_CMD_J2B_LSB_N ||
                                   cmd == SYS600_MODE_CMD_J2B_USB_N);

            if (flags.j2b != want_usb || flags.j2b_narrow != want_narrow)
            {
                rig_debug(RIG_DEBUG_ERR,
                          "%s: J2B did not stick: asked %s, Flag2 bit5(sideband)=%u "
                          "bit3(narrow)=%u\n",
                          __func__, want_usb ? "USB" : "LSB",
                          (unsigned)flags.j2b, (unsigned)flags.j2b_narrow);
                return -RIG_EPROTO;
            }
        }

        break;

    default:
        break;
    }

    return RIG_OK;
}

/*
 * Get operating mode + passband.  The status mode byte selects the mode
 * family (protocol.py:115-122 STATUS_MODE_TO_TOKEN); for status 5 (J2B) the
 * sideband comes from Flag2 bit5 (agent ref sec 10 - see the case below).
 * Passband comes from the Flag2 narrow bits exactly as radio.py:209-221:
 * CW -> 250 if A1A-narrow else 500; J2B -> 500 if J2B-narrow else 2400;
 * LSB/USB -> 2400; AM -> 6000.  A flags read failure falls back to the wide
 * values (radio.py:212-213 swallows RadioError and uses wide defaults).
 */
static int system600_get_mode(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width)
{
    uint8_t rec[SYS600_REC_LEN];
    struct sys600_flags flags;
    int have_flags;
    uint8_t st;
    int ret;

    (void)vfo; /* single operating mode, VFO-agnostic */

    ret = sys600_read_status(rig, rec);
    if (ret != RIG_OK)
    {
        return ret;
    }

    st = rec[SYS600_REC_RX_MODE];

    switch (st)
    {
    case SYS600_MODE_ST_LSB:
        *mode = RIG_MODE_LSB;
        *width = SYS600_PASSBAND_LSB_HZ;
        return RIG_OK;

    case SYS600_MODE_ST_USB:
        *mode = RIG_MODE_USB;
        *width = SYS600_PASSBAND_USB_HZ;
        return RIG_OK;

    case SYS600_MODE_ST_AM:
        *mode = RIG_MODE_AM;
        *width = SYS600_PASSBAND_AM_HZ;
        return RIG_OK;

    case SYS600_MODE_ST_CW_W:
    case SYS600_MODE_ST_CW_N:
        *mode = RIG_MODE_CW;
        break;

    case SYS600_MODE_ST_J2B:
        /* The status mode byte says only "J2B"; the sideband is Flag2 bit5
         * (agent ref sec 10: bits 3 and 5 carry the data-mode width/sideband).
         * Hardware-verified 2026-10-16: bit5 toggles with the sideband and
         * nothing else does - 1 = USB (PKTUSB), 0 = LSB (PKTLSB), confirmed
         * against the front-panel "J2B LSB" display.  The manual's label "J2B
         * operation selected" is misleading: the radio clears bit5 in J2B-LSB.
         * If the flags read fails the sideband is unknown -> PKTUSB default. */
        if (sys600_read_flags(rig, &flags) != RIG_OK)
        {
            *mode = RIG_MODE_PKTUSB;
            *width = SYS600_PASSBAND_J2B_W_HZ;
            return RIG_OK;
        }

        *mode = flags.j2b ? RIG_MODE_PKTUSB : RIG_MODE_PKTLSB;
        *width = flags.j2b_narrow ? SYS600_PASSBAND_J2B_N_HZ
                                  : SYS600_PASSBAND_J2B_W_HZ;
        return RIG_OK;

    default:
        /* Unknown status mode byte: radio.py:206 falls back to USB via
         * STATUS_MODE_TO_TOKEN.get(status_mode, "USB") instead of raising;
         * match that (never error on decode) but shout in the log. */
        rig_debug(RIG_DEBUG_ERR, "%s: unknown status mode byte 0x%x, "
                  "reporting USB\n", __func__, (unsigned)st);
        *mode = RIG_MODE_USB;
        *width = SYS600_PASSBAND_USB_HZ;
        return RIG_OK;
    }

    /* Only CW falls through to here (J2B returns above; LSB/USB/AM return in
     * their own cases).  Passband from Flag2 bit4, exactly as radio.py:215. */
    have_flags = (sys600_read_flags(rig, &flags) == RIG_OK);
    *width = (have_flags && flags.a1a_narrow) ?
             SYS600_PASSBAND_CW_N_HZ : SYS600_PASSBAND_CW_W_HZ;

    return RIG_OK;
}

/*
 * Map a *stored* mode byte (a memory record's Rx/Tx sub-record offset 6) to a
 * Hamlib mode + passband.  Unlike the live operating record, a memory channel
 * cannot carry filter variants: hardware-verified 2026-10-17, A1A-N stores
 * 02h (same as A1A-W) and every J2B code stores 05h, and bytes 4/5/7/8 are
 * always zero (agent ref sec 23c).  So CW reports the wide position, J2B
 * reports the wide PKTUSB default (the same fallback get_mode uses when the
 * sideband flag cannot be read), and no width is invented for anything else.
 * Returns 0, or -1 for a mode byte that is not in the status table.
 */
static int sys600_record_mode(uint8_t st, rmode_t *mode, pbwidth_t *width)
{
    switch (st)
    {
    case SYS600_MODE_ST_LSB:
        *mode = RIG_MODE_LSB;
        *width = SYS600_PASSBAND_LSB_HZ;
        return 0;

    case SYS600_MODE_ST_USB:
        *mode = RIG_MODE_USB;
        *width = SYS600_PASSBAND_USB_HZ;
        return 0;

    case SYS600_MODE_ST_CW_W:
        *mode = RIG_MODE_CW;
        *width = SYS600_PASSBAND_CW_W_HZ;
        return 0;

    case SYS600_MODE_ST_CW_N:
        /* The manual documents a separate A1A-N status code; the live radio
         * never reports it and stores 02h instead.  Honour it if it appears. */
        *mode = RIG_MODE_CW;
        *width = SYS600_PASSBAND_CW_N_HZ;
        return 0;

    case SYS600_MODE_ST_AM:
        *mode = RIG_MODE_AM;
        *width = SYS600_PASSBAND_AM_HZ;
        return 0;

    case SYS600_MODE_ST_J2B:
        /* Sideband/width are not stored (see above) -> wide USB default, the
         * same choice get_mode makes when the flags read fails. */
        *mode = RIG_MODE_PKTUSB;
        *width = SYS600_PASSBAND_J2B_W_HZ;
        return 0;

    default:
        rig_debug(RIG_DEBUG_ERR, "%s: unknown record mode byte 0x%x, "
                  "reporting USB\n", __func__, (unsigned)st);
        *mode = RIG_MODE_USB;
        *width = SYS600_PASSBAND_USB_HZ;
        return 0;
    }
}

/*
 * Set VFO: the CAT protocol has a single operating VFO. RIG_VFO_A selects the
 * VFO (send Recall VFO 05h). RIG_VFO_MEM cannot be selected without a channel
 * (02h) → return -RIG_ENIMPL (honest error rather than silent no-op).
 * Anything else is -RIG_ENIMPL.
 */
static int system600_set_vfo(RIG *rig, vfo_t vfo)
{
    uint8_t frame[SYS600_FRAME_LEN];

    if (vfo == RIG_VFO_A)
    {
        /* Recall VFO (05h): return to the last-used VFO */
        sys600_frame(SYS600_OP_RECALL_VFO, 0, 0, 0, 0, frame);
        return write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    }

    return -RIG_ENIMPL;
}

/*
 * Get VFO: Read Flags (FAh), Flag1 memory bit -> RIG_VFO_MEM,
 * else RIG_VFO_A (radio.py:246-249 get_vfo).
 */
static int system600_get_vfo(RIG *rig, vfo_t *vfo)
{
    struct sys600_flags flags;
    int ret;

    ret = sys600_read_flags(rig, &flags);
    if (ret != RIG_OK)
    {
        return ret;
    }

    *vfo = flags.mem ? RIG_VFO_MEM : RIG_VFO_A;
    return RIG_OK;
}

/*
 * Set PTT (0Fh, P1=T: 1 = on / 0 = off; radio.py:224-227 set_ptt, protocol
 * ptt_frame; agent ref sec 6(7) and the wire table sec 7).
 *
 *   PTT on  -> 00 00 00 01 0F
 *   PTT off -> 00 00 00 00 0F
 *
 * The radio returns NO reply to 0Fh, so this function must never read back:
 * a read would block for the whole SYS600_READ_TIMEOUT_MS with the
 * transmitter still keyed.  PTT-ON is fully supported: ASK THE USER before
 * keying (agent ref sec 6(7)); the OFF path is always sent on error and on
 * close (system600_close), so any error leaves the rig unkeyed.
 */
static int system600_set_ptt(RIG *rig, vfo_t vfo, ptt_t ptt)
{
    uint8_t frame[SYS600_FRAME_LEN];
    uint8_t off[SYS600_FRAME_LEN];
    uint8_t t;
    int ret;

    (void)vfo; /* 0F always targets the current operating state */

    switch (ptt)
    {
    case RIG_PTT_OFF:
        t = 0;
        break;

    case RIG_PTT_ON:
    case RIG_PTT_ON_MIC:  /* rig_set_ptt folds these to ON for RIG_PTT_RIG */
    case RIG_PTT_ON_DATA: /* (agent ref sec 18 R8: one CAT PTT, no mic/data) */
        t = 1;
        break;

    default:
        return -RIG_EINVAL;
    }

    /* Wire order is P4 P3 P2 P1 OPCODE, so P1=T lands just before the opcode:
     * sys600_frame(OP_PTT, t, 0, 0, 0) == 00 00 00 t 0F. */
    sys600_frame(SYS600_OP_PTT, t, 0, 0, 0, frame);
    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);

    /* Fail-safe (agent ref sec 18 R8): if keying failed, make one attempt to
     * drop PTT and report the original error. */
    if (ret != RIG_OK && t != 0)
    {
        sys600_frame(SYS600_OP_PTT, 0, 0, 0, 0, off);
        (void)write_block(RIGPORT(rig), off, SYS600_FRAME_LEN);
    }

    return ret;
}

/*
 * Set Split VFO: RIG_SPLIT_OFF -> Rx->Tx copy (85h). RIG_SPLIT_ON -> accept
 * no-op (tx freq set separately). tx_vfo is ignored for this radio.
 */
static int system600_set_split_vfo(RIG *rig, vfo_t vfo, split_t split, vfo_t tx_vfo)
{
    uint8_t frame[SYS600_FRAME_LEN];

    (void)vfo;
    (void)tx_vfo;

    if (split == RIG_SPLIT_OFF)
    {
        sys600_frame(SYS600_OP_RX_TO_TX, 0, 0, 0, 0, frame);
        return write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    }
    else if (split == RIG_SPLIT_ON)
    {
        return RIG_OK;
    }

    return -RIG_EINVAL;
}

/*
 * Get Split VFO: infer split if Tx freq differs from Rx freq.
 * Returns RIG_SPLIT_ON if tx_hz != rx_hz, else RIG_SPLIT_OFF.
 * tx_vfo returned as RIG_VFO_A (single operating VFO).
 */
static int system600_get_split_vfo(RIG *rig, vfo_t vfo, split_t *split, vfo_t *tx_vfo)
{
    uint8_t rec[SYS600_REC_LEN];
    uint32_t rx_hz, tx_hz;
    int ret;

    (void)vfo;

    ret = sys600_read_status(rig, rec);
    if (ret != RIG_OK)
    {
        return ret;
    }

    sys600_decode_record(rec, &rx_hz, &tx_hz);

    if (tx_hz != rx_hz)
    {
        *split = RIG_SPLIT_ON;
    }
    else
    {
        *split = RIG_SPLIT_OFF;
    }
    *tx_vfo = RIG_VFO_A;

    return RIG_OK;
}

/*
 * Set Tx frequency (8Ah, BCD) for split operation.
 * Read-back verification against Tx sub-record.
 */
static int system600_set_split_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
    uint8_t frame[SYS600_FRAME_LEN];
    uint8_t bcd[4];
    uint8_t rec[SYS600_REC_LEN];
    uint32_t rx_hz, tx_hz;
    int ret;

    (void)vfo;

    if (!(freq >= (freq_t)SYS600_FREQ_MIN_HZ &&
            freq <= (freq_t)SYS600_FREQ_MAX_HZ))
    {
        return -RIG_EINVAL;
    }

    if (sys600_freq_to_bcd((uint32_t)freq, bcd) != 0)
    {
        return -RIG_EINVAL;
    }

    sys600_frame(SYS600_OP_SET_TX_FREQ, bcd[0], bcd[1], bcd[2], bcd[3], frame);
    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    if (ret != RIG_OK)
    {
        return ret;
    }

    ret = sys600_read_status(rig, rec);
    if (ret != RIG_OK)
    {
        return ret;
    }

    sys600_decode_record(rec, &rx_hz, &tx_hz);
    if (tx_hz != (uint32_t)freq)
    {
        rig_debug(RIG_DEBUG_ERR,
                  "%s: set did not stick: asked %.0f Hz, radio reports %u Hz\n",
                  __func__, (double)freq, (unsigned)tx_hz);
        return -RIG_EPROTO;
    }

    return RIG_OK;
}

/*
 * Get Tx frequency from Status Update U=2 (Tx sub-record d[10..18]).
 */
static int system600_get_split_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
    uint8_t rec[SYS600_REC_LEN];
    uint32_t rx_hz, tx_hz;
    int ret;

    (void)vfo;

    ret = sys600_read_status(rig, rec);
    if (ret != RIG_OK)
    {
        return ret;
    }

    sys600_decode_record(rec, &rx_hz, &tx_hz);
    *freq = (freq_t)tx_hz;
    return RIG_OK;
}

/*
 * Set RIT: Carrier (50h), 16-bit BINARY twos-complement in 10 Hz units
 * (NOT BCD despite the manual - protocol.py:365-381 carrier_frame; agent ref
 * sec 17.3).  Wire order is [F2][F1][S][W][0x50] where F1:F2 is the big-endian
 * 16-bit value, S=1 (set now), W=0.  Worked example:
 * +1.5k -> +1500 Hz = +150 units = 0x0096 -> 96 00 01 00 50.
 * The carrier offset is not echoed in the status record, so the value is
 * cached in priv for get_rit.
 */
static int system600_set_rit(RIG *rig, vfo_t vfo, shortfreq_t rit)
{
    uint8_t frame[SYS600_FRAME_LEN];
    int32_t units;
    uint16_t value;
    uint8_t f1, f2;
    struct sys600_priv *p = (struct sys600_priv *)STATE(rig)->priv;
    int ret;

    (void)vfo;

    /* The CAT carrier field is a 10 Hz grid, so round to the nearest 10 Hz
     * unit; sub-10-Hz RIT values (not representable) snap to the nearer step. */
    units = (rit >= 0) ? ((int32_t)rit + 5) / 10 : ((int32_t)rit - 5) / 10;

    if (units < -32767)
    {
        units = -32767;
    }
    if (units > 32767)
    {
        units = 32767;
    }

    value = (uint16_t)(units & 0xFFFF);
    f1 = (uint8_t)((value >> 8) & 0xFF);
    f2 = (uint8_t)(value & 0xFF);

    /* Wire: [P4][P3][P2][P1][OPCODE] = [f2][f1][s=1][w=0][0x50] */
    sys600_frame(SYS600_OP_CARRIER, 0, 1, f1, f2, frame);
    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    if (ret == RIG_OK && p != NULL)
    {
        p->rit_hz = (int)(units * 10);  /* cache the value actually sent */
        p->rit_valid = 1;
    }
    return ret;
}

/*
 * Get RIT: return cached value from priv (carrier offset not in status record).
 */
static int system600_get_rit(RIG *rig, vfo_t vfo, shortfreq_t *rit)
{
    struct sys600_priv *p = (struct sys600_priv *)STATE(rig)->priv;

    (void)vfo;

    if (p == NULL || !p->rit_valid)
    {
        *rit = 0;
        return RIG_OK;
    }

    *rit = (shortfreq_t)p->rit_hz;
    return RIG_OK;
}

/*
 * VFO OP: support RIG_OP_UP / RIG_OP_DOWN via 8Eh.
 */
static int system600_vfo_op(RIG *rig, vfo_t vfo, vfo_op_t op)
{
    uint8_t frame[SYS600_FRAME_LEN];

    (void)vfo;

    if (op == RIG_OP_UP)
    {
        sys600_frame(SYS600_OP_STEP, 0, 0, 0, 0, frame);
        return write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    }
    else if (op == RIG_OP_DOWN)
    {
        sys600_frame(SYS600_OP_STEP, 1, 0, 0, 0, frame);
        return write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    }

    return -RIG_EINVAL;
}

/*
 * Get PTT: Read Flags (FAh) and report the transmitting state, Flag3 bit7
 * (radio.py:231 uses transmitting; agent ref sec 10 lists Flag3 bit7 as
 * "Transmission in progress") - not the CAT-PTT Flag2 bit0.
 */
static int system600_get_ptt(RIG *rig, vfo_t vfo, ptt_t *ptt)
{
    struct sys600_flags flags;
    int ret;

    (void)vfo;

    ret = sys600_read_flags(rig, &flags);
    if (ret != RIG_OK)
    {
        return ret;
    }

    rig_debug(RIG_DEBUG_VERBOSE,
              "%s: Flag3 bit7 (transmission in progress)=%d, Flag2 bit0 (PTT by CAT)=%d\n",
              __func__, flags.transmitting, flags.ptt_by_cat);

    /* Use transmitting (Flag3 bit7) per protocol/implementation source of truth
     * (radio.py:231 uses transmitting; agent ref sec 10 lists Flag3 bit7 as
     * "Transmission in progress"). This is the honest transmit state. */
    *ptt = flags.transmitting ? RIG_PTT_ON : RIG_PTT_OFF;
    return RIG_OK;
}

/*
 * =====================================================================
 * Functions (read-only flags), levels, memory channel ops + get_info.
 * =====================================================================
 */

/*
 * Get a rig function.  The CAT protocol has no command to *change* any
 * function - the LOCK button and the antenna tuner are panel-only - but two
 * functions are reported by the Read Flags (FAh) bytes, so they are exposed
 * read-only (set_func stays RIG_FUNC_NONE in caps; rig_set_func returns
 * -RIG_ENIMPL, which is the honest answer):
 *
 *   RIG_FUNC_LOCK  <- Flag1 bit0 "LOCK is active" (agent ref sec 10)
 *   RIG_FUNC_TUNER <- Flag3 bit5 "antenna tuner working" (TUNING on LCD)
 *
 * Everything else is -RIG_ENIMPL: no other flag bit maps onto a Hamlib
 * function, and none of them can be commanded anyway.  The scan-state bits
 * (Flag2 bits 1-2) have no getter in the Hamlib API at all.
 */
static int system600_get_func(RIG *rig, vfo_t vfo, setting_t func, int *status)
{
    struct sys600_flags flags;
    int ret;

    (void)vfo; /* functions are read from the single operating state */

    ret = sys600_read_flags(rig, &flags);
    if (ret != RIG_OK)
    {
        return ret;
    }

    switch (func)
    {
    case RIG_FUNC_LOCK:
        *status = flags.lock;
        return RIG_OK;

    case RIG_FUNC_TUNER:
        *status = flags.tuner;
        return RIG_OK;

    default:
        return -RIG_ENIMPL;
    }
}

/*
 * Get a receiver level.  Only the values the hardware actually exposes are
 * supported:
 *
 *   RIG_LEVEL_RAWSTR   -> the raw meter byte 0..255 (F7h, agent ref sec 8.3).
 *   RIG_LEVEL_STRENGTH -> the raw byte mapped through the provisional linear
 *                         str_cal (see SYS600_STR_CAL).  The reference
 *                         documents meter deflection only, not S-units, so
 *                         this calibration is a documented approximation.
 *   RIG_LEVEL_RFPOWER  -> the TX power *selection* from Flag3 bit1 ("TX
 *                         output of 10 W selected", agent ref sec 10): 0.1
 *                         when the 10 W position is selected, else 1.0
 *                         (the caps TX range is 10..100 W).  Read-only: the
 *                         protocol has no power command, so has_set_level
 *                         does not include it.  Only the 100 W state was
 *                         observable on hardware (the panel switch was not
 *                         moved); the flag itself is straight from the
 *                         manual's bit table.
 *
 * Everything else (AF, RF, SQL, ...) is -RIG_ENIMPL: the CAT protocol has no
 * command to read those, and this backend never invents a value.
 */
static int system600_get_level(RIG *rig, vfo_t vfo, setting_t level, value_t *val)
{
    struct sys600_flags flags;
    int meter;
    int ret;

    (void)vfo; /* level reads are always from the single operating receiver */

    switch (level)
    {
    case RIG_LEVEL_RAWSTR:
    case RIG_LEVEL_STRENGTH:
        ret = sys600_read_meter(rig, &meter);
        if (ret != RIG_OK)
        {
            return ret;
        }
        break;

    case RIG_LEVEL_RFPOWER:
        ret = sys600_read_flags(rig, &flags);
        if (ret != RIG_OK)
        {
            return ret;
        }
        /* 10 W out of the 100 W maximum, or the full-power position. */
        val->f = flags.tx_10w ? 0.1f : 1.0f;
        return RIG_OK;

    default:
        return -RIG_ENIMPL;
    }

    if (level == RIG_LEVEL_STRENGTH)
    {
        val->i = (int)rig_raw2val(meter, &rig->caps->str_cal);
    }
    else
    {
        val->i = meter;
    }

    return RIG_OK;
}

/*
 * Select a memory channel (Recall Memory, 02h; agent ref sec 6.1 item 1 and
 * sec 7).  This matches Hamlib's rig_set_mem semantics ("set the current
 * memory channel number", src/mem.c:53 - the same mapping the sibling
 * ft1000d backend uses, ft1000d.c:2831 sends RECALL_MEM).  03h (Write VFO to
 * Memory) stores the VFO and does not change the current channel, so it
 * cannot implement set_mem.
 *
 * No read-back: the radio has no ack and, right after a VFO<->memory switch,
 * ignores Status queries entirely for ~150 ms (hardware probe9: a U=1 sent
 * 0/20/50/100 ms after the recall got no reply at all, >=200 ms always
 * worked), even though the recall itself took effect (the panel showed the
 * new channel).  So the recall is fire-and-forget here, exactly as in the
 * sibling ft1000d backend, and the write is padded by
 * SYS600_MEM_RECALL_SETTLE_MS so the next client command lands after the deaf
 * window; the current channel is read separately by system600_get_mem.
 */
static int system600_set_mem(RIG *rig, vfo_t vfo, int ch)
{
    uint8_t frame[SYS600_FRAME_LEN];
    int ret;

    (void)vfo;

    if (ch < SYS600_MEM_CH_MIN || ch > SYS600_MAX_MEM_CH)
    {
        return -RIG_EINVAL;
    }

    /* Wire order CH,0,0,0,02 (agent ref sec 7 "Recall Memory N"). */
    sys600_frame(SYS600_OP_RECALL_MEM, (uint8_t)ch, 0, 0, 0, frame);
    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    if (ret != RIG_OK)
    {
        return ret;
    }

    /* Settle: the radio ignores every command for ~150 ms after the recall,
     * so pause here rather than let the next client command be dropped. */
    hl_usleep(SYS600_MEM_RECALL_SETTLE_MS * 1000);
    return RIG_OK;
}

/*
 * Get the current memory channel number: Status Update U=1 (0-based index,
 * so channel = byte + 1).  In VFO mode the radio still reports the
 * last-selected memory channel, which Hamlib explicitly allows (src/mem.c:
 * "it is not mandatory to be in memory mode"); an index outside 0..99 is a
 * protocol error.
 */
static int system600_get_mem(RIG *rig, vfo_t vfo, int *ch)
{
    uint8_t num;
    int ret;

    (void)vfo; /* single operating VFO/memory state, VFO-agnostic */

    ret = sys600_read_memory_number(rig, &num);
    if (ret != RIG_OK)
    {
        return ret;
    }

    if (num > SYS600_MAX_MEM_CH - 1)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: out-of-range memory index %u\n",
                  __func__, (unsigned)num);
        return -RIG_EPROTO;
    }

    *ch = (int)num + 1;
    return RIG_OK;
}

/*
 * =====================================================================
 * Memory channels: read (Status U=4) and write (03h Write VFO to Memory).
 * =====================================================================
 */

/* Operating state saved around a channel write (Hamlib requires channel
 * access to leave the current VFO/memory selection untouched). */
struct sys600_saved_state {
    freq_t rx_hz;
    freq_t tx_hz;
    rmode_t mode;
    pbwidth_t width;
    int in_memory;      /* radio was in memory mode */
    int mem_idx;        /* U=1 index as read (0-based) */
    int mem_ch;         /* mem_idx + 1 (what 02h takes) */
};

static int sys600_save_state(RIG *rig, struct sys600_saved_state *s)
{
    struct sys600_flags flags;
    uint8_t rec[SYS600_REC_LEN];
    uint8_t num;
    uint32_t rx_hz, tx_hz;
    int ret;

    memset(s, 0, sizeof(*s));

    ret = system600_get_mode(rig, RIG_VFO_CURR, &s->mode, &s->width);
    if (ret != RIG_OK)
    {
        return ret;
    }

    ret = sys600_read_status(rig, rec);
    if (ret != RIG_OK)
    {
        return ret;
    }
    sys600_decode_record(rec, &rx_hz, &tx_hz);
    s->rx_hz = (freq_t)rx_hz;
    s->tx_hz = (freq_t)tx_hz;

    ret = sys600_read_flags(rig, &flags);
    if (ret != RIG_OK)
    {
        return ret;
    }
    s->in_memory = flags.mem;

    /* Always read it: the value is needed both to put the radio back on the
     * channel it names and to drive sys600_force_clean_vfo().  The radio
     * reports the last-selected channel even while in VFO operation. */
    ret = sys600_read_memory_number(rig, &num);
    if (ret != RIG_OK)
    {
        return ret;
    }
    s->mem_idx = (int)num;      /* U=1 is 0-based */
    s->mem_ch = (int)num + 1;

    return RIG_OK;
}

/* Best effort: report the first error but try every step, so the radio is
 * never left on an intermediate frequency/mode. */
static int sys600_restore_state(RIG *rig, const struct sys600_saved_state *s)
{
    int first_err = RIG_OK;
    int ret;

    ret = system600_set_freq(rig, RIG_VFO_CURR, s->rx_hz);
    if (ret != RIG_OK && first_err == RIG_OK)
    {
        first_err = ret;
    }

    /* When Rx == Tx the 0Ah above already dragged the Tx record along
     * (verified: 0Ah follows the split state); otherwise restore it.  Doing it
     * after Rx also covers "the VFO was split before we wrote a channel". */
    if (s->tx_hz != s->rx_hz)
    {
        ret = system600_set_split_freq(rig, RIG_VFO_CURR, s->tx_hz);
        if (ret != RIG_OK && first_err == RIG_OK)
        {
            first_err = ret;
        }
    }

    ret = system600_set_mode(rig, RIG_VFO_CURR, s->mode, s->width);
    if (ret != RIG_OK && first_err == RIG_OK)
    {
        first_err = ret;
    }

    if (s->in_memory)
    {
        ret = system600_set_mem(rig, RIG_VFO_CURR, s->mem_ch);
        if (ret != RIG_OK && first_err == RIG_OK)
        {
            first_err = ret;
        }
    }

    return first_err;
}

/*
 * Get one memory channel's contents: Status Update U=4 CH = the 19-byte memory
 * record (agent ref sec 14).  Non-invasive - it does not recall the channel -
 * and read_only is therefore moot: this backend never moves the radio to read
 * a channel.
 *
 * Record decoding (hardware-verified 2026-10-17, agent ref sec 23c):
 *  - byte 0: SYS600_MEM_REC_EMPTY (0x80) marks an empty channel.  As in the
 *    sibling ft1000d backend (ft1000d.c:3063-3067), an empty channel returns
 *    RIG_OK with the remaining fields left zero (freq 0, mode NONE).
 *  - [1..9]  = Rx sub-record, [10..18] = Tx sub-record (same order as U=2),
 *    so a stored split channel reports tx_freq != freq and RIG_SPLIT_ON.
 *  - the sub-record mode byte carries the mode FAMILY only (CW width and J2B
 *    sideband/width are not stored) -> see sys600_record_mode.
 */
static int system600_get_channel(RIG *rig, vfo_t vfo, channel_t *chan,
                                 int read_only)
{
    uint8_t rec[SYS600_REC_LEN];
    uint32_t rx_hz, tx_hz;
    int ch;
    int ret;

    (void)vfo;       /* channel access is VFO-independent */
    (void)read_only; /* reading a channel never changes the radio */

    if (chan == NULL)
    {
        return -RIG_EINVAL;
    }

    ch = chan->channel_num;

    if (ch < SYS600_MEM_CH_MIN || ch > SYS600_MAX_MEM_CH)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: channel %d out of range %d..%d\n",
                  __func__, ch, SYS600_MEM_CH_MIN, SYS600_MAX_MEM_CH);
        return -RIG_EINVAL;
    }

    ret = sys600_read_mem_record(rig, ch, rec);
    if (ret != RIG_OK)
    {
        return ret;
    }

    /* Clean slate, then only the fields this radio really has (ft1000d idiom
     * ft1000d.c:2999-3007). */
    memset(chan, 0, sizeof(*chan));
    chan->vfo = RIG_VFO_MEM;
    chan->channel_num = ch;

    if (rec[0] & SYS600_MEM_REC_EMPTY)
    {
        /* Blank channel: leave freq/mode zero and mark it skippable, the same
         * convention the ic746 backend uses for a blank memory (ic746.c:982). */
        chan->flags = RIG_CHFLAG_SKIP;
        rig_debug(RIG_DEBUG_VERBOSE, "%s: channel %d is empty\n", __func__, ch);
        return RIG_OK;
    }

    sys600_decode_record(rec, &rx_hz, &tx_hz);

    chan->freq = (freq_t)rx_hz;
    chan->tx_freq = (freq_t)tx_hz;
    sys600_record_mode(rec[SYS600_REC_RX_MODE], &chan->mode, &chan->width);
    sys600_record_mode(rec[SYS600_REC_TX_MODE], &chan->tx_mode, &chan->tx_width);

    if (tx_hz != rx_hz)
    {
        chan->split = RIG_SPLIT_ON;
        chan->tx_vfo = RIG_VFO_A;   /* single operating VFO */
    }
    else
    {
        chan->split = RIG_SPLIT_OFF;
    }

    return RIG_OK;
}

/*
 * Mode byte a 03h-written channel will actually carry: the MODE command code
 * is not what the record keeps.  A1A-N (03h cmd) is stored as 02h and every
 * J2B variant (08h/09h/0Eh/0Fh) as 05h - hardware-verified, agent ref sec 23c.
 */
static uint8_t sys600_record_code(uint8_t cmd)
{
    switch (cmd)
    {
    case SYS600_MODE_CMD_LSB:
        return SYS600_MODE_ST_LSB;

    case SYS600_MODE_CMD_USB:
        return SYS600_MODE_ST_USB;

    case SYS600_MODE_CMD_CW_W:
    case SYS600_MODE_CMD_CW_N:
        return SYS600_MODE_ST_CW_W;

    case SYS600_MODE_CMD_AM:
        return SYS600_MODE_ST_AM;

    default:    /* 08h/09h/0Eh/0Fh: all four J2B variants store 05h */
        return SYS600_MODE_ST_J2B;
    }
}

/*
 * Put the radio on a real VFO before a channel store.
 *
 * 03h does not copy "the VFO" as such: it copies whatever the radio currently
 * regards as its data source, and that source survives a plain VFO recall
 * (05h) - after an empty memory channel has been recalled, a U=2 read keeps
 * reporting 0xA0 (record byte 0 bit7 set) and a 03h sent in that state stores
 * an EMPTY record, silently WIPING the target channel.  Reproduced on hardware
 * (agent ref sec 23c), including the wipe of a channel that held data.
 *
 * A real MEM->VFO transition (02h, then 05h) does clear it, so the write path
 * always performs that pair and then re-reads U=2: if bit7 is still set the
 * caller must not send 03h.
 *
 * mem_idx is the 0-based U=1 index, i.e. the channel the radio already names,
 * so recalling it changes nothing observable (the last-selected channel and
 * the operating frequency/mode are left alone - verified on hardware).
 */
static int sys600_force_clean_vfo(RIG *rig, int mem_idx)
{
    uint8_t frame[SYS600_FRAME_LEN];
    uint8_t rec[SYS600_REC_LEN];
    int ret;

    if (mem_idx < 0 || mem_idx >= SYS600_MAX_MEM_CH)
    {
        return -RIG_EPROTO;
    }

    /* Recall Memory (02h): a real switch into memory operation. */
    sys600_frame(SYS600_OP_RECALL_MEM, (uint8_t)(mem_idx + 1), 0, 0, 0, frame);
    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);

    if (ret != RIG_OK)
    {
        return ret;
    }

    hl_usleep(SYS600_MEM_RECALL_SETTLE_MS * 1000);

    /* Recall VFO (05h): back to the VFO - this is the transition that
     * discards the stale memory source. */
    sys600_frame(SYS600_OP_RECALL_VFO, 0, 0, 0, 0, frame);
    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);

    if (ret != RIG_OK)
    {
        return ret;
    }

    hl_usleep(SYS600_MEM_RECALL_SETTLE_MS * 1000);

    /* Prove the source is a real VFO before letting 03h near a channel: if
     * this bit is still set, 03h would erase the target. */
    ret = sys600_read_status(rig, rec);

    if (ret != RIG_OK)
    {
        return ret;
    }

    if (rec[0] & SYS600_MEM_REC_EMPTY)
    {
        rig_debug(RIG_DEBUG_ERR,
                  "%s: VFO recall left data source 0x%02x (empty); "
                  "refusing to write a channel\n", __func__, (unsigned)rec[0]);
        return -RIG_EIO;
    }

    return RIG_OK;
}

/*
 * Read the channel back and confirm it holds what we asked for.  The radio
 * has no write acknowledgement, and a silently dropped 03h is exactly how the
 * "empty source" failure showed up, so the write is never reported as
 * successful unless the record agrees.  Only the fields the record can hold
 * are compared (agent ref sec 23c).
 */
static int sys600_verify_channel(RIG *rig, int ch, const channel_t *chan,
                                 uint8_t cmd)
{
    uint8_t rec[SYS600_REC_LEN];
    uint32_t rx_hz, tx_hz;
    uint8_t want_mode = sys600_record_code(cmd);
    int ret;

    ret = sys600_read_mem_record(rig, ch, rec);

    if (ret != RIG_OK)
    {
        return ret;
    }

    sys600_decode_record(rec, &rx_hz, &tx_hz);

    if ((freq_t)rx_hz != chan->freq || rec[SYS600_REC_RX_MODE] != want_mode)
    {
        rig_debug(RIG_DEBUG_ERR,
                  "%s: channel %d stored %.0f Hz/mode 0x%02x, wanted "
                  "%.0f Hz/mode 0x%02x\n", __func__, ch, (double)rx_hz,
                  (unsigned)rec[SYS600_REC_RX_MODE], (double)chan->freq,
                  want_mode);
        return -RIG_EIO;
    }

    if (chan->split == RIG_SPLIT_ON && chan->tx_freq > 0 &&
            (freq_t)tx_hz != chan->tx_freq)
    {
        rig_debug(RIG_DEBUG_ERR,
                  "%s: channel %d stored Tx %.0f Hz, wanted %.0f Hz\n",
                  __func__, ch, (double)tx_hz, (double)chan->tx_freq);
        return -RIG_EIO;
    }

    return RIG_OK;
}

/*
 * Program a memory channel: Write VFO to Memory (03h, P2=0; agent ref sec 6.1
 * item 2, hardware-verified 2026-10-17 sec 23: P2=0 writes, P2=1 erases/hides).
 *
 * 03h copies the *displayed* VFO, so the sequence is: save the current
 * operating state, force a clean VFO (sys600_force_clean_vfo), load the
 * requested frequency/mode/Tx, send 03h, read the channel back
 * (sys600_verify_channel), then put the radio back exactly where it was.
 * Hamlib requires channel access to leave the current VFO and memory number
 * selected (src/mem.c:674-676).
 *
 * Only what the record can hold is written: frequency, mode family and the Tx
 * (split) frequency.  CW width and J2B sideband/width are NOT stored by the
 * radio (agent ref sec 23c) - the mode family is what gets programmed, exactly
 * as a panel store would.
 *
 * RIG_CHFLAG_SKIP is not mapped onto 03h P2=1 ("Hide CH"): hiding a channel
 * erases it here (the U=4 record becomes 80h + zeros), which is not what
 * Hamlib's "skip during scan" flag means.  The erase command is deliberately
 * left at the protocol level (the Python reference's write_memory(hide=True)).
 */
static int system600_set_channel(RIG *rig, vfo_t vfo, const channel_t *chan)
{
    struct sys600_saved_state saved;
    uint8_t frame[SYS600_FRAME_LEN];
    uint8_t cmd;
    int ch, ret;

    (void)vfo;

    if (chan == NULL)
    {
        return -RIG_EINVAL;
    }

    ch = chan->channel_num;

    if (ch < SYS600_MEM_CH_MIN || ch > SYS600_MAX_MEM_CH)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: channel %d out of range %d..%d\n",
                  __func__, ch, SYS600_MEM_CH_MIN, SYS600_MAX_MEM_CH);
        return -RIG_EINVAL;
    }

    /* 03h writes into a channel; storing "a VFO" is not a radio concept. */
    if (chan->vfo != RIG_VFO_MEM)
    {
        return -RIG_ENIMPL;
    }

    if (chan->freq <= 0 || chan->mode == RIG_MODE_NONE)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: channel %d: freq %.0f / mode 0x%x: "
                  "nothing to store\n", __func__, ch, (double)chan->freq,
                  (unsigned)chan->mode);
        return -RIG_EINVAL;
    }

    if (sys600_rmode_to_cmd(chan->mode, chan->width, &cmd) != 0)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: unsupported mode 0x%x\n", __func__,
                  (unsigned)chan->mode);
        return -RIG_EINVAL;
    }

    ret = sys600_save_state(rig, &saved);
    if (ret != RIG_OK)
    {
        return ret;
    }

    /* Force a real VFO source: 03h must copy the VFO, and a blank/stale memory
     * source would make it store an empty record (i.e. erase the channel).
     * This also covers "the radio is sitting in memory mode". */
    ret = sys600_force_clean_vfo(rig, saved.mem_idx);

    if (ret != RIG_OK)
    {
        return ret; /* nothing has been touched yet: no restore needed */
    }

    ret = system600_set_freq(rig, RIG_VFO_CURR, chan->freq);

    if (ret == RIG_OK)
    {
        if (chan->split == RIG_SPLIT_ON && chan->tx_freq > 0)
        {
            ret = system600_set_split_freq(rig, RIG_VFO_CURR, chan->tx_freq);
        }
        else
        {
            /* Not split: VFO Rx->Tx (85h) so a stale Tx record is not stored
             * with the channel (0Ah only drags Tx along when not split). */
            sys600_frame(SYS600_OP_RX_TO_TX, 0, 0, 0, 0, frame);
            ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
        }
    }

    if (ret == RIG_OK)
    {
        ret = system600_set_mode(rig, RIG_VFO_CURR, chan->mode, chan->width);
    }

    if (ret == RIG_OK)
    {
        /* Wire order CH,0,0,0,03 (P1=CH, P2=0 write; agent ref sec 7). */
        sys600_frame(SYS600_OP_WRITE_MEM, (uint8_t)ch, 0, 0, 0, frame);
        ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    }

    if (ret == RIG_OK)
    {
        /* 03h has no acknowledgement: read the channel back so a store the
         * radio silently dropped is reported instead of claimed as success.
         * The read must wait out the radio's post-write deaf window. */
        hl_usleep(SYS600_MEM_WRITE_SETTLE_MS * 1000);
        ret = sys600_verify_channel(rig, ch, chan, cmd);
    }

    /* Put the radio back (or back as far as it can be) either way. */
    if (sys600_restore_state(rig, &saved) != RIG_OK)
    {
        rig_debug(RIG_DEBUG_ERR, "%s: state restore failed\n", __func__);
    }

    return ret;
}


/*
 * get_info: short, honest, static description.  No dynamic rig state is
 * claimed (the protocol has no identification query).
 */
static const char *system600_get_info(RIG *rig)
{
    (void)rig;
    return "Yaesu System 600 CAT";
}

/*
 * PROVISIONAL S-meter calibration.  The protocol reference documents only the
 * raw digitized meter deflection (0..255, agent ref sec 8.3) - it does NOT map
 * that byte to S-units or dBm.  This table is therefore an explicit
 * approximation: a linear 8-bit mapping with the same shape as the PCR1000
 * family (rigs/pcr/pcr1000.c), i.e. 0 -> -60 dB, 127 -> 0 dB (S9), 255 -> +60 dB,
 * relative to S9.  Used only for RIG_LEVEL_STRENGTH; RAWSTR returns the raw byte.
 */
#define SYS600_STR_CAL { 3, { { 0, -60 }, { 127, 0 }, { 255, 60 } } }

/*
 * Memory-channel capability: Write/Recall move the whole VFO record, so a
 * stored channel carries frequency, mode and (mode-associated) passband
 * (frg100 idiom, rigs/yaesu/frg100.c:81-85).  The Tx (split) frequency is
 * honoured too - the record has an independent Tx sub-record - while tx
 * MODE/width, channel description, tones, flags etc. are not stored by this
 * radio and are deliberately left undeclared (agent ref sec 23c).
 */
#define SYS600_MEM_CAP { \
        .freq = 1,       \
        .mode = 1,       \
        .width = 1,      \
        .tx_freq = 1,    \
        .split = 1       \
    }

/*
 * Minimal registration.  All fields dump_caps can touch are filled with
 * honest provisional values.
 * Serial defaults: 4800 8N2 (SYS600_BAUD, radio.py:3-7/54-57) applied by
 * the frontend from these caps fields.
 */
struct rig_caps system600_caps =
{
    RIG_MODEL(RIG_MODEL_SYSTEM600),
    .model_name =         "System 600",
    .mfg_name =           "Yaesu",
    .version =            "1.0",
    .copyright =          "LGPL",
    /* Hardware-verified on a real System 600 (2026-10-16/17): frequency,
     * mode (incl. AM and both J2B sidebands), memory recall, split, step ops,
     * levels, flags, PTT.  BETA, not STABLE: the S-meter calibration is a
     * documented approximation and the tuning step / carrier offset are not
     * readable from the radio. */
    .status =             RIG_STATUS_BETA,
    .rig_type =           RIG_TYPE_TRANSCEIVER,
    .ptt_type =           RIG_PTT_RIG, /* 0Fh CAT PTT */
    .dcd_type =           RIG_DCD_NONE,
    .port_type =          RIG_PORT_SERIAL,
    .serial_rate_min =    SYS600_BAUD,
    .serial_rate_max =    SYS600_BAUD,
    .serial_data_bits =   8,
    .serial_stop_bits =   2,
    .serial_parity =      RIG_PARITY_NONE,
    .serial_handshake =   RIG_HANDSHAKE_NONE,
    .write_delay =        0,
    /* The radio drops a frame written back-to-back with the previous one
     * (measured on hardware: a Status U=1 query sent 0 ms after Recall Memory
     * gets no reply, >=10 ms always works).  Give every write a ~20 ms gap. */
    .post_write_delay =   SYS600_POST_WRITE_DELAY_MS,
    .timeout =            SYS600_READ_TIMEOUT_MS,
    .retry =              0,
    /* Read-only functions: only LOCK (Flag1 bit0) and TUNER (Flag3 bit5) are
     * readable, and neither can be *commanded* over CAT (no opcode), so
     * has_set_func stays empty - rig_set_func answers -RIG_ENIMPL honestly. */
    .has_get_func =       RIG_FUNC_LOCK | RIG_FUNC_TUNER,
    .has_set_func =       RIG_FUNC_NONE,
    /* RFPOWER is read-only too: Flag3 bit1 only reports whether the panel's
     * 10 W position is selected (there is no power opcode). */
    .has_get_level =      RIG_LEVEL_STRENGTH | RIG_LEVEL_RAWSTR |
                          RIG_LEVEL_RFPOWER,
    .has_set_level =      RIG_LEVEL_NONE,
    .has_get_parm =       RIG_PARM_NONE,
    .has_set_parm =       RIG_PARM_NONE,
    .vfo_ops =            RIG_OP_UP | RIG_OP_DOWN,
    .scan_ops =           RIG_SCAN_NONE,
    /* The rig exposes its Tx frequency independently (8Ah / U=2 Tx sub-record),
     * so split set/get route straight through instead of a VFO-swap dance (a
     * single-VFO rig cannot swap).  frg9600 precedent. */
    .targetable_vfo =     RIG_TARGETABLE_FREQ,
    .transceive =         RIG_TRN_OFF,
    .preamp =             { RIG_DBLST_END, },
    .attenuator =         { RIG_DBLST_END, },
    /* Carrier (50h) offset is 16-bit binary in 10 Hz units: +/-327670 Hz. */
    .max_rit =            Hz(327670),
    .max_xit =            Hz(0),
    .max_ifshift =        Hz(0),
    .bank_qty =           0,
    .chan_desc_sz =       0,
    /* One memory bank of 100 channels; set_mem/get_mem are implemented
     * (Recall Memory 02h + Status U=2 d[0]), frg100-style cap. */
    .chan_list =          {
        { SYS600_MEM_CH_MIN, SYS600_MAX_MEM_CH, RIG_MTYPE_MEM, SYS600_MEM_CAP },
        RIG_CHAN_END,
    },
    .rx_range_list1 =     {
        {kHz(100), MHz(30), SYSTEM600_MODES, -1, -1, RIG_VFO_A },
        RIG_FRNG_END,
    },
    /* TX power bounds: the reference documents only the 10 W selection flag
     * (Flag3 bit1, agent ref sec 10), so 10 W is the honest low bound; the
     * 100 W high bound is taken from the legacy `ft600` backend
     * (rigs/yaesu/ft600.c).
     * NOTE: -1/-1 here made dump_caps report "TX ranges #1 status: Bad (-3)"
     * (range_sanity_check requires low/high_power >= 0 for TX ranges). */
    .tx_range_list1 =     {
        {kHz(100), MHz(30), SYSTEM600_MODES, W(10), W(100), RIG_VFO_A },
        RIG_FRNG_END,
    },
    .tuning_steps =       {
        {SYSTEM600_MODES, Hz(10)},
        RIG_TS_END,
    },
    /* The radio has two filter positions per mode (hardware-verified
     * 2026-10-17).  The panel default is the WIDE position, so list it first:
     * the frontend's rig_passband_normal() returns the first entry matching
     * the mode, and sys600_rmode_to_cmd() maps that width to the wide
     * position.  LSB/USB/AM have a single fixed filter. */
    .filters =            {
        {RIG_MODE_LSB | RIG_MODE_USB,       Hz(2400)},
        {RIG_MODE_CW,                       Hz(500)},
        {RIG_MODE_CW,                       Hz(250)},
        {RIG_MODE_AM,                       Hz(6000)},
        {RIG_MODE_PKTLSB | RIG_MODE_PKTUSB, Hz(2400)},
        {RIG_MODE_PKTLSB | RIG_MODE_PKTUSB, Hz(500)},
        RIG_FLT_END,
    },
    /* PROVISIONAL: see SYS600_STR_CAL above.  Only consulted for
     * RIG_LEVEL_STRENGTH; RIG_LEVEL_RAWSTR returns the raw byte. */
    .str_cal =            SYS600_STR_CAL,

    .rig_init =           system600_init,
    .rig_cleanup =        system600_cleanup,
    .rig_open =           system600_open,
    .rig_close =          system600_close,
    .set_freq =           system600_set_freq,
    .get_freq =           system600_get_freq,
    .set_mode =           system600_set_mode,
    .get_mode =           system600_get_mode,
    .set_vfo =            system600_set_vfo,
    .get_vfo =            system600_get_vfo,
    .set_ptt =            system600_set_ptt,
    .get_ptt =            system600_get_ptt,
    .set_split_vfo =      system600_set_split_vfo,
    .get_split_vfo =      system600_get_split_vfo,
    .set_split_freq =     system600_set_split_freq,
    .get_split_freq =     system600_get_split_freq,
    .set_rit =            system600_set_rit,
    .get_rit =            system600_get_rit,
    .vfo_op =             system600_vfo_op,
    .get_func =           system600_get_func,
    .get_level =          system600_get_level,
    .set_mem =            system600_set_mem,
    .get_mem =            system600_get_mem,
    .set_channel =        system600_set_channel,
    .get_channel =        system600_get_channel,
    .get_info =           system600_get_info,

    /* Per-instance priv data is calloc'ed in system600_init into
     * STATE(rig)->priv; this caps-level slot stays NULL (ft817 idiom). */
    .priv =               NULL,
    .hamlib_check_rig_caps = HAMLIB_CHECK_RIG_CAPS
};

#endif /* SYSTEM600_CODEC_ONLY */
