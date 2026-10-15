/*
 * system600.c - Yaesu System 600 (FT-600) CAT codec + Hamlib backend.
 *
 * Ported strictly from ft600rig/protocol.py (the source of truth for this
 * backend); the protocol.py -> C helper mapping with line numbers is
 * recorded in task-3-report.md.  Agent-reference section numbers in
 * comments refer to YAESU_SYSTEM600_CAT_PROTOCOL_AGENT_REFERENCE.txt.
 *
 * Two sections: (1) the pure codec - no serial I/O, no Hamlib callbacks,
 * no registration, <stdint.h>/<string.h> only - and (2) below the codec,
 * the Hamlib glue (open/close/set_freq/get_freq + system600_caps, Task 4).
 * Pure-codec consumers (sys600_veccheck.c) define SYSTEM600_CODEC_ONLY
 * before including this file to compile section 1 alone.
 *
 * Return convention: 0 on success, negative on invalid input - mirroring
 * Hamlib's RIG_OK / negative-error convention (doc/hamlib-impl-notes.md
 * sec 2) without including rig.h.
 */

#include <stdint.h>
#include <string.h>

#ifndef SYSTEM600_CODEC_ONLY
#include <stdlib.h>     /* calloc/free for the priv data (ft817 idiom) */

#include "hamlib/rig.h"
#include "hamlib/rig_state.h"  /* STATE(rig)->priv (ft817_init idiom) */
#include "iofunc.h"
#include "misc.h"
#endif

#include "system600.h"

/*
 * Not every codec helper is consumed by the Task 4 glue yet (BCD inverse,
 * flags/meter decode and the mode translators are wired up by later tasks;
 * the vector harness exercises them today).  Silence -Wunused-function for
 * them until then.
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

/*
 * Inverse of sys600_freq_to_bcd: F1..F4 -> Hz
 * (protocol.py:175-183 bcd_params_to_freq; used by tests/simulators).
 * Returns 0, or -1 if a nibble is not a decimal digit.
 */
static SYS600_UNUSED_FN int sys600_bcd_to_freq(const uint8_t in[4], uint32_t *hz)
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
 * Decode the 19-byte operating/memory record [mem(1)][Tx(9)][Rx(9)]
 * (protocol.py:236-247 decode_operating_record; agent ref sec 11).
 * Yields Rx freq, Tx freq, the Rx status mode byte (d[16], offsets per
 * system600.h) and the memory channel number in d[0] (0 = VFO).
 * Diagram order (Tx first) is fixed; protocol.py:240's rx_second option is
 * deliberately not ported - the brief mandates d[10..18] = Rx.  The caller
 * guarantees 19 bytes.
 */
static void sys600_decode_record(const uint8_t rec[SYS600_REC_LEN],
                                 uint32_t *rx_hz, uint32_t *tx_hz,
                                 uint8_t *mode, uint8_t *mem)
{
    *mem = rec[SYS600_REC_MEM];
    sys600_vfo_freq(rec + SYS600_REC_TX, tx_hz);
    sys600_vfo_freq(rec + SYS600_REC_RX, rx_hz);
    *mode = rec[SYS600_REC_RX_MODE];
}

/*
 * Flag bits of the 3 status flag bytes (protocol.py:253-267; agent ref
 * sec 8.2, sec 10).  Flag1: LOCK / memory-vs-VFO operation.  Flag2: PTT by
 * CAT, scan state, filter narrow bits, J2B selected.  Flag3: TX state.
 */
#define SYS600_F1_LOCK         0x01
#define SYS600_F1_MEM          0x20
#define SYS600_F1_VFO          0x80
#define SYS600_F2_PTT_CAT      0x01
#define SYS600_F2_SCAN_PAUSED  0x02
#define SYS600_F2_SCAN         0x04
#define SYS600_F2_J2B_NARROW   0x08
#define SYS600_F2_A1A_NARROW   0x10
#define SYS600_F2_J2B          0x20
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

/*
 * Translate a status-data mode byte to a MODE (0Ch) command code.
 * Returns 0, or -1 for an out-of-table status code.
 */
static SYS600_UNUSED_FN int sys600_mode_status_to_cmd(uint8_t status, uint8_t *cmd)
{
    if ((unsigned)status >= sizeof(sys600_mode_status_to_cmd_tbl)) {
        return -1;
    }
    *cmd = sys600_mode_status_to_cmd_tbl[status];
    return 0;
}


/*
 * =====================================================================
 * Hamlib glue (Task 4): open/close, set/get freq, minimal registration.
 * Port semantics come from ft600rig/radio.py; call idioms from
 * doc/hamlib-impl-notes.md sec 1.
 * =====================================================================
 */

#ifndef SYSTEM600_CODEC_ONLY

/*
 * Mode set advertised in the range/tuning-step lists.  CAT has no CWR, so
 * CW only (wide/narrow are filter variants of the same RIG_MODE_CW).
 */
#define SYSTEM600_MODES (RIG_MODE_LSB | RIG_MODE_USB | RIG_MODE_CW | \
                         RIG_MODE_AM | RIG_MODE_PKTLSB | RIG_MODE_PKTUSB)

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

/*
 * Per-instance private data, allocated in system600_init and released in
 * system600_cleanup (ft817_init/cleanup idiom: rigs/yaesu/ft817.c:622-646,
 * STATE(rig)->priv).  j2b_sideband caches the last PKTLSB/PKTUSB mode set:
 * the status mode byte cannot encode the J2B sideband (it lives in Flag2,
 * agent ref sec 17.10), so a get_mode on status 5 needs the last-set value.
 * rit_* is the reserved slot for Task 6's RIT cache.
 */
struct sys600_priv {
    rmode_t j2b_sideband;       /* last-set RIG_MODE_PKTLSB / RIG_MODE_PKTUSB */
    int j2b_sideband_valid;     /* j2b_sideband holds a set value */
    int rit_hz;                 /* Task 6 slot: cached RIT offset, Hz */
    int rit_valid;              /* Task 6 slot: rit_hz holds a value */
};

/*
 * Status Update (10h) with U=2: request, then read exactly the 19-byte
 * operating record.  Mirrors radio.py:115-132 (_query): flush stale input
 * first so the reply stays aligned, then write, then read the expected
 * size.  Never U=0 (1941 bytes).
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
static int sys600_read_status(RIG *rig, uint8_t rec[SYS600_REC_LEN])
{
    uint8_t frame[SYS600_FRAME_LEN];
    uint8_t buf[SYS600_STATUS_SIZE_U2];
    int ret;

    /* P1 = U selector, everything else 0 (protocol.py:355-362 status_frame). */
    sys600_frame(SYS600_OP_STATUS, 2, 0, 0, 0, frame);

    /* Drop stale bytes so we stay aligned (radio.py:117 reset_input_buffer). */
    rig_flush(RIGPORT(rig));

    ret = write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
    if (ret != RIG_OK)
    {
        return ret;
    }

    ret = read_block(RIGPORT(rig), buf, SYS600_STATUS_SIZE_U2);
    if (ret < 0)
    {
        return ret; /* -RIG_ETIMEOUT (a timeout is a timeout, even partial) */
    }

    if (ret != SYS600_STATUS_SIZE_U2)
    {
        rig_debug(RIG_DEBUG_ERR,
                  "%s: short status reply: expected %d bytes, got %d\n",
                  __func__, SYS600_STATUS_SIZE_U2, ret);
        return -RIG_EPROTO;
    }

    memcpy(rec, buf, SYS600_REC_LEN);
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
 * Init: allocate the per-instance priv data (ft817_init idiom,
 * rigs/yaesu/ft817.c:622-636).  calloc zeroes everything: j2b_sideband
 * starts "never set" so get_mode reports PKTUSB until a PKTLSB/PKTUSB
 * set_mode teaches us the sideband (task-5-brief line 5).
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
 * the frontend (doc/hamlib-impl-notes.md sec 4).
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
    uint8_t mode, mem;
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

    sys600_decode_record(rec, &rx_hz, &tx_hz, &mode, &mem);
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
 * the Rx sub-record d[10..18] (absolute offsets d[11..13], binary 10 Hz
 * units - see codec).  Never U=0.
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
 * Task 5: mode / VFO / PTT callbacks.  Semantics from ft600rig/radio.py
 * (set/get_mode 189-222, set/get_ptt 224-231, set/get_vfo 233-249).
 * =====================================================================
 */

/*
 * Map a Hamlib mode + width to a MODE (0Ch) command byte
 * (radio.py:189-192 set_mode: narrow = 0 < passband <= 900; passband <= 0
 * selects the wide variant).  The radio has no CWR/RTTY/... modes: any
 * rmode outside the CAT table is rejected with -RIG_EINVAL (frg9600.c:189
 * idiom per notes sec 4).
 */
static int sys600_rmode_to_cmd(rmode_t mode, pbwidth_t width, uint8_t *cmd)
{
    int narrow = (width > 0 && width <= SYS600_NARROW_MAX_HZ);

    switch (mode)
    {
    case RIG_MODE_LSB:
        *cmd = SYS600_MODE_CMD_LSB;
        return 0;

    case RIG_MODE_USB:
        *cmd = SYS600_MODE_CMD_USB;
        return 0;

    case RIG_MODE_CW:
        *cmd = narrow ? SYS600_MODE_CMD_CW_N : SYS600_MODE_CMD_CW_W;
        return 0;

    case RIG_MODE_AM:
        *cmd = SYS600_MODE_CMD_AM;
        return 0;

    case RIG_MODE_PKTLSB:
        *cmd = narrow ? SYS600_MODE_CMD_J2B_LSB_N : SYS600_MODE_CMD_J2B_LSB_W;
        return 0;

    case RIG_MODE_PKTUSB:
        *cmd = narrow ? SYS600_MODE_CMD_J2B_USB_N : SYS600_MODE_CMD_J2B_USB_W;
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
    struct sys600_priv *p = (struct sys600_priv *)STATE(rig)->priv;
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

    if (rec[SYS600_REC_RX_MODE] != want_status)
    {
        rig_debug(RIG_DEBUG_ERR,
                  "%s: set did not stick: asked status mode 0x%x, radio reports 0x%x\n",
                  __func__, (unsigned)want_status,
                  (unsigned)rec[SYS600_REC_RX_MODE]);
        return -RIG_EPROTO;
    }

    if (p != NULL &&
            (mode == RIG_MODE_PKTLSB || mode == RIG_MODE_PKTUSB))
    {
        p->j2b_sideband = mode;
        p->j2b_sideband_valid = 1;
    }

    return RIG_OK;
}

/*
 * Get operating mode + passband.  The status mode byte selects the mode
 * family (protocol.py:115-122 STATUS_MODE_TO_TOKEN); status 5 (J2B)
 * reports the cached last-set sideband, else the PKTUSB default
 * (task-5-brief line 5, agent ref sec 17.10).  Passband comes from the
 * Flag2 narrow bits exactly as radio.py:209-221: CW -> 250 if A1A-narrow
 * else 500; J2B -> 500 if J2B-narrow else 2400; LSB/USB -> 2400;
 * AM -> 6000.  A flags read failure falls back to the wide values
 * (radio.py:212-213 swallows RadioError and uses wide defaults).
 */
static int system600_get_mode(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width)
{
    uint8_t rec[SYS600_REC_LEN];
    struct sys600_flags flags;
    struct sys600_priv *p = (struct sys600_priv *)STATE(rig)->priv;
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
        /* Sideband not representable: cached last-set, else PKTUSB. */
        if (p != NULL && p->j2b_sideband_valid &&
                p->j2b_sideband == RIG_MODE_PKTLSB)
        {
            *mode = RIG_MODE_PKTLSB;
        }
        else
        {
            *mode = RIG_MODE_PKTUSB;
        }
        break;

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

    have_flags = (sys600_read_flags(rig, &flags) == RIG_OK);

    if (*mode == RIG_MODE_CW)
    {
        *width = (have_flags && flags.a1a_narrow) ?
                 SYS600_PASSBAND_CW_N_HZ : SYS600_PASSBAND_CW_W_HZ;
    }
    else /* J2B */
    {
        *width = (have_flags && flags.j2b_narrow) ?
                 SYS600_PASSBAND_J2B_N_HZ : SYS600_PASSBAND_J2B_W_HZ;
    }

    return RIG_OK;
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
 * Set PTT (0Fh, T=1 on / T=0 off; radio.py:224-227 set_ptt, protocol
 * ptt_frame).  SAFETY: this radio has NO antenna - the RIG_PTT_ON branch
 * exists in code but must never be exercised by tests or the simulator
 * session (task-5-brief line 7: test only PTT-off).
 */
static int system600_set_ptt(RIG *rig, vfo_t vfo, ptt_t ptt)
{
    uint8_t frame[SYS600_FRAME_LEN];
    uint8_t t;

    (void)vfo; /* 0F always targets the current operating state */

    switch (ptt)
    {
    case RIG_PTT_OFF:
        t = 0;
        break;

    case RIG_PTT_ON:
        t = 1;
        break;

    default:
        return -RIG_EINVAL;
    }

    sys600_frame(SYS600_OP_PTT, t, 0, 0, 0, frame);
    return write_block(RIGPORT(rig), frame, SYS600_FRAME_LEN);
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
    uint8_t mode, mem;
    int ret;

    (void)vfo;

    ret = sys600_read_status(rig, rec);
    if (ret != RIG_OK)
    {
        return ret;
    }

    sys600_decode_record(rec, &rx_hz, &tx_hz, &mode, &mem);

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
    uint8_t mode, mem;
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

    sys600_decode_record(rec, &rx_hz, &tx_hz, &mode, &mem);
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
 * Get Tx frequency from Status Update U=2 (Tx sub-record).
 */
static int system600_get_split_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
    uint8_t rec[SYS600_REC_LEN];
    uint32_t rx_hz, tx_hz;
    uint8_t mode, mem;
    int ret;

    (void)vfo;

    ret = sys600_read_status(rig, rec);
    if (ret != RIG_OK)
    {
        return ret;
    }

    sys600_decode_record(rec, &rx_hz, &tx_hz, &mode, &mem);
    *freq = (freq_t)tx_hz;
    return RIG_OK;
}

/*
 * Set RIT: Carrier (50h), 16-bit BINARY twos-complement in 10 Hz units
 * (NOT BCD despite the manual - protocol.py:365-381 carrier_frame; agent ref
 * sec 17.3).  Wire order is [F2][F1][S][W][0x50] where F1:F2 is the big-endian
 * 16-bit value, S=1 (set now), W=0.  Worked example from the brief:
 * +1.5k -> +1500 Hz = +150 units = 0x0096 -> 96 00 01 00 50.
 * The carrier offset is not echoed in the status record, so the value is
 * cached in priv for get_rit (brief line 5).
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
 * Get PTT: Read Flags (FAh), Flag2 bit0 = PTT closed by CAT command
 * (task-5-brief line 8; protocol.py:257 FLAG_F2_PTT_CAT).
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

    /* Use transmitting (Flag3 bit7) per protocol/implementation source of truth
     * (radio.py:231 uses transmitting; agent ref sec 10 lists Flag3 bit7 as
     * "Transmission in progress"). This is the honest transmit state. */
    *ptt = flags.transmitting ? RIG_PTT_ON : RIG_PTT_OFF;
    return RIG_OK;
}

/*
 * Minimal registration (Task 4).  All fields dump_caps can touch are
 * filled with honest provisional values; a later task refines them.
 * Serial defaults: 4800 8N2 (SYS600_BAUD, radio.py:3-7/54-57) applied by
 * the frontend from these caps fields (notes sec 1).
 */
struct rig_caps system600_caps =
{
    RIG_MODEL(RIG_MODEL_SYSTEM600),
    .model_name =         "System 600",
    .mfg_name =           "Yaesu",
    .version =            "1.0",
    .copyright =          "GPL",
    /* No RIG_STATUS_OK in this enum; simulator-tested only, no hardware. */
    .status =             RIG_STATUS_UNTESTED,
    .rig_type =           RIG_TYPE_TRANSCEIVER,
    .ptt_type =           RIG_PTT_RIG, /* 0Fh CAT PTT; set_ptt/get_ptt wired (Task 5) */
    .dcd_type =           RIG_DCD_NONE,
    .port_type =          RIG_PORT_SERIAL,
    .serial_rate_min =    SYS600_BAUD,
    .serial_rate_max =    SYS600_BAUD,
    .serial_data_bits =   8,
    .serial_stop_bits =   2,
    .serial_parity =      RIG_PARITY_NONE,
    .serial_handshake =   RIG_HANDSHAKE_NONE,
    .write_delay =        0,
    .post_write_delay =   0,
    .timeout =            SYS600_READ_TIMEOUT_MS,
    .retry =              0,
    .has_get_func =       RIG_FUNC_NONE,
    .has_set_func =       RIG_FUNC_NONE,
    .has_get_level =      RIG_LEVEL_NONE,
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
    .chan_list =          { RIG_CHAN_END, },
    .rx_range_list1 =     {
        {kHz(100), MHz(30), SYSTEM600_MODES, -1, -1, RIG_VFO_A },
        RIG_FRNG_END,
    },
    .tx_range_list1 =     {
        {kHz(100), MHz(30), SYSTEM600_MODES, -1, -1, RIG_VFO_A },
        RIG_FRNG_END,
    },
    .tuning_steps =       {
        {SYSTEM600_MODES, Hz(1)},
        RIG_TS_END,
    },
    .filters =            {
        RIG_FLT_END,
    },

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

    /* Per-instance priv data is calloc'ed in system600_init into
     * STATE(rig)->priv; this caps-level slot stays NULL (ft817 idiom). */
    .priv =               NULL,
    .hamlib_check_rig_caps = HAMLIB_CHECK_RIG_CAPS
};

#endif /* SYSTEM600_CODEC_ONLY */
