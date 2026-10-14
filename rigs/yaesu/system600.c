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
#include "hamlib/rig.h"
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
static int system600_open(RIG *rig);
static int system600_close(RIG *rig);
static int system600_set_freq(RIG *rig, vfo_t vfo, freq_t freq);
static int system600_get_freq(RIG *rig, vfo_t vfo, freq_t *freq);

/*
 * Status Update (10h) with U=2: request, then read exactly the 19-byte
 * operating record.  Mirrors radio.py:115-132 (_query): flush stale input
 * first so the reply stays aligned, then write, then read the expected
 * size.  Never U=0 (1941 bytes).
 *
 * Returns RIG_OK, -RIG_ETIMEOUT if the radio did not answer within the
 * port timeout, -RIG_EPROTO for any short/wrong-length reply, or the
 * write_block error verbatim.
 */
static int sys600_read_status(RIG *rig, uint8_t rec[SYS600_REC_LEN])
{
    uint8_t frame[SYS600_FRAME_LEN];
    uint8_t buf[SYS600_STATUS_SIZE_U2 + 1];
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

    /*
     * No stopset: the status record is binary and every byte value (0x00
     * padding included) can occur, so termination is by length only.
     * expected_len=1 is the documented read idiom (notes sec 1, netrigctl
     * rigs/dummy/netrigctl.c:326): read_string then pulls exactly one
     * byte per port_read and stops at rxmax-1 = 19 bytes (rxmax carries
     * the +1 for its end-of-string byte).  An expected_len of 19 would
     * instead take read_string_generic's adaptive-minlen path, which
     * decrements its static minlen per read and issues a zero-length
     * read after the first byte.
     */
    ret = read_string(RIGPORT(rig), buf, sizeof(buf), NULL, 0, 0, 1);
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
    .ptt_type =           RIG_PTT_NONE, /* set_ptt not wired until a later task */
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
    .vfo_ops =            RIG_OP_NONE,
    .scan_ops =           RIG_SCAN_NONE,
    .targetable_vfo =     0,
    .transceive =         RIG_TRN_OFF,
    .preamp =             { RIG_DBLST_END, },
    .attenuator =         { RIG_DBLST_END, },
    .max_rit =            Hz(0),
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

    .rig_open =           system600_open,
    .rig_close =          system600_close,
    .set_freq =           system600_set_freq,
    .get_freq =           system600_get_freq,

    .priv =               NULL,
    .hamlib_check_rig_caps = HAMLIB_CHECK_RIG_CAPS
};

#endif /* SYSTEM600_CODEC_ONLY */
