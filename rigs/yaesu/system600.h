/*
 * system600.h - Yaesu System 600 CAT protocol constants.
 *
 * system600.h - (C) soxxz 2026
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
 * Pure codec header: wire-frame layout, opcode and reply constants, timing,
 * mode tables and the 19-byte status record layout.  Ported strictly from
 * ft600rig/protocol.py (source of truth for this backend); agent-reference
 * section numbers in comments refer to
 * YAESU_SYSTEM600_CAT_PROTOCOL_AGENT_REFERENCE.txt.
 *
 * No I/O, no Hamlib dependencies: <stdint.h> only.
 */
#ifndef SYSTEM600_H
#define SYSTEM600_H

#include <stdint.h>

/*
 * Wire frame (protocol.py:50, 132-139; agent ref sec 4.2, sec 21).
 * Every command sent to the radio is exactly five bytes on the wire:
 *
 *     [ P4 ][ P3 ][ P2 ][ P1 ][ OPCODE ]
 *
 * Padding ("dummy") bytes may be any value but must be present.
 */
#define SYS600_FRAME_LEN            5

/*
 * Opcodes (protocol.py:36-48; agent ref sec 6, sec 21 cheat sheet).
 */
#define SYS600_OP_RECALL_MEM        0x02    /* Recall memory CH (1..100) */
#define SYS600_OP_WRITE_MEM         0x03    /* Write VFO to CH (P2=0) / Hide CH (P2=1) */
#define SYS600_OP_RECALL_VFO        0x05    /* Return to the last-used VFO */
#define SYS600_OP_SET_RX_FREQ       0x0A    /* Set receive freq (BCD) */
#define SYS600_OP_SET_TX_FREQ       0x8A    /* Set transmit freq (BCD, split) */
#define SYS600_OP_MODE              0x0C    /* Set operating mode */
#define SYS600_OP_PTT               0x0F    /* Transmitter on/off */
#define SYS600_OP_STATUS            0x10    /* Status update query (U selector) */
#define SYS600_OP_CARRIER           0x50    /* Change receiver carrier point */
#define SYS600_OP_RX_TO_TX          0x85    /* Copy current VFO Rx freq to Tx freq */
#define SYS600_OP_STEP              0x8E    /* Step freq up (D=0) / down (D=1) */
#define SYS600_OP_READ_METER        0xF7    /* Read digitized meter */
#define SYS600_OP_READ_FLAGS        0xFA    /* Read the 3 status flag bytes */

/*
 * Reply sizes (protocol.py:53-54; agent ref sec 8.2, sec 8.3, sec 14, sec 21).
 *
 * Read Flags reply : [F1][F2][F3][0x06][0x04]
 * Read Meter reply : [M][M][M][M][0xF7]
 * Status Update reply size keyed by U selector.
 */
#define SYS600_FLAGS_REPLY_LEN      5
#define SYS600_FLAGS_REPLY_TAIL0    0x06
#define SYS600_FLAGS_REPLY_TAIL1    0x04
#define SYS600_METER_REPLY_LEN      5
#define SYS600_METER_REPLY_TAIL     0xF7

#define SYS600_STATUS_SIZE_U0       1941
#define SYS600_STATUS_SIZE_U1       1
#define SYS600_STATUS_SIZE_U2       19
#define SYS600_STATUS_SIZE_U3       18
#define SYS600_STATUS_SIZE_U4       19
#define SYS600_STATUS_SIZE_DEFAULT  19  /* U=2, the recommended polling size (protocol.py:54) */

/*
 * Timing (agent ref sec 21: line 4800 8N2, <=200 ms between bytes of one
 * frame; radio.py:54-59: generous timeouts because a U=0 dump is 1941 bytes,
 * ~4.4 s at 4800 baud).  protocol.py itself defines no delays.
 */
#define SYS600_BAUD                4800
#define SYS600_INTERBYTE_MS        200
#define SYS600_READ_TIMEOUT_MS     8000
#define SYS600_WRITE_TIMEOUT_MS    4000
/* Gap after every write.  Hardware: the System 600 silently drops a frame
 * written back-to-back with the previous one (measuring a Status U=1 issued
 * 0 ms after Recall Memory: no reply; >=10 ms: always a reply). */
#define SYS600_POST_WRITE_DELAY_MS 20
/* After a VFO<->memory recall the radio ignores every command for ~150 ms
 * (probe9); pad the recall write by this much so the next command is heard. */
#define SYS600_MEM_RECALL_SETTLE_MS 250

/*
 * Documented frequency span implied by the 9-byte record frequency vectors
 * (protocol.py:56-60; agent ref sec 12, sec 20 decode check).
 */
#define SYS600_FREQ_MIN_HZ         100000u
#define SYS600_FREQ_MAX_HZ         30000000u

/*
 * 19-byte operating / memory data record layout (agent ref sec 11/17.6):
 * [mem(1)][Rx VFO(9)][Tx VFO(9)].
 *
 * NOTE: the manual's diagram shows Tx first and the prose headings say
 * "Rx VFO and Tx VFO".  Verified on real hardware (System 600, 2026-10):
 * 8Ah (Set Transmit Freq) changes ONLY the second sub-record while 0Ah
 * (Set Receive Freq) changes the first (and pulls Tx along when not split).
 * So Rx is first, Tx second - the prose order, not the diagram.
 */
#define SYS600_REC_LEN             19
#define SYS600_REC_FLAG0           0       /* d[0]: mode flags (0x20 VFO / 0xA0 MEM), NOT a channel */
#define SYS600_REC_RX              1       /* d[1..9]: Rx VFO sub-record */
#define SYS600_REC_TX              10      /* d[10..18]: Tx VFO sub-record */
#define SYS600_REC_RX_MODE         7       /* d[SYS600_REC_RX + SYS600_VFO_MODE] */
#define SYS600_REC_TX_MODE         16      /* d[SYS600_REC_TX + SYS600_VFO_MODE] */

/*
 * 9-byte VFO / memory sub-record (protocol.py:187-222; agent ref sec 12).
 * The manual labels the frequency field "(BCD)" but the quoted limits
 * (0x002710 = 100 kHz, 0x2DC6C0 = 30 MHz) are only correct as BINARY, so the
 * frequency is a 24-bit big-endian integer in 10 Hz units - the BCD/binary
 * trap: BCD for SET commands (0Ah/8Ah), binary for status records.
 */
#define SYS600_VFO_LEN             9
#define SYS600_VFO_BAND            0       /* BPF selection 00h..09h */
#define SYS600_VFO_FREQ            1       /* offsets 1..3: 24-bit big-endian, 10 Hz units */
#define SYS600_VFO_MODE            6       /* status mode byte (SYS600_MODE_ST_*) */

/*
 * MODE (0Ch) command parameter values (protocol.py:66-75; agent ref sec 6,
 * sec 17.10, sec 21 cheat sheet).
 */
#define SYS600_MODE_CMD_LSB        0x00    /* A3J-LSB */
#define SYS600_MODE_CMD_USB        0x01    /* A3J-USB */
#define SYS600_MODE_CMD_CW_W       0x02    /* A1A-W (CW wide) */
#define SYS600_MODE_CMD_CW_N       0x03    /* A1A-N (CW narrow) */
#define SYS600_MODE_CMD_AM         0x04    /* A3E-W (AM wide) */
#define SYS600_MODE_CMD_J2B_LSB_W  0x08
#define SYS600_MODE_CMD_J2B_USB_W  0x09
#define SYS600_MODE_CMD_J2B_LSB_N  0x0E
#define SYS600_MODE_CMD_J2B_USB_N  0x0F

/*
 * Status-data mode byte values, record offset 6 (protocol.py:77-84; agent
 * ref sec 12).  TRAP: this is a DIFFERENT table from the 0Ch command codes -
 * J2B collapses to a single status code 05 (its sideband lives in Flag2).
 * Do not mix the two tables.
 */
#define SYS600_MODE_ST_LSB         0x00
#define SYS600_MODE_ST_USB         0x01
#define SYS600_MODE_ST_CW_W        0x02
#define SYS600_MODE_ST_CW_N        0x03
#define SYS600_MODE_ST_AM          0x04
#define SYS600_MODE_ST_J2B         0x05

/*
 * Passband handling (radio.py:189-222 set_mode/get_mode).  A requested
 * passband <= 900 Hz selects the narrow CW/J2B filter variant; <= 0
 * selects wide (radio.py:190-191).  The passband reported by get_mode is
 * the radio.py surface-hint mapping: CW narrow -> 250 / wide -> 500,
 * J2B narrow -> 500 / wide -> 2400, SSB -> 2400, AM -> 6000 Hz.
 */
#define SYS600_NARROW_MAX_HZ       900
#define SYS600_PASSBAND_LSB_HZ     2400
#define SYS600_PASSBAND_USB_HZ     2400
#define SYS600_PASSBAND_CW_W_HZ    500
#define SYS600_PASSBAND_CW_N_HZ    250
#define SYS600_PASSBAND_AM_HZ      6000
#define SYS600_PASSBAND_J2B_W_HZ   2400
#define SYS600_PASSBAND_J2B_N_HZ   500

#endif /* SYSTEM600_H */
