# system600 implementation notes (Task 2) — extracted from Hamlib @ system600 branch

All refs are `file:line` in this repo. Zero prose beyond fact lines.

## 1. I/O signatures

- `write_block` prototype — src/iofunc.h:39-41:
  ```c
  extern HAMLIB_EXPORT(int) write_block(hamlib_port_t *p,
                                        const unsigned char *txbuffer,
                                        size_t count);
  ```
- `read_string` prototype — src/iofunc.h:53-59:
  ```c
  extern HAMLIB_EXPORT(int) read_string(hamlib_port_t *p,
                                        unsigned char *rxbuffer,
                                        size_t rxmax,
                                        const char *stopset,
                                        int stopset_len,
                                        int flush_flag,
                                        int expected_len);
  ```
  (also `write_block_sync` src/iofunc.h:43, `write_block_sync_error` src/iofunc.h:47, `read_string_direct` src/iofunc.h:61)
- Call idiom (backend → port) — rigs/yaesu/frg9600.c:151 and :194, verbatim:
  ```c
  return write_block(RIGPORT(rig), cmd, YAESU_CMD_LENGTH);
  ```
- Port params applied via **caps designated initializers** (model defaults), rigs/yaesu/frg9600.c:55-70 verbatim:
  ```c
  .rig_type =           RIG_TYPE_RECEIVER,
  .ptt_type =           RIG_PTT_NONE,
  .dcd_type =           RIG_DCD_NONE,
  .port_type =          RIG_PORT_SERIAL,
  .serial_rate_min =    4800,
  .serial_rate_max =    4800,
  .serial_data_bits =   8,
  .serial_stop_bits =   2,
  .serial_parity =      RIG_PARITY_NONE,
  .serial_handshake =   RIG_HANDSHAKE_NONE,
  .write_delay =        0,
  .post_write_delay =   300,
  .timeout =            2000,
  .retry =              0,
  ```
  → 4800 8N2 model default lives in `serial_*` caps fields (frg9600.c:59-64).
- `rig_set_conf` override mechanism: NOT FOUND — tried `grep -n "write_block\|read_string\|serial_open\|port_open\|rig_set_conf\|4800\|parity" rigs/yaesu/frg9600.c` (only write_block + caps defaults hit) and `grep -n "set_conf\|RIG_CONF\|confparm\|port settings\|serial settings" README.developer` (no hits).
- (Supplement) Prototype location confirmed: `write_block`/`read_string` are declared ONLY in the private `src/iofunc.h` (39, 43, 47, 53, 61), not in `include/hamlib/`; `grep -rn "write_block\|read_string" include/ src/` hits headers exactly at src/iofunc.h:39/43/47 (write_block family) and src/iofunc.h:53/61 (read_string family); implementation `read_string_generic` src/iofunc.c:1312.
- (Supplement) How frg9600 reads replies: **it doesn't** — `grep -in "read\|frame\|recv" rigs/yaesu/frg9600.c` matches only the comment at frg9600.c:43 ("Also this struct is READONLY!"). FRG9600 is write-only: caps registers only `.set_freq` and `.set_mode` (frg9600.c:128-129), no get_* hooks, so no read idiom exists there. Read idiom reference (netrigctl, verbatim): `ret = read_string(rp, (unsigned char *) buf, BUF_MAX, "\n", 1, 0, 1);` — rigs/dummy/netrigctl.c:326 (stopset `"\n"`, stopset_len 1, flush_flag 0, expected_len 1).

## 2. Required rig_caps fields

Struct: `struct rig_caps {` rig.h:2355 … `};` rig.h:2721. Field lines: rig.h:2356–2452 (data), 2459–2462 (lifecycle), 2469–2657+ (API hooks).

Data fields:

- `rig_model` → `rig_model_t` → system600: TBD (RIG_MODEL_SYSTEM600, §5)
- `model_name` → `const char *` → system600: TBD (e.g. "SYSTEM600")
- `mfg_name` → `const char *` → system600: TBD
- `version` → `const char *` → system600: TBD
- `copyright` → `const char *` → system600: TBD
- `status` → `enum rig_status_e` → system600: TBD
- `rig_type` → `int` → system600: TBD (frg9600 uses RIG_TYPE_RECEIVER, frg9600.c:55)
- `ptt_type` → `ptt_type_t` → system600: 0 (RIG_PTT_NONE=0, rig.h:739)
- `dcd_type` → `dcd_type_t` → system600: 0
- `port_type` → `rig_port_t` → system600: RIG_PORT_SERIAL (frg9600.c:58)
- `serial_rate_min` → `int` → system600: 4800 (frg9600.c:59)
- `serial_rate_max` → `int` → system600: 4800 (frg9600.c:60)
- `serial_data_bits` → `int` → system600: 8 (frg9600.c:61)
- `serial_stop_bits` → `int` → system600: 2 (frg9600.c:62)
- `serial_parity` → `enum serial_parity_e` → system600: RIG_PARITY_NONE (frg9600.c:63)
- `serial_handshake` → `enum serial_handshake_e` → system600: RIG_HANDSHAKE_NONE (frg9600.c:64)
- `write_delay` → `int` → system600: 0 (frg9600.c:65)
- `post_write_delay` → `int` → system600: TBD (frg9600.c:66 = 300)
- `timeout` → `int` → system600: TBD (frg9600.c:67 = 2000)
- `retry` → `int` → system600: 0 (frg9600.c:68)
- `has_get_func` → `setting_t` → system600: 0 (RIG_FUNC_NONE, frg9600.c:69)
- `has_set_func` → `setting_t` → system600: 0 (RIG_FUNC_NONE, frg9600.c:70)
- `has_get_level` → `setting_t` → system600: 0
- `has_set_level` → `setting_t` → system600: 0
- `has_get_parm` → `setting_t` → system600: 0
- `has_set_parm` → `setting_t` → system600: 0
- `level_gran` → `gran_t[RIG_SETTING_MAX]` → system600: 0
- `parm_gran` → `gran_t[RIG_SETTING_MAX]` → system600: 0
- `extparms` → `const struct confparams *` → system600: NULL
- `extlevels` → `const struct confparams *` → system600: NULL
- `extfuncs` → `const struct confparams *` → system600: NULL
- `ext_tokens` → `int *` → system600: NULL
- `ctcss_list` → `tone_t *` → system600: NULL
- `dcs_list` → `tone_t *` → system600: NULL
- `preamp` → `int[HAMLIB_MAXDBLSTSIZ]` → system600: 0
- `attenuator` → `int[HAMLIB_MAXDBLSTSIZ]` → system600: 0
- `max_rit` → `shortfreq_t` → system600: 0
- `max_xit` → `shortfreq_t` → system600: 0
- `max_ifshift` → `shortfreq_t` → system600: 0
- `agc_level_count` → `int` → system600: 0
- `agc_levels` → `enum agc_level_e[HAMLIB_MAX_AGC_LEVELS]` → system600: 0
- `announces` → `ann_t` → system600: 0
- `vfo_ops` → `vfo_op_t` → system600: 0
- `scan_ops` → `scan_t` → system600: 0
- `targetable_vfo` → `int` → system600: 0
- `transceive` → `int` → system600: 0
- `bank_qty` → `int` → system600: 0
- `chan_desc_sz` → `int` → system600: 0
- `chan_list` → `chan_t[HAMLIB_CHANLSTSIZ]` → system600: 0 (zero-ended)
- `rx_range_list1` → `freq_range_t[HAMLIB_FRQRANGESIZ]` → system600: TBD (REQUIRED — zero-terminated)
- `tx_range_list1` → `freq_range_t[HAMLIB_FRQRANGESIZ]` → system600: TBD (REQUIRED — zero-terminated)
- `rx_range_list2` … `tx_range_list5` (8 more, rig.h:2425–2432) → `freq_range_t[HAMLIB_FRQRANGESIZ]` → system600: 0
- `tuning_steps` → `struct tuning_step_list[HAMLIB_TSLSTSIZ]` → system600: TBD/0 (zero-terminated)
- `filters` → `struct filter_list[HAMLIB_FLTLSTSIZ]` → system600: TBD/0 (zero-terminated)
- `str_cal` → `cal_table_t` → system600: 0
- `swr_cal` → `cal_table_float_t` → system600: 0
- `alc_cal` → `cal_table_float_t` → system600: 0
- `rfpower_meter_cal` → `cal_table_float_t` → system600: 0
- `comp_meter_cal` → `cal_table_float_t` → system600: 0
- `vd_meter_cal` → `cal_table_float_t` → system600: 0
- `id_meter_cal` → `cal_table_float_t` → system600: 0
- `spectrum_scopes` → `struct rig_spectrum_scope[HAMLIB_MAX_SPECTRUM_SCOPES]` → system600: 0
- `spectrum_modes` → `enum rig_spectrum_mode_e[HAMLIB_MAX_SPECTRUM_MODES]` → system600: 0
- `spectrum_spans` → `freq_t[HAMLIB_MAX_SPECTRUM_SPANS]` → system600: 0
- `spectrum_avg_modes` → `struct rig_spectrum_avg_mode[HAMLIB_MAX_SPECTRUM_AVG_MODES]` → system600: 0
- `spectrum_attenuator` → `int[HAMLIB_MAXDBLSTSIZ]` → system600: 0
- `cfgparams` → `const struct confparams *` → system600: NULL
- `priv` → `const rig_ptr_t` → system600: NULL initially (per-backend priv_data assigned in rig_init; see §6 registration, README.developer §2/§3)

Lifecycle hooks (rig.h:2459–2462):

- `rig_init` → `int (*)(RIG *rig)` → system600: TBD (allocates priv)
- `rig_cleanup` → `int (*)(RIG *rig)` → system600: TBD (frees priv)
- `rig_open` → `int (*)(RIG *rig)` → system600: 0/NULL-allowed (frontend returns -RIG_ENIMPL when a hook is NULL, src/rig.c:8239-8241)
- `rig_close` → `int (*)(RIG *rig)` → system600: 0/NULL-allowed (src/rig.c:8259-8261)

API hooks — 17+ required per Task (rig.h:2469+):

- `set_freq` → `int (*)(RIG *, vfo_t, freq_t)` → rig.h:2469 → system600: TBD REQUIRED
- `get_freq` → `int (*)(RIG *, vfo_t, freq_t *)` → rig.h:2470 → system600: TBD REQUIRED
- `set_mode` → `int (*)(RIG *, vfo_t, rmode_t, pbwidth_t)` → rig.h:2472 → system600: TBD REQUIRED
- `get_mode` → `int (*)(RIG *, vfo_t, rmode_t *, pbwidth_t *)` → rig.h:2473 → system600: TBD REQUIRED
- `set_vfo` → `int (*)(RIG *, vfo_t)` → rig.h:2475 → system600: TBD REQUIRED
- `get_vfo` → `int (*)(RIG *, vfo_t *)` → rig.h:2476 → system600: TBD REQUIRED
- `set_ptt` → `int (*)(RIG *, vfo_t, ptt_t)` → rig.h:2478 → system600: 0/NULL (no PTT in scope)
- `get_ptt` → `int (*)(RIG *, vfo_t, ptt_t *)` → rig.h:2479 → system600: 0/NULL
- `get_dcd` → `int (*)(RIG *, vfo_t, dcd_t *)` → rig.h:2481 → system600: 0/NULL
- `set_rptr_shift`/`get_rptr_shift` → rig.h:2483-2484 → system600: 0/NULL
- `set_rptr_offs`/`get_rptr_offs` → rig.h:2486-2487 → system600: 0/NULL
- `set_split_freq` → `int (*)(RIG *, vfo_t, freq_t)` → rig.h:2489 → system600: TBD REQUIRED
- `get_split_freq` → `int (*)(RIG *, vfo_t, freq_t *)` → rig.h:2490 → system600: TBD REQUIRED
- `set_split_mode`/`get_split_mode` → rig.h:2492-2499 → system600: 0/NULL
- `set_split_freq_mode`/`get_split_freq_mode` → rig.h:2501-2510 → system600: 0/NULL
- `set_split_vfo` → `int (*)(RIG *, vfo_t, split_t, vfo_t)` → rig.h:2512 → system600: TBD REQUIRED
- `get_split_vfo` → `int (*)(RIG *, vfo_t, split_t *, vfo_t *)` → rig.h:2513 → system600: TBD REQUIRED
- `set_rit` → `int (*)(RIG *, vfo_t, shortfreq_t)` → rig.h:2515 → system600: TBD REQUIRED
- `get_rit` → `int (*)(RIG *, vfo_t, shortfreq_t *)` → rig.h:2516 → system600: TBD REQUIRED
- `set_xit`/`get_xit` → rig.h:2518-2519 → system600: 0/NULL
- `set_ts`/`get_ts` → rig.h:2521-2522 → system600: 0/NULL
- `set_dcs_code`/`get_dcs_code` → rig.h:2524-2525 → system600: 0/NULL
- `set_tone`/`get_tone` → rig.h:2527-2528 → system600: 0/NULL
- `set_ctcss_tone`/`get_ctcss_tone` → rig.h:2530-2531 → system600: 0/NULL
- `set_dcs_sql`/`get_dcs_sql` → rig.h:2533-2534 → system600: 0/NULL
- `set_tone_sql`/`get_tone_sql` → rig.h:2536-2537 → system600: 0/NULL
- `set_ctcss_sql`/`get_ctcss_sql` → rig.h:2539-2540 → system600: 0/NULL
- `power2mW`/`mW2power` → rig.h:2542-2551 → system600: 0/NULL
- `set_powerstat`/`get_powerstat` → rig.h:2553-2554 → system600: 0/NULL
- `reset` → rig.h:2556 → system600: 0/NULL
- `set_ant`/`get_ant` → rig.h:2558-2559 → system600: 0/NULL
- `set_level` → `int (*)(RIG *, vfo_t, setting_t, value_t)` → rig.h:2561 → system600: 0/NULL (unless RIG_LEVEL_* enabled via has_set_level)
- `get_level` → `int (*)(RIG *, vfo_t, setting_t, value_t *)` → rig.h:2562 → system600: TBD REQUIRED (STRENGTH/RAWSTR)
- `set_func`/`get_func` → rig.h:2564-2565 → system600: 0/NULL
- `set_parm`/`get_parm` → rig.h:2567-2568 → system600: 0/NULL
- `set_ext_level`/`get_ext_level` → rig.h:2570-2571 → system600: 0/NULL
- `set_ext_func`/`get_ext_func` → rig.h:2573-2574 → system600: 0/NULL
- `set_ext_parm`/`get_ext_parm` → rig.h:2576-2577 → system600: 0/NULL
- `set_conf`/`get_conf` → rig.h:2579-2580 → system600: 0/NULL
- `send_dtmf`/`recv_dtmf` → rig.h:2582-2583 → system600: 0/NULL
- `send_morse`/`stop_morse`/`wait_morse` → rig.h:2585-2587 → system600: 0/NULL
- `send_voice_mem`/`stop_voice_mem` → rig.h:2589-2590 → system600: 0/NULL
- `set_bank` → rig.h:2592 → system600: 0/NULL
- `set_mem`/`get_mem` → rig.h:2594-2595 → system600: 0/NULL
- `vfo_op` → rig.h:2597 → system600: 0/NULL
- `scan` → rig.h:2599 → system600: 0/NULL
- `set_trn`/`get_trn` → rig.h:2601-2602 → system600: 0/NULL
- `decode_event` → rig.h:2604 → system600: 0/NULL
- `set_channel`/`get_channel` → rig.h:2606-2607 → system600: 0/NULL
- `get_info` → `const char * (*)(RIG *rig)` → rig.h:2609 → system600: TBD REQUIRED
- `set_chan_all_cb`/`get_chan_all_cb` → rig.h:2611-2612 → system600: 0/NULL
- `set_mem_all_cb`/`get_mem_all_cb` → rig.h:2614-2623 → system600: 0/NULL
- `set_vfo_opt` → rig.h:2625 (comment: "only for Net Rigctl device") → system600: 0/NULL
- `rig_get_vfo_info` → rig.h:2626-2631 → system600: 0/NULL
- `set_clock`/`get_clock` → rig.h:2632-2633 → system600: 0/NULL
- `clone_combo_set` → `const char *` → rig.h:2635 → system600: NULL
- `clone_combo_get` → `const char *` → rig.h:2636 → system600: NULL
- `macro_name` → `const char *` → rig.h:2637 → system600: NULL
- `async_data_supported` → `int` → rig.h:2639 → system600: 0
- `read_frame_direct` → rig.h:2640-2643 → system600: 0/NULL
- `is_async_frame` → rig.h:2644-2647 → system600: 0/NULL
- `process_async_frame` → rig.h:2648-2651 → system600: 0/NULL
- `hamlib_check_rig_caps` → `char *` (integrity check field, rig.h comment "a constant value we can check for hamlib integrity") → system600: `HAMLIB_CHECK_RIG_CAPS` — idiom verbatim: `.hamlib_check_rig_caps = HAMLIB_CHECK_RIG_CAPS` rigs/yaesu/frg9600.c:131; macro `#define HAMLIB_CHECK_RIG_CAPS "HAMLIB_CHECK_RIG_CAPS"` rig.h:2334
- `get_conf2` → `int (*)(RIG *, hamlib_token_t, char *, int)` → system600: 0/NULL
- `password` → `int (*)(RIG *, const char *key1)` → system600: 0/NULL
- `set_lock_mode`/`get_lock_mode` → system600: 0/NULL
- `timeout_retry` → `short` → system600: 0 (0 = default, -1 = disable)
- `morse_qsize` → `short` → system600: 0
- `stream_caps` → `const struct rig_stream_caps *` → system600: NULL (comment rig.h: "0-terminated by type, or NULL"; only stream_open/stream_close required if used)
- `stream_open` → `int (*)(RIG *, struct rig_stream *)` → system600: 0/NULL
- `stream_close` → `int (*)(RIG *, struct rig_stream *)` → system600: 0/NULL
- `stream_read`/`stream_write`/`stream_drain`/`stream_pause`/`stream_resume`/`stream_apply_metadata`/`stream_hardware_time` → optional overrides (comment: "Optional") → system600: 0/NULL

(Line numbers for multi-line members are block starts; all within rig.h:2469–2720.)

## 3. Callback prototypes

MISMATCH NOTE: `grep -nE "typedef .*\(* *(set|get)_[a-z_]+ *\)" include/hamlib/rig.h` → no output (exit 1). There are NO callback typedefs; the signatures are struct members of `rig_caps` (rig.h:2355). `open`/`close` are named `rig_open`/`rig_close`. Verbatim from include/hamlib/rig.h:

```c
int (*set_freq)(RIG *rig, vfo_t vfo, freq_t freq);                                      /* rig.h:2469 */
int (*get_freq)(RIG *rig, vfo_t vfo, freq_t *freq);                                     /* rig.h:2470 */
int (*set_mode)(RIG *rig, vfo_t vfo, rmode_t mode, pbwidth_t width);                    /* rig.h:2472 */
int (*get_mode)(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width);                  /* rig.h:2473 */
int (*set_vfo)(RIG *rig, vfo_t vfo);                                                    /* rig.h:2475 */
int (*get_vfo)(RIG *rig, vfo_t *vfo);                                                   /* rig.h:2476 */
int (*set_ptt)(RIG *rig, vfo_t vfo, ptt_t ptt);                                         /* rig.h:2478 */
int (*get_ptt)(RIG *rig, vfo_t vfo, ptt_t *ptt);                                        /* rig.h:2479 */
int (*set_split_vfo)(RIG *rig, vfo_t vfo, split_t split, vfo_t tx_vfo);                 /* rig.h:2512 */
int (*get_split_vfo)(RIG *rig, vfo_t vfo, split_t *split, vfo_t *tx_vfo);               /* rig.h:2513 */
int (*set_split_freq)(RIG *rig, vfo_t vfo, freq_t tx_freq);                              /* rig.h:2489 */
int (*get_split_freq)(RIG *rig, vfo_t vfo, freq_t *tx_freq);                             /* rig.h:2490 */
int (*set_rit)(RIG *rig, vfo_t vfo, shortfreq_t rit);                                   /* rig.h:2515 */
int (*get_rit)(RIG *rig, vfo_t vfo, shortfreq_t *rit);                                  /* rig.h:2516 */
int (*set_level)(RIG *rig, vfo_t vfo, setting_t level, value_t val);                    /* rig.h:2561 */
int (*get_level)(RIG *rig, vfo_t vfo, setting_t level, value_t *val);                   /* rig.h:2562 */
const char * (*get_info)(RIG *rig);                                                     /* rig.h:2609 */
int (*rig_open)(RIG *rig);                                                              /* rig.h:2461 */
int (*rig_close)(RIG *rig);                                                             /* rig.h:2462 */
```

## 4. Return-code policy

MISMATCH NOTE: `grep "#define RIG_OK\|…"` → no output; codes are members of `enum rig_errcode_e`, include/hamlib/rig.h:204–233. Hamlib functions return the **negative** value on error, `RIG_OK` on success (rig.h:201-202).

- `RIG_OK = 0` — rig.h:205
- `RIG_EINVAL,` = 1 — rig.h:206
- `RIG_ENIMPL,` = 4 ("Function not implemented, but will be") — rig.h:209
- `RIG_ETIMEOUT,` = 5 — rig.h:210
- `RIG_EPROTO,` = 8 — rig.h:213
- `RIG_ENAVAIL,` = 11 ("Function not available") — rig.h:216
- Full enum rig.h:205-233 (ECONF=2, ENOMEM=3, EIO=6, EINTERNAL=7, ERJCTED=9, ETRUNC=10, ENTARGET=12, BUSERROR=13, BUSBUSY=14, EARG=15, EVFO=16, EDOM=17, EDEPRECATED=18, ESECURITY=19, EPOWER=20, ELIMIT=21, EACCESS=22, EEND last).
- Soft errors (no retry/re-init helps): `#define RIG_IS_SOFT_ERRCODE(errcode)` — rig.h:236-240, includes `-RIG_EINVAL || -RIG_ENIMPL || -RIG_ERJCTED || -RIG_ETRUNC || -RIG_ENAVAIL || …`.
- Wire token: `#define NETRIGCTL_RET "RPRT "` — rig.h:245.
- ENIMPL attribution: frontend returns `-RIG_ENIMPL` when the caps hook is NULL — src/rig.c:8239-8241 verbatim:
  ```c
  if (rig->caps->set_clock == NULL)
  {
      return -RIG_ENIMPL;
  }
  ```
  same pattern src/rig.c:8259-8261 (`get_clock`). Other frontend ENIMPL returns: src/rig.c:1041 (doc: "port type communication is not implemented yet"), 2837, 3719, 4359, 5081, 5327, 5481.
- has_* mask → ENIMPL verbatim line: NOT FOUND — tried `grep -n "ENIMPL" src/rig.c | head -12`; hits are NULL-hook and path returns above, no `has_get_level`-mask check captured in one attempt. Backend rule: a backend that does not implement a NULLed-out hook gets `-RIG_ENIMPL` from the frontend for free; if it registers a hook it must itself `return -RIG_ENIMPL;` for unsupported args (frg9600.c idiom: `return -RIG_EINVAL;` for wrong MODE, rigs/yaesu/frg9600.c:189).
- (Supplement) Actual frontend ENIMPL mechanism (verified `grep -n -B3 -A6 "RIG_ENIMPL" src/rig.c`) — the frontend returns `-RIG_ENIMPL` WITHOUT calling the backend in three cases:
  1. caps hook pointer is NULL (ternary guard): `retcode = caps->get_freq ? caps->get_freq(rig, RIG_VFO_CURR, tx_freq) : -RIG_ENIMPL;` — src/rig.c:5080-5081 (split-freq get path); identical guards for `set_mode` src/rig.c:5326-5327 and `get_mode` src/rig.c:5480-5481; `if (rig->caps->set_clock == NULL) { return -RIG_ENIMPL; }` src/rig.c:8239-8241 (rig_set_clock) and get_clock src/rig.c:8259-8261; `if (caps->get_dcd == NULL) { … RETURNFUNC(-RIG_ENIMPL); }` src/rig.c:4356-4359.
  2. caps `targetable_vfo` bitmask lacks the needed capability: ENIMPL returned in the block whose guard is `if ((caps->targetable_vfo & RIG_TARGETABLE_PTT) || vfo == RIG_VFO_CURR || …)` failing — src/rig.c:3719 with condition at :3722 (same idiom :4074).
  3. unimplemented port type in `rig_open`: doc `\retval -RIG_ENIMPL   port type communication is not implemented yet.` src/rig.c:1041 (function `rig_open` at :1045).
  No `has_get_*`/`has_set_*` mask→ENIMPL check was found in src/rig.c in the greps performed; the has_* masks gate which levels/funcs the frontend advertises, while the NULL-hook/targetable_vfo checks above are what produce -RIG_ENIMPL.

## 5. Registration checklist

FT600 (index 39) exact pattern lines — 3 edits + makefile:

1. `include/hamlib/riglist.h:165` (context 161-169), verbatim:
   ```c
   #define RIG_MODEL_FT600 RIG_MAKE_MODEL(RIG_YAESU, 39)
   ```
   Macro definitions: `#define MAX_MODELS_PER_BACKEND 1000` riglist.h:51; `#define RIG_MAKE_MODEL(a,b) (MAX_MODELS_PER_BACKEND*(a)+(b))` riglist.h:62; `#define RIG_BACKEND_NUM(a) ((a)/MAX_MODELS_PER_BACKEND)` riglist.h:65.
2. `rigs/yaesu/yaesu.h:56`, verbatim:
   ```c
   extern struct rig_caps ft600_caps;
   ```
   MISMATCH: README.developer §3.5 says `extern const struct rig_caps <mymodel>_caps;` — actual yaesu.h:56 is non-`const`.
3. `rigs/yaesu/yaesu.c:86` inside `initrigs_yaesu` (context 82-90: `rig_register(&ft736_caps); … rig_register(&ft600_caps); …`), verbatim:
   ```c
   rig_register(&ft600_caps);
   ```
   README.developer:693: `add "rig_register(&<mymodel>_caps);"` (§3.6).
4. `rigs/yaesu/Makefile.am:7` (YAESUSRC list, context 3-11), verbatim line:
   ```make
   	ft757gx.h ft600.h ft600.c ft736.c frg100.c frg100.h frg9600.c ft1000d.c \
   ```
   README.developer §3.4: "Add the new C file to the _SOURCES variable of the backend's Makefile.am".

New lines to add for SYSTEM600 (index 52):

- (Supplement) **index 52 verified free in-tree** — `grep -n "RIG_YAESU, 5[0-9]" include/hamlib/riglist.h` returns only `riglist.h:176:#define RIG_MODEL_FT9000OLD RIG_MAKE_MODEL(RIG_YAESU, 50)` and `riglist.h:177:#define RIG_MODEL_FTX1 RIG_MAKE_MODEL(RIG_YAESU, 51)`; last used: 51 FTX1, riglist.h:177. No `RIG_YAESU, 52` exists.
- riglist.h (after neighbors of :165): `#define RIG_MODEL_SYSTEM600 RIG_MAKE_MODEL(RIG_YAESU, 52)`  ← skeleton ruling: index 52; backend macro `RIG_YAESU` assumed from the FT600 template hits (verify if system600 gets its own backend).
- yaesu.h (after :56): `extern struct rig_caps system600_caps;`
- yaesu.c (in initrigs_yaesu list, near :86): `rig_register(&system600_caps);`
- Makefile.am YAESUSRC (near :7): add `system600.h system600.c`.

## 6. caps → dump_caps/dump_state

- `\chk_vfo` send — rigs/dummy/netrigctl.c:282: `SNPRINTF(cmd, sizeof(cmd), "\\chk_vfo\n");` then `netrigctl_transaction`; parse netrigctl.c:285-287: `sscanf(buf, "%d", &priv->rigctld_vfo_mode)` → `STATE(rig)->vfo_opt = priv->rigctld_vfo_mode;`.
- What `\chk_vfo` emits (server side) — doc/man1/rigctld.1:1704-1714: `chk_vfo` "Returns Status as 1 if vfo option is on and 0 if vfo option is off. This command reflects the -o switch for rigctl and rigctld and can be dynamically changed by set_vfo_opt." → response is a single line "1" or "0".
- `\dump_state` send — netrigctl.c:309: `SNPRINTF(cmd, sizeof(cmd), "\\dump_state\n");`. Parse order (netrigctl.c:309–510), each line must be non-garbage or the backend bails `-RIG_EPROTO`:
  1. line 1: `prot_ver = atoi(buf);` compared `if (prot_ver < RIGCTLD_PROT_VER)` with `#define RIGCTLD_PROT_VER 0` → else `-RIG_EPROTO` (netrigctl.c:~319-325).
  2. one line read (netrigctl.c:~327-333), then `rs->deprecated_itu_region = atoi(buf);` (netrigctl.c:~335).
  3. `rx_range_list[HAMLIB_FRQRANGESIZ]` loop — one line each, `dummy_parse_rigctl_range(buf, &rs->rx_range_list[i])`, break on `RIG_IS_FRNG_END` (netrigctl.c:~337-352).
  4. `tx_range_list[HAMLIB_FRQRANGESIZ]` loop — same, plus copies into `rig->caps->tx_range_list1->{startf,endf,modes,low_power,high_power,vfo,ant}` (netrigctl.c:~354-379).
  5. `tuning_steps[HAMLIB_TSLSTSIZ]` loop — `sscanf(buf, "%"SCNXll"%ld", &modes, &ts)`, must return 2, break on `RIG_IS_TS_END` (netrigctl.c:~381-402).
  6. `filters[HAMLIB_FLTLSTSIZ]` loop — `sscanf(buf, "%"SCNXll"%ld", &modes, &width)`, must return 2, break on `RIG_IS_FLT_END` (netrigctl.c:~404-444).
  7. `max_rit` → `atol(buf)` (netrigctl.c:~455); `max_xit` → `atol` (~465); `max_ifshift` → `atol` (~475); `announces` → `atoi` (~485); `preamp[7]` → `sscanf("%d%d%d%d%d%d%d")` into `rs->preamp[0..6]`, copied to `rig->caps->preamp[]` (netrigctl.c:~490-510).
  `chan_list` parse is `#if 0` / TODO (netrigctl.c:~447-450).
- `\dump_caps` behavior — doc/man1/rigctld.1:1577-1584: "Not a real rig remote command, it just dumps capabilities, i.e. what the backend knows about this model, and what it can do." + "This command will produce many lines of output so be very careful if using a fixed length array!" (rigctld.1:1582); rigctld.1:2096-2103: "A notable exception is the dump_caps command which returns many lines of key:value pairs."; last record of each block is `RPRT x\n` (rigctld.1:2130); Perl example `print $socket "\\dump_caps\n";` (rigctld.1:600).
- RPRT format — rigctld.1:67-70: on error, one line `RPRT x` where x is the negative error code; valueless commands reply `RPRT x` with x=0 on success; each line terminated with `\n`. rigctld.1:2066-2068: set commands → one line `RPRT x\n`, x=0 success. rigctld.1:2076-2079: get commands → text values one per line; on error `RPRT x\n`.
- Extended Response Protocol — rigctld.1:78-81: echoes the received command string as a header, returned values as key: value pairs, `RPRT x` as end-of-response marker; rigctld.1:2125-2131: first record = long command name + value(s) terminated by response separator, last record = `RPRT x\n`. Separator examples: `get_mode:;Mode: USB;Passband: 2400;RPRT 0` (rigctld.1:2242), `|` variant (rigctld.1:2252).
- rigctl model selection — doc/man1/rigctl.1:78-79: `-m, --model = id` — "Select radio model number. Defaults to dummy rig."; rigctl.1:81: "See model list (use `rigctl -l`)."
- Enforceable coding rules used for the backend (README.coding_style §2, head -40): no tabs for indentation (spaces only, ASCII 0x20); **indentation = 4 spaces**; "Bare conditionals shall be avoided. All conditionals--'if', 'else', 'for', 'while', and 'do while' shall have their statements enclosed in braces."; README.coding_style:113: "No space between a function name and its opening parentheses." Naming conventions: NOT FOUND — tried `grep -n "naming\|Naming\|CamelCase\|snake"` and `grep -n "identifier\|variable name\|function name\|lowercase\|Capitaliz\|capitaliz" README.coding_style` (Supplement §7: confirmed no identifier-naming scheme exists; only paren/pointer spacing rules).

## 7. Style rules (README.coding_style, enforceable only)

- No tab characters (0x09) for indentation/alignment — spaces (0x20) only; `'\t'` allowed in rig_debug()/static strings — README.coding_style:24-28.
- Indentation shall be 4 spaces — README.coding_style:30-31.
- All conditionals (`if`, `else`, `for`, `while`, `do while`) shall have their statements enclosed in braces ("bare conditionals" avoided; rare exceptions per §2 NOTE) — README.coding_style:39-43.
- No space between a function name and its opening parenthesis — README.coding_style:113.
- No space between pointer operator `*` and the pointer name — README.coding_style:115-116.
- Identifier naming conventions: NOT FOUND — grep naming/CamelCase/snake/identifier/variable name/function name/lowercase/Capitaliz + read of sections 0–3.4; only the two pointer/paren spacing rules above govern names/spacing.
