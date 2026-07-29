/*
 * STS1000 "Meridian" — core/mp: capability manifest (FMT §4).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11, const data only. The manifest is the contract the Field
 * Maintenance Tool discovers the box through: every object it may observe or
 * command, with the object's kind, its guard class, its capability envelope
 * (range/step/enum/unit), the interlocks that constrain it, and a description
 * carrying the schematic designator so a technician standing at the board can
 * tie a manifest entry to a part.
 *
 * The table is build-time const (flash, not RAM) and is serialised to JSON on
 * demand, paged by object index rather than by byte offset — a JSON array
 * fragment is a natural paging unit and it means the device never has to hold
 * the whole ~14 KB document in RAM.
 *
 * As-built corrections against the spec's illustrative examples, all of which
 * are hardware facts and not choices:
 *
 *   - There is **no I/O expander**. The spec's `U47`/`EXP_RESET` object does
 *     not exist: the power-good bits, the load-switch fault flags and the panel
 *     inputs are a direct 1 kHz GPIOF/GPIOG scan (`core/fault`), exposed here as
 *     the `sensor.pg`, `sensor.ina.alert`, `sensor.en.fault` and
 *     `sensor.buttons` bitmap objects.
 *   - The GNSS current monitor is at **0x4A**, not 0x44 — SHT45 U72 owns 0x44.
 *   - The status RGB is **D5** on TIM4 CH1..3 (PD12/13/14), not D36.
 *   - The panel-LED dimmer is **PE0 / LPTIM2_CH2**.
 *   - `pwr.rb.vset_mv` is bounded by the *configured* ceiling
 *     `pwr.rb.vmax.mv` (cfg 0x0703, default 15000 mV), not by the supply's
 *     24.45 V full scale, over the transfer VCC_RB = 24.45 - 6.645*VCTRL.
 *
 * Rail identity (name, designator, I2C address, shunt) is read from
 * `ina228_rail_tbl` rather than copied, so the manifest cannot drift from the
 * as-built monitor table.
 */

#ifndef STS1000_CORE_MP_MP_MANIFEST_H_
#define STS1000_CORE_MP_MP_MANIFEST_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Manifest layout version. Bump on any change to the emitted JSON shape. */
#define MP_MANIFEST_VER 1U

/* ------------------------------------------------------------ guard classes */

/**
 * Guard classes (FMT §5.2, §5.3).
 *
 * The escalation is cumulative: every requirement of a lower class also applies.
 * Each class above G0 also carries a **minimum role**, granted at session open
 * from the credential store the web and console planes share; the typed serial
 * and phrase are anti-mistake interlocks *on top of* that authentication, never
 * a substitute for it (the serial is printed on the label and returned by
 * `hello`, so it authenticates nobody).
 *
 *   G0  observe only, or a change with no service consequence. No session, no
 *       role.
 *   G1  an open session with a fresh keepalive, role >= operator.
 *   G2  G1 + role >= admin + a typed device-serial confirmation + every declared
 *       interlock must pass. This is the class for anything that can interrupt
 *       served time or damage an attached instrument.
 *   G3  G2 + a typed phrase + a hold: the request arms, and a second request
 *       carrying the same nonce completes it only after the hold has elapsed.
 *       Reserved for actions that stop the clock or the board.
 *
 * The role floor is enforced in mp_ovr_guard(), which every guarded RPC reaches
 * through one function (guard_or_fail() in mp_rpc.c). No object carrying
 * MP_OF_WRITE, MP_OF_OVERRIDE or MP_OF_PULSE may be declared G0 — that would
 * route actuation around the check, because a G0 request is answered for a
 * session with no role at all (MP_ROLE_NONE), including one that never presented
 * a credential. The rule has no exceptions and
 * tests/host/test_mp_manifest.c::test_writable_objects_are_guarded fails the
 * build if one is ever introduced; `ui.identify` was the single exception and is
 * now G1 (see the note on its row in mp_manifest.c).
 */
typedef enum {
	MP_GUARD_G0 = 0,
	MP_GUARD_G1 = 1,
	MP_GUARD_G2 = 2,
	MP_GUARD_G3 = 3,
	MP_GUARD_COUNT = 4,
} mp_guard_t;

/** Guard class name as it appears in the manifest ("G0".."G3"). Never NULL. */
const char *mp_guard_name(uint8_t g);

/* --------------------------------------------------------------- interlocks */

/**
 * Device-side interlocks (FMT §5.5).
 *
 * Every one of these is enforced on the device and none is overridable from the
 * host: an interlock the tool could switch off is documentation, not a safety
 * property. The manifest publishes which interlocks bind each object so the
 * tool can explain a refusal before it happens.
 */
#define MP_ILK_RB_VMAX (1U << 0)   /**< clamp VCC_RB to cfg pwr.rb.vmax.mv */
#define MP_ILK_RB_OV (1U << 1)     /**< refuse while the 26 V OV latch is set */
#define MP_ILK_RB_VERIFY (1U << 2) /**< post-set INA 0x47 read-back + revert */
#define MP_ILK_RB_WARM (1U << 3)   /**< needs OCXO warm + supercaps charged */
#define MP_ILK_FAN_FLOOR (1U << 4) /**< duty floor from the thermal loop */
#define MP_ILK_RELAY_OK (1U << 5)  /**< no force-energize while a fault is up */
#define MP_ILK_DISP_OFF (1U << 6)  /**< display rail minimum off-time */
#define MP_ILK_WDT_LIVE (1U << 7)  /**< arm only with the liveness gate healthy */
#define MP_ILK_TUNNEL (1U << 8)    /**< suspends firmware's use of that UART */
#define MP_ILK_MUX_GUARD (1U << 9) /**< clock handoff needs extref + rb_lock */
#define MP_ILK_DAC_PARK (1U << 10) /**< discipline must be parked first */

/** Number of defined interlocks. */
#define MP_ILK_COUNT 11U

/** Every defined interlock bit. */
#define MP_ILK_ALL ((1U << MP_ILK_COUNT) - 1U)

/**
 * Stable interlock name, e.g. "rb.vmax". NULL when @p bit is not a single
 * defined interlock bit — which is what makes the manifest self-consistency
 * test able to prove every published interlock resolves.
 */
const char *mp_ilk_name(uint32_t bit);

/** One-line explanation of @p bit, for the tool's refusal message. */
const char *mp_ilk_reason(uint32_t bit);

/* -------------------------------------------------------------------- kinds */

/** Object kinds. The kind fixes how `value` is carried in JSON. */
typedef enum {
	MP_KIND_BOOL = 0, /**< true/false */
	MP_KIND_ENUM,     /**< integer index into `enums` */
	MP_KIND_PCT,      /**< integer percent, 0..100 */
	MP_KIND_MV,       /**< integer millivolts */
	MP_KIND_CODE,     /**< raw device code (DAC, digipot wiper) */
	MP_KIND_PULSE,    /**< momentary: value is a duration in ms */
	MP_KIND_SCALAR,   /**< integer reading in `unit` */
	MP_KIND_REAL,     /**< fractional reading in `unit` */
	MP_KIND_RAIL,     /**< composite INA228 reading: mv/ma/mw/diag/valid */
	MP_KIND_BITS,     /**< bitmap reading; `enums` names the bits */
	MP_KIND_TEXT,     /**< string reading */
	MP_KIND_COUNT,
} mp_kind_t;

/** Kind name as emitted in the manifest. Never NULL. */
const char *mp_kind_name(uint8_t k);

/* ------------------------------------------------------------------- groups */

/**
 * Manifest groups.
 *
 * The five control groups follow FMT §6.1-6.5 and the sensor group §7.1; the
 * names are this project's, because the object *inventory* had to be derived
 * from the as-built pin contract rather than copied from the spec's examples.
 */
typedef enum {
	MP_GRP_POWER = 0, /**< §6.1 rails, enables, Rb supply, PoE */
	MP_GRP_REF,       /**< §6.2 references, clock mux, OCXO steering, relay */
	MP_GRP_GNSS,      /**< §6.3 receiver control */
	MP_GRP_PANEL,     /**< §6.4 display, panel LEDs, status RGB */
	MP_GRP_SYSTEM,    /**< §6.5 watchdog, NOR, PHY, board power */
	MP_GRP_SENSOR,    /**< §7.1 observables */
	MP_GRP_COUNT,
} mp_group_t;

/** Group name as emitted in the manifest. Never NULL. */
const char *mp_group_name(uint8_t g);

/* -------------------------------------------------------------------- flags */

#define MP_OF_READ (1U << 0)      /**< obj.get answers */
#define MP_OF_WRITE (1U << 1)     /**< obj.set accepted (persistent where cfg) */
#define MP_OF_OVERRIDE (1U << 2)  /**< obj.override accepted (leased) */
#define MP_OF_PULSE (1U << 3)     /**< obj.pulse accepted */
#define MP_OF_ACTIVE_LOW (1U << 4)/**< the net is asserted low */
#define MP_OF_CFG (1U << 5)       /**< `cfg_key` is a real cfg key id */

/**
 * Published, but nothing is wired behind it: the device refuses (FMT §4).
 *
 * The manifest is the whole of what the host knows — FMT §4 says the tool
 * generates its UI from it and hardcodes nothing — so an object with no
 * implementation behind it renders as a working control and fails on use. This
 * bit is how the device says so in advance, and it is not decoration: the
 * shipped glue answers `mp_wiring_t::obj_supported` straight out of it, and
 * `obj.set`/`obj.override`/`obj.pulse` refuse before the guard runs, so an
 * unimplemented G3 object no longer spends a typed phrase and a hold before
 * admitting it does nothing.
 *
 * WHAT IT IS THE COMPLEMENT OF. Exactly the dispatch in the Zephyr glue —
 * `obj_apply()`, `obj_pulse()` and `obj_read()` in `zephyr/console/mp_glue.c`,
 * plus `cfg_write()` in `mp_rpc.c` for a `MP_OF_CFG` object — read per object
 * against the operation that object exists for:
 *
 *   - an object declaring MP_OF_WRITE, MP_OF_OVERRIDE or MP_OF_PULSE is
 *     deferred when **none** of its declared mutations reaches an actuator;
 *   - a read-only object is deferred when `obj_read()` does not answer it.
 *
 * One bit cannot say more than that. Where an object both mutates and reads,
 * the bit describes the MUTATION, because that is the operation whose absence
 * a technician discovers by acting on the board. Three objects are today
 * mutable-and-wired with no read-back accessor — `ui.identify`,
 * `pwr.poe.kill`, `pwr.rb.ov.reset`, the first a write-only beacon and the
 * other two momentary pulses with no state to read — and
 * tests/host/test_mp_deferred.c pins that set by name so it cannot silently
 * grow.
 *
 * The bit describes WIRING, not availability. An object without it answers
 * every operation it declares; it may still report -EIO when a sensor sweep
 * has not landed, which is a transient and is deliberately a different error
 * from MP_E_NOTSUP.
 */
#define MP_OF_DEFERRED (1U << 6)

/* ------------------------------------------------------------------ objects */

/** One manifest object. All pointers are to string literals in flash. */
typedef struct {
	const char *id;    /**< dotted identifier, unique across the table */
	uint8_t kind;      /**< mp_kind_t */
	uint8_t guard;     /**< mp_guard_t */
	uint8_t group;     /**< mp_group_t */
	uint8_t flags;     /**< MP_OF_* */
	int32_t min;       /**< inclusive lower bound (kind-dependent) */
	int32_t max;       /**< inclusive upper bound */
	int32_t step;      /**< granularity; 0 when continuous/not applicable */
	const char *unit;  /**< SI-ish unit tag, or NULL */
	const char *enums; /**< comma-separated names for ENUM/BITS, else NULL */
	uint16_t cfg_key;  /**< cfg key id when MP_OF_CFG, else 0 */
	uint32_t ilk;      /**< MP_ILK_* mask */
	const char *net;   /**< canonical net name, or NULL */
	const char *desc;  /**< includes the schematic designator */
} mp_obj_t;

/** The object table. Order is the manifest order and the hash order. */
extern const mp_obj_t mp_objs[];

/** Number of objects in mp_objs[]. */
size_t mp_obj_count(void);

/** Object @p i, or NULL when out of range. */
const mp_obj_t *mp_obj_at(size_t i);

/**
 * Look up an object by id.
 *
 * @retval >=0      Index into mp_objs[].
 * @retval -EINVAL  @p id is NULL.
 * @retval -ENOENT  No such object.
 */
int mp_obj_find(const char *id);

/** Objects in @p group. 0 for an out-of-range group. */
size_t mp_obj_count_in(uint8_t group);

/**
 * Check @p v against object @p idx's published envelope.
 *
 * Every write path must go through this, not just the leased one: a manifest
 * range the device does not actually enforce is documentation, and a cfg-backed
 * object would otherwise be bounded only by its schema row, which may be wider
 * than what the manifest promises.
 *
 * @retval 0         Inside the envelope.
 * @retval -EINVAL   @p idx out of range.
 * @retval -ERANGE   Outside min..max, or not 0/1 for a boolean.
 * @retval -ENOTSUP  The kind is not settable (REAL/RAIL/BITS/TEXT).
 */
int mp_obj_check_value(size_t idx, int32_t v);

/* -------------------------------------------------------------- serialising */

/**
 * Serialise one object as a JSON object into @p buf.
 *
 * This is the canonical form: the manifest document is the concatenation of
 * these, and mp_manifest_hash() is taken over exactly these bytes.
 *
 * @retval >=0      Bytes written, excluding the NUL.
 * @retval -EINVAL  Bad argument or index.
 * @retval -ENOSPC  @p cap too small.
 */
int mp_manifest_obj_json(size_t idx, char *buf, size_t cap);

/**
 * Largest single-object JSON form the table can produce, including the NUL.
 *
 * Dominated by `sensor.alarms`, whose 49 bit names run to about 1.1 KB; every
 * other object is under 400 B. A fixed bound is what keeps the emitter
 * allocation-free, and the manifest self-consistency test proves no object
 * exceeds it.
 */
#define MP_MANIFEST_OBJ_JSON_MAX 1408U

/**
 * Emit a page of the manifest as a JSON array fragment (no enclosing brackets).
 *
 * @param from     First object index to emit.
 * @param buf      Destination.
 * @param cap      Capacity of @p buf including the NUL.
 * @param out_len  Receives the bytes written, excluding the NUL.
 * @param out_next Receives the next index to request; equals mp_obj_count()
 *                 when the page completed the table.
 *
 * @retval >=0      Number of objects emitted. 0 with `*out_next == from` means
 *                  @p cap cannot hold even one object.
 * @retval -EINVAL  Bad argument, or @p from past the end.
 */
int mp_manifest_page(size_t from, char *buf, size_t cap, size_t *out_len,
		     size_t *out_next);

/**
 * Content hash of the manifest.
 *
 * CRC-32/ISO-HDLC (`sts_crc32_ieee`) over MP_MANIFEST_VER as one decimal ASCII
 * byte sequence followed by every object's canonical JSON form in table order.
 * A CRC rather than a digest because the manifest is not a security object — the
 * host uses the hash to cache the document and to detect a firmware change, and
 * a 32-bit check is enough for both.
 *
 * The serialisation scratch is caller-supplied so this never puts a
 * MP_MANIFEST_OBJ_JSON_MAX frame on the calling thread's stack; `mp_init()`
 * lends it the reply buffer and caches the result.
 *
 * @param scratch  Working buffer, at least MP_MANIFEST_OBJ_JSON_MAX bytes.
 * @param cap      Capacity of @p scratch.
 * @return         The hash, or 0 when @p scratch is NULL or too small.
 */
uint32_t mp_manifest_hash(char *scratch, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MP_MP_MANIFEST_H_ */
