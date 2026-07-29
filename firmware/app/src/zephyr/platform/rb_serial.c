/*
 * STS1000 "Meridian" — FE-5680A serial glue: UART7, the K1 RS-232/CMOS relay,
 * the RB_LOCK polarity bit, and a raw byte tunnel for the maintenance tool.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pin contract (docs/sts1000_firmware_hardware_interface.md §9,
 * docs/rb_rs232_interface.md):
 *
 *   UART7 TX          PB4
 *   UART7 RX          PE7
 *   RB_RS232_CMOS_SW  PE4   K1 DPDT relay. **Default LOW = RS-232**, which is
 *                           the level shifter (SN65C3221E, U46) path.
 *   RB_LOCK           PB13  opto-isolated lock status; polarity varies by
 *                           surplus variant, so it is a configuration bit.
 *
 * ---------------------------------------------------------------------------
 * The transceiver is powered from the Rb rail
 * ---------------------------------------------------------------------------
 *
 * docs/rb_rs232_interface.md: the SN65C3221E takes its supply from the Rb
 * domain, so with RB_PWR_EN (PB7) low there is nothing on the far side of the
 * relay and nothing to talk to. rb_serial_rail_up() reports that, core/fwupd's
 * rb_fwupd uses it to distinguish "rail down" from "dead unit", and this file
 * refuses to transmit while it is false — driving a level shifter that has no
 * supply is how a shifter dies.
 *
 * ---------------------------------------------------------------------------
 * Raw tunnel
 * ---------------------------------------------------------------------------
 *
 * A maintenance tool needs to put arbitrary bytes on UART7: these are surplus
 * units, variants differ, and the only way to find out what one speaks is to
 * talk to it. rb_serial_tunnel_open() suspends this file's own use of the port
 * and hands over the byte path; rb_serial_tunnel_close() takes it back.
 *
 * The tunnel *channel* (framing, authorisation, the MP command that carries it)
 * belongs to core/mp and is owned by another area. The contract this file
 * exposes to it is deliberately small and is documented here so that side can be
 * written against it without touching this file:
 *
 *   rb_serial_tunnel_open(cb, user)   suspend normal use; cb receives every
 *                                     octet that arrives, in ISR context
 *   rb_serial_tunnel_write(buf, len)  put octets on the wire
 *   rb_serial_tunnel_close()          resume normal use, flush the parser
 *   rb_serial_tunnel_active()         true while a tunnel holds the port
 *
 * As built, the caller is src/zephyr/console/mp_tunnel.c and the sink it
 * registers stages octets into a ring the console supervisor drains — which is
 * what makes an ISR-context callback usable at all. Nothing here knows that; the
 * only requirement this file imposes is the one stated above, that `cb` runs in
 * ISR context and must behave accordingly.
 *
 * While a tunnel is open, rb_serial_ops() still works but its transmit path
 * returns -EBUSY and its receive path yields nothing (the ISR routes to `cb`
 * instead of the ring): two writers on one UART is not a thing, and failing
 * loudly is better than interleaving frames. Opening a tunnel underneath a live
 * core/fwupd rubidium session is therefore refused only by that session failing
 * its next exchange — this file has no visibility of one, and inventing an
 * interlock here would put fwupd state in the wrong area.
 *
 * ---------------------------------------------------------------------------
 * The deferred fail-safe restore
 * ---------------------------------------------------------------------------
 *
 * rb_serial_set_mode() also refuses while a tunnel is open, for a stronger
 * reason than the transmit path's: throwing a mechanical relay under a live
 * passthrough breaks whatever the host is mid-transaction with, and the host is
 * the one party here nobody can ask. That refusal is correct and it stays.
 *
 * It had one consequence nobody had followed through, and it stranded the
 * relay: an override release that lands here during a tunnel is refused, the
 * engine counts the failure and forgets it, and the *next* release closes the
 * tunnel — leaving K1 in a commissioning position no lease holds. So a refused
 * move TO THE FAIL-SAFE POSITION is remembered and performed by
 * rb_serial_tunnel_close(), while a refused move away from it is simply refused.
 *
 * THAT RULE IS NOT DECIDED HERE. It lives in sts_rb_serial_policy.h — the
 * asymmetry, why an unconditional restore on close would be worse than the bug,
 * why the discharge must follow the retraction — and tests/host executes it
 * there, which is the only place it can be executed at all: this file needs a
 * devicetree, an ISR and k_msleep() to link. What stays here is the state the
 * policy describes (rb.relay), the pin, the settle, and the ordering of the two
 * steps in rb_serial_tunnel_close().
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fwupd/rb_fwupd.h"
#include "zephyr/platform/platform.h"
#include "zephyr/platform/sts_rb_serial_policy.h"
#include "zephyr/platform/sts_rbguard.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_rb_serial, CONFIG_STS1000_LOG_LEVEL);

/*
 * The policy restates the two relay positions rather than including platform.h,
 * which drags in devicetree and the GPIO API and would put it out of reach of
 * tests/host. These are what hold the two enumerations together: this file is
 * the one translation unit that sees both.
 */
BUILD_ASSERT((int)STS_RB_POS_RS232 == (int)RB_SERIAL_MODE_RS232);
BUILD_ASSERT((int)STS_RB_POS_CMOS == (int)RB_SERIAL_MODE_CMOS);
BUILD_ASSERT((int)STS_RB_POS__COUNT == (int)RB_SERIAL_MODE__COUNT);

#define ZEPHYR_USER DT_PATH(zephyr_user)
#define RB_UART_NODE DT_NODELABEL(uart7)

/* Receive ring. One second of 9600 8N1 is 960 octets; 512 is ample for the
 * short frames this protocol uses and for a tunnel that is drained promptly. */
#define RB_RX_RING 512U

static const struct device *const rb_uart = DEVICE_DT_GET(RB_UART_NODE);

static const struct gpio_dt_spec rb_mode =
	GPIO_DT_SPEC_GET(ZEPHYR_USER, rb_rs232_cmos_gpios);
static const struct gpio_dt_spec rb_lock =
	GPIO_DT_SPEC_GET(ZEPHYR_USER, rb_lock_gpios);
static const struct gpio_dt_spec rb_pwr_en =
	GPIO_DT_SPEC_GET(ZEPHYR_USER, rb_pwr_en_gpios);
/* Read-only here; pwrseq_exec.c owns driving it. See rb_serial_rail_up(). */
static const struct gpio_dt_spec rb_vcc_gate =
	GPIO_DT_SPEC_GET(ZEPHYR_USER, rb_vcc_gate_gpios);

/* ------------------------------------------------------------------ state -- */

static struct {
	bool ready;
	/** RB_LOCK is asserted when the pin reads LOW. Set from cfg. */
	bool lock_active_low;

	/* receive ring, written from the UART ISR */
	uint8_t ring[RB_RX_RING];
	volatile uint16_t head;
	volatile uint16_t tail;
	volatile uint32_t overruns;

	/*
	 * Tunnel hand-over. Both are read from the UART7 ISR and written from a
	 * thread, so they are volatile: without it the compiler is free to cache
	 * `tunnel_cb` across the ISR's loop, or to sink the `tunnel_user` store
	 * past the `tunnel_cb` store that publishes it. A single-core Cortex-M
	 * needs no more than that — the ISR runs on the same PE, so program order
	 * plus a compiler barrier is the whole ordering requirement.
	 */
	rb_serial_tunnel_cb_t volatile tunnel_cb;
	void *volatile tunnel_user;

	/*
	 * Where the relay stands, and whether a fail-safe restore is owed.
	 * sts_rb_serial_policy.h owns what those mean and what changes them; this
	 * file owns the pin they describe.
	 *
	 * Deliberately NOT volatile: unlike the two fields above it is never
	 * touched by the ISR. It is written by the same thread-context callers
	 * that already drive the relay pin without a lock.
	 */
	sts_rb_relay_t relay;

	uint32_t tx_bytes;
	uint32_t rx_bytes;
} rb;

static void ring_put(uint8_t b)
{
	uint16_t next = (uint16_t)((rb.head + 1U) % RB_RX_RING);

	if (next == rb.tail) {
		/*
		 * Drop the NEWEST octet rather than advancing the tail. Overwriting
		 * the oldest would corrupt a frame that is already half-parsed, and
		 * this protocol has no sync byte to recover from that with.
		 */
		rb.overruns++;
		return;
	}
	rb.ring[rb.head] = b;
	rb.head = next;
}

static void uart_cb(const struct device *dev, void *user)
{
	uint8_t buf[32];
	int n;

	ARG_UNUSED(user);

	if (!uart_irq_update(dev)) {
		return;
	}
	while (uart_irq_rx_ready(dev)) {
		n = uart_fifo_read(dev, buf, sizeof(buf));
		if (n <= 0) {
			break;
		}
		rb.rx_bytes += (uint32_t)n;
		if (rb.tunnel_cb != NULL) {
			/* The tunnel owns the byte path: hand it over untouched. */
			rb.tunnel_cb(rb.tunnel_user, buf, (size_t)n);
			continue;
		}
		for (int i = 0; i < n; i++) {
			ring_put(buf[i]);
		}
	}
}

/* --------------------------------------------------------------- lifecycle -- */

int rb_serial_init(bool lock_active_low)
{
	int rc;

	if (!device_is_ready(rb_uart)) {
		LOG_ERR("uart7 not ready");
		return -ENODEV;
	}

	/*
	 * Reset the model first and drive the pin to what it says, so the reset
	 * position comes from the one place that defines it. INACTIVE on an
	 * active-high pin is LOW, which is the RS-232 path through U46 — the
	 * documented default. A CMOS-direct variant is an explicit commissioning
	 * decision, never a reset state, and neither is an inherited obligation:
	 * sts_rb_relay_reset() clears the deferred restore in the same breath.
	 */
	sts_rb_relay_reset(&rb.relay);

	rc = gpio_pin_configure_dt(&rb_mode, sts_rb_relay_pin_active(rb.relay.pos)
						     ? GPIO_OUTPUT_ACTIVE
						     : GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		LOG_ERR("RB_RS232_CMOS_SW configure: %d", rc);
		return rc;
	}

	rc = gpio_pin_configure_dt(&rb_lock, GPIO_INPUT);
	if (rc != 0) {
		LOG_ERR("RB_LOCK configure: %d", rc);
		return rc;
	}

	rb.lock_active_low = lock_active_low;
	rb.head = 0U;
	rb.tail = 0U;
	rb.tunnel_cb = NULL;

	uart_irq_callback_user_data_set(rb_uart, uart_cb, NULL);
	uart_irq_rx_enable(rb_uart);

	rb.ready = true;
	LOG_INF("UART7 ready (RS-232 path, RB_LOCK active-%s)",
		lock_active_low ? "low" : "high");
	return 0;
}

int rb_serial_set_mode(uint8_t mode)
{
	int rc;

	if (!rb.ready) {
		return -ENODEV;
	}
	/*
	 * The verdict — and the deferral behind it — belongs to
	 * sts_rb_serial_policy.h, which tests/host executes. What is left here is
	 * the pin, the settle and the log line.
	 */
	switch (sts_rb_move_decide(mode, rb.tunnel_cb != NULL)) {
	case STS_RB_MOVE_INVALID:
		return -EINVAL;
	case STS_RB_MOVE_OWE:
		/*
		 * Refused — the relay must not move under a live passthrough —
		 * but a refused move to the fail-safe position is owed, not
		 * cancelled: without this, an override release that lands here
		 * during a tunnel leaves K1 in the commissioning position with
		 * no lease holding it and nothing left to move it.
		 * rb_serial_tunnel_close() pays it.
		 */
		rb.relay.restore_rs232_on_close = true;
		LOG_WRN("rb: RS-232 restore refused (tunnel open), "
			"deferred to tunnel close");
		return -EBUSY;
	case STS_RB_MOVE_REFUSE:
		/* CMOS under a tunnel: refused outright, and owed nothing. The
		 * asymmetry is argued in the policy header. */
		return -EBUSY;
	case STS_RB_MOVE_APPLY:
	default:
		break;
	}

	/*
	 * No "already in this position" short-circuit — see sts_rb_move_decide().
	 * rb.relay.pos is only updated *after* a successful gpio_pin_set_dt(), so
	 * a failed write deliberately leaves the model and the pin disagreeing:
	 * that is what makes the next attempt retry the write that is the fix.
	 */
	rc = gpio_pin_set_dt(&rb_mode, sts_rb_relay_pin_active(mode) ? 1 : 0);
	if (rc != 0) {
		return rc;
	}
	rb.relay.pos = mode;
	/*
	 * The relay is a mechanical DPDT part: give the contacts time to settle
	 * before anything is driven through them, or the first frame goes out
	 * during the bounce.
	 */
	k_msleep(RB_SERIAL_RELAY_SETTLE_MS);
	/* Whatever was mid-flight through the old path is meaningless now. */
	rb.head = 0U;
	rb.tail = 0U;
	return 0;
}

uint8_t rb_serial_mode(void)
{
	return rb.relay.pos;
}

/*
 * "Can the FE-5680A answer right now?" — and that is BOTH pins, not one.
 *
 * RB_PWR_EN (pwrseq step 8.12) brings up the buck. RB_VCC_GATE (step 8.14) is
 * what actually connects VCC_RB to the FE, and between them sit the soft-start
 * delay, the rail-safety window, two digipot write/verify rows with up to two
 * retries each, and the operating-rail ramp — see pwrseq.c:920-993. Throughout
 * all of that the buck is live and the rubidium is not connected.
 *
 * Testing RB_PWR_EN alone therefore answered "yes" during a window in which
 * nothing could possibly reply, and rb_fwupd_probe() takes that as licence to
 * conclude RB_CAP_NONE — "absent, check the cable" — and latch it for the rest
 * of the uptime, because cap returns to UNKNOWN only in rb_fwupd_init().
 *
 * 6fe50ca closed the RB_PWR_EN-low half of that and its commit message claimed
 * the whole of it. It did not: this half is the longer one, and an
 * unauthenticated G0 `fw.inventory` lands in it during ordinary bring-up.
 * Found by an adversarial reviewer refuting the fix, not by the fix's tests.
 *
 * Gating on both pins also keeps the probe one-shot. While the gate is shut the
 * probe declines immediately (-EHOSTDOWN, no bus traffic); once it is open,
 * silence is a real verdict worth latching. Without that, a probe that never
 * latched would let one unauthenticated inventory hold the MP engine lock for
 * rb_fwupd's full 1 s reply timeout, repeatably.
 */
bool rb_serial_rail_up(void)
{
	if (!rb.ready) {
		return false;
	}
	return sts_rb_serial_rail_up(gpio_pin_get_dt(&rb_pwr_en),
				     gpio_pin_get_dt(&rb_vcc_gate));
}

bool rb_serial_locked(void)
{
	int v;

	if (!rb.ready) {
		return false;
	}
	v = gpio_pin_get_dt(&rb_lock);
	if (v < 0) {
		return false;
	}
	/*
	 * gpio_pin_get_dt() already applies the devicetree's active-level flag, so
	 * this bit inverts on top of it — it exists because the *variant*, not the
	 * board, decides the polarity (docs/rb_rs232_interface.md), and that is
	 * only known at commissioning.
	 */
	return rb.lock_active_low ? (v == 0) : (v == 1);
}

/* ------------------------------------------------------------ core ports -- */

static int op_tx(void *user, const uint8_t *data, size_t len)
{
	ARG_UNUSED(user);

	if (!rb.ready) {
		return -ENODEV;
	}
	if (rb.tunnel_cb != NULL) {
		/* A tunnel holds the port. Two writers on one UART interleave
		 * frames, so fail rather than corrupt. */
		return -EBUSY;
	}
	if (!rb_serial_rail_up()) {
		/* No supply on the level shifter: driving it is how it dies. */
		return -ENODEV;
	}

	for (size_t i = 0U; i < len; i++) {
		uart_poll_out(rb_uart, data[i]);
	}
	rb.tx_bytes += (uint32_t)len;
	return 0;
}

static int op_rx(void *user, uint8_t *data, size_t cap)
{
	size_t n = 0U;

	ARG_UNUSED(user);

	if (!rb.ready) {
		return -ENODEV;
	}
	while ((n < cap) && (rb.tail != rb.head)) {
		data[n] = rb.ring[rb.tail];
		rb.tail = (uint16_t)((rb.tail + 1U) % RB_RX_RING);
		n++;
	}
	return (int)n;
}

static uint64_t op_now(void *user)
{
	ARG_UNUSED(user);
	return (uint64_t)k_uptime_get();
}

static void op_delay(void *user, uint32_t ms)
{
	ARG_UNUSED(user);
	k_msleep((int32_t)ms);
}

static bool op_rail(void *user)
{
	ARG_UNUSED(user);
	return rb_serial_rail_up();
}

static void op_audit(void *user, const char *what, int32_t value, int rc)
{
	ARG_UNUSED(user);

	/*
	 * Every trim attempt is audited at notice level, accepted or refused: a
	 * frequency reference that somebody adjusted is the first thing anyone
	 * asks about when the timing later looks wrong (spec §3.4).
	 */
	if (rc == 0) {
		LOG_INF("rb: %s value=%d", what, (int)value);
	} else {
		LOG_WRN("rb: %s value=%d rc=%d", what, (int)value, rc);
	}
}

const rb_fwupd_ops_t *rb_serial_ops(void)
{
	static const rb_fwupd_ops_t ops = {
		.tx = op_tx,
		.rx = op_rx,
		.now_ms = op_now,
		.delay_ms = op_delay,
		.rail_up = op_rail,
		.audit = op_audit,
		.user = NULL,
	};

	return &ops;
}

/* ---------------------------------------------------------------- tunnel -- */

int rb_serial_tunnel_open(rb_serial_tunnel_cb_t cb, void *user)
{
	if (!rb.ready) {
		return -ENODEV;
	}
	if (cb == NULL) {
		return -EINVAL;
	}
	if (rb.tunnel_cb != NULL) {
		return -EBUSY;
	}

	/* Drop anything buffered: it belongs to the previous owner of the port. */
	rb.head = 0U;
	rb.tail = 0U;
	rb.tunnel_user = user;
	/*
	 * Published last, and behind a barrier, so the ISR can never observe the
	 * callback paired with a stale user pointer. The volatile qualifiers stop
	 * the compiler caching either across the ISR's loop; this stops it moving
	 * the two stores past one another.
	 */
	compiler_barrier();
	rb.tunnel_cb = cb;

	LOG_WRN("rb: raw tunnel opened, normal UART7 use suspended");
	return 0;
}

int rb_serial_tunnel_write(const uint8_t *data, size_t len)
{
	if (!rb.ready) {
		return -ENODEV;
	}
	if ((data == NULL) && (len != 0U)) {
		return -EINVAL;
	}
	if (rb.tunnel_cb == NULL) {
		return -EPERM;
	}
	if (!rb_serial_rail_up()) {
		return -ENODEV;
	}

	for (size_t i = 0U; i < len; i++) {
		uart_poll_out(rb_uart, data[i]);
	}
	rb.tx_bytes += (uint32_t)len;
	return 0;
}

int rb_serial_tunnel_close(void)
{
	sts_rb_close_act_t act;
	int rc = 0;

	if (!rb.ready) {
		return -ENODEV;
	}

	act = sts_rb_close_decide(rb.tunnel_cb != NULL,
				  rb.relay.restore_rs232_on_close);
	if (act == STS_RB_CLOSE_NOTHING) {
		/* No tunnel to give back. Idempotent, by contract. */
		return 0;
	}

	/* Retracted first, so the ISR stops routing before the user pointer it
	 * would have been handed goes away. */
	rb.tunnel_cb = NULL;
	compiler_barrier();
	rb.tunnel_user = NULL;
	rb.head = 0U;
	rb.tail = 0U;

	LOG_INF("rb: raw tunnel closed, normal UART7 use resumed");

	/*
	 * Pay the deferred fail-safe restore, if the policy said one is owed.
	 *
	 * AFTER tunnel_cb is retracted, and that ordering is the mechanism, not
	 * a tidiness preference: sts_rb_move_decide() refuses whenever a tunnel
	 * holds the port, so calling set_mode() any earlier would refuse itself
	 * and re-arm the very latch it is meant to discharge. The verdict is read
	 * above, before the retraction, precisely so the retraction cannot change
	 * the answer between deciding and acting.
	 *
	 * The latch is cleared whether or not the move succeeds — see
	 * STS_RB_CLOSE_RESTORE. The port is given back either way: the tunnel is
	 * already closed above, and no GPIO failure may be allowed to keep a
	 * passthrough alive.
	 */
	if (act == STS_RB_CLOSE_RESTORE) {
		rb.relay.restore_rs232_on_close = false;
		rc = rb_serial_set_mode((uint8_t)RB_SERIAL_MODE_RS232);
		if (rc != 0) {
			LOG_ERR("rb: deferred K1 restore to RS-232 failed: %d",
				rc);
		} else {
			LOG_INF("rb: K1 restored to RS-232 (deferred)");
		}
	}

	return rc;
}

bool rb_serial_tunnel_active(void)
{
	return rb.ready && (rb.tunnel_cb != NULL);
}

void rb_serial_stats(uint32_t *tx, uint32_t *rx, uint32_t *overruns)
{
	if (tx != NULL) {
		*tx = rb.tx_bytes;
	}
	if (rx != NULL) {
		*rx = rb.rx_bytes;
	}
	if (overruns != NULL) {
		*overruns = rb.overruns;
	}
}

/* ------------------------------------------------- cross-area accessors --- */

/*
 * The console and MP planes may not include platform.h (ARCHITECTURE.md §2), so
 * the operator-facing view of this file crosses the seam through sts_app.h.
 * One snapshot rather than six getters: every caller wants the whole picture,
 * and six calls would let the relay position and the lock state come from
 * different instants — which is exactly the pair an operator is comparing when
 * they are commissioning the polarity.
 */

int sts_rb_serial_status(sts_rb_serial_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}

	(void)memset(out, 0, sizeof(*out));
	out->mode = rb_serial_mode();
	out->locked = rb_serial_locked();
	out->rail_up = rb_serial_rail_up();
	out->tunnel_open = rb_serial_tunnel_active();
	rb_serial_stats(&out->tx_bytes, &out->rx_bytes, &out->overruns);
	return 0;
}

int sts_rb_serial_set_mode(uint8_t mode)
{
	return rb_serial_set_mode(mode);
}
