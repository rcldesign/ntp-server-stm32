/*
 * STS1000 "Meridian" — core/atecc: ATECC608B command protocol.
 *
 * Platform-neutral C11, no dynamic allocation, all state in a caller-owned
 * context (ARCHITECTURE.md §4, dependency edge `atecc→util`). This module is
 * the *protocol*: it frames command packets, computes the Microchip CRC-16,
 * parses and classifies responses, and sequences the wake/idle/sleep tokens.
 *
 * It performs no I²C. Every byte leaves and arrives through @ref atecc_bus_t,
 * which the Zephyr glue (`src/zephyr/storage/sts_atecc.c`) binds to I²C1 at
 * 0x60 (interface ref §4). A host test binds it to a scripted fake device,
 * which is how the state machine, the retry paths and the malformed-response
 * handling are covered without hardware.
 *
 * ---------------------------------------------------------------------------
 * Wire format (ATECC608B datasheet, "I/O Transactions")
 * ---------------------------------------------------------------------------
 *
 * Every I²C write starts with a *word address* selecting what follows:
 *
 *   0x00  Reset  — rewind the output-buffer read pointer
 *   0x01  Sleep  — enter low-power sleep, clearing TempKey and the RNG seed
 *   0x02  Idle   — enter idle, clearing TempKey but keeping the seed
 *   0x03  Command— a command packet follows
 *
 * A command packet is
 *
 *   count(1) opcode(1) param1(1) param2(2, little-endian) data(0..n) crc(2, LE)
 *
 * where `count` covers itself, the header, the data and the CRC — i.e.
 * 7 + data_len. A response is
 *
 *   count(1) payload(count-3) crc(2, LE)
 *
 * and a four-octet response is a bare status code rather than data. Two status
 * responses are fixed byte strings worth knowing by sight, because they are the
 * cheapest end-to-end proof that framing and CRC agree with the part:
 *
 *   04 11 33 43   "after wake, before the first command"
 *   04 00 03 40   success, no data returned
 *
 * Both are checked as known-answer vectors in tests/host/test_atecc.c.
 *
 * ---------------------------------------------------------------------------
 * Absence is a supported configuration
 * ---------------------------------------------------------------------------
 *
 * Spec §9.1 puts the device identity key in the ATECC608B, but a board whose
 * part is unpopulated, unprovisioned or dead must still boot and still serve
 * time. Every entry point here returns -ENODEV once @ref atecc_ctx_t has
 * concluded the part is absent, and never blocks longer than the bounded
 * retry budget. The glue logs that once and falls back to software keys.
 */

#ifndef STS1000_CORE_ATECC_ATECC_H_
#define STS1000_CORE_ATECC_ATECC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

/** I²C address of U60 on I²C1 (interface ref §4). 7-bit. */
#define ATECC_ADDR_DEFAULT 0x60U

/** Longest command data field this module emits (Verify External: 64+64). */
#define ATECC_DATA_MAX 128U

/** Longest framed write: word address + 7 header/CRC octets + data. */
#define ATECC_PKT_MAX (1U + 7U + ATECC_DATA_MAX)

/** Longest response: count + 64 payload octets + CRC. */
#define ATECC_RSP_MAX 72U

/** Public-key / signature / digest sizes for P-256 and SHA-256. */
#define ATECC_PUBKEY_LEN 64U
#define ATECC_SIG_LEN 64U
#define ATECC_DIGEST_LEN 32U
#define ATECC_SERIAL_LEN 9U
#define ATECC_REVISION_LEN 4U

/** Monotonic counters the part implements (datasheet: two). */
#define ATECC_COUNTER_COUNT 2U

/* --------------------------------------------------------- word addresses */

#define ATECC_WA_RESET 0x00U
#define ATECC_WA_SLEEP 0x01U
#define ATECC_WA_IDLE 0x02U
#define ATECC_WA_COMMAND 0x03U

/* ---------------------------------------------------------------- opcodes */

#define ATECC_OP_CHECKMAC 0x28U
#define ATECC_OP_COUNTER 0x24U
#define ATECC_OP_DERIVEKEY 0x1CU
#define ATECC_OP_ECDH 0x43U
#define ATECC_OP_GENDIG 0x15U
#define ATECC_OP_GENKEY 0x40U
#define ATECC_OP_HMAC 0x11U
#define ATECC_OP_INFO 0x30U
#define ATECC_OP_LOCK 0x17U
#define ATECC_OP_MAC 0x08U
#define ATECC_OP_NONCE 0x16U
#define ATECC_OP_PRIVWRITE 0x46U
#define ATECC_OP_RANDOM 0x1BU
#define ATECC_OP_READ 0x02U
#define ATECC_OP_SELFTEST 0x77U
#define ATECC_OP_SHA 0x47U
#define ATECC_OP_SIGN 0x41U
#define ATECC_OP_UPDATEEXTRA 0x20U
#define ATECC_OP_VERIFY 0x45U
#define ATECC_OP_WRITE 0x12U

/* ----------------------------------------------------------- mode selectors */

/** Info modes (param1). */
#define ATECC_INFO_REVISION 0x00U
#define ATECC_INFO_KEYVALID 0x01U
#define ATECC_INFO_STATE 0x02U
#define ATECC_INFO_GPIO 0x03U

/** Random modes (param1). 0x00 also stirs the seed; 0x01 does not. */
#define ATECC_RANDOM_SEED_UPDATE 0x00U
#define ATECC_RANDOM_NO_SEED 0x01U

/** Nonce modes (param1). */
#define ATECC_NONCE_RANDOM 0x00U      /**< 20-octet NumIn, device adds entropy */
#define ATECC_NONCE_PASSTHROUGH 0x03U /**< 32-octet value loaded into TempKey  */

/** Sign modes (param1). */
#define ATECC_SIGN_EXTERNAL 0x80U /**< sign the message digest held in TempKey */

/** Verify modes (param1). */
#define ATECC_VERIFY_EXTERNAL 0x02U
/** Verify param2 key type for NIST P-256. */
#define ATECC_KEY_TYPE_P256 0x0004U

/** GenKey modes (param1). */
#define ATECC_GENKEY_PRIVATE 0x04U /**< create a new private key in the slot  */
#define ATECC_GENKEY_PUBLIC 0x00U  /**< derive the public key of an existing one */

/** ECDH modes (param1). 0x0C = source TempKey is not used, output in the clear. */
#define ATECC_ECDH_OUTPUT_CLEAR 0x0CU

/** SHA modes (param1). */
#define ATECC_SHA_START 0x00U
#define ATECC_SHA_UPDATE 0x01U
#define ATECC_SHA_END 0x02U
#define ATECC_SHA_HMAC_START 0x04U
#define ATECC_SHA_HMAC_END 0x05U

/** SHA Update takes exactly one 64-octet block. */
#define ATECC_SHA_BLOCK 64U

/** Counter modes (param1). */
#define ATECC_COUNTER_READ 0x00U
#define ATECC_COUNTER_INCREMENT 0x01U

/** Read/Write zone selectors (param1 bits 0..1) and flags. */
#define ATECC_ZONE_CONFIG 0x00U
#define ATECC_ZONE_OTP 0x01U
#define ATECC_ZONE_DATA 0x02U
#define ATECC_ZONE_MASK 0x03U
#define ATECC_ZONE_32 0x80U        /**< 32-octet transfer instead of 4        */
#define ATECC_ZONE_ENCRYPTED 0x40U /**< encrypted read/write                  */

/** Lock zones (param1). */
#define ATECC_LOCK_ZONE_CONFIG 0x00U
#define ATECC_LOCK_ZONE_DATA 0x01U
#define ATECC_LOCK_ZONE_SLOT 0x02U
#define ATECC_LOCK_NO_CRC 0x80U /**< skip the expected-CRC check in param2   */

/** SelfTest mode bits (param1). */
#define ATECC_SELFTEST_RNG 0x01U
#define ATECC_SELFTEST_ECDSA_SIGN 0x02U
#define ATECC_SELFTEST_ECDSA_VERIFY 0x04U
#define ATECC_SELFTEST_ECDH 0x08U
#define ATECC_SELFTEST_AES 0x10U
#define ATECC_SELFTEST_SHA 0x20U
#define ATECC_SELFTEST_ALL 0x3FU

/* -------------------------------------------------------- status responses */

#define ATECC_ST_SUCCESS 0x00U
#define ATECC_ST_MISCOMPARE 0x01U
#define ATECC_ST_PARSE_ERR 0x03U
#define ATECC_ST_ECC_FAULT 0x05U
#define ATECC_ST_SELFTEST_ERR 0x07U
#define ATECC_ST_HEALTH_TEST_ERR 0x08U
#define ATECC_ST_EXEC_ERR 0x0FU
#define ATECC_ST_AFTER_WAKE 0x11U
#define ATECC_ST_WDT_EXPIRE 0xEEU
#define ATECC_ST_COMM_ERR 0xFFU

/* ------------------------------------------------------------------ CRC-16 */

/**
 * The Microchip CryptoAuthentication CRC-16.
 *
 * Polynomial 0x8005, register initialised to 0, data bits consumed
 * least-significant-first, no final inversion. Returned in the host's integer
 * domain; the wire carries it little-endian, which @ref atecc_crc16_put does.
 *
 * This is not any of the named CRC-16 variants a generic table would give you —
 * mixing bit order like this is exactly why it is spelled out here rather than
 * reused from core/util.
 */
uint16_t atecc_crc16(const uint8_t *data, size_t len);

/** Store @p crc little-endian into @p out[0..1]. */
void atecc_crc16_put(uint8_t *out, uint16_t crc);

/** Read a little-endian CRC from @p in[0..1]. */
uint16_t atecc_crc16_get(const uint8_t *in);

/* ------------------------------------------------------------ packet build */

/**
 * Frame one command packet, word address included.
 *
 * @param out       Receives `03 count opcode p1 p2lo p2hi data... crclo crchi`.
 * @param cap       Capacity of @p out; ATECC_PKT_MAX always suffices.
 * @param opcode    ATECC_OP_*.
 * @param p1        param1 (the mode byte for most commands).
 * @param p2        param2, emitted little-endian.
 * @param data      Command data, or NULL when @p data_len is 0.
 * @param data_len  0..ATECC_DATA_MAX.
 *
 * @retval >0       Octets written.
 * @retval -EINVAL  NULL @p out, NULL @p data with a non-zero length, or a data
 *                  length over ATECC_DATA_MAX.
 * @retval -ENOSPC  @p cap too small.
 */
int atecc_pkt_build(uint8_t *out, size_t cap, uint8_t opcode, uint8_t p1,
		    uint16_t p2, const uint8_t *data, size_t data_len);

/* ------------------------------------------------------------ response parse */

/** A parsed response. @p payload points into the caller's buffer. */
typedef struct {
	const uint8_t *payload; /**< NULL for a status-only response */
	size_t payload_len;     /**< 0 for a status-only response    */
	uint8_t status;         /**< ATECC_ST_* when @p is_status    */
	bool is_status;         /**< true for the four-octet form    */
} atecc_rsp_t;

/**
 * Parse and CRC-check one response.
 *
 * @retval 0         Parsed; @p out describes it.
 * @retval -EINVAL   NULL argument.
 * @retval -EAGAIN   Fewer octets than the declared count — read more.
 * @retval -EBADMSG  A count below 4 or above ATECC_RSP_MAX, or a CRC mismatch.
 */
int atecc_rsp_parse(const uint8_t *buf, size_t len, atecc_rsp_t *out);

/**
 * How many octets a response claims, from its first octet.
 *
 * @retval >0        The declared count.
 * @retval -EAGAIN   @p len is 0.
 * @retval -EBADMSG  A count outside 4..ATECC_RSP_MAX.
 */
int atecc_rsp_count(const uint8_t *buf, size_t len);

/** Map an ATECC_ST_* status onto an errno-style value. Success maps to 0. */
int atecc_status_to_errno(uint8_t status);

/** Human-readable name of an ATECC_ST_* status. Never NULL. */
const char *atecc_status_name(uint8_t status);

/**
 * Conservative upper bound on a command's execution time, microseconds.
 *
 * These are ceilings above the datasheet's maximum execution times, not typical
 * figures: @ref atecc_exec polls after the delay and retries a communication
 * error, so an over-long bound costs latency on one command while an under-long
 * one costs a retry. Unknown opcodes get the largest bound.
 */
uint32_t atecc_exec_delay_us(uint8_t opcode);

/* ------------------------------------------------------------- transport */

/**
 * The transport this module drives. All four members are required.
 *
 * @p write and @p read take a 7-bit address and return 0 on success, negative
 * on failure. A NAK must be reported as a failure — that is how the wake
 * handshake detects an absent part. @p delay_us may busy-wait or sleep.
 *
 * @p set_speed exists solely for the wake token: waking the part needs the bus
 * held low for at least 60 µs, which at the board's 400 kHz cannot be produced
 * by a single-byte transfer. The glue drops to 100 kHz for the token and
 * restores 400 kHz afterwards. A transport that cannot change speed may return
 * -ENOTSUP; @ref atecc_wake then still emits the token, which works on parts
 * whose bus is already slow enough, and reports -ENODEV if it does not answer.
 */
typedef struct {
	int (*write)(void *ctx, uint8_t addr, const uint8_t *buf, size_t len);
	int (*read)(void *ctx, uint8_t addr, uint8_t *buf, size_t len);
	int (*delay_us)(void *ctx, uint32_t us);
	int (*set_speed)(void *ctx, uint32_t hz);
	void *ctx;
} atecc_bus_t;

/** Bus speed used for the wake token, and the normal speed restored after. */
#define ATECC_WAKE_SPEED_HZ 100000U
#define ATECC_RUN_SPEED_HZ 400000U

/** Wake-token low time and the settle delay before the wake response is read. */
#define ATECC_WAKE_LOW_US 80U
#define ATECC_WAKE_DELAY_US 1500U

/** Retries @ref atecc_exec spends on a communication error before giving up. */
#define ATECC_EXEC_RETRIES 3U

/* -------------------------------------------------------------- statistics */

typedef struct {
	uint32_t commands;   /**< atecc_exec() calls that reached the bus */
	uint32_t retries;    /**< response re-reads after a comms error   */
	uint32_t crc_errors; /**< responses that failed the CRC check     */
	uint32_t status_errors; /**< non-success status responses         */
	uint32_t wakes;      /**< successful wake handshakes              */
	uint32_t wake_fails; /**< wake handshakes that found no device    */
	uint32_t bus_errors; /**< transport write/read failures           */
} atecc_stats_t;

/* ----------------------------------------------------------------- context */

/** Device session. Caller-owned; all fields private. */
typedef struct {
	atecc_bus_t bus;
	uint8_t addr;
	bool ready;   /**< atecc_init() succeeded */
	bool awake;   /**< a wake handshake has completed and no sleep since */
	bool absent;  /**< concluded not present; every call returns -ENODEV */
	bool have_serial;
	uint8_t serial[ATECC_SERIAL_LEN];
	uint8_t revision[ATECC_REVISION_LEN];
	bool have_revision;
	atecc_stats_t stats;
} atecc_ctx_t;

/**
 * Bind a context to a transport.
 *
 * Performs no I/O: presence is established by the first @ref atecc_wake, so a
 * caller may initialise during early bring-up and probe later.
 *
 * @param addr  7-bit address, or 0 for ATECC_ADDR_DEFAULT.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument, or a transport missing a required member.
 */
int atecc_init(atecc_ctx_t *c, const atecc_bus_t *bus, uint8_t addr);

/**
 * Wake the part and consume its wake response.
 *
 * Idempotent while already awake. On a device that never answers, the context
 * is latched @p absent and every later call returns -ENODEV without touching
 * the bus, so an unpopulated part costs one bounded probe for the whole boot.
 *
 * @retval 0         Awake.
 * @retval -ENODEV   No answer, or an answer that was not the wake status.
 * @retval -EINVAL   Uninitialised context.
 */
int atecc_wake(atecc_ctx_t *c);

/** Send the Idle token (TempKey cleared, RNG seed kept). */
int atecc_idle(atecc_ctx_t *c);

/** Send the Sleep token (TempKey and RNG seed cleared). */
int atecc_sleep(atecc_ctx_t *c);

/**
 * Clear the @p absent latch so a later @ref atecc_wake retries the part.
 *
 * For an operator-driven re-probe (`sec.attest` after reseating a board), not
 * for a retry loop: the point of the latch is that boot does not pay for an
 * absent device more than once.
 */
void atecc_reprobe(atecc_ctx_t *c);

/** True once the part has answered a wake handshake. */
bool atecc_present(const atecc_ctx_t *c);

/** Snapshot the counters. @p out untouched on -EINVAL. */
int atecc_stats_get(const atecc_ctx_t *c, atecc_stats_t *out);

/* -------------------------------------------------------------- execution */

/**
 * Run one framed packet and return its response.
 *
 * Wakes the part if needed, writes @p pkt, waits @p exec_delay_us, then reads
 * the response — retrying the read up to ATECC_EXEC_RETRIES times when the
 * transport fails or the response is malformed, which is how the part signals
 * "still busy". A status response other than success is reported through the
 * return value; @p out_rsp still describes it.
 *
 * @param exec_delay_us  0 selects @ref atecc_exec_delay_us for @p pkt's opcode.
 * @param rsp            Receives the raw response; must hold ATECC_RSP_MAX.
 * @param out_rsp        Optional parsed view; @p payload points into @p rsp.
 *
 * @retval 0        Success (status 0x00, or a data response).
 * @retval -ENODEV  Absent part.
 * @retval -EIO     The transport failed, or no well-formed response arrived.
 * @retval other    @ref atecc_status_to_errno of the returned status.
 */
int atecc_exec(atecc_ctx_t *c, const uint8_t *pkt, size_t pkt_len,
	       uint32_t exec_delay_us, uint8_t *rsp, size_t rsp_cap,
	       atecc_rsp_t *out_rsp);

/* ------------------------------------------------------- command wrappers */

/** Info(Revision) — four octets identifying the silicon. */
int atecc_revision(atecc_ctx_t *c, uint8_t out[ATECC_REVISION_LEN]);

/**
 * The nine-octet factory serial number (config bytes 0..3 and 8..12).
 *
 * Cached after the first read: it is immutable, and it is what the SNMP
 * engineID and the USB serial string are built from.
 */
int atecc_serial(atecc_ctx_t *c, uint8_t out[ATECC_SERIAL_LEN]);

/** Random(no-seed-update) — 32 octets from the hardware TRNG. */
int atecc_random(atecc_ctx_t *c, uint8_t out[32]);

/** Nonce(passthrough) — load @p digest into TempKey for a later Sign. */
int atecc_nonce_passthrough(atecc_ctx_t *c,
			    const uint8_t digest[ATECC_DIGEST_LEN]);

/**
 * ECDSA P-256 sign of @p digest with the private key in @p key_id.
 *
 * Issues Nonce(passthrough) then Sign(external); the signature is R||S, 32
 * octets each, big-endian, which is what PSA and mbedTLS raw-ECDSA expect.
 */
int atecc_sign(atecc_ctx_t *c, uint16_t key_id,
	       const uint8_t digest[ATECC_DIGEST_LEN],
	       uint8_t sig[ATECC_SIG_LEN]);

/**
 * Verify @p sig over @p digest against the external public key @p pub.
 *
 * @param out_ok  Receives true only for a status of success.
 * @retval 0      The signature verified.
 * @retval -EBADE The part reported a miscompare (@p out_ok false).
 */
int atecc_verify_extern(atecc_ctx_t *c, const uint8_t pub[ATECC_PUBKEY_LEN],
			const uint8_t digest[ATECC_DIGEST_LEN],
			const uint8_t sig[ATECC_SIG_LEN], bool *out_ok);

/** GenKey(private) — create a new keypair in @p key_id, returning its public key. */
int atecc_genkey_private(atecc_ctx_t *c, uint16_t key_id,
			 uint8_t pub[ATECC_PUBKEY_LEN]);

/** GenKey(public) — the public key of the private key already in @p key_id. */
int atecc_pubkey(atecc_ctx_t *c, uint16_t key_id,
		 uint8_t pub[ATECC_PUBKEY_LEN]);

/** ECDH — X coordinate of @p peer × the private key in @p key_id, in the clear. */
int atecc_ecdh(atecc_ctx_t *c, uint16_t key_id,
	       const uint8_t peer[ATECC_PUBKEY_LEN], uint8_t secret[32]);

/**
 * SHA-256 of @p msg using the part's hash engine.
 *
 * Start, then one Update per whole 64-octet block, then End with the tail.
 * Present for attestation flows that must be provably computed on the device;
 * ordinary hashing goes through port_crypto and the STM32 HASH block.
 */
int atecc_sha256(atecc_ctx_t *c, const uint8_t *msg, size_t len,
		 uint8_t out[ATECC_DIGEST_LEN]);

/** HMAC-SHA-256 under the key in @p key_id over the TempKey-loaded nonce. */
int atecc_hmac(atecc_ctx_t *c, uint16_t key_id, uint8_t mode,
	       uint8_t out[ATECC_DIGEST_LEN]);

/**
 * Read from a zone.
 *
 * @param zone  ATECC_ZONE_CONFIG / _OTP / _DATA.
 * @param word  Word address within the zone (the part's own units).
 * @param len   4 or 32; 32 selects the 32-octet transfer flag.
 */
int atecc_read_zone(atecc_ctx_t *c, uint8_t zone, uint16_t word, uint8_t *out,
		    size_t len);

/** Write to a zone. @p len is 4 or 32. */
int atecc_write_zone(atecc_ctx_t *c, uint8_t zone, uint16_t word,
		     const uint8_t *in, size_t len);

/**
 * Lock a zone.
 *
 * @param zone      ATECC_LOCK_ZONE_*.
 * @param crc       Expected zone CRC, or 0 with @p ignore_crc.
 * @param ignore_crc Set ATECC_LOCK_NO_CRC. Irreversible either way.
 */
int atecc_lock(atecc_ctx_t *c, uint8_t zone, uint16_t crc, bool ignore_crc);

/** Read monotonic counter @p idx (0..1). */
int atecc_counter_read(atecc_ctx_t *c, uint8_t idx, uint32_t *out);

/** Increment monotonic counter @p idx (0..1) and return the new value. */
int atecc_counter_increment(atecc_ctx_t *c, uint8_t idx, uint32_t *out);

/**
 * Run the built-in self tests selected by @p mode (ATECC_SELFTEST_*).
 *
 * @param out_result  Receives the result bitmap: a set bit is a *failed*
 *                    subsystem, which is why success is reported as 0.
 * @retval 0       Every selected test passed.
 * @retval -EBADE  At least one failed; @p out_result says which.
 */
int atecc_selftest(atecc_ctx_t *c, uint8_t mode, uint8_t *out_result);

/* ------------------------------------------------------------ attestation */

/**
 * Everything `sec.attest` reports (spec §9.1, §9.6).
 *
 * @p counter_valid distinguishes "counter is 0" from "counter unreadable",
 * which matters because the counters are the anti-rollback evidence.
 */
typedef struct {
	bool present;
	uint8_t serial[ATECC_SERIAL_LEN];
	uint8_t revision[ATECC_REVISION_LEN];
	uint32_t counter[ATECC_COUNTER_COUNT];
	bool counter_valid[ATECC_COUNTER_COUNT];
	bool config_locked;
	bool data_locked;
	bool lock_valid;
	uint8_t selftest_result; /**< 0 = all passed */
	bool selftest_valid;
	uint8_t pubkey[ATECC_PUBKEY_LEN];
	bool pubkey_valid;
} atecc_attest_t;

/**
 * Collect an attestation report for @p key_id.
 *
 * Best-effort: a sub-report that cannot be read leaves its `*_valid` flag
 * false rather than failing the whole call, because a partial report is what
 * lets an operator tell an unprovisioned part from an absent one.
 *
 * @retval 0        @p out filled (possibly with unset validity flags).
 * @retval -ENODEV  Absent part; @p out is zeroed with @p present false.
 * @retval -EINVAL  NULL argument.
 */
int atecc_attest(atecc_ctx_t *c, uint16_t key_id, atecc_attest_t *out);

/**
 * Decode the lock bytes from a 32-octet read of config block 2.
 *
 * Config-zone bytes 86 (LockValue, the data/OTP zones) and 87 (LockConfig, the
 * config zone) each read 0x55 while unlocked and 0x00 once locked. Block 2
 * covers bytes 64..95, so they land at offsets 22 and 23 of the block. Exposed
 * because those are the kind of magic numbers that should be asserted by a
 * test, not buried in a caller.
 *
 * @retval 0        Decoded.
 * @retval -EINVAL  NULL argument or a short buffer.
 */
int atecc_decode_locks(const uint8_t *cfg_block, size_t len, bool *out_data,
		       bool *out_config);

/** Byte offsets of LockValue / LockConfig inside the 128-octet config zone. */
#define ATECC_CFG_OFF_LOCK_VALUE 86U
#define ATECC_CFG_OFF_LOCK_CONFIG 87U
/** The same two, relative to the 32-octet config block that contains them. */
#define ATECC_CFG_BLK_OFF_LOCK_VALUE (ATECC_CFG_OFF_LOCK_VALUE - 64U)
#define ATECC_CFG_BLK_OFF_LOCK_CONFIG (ATECC_CFG_OFF_LOCK_CONFIG - 64U)

/**
 * Read/Write zone addresses.
 *
 * The address parameter is `(block << 3) | word-offset`; a 32-octet transfer
 * requires a word offset of 0. Block 0 carries the serial number halves (bytes
 * 0..3 and 8..12), block 2 carries the lock bytes.
 */
#define ATECC_CFG_WORD_BLOCK0 0x0000U
#define ATECC_CFG_WORD_LOCKS 0x0010U

/** Byte offsets of the two serial-number runs inside config block 0. */
#define ATECC_CFG_OFF_SN03 0U
#define ATECC_CFG_OFF_SN48 8U

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_ATECC_ATECC_H_ */
