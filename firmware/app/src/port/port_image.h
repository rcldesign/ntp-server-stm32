/*
 * Firmware-image / DFU port: how core/mcp reaches the staging slot and the
 * bootloader's swap machinery. Target: Zephyr flash_map + flash_img +
 * boot_request_upgrade/boot_write_img_confirmed. Host tests: RAM-backed fake.
 */
#ifndef STS1000_PORT_IMAGE_H_
#define STS1000_PORT_IMAGE_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct {
	uint8_t  slot;          /* 0 = active, 1 = staged */
	uint32_t size;          /* image size if valid, else 0 */
	uint32_t version[4];    /* major.minor.rev.build */
	bool     valid;         /* parses as an MCUboot image */
	bool     active;        /* currently running */
	bool     pending;       /* marked for test-swap on next boot */
	bool     confirmed;     /* image confirmed (won't revert) */
} port_image_info_t;

typedef struct {
	/* Staging-slot geometry. */
	uint32_t (*staging_size)(void *ctx);
	/* Erase [off, off+len) in the staging slot (sector-aligned by impl). */
	int (*staging_erase)(void *ctx, uint32_t off, uint32_t len);
	/* Write chunk at offset (impl handles write-block alignment buffering
	 * for the FINAL short block when flush=true). */
	int (*staging_write)(void *ctx, uint32_t off, const uint8_t *data,
			     size_t len, bool flush);
	/* Read back from the staging slot (for verify). */
	int (*staging_read)(void *ctx, uint32_t off, uint8_t *data, size_t len);
	/* Image table for both slots. */
	int (*image_info)(void *ctx, uint8_t slot, port_image_info_t *out);
	/* Mark staged image pending (test boot on next reset). */
	int (*mark_pending)(void *ctx);
	/* Confirm the currently-running image. */
	int (*confirm_active)(void *ctx);
	/* Request revert (erase trailer pending/confirm of staged). */
	int (*request_revert)(void *ctx);
	/* System reset. mode: 0 normal, 1 stay-in-bootloader-recovery. */
	void (*reboot)(void *ctx, int mode);
	void *ctx;
} port_image_t;

#endif /* STS1000_PORT_IMAGE_H_ */
