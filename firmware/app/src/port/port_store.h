/*
 * Persistent key/value storage port for core/cfg (target: Zephyr settings/NVS;
 * host tests: RAM map). Keys are the cfg numeric IDs (see ARCHITECTURE.md §8).
 */
#ifndef STS1000_PORT_STORE_H_
#define STS1000_PORT_STORE_H_

#include <stdint.h>
#include <stddef.h>

typedef struct {
	/* Returns byte count read (>=0) or negative error; -2 (ENOENT-like)
	 * when the key does not exist. */
	int (*load)(void *ctx, uint16_t id, void *buf, size_t cap);
	int (*save)(void *ctx, uint16_t id, const void *buf, size_t len);
	int (*erase)(void *ctx, uint16_t id);
	void *ctx;
} port_store_t;

#endif /* STS1000_PORT_STORE_H_ */
