/*
 * STS1000 "Meridian" — core/mcp: internals shared between mcp.c and mcp_dfu.c.
 *
 * Private to core/mcp. Nothing outside the module may include this header;
 * mcp.h is the public API.
 */

#ifndef STS1000_CORE_MCP_MCP_INTERNAL_H_
#define STS1000_CORE_MCP_MCP_INTERNAL_H_

#include "mcp/mcp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Scratch for the response payload: handlers write here and pass the length to
 * mcp__reply(), which seals the header and CRC around it in place. Capacity is
 * MCP_MAX_PAYLOAD.
 */
uint8_t *mcp__rsp_buf(mcp_ctx_t *c);

/**
 * Frame and transmit the response payload of @p len bytes.
 *
 * @retval 0        Transmitted.
 * @retval -EAGAIN  Held for retry (mcp_poll_tx()); the engine stops consuming
 *                  input until it drains.
 */
int mcp__reply(mcp_ctx_t *c, uint8_t cmd, uint16_t seq, uint16_t len);

/** mcp__reply() for a bare status byte. */
int mcp__reply_status(mcp_ctx_t *c, uint8_t cmd, uint16_t seq, uint8_t status);

/** Map a port_*_t return value onto an mcp_err_t. */
uint8_t mcp__port_err(int rc);

/** Log to the wired ring under LOGR_SUB_MCP; a no-op when no ring is wired. */
void mcp__log(mcp_ctx_t *c, uint8_t level, const char *msg);

/* --------------------------------------------------------------- mcp_dfu.c */

/** Handle FW_INFO/BEGIN/DATA/END/CONFIRM/REVERT. Same return as mcp__reply(). */
int mcp_dfu__handle(mcp_ctx_t *c, const mcp_frame_t *f);

/** Advance the DFU idle timeout. */
void mcp_dfu__tick(mcp_ctx_t *c, uint64_t now_ms);

/** Forget the DFU session entirely (no resume possible afterwards). */
void mcp_dfu__reset(mcp_ctx_t *c);

/** Pack a port_image_info_t's booleans into the MCP_SLOT_* bitmap. */
uint8_t mcp_dfu__slot_flags(const port_image_info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MCP_MCP_INTERNAL_H_ */
