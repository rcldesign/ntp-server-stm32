/*
 * STS1000 "Meridian" — core/ui: local-panel navigation, policy and rendering.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers, no globals,
 * no stdio, no libm. Dependencies: `util` and `quality` only (ARCHITECTURE.md
 * §4 dependency map).
 *
 * This module owns spec §6 in its entirety *except* pixels and wires:
 *
 *   §6.1  the seven panel buttons, the encoder and the screen-stack navigation
 *         state machine, including value edit and confirm dialogs;
 *   §6.2  the screen hierarchy Home / Skyplot / Clocks / Network / Power-Health
 *         / Alarms / Menu, with Left-Back popping and Right-Enter descending;
 *   §6.3  the sky page: the polar plot's placement and the view toggle, with the
 *         plot itself rendered by core/ui/skyplot.h (see below);
 *   §6.4  the touch/proximity wake policy and the backlight-duty-only sleep.
 *
 * What it does NOT own: the ST7796 command stream, the FT6336U registers, the
 * TIM1 counter and the TIM15_CH2 duty register. Those belong to the Zephyr glue
 * in `src/zephyr/ui/`. The seam between the two is deliberately narrow:
 *
 *   glue -> core   ui_input()   one debounced/decoded event at a time
 *   core -> glue   ui_render()  a text-tile surface the glue rasterises
 *                  ui_action_get()      discrete requests (reboot, brightness…)
 *                  ui_backlight_permille()  the continuous wake/sleep output
 *
 * Nothing in core touches configuration or hardware directly. A menu edit does
 * not *apply* anything: it emits a UI_ACTION_* request and the glue decides
 * whether the request is legal, persists it through `cfg`, and drives the pin.
 * That is what keeps the whole navigation tree testable on the host, and it is
 * why a "factory reset" here is three enum values in a queue rather than a
 * flash erase.
 *
 * Skyplot. Spec §6.3's true-north-rotated polar plot needs trigonometry, a
 * calibrated e-compass and a pixel canvas, none of which belong in a text-tile
 * surface — so it lives in its own module, `core/ui/skyplot.h`, which renders
 * into an indexed-colour canvas and is therefore still host-testable against
 * golden images. This module owns the *page*: UI_PAGE_SKYPLOT carries two views
 * (ui_sky_view_t), cycled with FN, and emits UI_HINT_SKYPLOT with the plot's cell
 * rectangle for the glue to fill from skyplot.h. The numeric per-SV table remains
 * the second view.
 */

#ifndef STS1000_CORE_UI_UI_H_
#define STS1000_CORE_UI_UI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "quality/quality.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================== *
 *  The render surface: a text-tile grid
 * ===================================================================== */

/**
 * Upper bounds on the grid.
 *
 * 32 rows is the width of the dirty-row bitmap; 80 columns is the width of the
 * internal line scratch. The as-built panel is a 480x320 ST7796 with an 8x16
 * font, i.e. 60x20 — comfortably inside both.
 */
#define UI_SURF_MAX_ROWS 32u
#define UI_SURF_MAX_COLS 80u

/** Attribute bits. The glue maps these to colours and to font weight. */
#define UI_ATTR_NORMAL 0x00u
#define UI_ATTR_INVERSE 0x01u /**< selected row / focused field */
#define UI_ATTR_DIM 0x02u     /**< label text, inactive items */
#define UI_ATTR_ACCENT 0x04u  /**< headings, the live time */
#define UI_ATTR_OK 0x08u      /**< green */
#define UI_ATTR_WARN 0x10u    /**< amber */
#define UI_ATTR_ALARM 0x20u   /**< red */

/** Overlay hints the glue draws on top of the text cells. */
typedef enum {
	UI_HINT_NONE = 0,
	/**
	 * Draw the cells at (row, col..col+len-1) with double-size glyphs,
	 * occupying the pixel band of rows `row` and `row+1`. The page code
	 * leaves row+1 blank across those columns; the glue skips normal text
	 * rendering there.
	 */
	UI_HINT_BIGNUM,
	/**
	 * Horizontal progress bar filling `value` permille of the cell
	 * rectangle (row, col..col+len-1). The page code leaves those cells
	 * blank — the bar is drawn over them.
	 */
	UI_HINT_PROGRESS,
	/** Horizontal rule across (row, col..col+len-1). */
	UI_HINT_RULE,
	/**
	 * Draw the polar skyplot into the cell rectangle
	 * (row..row+len-1, col..cols-1) — `len` is a ROW count here, not a column
	 * count, because the plot is square and its extent is set by the taller of
	 * the two axes.
	 *
	 * The page code leaves those cells blank. The glue renders with
	 * core/ui/skyplot.h into its own indexed canvas, maps the palette to
	 * RGB565 and blits. `value` carries the ui_sky_view_t currently selected so
	 * the glue can label the plot.
	 */
	UI_HINT_SKYPLOT,
	UI_HINT__COUNT
} ui_hint_kind_t;

/** Most overlay hints one frame may carry. */
#define UI_SURF_MAX_HINTS 8u

typedef struct {
	uint8_t kind; /**< ui_hint_kind_t */
	uint8_t row;
	uint8_t col;
	uint8_t len;
	uint8_t attr;   /**< UI_ATTR_* */
	uint16_t value; /**< UI_HINT_PROGRESS: 0..1000 permille */
} ui_hint_t;

/**
 * A rows x cols grid of character cells plus a small overlay-hint list.
 *
 * The two cell arrays are caller-owned and must each hold at least rows*cols
 * bytes (no allocation in core). `cell_w`/`cell_h` are the glue's font metrics
 * and exist only so the module can map a touch coordinate onto a cell; they do
 * not affect any text output.
 */
typedef struct {
	uint8_t rows;
	uint8_t cols;
	uint8_t cell_w; /**< glyph width in pixels (8 for the as-built font) */
	uint8_t cell_h; /**< glyph height in pixels (16) */
	char *ch;       /**< rows*cols characters, row-major */
	uint8_t *attr;  /**< rows*cols attributes, row-major */
	ui_hint_t hint[UI_SURF_MAX_HINTS];
	uint8_t hint_count;
	uint32_t hint_dropped; /**< hints refused because the list was full */
} ui_surface_t;

/**
 * Bind caller-owned storage to a surface and blank it.
 *
 * @param s      Surface.
 * @param rows   1..UI_SURF_MAX_ROWS.
 * @param cols   1..UI_SURF_MAX_COLS.
 * @param ch     rows*cols character cells.
 * @param attr   rows*cols attribute cells.
 * @param cells  Capacity of @p ch and @p attr, in cells. Must be >= rows*cols.
 *
 * @retval 0        Bound and cleared.
 * @retval -EINVAL  NULL argument, zero/oversized geometry, or @p cells too
 *                  small for the requested grid.
 */
int ui_surface_init(ui_surface_t *s, uint8_t rows, uint8_t cols, char *ch,
		    uint8_t *attr, size_t cells);

/** Blank every cell to ' '/UI_ATTR_NORMAL and drop all hints. */
void ui_surface_clear(ui_surface_t *s);

/**
 * Write a NUL-terminated string, clipped to the row.
 *
 * Off-grid rows, negative-equivalent columns and over-long strings are clipped
 * rather than rejected: page layout code composes freely and the surface is the
 * one place that has to know the geometry.
 *
 * @return Number of cells written (0 if entirely clipped away).
 */
size_t ui_surface_put(ui_surface_t *s, uint8_t row, uint8_t col, const char *str,
		      uint8_t attr);

/** As ui_surface_put(), but at most @p max characters of @p str. */
size_t ui_surface_putn(ui_surface_t *s, uint8_t row, uint8_t col,
		       const char *str, size_t max, uint8_t attr);

/** Fill @p len cells from (row,col) with @p c. @return cells written. */
size_t ui_surface_fill(ui_surface_t *s, uint8_t row, uint8_t col, size_t len,
		       char c, uint8_t attr);

/**
 * Append an overlay hint.
 *
 * Unlike the text accessors, which clip, a hint is validated against the
 * surface geometry and refused if it does not fit. The reason is the consumer:
 * text lands in this module's own bounded arrays, while a hint is a rectangle
 * the glue rasterises straight into the framebuffer, so an out-of-range hint
 * accepted here becomes an out-of-bounds write there.
 *
 * @retval 0        Appended.
 * @retval -EINVAL  @p s or @p h is NULL, or the hint kind is out of range.
 * @retval -EDOM    The hint does not fit: `row` outside the grid, `col + len`
 *                  past the right edge, a UI_HINT_BIGNUM whose second row is
 *                  off-grid, or a UI_HINT_PROGRESS `value` above 1000 permille.
 * @retval -ENOSPC  The list is full; ui_surface_t::hint_dropped is bumped.
 */
int ui_surface_hint(ui_surface_t *s, const ui_hint_t *h);

/**
 * True when @p row is the lower half of a UI_HINT_BIGNUM band, i.e. the glue
 * must not render normal text there.
 */
bool ui_surface_is_bignum_tail(const ui_surface_t *s, uint8_t row);

/**
 * Copy @p row into @p buf as a NUL-terminated string with trailing blanks
 * removed. This is the golden-text accessor the host tests assert against and
 * the glue's debug dump; it is not on the render path.
 *
 * @return Length written, excluding the NUL. 0 for a bad row or a NULL buffer.
 */
size_t ui_surface_row_text(const ui_surface_t *s, uint8_t row, char *buf,
			   size_t cap);

/**
 * Per-row change detection between two same-geometry surfaces.
 *
 * @param cur   Freshly rendered surface.
 * @param prev  Previous frame. NULL means "everything is dirty".
 * @param out   Receives a bit per row (bit n = row n changed).
 *
 * @retval 0        @p out written.
 * @retval -EINVAL  @p cur or @p out is NULL.
 * @retval -EDOM    @p prev has a different geometry from @p cur.
 */
int ui_surface_diff_rows(const ui_surface_t *cur, const ui_surface_t *prev,
			 uint32_t *out);

/**
 * Narrow a dirty row to the smallest column span that actually changed.
 *
 * @param cur   Freshly rendered surface.
 * @param prev  Previous frame, or NULL for "the whole row".
 * @param row   Row to examine.
 * @param c0    Receives the first changed column.
 * @param c1    Receives the last changed column.
 *
 * @retval 0        A change was found; *c0 <= *c1.
 * @retval -ENOENT  Nothing changed in that row.
 * @retval -EINVAL  Bad argument or row.
 * @retval -EDOM    Geometry mismatch.
 */
int ui_surface_row_span(const ui_surface_t *cur, const ui_surface_t *prev,
			uint8_t row, uint8_t *c0, uint8_t *c1);

/* ===================================================================== *
 *  Pages
 * ===================================================================== */

/** Screen identifiers. Numeric values are part of the MCP/telemetry contract. */
typedef enum {
	UI_PAGE_HOME = 0,
	UI_PAGE_SKYPLOT,
	UI_PAGE_CLOCKS,
	UI_PAGE_NETWORK,
	UI_PAGE_POWER,
	UI_PAGE_ALARMS,
	UI_PAGE_MENU,
	UI_PAGE_EDIT,    /**< numeric/enumerated value edit for one menu item */
	UI_PAGE_CONFIRM, /**< guarded-action confirmation dialog */
	UI_PAGE__COUNT
} ui_page_t;

/** Short page title, e.g. "HOME", "POWER/HEALTH". Never NULL. */
const char *ui_page_name(uint8_t page);

/**
 * The pages reachable from Home, in the spec §6.2 order.
 *
 * Home is the stack floor and is not a member of its own destination list.
 */
#define UI_HOME_DEST_COUNT 6u

/** Destination page for Home selection index @p i, or UI_PAGE_HOME if out of range. */
ui_page_t ui_home_dest(uint8_t i);

/* ===================================================================== *
 *  Menu
 * ===================================================================== */

/**
 * The locally editable set.
 *
 * Spec §6.1 also lists network basics and a guarded reference override as
 * panel-editable. Both are deliberately absent this phase: they mutate `cfg`
 * groups (0x01 net, 0x06 timing) whose validation and apply paths live in the
 * net and platform areas, and a half-wired override of the timing reference is
 * a worse outcome than no override at all. Everything here is either display
 * policy or a whole-unit action, so the panel owns it end to end.
 */
typedef enum {
	UI_MENU_BRIGHTNESS = 0,
	UI_MENU_TIMEOUT,
	UI_MENU_IDENTIFY,
	UI_MENU_REBOOT,
	UI_MENU_FACTORY_RESET,
	UI_MENU__COUNT
} ui_menu_item_t;

/** Menu item label, e.g. "Brightness". Never NULL. */
const char *ui_menu_name(uint8_t item);

/** Backlight brightness choices, percent. */
#define UI_BRIGHTNESS_MIN_PCT 10u
#define UI_BRIGHTNESS_MAX_PCT 100u
#define UI_BRIGHTNESS_STEP_PCT 10u

/** Idle-timeout choices, seconds. Index 0 is "never". */
#define UI_TIMEOUT_CHOICES 7u
extern const uint16_t ui_timeout_choices[UI_TIMEOUT_CHOICES];

/* ===================================================================== *
 *  Input
 * ===================================================================== */

/**
 * Logical input events.
 *
 * The panel's physical set (docs/sts1000_panel_controls.md §3) maps onto this
 * as: MENU -> FN, BACK -> LEFT, DISPLAY -> DISPLAY, ACK -> ACK, INFO -> INFO,
 * LAMP -> LAMP, RESET (long) -> RESET_HOLD, encoder -> ENCODER, encoder push ->
 * ENTER. UP/DOWN/RIGHT have no dedicated key on the as-built panel — they exist
 * because the encoder produces them, because the touch layer synthesises them,
 * and because a navigation machine that can only be driven by a knob is not
 * testable in a way that would catch an off-by-one.
 */
typedef enum {
	UI_IN_NONE = 0,
	UI_IN_UP,
	UI_IN_DOWN,
	UI_IN_LEFT, /**< back: pops the screen stack everywhere */
	/**
	 * Descend. Identical to ENTER except on a confirm dialog, where it only
	 * moves the highlight to "Yes" — the one place the asymmetry matters,
	 * because a right-arrow must never be able to reboot the unit on its
	 * own. On an edit page it commits, like ENTER.
	 */
	UI_IN_RIGHT,
	UI_IN_ENTER,
	UI_IN_FN,         /**< MENU key: to the menu, or back to Home from it */
	UI_IN_ENCODER,    /**< ui_input_t::delta detents, signed */
	UI_IN_TOUCH,      /**< ui_input_t::x / ::y in pixels */
	UI_IN_PROX,       /**< reed switch closed */
	UI_IN_DISPLAY,    /**< DISPLAY key: cycle Full/Dim/Night/Off */
	UI_IN_ACK,        /**< acknowledge alarms and open the alarm list */
	UI_IN_INFO,       /**< identity card (the Network page) */
	UI_IN_LAMP,       /**< lamp test; ui_input_t::delta 1 = held, 0 = released */
	UI_IN_RESET_HOLD, /**< RESET held past its guard: offer the reboot dialog */
	UI_IN_TICK,       /**< ui_input_t::dt_ms of elapsed time, no user action */
	UI_IN__COUNT
} ui_input_kind_t;

typedef struct {
	uint8_t kind;   /**< ui_input_kind_t */
	int16_t delta;  /**< UI_IN_ENCODER detents; UI_IN_LAMP hold flag */
	uint16_t x;     /**< UI_IN_TOUCH, pixels from the left edge */
	uint16_t y;     /**< UI_IN_TOUCH, pixels from the top edge */
	uint32_t dt_ms; /**< UI_IN_TICK, elapsed milliseconds */
} ui_input_t;

/* ===================================================================== *
 *  Actions (core -> glue)
 * ===================================================================== */

typedef enum {
	UI_ACTION_NONE = 0,
	UI_ACTION_SET_BRIGHTNESS, /**< arg = percent, 10..100 */
	UI_ACTION_SET_TIMEOUT,    /**< arg = seconds, 0 = never blank */
	UI_ACTION_IDENTIFY,       /**< arg = 1 start, 0 stop */
	UI_ACTION_LAMP_TEST,      /**< arg = 1 held, 0 released */
	UI_ACTION_ACK_ALARMS,     /**< arg = alarm id, or UI_ALARM_ALL */
	UI_ACTION_REBOOT,         /**< arg unused; confirmed once */
	UI_ACTION_FACTORY_RESET,  /**< arg unused; confirmed twice */
	UI_ACTION__COUNT
} ui_action_kind_t;

/** UI_ACTION_ACK_ALARMS argument meaning "every latched alarm". */
#define UI_ALARM_ALL (-1)

typedef struct {
	uint8_t kind; /**< ui_action_kind_t */
	uint8_t _pad;
	int16_t _pad2;
	int32_t arg;
	uint32_t mono_ms; /**< module time at which the request was raised */
} ui_action_t;

/**
 * Action queue depth.
 *
 * A user cannot produce more than a couple of requests between two 10 Hz drains;
 * eight covers a stuck-button burst without letting the context grow.
 */
#ifndef UI_ACTION_QUEUE_LEN
#define UI_ACTION_QUEUE_LEN 8
#endif

/* ===================================================================== *
 *  Health input
 * ===================================================================== */

#define UI_MAX_SV 32u
#define UI_MAX_RAILS 12u
#define UI_MAX_ALARMS 16u
#define UI_STR_LEN 24u

/** Satellites listed on the sky page, strongest C/N0 first. */
#define UI_SKY_TOP_N 8u

/**
 * The sky page has two views, cycled with Right/Enter.
 *
 * §6.3 asks for the polar plot; the numeric table stays reachable because it is
 * the view that answers "what exactly is SV 14 doing" and because it is the only
 * one that works when the display is unavailable and the page is being read
 * through the console dump.
 */
typedef enum {
	UI_SKY_VIEW_PLOT = 0, /**< polar skyplot (the default) */
	UI_SKY_VIEW_TABLE,    /**< numeric per-SV table */
	UI_SKY_VIEW__COUNT
} ui_sky_view_t;

/** Name of a sky view, never NULL. */
const char *ui_sky_view_name(uint8_t v);

typedef enum {
	UI_GNSS_GPS = 0,
	UI_GNSS_GALILEO,
	UI_GNSS_GLONASS,
	UI_GNSS_BEIDOU,
	UI_GNSS_SBAS,
	UI_GNSS_QZSS,
	UI_GNSS_OTHER,
	UI_GNSS__COUNT
} ui_gnss_sys_t;

/** Three-letter constellation tag, e.g. "GPS", "GAL". Never NULL. */
const char *ui_gnss_sys_name(uint8_t sys);

typedef struct {
	uint8_t sys;      /**< ui_gnss_sys_t */
	uint8_t svid;     /**< receiver-scoped satellite id */
	int8_t elev_deg;  /**< -90..90 */
	int16_t azim_deg; /**< 0..359, true/GNSS north */
	uint8_t cno_dbhz;
	bool used; /**< in the navigation/timing solution */
} ui_sv_t;

typedef enum {
	UI_ANT_UNKNOWN = 0,
	UI_ANT_OK,
	UI_ANT_OPEN,
	UI_ANT_SHORT,
	UI_ANT_OFF,
	UI_ANT__COUNT
} ui_ant_state_t;

/** Antenna-supervisor state name. Never NULL. */
const char *ui_ant_state_name(uint8_t st);

typedef struct {
	char name[10]; /**< canonical rail net name, e.g. "3V3_STM" */
	uint32_t mv;
	int32_t ma;
	bool pg;    /**< power-good line is asserted */
	bool alert; /**< the rail's INA228 is asserting ALERT */
} ui_rail_t;

typedef struct {
	uint16_t id; /**< fault_alarm_id_t value; core keeps it opaque */
	bool active;
	bool latched;
	uint32_t first_s; /**< uptime seconds at first assertion */
	uint32_t count;
} ui_alarm_row_t;

typedef struct {
	bool valid;
	uint16_t year;
	uint8_t mon;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
} ui_time_t;

/** PTP port role, reduced to what the panel shows. */
typedef enum {
	UI_PTP_DISABLED = 0,
	UI_PTP_LISTENING,
	UI_PTP_MASTER,
	UI_PTP_PASSIVE,
	UI_PTP__COUNT
} ui_ptp_state_t;

/** PTP state name. Never NULL. */
const char *ui_ptp_state_name(uint8_t st);

/**
 * Everything the panel shows that is not in the quality block.
 *
 * The glue fills one of these from its own snapshots before each render; core
 * never retains a pointer to it. All strings are NUL-terminated and fixed
 * length, so the whole structure is memcpy-able and allocation-free.
 */
typedef struct {
	ui_time_t utc;
	uint32_t uptime_s;

	char hostname[UI_STR_LEN];
	char fw_version[UI_STR_LEN];
	char serial[UI_STR_LEN];

	/* network */
	bool link_up;
	bool dhcp;
	uint16_t link_mbps;
	char ipv4[UI_STR_LEN];
	char ipv6[UI_STR_LEN];
	uint32_t ntp_req_per_s;
	uint32_t ntp_served;
	uint32_t ntp_dropped;
	bool nts_enabled;
	uint32_t nts_sessions;
	uint8_t ptp_state; /**< ui_ptp_state_t */
	uint8_t ptp_clock_class;
	uint8_t ptp_clients;

	/* gnss */
	uint8_t sv_count; /**< valid entries in sv[] */
	ui_sv_t sv[UI_MAX_SV];
	uint8_t ant_state; /**< ui_ant_state_t */
	bool survey_active;
	uint32_t survey_dur_s;
	uint32_t survey_acc_mm;

	/* references */
	bool rb_present;
	bool rb_powered;
	bool rb_locked;
	int32_t rb_temp_mc;
	bool extref_ok;
	uint32_t extref_hz;

	/* power / thermal */
	uint8_t rail_count; /**< valid entries in rail[] */
	ui_rail_t rail[UI_MAX_RAILS];
	int32_t temp_enclosure_mc;
	int32_t temp_osc_mc;
	int32_t temp_die_mc;
	uint8_t fan_duty_pct;
	uint16_t fan_rpm;
	uint32_t poe_mw;
	uint8_t poe_class;
	uint16_t supercap_stm_mv;
	uint16_t supercap_gps_mv;
	bool bkp_stm_pg;
	bool bkp_gps_pg;

	/* alarms */
	uint8_t alarm_count; /**< valid entries in alarm[] */
	ui_alarm_row_t alarm[UI_MAX_ALARMS];
	/**
	 * Resolve an alarm id to a display name. Optional: when NULL the page
	 * prints "ALARM <id>". Core cannot depend on `fault`, so the mapping
	 * arrives as a function the glue supplies.
	 */
	const char *(*alarm_name)(uint16_t id);
} ui_health_t;

/* ===================================================================== *
 *  Configuration and context
 * ===================================================================== */

typedef struct {
	uint8_t brightness_pct;      /**< 10..100 */
	uint16_t timeout_s;          /**< 0 = never blank */
	uint16_t identify_timeout_s; /**< identify auto-cancel; 0 = latch on */
} ui_cfg_t;

/** Fill @p cfg with the documented defaults (60 %, 120 s, identify 120 s). */
void ui_cfg_default(ui_cfg_t *cfg);

/** Backlight steps cycled by the DISPLAY key (§3 of the panel-controls doc). */
typedef enum {
	UI_BL_FULL = 0,
	UI_BL_DIM,
	UI_BL_NIGHT,
	UI_BL_OFF,
	UI_BL__COUNT
} ui_bl_step_t;

/** Deepest screen stack: Home -> Menu -> Confirm -> Confirm(second). */
#define UI_STACK_MAX 6u

typedef struct {
	uint8_t page;   /**< ui_page_t */
	uint8_t sel;    /**< selection index within the page */
	uint8_t scroll; /**< first visible body row */
	uint8_t item;   /**< UI_PAGE_EDIT / UI_PAGE_CONFIRM: the menu item */
	uint8_t stage;  /**< UI_PAGE_CONFIRM: 0 = first ask, 1 = second ask */
} ui_frame_t;

typedef struct {
	ui_cfg_t cfg;

	ui_frame_t stack[UI_STACK_MAX];
	uint8_t depth;

	/* wake / backlight policy (spec §6.4) */
	bool awake;
	uint8_t bl_step; /**< ui_bl_step_t */
	uint32_t idle_ms;
	uint32_t now_ms;

	/* transient operator states */
	bool identify;
	uint32_t identify_ms;
	bool lamp_test;

	/* value being edited on UI_PAGE_EDIT, committed on ENTER */
	int32_t edit_val;

	/** Selected view on UI_PAGE_SKYPLOT; ui_sky_view_t. */
	uint8_t sky_view;

	/*
	 * Alarm ids as of the last render of UI_PAGE_ALARMS. ENTER on that page
	 * acknowledges the selected row, and the row->id mapping only exists in
	 * the health snapshot the render saw. Capturing it here keeps
	 * ui_input() free of any dependency on health.
	 */
	uint16_t alarm_id[UI_MAX_ALARMS];
	uint8_t alarm_id_count;

	/*
	 * Geometry of the last rendered surface. A touch arrives as a pixel
	 * coordinate and has to be mapped onto a cell; ui_input() has no
	 * surface, so the mapping uses what the last ui_render() saw. Zero
	 * until the first render, in which case a touch still wakes the panel
	 * but is not located.
	 */
	uint8_t geom_rows;
	uint8_t geom_cols;
	uint8_t geom_cell_h;

	ui_action_t q[UI_ACTION_QUEUE_LEN];
	uint16_t q_head;
	uint16_t q_len;
	uint32_t q_dropped;

	uint32_t inputs;
	uint32_t renders;
	bool started;
} ui_ctx_t;

/* --------------------------------------------------------------- lifecycle */

/**
 * Initialise @p ctx. @p cfg may be NULL for ui_cfg_default().
 *
 * The panel starts awake on Home at full backlight: a unit that has just booted
 * should be readable without anyone touching it.
 *
 * @retval 0        Initialised.
 * @retval -EINVAL  @p ctx is NULL, or @p cfg holds an out-of-range brightness.
 */
int ui_init(ui_ctx_t *ctx, const ui_cfg_t *cfg);

/* ------------------------------------------------------------------- input */

/**
 * Feed one decoded input event.
 *
 * Wake handling comes first for every non-tick event: if the panel is asleep,
 * the event wakes it and is then *consumed*, so the touch that lights the screen
 * cannot also press whatever was underneath it. Touch and proximity additionally
 * return the stack to Home (spec §6.4).
 *
 * @retval 0        Handled.
 * @retval -EINVAL  @p ctx or @p in is NULL, or the kind is out of range.
 */
int ui_input(ui_ctx_t *ctx, const ui_input_t *in);

/** Convenience: post a UI_IN_TICK of @p dt_ms. */
int ui_tick(ui_ctx_t *ctx, uint32_t dt_ms);

/* ----------------------------------------------------------------- outputs */

/** Page currently on top of the stack. */
ui_page_t ui_page(const ui_ctx_t *ctx);

/** Screen-stack depth; 1 on Home. 0 for a NULL context. */
uint8_t ui_depth(const ui_ctx_t *ctx);

/** Selection index of the top frame. */
uint8_t ui_selection(const ui_ctx_t *ctx);

/** True while the idle timeout has not expired. False for a NULL context. */
bool ui_awake(const ui_ctx_t *ctx);

/**
 * Backlight duty target, 0..1000 permille (spec §6.4).
 *
 * Sleep is a duty change only: `DISP_EN`, the 5 V display rail and the rendered
 * frame all persist, so waking is a PWM ramp and not a panel cold start.
 */
uint16_t ui_backlight_permille(const ui_ctx_t *ctx);

/** True while the operator-requested identify pulse is running. */
bool ui_identify(const ui_ctx_t *ctx);

/* -------------------------------------------------------------- action queue */

/**
 * Pop the oldest queued action.
 *
 * @retval 0        Written to @p out.
 * @retval -EINVAL  @p ctx or @p out is NULL.
 * @retval -EAGAIN  Queue empty.
 */
int ui_action_get(ui_ctx_t *ctx, ui_action_t *out);

/** Number of actions waiting. */
size_t ui_action_count(const ui_ctx_t *ctx);

/**
 * Actions discarded because the queue was full.
 *
 * Like `fault`, the queue drops the **newest**: an operator mashing a button
 * must not be able to push an earlier REBOOT or FACTORY_RESET request out of
 * the queue unexecuted.
 */
uint32_t ui_action_dropped(const ui_ctx_t *ctx);

/** Reset the dropped-action counter. */
void ui_action_clear_dropped(ui_ctx_t *ctx);

/* ------------------------------------------------------------------ render */

/**
 * Render the current page into @p s.
 *
 * Pure with respect to hardware and free of hidden state beyond the selection
 * clamping and the alarm-id capture documented on ui_ctx_t. Safe to call at any
 * rate; the glue runs it at 10 Hz.
 *
 * @param ctx  Context.
 * @param q    Quality block snapshot (spec §3.8).
 * @param h    Health snapshot.
 * @param s    Destination surface, already bound by ui_surface_init().
 *
 * @retval 0        Rendered.
 * @retval -EINVAL  Any argument is NULL, or @p s is not bound.
 */
int ui_render(ui_ctx_t *ctx, const quality_block_t *q, const ui_health_t *h,
	      ui_surface_t *s);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_UI_UI_H_ */
