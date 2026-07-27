# STS1000 "Meridian" — operator SPA

The web management interface the device serves over HTTPS from the SPI-NOR
LittleFS volume. Three static files, no build toolchain, no npm, no framework,
no CDN, no external fonts or images. It talks to the firmware's REST API under
`/api/v1/` and the WebSocket telemetry feed at `/ws`.

Dark, dense, instrument-panel layout. Designed for a laptop, usable down to
900 px wide (the nav collapses to a horizontal strip).

---

## Files

| File | Role |
|---|---|
| `index.html` | Shell: nav, header status pills, login overlay, modal, toast host. Static markup only — every data-bearing node is built in JS. |
| `app.css` | Theme and layout. CSS custom properties, grid, no preprocessor. |
| `app.js` | The whole application: API client, session/CSRF handling, WSS feed with REST fallback, hand-drawn canvas widgets, hash router, nine pages. |
| `pack_assets.py` | Packer — gzips the three files and emits the `/lfs/www` tree plus a fallback C header. |
| `README.md` | This file. |

## Size (gzip, measured)

| Asset | Raw | Gzip | Ratio |
|---|---:|---:|---:|
| `index.html` | 4 142 | 1 366 | 33.0 % |
| `app.css` | 19 111 | 4 623 | 24.2 % |
| `app.js` | 136 114 | 37 226 | 27.3 % |
| **Total** | **159 367** | **43 215** | **27.1 %** |

**42.2 KiB gzipped** — 16.5 % of the 256 KiB budget. Run the packer for the
current figure; the table above is regenerated on every run.

## Browser requirements

- `fetch` with `credentials: 'same-origin'`
- `WebSocket`
- `crypto.subtle.digest` — used to SHA-256 the firmware image in the browser.
  This is a **secure-context API**: the firmware-upload page only works over
  HTTPS (or `http://localhost`). Everything else works either way.
- `<canvas>` 2D context — all charts and the skyplot
- `Blob` / `File.arrayBuffer()` / `URL.createObjectURL` — uploads and downloads
- ES2019 (`async`/`await`, `const`/`let`, arrow functions, template literals).
  No optional chaining or nullish coalescing, so ES2019 is genuinely the floor.

Any current Chrome, Firefox, Safari or Edge is fine. There is no IE fallback and
no transpilation step.

---

## Running the packer

```sh
python3 pack_assets.py                       # src = this dir, out = ./build
python3 pack_assets.py --src . --out-dir /tmp/www
python3 pack_assets.py --max-bytes 131072    # tighter budget
python3 pack_assets.py --assets index.html app.css app.js extra.svg
```

Stdlib only (`gzip`, `hashlib`, `json`, `argparse`, `pathlib`). It exits
non-zero if the total gzip size exceeds `--max-bytes` (default 262 144), so it
works as a CI size gate.

### Output

```
build/
  lfs/www/
    index.html.gz        ← the tree the device serves from /lfs/www
    app.css.gz
    app.js.gz
    manifest.json        ← {name, content_type, size, gzip_size, sha256, etag}
  sts_webfs_assets.h     ← FALLBACK ONLY (see below)
```

`build/` is already covered by the repo's `.gitignore`.

Serve the `.gz` payloads with `Content-Encoding: gzip` and the `content_type`
and `etag` from the manifest. The ETag is the first 16 hex characters of the
SHA-256 of the **uncompressed** bytes, quoted per RFC 9110 — so it is stable
even if the compressor changes.

Output is **deterministic and idempotent**: gzip `mtime` is pinned to 0, the
manifest is key-sorted, and files are only rewritten when their content
actually differs (`files updated : 0` on a no-op run). Repacking unchanged
sources reproduces byte-identical output, which keeps the NOR image
reproducible.

### The C header is a fallback

`sts_webfs_assets.h` contains the same gzip bytes as C arrays plus:

```c
typedef struct {
	const char *path;
	const char *content_type;
	const unsigned char *gz;
	unsigned int gz_len;
	const char *etag;
} sts_web_asset_t;

static const sts_web_asset_t sts_web_assets[] = { /* ... */ };
#define STS_WEB_ASSET_COUNT 3
```

It is guarded with `STS1000_ZEPHYR_NET_STS_WEBFS_ASSETS_H_` and **nothing
includes it automatically**. The normal delivery path is the `/lfs/www` tree,
which lets the UI be replaced without rebuilding firmware. Compile the header in
only when an image must serve the UI before the filesystem mounts — for example
a recovery build. It costs ~43 KiB of internal flash.

---

## What each page does

All nine pages are implemented against live data — none is a placeholder.

1. **Dashboard** — tiles for stratum, lock state, reference, PPS offset,
   GNSS fix, holdover and active alarms; a 300-sample phase-offset trend;
   quality/holdover readouts (mean, sigma, freq error, root delay/dispersion,
   time-to-demote); the skyplot; NTP/NTS counters with per-service up/off pills;
   PTP summary; a compact rail table; environment and PoE budget bar; the
   active/latched alarm list; a link to `/api/v1/metrics`.
2. **Timing** — the lock-state machine as a stepper plus reference pills; full
   loop state; four canvas plots (phase error, frequency error, Vc commanded vs
   sensed as two series, and ADEV on log-log axes at τ = 1/10/100 s); a guarded
   manual reference override (auto/ocxo/rb) behind a confirm dialog that
   surfaces `guard_not_satisfied` in plain language; the `tim.*` key editor.
3. **Calibration** — the four procedures (`holdover`, `ocxo_tune`, `ina_trim`,
   `compass`) each with a description, an argument field (`ina_trim` gets a
   named rail selector for `cal.ina.0..8`) and a confirm dialog; the `CFG_F_CAL`
   constants from `GET /calibration` as an editable table.
4. **GNSS** — fix/SV/time-accuracy/leap/antenna/survey tiles; the large polar
   skyplot; survey-in with start/stop and progress bars against
   `gnss.survey.acc` and `gnss.survey.dur`; stored ECEF position with a manual
   set form; antenna supervisor and receiver versions; a C/N0-sorted satellite
   table; the `gnss.*` key editor.
5. **Network** — interface/IP/DHCP/MAC/hostname state; PTP engine and MAC-clock
   servo state (port state mapped to its 1588 name); service enable/disable for
   ntp, nts, ptp, snmp and syslog; the `net.*`, `ntp.*`, `nts.*` and `ptp.*`
   key editors.
6. **Security** — users and roles with credential/lockout state; password change
   with byte-accurate length validation and an inline commit offer; TLS
   certificate details with SHA-256 fingerprint, PEM upload (paste or file), CSR
   generation and download, ACME state (read-only); configuration export/import;
   the `sec.*` key editor; factory reset behind a typed `FACTORY` confirmation.
7. **Power/Health** — PoE/thermal/humidity/fan/alert tiles; all nine INA228
   rails with bus voltage, current, power, a load bar against
   `design_max_ma`, and alert state; both TMP117s, MCU die temp and SHT45
   humidity; PoE class/draw/budget with a utilisation bar; supercap PG flags;
   the alarm latch list.
8. **Logs** — live tail over the WSS `logs` group with level and subsystem
   filters, pause/resume, clear, and a paged REST download that saves a text
   file; cursor/head/oldest/dropped metadata; the `log.*` key editor; a link to
   `/api/v1/metrics`.
9. **Firmware** — both slots with version/valid/active/pending/confirmed; the
   upload flow (browser-side SHA-256 → `begin` → block-aligned chunks →
   `end`) with a progress bar and `offset_gap` rewind; confirm/revert offered
   only while `pending_confirm`; reboot, bootloader-recovery and halt-to-test
   controls.

### Configuration editing

One editor drives every `*.` key group. It types each field from the schema
(bool → checkbox, integers → range-bounded number input, str/blob → text with
`maxlen`), shows staged values as `→ new`, tags `reboot`/`secret`/`cal` keys,
and validates client-side before staging. Edits are staged with
`PUT /api/v1/config`; a floating bar then reports the staged count, warns how
many keys need a reboot, and offers commit or discard.

`f32` keys are sent the way the device parses them — a whole number as a plain
integer, anything fractional as integer `{"micro": N}` micro-units — because the
firmware deliberately has no decimal float parser. `blob` keys are lowercase
hex, validated for even length before submission.

### Security posture

- No `innerHTML`, `outerHTML`, `insertAdjacentHTML`, `document.write`, `eval`
  or `new Function` anywhere. Every server-supplied value reaches the DOM
  through `textContent` or `createTextNode`. Verified by grep — the only matches
  for those names in `app.js` are inside the header comment.
- `index.html` ships a restrictive CSP (`default-src 'none'`, `script-src
  'self'`, `connect-src 'self'`, `frame-ancestors 'none'`, `base-uri 'none'`)
  and `referrer: no-referrer`.
- Every request uses `credentials: 'same-origin'`; every POST/PUT carries
  `X-CSRF-Token`. A `403 csrf_failed` re-fetches the session and retries **once**.
- `401` shows the login overlay and stops the telemetry feed — including a 401
  that arrives on a background poll, so a session that expires while the tab
  sits idle prompts for sign-in rather than silently going stale.
- `403 insufficient_role` surfaces as "your role may not perform this action";
  controls above the session's role are rendered disabled with a note rather
  than failing on click.
- Secret keys are never rendered as values — a withheld secret shows
  `set`/`not set` with a write-only field. `sec.admin.pw` and any secret blob
  are read-only in the generic editor and routed through
  `POST /security/password`, because writing a raw blob into a credential slot
  would corrupt it.

### Telemetry transport

Connects to `wss://<host>/ws` (derived from `location`), subscribes to the seven
status groups at 1 Hz, and adds the `logs` group only while the Logs page is
mounted. Reconnects with exponential backoff — 1 s, 2 s, 4 s, 8 s, capped at
15 s — and falls back to polling `GET /api/v1/status` at 1 Hz whenever the
socket is not open. One priming poll is issued at start-up so the first paint
does not wait on the WebSocket handshake; polling then stops for as long as the
socket stays healthy. The feed pill in the header reads `live`, `poll 1Hz`,
`offline` or `idle`.

Trend samples are keyed on the discipline `tick`, so REST polling and WSS frames
cannot double-push the same 1 Hz sample. Alarm masks are up to 2^48, so they are
counted and formatted without 32-bit bitwise operators.

### The skyplot

Hand-drawn on `<canvas>`, mirroring the local-UI renderer:

- Polar, north up, azimuth clockwise; elevation 90° at the centre, 0° at the rim
  (`r = (90 − elev) / 90 × R`).
- Horizon circle, 30°/60° elevation rings with labels, a zenith cross, azimuth
  spokes every 30°, and N/E/S/W ticks outside the rim.
- The elevation-mask band is a shaded annulus from the rim inward to
  `gnss.elev.mask`, with a dashed inner edge.
- One marker per satellite coloured by constellation, filled when `used` and
  hollow when only tracked, radius scaled slightly by C/N0, with the SV number
  beside it and a hover tooltip giving constellation, SV, C/N0, elevation,
  azimuth and whether it is in the solution.
- Elevation is clamped to 0…90 before projection, so a negative elevation clips
  to the rim instead of reflecting through the centre; a missing azimuth is
  treated as 0.
- Redrawn on every telemetry frame, and on resize, at `devicePixelRatio`.
- When `detail_available` is false it draws the empty sky with an explicit
  "no receiver detail" note — never a spinner and never an error.

---

## Known gaps and assumptions

Honest list. Nothing below is hidden behind a placeholder that looks functional.

1. **`/ws` works, but the server does not actually check the path.** The upgrade
   is routed by *header*, not by URL: `sts_web.c:1418` dispatches on
   `HTTP_F_UPGRADE_WS`, authenticates, then runs `ws_handshake()` — so a
   `Upgrade: websocket` request on **any** path is accepted and `WS_PATH` in
   `app.js` can be anything. Auth is resolved before the handshake, so this is
   loose rather than unsafe. If the server later gains a real path check, `/ws`
   is the path to register.
2. **Log level and subsystem filtering on the live tail is client-side.** The
   WSS `subscribe` op accepts only `groups`, `rate` and `log_cursor` — there is
   no server-side level filter for the stream. The REST download does pass
   `level` to the server. Records already dropped by the device's own
   `log.level` threshold never reach the browser at all.
3. **Log download pages at 32 records per request** because that is the server's
   `REST_LOG_PAGE_MAX` cap. It stops at 20 000 records to bound a runaway ring.
4. **No mDNS-specific config keys exist** in the schema, so the Network page
   surfaces `net.hostname` (which is also the mDNS instance name) and
   `net.mgmt.acl`, and says so. There is nothing to expose for `net.mdns.*`.
5. **The reference override mode is not read back.** `POST /timing/reference`
   accepts auto/ocxo/rb but no endpoint reports which mode is in force, so the
   Timing page shows the *active* reference from telemetry and labels the three
   buttons as requests. The page states this rather than implying a readback.
6. **ACME is display-only** — there is no API to drive enrolment, so the SPA
   shows state and detail and nothing more.
7. **`ptp.clock_accuracy` is shown as a hex enum value**, not mapped to its
   IEEE 1588 name; `port_state` and `transport` *are* mapped.
8. **Only the skyplot has a tooltip.** The trend and ADEV plots label their last
   value and axes but have no hover crosshair.
9. **`osc_temp_c`, `vc_sense_mv`, `die_c`, `humidity_pct` and per-rail readings
   render as `n/a` when the API sends `null`** rather than being hidden, so a
   missing sensor is visibly distinct from a zero reading. Likewise a whole
   status group arriving as `null` produces an explicit "No … provider in this
   build" panel.
10. **No checked-in test suite.** The SPA was verified by `node --check`, a
    scratchpad DOM/API harness that boots the app, walks all nine pages and
    drives 25 flow assertions (config staging and commit, the firmware upload
    including a forced `offset_gap` rewind and block alignment, null providers,
    the `detail_available: false` sky, the 401 path, and the WSS transport
    including log streaming and subscription add/drop), plus verification that
    the packer output round-trips and that the emitted header compiles under
    `-Wall -Wextra -Werror -pedantic`. The harness is not part of this
    deliverable and is not in the repo.
