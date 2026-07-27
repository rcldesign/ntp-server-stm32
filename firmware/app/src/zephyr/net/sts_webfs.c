/*
 * STS1000 "Meridian" — static SPA server out of /lfs/www (spec §5.1, §12).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The SPA is pre-compressed by firmware/web/pack_assets.py and dropped on the
 * external NOR as `<name>.gz` plus a `manifest.json`. This file serves those
 * bytes straight through with `Content-Encoding: gzip` — the device never
 * compresses anything at runtime, which is the only way a 250 MHz part with no
 * spare RAM can serve a real UI.
 *
 * NOR is not required to boot (ARCHITECTURE.md §3), and an operator staring at a
 * blank page has no way to fix that. So a minimal page is compiled in and served
 * whenever /lfs is missing, unmounted, or empty: it states what happened, and
 * gives the operator the two things they can still act on — the REST API is up,
 * and the USB console can reflash the bundle.
 *
 * The manifest is read once and cached, so the per-request cost is one
 * fs_open/fs_read; the ETag comes out of the cache rather than being recomputed
 * per request.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>

#include "net/sts_web.h"
#include "storage/sts_store.h"
#include "web/web.h"

LOG_MODULE_REGISTER(sts_webfs, CONFIG_STS1000_LOG_LEVEL);

/* ------------------------------------------------------------------------- */
/* the compiled-in fallback                                                  */
/* ------------------------------------------------------------------------- */

/*
 * Deliberately NOT the generated sts_webfs_assets.h: that header is a build
 * product of firmware/web/pack_assets.py and is not in the tree, so including it
 * would make the firmware un-buildable from a clean checkout. This page is
 * hand-written, tiny, and self-contained.
 */
static const char fallback_page[] =
	"<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">"
	"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
	"<title>STS1000 Meridian</title><style>"
	"body{background:#11151a;color:#d7dde4;font:14px/1.5 system-ui,sans-serif;"
	"margin:0;padding:2.5rem 1.5rem;max-width:44rem}"
	"h1{font-size:1.25rem;margin:0 0 .25rem}"
	"p{margin:.75rem 0}code{background:#1c232b;padding:.1rem .35rem;"
	"border-radius:3px;color:#8fd0ff}"
	"a{color:#8fd0ff}.w{border-left:3px solid #d9a05b;padding-left:.85rem}"
	"</style></head><body>"
	"<h1>STS1000 &ldquo;Meridian&rdquo;</h1>"
	"<p>GPS-disciplined Stratum-1 NTP / NTS / PTP grandmaster.</p>"
	"<div class=\"w\">"
	"<p><strong>The web interface bundle is not installed.</strong> The SPA "
	"lives on the external SPI-NOR at <code>/lfs/www</code>, which is either "
	"absent, unmounted, or empty. The rest of the box is unaffected: time "
	"service, the REST API and the console are all running.</p></div>"
	"<p>What still works from here:</p><ul>"
	"<li>The REST API, e.g. <a href=\"/api/v1/status\">/api/v1/status</a>, "
	"<a href=\"/api/v1/metrics\">/api/v1/metrics</a> "
	"(read routes need a session unless <code>sec.auth.req</code> is off).</li>"
	"<li>The USB-C console: the shell on CDC-ACM&nbsp;0 and the Meridian "
	"Console Protocol on CDC-ACM&nbsp;1, which is also how the bundle and the "
	"firmware get installed.</li>"
	"<li>The front-panel display and buttons.</li></ul>"
	"<p>To install the interface, run "
	"<code>firmware/web/pack_assets.py</code> and copy the resulting "
	"<code>lfs/www</code> tree onto the NOR volume.</p>"
	"</body></html>";

/** ETag of the fallback page. Fixed: the bytes are fixed. */
#define FALLBACK_ETAG "\"fallback-1\""

/* ------------------------------------------------------------------------- */
/* manifest cache                                                            */
/* ------------------------------------------------------------------------- */

#ifndef STS_WEBFS_MAX_ASSETS
#define STS_WEBFS_MAX_ASSETS 8U
#endif

#ifndef STS_WEBFS_MANIFEST_MAX
#define STS_WEBFS_MANIFEST_MAX 1536U
#endif

typedef struct {
	char name[32];
	char content_type[40];
	char etag[24];
	bool valid;
} entry_t;

static entry_t entries[STS_WEBFS_MAX_ASSETS];
static uint8_t entry_count;
static bool have_bundle;
static bool inited;
static struct k_mutex lock;

static const char *content_type_for(const char *name)
{
	size_t n = strlen(name);

	if (n > 5U && strcmp(&name[n - 5], ".html") == 0) {
		return "text/html; charset=utf-8";
	}
	if (n > 4U && strcmp(&name[n - 4], ".css") == 0) {
		return "text/css; charset=utf-8";
	}
	if (n > 3U && strcmp(&name[n - 3], ".js") == 0) {
		return "application/javascript; charset=utf-8";
	}
	if (n > 5U && strcmp(&name[n - 5], ".json") == 0) {
		return "application/json";
	}
	if (n > 4U && strcmp(&name[n - 4], ".svg") == 0) {
		return "image/svg+xml";
	}
	if (n > 4U && strcmp(&name[n - 4], ".ico") == 0) {
		return "image/x-icon";
	}
	return "application/octet-stream";
}

static entry_t *entry_find(const char *name)
{
	uint8_t i;

	for (i = 0U; i < entry_count; i++) {
		if (entries[i].valid && strcmp(entries[i].name, name) == 0) {
			return &entries[i];
		}
	}
	return NULL;
}

/*
 * Parse the packer's manifest. The reader from core/web is reused rather than
 * hand-rolling a second JSON scanner; the manifest is small and its shape is
 * fixed by pack_assets.py:
 *   { "assets": [ {"name":..., "content_type":..., "etag":..., ...}, ... ] }
 */
static void manifest_load(void)
{
	static char buf[STS_WEBFS_MANIFEST_MAX];
	struct fs_file_t f;
	web_json_val_t assets;
	web_json_val_t el;
	size_t cur = 0U;
	ssize_t n;
	int rc;

	entry_count = 0U;
	memset(entries, 0, sizeof(entries));

	fs_file_t_init(&f);
	rc = fs_open(&f, STS_WEBFS_ROOT "/manifest.json", FS_O_READ);
	if (rc != 0) {
		return;
	}
	n = fs_read(&f, buf, sizeof(buf) - 1U);
	(void)fs_close(&f);
	if (n <= 0) {
		return;
	}
	buf[n] = '\0';

	if (web_json_obj_get(buf, (size_t)n, "assets", &assets) != 0 ||
	    assets.type != (uint8_t)WEB_JSON_ARR) {
		LOG_WRN("/lfs/www/manifest.json has no assets array");
		return;
	}
	while (web_json_arr_next(&assets, &cur, &el) == 1) {
		web_json_val_t v;
		entry_t *e;

		if (el.type != (uint8_t)WEB_JSON_OBJ ||
		    entry_count >= STS_WEBFS_MAX_ASSETS) {
			continue;
		}
		e = &entries[entry_count];
		if (web_json_obj_get(el.p, el.n, "name", &v) != 0 ||
		    web_json_str_copy(&v, e->name, sizeof(e->name)) < 0) {
			continue;
		}
		if (web_json_obj_get(el.p, el.n, "content_type", &v) != 0 ||
		    web_json_str_copy(&v, e->content_type,
				      sizeof(e->content_type)) < 0) {
			(void)web_span_copy(e->content_type,
					    sizeof(e->content_type),
					    content_type_for(e->name),
					    strlen(content_type_for(e->name)));
		}
		if (web_json_obj_get(el.p, el.n, "etag", &v) != 0 ||
		    web_json_str_copy(&v, e->etag, sizeof(e->etag)) < 0) {
			e->etag[0] = '\0';
		}
		e->valid = true;
		entry_count++;
	}
	LOG_INF("/lfs/www manifest: %u asset(s)", entry_count);
}

void sts_webfs_init(void)
{
	struct fs_dirent d;

	if (!inited) {
		k_mutex_init(&lock);
		inited = true;
	}
	k_mutex_lock(&lock, K_FOREVER);
	have_bundle = false;
	entry_count = 0U;

	if (!sts_fs_ready()) {
		LOG_WRN("/lfs not mounted; serving the compiled-in fallback page");
		k_mutex_unlock(&lock);
		return;
	}
	/* The bundle exists only if index.html.gz does. */
	if (fs_stat(STS_WEBFS_ROOT "/index.html.gz", &d) != 0) {
		LOG_WRN("%s/index.html.gz absent; serving the fallback page",
			STS_WEBFS_ROOT);
		k_mutex_unlock(&lock);
		return;
	}
	have_bundle = true;
	manifest_load();
	k_mutex_unlock(&lock);
}

bool sts_webfs_have_bundle(void)
{
	return have_bundle;
}

/* ------------------------------------------------------------------------- */
/* resolution                                                               */
/* ------------------------------------------------------------------------- */

/*
 * Accept only a flat, safe file name: [A-Za-z0-9._-], no leading dot, no "..",
 * no separators. The bundle is flat by construction, so nothing legitimate needs
 * a subdirectory and refusing them removes the whole traversal class rather than
 * trying to normalise it away.
 */
static bool name_ok(const char *p, size_t n)
{
	size_t i;

	if (n == 0U || n >= 32U) {
		return false;
	}
	if (p[0] == '.') {
		return false;
	}
	for (i = 0U; i < n; i++) {
		char c = p[i];

		if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
		    (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-') {
			continue;
		}
		return false;
	}
	/* No ".." anywhere, even though '/' is already excluded. */
	for (i = 1U; i < n; i++) {
		if (p[i] == '.' && p[i - 1U] == '.') {
			return false;
		}
	}
	return true;
}

static void fill_fallback(sts_webfs_asset_t *out)
{
	memset(out, 0, sizeof(*out));
	out->content_type = "text/html; charset=utf-8";
	out->etag = FALLBACK_ETAG;
	out->gzip = false;
	out->from_flash = true;
	out->data = (const uint8_t *)fallback_page;
	out->len = sizeof(fallback_page) - 1U;
}

int sts_webfs_resolve(const char *path, size_t path_len, bool gzip_ok,
		      sts_webfs_asset_t *out)
{
	char name[32];
	const char *p;
	size_t n;
	struct fs_dirent d;
	entry_t *e;

	if (path == NULL || out == NULL || path_len == 0U || path[0] != '/') {
		return -EINVAL;
	}

	/* Strip the leading '/'; "" and any unknown path become index.html so a
	 * deep-linked reload of the SPA's hash router still gets the shell. */
	p = &path[1];
	n = path_len - 1U;
	if (n == 0U) {
		p = "index.html";
		n = 10U;
	}
	if (!name_ok(p, n)) {
		if (!have_bundle) {
			fill_fallback(out);
			return 0;
		}
		p = "index.html";
		n = 10U;
	}
	(void)web_span_copy(name, sizeof(name), p, n);

	if (!have_bundle) {
		fill_fallback(out);
		return 0;
	}

	k_mutex_lock(&lock, K_FOREVER);

	/*
	 * Only the pre-compressed form exists on the volume. A client that will
	 * not take gzip is answered with the fallback page rather than a broken
	 * download — every browser since 2000 sends Accept-Encoding: gzip, so
	 * this is a curl-without-flags path, not a real client.
	 */
	(void)snprintf(out->path, sizeof(out->path), "%s/%s.gz", STS_WEBFS_ROOT,
		       name);
	if (fs_stat(out->path, &d) != 0) {
		/* Unknown file: hand back the SPA shell. */
		(void)snprintf(out->path, sizeof(out->path),
			       "%s/index.html.gz", STS_WEBFS_ROOT);
		(void)web_span_copy(name, sizeof(name), "index.html", 10U);
		if (fs_stat(out->path, &d) != 0) {
			k_mutex_unlock(&lock);
			fill_fallback(out);
			return 0;
		}
	}
	if (!gzip_ok) {
		k_mutex_unlock(&lock);
		fill_fallback(out);
		return 0;
	}

	e = entry_find(name);
	out->content_type = (e != NULL && e->content_type[0] != '\0')
				    ? e->content_type
				    : content_type_for(name);
	out->etag = (e != NULL && e->etag[0] != '\0') ? e->etag : "";
	out->gzip = true;
	out->from_flash = false;
	out->data = NULL;
	out->len = (size_t)d.size;
	k_mutex_unlock(&lock);
	return 0;
}

int sts_webfs_read(const sts_webfs_asset_t *a, size_t off, uint8_t *buf,
		   size_t cap)
{
	struct fs_file_t f;
	ssize_t n;
	int rc;

	if (a == NULL || buf == NULL || cap == 0U) {
		return -EINVAL;
	}
	if (off >= a->len) {
		return 0;
	}
	if (a->from_flash) {
		size_t take = a->len - off;

		if (take > cap) {
			take = cap;
		}
		memcpy(buf, &a->data[off], take);
		return (int)take;
	}

	fs_file_t_init(&f);
	rc = fs_open(&f, a->path, FS_O_READ);
	if (rc != 0) {
		return rc;
	}
	rc = fs_seek(&f, (off_t)off, FS_SEEK_SET);
	if (rc != 0) {
		(void)fs_close(&f);
		return rc;
	}
	n = fs_read(&f, buf, cap);
	(void)fs_close(&f);
	if (n < 0) {
		return (int)n;
	}
	return (int)n;
}
