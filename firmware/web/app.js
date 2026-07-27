/* STS1000 "Meridian" - operator SPA.
 *
 * Plain ES2019. No framework, no build step, no external resources.
 * Served as three static files from /lfs/www on the device.
 *
 * Security posture: this is a management plane. Server-supplied data is only
 * ever written through textContent / createElement - there is no innerHTML
 * assignment anywhere in this file, and no eval / new Function.
 */
(function () {
	'use strict';

	var API = '/api/v1/';
	var WS_PATH = '/ws';
	var RING_MAX = 300;   /* timing trend samples */
	var LOG_MAX = 600;    /* rendered log records */
	var LOG_DL_MAX = 20000;

	/* --------------------------------------------------------------- palette
	 * Kept in JS as well as CSS because canvas cannot read custom properties
	 * without a getComputedStyle round-trip on every frame. */
	var C = {
		line: '#1f2b3a', line2: '#2b3a4d',
		fg: '#c9d6e4', fg2: '#8ea1b6', fg3: '#5f7288',
		accent: '#35c8f0', ok: '#3ddc84', amber: '#f5a524',
		red: '#ff5c68', violet: '#b48ef7', blue: '#7aa2f7',
		grid: '#16202c'
	};

	var CONST_COLOR = {
		gps: '#35c8f0', galileo: '#3ddc84', glonass: '#f5a524',
		beidou: '#ff5c68', qzss: '#b48ef7', sbas: '#7aa2f7',
		imes: '#e0e6ee', other: '#8ea1b6'
	};

	var LOCK_STEPS = ['acquiring', 'locking', 'locked', 'holdover', 'recovering'];
	var LEVEL_NAME = ['emerg', 'alert', 'crit', 'err', 'warning', 'notice', 'info', 'debug'];

	var PTP_PORT_STATE = {
		1: 'INITIALIZING', 2: 'FAULTY', 3: 'DISABLED', 4: 'LISTENING',
		5: 'PRE_MASTER', 6: 'MASTER', 7: 'PASSIVE', 8: 'UNCALIBRATED', 9: 'SLAVE'
	};
	var PTP_TRANSPORT = { 0: 'UDP/IPv4', 1: 'UDP/IPv6', 2: 'L2 (0x88F7)' };
	var ACME_STATE = ['disabled', 'idle', 'account', 'order', 'challenge',
		'finalize', 'issued', 'failed'];
	var DFU_STATE = ['idle', 'query', 'prepare', 'transfer', 'verify',
		'restore', 'done', 'failed'];
	/* cal.ina.0..8, in the order fixed by the comment on the cal group. */
	var INA_RAILS = ['PoE input', 'STM 3V3', '5V_DISP', 'main 3V3', 'antenna bias',
		'OCXO', 'VCC_RB', 'GPS VCC', 'panel LED 5V'];

	var ROLES = { none: 0, viewer: 1, operator: 2, admin: 3 };

	/* ===================================================================== */
	/* DOM helpers                                                           */
	/* ===================================================================== */

	function $(id) { return document.getElementById(id); }

	function add(n, kids) {
		if (kids === null || kids === undefined || kids === false || kids === true) { return; }
		if (Array.isArray(kids)) {
			for (var i = 0; i < kids.length; i++) { add(n, kids[i]); }
			return;
		}
		if (typeof kids === 'string' || typeof kids === 'number') {
			n.appendChild(document.createTextNode(String(kids)));
			return;
		}
		n.appendChild(kids);
	}

	/* el('div', {cls:'x', text:'y', on:{click:fn}}, [children]) */
	function el(tag, attrs, kids) {
		var n = document.createElement(tag);
		if (attrs) {
			for (var k in attrs) {
				if (!Object.prototype.hasOwnProperty.call(attrs, k)) { continue; }
				var v = attrs[k];
				if (v === null || v === undefined || v === false) { continue; }
				if (k === 'text') { n.textContent = String(v); }
				else if (k === 'cls') { n.className = String(v); }
				else if (k === 'on') {
					for (var e in v) {
						if (Object.prototype.hasOwnProperty.call(v, e)) { n.addEventListener(e, v[e]); }
					}
				} else if (k === 'value' || k === 'checked' || k === 'disabled' ||
					k === 'hidden' || k === 'readOnly' || k === 'selected') {
					n[k] = (k === 'value') ? String(v) : !!v;
				} else if (v === true) { n.setAttribute(k, ''); }
				else { n.setAttribute(k, String(v)); }
			}
		}
		add(n, kids);
		return n;
	}

	function clear(n) { while (n.firstChild) { n.removeChild(n.firstChild); } }

	function panel(title, body, extra) {
		var h = el('h2', null, title);
		if (extra) { h.appendChild(el('span', { cls: 'hx' }, extra)); }
		return el('section', { cls: 'panel' }, [h, body]);
	}

	function pbody(kids, tight) {
		return el('div', { cls: tight ? 'pbody tight' : 'pbody' }, kids);
	}

	function kv(pairs) {
		var d = el('dl', { cls: 'kv' });
		for (var i = 0; i < pairs.length; i++) {
			var p = pairs[i];
			if (!p) { continue; }
			d.appendChild(el('dt', { text: p[0] }));
			d.appendChild(el('dd', { cls: p[2] || null }, p[1] === null || p[1] === undefined
				? '—' : (typeof p[1] === 'object' ? p[1] : String(p[1]))));
		}
		return d;
	}

	function tile(label, value, sub, cls) {
		return el('div', { cls: 'tile' + (cls ? ' ' + cls : '') }, [
			el('div', { cls: 'tl', text: label }),
			el('div', { cls: 'tv', text: value === null || value === undefined ? '—' : String(value) }),
			el('div', { cls: 'ts', text: sub || '' })
		]);
	}

	function table(cols, cls) {
		var thead = el('thead'), tr = el('tr');
		for (var i = 0; i < cols.length; i++) {
			var c = cols[i];
			tr.appendChild(el('th', { cls: (c && c.num) ? 'num' : null },
				(c && c.t !== undefined) ? c.t : String(c)));
		}
		thead.appendChild(tr);
		var tb = el('tbody');
		var t = el('table', { cls: 't' + (cls ? ' ' + cls : '') }, [thead, tb]);
		return { el: el('div', { cls: 'tw' }, t), body: tb, table: t };
	}

	function cell(td, text, cls) {
		var s = (text === null || text === undefined || text === '') ? '—' : String(text);
		if (td.textContent !== s) { td.textContent = s; }
		var c = cls || '';
		if (td.className !== c) { td.className = c; }
	}

	/* Keyed <tr> reconciler: stable nodes across frames, no per-frame churn. */
	function RowSet(tbody, ncols) {
		var map = Object.create(null);
		return {
			sync: function (items, keyOf, fill) {
				var seen = Object.create(null), i, j;
				for (i = 0; i < items.length; i++) {
					var key = String(keyOf(items[i], i));
					seen[key] = 1;
					var row = map[key];
					if (!row) {
						var tr = el('tr'), tds = [];
						for (j = 0; j < ncols; j++) {
							var td = el('td');
							tds.push(td);
							tr.appendChild(td);
						}
						row = { tr: tr, tds: tds };
						map[key] = row;
					}
					fill(row.tds, items[i], row.tr, i);
					var at = tbody.childNodes[i];
					if (at !== row.tr) { tbody.insertBefore(row.tr, at || null); }
				}
				for (var k in map) {
					if (!seen[k]) {
						if (map[k].tr.parentNode) { tbody.removeChild(map[k].tr); }
						delete map[k];
					}
				}
			},
			clear: function () { clear(tbody); map = Object.create(null); }
		};
	}

	function bar(frac, cls) {
		var b = el('div', { cls: 'bar' + (cls ? ' ' + cls : '') });
		var i = el('i');
		b.appendChild(i);
		b._fill = i;
		setBar(b, frac, cls);
		return b;
	}

	function setBar(b, frac, cls) {
		var f = (typeof frac === 'number' && isFinite(frac)) ? frac : 0;
		if (f < 0) { f = 0; }
		if (f > 1) { f = 1; }
		b._fill.style.width = (f * 100).toFixed(1) + '%';
		var c = 'bar' + (cls ? ' ' + cls : '');
		if (b.className !== c) { b.className = c; }
	}

	function dot(cls, label) {
		return el('span', null, [el('span', { cls: 'dot ' + (cls || 'off') }), label]);
	}

	function empty(msg) { return el('div', { cls: 'empty', text: msg }); }

	function hint(kids) { return el('div', { cls: 'hint' }, kids); }

	function code(s) { return el('code', { cls: 'k', text: s }); }

	/* ===================================================================== */
	/* formatting                                                            */
	/* ===================================================================== */

	function isNum(v) { return typeof v === 'number' && isFinite(v); }
	function num(v) { return isNum(v) ? v : null; }

	function grp(s) { return String(s).replace(/\B(?=(\d{3})+(?!\d))/g, ' '); }

	function fint(v) { return isNum(v) ? grp(Math.round(v)) : '—'; }

	function ffix(v, dp) { return isNum(v) ? v.toFixed(dp) : '—'; }

	function fns(v) { return isNum(v) ? grp(Math.round(v)) + ' ns' : '—'; }

	/* Compact significant-figure formatter for axis labels. */
	function fsig(v) {
		if (!isNum(v)) { return '—'; }
		var a = Math.abs(v);
		if (a === 0) { return '0'; }
		if (a >= 1e6 || a < 1e-4) { return v.toExponential(1); }
		if (a >= 1000) { return v.toFixed(0); }
		if (a >= 100) { return v.toFixed(1); }
		if (a >= 1) { return v.toFixed(2); }
		return v.toFixed(4);
	}

	function fdur(s) {
		if (!isNum(s)) { return '—'; }
		s = Math.floor(s);
		var d = Math.floor(s / 86400);
		var h = Math.floor((s % 86400) / 3600);
		var m = Math.floor((s % 3600) / 60);
		var q = s % 60;
		function p2(n) { return (n < 10 ? '0' : '') + n; }
		return (d > 0 ? d + 'd ' : '') + p2(h) + ':' + p2(m) + ':' + p2(q);
	}

	function fbytes(n) {
		if (!isNum(n)) { return '—'; }
		if (n < 1024) { return n + ' B'; }
		if (n < 1048576) { return (n / 1024).toFixed(1) + ' KiB'; }
		return (n / 1048576).toFixed(2) + ' MiB';
	}

	function fmono(ms) {
		if (!isNum(ms)) { return '—'; }
		var s = Math.floor(ms / 1000);
		var f = Math.floor(ms % 1000);
		function p2(n) { return (n < 10 ? '0' : '') + n; }
		function p3(n) { return (n < 10 ? '00' : n < 100 ? '0' : '') + n; }
		return p2(Math.floor(s / 3600) % 100) + ':' + p2(Math.floor(s / 60) % 60) +
			':' + p2(s % 60) + '.' + p3(f);
	}

	function maskHex(v) {
		if (!isNum(v)) { return '—'; }
		return '0x' + Math.floor(v).toString(16);
	}

	/* Masks can exceed 32 bits, so no bitwise operators here. */
	function popcount(v) {
		if (!isNum(v) || v <= 0) { return 0; }
		var n = 0;
		v = Math.floor(v);
		while (v >= 1) {
			if (v % 2 === 1) { n++; }
			v = Math.floor(v / 2);
		}
		return n;
	}

	function hexOf(u8) {
		var s = '';
		for (var i = 0; i < u8.length; i++) {
			s += (u8[i] < 16 ? '0' : '') + u8[i].toString(16);
		}
		return s;
	}

	function ipOf(v) { return typeof v === 'string' ? v : '—'; }

	function boolTxt(v, yes, no) {
		if (typeof v !== 'boolean') { return '—'; }
		return v ? (yes || 'yes') : (no || 'no');
	}

	function saveBlob(blob, name) {
		var url = URL.createObjectURL(blob);
		var a = el('a', { href: url, download: name });
		document.body.appendChild(a);
		a.click();
		document.body.removeChild(a);
		setTimeout(function () { URL.revokeObjectURL(url); }, 20000);
	}

	/* ===================================================================== */
	/* toast / modal                                                         */
	/* ===================================================================== */

	function toast(msg, kind, title) {
		var t = el('div', { cls: 'toast' + (kind ? ' ' + kind : '') });
		if (title) { t.appendChild(el('b', { text: title })); }
		t.appendChild(document.createTextNode(String(msg)));
		$('toasts').appendChild(t);
		setTimeout(function () {
			if (t.parentNode) { t.parentNode.removeChild(t); }
		}, kind === 'err' ? 9000 : 5000);
	}

	function errToast(e, what) {
		var m = (e && e.code) ? e.code : String(e);
		if (e && e.detail) { m += ' — ' + e.detail; }
		if (e && e.code === 'insufficient_role') {
			m = 'your role may not perform this action';
		}
		toast(m, 'err', what || 'request failed');
	}

	/* confirm(opts) -> Promise<value|null>.  opts.body may add form nodes;
	 * opts.value() supplies the resolved value; opts.enable() gates OK. */
	function confirmDlg(opts) {
		return new Promise(function (resolve) {
			var m = $('modal');
			var ok = $('mdl-ok'), cancel = $('mdl-cancel');
			$('mdl-title').textContent = opts.title || 'Confirm';
			var body = $('mdl-body');
			clear(body);
			if (opts.text) { body.appendChild(el('p', { text: opts.text })); }
			if (opts.body) { add(body, opts.body); }
			ok.textContent = opts.okText || 'Confirm';
			ok.className = 'btn ' + (opts.okClass || 'danger');
			var prev = document.activeElement;

			function gate() {
				ok.disabled = opts.enable ? !opts.enable() : false;
			}
			function done(v) {
				m.hidden = true;
				ok.removeEventListener('click', onOk);
				cancel.removeEventListener('click', onNo);
				document.removeEventListener('keydown', onKey);
				body.removeEventListener('input', gate);
				clear(body);
				if (prev && prev.focus) { try { prev.focus(); } catch (e) { /* gone */ } }
				resolve(v);
			}
			function onOk() {
				if (ok.disabled) { return; }
				done(opts.value ? opts.value() : true);
			}
			function onNo() { done(null); }
			function onKey(ev) {
				if (ev.key === 'Escape') { ev.preventDefault(); onNo(); }
				else if (ev.key === 'Enter' && ev.target.tagName !== 'TEXTAREA') {
					ev.preventDefault(); onOk();
				}
			}
			ok.addEventListener('click', onOk);
			cancel.addEventListener('click', onNo);
			document.addEventListener('keydown', onKey);
			body.addEventListener('input', gate);
			gate();
			m.hidden = false;
			var f = body.querySelector('input, textarea, select');
			if (f) { f.focus(); } else { ok.focus(); }
		});
	}

	/* ===================================================================== */
	/* API layer                                                             */
	/* ===================================================================== */

	function ApiError(cod, status, detail, extra) {
		this.name = 'ApiError';
		this.code = cod;
		this.status = status || 0;
		this.detail = detail || '';
		this.extra = extra || {};
		this.message = cod + (detail ? ': ' + detail : '');
	}
	ApiError.prototype = Object.create(Error.prototype);
	ApiError.prototype.constructor = ApiError;

	function readErr(res) {
		return res.text().then(function (t) {
			var out = { code: 'http_' + res.status, detail: '', extra: {} };
			if (t) {
				try {
					var j = JSON.parse(t);
					if (j && typeof j === 'object') {
						if (typeof j.error === 'string') { out.code = j.error; }
						if (typeof j.detail === 'string') { out.detail = j.detail; }
						out.extra = j;
					}
				} catch (e) { out.detail = t.slice(0, 140); }
			}
			return out;
		}, function () { return { code: 'http_' + res.status, detail: '', extra: {} }; });
	}

	/* req(method, path, opts)
	 *   opts.json    - JSON body
	 *   opts.body    - raw body (ArrayBuffer / string / Blob) with opts.type
	 *   opts.expect  - 'json' (default) | 'text' | 'blob' | 'none'
	 *   opts.quiet   - do not force the login overlay on 401 */
	function req(method, path, opts) {
		opts = opts || {};
		var headers = {};
		var body;
		if (opts.json !== undefined) {
			headers['Content-Type'] = 'application/json';
			body = JSON.stringify(opts.json);
		} else if (opts.body !== undefined) {
			body = opts.body;
			if (opts.type) { headers['Content-Type'] = opts.type; }
		}
		var mutating = (method !== 'GET' && method !== 'HEAD');
		if (mutating && Session.csrf) { headers['X-CSRF-Token'] = Session.csrf; }

		return fetch(API + path, {
			method: method,
			headers: headers,
			body: body,
			credentials: 'same-origin',
			cache: 'no-store',
			redirect: 'error'
		}).catch(function (e) {
			throw new ApiError('network_error', 0,
				(e && e.message) ? e.message : String(e));
		}).then(function (res) {
			if (res.status === 401) {
				Session.authenticated = false;
				Session.role = 'none';
				if (!opts.quiet) { Session.requireLogin('Session ended. Sign in again.'); }
				throw new ApiError('unauthenticated', 401, '');
			}
			if (!res.ok) {
				return readErr(res).then(function (e) {
					/* One retry after re-fetching the CSRF token. */
					if (res.status === 403 && e.code === 'csrf_failed' &&
						mutating && !opts._retried) {
						var o2 = {};
						for (var k in opts) {
							if (Object.prototype.hasOwnProperty.call(opts, k)) { o2[k] = opts[k]; }
						}
						o2._retried = true;
						return Session.refresh().then(function () {
							return req(method, path, o2);
						});
					}
					if (res.status === 429) {
						e.extra.retry_after = res.headers.get('Retry-After');
					}
					throw new ApiError(e.code, res.status, e.detail, e.extra);
				});
			}
			if (opts.expect === 'blob') { return res.blob(); }
			if (opts.expect === 'text') { return res.text(); }
			if (opts.expect === 'none') { return null; }
			if (res.status === 204) { return null; }
			return res.text().then(function (t) {
				if (!t) { return null; }
				try { return JSON.parse(t); }
				catch (e) { throw new ApiError('bad_json', res.status, t.slice(0, 140)); }
			});
		});
	}

	/* ===================================================================== */
	/* session                                                               */
	/* ===================================================================== */

	var Session = {
		authenticated: false,
		authRequired: true,
		role: 'none',
		user: '',
		csrf: null,
		idle: 0,
		abs: 0,
		age: 0,
		requests: 0,

		can: function (need) {
			return (ROLES[this.role] || 0) >= (ROLES[need] || 0);
		},

		apply: function (s) {
			if (!s || typeof s !== 'object') { return; }
			this.authenticated = !!s.authenticated;
			if (typeof s.auth_required === 'boolean') { this.authRequired = s.auth_required; }
			this.role = typeof s.role === 'string' ? s.role : 'none';
			this.user = typeof s.user === 'string' ? s.user : '';
			if (typeof s.csrf_token === 'string') { this.csrf = s.csrf_token; }
			if (isNum(s.age_s)) { this.age = s.age_s; }
			if (isNum(s.requests)) { this.requests = s.requests; }
			if (isNum(s.idle_timeout_s)) { this.idle = s.idle_timeout_s; }
			if (isNum(s.absolute_timeout_s)) { this.abs = s.absolute_timeout_s; }
			paintRole();
		},

		refresh: function () {
			var self = this;
			return req('GET', 'auth/session', { quiet: true }).then(function (s) {
				self.apply(s);
				return s;
			});
		},

		login: function (user, pass) {
			var self = this;
			return req('POST', 'auth/login', {
				json: { user: user, password: pass }, quiet: true
			}).then(function (r) {
				self.authenticated = true;
				self.user = user;
				self.apply(r);
				return r;
			});
		},

		logout: function () {
			var self = this;
			return req('POST', 'auth/logout', { quiet: true }).catch(function () {
				return null;
			}).then(function () {
				self.authenticated = false;
				self.role = 'none';
				self.csrf = null;
				Live.stop();
				self.requireLogin('Signed out.');
			});
		},

		requireLogin: function (msg) {
			Live.stop();
			var lg = $('login');
			if (!lg.classList.contains('show')) {
				lg.classList.add('show');
				$('lg-pass').value = '';
				var u = $('lg-user');
				if (Session.user) { u.value = Session.user; }
			}
			var m = $('lg-msg');
			m.className = 'lg-msg info';
			m.textContent = msg || '';
			setTimeout(function () { $('lg-pass').focus(); }, 30);
		},

		hideLogin: function () {
			$('login').classList.remove('show');
			$('lg-msg').textContent = '';
			$('lg-pass').value = '';
		}
	};

	function paintRole() {
		var p = $('p-role');
		p.lastChild.textContent = Session.role || '—';
		p.className = 'pill ' + (Session.role === 'admin' ? 'info'
			: Session.role === 'none' ? 'mute' : 'ok');
		$('btn-logout').hidden = !Session.authenticated;
	}

	/* ===================================================================== */
	/* config store                                                          */
	/* ===================================================================== */

	var Cfg = {
		keys: [], byName: Object.create(null), byId: Object.create(null),
		schema: 0, staged: 0, loaded: false, pending: null,

		load: function (force) {
			var self = this;
			if (this.pending) { return this.pending; }
			if (this.loaded && !force) { return Promise.resolve(this); }
			var acc = [], guard = 0;

			function page(start) {
				return req('GET', 'config?start=' + start).then(function (r) {
					if (!r) { throw new ApiError('no_config', 0, ''); }
					self.schema = isNum(r.schema_version) ? r.schema_version : 0;
					self.staged = isNum(r.staged_count) ? r.staged_count : 0;
					if (Array.isArray(r.keys)) { acc = acc.concat(r.keys); }
					var nx = isNum(r.next_id) ? r.next_id : 0;
					/* next_id is the id to resume at; 0 ends the walk. The
					 * progress check stops a malformed reply looping. */
					if (nx && nx > start && ++guard < 40) { return page(nx); }
					return null;
				});
			}

			this.pending = page(0).then(function () {
				self.keys = acc;
				self.byName = Object.create(null);
				self.byId = Object.create(null);
				for (var i = 0; i < acc.length; i++) {
					var k = acc[i];
					if (k && typeof k.name === 'string') { self.byName[k.name] = k; }
					if (k && typeof k.id === 'string') { self.byId[k.id] = k; }
				}
				self.loaded = true;
				self.pending = null;
				paintStage();
				return self;
			}, function (e) {
				self.pending = null;
				throw e;
			});
			return this.pending;
		},

		/* Effective value: the staged one wins because that is what a commit
		 * will install. */
		val: function (name) {
			var k = this.byName[name];
			if (!k) { return null; }
			if (k.staged && k.staged_value !== undefined) { return k.staged_value; }
			return k.value === undefined ? null : k.value;
		},

		put: function (map) {
			var self = this;
			return req('PUT', 'config', { json: map }).then(function (r) {
				if (r && isNum(r.staged_total)) { self.staged = r.staged_total; }
				return r;
			}, function (e) {
				/* 422 = everything rejected; the body still carries results. */
				if (e.status === 422 && e.extra && Array.isArray(e.extra.results)) {
					return e.extra;
				}
				throw e;
			});
		},

		commit: function () {
			var self = this;
			return req('POST', 'config/commit').then(function (r) {
				self.staged = 0;
				return self.load(true).then(function () { return r; });
			});
		},

		revert: function () {
			var self = this;
			return req('POST', 'config/revert').then(function (r) {
				self.staged = 0;
				return self.load(true).then(function () { return r; });
			});
		}
	};

	function paintStage() {
		var sb = $('stagebar');
		var n = Cfg.staged || 0;
		sb.hidden = !(n > 0);
		$('sb-count').textContent = String(n);
		var reboot = 0;
		for (var i = 0; i < Cfg.keys.length; i++) {
			var k = Cfg.keys[i];
			if (k && k.staged && k.reboot_required) { reboot++; }
		}
		$('sb-note').textContent = reboot > 0
			? reboot + ' of them need a reboot to take effect' : '';
		var may = Session.can('operator');
		$('sb-commit').disabled = !may;
		$('sb-revert').disabled = !may;
	}

	/* ===================================================================== */
	/* telemetry store                                                       */
	/* ===================================================================== */

	var GROUPS = ['summary', 'timing', 'gnss', 'power', 'net', 'ptp', 'alarms'];

	var Store = {
		summary: null, timing: null, gnss: null, power: null,
		net: null, ptp: null, alarms: null,
		t_ms: 0, seq: 0, lastTick: null,
		ring: [],
		logs: [], logMeta: null, lastLogSeq: -1,

		ingest: function (m) {
			for (var i = 0; i < GROUPS.length; i++) {
				var g = GROUPS[i];
				if (Object.prototype.hasOwnProperty.call(m, g)) { this[g] = m[g]; }
			}
			if (isNum(m.t_ms)) { this.t_ms = m.t_ms; }
			if (isNum(m.seq)) { this.seq = m.seq; }
			var t = this.timing;
			if (t && typeof t === 'object') {
				/* Keyed on the discipline tick so REST polling and WSS frames
				 * cannot double-push the same 1 Hz sample. */
				var tick = isNum(t.tick) ? t.tick : this.t_ms;
				if (tick !== this.lastTick) {
					this.lastTick = tick;
					this.ring.push({
						tick: tick,
						off: num(t.last_pps_offset_ns),
						ppb: num(t.freq_err_ppb),
						vc: num(t.vc_cmd_mv),
						vs: num(t.vc_sense_mv)
					});
					if (this.ring.length > RING_MAX) {
						this.ring.splice(0, this.ring.length - RING_MAX);
					}
				}
			}
		},

		ingestLogs: function (m) {
			this.logMeta = m;
			var recs = Array.isArray(m.records) ? m.records : [];
			var added = [];
			if (isNum(m.gap) && m.gap > 0) {
				var g = { _gap: m.gap };
				this.logs.push(g);
				added.push(g);
			}
			for (var i = 0; i < recs.length; i++) {
				var r = recs[i];
				if (!r || !isNum(r.seq)) { continue; }
				if (r.seq <= this.lastLogSeq) { continue; }
				this.lastLogSeq = r.seq;
				this.logs.push(r);
				added.push(r);
			}
			if (this.logs.length > LOG_MAX) {
				this.logs.splice(0, this.logs.length - LOG_MAX);
			}
			return added;
		},

		clearLogs: function () { this.logs = []; }
	};

	function activeAlarms() {
		var a = Store.alarms;
		if (a && Array.isArray(a.alarms)) {
			return a.alarms.filter(function (x) { return x && x.active; });
		}
		return null;
	}

	function alarmCount() {
		var l = activeAlarms();
		if (l) { return l.length; }
		var s = Store.summary;
		if (s && isNum(s.alarms)) { return popcount(s.alarms); }
		return null;
	}

	/* ===================================================================== */
	/* live feed: WSS with REST fallback                                     */
	/* ===================================================================== */

	var Live = {
		ws: null, want: false, backoff: 1000, reTimer: null, pollTimer: null,
		rate: 1, extra: Object.create(null), pollSeq: 0, mode: 'init',
		logCursor: null, pollBusy: false,

		groupList: function () {
			var l = GROUPS.slice();
			for (var k in this.extra) { l.push(k); }
			return l;
		},

		start: function () {
			this.want = true;
			this.connect();
			this.startPoll();
		},

		stop: function () {
			this.want = false;
			if (this.reTimer) { clearTimeout(this.reTimer); this.reTimer = null; }
			this.stopPoll();
			if (this.ws) {
				var w = this.ws;
				this.ws = null;
				w.onopen = w.onclose = w.onerror = w.onmessage = null;
				try { w.close(); } catch (e) { /* already gone */ }
			}
			this.setMode('off');
		},

		setMode: function (m) {
			if (this.mode === m) { return; }
			this.mode = m;
			var p = $('p-link');
			var txt = { ws: 'live', poll: 'poll 1Hz', down: 'offline', off: 'idle', init: 'init' }[m] || m;
			p.lastChild.textContent = txt;
			p.className = 'pill link ' + (m === 'ws' ? 'ok' : m === 'poll' ? 'warn'
				: m === 'down' ? 'crit' : 'mute');
		},

		setExtra: function (obj) {
			this.extra = obj || Object.create(null);
			this.sendSub();
		},

		addGroup: function (g) {
			if (this.extra[g]) { return; }
			this.extra[g] = 1;
			this.sendSub();
		},

		connect: function () {
			if (!this.want || this.ws) { return; }
			var self = this;
			var url = (location.protocol === 'https:' ? 'wss://' : 'ws://') +
				location.host + WS_PATH;
			var w;
			try { w = new WebSocket(url); }
			catch (e) { this.schedule(); return; }
			this.ws = w;

			w.onopen = function () {
				if (self.ws !== w) { return; }
				self.backoff = 1000;
				self.setMode('ws');
				self.stopPoll();
				self.sendSub();
			};
			w.onmessage = function (ev) {
				if (self.ws !== w || typeof ev.data !== 'string') { return; }
				var m;
				try { m = JSON.parse(ev.data); }
				catch (e) { return; }
				dispatch(m);
			};
			w.onerror = function () { /* close follows */ };
			w.onclose = function () {
				if (self.ws !== w) { return; }
				self.ws = null;
				if (!self.want) { return; }
				self.startPoll();
				self.schedule();
			};
		},

		schedule: function () {
			var self = this;
			if (!this.want || this.reTimer) { return; }
			var d = this.backoff;
			this.backoff = Math.min(this.backoff * 2, 15000);
			this.reTimer = setTimeout(function () {
				self.reTimer = null;
				self.connect();
			}, d);
		},

		sendSub: function () {
			if (!this.ws || this.ws.readyState !== 1) { return; }
			var msg = { op: 'subscribe', groups: this.groupList(), rate: this.rate };
			if (this.extra.logs && isNum(this.logCursor)) {
				msg.log_cursor = this.logCursor;
			}
			try { this.ws.send(JSON.stringify(msg)); } catch (e) { /* closing */ }
		},

		startPoll: function () {
			var self = this;
			if (this.pollTimer || !this.want) { return; }
			if (this.mode !== 'down') { this.setMode('poll'); }
			this.pollTimer = setInterval(function () { self.pollOnce(); }, 1000);
			this.pollOnce();
		},

		stopPoll: function () {
			if (this.pollTimer) { clearInterval(this.pollTimer); this.pollTimer = null; }
		},

		pollOnce: function () {
			var self = this;
			if (this.pollBusy || !this.want) { return; }
			this.pollBusy = true;
			req('GET', 'status', { quiet: true }).then(function (s) {
				if (!self.want) { return null; }
				if (s && typeof s === 'object') {
					s.type = 'telemetry';
					s.seq = ++self.pollSeq;
					s.t_ms = Date.now();
					dispatch(s);
				}
				if (self.mode !== 'ws') { self.setMode('poll'); }
				if (self.extra.logs) { return self.pollLogs(); }
				return null;
			}).catch(function (e) {
				/* The poll is 'quiet' so a transient outage does not throw the
				 * login overlay up, but a 401 means the session really is gone
				 * and the operator must be told - otherwise the UI just sits
				 * there looking offline forever. */
				if (e && e.code === 'unauthenticated') {
					Session.requireLogin('Session ended. Sign in again.');
					return;
				}
				if (self.mode !== 'ws') { self.setMode('down'); }
			}).then(function () { self.pollBusy = false; });
		},

		pollLogs: function () {
			var self = this;
			var q = 'logs?max=32';
			if (isNum(this.logCursor)) { q += '&cursor=' + this.logCursor; }
			return req('GET', q, { quiet: true }).then(function (r) {
				if (r && r.type === 'logs') { dispatch(r); }
			}, function () { /* the status poll already reported the outage */ });
		}
	};

	function dispatch(m) {
		if (!m || typeof m !== 'object') { return; }
		if (m.type === 'logs') {
			var added = Store.ingestLogs(m);
			if (isNum(m.next_cursor)) { Live.logCursor = m.next_cursor; }
			if (Page.cur && Page.cur.onLogs) {
				try { Page.cur.onLogs(added); } catch (e) { /* page-local */ }
			}
			return;
		}
		/* Anything else with group keys is treated as a telemetry envelope so
		 * REST /status and WSS frames share one path. */
		Store.ingest(m);
		paintHeader();
		if (Page.cur && Page.cur.update) {
			try { Page.cur.update(); } catch (e) { /* page-local */ }
		}
	}

	/* ===================================================================== */
	/* header                                                                */
	/* ===================================================================== */

	function setPill(id, text, cls) {
		var p = $(id);
		p.lastChild.textContent = (text === null || text === undefined) ? '—' : String(text);
		var c = 'pill' + (cls ? ' ' + cls : '');
		if (p.className !== c) { p.className = c; }
	}

	function lockClass(s) {
		if (s === 'locked') { return 'ok'; }
		if (s === 'holdover') { return 'warn'; }
		if (s === 'parked' || s === 'unknown' || s === 'invalid') { return 'crit'; }
		return 'info';
	}

	function paintHeader() {
		var s = Store.summary, t = Store.timing;
		var lock = (s && s.lock_state) || (t && t.lock_state) || null;
		setPill('p-lock', lock, lockClass(lock));

		var st = (s && isNum(s.stratum)) ? s.stratum : (t && isNum(t.stratum) ? t.stratum : null);
		setPill('p-stratum', st, st === 1 ? 'ok' : st === null ? 'mute' : 'warn');

		var ref = (s && s.reference) || (t && t.reference) || null;
		setPill('p-ref', ref, ref === 'none' ? 'crit' : ref ? 'info' : 'mute');

		var fix = s ? s.gnss_fix : (Store.gnss ? Store.gnss.fix : null);
		var used = s ? s.gnss_sv_used : (Store.gnss ? Store.gnss.sv_used : null);
		var vis = s ? s.gnss_sv_visible : (Store.gnss ? Store.gnss.sv_visible : null);
		setPill('p-gnss', fix ? fix + ' ' + (isNum(used) ? used : '?') + '/' +
			(isNum(vis) ? vis : '?') : null,
			(fix === 'time' || fix === '3d') ? 'ok' : fix === 'none' ? 'crit' : 'warn');

		var off = s ? s.last_pps_offset_ns : (t ? t.last_pps_offset_ns : null);
		setPill('p-offset', isNum(off) ? fns(off) : null,
			!isNum(off) ? 'mute' : Math.abs(off) < 100 ? 'ok'
				: Math.abs(off) < 1000 ? 'warn' : 'crit');

		var n = alarmCount();
		setPill('p-alarms', n === null ? null : n, n === null ? 'mute' : n === 0 ? 'ok' : 'crit');

		if (s) {
			$('nf-model').textContent = s.model || '—';
			$('nf-fw').textContent = s.firmware || '—';
			$('nf-board').textContent = s.board_id || '—';
			$('nf-uptime').textContent = isNum(s.uptime_s) ? fdur(s.uptime_s) : '—';
		}
	}

	/* ===================================================================== */
	/* canvas plumbing                                                       */
	/* ===================================================================== */

	var Redraws = [];
	var rafPend = false;

	function regDraw(fn) { Redraws.push(fn); return fn; }
	function clearDraws() { Redraws.length = 0; }

	function redrawAll() {
		if (rafPend) { return; }
		rafPend = true;
		requestAnimationFrame(function () {
			rafPend = false;
			for (var i = 0; i < Redraws.length; i++) {
				try { Redraws[i](); } catch (e) { /* one bad widget must not stop the rest */ }
			}
		});
	}
	window.addEventListener('resize', redrawAll);

	function fitCanvas(cv) {
		var w = cv.clientWidth, h = cv.clientHeight;
		if (!w || !h) { return null; }
		var dpr = window.devicePixelRatio || 1;
		var W = Math.round(w * dpr), H = Math.round(h * dpr);
		if (cv.width !== W || cv.height !== H) { cv.width = W; cv.height = H; }
		var g = cv.getContext('2d');
		if (!g) { return null; }
		g.setTransform(dpr, 0, 0, dpr, 0, 0);
		g.clearRect(0, 0, w, h);
		return { g: g, w: w, h: h };
	}

	function centerNote(g, w, h, lines) {
		g.fillStyle = C.fg3;
		g.font = '11.5px ui-monospace, monospace';
		g.textAlign = 'center';
		g.textBaseline = 'middle';
		var y = h / 2 - (lines.length - 1) * 8;
		for (var i = 0; i < lines.length; i++) {
			g.fillText(lines[i], w / 2, y + i * 16);
		}
		g.textAlign = 'left';
		g.textBaseline = 'alphabetic';
	}

	var Tip = {
		show: function (x, y, text) {
			var t = $('tooltip');
			t.textContent = text;
			t.hidden = false;
			var r = t.getBoundingClientRect();
			var px = x + 12, py = y + 12;
			if (px + r.width > window.innerWidth - 6) { px = x - r.width - 12; }
			if (py + r.height > window.innerHeight - 6) { py = y - r.height - 12; }
			t.style.left = Math.max(4, px) + 'px';
			t.style.top = Math.max(4, py) + 'px';
		},
		hide: function () { $('tooltip').hidden = true; }
	};

	/* ===================================================================== */
	/* skyplot                                                               */
	/* ===================================================================== */

	function Skyplot(big) {
		var cv = el('canvas', { cls: 'sky' + (big ? ' lg' : '') });
		var wrap = el('div', { cls: 'cvw' }, cv);
		var sats = [], mask = null, detail = true;
		var geom = null, hit = [];

		/* Polar: north up, azimuth clockwise, 90 deg elevation at the centre,
		 * 0 deg at the rim. Elevation outside 0..90 clips to the rim/centre so
		 * a receiver reporting a negative elevation still plots. */
		function project(cx, cy, R, elev, azim) {
			var e = isNum(elev) ? elev : 0;
			if (e > 90) { e = 90; }
			if (e < 0) { e = 0; }
			var r = (90 - e) / 90 * R;
			var a = (isNum(azim) ? azim : 0) * Math.PI / 180;
			return [cx + r * Math.sin(a), cy - r * Math.cos(a)];
		}

		function draw() {
			var c = fitCanvas(cv);
			if (!c) { return; }
			var g = c.g, w = c.w, h = c.h;
			var cx = w / 2, cy = h / 2;
			var R = Math.max(30, Math.min(w, h) / 2 - 22);
			geom = { cx: cx, cy: cy, R: R };
			hit = [];

			/* elevation-mask band: rim inward to the mask elevation */
			if (isNum(mask) && mask > 0) {
				var rm = (90 - Math.min(90, mask)) / 90 * R;
				g.beginPath();
				g.arc(cx, cy, R, 0, Math.PI * 2);
				g.arc(cx, cy, rm, 0, Math.PI * 2, true);
				g.fillStyle = 'rgba(245,165,36,0.11)';
				g.fill();
				g.beginPath();
				g.arc(cx, cy, rm, 0, Math.PI * 2);
				g.setLineDash([3, 3]);
				g.strokeStyle = 'rgba(245,165,36,0.6)';
				g.lineWidth = 1;
				g.stroke();
				g.setLineDash([]);
			}

			/* azimuth spokes every 30 deg */
			g.strokeStyle = C.grid;
			g.lineWidth = 1;
			for (var a = 0; a < 360; a += 30) {
				var p = project(cx, cy, R, 0, a);
				g.beginPath();
				g.moveTo(cx, cy);
				g.lineTo(p[0], p[1]);
				g.stroke();
			}

			/* elevation rings 0 / 30 / 60 + zenith cross */
			var rings = [0, 30, 60];
			for (var i = 0; i < rings.length; i++) {
				var rr = (90 - rings[i]) / 90 * R;
				g.beginPath();
				g.arc(cx, cy, rr, 0, Math.PI * 2);
				g.strokeStyle = rings[i] === 0 ? C.line2 : C.line;
				g.lineWidth = rings[i] === 0 ? 1.4 : 1;
				g.stroke();
			}
			g.strokeStyle = C.line2;
			g.beginPath();
			g.moveTo(cx - 3, cy); g.lineTo(cx + 3, cy);
			g.moveTo(cx, cy - 3); g.lineTo(cx, cy + 3);
			g.stroke();

			/* elevation ring labels along the vertical */
			g.fillStyle = C.fg3;
			g.font = '9.5px ui-monospace, monospace';
			g.textAlign = 'left';
			for (i = 0; i < rings.length; i++) {
				var ry = cy - (90 - rings[i]) / 90 * R;
				g.fillText(rings[i] + '°', cx + 3, ry - 3);
			}

			/* cardinals */
			g.font = '600 11px ui-monospace, monospace';
			g.fillStyle = C.fg2;
			g.textAlign = 'center';
			g.textBaseline = 'middle';
			var card = [['N', 0], ['E', 90], ['S', 180], ['W', 270]];
			for (i = 0; i < card.length; i++) {
				var q = project(cx, cy, R + 12, 0, card[i][1]);
				g.fillText(card[i][0], q[0], q[1]);
			}
			g.textBaseline = 'alphabetic';
			g.textAlign = 'left';

			if (!detail) {
				centerNote(g, w, h, ['no receiver detail',
					'GNSS detail is not available in this build']);
				return;
			}
			if (!sats.length) {
				centerNote(g, w, h, ['no satellites reported']);
				return;
			}

			for (i = 0; i < sats.length; i++) {
				var s = sats[i];
				if (!s || typeof s !== 'object') { continue; }
				var con = typeof s.constellation === 'string' ? s.constellation : 'other';
				var col = CONST_COLOR[con] || CONST_COLOR.other;
				var pt = project(cx, cy, R, s.elev, s.azim);
				var cno = isNum(s.cno) ? s.cno : 0;
				var rad = 4.5 + Math.min(50, Math.max(0, cno)) / 50 * 3;

				g.beginPath();
				g.arc(pt[0], pt[1], rad, 0, Math.PI * 2);
				if (s.used) {
					g.fillStyle = col;
					g.fill();
					g.strokeStyle = 'rgba(0,0,0,0.55)';
					g.lineWidth = 1;
					g.stroke();
				} else {
					g.fillStyle = 'rgba(8,11,16,0.85)';
					g.fill();
					g.strokeStyle = col;
					g.lineWidth = 1.4;
					g.stroke();
				}

				g.fillStyle = s.used ? C.fg : C.fg3;
				g.font = '9.5px ui-monospace, monospace';
				g.fillText(String(isNum(s.sv) ? s.sv : '?'), pt[0] + rad + 2, pt[1] + 3.5);

				hit.push({ x: pt[0], y: pt[1], r: rad + 4, s: s, con: con });
			}
		}

		function onMove(ev) {
			if (!geom || !hit.length) { Tip.hide(); return; }
			var b = cv.getBoundingClientRect();
			var mx = ev.clientX - b.left, my = ev.clientY - b.top;
			var best = null, bd = 1e9;
			for (var i = 0; i < hit.length; i++) {
				var d = Math.pow(hit[i].x - mx, 2) + Math.pow(hit[i].y - my, 2);
				if (d < Math.pow(hit[i].r + 3, 2) && d < bd) { bd = d; best = hit[i]; }
			}
			if (!best) { Tip.hide(); return; }
			var s = best.s;
			Tip.show(ev.clientX, ev.clientY,
				best.con.toUpperCase() + ' SV ' + (isNum(s.sv) ? s.sv : '?') + '\n' +
				'C/N0  ' + (isNum(s.cno) ? s.cno + ' dBHz' : '—') + '\n' +
				'elev  ' + (isNum(s.elev) ? s.elev + '°' : '—') + '\n' +
				'azim  ' + (isNum(s.azim) ? s.azim + '°' : '—') + '\n' +
				(s.used ? 'used in solution' : 'tracked, not used'));
		}

		cv.addEventListener('mousemove', onMove);
		cv.addEventListener('mouseleave', Tip.hide);

		return {
			el: wrap,
			draw: regDraw(draw),
			set: function (satArr, maskDeg, detailAvail) {
				sats = Array.isArray(satArr) ? satArr : [];
				mask = isNum(maskDeg) ? maskDeg : null;
				detail = detailAvail !== false;
				draw();
			},
			destroy: function () {
				cv.removeEventListener('mousemove', onMove);
				cv.removeEventListener('mouseleave', Tip.hide);
				Tip.hide();
			}
		};
	}

	function skyLegend() {
		var lg = el('div', { cls: 'legend' });
		var order = ['gps', 'galileo', 'glonass', 'beidou', 'qzss', 'sbas'];
		for (var i = 0; i < order.length; i++) {
			lg.appendChild(el('span', null, [
				el('i', { style: 'background:' + CONST_COLOR[order[i]] }),
				order[i].toUpperCase()
			]));
		}
		lg.appendChild(el('span', { style: 'color:' + C.fg3 }, [
			el('i', { cls: 'hollow' }), 'hollow = tracked, not used'
		]));
		lg.appendChild(el('span', null, [
			el('i', { style: 'background:rgba(245,165,36,0.5)' }), 'elevation mask'
		]));
		return lg;
	}

	/* ===================================================================== */
	/* trend plot                                                            */
	/* ===================================================================== */

	/* Trend({series:[{key,color,label,dash}], dp, unit, zero, tall}) over
	 * Store.ring-shaped sample arrays. Nulls break the line. */
	function Trend(opts) {
		var cv = el('canvas', { cls: 'plot' + (opts.tall ? ' tall' : '') });
		var wrap = el('div', { cls: 'cvw' }, cv);
		var data = [];
		var series = opts.series;
		var dp = isNum(opts.dp) ? opts.dp : 0;

		function draw() {
			var c = fitCanvas(cv);
			if (!c) { return; }
			var g = c.g, w = c.w, h = c.h;
			var L = 52, Rp = 8, T = 8, B = 16;
			var pw = w - L - Rp, ph = h - T - B;
			if (pw <= 4 || ph <= 4) { return; }

			var lo = Infinity, hi = -Infinity, i, j, n = 0;
			for (i = 0; i < data.length; i++) {
				for (j = 0; j < series.length; j++) {
					var v = data[i][series[j].key];
					if (isNum(v)) { if (v < lo) { lo = v; } if (v > hi) { hi = v; } n++; }
				}
			}
			if (!n) {
				g.strokeStyle = C.line;
				g.strokeRect(L, T, pw, ph);
				centerNote(g, w, h, ['waiting for samples']);
				return;
			}
			if (opts.zero) { if (lo > 0) { lo = 0; } if (hi < 0) { hi = 0; } }
			if (hi - lo < 1e-12) { hi = lo + 1; lo -= 1; }
			var pad = (hi - lo) * 0.08;
			lo -= pad; hi += pad;

			function py(v) { return T + ph - (v - lo) / (hi - lo) * ph; }
			/* Fixed sample axis: the trace fills left-to-right as the ring
			 * fills, so the x position of a sample never shifts under it. */
			function px(i) { return L + (i / (RING_MAX - 1)) * pw; }

			/* grid + y labels */
			g.font = '9.5px ui-monospace, monospace';
			g.textAlign = 'right';
			g.textBaseline = 'middle';
			for (i = 0; i <= 4; i++) {
				var vv = lo + (hi - lo) * (i / 4);
				var yy = py(vv);
				g.strokeStyle = C.grid;
				g.lineWidth = 1;
				g.beginPath();
				g.moveTo(L, yy);
				g.lineTo(L + pw, yy);
				g.stroke();
				g.fillStyle = C.fg3;
				g.fillText(dp > 0 ? vv.toFixed(dp) : fsig(vv), L - 5, yy);
			}
			if (lo < 0 && hi > 0) {
				g.strokeStyle = C.line2;
				g.beginPath();
				g.moveTo(L, py(0));
				g.lineTo(L + pw, py(0));
				g.stroke();
			}
			g.strokeStyle = C.line;
			g.strokeRect(L, T, pw, ph);

			/* series */
			for (j = 0; j < series.length; j++) {
				var s = series[j];
				g.strokeStyle = s.color;
				g.lineWidth = 1.4;
				if (s.dash) { g.setLineDash([4, 3]); } else { g.setLineDash([]); }
				g.beginPath();
				var pen = false, lastPt = null;
				for (i = 0; i < data.length; i++) {
					var val = data[i][s.key];
					if (!isNum(val)) { pen = false; continue; }
					var X = px(i), Y = py(val);
					if (!pen) { g.moveTo(X, Y); pen = true; }
					else { g.lineTo(X, Y); }
					lastPt = [X, Y, val];
				}
				g.stroke();
				g.setLineDash([]);
				if (lastPt) {
					g.fillStyle = s.color;
					g.beginPath();
					g.arc(lastPt[0], lastPt[1], 2.4, 0, Math.PI * 2);
					g.fill();
				}
			}

			/* x hint */
			g.textAlign = 'left';
			g.textBaseline = 'alphabetic';
			g.fillStyle = C.fg3;
			g.fillText('← ' + RING_MAX + ' samples', L + 2, h - 4);
			g.textAlign = 'right';
			g.fillText('now', L + pw, h - 4);
			g.textAlign = 'left';
		}

		return {
			el: wrap,
			draw: regDraw(draw),
			set: function (arr) { data = arr || []; draw(); }
		};
	}

	function trendLegend(series, unit) {
		var lg = el('div', { cls: 'legend' });
		for (var i = 0; i < series.length; i++) {
			lg.appendChild(el('span', null, [
				el('i', { style: 'background:' + series[i].color }), series[i].label
			]));
		}
		if (unit) { lg.appendChild(el('span', { style: 'color:' + C.fg3, text: unit })); }
		return lg;
	}

	/* ===================================================================== */
	/* ADEV plot (log-log)                                                   */
	/* ===================================================================== */

	function AdevPlot() {
		var cv = el('canvas', { cls: 'plot tall' });
		var wrap = el('div', { cls: 'cvw' }, cv);
		var pts = [];

		function draw() {
			var c = fitCanvas(cv);
			if (!c) { return; }
			var g = c.g, w = c.w, h = c.h;
			var L = 58, Rp = 10, T = 10, B = 22;
			var pw = w - L - Rp, ph = h - T - B;
			if (pw <= 4 || ph <= 4) { return; }

			var good = pts.filter(function (p) { return isNum(p.v) && p.v > 0; });
			g.strokeStyle = C.line;
			g.lineWidth = 1;
			g.strokeRect(L, T, pw, ph);
			if (!good.length) {
				centerNote(g, w, h, ['no ADEV estimate yet']);
				return;
			}

			var ylo = Infinity, yhi = -Infinity, i;
			for (i = 0; i < good.length; i++) {
				var l = Math.log10(good[i].v);
				if (l < ylo) { ylo = l; }
				if (l > yhi) { yhi = l; }
			}
			ylo = Math.floor(ylo) - 1;
			yhi = Math.ceil(yhi) + 1;
			if (yhi - ylo < 2) { yhi = ylo + 2; }

			/* x: log10(tau), 0 .. 2 (1 s .. 100 s) */
			function px(tau) { return L + (Math.log10(tau) / 2) * pw; }
			function py(v) { return T + ph - (Math.log10(v) - ylo) / (yhi - ylo) * ph; }

			g.font = '9.5px ui-monospace, monospace';
			g.textAlign = 'right';
			g.textBaseline = 'middle';
			for (var d = ylo; d <= yhi; d++) {
				var yy = py(Math.pow(10, d));
				g.strokeStyle = C.grid;
				g.beginPath();
				g.moveTo(L, yy);
				g.lineTo(L + pw, yy);
				g.stroke();
				g.fillStyle = C.fg3;
				g.fillText('1e' + d, L - 5, yy);
			}
			/* decade + intermediate x gridlines */
			g.textAlign = 'center';
			g.textBaseline = 'alphabetic';
			var xt = [1, 2, 5, 10, 20, 50, 100];
			for (i = 0; i < xt.length; i++) {
				var xx = px(xt[i]);
				var major = (xt[i] === 1 || xt[i] === 10 || xt[i] === 100);
				g.strokeStyle = major ? C.line : C.grid;
				g.beginPath();
				g.moveTo(xx, T);
				g.lineTo(xx, T + ph);
				g.stroke();
				if (major) {
					g.fillStyle = C.fg3;
					g.fillText(xt[i] + ' s', xx, h - 7);
				}
			}
			g.strokeStyle = C.line;
			g.strokeRect(L, T, pw, ph);

			/* curve */
			g.strokeStyle = C.accent;
			g.lineWidth = 1.6;
			g.beginPath();
			for (i = 0; i < good.length; i++) {
				var X = px(good[i].tau), Y = py(good[i].v);
				if (i === 0) { g.moveTo(X, Y); } else { g.lineTo(X, Y); }
			}
			g.stroke();

			g.font = '9.5px ui-monospace, monospace';
			for (i = 0; i < good.length; i++) {
				var qx = px(good[i].tau), qy = py(good[i].v);
				g.fillStyle = C.accent;
				g.beginPath();
				g.arc(qx, qy, 3, 0, Math.PI * 2);
				g.fill();
				g.fillStyle = C.fg2;
				g.textAlign = good[i].tau === 100 ? 'right' : 'left';
				g.fillText(good[i].v.toExponential(1),
					qx + (good[i].tau === 100 ? -6 : 6), qy - 6);
			}
			g.textAlign = 'left';
			g.fillStyle = C.fg3;
			g.fillText('σy(τ)', L - 52, T + 8);
		}

		return {
			el: wrap,
			draw: regDraw(draw),
			set: function (adev) {
				var a = adev || {};
				pts = [
					{ tau: 1, v: num(a.tau_1s) },
					{ tau: 10, v: num(a.tau_10s) },
					{ tau: 100, v: num(a.tau_100s) }
				];
				draw();
			}
		};
	}

	/* ===================================================================== */
	/* config editor                                                         */
	/* ===================================================================== */

	var T_BOOL = 1, T_U8 = 2, T_U16 = 3, T_U32 = 4, T_U64 = 5,
		T_I32 = 6, T_F32 = 7, T_STR = 8, T_BLOB = 9;

	var TYPE_NAME = {
		1: 'bool', 2: 'u8', 3: 'u16', 4: 'u32', 5: 'u64',
		6: 'i32', 7: 'f32', 8: 'str', 9: 'blob'
	};

	function isIntType(t) {
		return t === T_U8 || t === T_U16 || t === T_U32 || t === T_U64 || t === T_I32;
	}

	/* Writing a raw blob into a credential slot would corrupt it; the Security
	 * page owns that key through /security/password. */
	function readOnlyKey(k) {
		return k.name === 'sec.admin.pw' || (k.secret && k.type === T_BLOB);
	}

	/* CfgTable({keys | prefixes | filter, title, note}) */
	function CfgTable(opts) {
		var may = Session.can('operator');
		var t = table([
			'Key', 'Value', { t: 'Staged' }, 'Type', 'Range', 'Flags'
		]);
		var pending = Object.create(null);
		var inputs = Object.create(null);
		var applyBtn = el('button', {
			cls: 'btn primary sm', type: 'button', disabled: true,
			text: 'Stage changes'
		});
		var resetBtn = el('button', {
			cls: 'btn sm', type: 'button', disabled: true, text: 'Undo edits'
		});
		var count = el('span', { cls: 'mono', text: '' });

		function pick() {
			if (opts.keys) { return opts.keys; }
			var out = [];
			for (var i = 0; i < Cfg.keys.length; i++) {
				var k = Cfg.keys[i];
				if (!k || typeof k.name !== 'string') { continue; }
				if (opts.filter && !opts.filter(k)) { continue; }
				if (opts.prefixes) {
					var m = false;
					for (var j = 0; j < opts.prefixes.length; j++) {
						if (k.name.indexOf(opts.prefixes[j]) === 0) { m = true; break; }
					}
					if (!m) { continue; }
				}
				out.push(k);
			}
			return out;
		}

		function effective(k) {
			if (k.value_withheld) { return null; }
			if (k.staged && k.staged_value !== undefined) { return k.staged_value; }
			return k.value === undefined ? null : k.value;
		}

		function repaintBtns() {
			var n = Object.keys(pending).length;
			applyBtn.disabled = !may || n === 0;
			resetBtn.disabled = n === 0;
			count.textContent = n ? n + ' edited' : '';
		}

		function mark(inp, dirty) {
			inp.classList.toggle('dirty', !!dirty);
			inp.classList.remove('bad');
		}

		/* Convert an editor's raw text into the JSON shape PUT /config wants. */
		function encode(k, raw) {
			if (k.type === T_BOOL) { return !!raw; }
			if (k.type === T_STR) { return String(raw); }
			if (k.type === T_BLOB) {
				var s = String(raw).trim().toLowerCase();
				if (!/^[0-9a-f]*$/.test(s) || (s.length % 2) !== 0) {
					throw new Error('expects an even-length lowercase hex string');
				}
				if (isNum(k.maxlen) && s.length / 2 > k.maxlen) {
					throw new Error('at most ' + k.maxlen + ' bytes');
				}
				return s;
			}
			var v = Number(String(raw).trim());
			if (!isFinite(v)) { throw new Error('not a number'); }
			if (isNum(k.min) && v < k.min) { throw new Error('min is ' + k.min); }
			if (isNum(k.max) && v > k.max) { throw new Error('max is ' + k.max); }
			if (k.type === T_F32) {
				/* The device deliberately has no decimal float parser: whole
				 * numbers go as plain integers, everything else as integer
				 * micro-units. A bare decimal literal would be rejected. */
				if (Math.abs(v - Math.round(v)) < 1e-9) { return Math.round(v); }
				return { micro: Math.round(v * 1e6) };
			}
			if (Math.abs(v - Math.round(v)) > 0) { throw new Error('must be an integer'); }
			return Math.round(v);
		}

		function editor(k) {
			var cur = effective(k);
			var ro = !may || readOnlyKey(k);

			if (k.value_withheld) {
				var wrap = el('span', null, [
					el('span', { cls: 'tag sec', text: k.value_set ? 'set' : 'not set' })
				]);
				if (ro) {
					wrap.appendChild(document.createTextNode(' '));
					wrap.appendChild(el('span', {
						cls: 'hint',
						text: k.name === 'sec.admin.pw'
							? 'change it on the Security page'
							: 'write-only'
					}));
					return wrap;
				}
				var si = el('input', {
					type: 'text', size: 18, placeholder: 'new value…',
					maxlength: isNum(k.maxlen) ? (k.type === T_BLOB ? k.maxlen * 2 : k.maxlen) : null
				});
				si.addEventListener('input', function () {
					if (si.value === '') { delete pending[k.name]; mark(si, false); }
					else { pending[k.name] = si.value; mark(si, true); }
					repaintBtns();
				});
				inputs[k.name] = si;
				wrap.appendChild(document.createTextNode(' '));
				wrap.appendChild(si);
				return wrap;
			}

			if (k.type === T_BOOL) {
				var cb = el('input', { type: 'checkbox', checked: cur === true, disabled: ro });
				cb.addEventListener('change', function () {
					if (cb.checked === (cur === true)) { delete pending[k.name]; mark(cb, false); }
					else { pending[k.name] = cb.checked; mark(cb, true); }
					repaintBtns();
				});
				inputs[k.name] = cb;
				return cb;
			}

			var isTxt = (k.type === T_STR || k.type === T_BLOB);
			var inp = el('input', {
				type: isTxt ? 'text' : 'number',
				value: cur === null || cur === undefined ? '' : String(cur),
				size: isTxt ? 20 : 11,
				disabled: ro,
				step: k.type === T_F32 ? 'any' : '1',
				min: (!isTxt && isNum(k.min)) ? k.min : null,
				max: (!isTxt && isNum(k.max)) ? k.max : null,
				maxlength: (isTxt && isNum(k.maxlen))
					? (k.type === T_BLOB ? k.maxlen * 2 : k.maxlen) : null
			});
			inp.addEventListener('input', function () {
				var same = String(inp.value) === (cur === null ? '' : String(cur));
				if (same) { delete pending[k.name]; mark(inp, false); }
				else { pending[k.name] = inp.value; mark(inp, true); }
				repaintBtns();
			});
			inputs[k.name] = inp;
			return inp;
		}

		function build() {
			clear(t.body);
			inputs = Object.create(null);
			pending = Object.create(null);
			repaintBtns();
			var list = pick();
			if (!list.length) {
				var tr = el('tr');
				tr.appendChild(el('td', { colspan: 6, cls: 'tx dim', text: 'no keys in this group' }));
				t.body.appendChild(tr);
				return;
			}
			for (var i = 0; i < list.length; i++) {
				var k = list[i];
				var row = el('tr');
				row.appendChild(el('td', { cls: 'tx' }, [
					el('span', { text: k.name }),
					el('span', { cls: 'hint', text: ' ' + (k.id || '') })
				]));
				row.appendChild(el('td', null, editor(k)));

				var stagedTd = el('td');
				if (k.staged) {
					stagedTd.appendChild(el('span', {
						cls: 'tag rb',
						text: k.value_withheld ? 'staged'
							: '→ ' + String(k.staged_value)
					}));
				} else { stagedTd.textContent = '—'; }
				row.appendChild(stagedTd);

				row.appendChild(el('td', { cls: 'dim', text: TYPE_NAME[k.type] || String(k.type) }));

				var rng = '—';
				if (isNum(k.maxlen)) { rng = 'max ' + k.maxlen + ' B'; }
				else if (isNum(k.min) || isNum(k.max)) {
					rng = (isNum(k.min) ? k.min : '?') + ' … ' + (isNum(k.max) ? k.max : '?');
				}
				row.appendChild(el('td', { cls: 'dim', text: rng }));

				var fl = el('td', { cls: 'tx' });
				if (k.reboot_required) { fl.appendChild(el('span', { cls: 'tag rb', text: 'reboot' })); }
				if (k.secret) { fl.appendChild(el('span', { cls: 'tag sec', text: 'secret' })); }
				if (k.calibration) { fl.appendChild(el('span', { cls: 'tag cal', text: 'cal' })); }
				if (!fl.firstChild) { fl.textContent = '—'; }
				row.appendChild(fl);

				t.body.appendChild(row);
			}
		}

		function apply() {
			var body = Object.create(null);
			var names = Object.keys(pending);
			var bad = 0;
			for (var i = 0; i < names.length; i++) {
				var k = Cfg.byName[names[i]];
				if (!k) { continue; }
				try { body[names[i]] = encode(k, pending[names[i]]); }
				catch (e) {
					bad++;
					if (inputs[names[i]]) { inputs[names[i]].classList.add('bad'); }
					toast(names[i] + ': ' + e.message, 'err', 'invalid value');
				}
			}
			if (bad || !Object.keys(body).length) { return; }
			applyBtn.disabled = true;
			Cfg.put(body).then(function (r) {
				var okN = 0, results = (r && Array.isArray(r.results)) ? r.results : [];
				for (var j = 0; j < results.length; j++) {
					var rr = results[j];
					if (!rr) { continue; }
					if (rr.status === 'staged') { okN++; continue; }
					var nm = rr.name || rr.key;
					if (inputs[nm]) { inputs[nm].classList.add('bad'); }
					toast(nm + ': ' + rr.status, 'err', 'rejected');
				}
				if (okN) {
					toast(okN + ' key' + (okN === 1 ? '' : 's') + ' staged. Commit to apply.',
						'ok', 'staged');
				}
				return Cfg.load(true).then(function () { build(); paintStage(); });
			}).catch(function (e) {
				errToast(e, 'config write');
				repaintBtns();
			});
		}

		applyBtn.addEventListener('click', apply);
		resetBtn.addEventListener('click', function () { build(); });

		build();

		var head = [count, resetBtn, applyBtn];
		var body = [t.el];
		if (opts.note) { body.push(el('div', { cls: 'pnote' }, opts.note)); }
		if (!may) {
			body.push(el('div', {
				cls: 'pnote',
				text: 'Read-only: staging configuration requires the operator role.'
			}));
		}
		return {
			el: panel(opts.title || 'Configuration', el('div', { cls: 'pbody tight' }, body), head),
			refresh: build
		};
	}

	/* ===================================================================== */
	/* shared blocks                                                         */
	/* ===================================================================== */

	function alarmsPanel(extraHead) {
		var t = table(['Alarm', 'ID', 'Active', 'Latched']);
		var rows = RowSet(t.body, 4);
		var note = el('div', { cls: 'pnote' });
		var p = panel('Alarms', el('div', { cls: 'pbody tight' }, [t.el, note]), extraHead);
		return {
			el: p,
			update: function () {
				var a = Store.alarms;
				if (!a) {
					rows.clear();
					note.textContent = 'No alarm provider in this build.';
					return;
				}
				var list = Array.isArray(a.alarms) ? a.alarms : [];
				var shown = list.filter(function (x) { return x && (x.active || x.latched); });
				rows.sync(shown, function (x) { return x.id; }, function (td, x) {
					cell(td[0], x.name || ('alarm ' + x.id), 'tx');
					cell(td[1], x.id, 'dim');
					cell(td[2], x.active ? 'ACTIVE' : 'clear', x.active ? 'crit' : 'dim');
					cell(td[3], x.latched ? 'LATCHED' : 'clear', x.latched ? 'warn' : 'dim');
				});
				note.textContent = shown.length
					? 'active mask ' + maskHex(a.active_mask) +
					'   latched mask ' + maskHex(a.latched_mask)
					: 'No active or latched alarms. active mask ' + maskHex(a.active_mask) +
					'   latched mask ' + maskHex(a.latched_mask);
			}
		};
	}

	function metricsLink() {
		return el('a', {
			cls: 'lnk', href: API + 'metrics', target: '_blank', rel: 'noreferrer',
			text: 'Prometheus metrics'
		});
	}

	function railRows(t, withBars) {
		var rows = RowSet(t.body, withBars ? 9 : 7);
		return function (power) {
			var rails = (power && Array.isArray(power.rails)) ? power.rails : [];
			rows.sync(rails, function (r, i) { return r.designator || r.name || i; },
				function (td, r) {
					cell(td[0], r.name, 'tx');
					cell(td[1], r.designator, 'dim');
					cell(td[2], isNum(r.addr) ? '0x' + r.addr.toString(16) : null, 'dim');
					cell(td[3], isNum(r.bus_v) ? r.bus_v.toFixed(3) + ' V' : null,
						r.valid ? '' : 'dim');
					cell(td[4], isNum(r.current_a) ? (r.current_a * 1000).toFixed(2) + ' mA' : null,
						r.valid ? '' : 'dim');
					cell(td[5], isNum(r.power_w) ? r.power_w.toFixed(3) + ' W' : null,
						r.valid ? '' : 'dim');
					if (withBars) {
						var b = td[6]._bar;
						if (!b) {
							clear(td[6]);
							b = bar(0);
							td[6].appendChild(b);
							td[6]._bar = b;
						}
						var maxA = isNum(r.design_max_ma) ? r.design_max_ma / 1000 : 0;
						var f = (maxA > 0 && isNum(r.current_a)) ? r.current_a / maxA : 0;
						setBar(b, f, f > 0.95 ? 'crit' : f > 0.8 ? 'warn' : 'ok');
						cell(td[7], isNum(r.design_max_ma) ? r.design_max_ma + ' mA' : null, 'dim');
						cell(td[8], r.diag_alrt ? 'ALERT ' + maskHex(r.diag_alrt)
							: (r.valid ? 'ok' : 'invalid'),
							r.diag_alrt ? 'crit' : r.valid ? 'ok' : 'warn');
					} else {
						cell(td[6], r.diag_alrt ? 'ALERT' : (r.valid ? 'ok' : 'invalid'),
							r.diag_alrt ? 'crit' : r.valid ? 'ok' : 'warn');
					}
				});
		};
	}

	/* ===================================================================== */
	/* pages                                                                 */
	/* ===================================================================== */

	var Page = { cur: null };

	/* ---------------------------------------------------------- dashboard */

	function pageDashboard() {
		var root = el('div', { cls: 'grid' });

		var tiles = el('div', { cls: 'tiles' });
		root.appendChild(el('div', { cls: 'c12' }, tiles));

		var dashSeries = [{ key: 'off', color: C.accent, label: 'PPS offset' }];
		var trend = Trend({ series: dashSeries, zero: true });
		root.appendChild(el('div', { cls: 'c8' },
			panel('Phase offset · last ' + RING_MAX + ' samples',
				el('div', { cls: 'pbody tight' },
					[trend.el, trendLegend(dashSeries, 'ns')]))));

		var qual = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c4' }, panel('Quality & holdover', qual)));

		var sky = Skyplot();
		root.appendChild(el('div', { cls: 'c5' },
			panel('Sky view', el('div', { cls: 'pbody tight' }, [sky.el, skyLegend()]))));

		var svc = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c4' }, panel('Services', svc)));

		var ptpB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c3' }, panel('PTP', ptpB)));

		var rt = table(['Rail', 'Ref', 'Addr', 'Bus', 'Current', 'Power', 'State']);
		var fillRails = railRows(rt, false);
		root.appendChild(el('div', { cls: 'c6' },
			panel('Power rails', el('div', { cls: 'pbody tight' }, rt.el),
				el('a', { cls: 'lnk', href: '#/power', text: 'detail →' }))));

		var envB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c6' }, panel('Environment & PoE', envB)));

		var al = alarmsPanel(metricsLink());
		root.appendChild(el('div', { cls: 'c12' }, al.el));

		function update() {
			var s = Store.summary, t = Store.timing, g = Store.gnss,
				p = Store.power, pt = Store.ptp;

			clear(tiles);
			var lock = (s && s.lock_state) || (t && t.lock_state) || null;
			add(tiles, [
				tile('Stratum', s && isNum(s.stratum) ? s.stratum : null,
					'NTP advertised', (s && s.stratum === 1) ? 'ok' : 'warn'),
				tile('Lock state', lock, t ? 'flags ' + maskHex(t.flags) : '', lockClass(lock)),
				tile('Reference', (s && s.reference) || null,
					t && isNum(t.dac_code) ? 'DAC ' + t.dac_code : '',
					(s && s.reference === 'none') ? 'crit' : 'info'),
				tile('PPS offset', s && isNum(s.last_pps_offset_ns) ? fint(s.last_pps_offset_ns) : null,
					'ns' + (t && isNum(t.pps_offset_sigma_ns)
						? '   σ ' + t.pps_offset_sigma_ns.toFixed(1) : ''),
					!(s && isNum(s.last_pps_offset_ns)) ? 'mute'
						: Math.abs(s.last_pps_offset_ns) < 100 ? 'ok' : 'warn'),
				tile('GNSS fix', s ? s.gnss_fix : null,
					s ? (isNum(s.gnss_sv_used) ? s.gnss_sv_used : '?') + ' used of ' +
						(isNum(s.gnss_sv_visible) ? s.gnss_sv_visible : '?') + ' visible' : '',
					(s && (s.gnss_fix === 'time' || s.gnss_fix === '3d')) ? 'ok' : 'warn'),
				tile('Holdover', s ? boolTxt(s.holdover, 'YES', 'no') : null,
					s && isNum(s.holdover_est_err_ns) && s.holdover
						? 'est err ' + fint(s.holdover_est_err_ns) + ' ns' : '',
					(s && s.holdover) ? 'warn' : 'ok'),
				tile('Active alarms', alarmCount(),
					Store.alarms ? 'latched ' + maskHex(Store.alarms.latched_mask) : '',
					alarmCount() ? 'crit' : 'ok')
			]);

			trend.set(Store.ring);

			clear(qual);
			if (!t) {
				qual.appendChild(empty('No timing provider in this build.'));
			} else {
				var hs = t.holdover_state || {};
				add(qual, kv([
					['mean offset', isNum(t.pps_offset_mean_ns) ? t.pps_offset_mean_ns.toFixed(3) + ' ns' : null],
					['sigma', isNum(t.pps_offset_sigma_ns) ? t.pps_offset_sigma_ns.toFixed(3) + ' ns' : null],
					['freq error', isNum(t.freq_err_ppb) ? t.freq_err_ppb.toFixed(4) + ' ppb' : null],
					['root delay', isNum(t.root_delay_ns) ? fns(t.root_delay_ns) : null],
					['root disp', isNum(t.root_disp_ns) ? fns(t.root_disp_ns) : null],
					['holdover est err', isNum(hs.est_err_ns) ? fns(hs.est_err_ns) : null,
						t.holdover ? 'warn' : 'dim'],
					['holdover elapsed', isNum(hs.elapsed_s) ? fdur(hs.elapsed_s) : null],
					['time to demote', isNum(hs.time_to_demote_s) ? fdur(hs.time_to_demote_s)
						: 'not scheduled'],
					['osc temp', isNum(t.osc_temp_c) ? t.osc_temp_c.toFixed(2) + ' °C' : 'n/a'],
					['tick', isNum(t.tick) ? fint(t.tick) : null]
				]));
			}

			sky.set(g ? g.satellites : [],
				Cfg.loaded ? Cfg.val('gnss.elev.mask') : null,
				g ? g.detail_available : false);

			clear(svc);
			if (!s || !s.services) {
				svc.appendChild(empty('No service counters.'));
			} else {
				var v = s.services;
				add(svc, [
					el('div', { cls: 'btnrow', style: 'margin-bottom:9px' }, [
						el('span', { cls: 'pill ' + (v.ntp ? 'ok' : 'mute') }, [el('i', { text: 'ntp' }), el('b', { text: v.ntp ? 'up' : 'off' })]),
						el('span', { cls: 'pill ' + (v.nts ? 'ok' : 'mute') }, [el('i', { text: 'nts' }), el('b', { text: v.nts ? 'up' : 'off' })]),
						el('span', { cls: 'pill ' + (v.ptp ? 'ok' : 'mute') }, [el('i', { text: 'ptp' }), el('b', { text: v.ptp ? 'up' : 'off' })]),
						el('span', { cls: 'pill ' + (v.snmp ? 'ok' : 'mute') }, [el('i', { text: 'snmp' }), el('b', { text: v.snmp ? 'up' : 'off' })])
					]),
					kv([
						['NTP received', fint(v.ntp_rx)],
						['NTP served', fint(v.ntp_served)],
						['NTP KoD', fint(v.ntp_kod), v.ntp_kod ? 'warn' : null],
						['NTS served', fint(v.nts_served)]
					])
				]);
			}

			clear(ptpB);
			if (!pt) {
				ptpB.appendChild(empty('No PTP provider.'));
			} else {
				add(ptpB, kv([
					['running', boolTxt(pt.running), pt.running ? 'ok' : 'dim'],
					['port state', PTP_PORT_STATE[pt.port_state] || pt.port_state,
						pt.port_state === 6 ? 'ok' : 'warn'],
					['clock class', pt.clock_class],
					['accuracy', isNum(pt.clock_accuracy) ? maskHex(pt.clock_accuracy) : null],
					['domain', pt.domain],
					['alarms', isNum(pt.alarms) ? maskHex(pt.alarms) : null,
						pt.alarms ? 'crit' : null]
				]));
			}

			fillRails(p);

			clear(envB);
			if (!p) {
				envB.appendChild(empty('No power/health provider.'));
			} else {
				var tp = p.temperature || {}, poe = p.poe || {}, bk = p.backup || {}, fan = p.fan || {};
				var bw = el('div', { cls: 'barw' });
				var budget = isNum(poe.budget_w) && poe.budget_w > 0
					? (isNum(poe.draw_w) ? poe.draw_w / poe.budget_w : 0) : 0;
				bw.appendChild(bar(budget, budget > 0.9 ? 'crit' : budget > 0.75 ? 'warn' : 'ok'));
				bw.appendChild(el('span', {
					cls: 'bv',
					text: (isNum(poe.draw_w) ? poe.draw_w.toFixed(2) : '?') + ' / ' +
						(isNum(poe.budget_w) ? poe.budget_w.toFixed(1) : '?') + ' W'
				}));
				add(envB, kv([
					['oscillator', isNum(tp.oscillator_c) ? tp.oscillator_c.toFixed(2) + ' °C' : 'n/a'],
					['enclosure', isNum(tp.enclosure_c) ? tp.enclosure_c.toFixed(2) + ' °C' : 'n/a'],
					['MCU die', isNum(tp.die_c) ? tp.die_c.toFixed(2) + ' °C' : 'n/a'],
					['humidity', isNum(p.humidity_pct) ? p.humidity_pct.toFixed(1) + ' %' : 'n/a'],
					['fan', (isNum(fan.rpm) ? fint(fan.rpm) + ' rpm' : '—') +
						(isNum(fan.duty_pct) ? '  @ ' + fan.duty_pct + ' %' : '')],
					['PoE class', poe['class']],
					['PoE budget', bw],
					['supercap STM', boolTxt(bk.stm_pg, 'good', 'LOW'), bk.stm_pg ? 'ok' : 'warn'],
					['supercap GPS', boolTxt(bk.gps_pg, 'good', 'LOW'), bk.gps_pg ? 'ok' : 'warn'],
					['sample age', isNum(p.age_ms) ? p.age_ms + ' ms' : null]
				]));
			}

			al.update();
		}

		return {
			el: root,
			update: update,
			destroy: function () { sky.destroy(); },
			init: function () { Cfg.load().then(update, function () { /* optional */ }); }
		};
	}

	/* ------------------------------------------------------------- timing */

	function pageTiming() {
		var root = el('div', { cls: 'grid' });

		var steps = el('div', { cls: 'steps' });
		var refRow = el('div', { cls: 'btnrow', style: 'margin-top:10px' });
		var loopKv = el('div');
		root.appendChild(el('div', { cls: 'c12' },
			panel('Loop & reference state machine',
				el('div', { cls: 'pbody' }, [steps, el('div', { cls: 'sep' }), loopKv, refRow]))));

		var phase = Trend({ series: [{ key: 'off', color: C.accent, label: 'PPS offset (ns)' }], zero: true, tall: true });
		root.appendChild(el('div', { cls: 'c6' },
			panel('Phase error trend', el('div', { cls: 'pbody tight' },
				[phase.el, trendLegend([{ color: C.accent, label: 'PPS offset' }], 'ns')]))));

		var freq = Trend({ series: [{ key: 'ppb', color: C.violet, label: 'freq error (ppb)' }], zero: true, dp: 3, tall: true });
		root.appendChild(el('div', { cls: 'c6' },
			panel('Frequency error trend', el('div', { cls: 'pbody tight' },
				[freq.el, trendLegend([{ color: C.violet, label: 'freq error' }], 'ppb')]))));

		var vcS = [
			{ key: 'vc', color: C.ok, label: 'Vc commanded' },
			{ key: 'vs', color: C.amber, label: 'Vc sensed', dash: true }
		];
		var vc = Trend({ series: vcS, tall: true });
		root.appendChild(el('div', { cls: 'c6' },
			panel('OCXO control voltage', el('div', { cls: 'pbody tight' },
				[vc.el, trendLegend(vcS, 'mV')]))));

		var adev = AdevPlot();
		root.appendChild(el('div', { cls: 'c6' },
			panel('Allan deviation', el('div', { cls: 'pbody tight' },
				[adev.el, el('div', { cls: 'pnote' },
					'τ = 1 / 10 / 100 s, log-log. Published by the discipline thread.')]))));

		var cfgHost = el('div', { cls: 'c12' });
		root.appendChild(cfgHost);

		var may = Session.can('operator');
		['auto', 'ocxo', 'rb'].forEach(function (mode) {
			var b = el('button', {
				cls: 'btn sm', type: 'button', disabled: !may,
				text: mode === 'auto' ? 'Auto select' : mode === 'ocxo' ? 'Force OCXO' : 'Force Rb'
			});
			b.addEventListener('click', function () {
				confirmDlg({
					title: 'Change timing reference',
					text: mode === 'auto'
						? 'Return reference selection to automatic control?'
						: 'Force the timing reference to ' + mode.toUpperCase() +
						'? The switch bridges SYSCLK through the HSI while the mux moves. ' +
						'The request is refused if the reference guard is not satisfied.',
					okText: 'Apply',
					okClass: mode === 'auto' ? 'primary' : 'danger'
				}).then(function (ok) {
					if (!ok) { return; }
					return req('POST', 'timing/reference', { json: { mode: mode } })
						.then(function () { toast('reference mode set to ' + mode, 'ok', 'timing'); },
							function (e) {
								if (e.code === 'guard_not_satisfied') {
									toast('Reference guard not satisfied: ' + mode.toUpperCase() +
										' is not usable right now (10 MHz in-band and lock must both hold).',
										'err', 'refused');
								} else { errToast(e, 'reference'); }
							});
				});
			});
			refRow.appendChild(b);
		});
		refRow.appendChild(el('span', {
			cls: 'hint',
			text: may ? ' Active reference is shown above; the override mode itself is not read back by the API.'
				: ' Operator role required to override the reference.'
		}));

		function update() {
			var t = Store.timing;
			clear(steps);
			var lock = t ? t.lock_state : null;
			for (var i = 0; i < LOCK_STEPS.length; i++) {
				if (i) { steps.appendChild(el('span', { cls: 'sp', text: '›' })); }
				var on = (LOCK_STEPS[i] === lock);
				var extra = on ? (lock === 'holdover' ? ' hold' : '') : '';
				steps.appendChild(el('span', {
					cls: 'st' + (on ? ' on' + extra : ''), text: LOCK_STEPS[i]
				}));
			}
			if (lock === 'parked' || lock === 'unknown' || lock === 'invalid') {
				steps.appendChild(el('span', { cls: 'sp', text: ' ' }));
				steps.appendChild(el('span', { cls: 'st on bad', text: lock }));
			}
			steps.appendChild(el('span', { cls: 'sp', text: ' reference ' }));
			['ocxo', 'rb', 'extref'].forEach(function (r) {
				steps.appendChild(el('span', {
					cls: 'st' + (t && t.reference === r ? ' on' : ''), text: r
				}));
			});

			clear(loopKv);
			if (!t) {
				loopKv.appendChild(empty('No timing provider in this build.'));
			} else {
				var hs = t.holdover_state || {};
				var a = t.adev || {};
				add(loopKv, el('div', { cls: 'grid' }, [
					el('div', { cls: 'c4' }, kv([
						['stratum', t.stratum],
						['holdover', boolTxt(t.holdover, 'YES', 'no'), t.holdover ? 'warn' : 'ok'],
						['flags', maskHex(t.flags)],
						['tick', fint(t.tick)],
						['updated', isNum(t.updated_ms) ? fmono(t.updated_ms) : null]
					])),
					el('div', { cls: 'c4' }, kv([
						['last offset', fns(t.last_pps_offset_ns)],
						['mean offset', isNum(t.pps_offset_mean_ns) ? t.pps_offset_mean_ns.toFixed(3) + ' ns' : null],
						['sigma', isNum(t.pps_offset_sigma_ns) ? t.pps_offset_sigma_ns.toFixed(3) + ' ns' : null],
						['freq error', isNum(t.freq_err_ppb) ? t.freq_err_ppb.toFixed(4) + ' ppb' : null],
						['σy(1/10/100)', [a.tau_1s, a.tau_10s, a.tau_100s].map(function (x) {
							return isNum(x) ? x.toExponential(1) : '—';
						}).join('  ')]
					])),
					el('div', { cls: 'c4' }, kv([
						['Vc commanded', isNum(t.vc_cmd_mv) ? fint(t.vc_cmd_mv) + ' mV' : null],
						['Vc sensed', isNum(t.vc_sense_mv) ? fint(t.vc_sense_mv) + ' mV' : 'not sensed'],
						['DAC code', t.dac_code],
						['root delay / disp', fns(t.root_delay_ns) + ' / ' + fns(t.root_disp_ns)],
						['holdover', isNum(hs.est_err_ns)
							? fns(hs.est_err_ns) + ' after ' + fdur(hs.elapsed_s) : null],
						['time to demote', isNum(hs.time_to_demote_s)
							? fdur(hs.time_to_demote_s) : 'not scheduled']
					]))
				]));
			}

			phase.set(Store.ring);
			freq.set(Store.ring);
			vc.set(Store.ring);
			adev.set(t ? t.adev : null);
		}

		return {
			el: root,
			update: update,
			init: function () {
				Cfg.load().then(function () {
					clear(cfgHost);
					cfgHost.appendChild(CfgTable({
						title: 'Loop constants (tim.*)',
						prefixes: ['tim.'],
						note: [
							'τ is the discipline time constant in seconds; ',
							code('tim.lock.ns'), ' and ', code('tim.lock.hold'),
							' set the stratum-1 admission gate; ', code('tim.demote.ns'),
							' is the holdover demotion threshold.'
						]
					}).el);
				}, function (e) { errToast(e, 'config'); });
			}
		};
	}

	/* -------------------------------------------------------- calibration */

	function pageCalibration() {
		var root = el('div', { cls: 'grid' });
		var procHost = el('div', { cls: 'pbody' }, empty('loading…'));
		root.appendChild(el('div', { cls: 'c12' }, panel('Bench procedures', procHost)));
		var constHost = el('div', { cls: 'c12' });
		root.appendChild(constHost);

		var DESC = {
			holdover: 'Characterise the OCXO holdover drift model. Long run; the loop is ' +
				'left disciplined and the result feeds root-dispersion growth.',
			ocxo_tune: 'Sweep the DAC and fit the OCXO Vc pull curve. Steers the oscillator ' +
				'across its range - served time is disturbed.',
			ina_trim: 'Trim one INA228 SHUNT_CAL against a reference load. Removes the 1 % ' +
				'shunt tolerance that dominates the uncalibrated current error.',
			compass: 'Calibrate the IIS2MDC/LIS2DH12 e-compass hard/soft-iron offsets.'
		};

		function renderProcs(procs) {
			clear(procHost);
			var may = Session.can('operator');
			if (!procs.length) {
				procHost.appendChild(empty('No procedures reported.'));
				return;
			}
			var t = table(['Procedure', 'Code', 'Argument', { t: '' }]);
			for (var i = 0; i < procs.length; i++) {
				(function (p) {
					var tr = el('tr');
					tr.appendChild(el('td', { cls: 'tx' }, [
						el('b', { text: p.id }),
						el('div', { cls: 'hint wrapn', text: DESC[p.id] || '' })
					]));
					tr.appendChild(el('td', { cls: 'dim', text: String(p.code) }));

					var argIn;
					if (p.id === 'ina_trim') {
						argIn = el('select');
						for (var j = 0; j < INA_RAILS.length; j++) {
							argIn.appendChild(el('option', { value: String(j) },
								j + ' · ' + INA_RAILS[j]));
						}
					} else {
						argIn = el('input', {
							type: 'number', size: 8, min: 0, placeholder: 'optional'
						});
					}
					argIn.disabled = !may;
					tr.appendChild(el('td', null, argIn));

					var b = el('button', {
						cls: 'btn sm', type: 'button', disabled: !may, text: 'Run…'
					});
					b.addEventListener('click', function () {
						var raw = String(argIn.value).trim();
						var arg = raw === '' ? null : Number(raw);
						confirmDlg({
							title: 'Run ' + p.id,
							text: (DESC[p.id] || '') + ' Start this procedure now?' +
								(arg !== null ? ' Argument: ' + arg + '.' : ''),
							okText: 'Run'
						}).then(function (ok) {
							if (!ok) { return; }
							var body = { procedure: p.id };
							if (arg !== null && isFinite(arg)) { body.arg = arg; }
							return req('POST', 'calibration/run', { json: body })
								.then(function () { toast(p.id + ' started', 'ok', 'calibration'); },
									function (e) { errToast(e, 'calibration'); });
						});
					});
					tr.appendChild(el('td', null, b));
					t.body.appendChild(tr);
				}(procs[i]));
			}
			procHost.appendChild(t.el);
			if (!may) {
				procHost.appendChild(el('div', { cls: 'hint', style: 'margin-top:8px' },
					'Operator role required to run a procedure.'));
			}
		}

		return {
			el: root,
			init: function () {
				Promise.all([
					req('GET', 'calibration'),
					Cfg.load()
				]).then(function (r) {
					var cal = r[0] || {};
					renderProcs(Array.isArray(cal.procedures) ? cal.procedures : []);
					clear(constHost);
					/* /calibration returns the CFG_F_CAL keys in config-key shape,
					 * so the same editor drives them. */
					var keys = Array.isArray(cal.constants) ? cal.constants : [];
					constHost.appendChild(CfgTable({
						title: 'Calibration constants (' + keys.length + ')',
						keys: keys,
						note: ['Bench-produced values. ', code('cal.ina.0..8'),
							' are per-INA228 SHUNT_CAL trims (default 4096); ',
							code('cal.tempco'), ' is the only f32 key and is sent as integer ' +
							'micro-units. Changes stage like any other key and need a commit.']
					}).el);
				}).catch(function (e) {
					clear(procHost);
					procHost.appendChild(empty('calibration unavailable: ' + e.code));
					errToast(e, 'calibration');
				});
			}
		};
	}

	/* --------------------------------------------------------------- gnss */

	function pageGnss() {
		var root = el('div', { cls: 'grid' });
		var tiles = el('div', { cls: 'tiles' });
		root.appendChild(el('div', { cls: 'c12' }, tiles));

		var sky = Skyplot(true);
		root.appendChild(el('div', { cls: 'c7' },
			panel('Skyplot', el('div', { cls: 'pbody tight' }, [sky.el, skyLegend()]))));

		var right = el('div', { cls: 'c5' });
		var surveyB = el('div', { cls: 'pbody' });
		var posB = el('div', { cls: 'pbody' });
		var antB = el('div', { cls: 'pbody' });
		right.appendChild(panel('Survey-in', surveyB));
		right.appendChild(el('div', { style: 'height:12px' }));
		right.appendChild(panel('Stored position (ECEF)', posB));
		right.appendChild(el('div', { style: 'height:12px' }));
		right.appendChild(panel('Antenna & receiver', antB));
		root.appendChild(right);

		var st = table(['Constellation', 'SV', { t: 'C/N0', num: true },
			{ t: 'Elev', num: true }, { t: 'Azim', num: true }, 'Used']);
		var srows = RowSet(st.body, 6);
		var satNote = el('div', { cls: 'pnote' });
		root.appendChild(el('div', { cls: 'c12' },
			panel('Satellites', el('div', { cls: 'pbody tight' }, [st.el, satNote]))));

		var cfgHost = el('div', { cls: 'c12' });
		root.appendChild(cfgHost);

		var may = Session.can('operator');

		/* Also built once - a per-frame rebuild can swallow a click. */
		function surveyBtns() {
			var row = el('div', { cls: 'btnrow', style: 'margin-top:9px' });
			var start = el('button', {
				cls: 'btn sm', type: 'button', disabled: true, text: 'Start survey'
			});
			var stop = el('button', {
				cls: 'btn sm', type: 'button', disabled: true, text: 'Stop survey'
			});
			start.addEventListener('click', function () {
				confirmDlg({
					title: 'Start survey-in',
					text: 'Survey-in re-derives the antenna position. The receiver leaves ' +
						'fixed-position timing mode until it completes, which degrades timing ' +
						'accuracy for the duration. Continue?',
					okText: 'Start'
				}).then(function (ok) {
					if (!ok) { return; }
					return req('POST', 'gnss/survey', { json: { action: 'start' } })
						.then(function () { toast('survey started', 'ok', 'gnss'); },
							function (e) { errToast(e, 'survey'); });
				});
			});
			stop.addEventListener('click', function () {
				req('POST', 'gnss/survey', { json: { action: 'stop' } })
					.then(function () { toast('survey stopped', 'ok', 'gnss'); },
						function (e) { errToast(e, 'survey'); });
			});
			row.appendChild(start);
			row.appendChild(stop);
			if (!may) { row.appendChild(el('span', { cls: 'hint', text: ' operator role required' })); }
			row.sync = function (active) {
				start.disabled = !may || active;
				stop.disabled = !may || !active;
			};
			return row;
		}

		/* Built once at mount: rebuilding it on every telemetry frame would
		 * wipe whatever the operator is typing. Prefill happens only while the
		 * fields are untouched. */
		var posTouched = false;
		var posPrefilled = false;

		function posForm() {
			var xs = el('input', { type: 'number', size: 12, disabled: !may });
			var ys = el('input', { type: 'number', size: 12, disabled: !may });
			var zs = el('input', { type: 'number', size: 12, disabled: !may });
			var b = el('button', { cls: 'btn sm', type: 'button', disabled: !may, text: 'Set position' });
			[xs, ys, zs].forEach(function (n) {
				n.addEventListener('input', function () { posTouched = true; });
			});
			b.addEventListener('click', function () {
				var v = [xs.value, ys.value, zs.value].map(function (s) { return Number(String(s).trim()); });
				if (!v.every(function (n) { return isFinite(n) && Math.abs(n - Math.round(n)) === 0; })) {
					toast('all three ECEF coordinates must be whole centimetres', 'err', 'invalid');
					return;
				}
				confirmDlg({
					title: 'Set stored ECEF position',
					text: 'Overwrite the surveyed antenna position with X=' + v[0] + ' Y=' + v[1] +
						' Z=' + v[2] + ' cm? A wrong position biases every timestamp the ' +
						'receiver produces.',
					okText: 'Set'
				}).then(function (ok) {
					if (!ok) { return; }
					return req('POST', 'gnss/position', {
						json: { ecef_x_cm: Math.round(v[0]), ecef_y_cm: Math.round(v[1]), ecef_z_cm: Math.round(v[2]) }
					}).then(function () { toast('position stored', 'ok', 'gnss'); },
						function (e) { errToast(e, 'position'); });
				});
			});
			var wrap = el('div', null, [
				el('div', { cls: 'frow', style: 'margin-top:9px' }, [
					el('div', { cls: 'field' }, [el('label', { cls: 'fl', text: 'X cm' }), xs]),
					el('div', { cls: 'field' }, [el('label', { cls: 'fl', text: 'Y cm' }), ys]),
					el('div', { cls: 'field' }, [el('label', { cls: 'fl', text: 'Z cm' }), zs])
				]),
				el('div', { cls: 'btnrow', style: 'margin-top:8px' }, b)
			]);
			if (!may) {
				wrap.appendChild(el('div', { cls: 'hint', text: 'Operator role required.' }));
			}
			wrap.prefill = function (p) {
				if (posTouched || posPrefilled || !p || !isNum(p.ecef_x_cm)) { return; }
				posPrefilled = true;
				xs.value = String(p.ecef_x_cm);
				ys.value = String(isNum(p.ecef_y_cm) ? p.ecef_y_cm : 0);
				zs.value = String(isNum(p.ecef_z_cm) ? p.ecef_z_cm : 0);
			};
			return wrap;
		}

		var posDisp = el('div');
		var posFormEl = posForm();
		add(posB, [posDisp, posFormEl]);

		var surveyDisp = el('div');
		var surveyRow = surveyBtns();
		add(surveyB, [surveyDisp, surveyRow]);

		function update() {
			var g = Store.gnss, s = Store.summary;

			clear(tiles);
			add(tiles, [
				tile('Fix', g ? g.fix : (s ? s.gnss_fix : null),
					g ? boolTxt(g.utc_valid, 'UTC valid', 'UTC invalid') : '',
					(g && (g.fix === 'time' || g.fix === '3d')) ? 'ok' : 'warn'),
				tile('SV used', g ? g.sv_used : null,
					g ? 'of ' + (isNum(g.sv_visible) ? g.sv_visible : '?') + ' visible' : '',
					(g && isNum(g.sv_used) && g.sv_used >= 4) ? 'ok' : 'warn'),
				tile('Time accuracy', g && isNum(g.time_acc_ns) ? fint(g.time_acc_ns) : null, 'ns',
					(g && isNum(g.time_acc_ns) && g.time_acc_ns < 100) ? 'ok' : 'info'),
				tile('Leap seconds', g && isNum(g.leap_current_s) ? g.leap_current_s : null,
					g && isNum(g.leap_pending) && g.leap_pending
						? 'pending ' + g.leap_pending : 'none pending',
					(g && g.leap_pending) ? 'warn' : 'mute'),
				tile('Antenna', g && g.antenna ? g.antenna.state : null,
					g && g.antenna ? boolTxt(g.antenna.bias_on, 'bias on', 'bias off') : '',
					(g && g.antenna && g.antenna.state === 'ok') ? 'ok' : 'crit'),
				tile('Survey', g && g.survey ? g.survey.state : null,
					g && g.survey ? fdur(g.survey.duration_s) : '',
					(g && g.survey && g.survey.state === 'fixed') ? 'ok' : 'info')
			]);

			sky.set(g ? g.satellites : [],
				Cfg.loaded ? Cfg.val('gnss.elev.mask') : null,
				g ? g.detail_available : false);

			clear(surveyDisp);
			surveyRow.sync(!!(g && g.survey && g.survey.state === 'active'));
			if (!g) {
				surveyDisp.appendChild(empty('No GNSS provider.'));
			} else {
				var sv = g.survey || {};
				var accTarget = Cfg.loaded ? Cfg.val('gnss.survey.acc') : null;
				var durTarget = Cfg.loaded ? Cfg.val('gnss.survey.dur') : null;
				var accBar = el('div', { cls: 'barw' });
				var accFrac = (isNum(accTarget) && accTarget > 0 && isNum(sv.accuracy_mm) && sv.accuracy_mm > 0)
					? Math.min(1, accTarget / sv.accuracy_mm) : 0;
				accBar.appendChild(bar(accFrac, accFrac >= 1 ? 'ok' : 'warn'));
				accBar.appendChild(el('span', {
					cls: 'bv', text: (isNum(sv.accuracy_mm) ? sv.accuracy_mm : '?') + ' mm'
				}));
				var durBar = el('div', { cls: 'barw' });
				var durFrac = (isNum(durTarget) && durTarget > 0 && isNum(sv.duration_s))
					? sv.duration_s / durTarget : 0;
				durBar.appendChild(bar(durFrac, durFrac >= 1 ? 'ok' : null));
				durBar.appendChild(el('span', { cls: 'bv', text: fdur(sv.duration_s) }));
				surveyDisp.appendChild(kv([
					['state', sv.state, sv.state === 'fixed' ? 'ok' : sv.state === 'active' ? 'warn' : 'dim'],
					['observations', fint(sv.observations)],
					['accuracy vs target', accBar],
					['elapsed vs target', durBar],
					['targets', (isNum(accTarget) ? accTarget + ' mm' : '?') + ' / ' +
						(isNum(durTarget) ? fdur(durTarget) : '?')]
				]));
			}

			clear(posDisp);
			if (!g) {
				posDisp.appendChild(empty('No GNSS provider.'));
			} else {
				var p = g.position || {};
				posDisp.appendChild(kv([
					['valid', boolTxt(p.valid), p.valid ? 'ok' : 'warn'],
					['X', isNum(p.ecef_x_cm) ? fint(p.ecef_x_cm) + ' cm' : null],
					['Y', isNum(p.ecef_y_cm) ? fint(p.ecef_y_cm) + ' cm' : null],
					['Z', isNum(p.ecef_z_cm) ? fint(p.ecef_z_cm) + ' cm' : null]
				]));
				posFormEl.prefill(p);
			}

			clear(antB);
			if (!g) {
				antB.appendChild(empty('No GNSS provider.'));
			} else {
				var an = g.antenna || {};
				add(antB, kv([
					['supervisor', an.state, an.state === 'ok' ? 'ok' : 'crit'],
					['bias', boolTxt(an.bias_on, 'on', 'off')],
					['detail available', boolTxt(g.detail_available),
						g.detail_available ? 'ok' : 'warn'],
					['software', g.sw_version],
					['hardware', g.hw_version],
					['cable delay', Cfg.loaded && isNum(Cfg.val('gnss.cable.ns'))
						? Cfg.val('gnss.cable.ns') + ' ns' : null]
				]));
			}

			var sats = (g && Array.isArray(g.satellites)) ? g.satellites.slice() : [];
			sats.sort(function (a, b) {
				return (isNum(b.cno) ? b.cno : -1) - (isNum(a.cno) ? a.cno : -1);
			});
			srows.sync(sats, function (x) { return (x.constellation || '?') + '|' + x.sv; },
				function (td, x) {
					cell(td[0], x.constellation, 'tx');
					cell(td[1], x.sv);
					cell(td[2], x.cno, 'num ' + (isNum(x.cno) && x.cno >= 35 ? 'ok'
						: isNum(x.cno) && x.cno >= 20 ? '' : 'warn'));
					cell(td[3], isNum(x.elev) ? x.elev + '°' : null, 'num');
					cell(td[4], isNum(x.azim) ? x.azim + '°' : null, 'num');
					cell(td[5], x.used ? 'yes' : 'no', x.used ? 'ok' : 'dim');
				});
			satNote.textContent = (g && g.detail_available === false)
				? 'No receiver detail: the GNSS receiver thread is not present in this build, ' +
				'so the satellite list is empty by design.'
				: sats.length + ' tracked';
		}

		return {
			el: root,
			update: update,
			destroy: function () { sky.destroy(); },
			init: function () {
				Cfg.load().then(function () {
					clear(cfgHost);
					cfgHost.appendChild(CfgTable({
						title: 'GNSS configuration (gnss.*)',
						prefixes: ['gnss.'],
						note: [code('gnss.constel'), ' is a constellation bitmask; ',
							code('gnss.elev.mask'), ' also drives the shaded band on the skyplot; ',
							code('gnss.cable.ns'), ' is a calibration constant, not a guess.']
					}).el);
					update();
				}, function (e) { errToast(e, 'config'); });
			}
		};
	}

	/* ------------------------------------------------------------ network */

	function pageNetwork() {
		var root = el('div', { cls: 'grid' });
		var ifB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c6' }, panel('Interface', ifB)));
		var servoB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c6' }, panel('PTP engine & servo', servoB)));
		var svcB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c12' }, panel('Services', svcB)));
		var hosts = {};
		['net', 'ntp', 'nts', 'ptp'].forEach(function (g) {
			var h = el('div', { cls: g === 'net' || g === 'ptp' ? 'c12' : 'c6' });
			hosts[g] = h;
			root.appendChild(h);
		});

		var may = Session.can('operator');
		var SERVICES = [
			{ id: 'ntp', label: 'NTP server' },
			{ id: 'nts', label: 'NTS key establishment' },
			{ id: 'ptp', label: 'PTP grandmaster' },
			{ id: 'snmp', label: 'SNMP agent' },
			{ id: 'syslog', label: 'Remote syslog' }
		];

		function svcState(id) {
			var s = Store.summary;
			if (id === 'syslog') {
				var v = Cfg.loaded ? Cfg.val('log.syslog.en') : null;
				return typeof v === 'boolean' ? v : null;
			}
			if (s && s.services && typeof s.services[id] === 'boolean') {
				return s.services[id];
			}
			return null;
		}

		/* Rebuilt only when a state actually changes, so the buttons are not
		 * replaced underneath a click at the telemetry rate. */
		var svcSig = null;

		function renderSvc(force) {
			var sig = SERVICES.map(function (s) { return String(svcState(s.id)); }).join(',') +
				'|' + Session.role;
			if (!force && sig === svcSig) { return; }
			svcSig = sig;
			clear(svcB);
			var t = table(['Service', 'State', 'Source', { t: '' }]);
			for (var i = 0; i < SERVICES.length; i++) {
				(function (sv) {
					var st = svcState(sv.id);
					var tr = el('tr');
					tr.appendChild(el('td', { cls: 'tx', text: sv.label }));
					tr.appendChild(el('td', null, dot(st === true ? 'ok' : st === false ? 'off' : 'warn',
						st === true ? 'running' : st === false ? 'stopped' : 'unknown')));
					tr.appendChild(el('td', {
						cls: 'dim',
						text: sv.id === 'syslog' ? 'config key log.syslog.en' : 'summary.services'
					}));
					var row = el('div', { cls: 'btnrow' });
					[['Enable', true], ['Disable', false]].forEach(function (a) {
						var b = el('button', {
							cls: 'btn sm' + (st === a[1] ? ' on' : ''), type: 'button',
							disabled: !may || st === a[1], text: a[0]
						});
						b.addEventListener('click', function () {
							req('POST', 'services/' + sv.id, { json: { enable: a[1] } })
								.then(function () {
									toast(sv.label + ' ' + (a[1] ? 'enabled' : 'disabled'), 'ok', 'services');
									function again() { renderSvc(true); }
									return Cfg.load(true).then(again, again);
								}, function (e) { errToast(e, 'services'); });
						});
						row.appendChild(b);
					});
					tr.appendChild(el('td', null, row));
					t.body.appendChild(tr);
				}(SERVICES[i]));
			}
			svcB.appendChild(t.el);
			svcB.appendChild(el('div', { cls: 'hint', style: 'margin-top:8px' },
				may ? ['Toggling a service is immediate and is not a staged config change. ',
					'Persist the matching ', code('*.enable'), ' key if it must survive a reboot.']
					: ['Operator role required to change service state.']));
		}

		function update() {
			var n = Store.net, pt = Store.ptp;
			clear(ifB);
			if (!n) {
				ifB.appendChild(empty('No network provider.'));
			} else {
				add(ifB, kv([
					['link', boolTxt(n.link_up, 'up', 'DOWN'), n.link_up ? 'ok' : 'crit'],
					['IPv4', boolTxt(n.ipv4_ok, 'ok', 'no'), n.ipv4_ok ? 'ok' : 'warn'],
					['IPv6', boolTxt(n.ipv6_ok, 'ok', 'no'), n.ipv6_ok ? 'ok' : 'warn'],
					['DHCP', boolTxt(n.dhcp_bound, 'bound', 'static/unbound')],
					['address', ipOf(n.ipv4_addr)],
					['netmask', ipOf(n.ipv4_mask)],
					['gateway', ipOf(n.ipv4_gateway)],
					['MAC', n.mac],
					['hostname', n.hostname],
					['link changes', fint(n.link_changes)]
				]));
			}

			clear(servoB);
			var pc = n ? n.ptp_clock : null;
			if (!pt && !pc) {
				servoB.appendChild(empty('No PTP provider.'));
			} else {
				add(servoB, kv([
					['engine', pt ? boolTxt(pt.running, 'running', 'stopped') : null,
						(pt && pt.running) ? 'ok' : 'dim'],
					['port state', pt ? (PTP_PORT_STATE[pt.port_state] || pt.port_state) : null,
						(pt && pt.port_state === 6) ? 'ok' : 'warn'],
					['clock class', pt ? pt.clock_class : null],
					['clock accuracy', pt && isNum(pt.clock_accuracy) ? maskHex(pt.clock_accuracy) : null],
					['domain', pt ? pt.domain : null],
					['transport', pt ? (PTP_TRANSPORT[pt.transport] || pt.transport) : null],
					['tx / rx', pt ? fint(pt.tx_total) + ' / ' + fint(pt.rx_total) : null],
					['announce timeouts', pt ? fint(pt.announce_timeouts) : null,
						(pt && pt.announce_timeouts) ? 'warn' : null],
					['follow-up missed', pt ? fint(pt.followup_missed) : null,
						(pt && pt.followup_missed) ? 'warn' : null],
					['alarms', pt && isNum(pt.alarms) ? maskHex(pt.alarms) : null,
						(pt && pt.alarms) ? 'crit' : null],
					['— MAC clock —', ''],
					['present', pc ? boolTxt(pc.present) : null, (pc && pc.present) ? 'ok' : 'dim'],
					['synced', pc ? boolTxt(pc.synced) : null, (pc && pc.synced) ? 'ok' : 'warn'],
					['offset', pc && isNum(pc.offset_ns) ? fns(pc.offset_ns) : null],
					['rate', pc && isNum(pc.rate_ppb) ? fint(pc.rate_ppb) + ' ppb' : null]
				]));
			}
			renderSvc();
		}

		return {
			el: root,
			update: update,
			init: function () {
				Cfg.load().then(function () {
					clear(hosts.net);
					hosts.net.appendChild(CfgTable({
						title: 'Interface configuration (net.*)',
						prefixes: ['net.'],
						note: ['IPv4 fields are u32 in host order. ', code('net.hostname'),
							' is also the mDNS instance name; ', code('net.mgmt.acl'),
							' is a management ACL blob (lowercase hex). ',
							'The schema has no separate mDNS enable key.']
					}).el);
					clear(hosts.ntp);
					hosts.ntp.appendChild(CfgTable({
						title: 'NTP (ntp.*)', prefixes: ['ntp.']
					}).el);
					clear(hosts.nts);
					hosts.nts.appendChild(CfgTable({
						title: 'NTS (nts.*)', prefixes: ['nts.']
					}).el);
					clear(hosts.ptp);
					hosts.ptp.appendChild(CfgTable({
						title: 'PTP (ptp.*)', prefixes: ['ptp.']
					}).el);
					renderSvc(true);
				}, function (e) { errToast(e, 'config'); });
			}
		};
	}

	/* ----------------------------------------------------------- security */

	function pageSecurity() {
		var root = el('div', { cls: 'grid' });
		var admin = Session.can('admin');

		var usersB = el('div', { cls: 'pbody tight' }, empty('loading…'));
		root.appendChild(el('div', { cls: 'c6' }, panel('Users & roles', usersB)));
		var pwB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c6' }, panel('Change password', pwB)));
		var tlsB = el('div', { cls: 'pbody' }, empty('loading…'));
		root.appendChild(el('div', { cls: 'c12' }, panel('TLS identity', tlsB)));
		var backupB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c12' },
			panel('Configuration backup', backupB)));
		var cfgHost = el('div', { cls: 'c12' });
		root.appendChild(cfgHost);
		var dangerB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c12' }, panel('Danger zone', dangerB)));

		var userNames = ['admin'];

		var IMPORT_ERR = {
			bad_magic: 'not a Meridian config blob (bad magic)',
			bad_crc: 'the file is corrupt (CRC mismatch)',
			schema_version: 'built for a different schema version',
			value_out_of_range: 'a record holds an out-of-range value',
			malformed_record: 'a record is malformed'
		};

		function renderBackup() {
			clear(backupB);
			if (!admin) {
				backupB.appendChild(empty('Admin role required to export or import configuration.'));
				return;
			}
			var withSecrets = el('input', { type: 'checkbox' });
			var exp = el('button', { cls: 'btn sm', type: 'button', text: 'Export' });
			exp.addEventListener('click', function () {
				exp.disabled = true;
				req('GET', 'config/export' + (withSecrets.checked ? '?secrets=1' : ''),
					{ expect: 'blob' }).then(function (b) {
					saveBlob(b, 'meridian-config.mcf');
					toast('configuration exported' +
						(withSecrets.checked ? ' including secrets' : ''), 'ok', 'backup');
				}, function (e) { errToast(e, 'export'); })
					.then(function () { exp.disabled = false; });
			});

			var file = el('input', { type: 'file', accept: '.mcf,.bin' });
			var imp = el('button', { cls: 'btn sm', type: 'button', text: 'Import' });
			imp.addEventListener('click', function () {
				var f = file.files && file.files[0];
				if (!f) { toast('choose an exported .mcf file first', 'err', 'import'); return; }
				confirmDlg({
					title: 'Import configuration',
					text: 'Applying ' + f.name + ' (' + fbytes(f.size) + ') overwrites the ' +
						'current configuration. Some groups may need a reboot. Continue?',
					okText: 'Import'
				}).then(function (ok) {
					if (!ok) { return; }
					imp.disabled = true;
					return f.arrayBuffer().then(function (buf) {
						return req('POST', 'config/import', {
							body: buf, type: 'application/octet-stream'
						});
					}).then(function (r) {
						var msg = (r && isNum(r.applied) ? r.applied : '?') + ' key(s) applied';
						if (r && r.reboot_groups) {
							msg += '; ' + r.reboot_groups + ' group(s) need a reboot';
						}
						if (r && r.persist_errors) {
							msg += '; ' + r.persist_errors + ' failed to persist';
						}
						toast(msg, (r && r.persist_errors) ? 'warn' : 'ok', 'imported');
						return Cfg.load(true).then(paintStage, paintStage);
					}, function (e) {
						toast(IMPORT_ERR[e.code] || (e.code + (e.detail ? ': ' + e.detail : '')),
							'err', 'import rejected');
					}).then(function () { imp.disabled = false; });
				});
			});

			add(backupB, [
				el('div', { cls: 'grid' }, [
					el('div', { cls: 'c6' }, [
						el('label', { cls: 'fl', text: 'Export' }),
						el('div', { cls: 'btnrow' }, [exp,
							el('label', { cls: 'hint' }, [withSecrets, ' include secrets'])]),
						el('div', { cls: 'hint', style: 'margin-top:8px' },
							'Downloads a binary blob (CRC-protected, schema-versioned). ' +
							'Including secrets is audited on the device.')
					]),
					el('div', { cls: 'c6' }, [
						el('label', { cls: 'fl', text: 'Import' }),
						el('div', { cls: 'btnrow' }, [file, imp]),
						el('div', { cls: 'hint', style: 'margin-top:8px' },
							'Rejected as a whole on bad magic, CRC mismatch, schema mismatch, ' +
							'or any out-of-range record.')
					])
				])
			]);
		}

		function loadUsers() {
			if (!admin) {
				clear(usersB);
				usersB.appendChild(empty('Admin role required to list users.'));
				return Promise.resolve();
			}
			return req('GET', 'security/users').then(function (r) {
				clear(usersB);
				var t = table(['#', 'User', 'Role', 'Credential', 'Persistent', 'Failed', 'Lockout']);
				var us = (r && Array.isArray(r.users)) ? r.users : [];
				userNames = us.map(function (u) { return u.name; }).filter(Boolean);
				if (!userNames.length) { userNames = ['admin']; }
				for (var i = 0; i < us.length; i++) {
					var u = us[i];
					var tr = el('tr');
					tr.appendChild(el('td', { cls: 'dim', text: String(u.index) }));
					tr.appendChild(el('td', { cls: 'tx', text: u.name || '—' }));
					tr.appendChild(el('td', { text: u.role || '—' }));
					tr.appendChild(el('td', {
						cls: u.credential_set ? 'ok' : 'crit',
						text: u.credential_set ? 'set' : 'NOT SET'
					}));
					tr.appendChild(el('td', { cls: 'dim', text: boolTxt(u.persistent) }));
					tr.appendChild(el('td', {
						cls: u.failed_attempts ? 'warn' : 'dim',
						text: String(u.failed_attempts)
					}));
					tr.appendChild(el('td', {
						cls: u.lockout_ms ? 'crit' : 'dim',
						text: u.lockout_ms ? u.lockout_ms + ' ms' : '—'
					}));
					t.body.appendChild(tr);
				}
				usersB.appendChild(t.el);
				usersB.appendChild(el('div', { cls: 'pnote' },
					'KDF ' + (r.kdf || '?') + '   auth required ' +
					boolTxt(r.auth_required) + '   active sessions ' +
					(isNum(r.sessions) ? r.sessions : '?')));
				renderPw();
			}, function (e) {
				clear(usersB);
				usersB.appendChild(empty('users unavailable: ' + e.code));
			});
		}

		function renderPw() {
			clear(pwB);
			if (!admin) {
				pwB.appendChild(empty('Admin role required to change a password.'));
				return;
			}
			var sel = el('select');
			for (var i = 0; i < userNames.length; i++) {
				sel.appendChild(el('option', { value: userNames[i] }, userNames[i]));
			}
			var p1 = el('input', { type: 'password', autocomplete: 'new-password' });
			var p2 = el('input', { type: 'password', autocomplete: 'new-password' });
			var msg = el('div', { cls: 'hint' });
			var go = el('button', { cls: 'btn primary sm', type: 'button', text: 'Set password' });
			var commitRow = el('div', { cls: 'btnrow', style: 'margin-top:9px' });

			function bytes(s) { return new TextEncoder().encode(s).length; }

			go.addEventListener('click', function () {
				var a = p1.value, b = p2.value;
				if (a !== b) { msg.textContent = 'the two entries differ'; return; }
				var n = bytes(a);
				if (n < 8 || n > 64) {
					msg.textContent = 'password must be 8..64 bytes (this one is ' + n + ')';
					return;
				}
				msg.textContent = '';
				go.disabled = true;
				req('POST', 'security/password', { json: { user: sel.value, password: a } })
					.then(function (r) {
						p1.value = ''; p2.value = '';
						toast('credential staged for ' + sel.value, 'ok', 'security');
						clear(commitRow);
						if (!r || r.commit_required !== false) {
							commitRow.appendChild(el('span', { cls: 'hint' },
								'The credential is staged. Commit to persist it: '));
							var cb = el('button', {
								cls: 'btn primary sm', type: 'button', text: 'Commit now'
							});
							cb.addEventListener('click', function () {
								cb.disabled = true;
								Cfg.commit().then(function () {
									toast('configuration committed', 'ok', 'config');
									clear(commitRow);
									paintStage();
								}, function (e) { errToast(e, 'commit'); cb.disabled = false; });
							});
							commitRow.appendChild(cb);
						}
						return Cfg.load(true).then(paintStage, function () { /* best effort */ });
					}, function (e) {
						if (e.code === 'password_policy') {
							msg.textContent = 'rejected by policy: ' + (e.detail || '8..64 bytes');
						} else { errToast(e, 'password'); }
					}).then(function () { go.disabled = false; });
			});

			add(pwB, [
				el('div', { cls: 'field' }, [el('label', { cls: 'fl', text: 'User' }), sel]),
				el('div', { cls: 'field' }, [el('label', { cls: 'fl', text: 'New password' }), p1]),
				el('div', { cls: 'field' }, [el('label', { cls: 'fl', text: 'Repeat' }), p2]),
				msg,
				el('div', { cls: 'btnrow' }, go),
				commitRow,
				el('div', { cls: 'hint', style: 'margin-top:9px' },
					['Minimum 8 bytes, maximum 64. The credential is written through ',
						code('POST /security/password'), ' and staged in ', code('sec.admin.pw'),
						' — it is never editable as a raw blob.'])
			]);
		}

		function loadTls() {
			if (!admin) {
				clear(tlsB);
				tlsB.appendChild(empty('Admin role required to view the certificate.'));
				return Promise.resolve();
			}
			return req('GET', 'security/tls').then(function (c) {
				clear(tlsB);
				var acme = c.acme || {};
				var left = el('div', { cls: 'c6' }, kv([
					['present', boolTxt(c.present), c.present ? 'ok' : 'crit'],
					['self-signed', boolTxt(c.self_signed), c.self_signed ? 'warn' : 'ok'],
					['operator supplied', boolTxt(c.operator_supplied)],
					['persisted', boolTxt(c.persisted), c.persisted ? 'ok' : 'warn'],
					['subject', c.subject],
					['issuer', c.issuer],
					['not before', c.not_before],
					['not after', c.not_after],
					['key', (c.key_type || '?') + ' ' + (isNum(c.key_bits) ? c.key_bits + ' bit' : '')],
					['SHA-256', el('span', { cls: 'fpr', text: c.sha256_fingerprint || '—' })],
					['ACME', (ACME_STATE[acme.state] || String(acme.state)) +
						(acme.enabled ? ' (enabled)' : ' (disabled)'), 'dim'],
					['ACME detail', acme.detail || '—', 'dim']
				]));

				var pem = el('textarea', {
					rows: 9,
					placeholder: '-----BEGIN CERTIFICATE-----\n…\n-----BEGIN PRIVATE KEY-----\n…'
				});
				var file = el('input', { type: 'file', accept: '.pem,.crt,.key,.txt' });
				file.addEventListener('change', function () {
					var f = file.files && file.files[0];
					if (!f) { return; }
					f.text().then(function (txt) { pem.value = txt; },
						function () { toast('could not read that file', 'err', 'upload'); });
				});
				var up = el('button', { cls: 'btn sm', type: 'button', text: 'Install PEM' });
				up.addEventListener('click', function () {
					var body = pem.value;
					if (!/-----BEGIN /.test(body)) {
						toast('paste a PEM bundle containing the certificate and its private key',
							'err', 'invalid');
						return;
					}
					confirmDlg({
						title: 'Install TLS certificate',
						text: 'Installing a new certificate closes every active session, this one ' +
							'included, and the HTTPS listener must be restarted (or the unit ' +
							'rebooted) before it serves the new chain. Continue?',
						okText: 'Install'
					}).then(function (ok) {
						if (!ok) { return; }
						up.disabled = true;
						return req('POST', 'security/tls', {
							body: body, type: 'application/x-pem-file'
						}).then(function () {
							toast('certificate installed; restart the listener to serve it',
								'ok', 'tls');
							pem.value = '';
						}, function (e) { errToast(e, 'tls install'); })
							.then(function () { up.disabled = false; });
					});
				});

				var subj = el('input', {
					type: 'text', value: c.subject || 'CN=meridian', size: 30
				});
				var csr = el('button', { cls: 'btn sm', type: 'button', text: 'Generate CSR' });
				csr.addEventListener('click', function () {
					csr.disabled = true;
					req('POST', 'security/csr', {
						json: { subject: subj.value }, expect: 'blob'
					}).then(function (b) {
						saveBlob(b, 'meridian.csr');
						toast('CSR downloaded', 'ok', 'security');
					}, function (e) { errToast(e, 'csr'); })
						.then(function () { csr.disabled = false; });
				});

				var right = el('div', { cls: 'c6' }, [
					el('label', { cls: 'fl', text: 'Certificate + key (PEM)' }),
					pem,
					el('div', { cls: 'btnrow', style: 'margin-top:7px' }, [file, up]),
					el('div', { cls: 'sep' }),
					el('div', { cls: 'frow' }, [
						el('div', { cls: 'field' }, [
							el('label', { cls: 'fl', text: 'CSR subject' }), subj
						]),
						csr
					]),
					el('div', { cls: 'hint', style: 'margin-top:9px' },
						'ACME state is read-only here; there is no API to drive enrolment from the SPA.')
				]);
				tlsB.appendChild(el('div', { cls: 'grid' }, [left, right]));
			}, function (e) {
				clear(tlsB);
				tlsB.appendChild(empty('certificate unavailable: ' + e.code));
			});
		}

		function renderDanger() {
			clear(dangerB);
			if (!admin) {
				dangerB.appendChild(empty('Admin role required.'));
				return;
			}
			var b = el('button', { cls: 'btn danger sm', type: 'button', text: 'Factory reset…' });
			b.addEventListener('click', function () {
				var inp = el('input', { type: 'text', size: 16, placeholder: 'FACTORY' });
				confirmDlg({
					title: 'Factory reset',
					text: 'This erases all configuration, credentials, calibration constants and ' +
						'the stored antenna position. Type FACTORY to confirm.',
					body: el('div', { cls: 'field', style: 'margin-top:6px' }, inp),
					okText: 'Erase everything',
					enable: function () { return inp.value === 'FACTORY'; },
					value: function () { return inp.value; }
				}).then(function (v) {
					if (v !== 'FACTORY') { return; }
					return req('POST', 'factory-reset', { json: { confirm: 'FACTORY' } })
						.then(function () {
							toast('factory reset accepted; the unit will restart with defaults',
								'warn', 'system');
						}, function (e) { errToast(e, 'factory reset'); });
				});
			});
			add(dangerB, [
				el('div', { cls: 'btnrow' }, b),
				el('div', { cls: 'hint', style: 'margin-top:8px' },
					['Reboot and bootloader-recovery controls are on the ',
						el('a', { cls: 'lnk', href: '#/firmware', text: 'Firmware' }), ' page.'])
			]);
		}

		return {
			el: root,
			init: function () {
				renderDanger();
				renderBackup();
				renderPw();
				loadUsers();
				loadTls();
				Cfg.load().then(function () {
					clear(cfgHost);
					cfgHost.appendChild(CfgTable({
						title: 'Security configuration (sec.*)',
						prefixes: ['sec.'],
						note: [code('sec.admin.pw'), ' is write-only and NOEXPORT — use the ' +
							'password form above. ', code('sec.auth.req'),
							' disables authentication entirely; do not clear it on a reachable network.']
					}).el);
				}, function (e) { errToast(e, 'config'); });
			}
		};
	}

	/* -------------------------------------------------------------- power */

	function pagePower() {
		var root = el('div', { cls: 'grid' });
		var tiles = el('div', { cls: 'tiles' });
		root.appendChild(el('div', { cls: 'c12' }, tiles));

		var rt = table(['Rail', 'Ref', 'Addr', 'Bus', 'Current', 'Power',
			'Load', 'Design max', 'State']);
		var fill = railRows(rt, true);
		var railNote = el('div', { cls: 'pnote' });
		root.appendChild(el('div', { cls: 'c12' },
			panel('INA228 rails', el('div', { cls: 'pbody tight' }, [rt.el, railNote]),
				metricsLink())));

		var envB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c6' }, panel('Thermal & humidity', envB)));
		var poeB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c6' }, panel('PoE & backup', poeB)));

		var al = alarmsPanel();
		root.appendChild(el('div', { cls: 'c12' }, al.el));

		function update() {
			var p = Store.power;
			clear(tiles);
			if (!p) {
				tiles.appendChild(empty('No power/health provider in this build.'));
				railNote.textContent = '';
				al.update();
				return;
			}
			var tp = p.temperature || {}, poe = p.poe || {}, fan = p.fan || {}, bk = p.backup || {};
			var totalW = 0, alerts = 0, invalid = 0;
			var rails = Array.isArray(p.rails) ? p.rails : [];
			for (var i = 0; i < rails.length; i++) {
				if (isNum(rails[i].power_w)) { totalW += rails[i].power_w; }
				if (rails[i].diag_alrt) { alerts++; }
				if (!rails[i].valid) { invalid++; }
			}
			add(tiles, [
				tile('PoE draw', isNum(poe.draw_w) ? poe.draw_w.toFixed(2) : null,
					'of ' + (isNum(poe.budget_w) ? poe.budget_w.toFixed(1) : '?') + ' W budget',
					(isNum(poe.draw_w) && isNum(poe.budget_w) && poe.budget_w > 0 &&
						poe.draw_w / poe.budget_w > 0.9) ? 'crit' : 'ok'),
				tile('PoE class', poe['class'], 'negotiated', 'info'),
				tile('Rail power', totalW ? totalW.toFixed(2) : null, 'W summed over rails', 'info'),
				tile('Oscillator', isNum(tp.oscillator_c) ? tp.oscillator_c.toFixed(2) : null,
					'°C  TMP117 #1',
					isNum(tp.oscillator_c) && tp.oscillator_c > 75 ? 'crit' : 'ok'),
				tile('Enclosure', isNum(tp.enclosure_c) ? tp.enclosure_c.toFixed(2) : null,
					'°C  TMP117 #2',
					isNum(tp.enclosure_c) && tp.enclosure_c > 60 ? 'warn' : 'ok'),
				tile('Humidity', isNum(p.humidity_pct) ? p.humidity_pct.toFixed(1) : null,
					'%RH  SHT45', 'info'),
				tile('Fan', isNum(fan.rpm) ? fint(fan.rpm) : null,
					isNum(fan.duty_pct) ? fan.duty_pct + ' % duty' : '',
					isNum(fan.rpm) && fan.rpm === 0 ? 'crit' : 'ok'),
				tile('Rail alerts', alerts, invalid ? invalid + ' invalid readings' : 'all valid',
					alerts ? 'crit' : 'ok')
			]);

			fill(p);
			railNote.textContent = rails.length + ' rails   sample age ' +
				(isNum(p.age_ms) ? p.age_ms + ' ms' : '?') +
				'   load bar is current against the rail design maximum';

			clear(envB);
			add(envB, kv([
				['oscillator (TMP117 #1)', isNum(tp.oscillator_c) ? tp.oscillator_c.toFixed(3) + ' °C' : 'n/a'],
				['enclosure (TMP117 #2)', isNum(tp.enclosure_c) ? tp.enclosure_c.toFixed(3) + ' °C' : 'n/a'],
				['MCU die', isNum(tp.die_c) ? tp.die_c.toFixed(3) + ' °C' : 'n/a'],
				['humidity (SHT45)', isNum(p.humidity_pct) ? p.humidity_pct.toFixed(2) + ' %' : 'n/a'],
				['fan speed', isNum(fan.rpm) ? fint(fan.rpm) + ' rpm' : null,
					isNum(fan.rpm) && fan.rpm === 0 ? 'crit' : null],
				['fan duty', isNum(fan.duty_pct) ? fan.duty_pct + ' %' : null]
			]));
			envB.appendChild(el('div', { cls: 'hint', style: 'margin-top:9px' },
				'The fan rests at maximum airflow, so a stalled control loop cannot overheat the box.'));

			clear(poeB);
			var bw = el('div', { cls: 'barw' });
			var f = (isNum(poe.budget_w) && poe.budget_w > 0 && isNum(poe.draw_w))
				? poe.draw_w / poe.budget_w : 0;
			bw.appendChild(bar(f, f > 0.9 ? 'crit' : f > 0.75 ? 'warn' : 'ok'));
			bw.appendChild(el('span', {
				cls: 'bv',
				text: (f * 100).toFixed(0) + ' %'
			}));
			add(poeB, kv([
				['class', poe['class']],
				['draw', isNum(poe.draw_w) ? poe.draw_w.toFixed(3) + ' W' : null],
				['budget', isNum(poe.budget_w) ? poe.budget_w.toFixed(3) + ' W' : null],
				['utilisation', bw],
				['supercap STM (BKP_STM_PG)', boolTxt(bk.stm_pg, 'charged', 'LOW'),
					bk.stm_pg ? 'ok' : 'warn'],
				['supercap GPS (BKP_GPS_PG)', boolTxt(bk.gps_pg, 'charged', 'LOW'),
					bk.gps_pg ? 'ok' : 'warn']
			]));

			al.update();
		}

		return { el: root, update: update };
	}

	/* --------------------------------------------------------------- logs */

	function pageLogs() {
		var root = el('div', { cls: 'grid' });
		var paused = false;
		var subFilter = '';
		var levelFilter = 7;
		var seenSubs = Object.create(null);

		var lvSel = el('select');
		for (var i = 0; i <= 7; i++) {
			lvSel.appendChild(el('option', { value: String(i) }, i + ' ' + LEVEL_NAME[i]));
		}
		lvSel.value = '7';
		var subSel = el('select');
		subSel.appendChild(el('option', { value: '' }, 'all subsystems'));

		var pauseBtn = el('button', { cls: 'btn sm', type: 'button', text: 'Pause' });
		var clearBtn = el('button', { cls: 'btn sm', type: 'button', text: 'Clear view' });
		var dlBtn = el('button', { cls: 'btn sm', type: 'button', text: 'Download…' });
		var meta = el('span', { cls: 'hint' });

		var view = el('div', { cls: 'logs' });
		var ctl = el('div', { cls: 'pbody' }, [
			el('div', { cls: 'frow' }, [
				el('div', { cls: 'field' }, [el('label', { cls: 'fl', text: 'Max level' }), lvSel]),
				el('div', { cls: 'field' }, [el('label', { cls: 'fl', text: 'Subsystem' }), subSel]),
				pauseBtn, clearBtn, dlBtn,
				el('span', { style: 'flex:1 1 auto' }),
				metricsLink()
			]),
			el('div', { style: 'margin-top:8px' }, meta)
		]);
		root.appendChild(el('div', { cls: 'c12' },
			panel('Live log tail', el('div', null, [ctl, view]))));
		var cfgHost = el('div', { cls: 'c12' });
		root.appendChild(cfgHost);

		function passes(r) {
			if (r._gap) { return true; }
			if (isNum(r.level) && r.level > levelFilter) { return false; }
			if (subFilter && r.subsystem !== subFilter) { return false; }
			return true;
		}

		function rowFor(r) {
			if (r._gap) {
				return el('div', { cls: 'lr gapline', text: '--- ' + r._gap + ' record(s) dropped ---' });
			}
			return el('div', { cls: 'lr l' + (isNum(r.level) ? r.level : 6) }, [
				el('span', { cls: 'lt', text: fmono(r.mono_ms) }),
				el('span', { cls: 'll', text: r.level_name || LEVEL_NAME[r.level] || '?' }),
				el('span', { cls: 'ls', text: r.subsystem || '-' }),
				el('span', { cls: 'lm', text: r.message === undefined ? '' : String(r.message) })
			]);
		}

		function noteSub(r) {
			if (r._gap || !r.subsystem || seenSubs[r.subsystem]) { return; }
			seenSubs[r.subsystem] = 1;
			var keep = subSel.value;
			subSel.appendChild(el('option', { value: r.subsystem }, r.subsystem));
			subSel.value = keep;
		}

		function atBottom() {
			return view.scrollHeight - view.scrollTop - view.clientHeight < 24;
		}

		function trimView() {
			while (view.childNodes.length > LOG_MAX) { view.removeChild(view.firstChild); }
		}

		function rebuild() {
			clear(view);
			var l = Store.logs;
			for (var j = 0; j < l.length; j++) {
				noteSub(l[j]);
				if (passes(l[j])) { view.appendChild(rowFor(l[j])); }
			}
			trimView();
			view.scrollTop = view.scrollHeight;
			paintMeta();
		}

		function paintMeta() {
			var m = Store.logMeta;
			if (!m) { meta.textContent = 'waiting for the first log frame…'; return; }
			meta.textContent = 'cursor ' + m.cursor + '  next ' + m.next_cursor +
				'  head ' + m.head + '  oldest ' + m.oldest +
				'  dropped ' + m.dropped + (paused ? '   [PAUSED]' : '') +
				'   showing ' + view.childNodes.length + ' of ' + Store.logs.length + ' buffered';
		}

		function onLogs(added) {
			if (paused) { paintMeta(); return; }
			var stick = atBottom();
			for (var j = 0; j < added.length; j++) {
				noteSub(added[j]);
				if (passes(added[j])) { view.appendChild(rowFor(added[j])); }
			}
			trimView();
			if (stick) { view.scrollTop = view.scrollHeight; }
			paintMeta();
		}

		lvSel.addEventListener('change', function () {
			levelFilter = Number(lvSel.value);
			rebuild();
		});
		subSel.addEventListener('change', function () {
			subFilter = subSel.value;
			rebuild();
		});
		pauseBtn.addEventListener('click', function () {
			paused = !paused;
			pauseBtn.textContent = paused ? 'Resume' : 'Pause';
			pauseBtn.classList.toggle('on', paused);
			if (!paused) { rebuild(); } else { paintMeta(); }
		});
		clearBtn.addEventListener('click', function () {
			Store.clearLogs();
			clear(view);
			paintMeta();
		});

		dlBtn.addEventListener('click', function () {
			var lvl = el('select');
			for (var k = 0; k <= 7; k++) {
				lvl.appendChild(el('option', { value: String(k) }, k + ' ' + LEVEL_NAME[k]));
			}
			lvl.value = String(levelFilter);
			var prog = el('div', { cls: 'hint', text: '' });
			confirmDlg({
				title: 'Download the log ring',
				text: 'Pages the whole ring over REST (32 records per request, the server cap) ' +
					'and saves it as a text file.',
				body: [el('div', { cls: 'field' }, [
					el('label', { cls: 'fl', text: 'Max level' }), lvl
				]), prog],
				okText: 'Download',
				okClass: 'primary'
			}).then(function (ok) {
				if (!ok) { return; }
				downloadLogs(Number(lvl.value));
			});
		});

		function downloadLogs(level) {
			var lines = [];
			var cursor = null;
			var guard = 0;
			toast('collecting log pages…', null, 'download');

			function page() {
				var q = 'logs?max=32&level=' + level;
				if (cursor !== null) { q += '&cursor=' + cursor; }
				return req('GET', q).then(function (r) {
					if (!r) { return null; }
					if (isNum(r.gap) && r.gap > 0) {
						lines.push('--- ' + r.gap + ' record(s) dropped ---');
					}
					var recs = Array.isArray(r.records) ? r.records : [];
					for (var j = 0; j < recs.length; j++) {
						var x = recs[j];
						lines.push([
							x.seq, fmono(x.mono_ms),
							(x.level_name || LEVEL_NAME[x.level] || '?'),
							(x.subsystem || '-'),
							String(x.message === undefined ? '' : x.message)
						].join('\t'));
					}
					var nx = isNum(r.next_cursor) ? r.next_cursor : null;
					var done = !recs.length || nx === null || nx === cursor ||
						lines.length >= LOG_DL_MAX || ++guard > 1200;
					cursor = nx;
					if (done) { return null; }
					return page();
				});
			}

			page().then(function () {
				var head = '# STS1000 Meridian log export\n' +
					'# records ' + lines.length + '  max level ' + level +
					'  exported ' + new Date().toISOString() + '\n' +
					'# seq\tmono\tlevel\tsubsystem\tmessage\n';
				saveBlob(new Blob([head + lines.join('\n') + '\n'], { type: 'text/plain' }),
					'meridian-log.txt');
				toast(lines.length + ' records saved', 'ok', 'download');
			}, function (e) { errToast(e, 'log download'); });
		}

		return {
			el: root,
			onLogs: onLogs,
			update: paintMeta,
			init: function () {
				Live.addGroup('logs');
				rebuild();
				Cfg.load().then(function () {
					clear(cfgHost);
					cfgHost.appendChild(CfgTable({
						title: 'Logging & syslog (log.*)',
						prefixes: ['log.'],
						note: [code('log.level'), ' is the device-side threshold (RFC 5424, 0 emerg ' +
							'.. 7 debug). The level and subsystem selectors above filter this view ' +
							'client-side; the WSS log stream carries no server-side level filter.']
					}).el);
				}, function (e) { errToast(e, 'config'); });
			}
		};
	}

	/* ----------------------------------------------------------- firmware */

	function pageFirmware() {
		var root = el('div', { cls: 'grid' });
		var admin = Session.can('admin');
		var slotsB = el('div', { cls: 'pbody tight' }, empty('loading…'));
		root.appendChild(el('div', { cls: 'c12' }, panel('Image slots', slotsB)));
		var upB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c7' }, panel('Upload a signed image', upB)));
		var actB = el('div', { cls: 'pbody' });
		root.appendChild(el('div', { cls: 'c5' }, panel('Boot actions', actB)));

		var info = null;
		var busy = false;
		var cancel = false;

		function loadInfo() {
			return req('GET', 'firmware').then(function (r) {
				info = r;
				renderSlots();
				renderActions();
				renderUpload();
			}, function (e) {
				clear(slotsB);
				slotsB.appendChild(empty('firmware info unavailable: ' + e.code));
			});
		}

		function renderSlots() {
			clear(slotsB);
			var t = table(['Slot', 'Version', 'Size', 'Valid', 'Active', 'Pending', 'Confirmed']);
			var ss = (info && Array.isArray(info.slots)) ? info.slots : [];
			for (var i = 0; i < ss.length; i++) {
				var s = ss[i];
				var tr = el('tr');
				tr.appendChild(el('td', { text: String(s.slot) }));
				tr.appendChild(el('td', { text: s.version || '—' }));
				tr.appendChild(el('td', { text: fbytes(s.size) }));
				tr.appendChild(el('td', { cls: s.valid ? 'ok' : 'crit', text: boolTxt(s.valid) }));
				tr.appendChild(el('td', { cls: s.active ? 'ok' : 'dim', text: boolTxt(s.active) }));
				tr.appendChild(el('td', { cls: s.pending ? 'warn' : 'dim', text: boolTxt(s.pending) }));
				tr.appendChild(el('td', {
					cls: s.confirmed ? 'ok' : 'warn', text: boolTxt(s.confirmed)
				}));
				t.body.appendChild(tr);
			}
			slotsB.appendChild(t.el);
			var u = (info && info.upload) || {};
			slotsB.appendChild(el('div', { cls: 'pnote' },
				'upload state ' + (DFU_STATE[u.state] || u.state) +
				'   written ' + fbytes(u.written) + ' of ' + fbytes(u.total) +
				'   chunk max ' + fbytes(u.chunk_max) +
				'   write block ' + fbytes(u.write_block) +
				(info && info.pending_confirm ? '   PENDING CONFIRM' : '')));
		}

		function renderActions() {
			clear(actB);
			if (!admin) {
				actB.appendChild(empty('Admin role required.'));
				return;
			}
			var rows = el('div');
			if (info && info.pending_confirm) {
				var cb = el('button', { cls: 'btn primary sm', type: 'button', text: 'Confirm this image' });
				cb.addEventListener('click', function () {
					req('POST', 'firmware/confirm').then(function () {
						toast('image confirmed', 'ok', 'firmware');
						loadInfo();
					}, function (e) { errToast(e, 'confirm'); });
				});
				var rb = el('button', { cls: 'btn danger sm', type: 'button', text: 'Revert' });
				rb.addEventListener('click', function () {
					confirmDlg({
						title: 'Revert to the previous image',
						text: 'The unit reboots into the previously active image. Timing service ' +
							'is interrupted for the duration of the restart.',
						okText: 'Revert'
					}).then(function (ok) {
						if (!ok) { return; }
						return req('POST', 'firmware/revert').then(function () {
							toast('revert requested', 'warn', 'firmware');
							loadInfo();
						}, function (e) { errToast(e, 'revert'); });
					});
				});
				rows.appendChild(el('div', { cls: 'btnrow' }, [cb, rb]));
				rows.appendChild(el('div', { cls: 'hint', style: 'margin:8px 0 12px' },
					'This image is running on trial. Confirm it to make the boot permanent, ' +
					'or revert. An unconfirmed image is rolled back automatically on the next reset.'));
			} else {
				rows.appendChild(el('div', { cls: 'hint', style: 'margin-bottom:12px' },
					'No image is pending confirmation, so confirm/revert are not offered.'));
			}

			var modes = [
				[0, 'Reboot', 'Normal restart.'],
				[1, 'Bootloader recovery', 'Restart into MCUboot serial recovery.'],
				[2, 'Halt to test', 'Restart and stop in the halt-to-test state.']
			];
			var mrow = el('div', { cls: 'btnrow' });
			for (var i = 0; i < modes.length; i++) {
				(function (m) {
					var b = el('button', {
						cls: 'btn sm' + (m[0] === 0 ? '' : ' danger'), type: 'button', text: m[1]
					});
					b.addEventListener('click', function () {
						confirmDlg({
							title: m[1],
							text: m[2] + ' Served time stops until the unit is back and re-locked. Continue?',
							okText: m[1]
						}).then(function (ok) {
							if (!ok) { return; }
							return req('POST', 'reboot', { json: { mode: m[0] } })
								.then(function () { toast(m[1] + ' requested', 'warn', 'system'); },
									function (e) { errToast(e, 'reboot'); });
						});
					});
					mrow.appendChild(b);
				}(modes[i]));
			}
			add(actB, [rows, el('div', { cls: 'sep' }),
				el('label', { cls: 'fl', text: 'System' }), mrow]);
		}

		function renderUpload() {
			clear(upB);
			if (!admin) {
				upB.appendChild(empty('Admin role required to upload firmware.'));
				return;
			}
			var file = el('input', { type: 'file', accept: '.bin,.signed.bin' });
			var shaOut = el('div', { cls: 'fpr' });
			var prog = el('div', { cls: 'prog' }, el('i'));
			var stat = el('div', { cls: 'hint', text: 'select a signed image' });
			var go = el('button', { cls: 'btn primary sm', type: 'button', text: 'Upload', disabled: true });
			var stopBtn = el('button', { cls: 'btn sm', type: 'button', text: 'Abort', disabled: true });
			var chosen = null;

			file.addEventListener('change', function () {
				chosen = (file.files && file.files[0]) || null;
				shaOut.textContent = '';
				prog.firstChild.style.width = '0%';
				if (!chosen) {
					go.disabled = true;
					stat.textContent = 'select a signed image';
					return;
				}
				stat.textContent = chosen.name + '  ' + fbytes(chosen.size) + '  hashing…';
				go.disabled = true;
				chosen.arrayBuffer().then(function (buf) {
					return crypto.subtle.digest('SHA-256', buf).then(function (h) {
						chosen._buf = buf;
						chosen._sha = hexOf(new Uint8Array(h));
						shaOut.textContent = 'SHA-256 ' + chosen._sha;
						stat.textContent = chosen.name + '  ' + fbytes(chosen.size) + '  ready';
						go.disabled = false;
					});
				}).catch(function (e) {
					stat.textContent = 'could not hash the file: ' + (e && e.message ? e.message : e);
					toast('crypto.subtle.digest failed - the page must be served over HTTPS',
						'err', 'upload');
				});
			});

			function setProg(f, msg) {
				prog.firstChild.style.width = (Math.max(0, Math.min(1, f)) * 100).toFixed(1) + '%';
				if (msg) { stat.textContent = msg; }
			}

			go.addEventListener('click', function () {
				if (!chosen || !chosen._buf || busy) { return; }
				confirmDlg({
					title: 'Upload firmware',
					text: 'Write ' + fbytes(chosen.size) + ' into the staging slot? The image is ' +
						'verified on the device and boots on trial after the next restart.',
					okText: 'Upload', okClass: 'primary'
				}).then(function (ok) {
					if (!ok) { return; }
					busy = true;
					cancel = false;
					go.disabled = true;
					file.disabled = true;
					stopBtn.disabled = false;
					runUpload(chosen).then(function () {
						setProg(1, 'upload complete; image staged and pending a trial boot');
						toast('image staged', 'ok', 'firmware');
					}, function (e) {
						if (e && e.code === 'aborted') {
							stat.textContent = 'aborted';
							toast('upload aborted', 'warn', 'firmware');
						} else {
							stat.textContent = 'failed: ' + (e && e.code ? e.code : e);
							errToast(e, 'upload');
						}
					}).then(function () {
						busy = false;
						go.disabled = false;
						file.disabled = false;
						stopBtn.disabled = true;
						loadInfo();
					});
				});
			});

			stopBtn.addEventListener('click', function () { cancel = true; });

			function runUpload(f) {
				var buf = f._buf;
				var total = buf.byteLength;
				var u = (info && info.upload) || {};
				var wb = isNum(u.write_block) && u.write_block > 0 ? u.write_block : 1;
				var cmax = isNum(u.chunk_max) && u.chunk_max > 0 ? u.chunk_max : 1024;
				/* Every non-final chunk must be a whole number of write blocks. */
				var step = Math.max(wb, Math.floor(cmax / wb) * wb);
				var off = 0;
				var stalls = 0;

				setProg(0, 'begin…');
				return req('POST', 'firmware/begin', {
					json: { size: total, sha256: f._sha }
				}).then(function (b) {
					off = (b && isNum(b.next_offset)) ? b.next_offset : 0;

					function chunk() {
						if (cancel) { throw new ApiError('aborted', 0, ''); }
						if (off >= total) { return null; }
						var end = Math.min(off + step, total);
						if (end < total) {
							/* keep the length block-aligned for non-final writes */
							var len = Math.floor((end - off) / wb) * wb;
							if (len < wb) { len = wb; }
							end = Math.min(off + len, total);
						}
						var slice = buf.slice(off, end);
						return req('POST', 'firmware/data?offset=' + off, {
							body: slice, type: 'application/octet-stream'
						}).then(function (r) {
							var nx = (r && isNum(r.next_offset)) ? r.next_offset : end;
							if (nx <= off) {
								if (++stalls > 5) {
									throw new ApiError('upload_stalled', 0,
										'the device stopped advancing at offset ' + off);
								}
							} else { stalls = 0; }
							off = nx;
							setProg(off / total, 'writing ' + fbytes(off) + ' of ' + fbytes(total));
							return chunk();
						}, function (e) {
							if (e.code === 'offset_gap' && isNum(e.extra.next_offset)) {
								/* the device dictates where to resume */
								if (++stalls > 8) { throw e; }
								off = e.extra.next_offset;
								setProg(off / total, 'rewound to ' + fbytes(off));
								return chunk();
							}
							throw e;
						});
					}
					return chunk();
				}).then(function () {
					setProg(1, 'verifying on the device…');
					return req('POST', 'firmware/end');
				});
			}

			add(upB, [
				el('div', { cls: 'frow' }, [file, go, stopBtn]),
				el('div', { style: 'margin-top:10px' }, prog),
				el('div', { style: 'margin-top:7px' }, stat),
				shaOut,
				el('div', { cls: 'hint', style: 'margin-top:10px' },
					['The SHA-256 is computed in the browser with ', code('crypto.subtle'),
						' and sent with ', code('/firmware/begin'), '. Chunks are ',
						code('upload.chunk_max'), ' bytes, block-aligned to ',
						code('upload.write_block'), '; a ', code('offset_gap'),
						' response rewinds to the offset the device reports.'])
			]);
		}

		return {
			el: root,
			init: loadInfo
		};
	}

	/* ===================================================================== */
	/* router                                                                */
	/* ===================================================================== */

	var PAGES = {
		dashboard: pageDashboard,
		timing: pageTiming,
		calibration: pageCalibration,
		gnss: pageGnss,
		network: pageNetwork,
		security: pageSecurity,
		power: pagePower,
		logs: pageLogs,
		firmware: pageFirmware
	};

	function route() {
		var h = location.hash.replace(/^#\/?/, '');
		var name = h.split(/[/?]/)[0] || 'dashboard';
		if (!Object.prototype.hasOwnProperty.call(PAGES, name)) { name = 'dashboard'; }

		if (Page.cur && Page.cur.destroy) {
			try { Page.cur.destroy(); } catch (e) { /* page-local */ }
		}
		Page.cur = null;
		clearDraws();
		Tip.hide();
		Live.setExtra({});

		var links = $('navlinks').querySelectorAll('a');
		for (var i = 0; i < links.length; i++) {
			links[i].classList.toggle('on', links[i].getAttribute('data-page') === name);
		}

		var view = $('view');
		clear(view);
		var p = PAGES[name]();
		Page.cur = p;
		view.appendChild(p.el);
		view.scrollTop = 0;
		if (p.update) { try { p.update(); } catch (e) { /* first paint may lack data */ } }
		if (p.init) { try { p.init(); } catch (e) { errToast(e, 'page'); } }
		redrawAll();
	}

	/* ===================================================================== */
	/* boot                                                                  */
	/* ===================================================================== */

	function startApp() {
		$('app').hidden = false;
		Session.hideLogin();
		paintRole();
		paintHeader();
		Cfg.load(true).catch(function () { /* pages retry and report */ });
		Live.start();
		if (!location.hash) { location.hash = '#/dashboard'; }
		route();
	}

	function wireChrome() {
		window.addEventListener('hashchange', route);

		$('btn-logout').addEventListener('click', function () { Session.logout(); });

		$('sb-commit').addEventListener('click', function () {
			var b = $('sb-commit');
			b.disabled = true;
			Cfg.commit().then(function (r) {
				var msg = (r && isNum(r.applied) ? r.applied : '?') + ' key(s) applied';
				if (r && r.reboot_keys) {
					msg += '; ' + r.reboot_keys + ' need a reboot to take effect';
				}
				if (r && r.persist_errors) {
					msg += '; ' + r.persist_errors + ' failed to persist';
				}
				toast(msg, (r && r.persist_errors) ? 'warn' : 'ok', 'committed');
				paintStage();
				if (Page.cur && Page.cur.init) { Page.cur.init(); }
			}, function (e) {
				if (e.code === 'validation_failed') {
					toast('Validation failed; the set is still staged. Fix the offending key ' +
						'and commit again.', 'err', 'commit rejected');
				} else { errToast(e, 'commit'); }
			}).then(function () { paintStage(); });
		});

		$('sb-revert').addEventListener('click', function () {
			confirmDlg({
				title: 'Discard staged changes',
				text: 'Drop every staged configuration change without applying it?',
				okText: 'Discard'
			}).then(function (ok) {
				if (!ok) { return; }
				return Cfg.revert().then(function (r) {
					toast((r && isNum(r.dropped) ? r.dropped : 0) + ' staged change(s) dropped',
						'ok', 'reverted');
					paintStage();
					if (Page.cur && Page.cur.init) { Page.cur.init(); }
				}, function (e) { errToast(e, 'revert'); });
			});
		});

		$('login-form').addEventListener('submit', function (ev) {
			ev.preventDefault();
			var btn = $('lg-go'), msg = $('lg-msg');
			var u = $('lg-user').value, p = $('lg-pass').value;
			btn.disabled = true;
			msg.className = 'lg-msg';
			msg.textContent = '';
			Session.login(u, p).then(function () {
				startApp();
			}, function (e) {
				if (e.code === 'invalid_credentials') {
					msg.textContent = 'Invalid credentials.';
				} else if (e.code === 'throttled') {
					msg.textContent = 'Too many attempts. Retry in ' +
						(e.extra.retry_after || 'a moment') + ' s.';
				} else if (e.code === 'no_credential') {
					msg.className = 'lg-msg info';
					msg.textContent = 'No admin password is provisioned yet. Set one over the ' +
						'USB console or the local front-panel UI, then sign in here.';
				} else if (e.code === 'network_error') {
					msg.textContent = 'Cannot reach the device.';
				} else {
					msg.textContent = 'Sign-in failed: ' + e.code;
				}
			}).then(function () { btn.disabled = false; });
		});
	}

	function boot() {
		wireChrome();
		Session.refresh().then(function (s) {
			if (s && (s.authenticated || s.auth_required === false)) { startApp(); }
			else { Session.requireLogin(''); }
		}, function (e) {
			Session.requireLogin(e && e.code === 'network_error'
				? 'Cannot reach the device.' : '');
		});
	}

	if (document.readyState === 'loading') {
		document.addEventListener('DOMContentLoaded', boot);
	} else { boot(); }
}());
