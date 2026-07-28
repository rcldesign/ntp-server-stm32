/*
 * STS1000 "Meridian" — what a factory reset must do, on every plane.
 *
 * sts_factory_policy.h exists because the defect it records was not in any one
 * implementation. Each of the three planes — the web UI, the MCP console
 * command and the front panel — is a short, obvious function, and the panel's
 * was obviously wrong only next to the other two: it cleared the config tree and
 * left the persisted TLS server key, its certificate, the ACME account key and
 * every outstanding web session in place, then reported a clean unit. The defect
 * lived in the *set*, which nothing described and nothing checked.
 *
 * So this suite has two halves, and it needs both:
 *
 *   THE ARITHMETIC. sts_factory_result() / _wipe_skipped() / _should_reboot()
 *   over their whole input domain — nine states, not a sample. In particular
 *   sts_factory_should_reboot() takes the outcome and ignores it, and that is
 *   the invariant, not an oversight: an operator who asked for a factory reset
 *   and got neither a wipe nor a reboot holds a unit in an undefined half-reset
 *   state with nothing to force it out. The test below asserts the function is
 *   *constant* over the entire domain, so reintroducing the conditional cannot
 *   pass by accident.
 *
 *   THE PLANES. The arithmetic is only worth anything if all three planes still
 *   run both halves and feed it honestly, and no C construct can express that.
 *   The second half therefore reads the three sources, strips comments and
 *   string literals (so a mention in prose cannot satisfy a check — sts_mcp.c
 *   describes both calls in a comment before making either), brace-matches each
 *   plane's body out of the file, and asserts inside it:
 *
 *     both halves run;
 *     the config reset comes first, because it is what runs the appliers that
 *       push the emptied credential keys into the subsystems holding RAM copies;
 *     nothing returns or jumps between them, because short-circuiting on the
 *       first failure is how a half-wiped unit gets reported as a clean one;
 *     nothing GATES the key wipe except the weak symbol's linkage — a
 *       short-circuit spelled `if (st.wipe_linked && st.cfg_reset_ok)` skips
 *       the wipe with no jump anywhere for a span scan to find, so the clauses
 *       the call is nested under are read and required to name linkage and
 *       nothing else;
 *     the reboot is not conditional on the outcome.
 *
 * This is an architecture-fitness test in the same sense as
 * test_smear_isolation.c, and like that one it is written so it cannot pass
 * vacuously: it asserts that the scan positively found each plane, that the
 * comment stripper actually stripped something, and that each body it matched
 * is a plausible size.
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "zephyr/ui/sts_factory_policy.h"

#ifndef STS_APP_SRC_DIR
#error "STS_APP_SRC_DIR must be defined by tests/host/CMakeLists.txt"
#endif

/* ===================================================================== */
/* Part 1 — the arithmetic                                               */
/* ===================================================================== */

/** The eight (cfg_reset_ok, wipe_linked, wipe_ok) states, as a bit triple. */
static sts_factory_steps_t steps_of(unsigned int bits)
{
	sts_factory_steps_t s;

	s.cfg_reset_ok = (bits & 1U) != 0U;
	s.wipe_linked = (bits & 2U) != 0U;
	s.wipe_ok = (bits & 4U) != 0U;
	return s;
}

/** What the header says the result must be, restated independently. */
static int want_result(const sts_factory_steps_t *s)
{
	if (!s->cfg_reset_ok) {
		return -EIO;
	}
	if (s->wipe_linked && !s->wipe_ok) {
		return -EIO;
	}
	return 0;
}

static void test_the_result_over_the_whole_domain(void)
{
	unsigned int bits;

	for (bits = 0U; bits < 8U; bits++) {
		sts_factory_steps_t s = steps_of(bits);
		char msg[96];

		(void)snprintf(msg, sizeof(msg),
			       "cfg_reset_ok=%d wipe_linked=%d wipe_ok=%d",
			       (int)s.cfg_reset_ok, (int)s.wipe_linked,
			       (int)s.wipe_ok);
		TEST_ASSERT_EQUAL_INT_MESSAGE(want_result(&s),
					      sts_factory_result(&s), msg);
	}

	/* A NULL step record is not "nothing happened"; it is a plane that
	 * cannot say what it did, which must never read as factory-clean. */
	TEST_ASSERT_EQUAL_INT(-EIO, sts_factory_result(NULL));
}

/**
 * Failures accumulate: step 2 runs even when step 1 failed, and either failure
 * is enough to make the whole thing -EIO.
 *
 * The dangerous direction is the one where a successful second half masks a
 * failed first: a unit that erased its TLS identity but could not clear its
 * config tree is not decommissioned, and reporting 0 is how it gets shipped as
 * if it were.
 */
static void test_a_failure_in_either_half_is_reported(void)
{
	sts_factory_steps_t s;

	/* Config reset failed, wipe succeeded — still not clean. */
	s.cfg_reset_ok = false;
	s.wipe_linked = true;
	s.wipe_ok = true;
	TEST_ASSERT_EQUAL_INT(-EIO, sts_factory_result(&s));

	/* Config reset succeeded, wipe failed — still not clean. */
	s.cfg_reset_ok = true;
	s.wipe_ok = false;
	TEST_ASSERT_EQUAL_INT(-EIO, sts_factory_result(&s));

	/* Both failed. */
	s.cfg_reset_ok = false;
	TEST_ASSERT_EQUAL_INT(-EIO, sts_factory_result(&s));

	/* Both succeeded — the only clean state with a wipe linked. */
	s.cfg_reset_ok = true;
	s.wipe_ok = true;
	TEST_ASSERT_EQUAL_INT(0, sts_factory_result(&s));
}

/**
 * An absent wipe is not a failure, and `wipe_ok` carries no meaning when the
 * wipe is not linked.
 *
 * With CONFIG_STS1000_NET=n there is no TLS identity to erase and the weak
 * symbol resolves to NULL. That reset is honest and complete for the image it
 * ran in, so it returns 0 — but the plane still owes the operator the
 * annunciation, because "config-only" and "complete" are different states of
 * the unit.
 */
static void test_an_absent_wipe_is_honest_rather_than_failed(void)
{
	sts_factory_steps_t s;

	s.cfg_reset_ok = true;
	s.wipe_linked = false;

	/* Both readings of the meaningless field give the same answer. */
	s.wipe_ok = false;
	TEST_ASSERT_EQUAL_INT(0, sts_factory_result(&s));
	TEST_ASSERT_TRUE(sts_factory_wipe_skipped(&s));
	s.wipe_ok = true;
	TEST_ASSERT_EQUAL_INT(0, sts_factory_result(&s));
	TEST_ASSERT_TRUE(sts_factory_wipe_skipped(&s));

	/* …but a failed config reset is still a failure in that image. */
	s.cfg_reset_ok = false;
	TEST_ASSERT_EQUAL_INT(-EIO, sts_factory_result(&s));
	TEST_ASSERT_TRUE(sts_factory_wipe_skipped(&s));
}

static void test_wipe_skipped_tracks_only_the_linkage(void)
{
	unsigned int bits;

	for (bits = 0U; bits < 8U; bits++) {
		sts_factory_steps_t s = steps_of(bits);
		char msg[96];

		(void)snprintf(msg, sizeof(msg),
			       "cfg_reset_ok=%d wipe_linked=%d wipe_ok=%d",
			       (int)s.cfg_reset_ok, (int)s.wipe_linked,
			       (int)s.wipe_ok);
		TEST_ASSERT_EQUAL_INT_MESSAGE((int)!s.wipe_linked,
					      (int)sts_factory_wipe_skipped(&s),
					      msg);
	}

	/* A NULL record says nothing about linkage, so it is not a claim that
	 * the wipe was skipped — sts_factory_result() has already called it
	 * -EIO, which is the stronger and correct statement. */
	TEST_ASSERT_FALSE(sts_factory_wipe_skipped(NULL));
}

/**
 * THE REBOOT IS UNCONDITIONAL — asserted as constancy, not as nine true's.
 *
 * sts_factory_should_reboot() takes the outcome and ignores it on purpose: the
 * invariant is "the reboot does not depend on the outcome", and a function that
 * could not see the outcome could not be shown to honour it. So the assertion is
 * that the value is the same for every input in the domain *and* that the value
 * is true — the pair of which no conditional implementation can satisfy.
 */
static void test_the_reboot_never_depends_on_the_outcome(void)
{
	bool first = sts_factory_should_reboot(NULL);
	unsigned int bits;

	TEST_ASSERT_TRUE_MESSAGE(first,
				 "a factory reset must reboot even when it "
				 "could not say what it managed to erase");

	for (bits = 0U; bits < 8U; bits++) {
		sts_factory_steps_t s = steps_of(bits);
		char msg[128];

		(void)snprintf(msg, sizeof(msg),
			       "the reboot became conditional: cfg_reset_ok=%d "
			       "wipe_linked=%d wipe_ok=%d",
			       (int)s.cfg_reset_ok, (int)s.wipe_linked,
			       (int)s.wipe_ok);
		TEST_ASSERT_EQUAL_INT_MESSAGE((int)first,
					      (int)sts_factory_should_reboot(&s),
					      msg);
	}

	/* Spelled out for the two states that tempt a conditional most: the
	 * wipe failed, and the wipe never ran. Both are exactly when the unit
	 * most needs the reboot that clears RAM-only key material. */
	{
		sts_factory_steps_t s;

		s.cfg_reset_ok = true;
		s.wipe_linked = true;
		s.wipe_ok = false;
		TEST_ASSERT_TRUE(sts_factory_should_reboot(&s));
		TEST_ASSERT_EQUAL_INT(-EIO, sts_factory_result(&s));

		s.wipe_linked = false;
		TEST_ASSERT_TRUE(sts_factory_should_reboot(&s));
	}
}

/* ===================================================================== */
/* Part 2 — the planes                                                   */
/* ===================================================================== */

#define SRC_MAX (256U * 1024U)

static char g_raw[SRC_MAX];
static char g_code[SRC_MAX]; /* g_raw with comments and literals blanked */
static size_t g_len;
static size_t g_stripped; /* characters the stripper blanked */

/** Read @p rel (relative to app/src) into g_raw, and blank it into g_code. */
static void load(const char *rel);

static void read_source(const char *rel)
{
	char path[512];
	FILE *f;
	size_t n;

	(void)snprintf(path, sizeof(path), "%s/%s", STS_APP_SRC_DIR, rel);
	f = fopen(path, "rb");
	TEST_ASSERT_NOT_NULL_MESSAGE(f, path);
	n = fread(g_raw, 1U, sizeof(g_raw), f);
	/* A file that exactly filled the buffer was probably truncated. */
	TEST_ASSERT_TRUE_MESSAGE(n < sizeof(g_raw), "raise SRC_MAX");
	(void)fclose(f);
	g_len = n;
	g_raw[n] = '\0';
}

/**
 * Blank every comment, string literal and character literal, in place, one
 * space per character and newlines preserved.
 *
 * Byte offsets therefore stay 1:1 with the file, so a failure can report a real
 * line number. Blanking rather than deleting is also what makes the checks below
 * honest: sts_mcp.c names both halves in a comment above the function that calls
 * them, and sts_ui.c's own comment quotes the other two planes' function names.
 * A scanner that counted those would pass on a plane that had stopped doing the
 * work and only kept describing it.
 */
static void strip(void)
{
	enum { CODE, BLOCK, LINE, STR, CHR } st = CODE;
	size_t i;

	g_stripped = 0U;
	for (i = 0U; i < g_len; i++) {
		char c = g_raw[i];
		char nx = (i + 1U < g_len) ? g_raw[i + 1U] : '\0';
		bool blank = (st != CODE);

		switch (st) {
		case CODE:
			if ((c == '/') && (nx == '*')) {
				st = BLOCK;
				blank = true;
			} else if ((c == '/') && (nx == '/')) {
				st = LINE;
				blank = true;
			} else if (c == '"') {
				st = STR;
				blank = true;
			} else if (c == '\'') {
				st = CHR;
				blank = true;
			}
			break;
		case BLOCK:
			if ((c == '*') && (nx == '/')) {
				g_code[i] = ' ';
				g_stripped++;
				i++;
				g_code[i] = ' ';
				g_stripped++;
				st = CODE;
				continue;
			}
			break;
		case LINE:
			/* A backslash-newline continues a // comment. */
			if (c == '\n') {
				st = CODE;
				blank = false;
			}
			break;
		case STR:
		case CHR:
			if (c == '\\') {
				g_code[i] = ' ';
				g_stripped++;
				if (i + 1U < g_len) {
					i++;
					g_code[i] = ' ';
					g_stripped++;
				}
				continue;
			}
			if (((st == STR) && (c == '"')) ||
			    ((st == CHR) && (c == '\''))) {
				st = CODE;
			}
			break;
		default:
			break;
		}

		if (blank && (c != '\n')) {
			g_code[i] = ' ';
			g_stripped++;
		} else {
			g_code[i] = c;
		}
	}
	g_code[g_len] = '\0';
}

static void load(const char *rel)
{
	read_source(rel);
	strip();
}

/** 1-based line number of byte offset @p off in the loaded file. */
static unsigned int line_of(size_t off)
{
	unsigned int line = 1U;
	size_t i;

	for (i = 0U; (i < off) && (i < g_len); i++) {
		if (g_code[i] == '\n') {
			line++;
		}
	}
	return line;
}

static bool ident_char(char c)
{
	return ((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z')) ||
	       ((c >= '0') && (c <= '9')) || (c == '_');
}

static bool space_char(char c)
{
	return (c == ' ') || (c == '\t') || (c == '\n') || (c == '\r');
}

/**
 * Last non-space offset at or before @p i, or SIZE_MAX when there is none.
 *
 * @p i must be a real offset; SIZE_MAX is the caller's "nothing to the left".
 */
static size_t skip_space_back(size_t i)
{
	for (;;) {
		if (!space_char(g_code[i])) {
			return i;
		}
		if (i == 0U) {
			return SIZE_MAX;
		}
		i--;
	}
}

/** Start of the identifier ending at @p last, or false when that is not one. */
static bool ident_ending_at(size_t last, size_t *out_begin)
{
	size_t b = last;

	if (!ident_char(g_code[last])) {
		return false;
	}
	while ((b > 0U) && ident_char(g_code[b - 1U])) {
		b--;
	}
	*out_begin = b;
	return true;
}

/** True when g_code[begin, end) is exactly @p word. */
static bool span_is(size_t begin, size_t end, const char *word)
{
	size_t n = strlen(word);

	return ((end - begin) == n) && (memcmp(&g_code[begin], word, n) == 0);
}

/**
 * Offset of the first whole-token occurrence of @p tok in
 * g_code[from, to), or SIZE_MAX.
 */
static size_t find_token(const char *tok, size_t from, size_t to)
{
	size_t n = strlen(tok);
	size_t i;

	if (to > g_len) {
		to = g_len;
	}
	for (i = from; (i + n) <= to; i++) {
		if (memcmp(&g_code[i], tok, n) != 0) {
			continue;
		}
		if ((i > 0U) && ident_char(g_code[i - 1U])) {
			continue;
		}
		if (((i + n) < g_len) && ident_char(g_code[i + n])) {
			continue;
		}
		return i;
	}
	return SIZE_MAX;
}

/** As find_token(), but only where the token is immediately applied as a call. */
static size_t find_call(const char *fn, size_t from, size_t to)
{
	size_t at = from;

	for (;;) {
		size_t j;

		at = find_token(fn, at, to);
		if (at == SIZE_MAX) {
			return SIZE_MAX;
		}
		j = at + strlen(fn);
		while ((j < g_len) && ((g_code[j] == ' ') ||
				       (g_code[j] == '\t') ||
				       (g_code[j] == '\n'))) {
			j++;
		}
		if ((j < g_len) && (g_code[j] == '(')) {
			return at;
		}
		at++;
	}
}

/** A brace-matched body: [open+1, close). */
typedef struct {
	size_t begin;
	size_t end;
} body_t;

/**
 * The block that follows @p anchor: from the first '{' after it to its match.
 *
 * Safe on the stripped text because every brace inside a comment or a string
 * literal has already been blanked out.
 */
static body_t body_after(const char *anchor)
{
	size_t at = find_token(anchor, 0U, g_len);
	body_t b = { 0U, 0U };
	size_t i;
	unsigned int depth = 0U;

	TEST_ASSERT_TRUE_MESSAGE(at != SIZE_MAX, anchor);

	for (i = at; i < g_len; i++) {
		if (g_code[i] == '{') {
			break;
		}
	}
	TEST_ASSERT_TRUE_MESSAGE(i < g_len, anchor);

	b.begin = i + 1U;
	for (; i < g_len; i++) {
		if (g_code[i] == '{') {
			depth++;
		} else if (g_code[i] == '}') {
			depth--;
			if (depth == 0U) {
				b.end = i;
				return b;
			}
		}
	}
	TEST_FAIL_MESSAGE(anchor);
	return b;
}

/** Brace depth of @p off relative to the start of @p b (1 = directly in it). */
static unsigned int depth_at(const body_t *b, size_t off)
{
	unsigned int depth = 1U;
	size_t i;

	for (i = b->begin; (i < off) && (i < b->end); i++) {
		if (g_code[i] == '{') {
			depth++;
		} else if (g_code[i] == '}') {
			depth--;
		}
	}
	return depth;
}

/* ---------------------------------------------------------- guard reading */

/**
 * What controls a block or a statement.
 *
 * `readable` spans the controlling expression, parentheses excluded. `opaque`
 * means the thing IS controlled but by something written elsewhere (`else`, the
 * tail of a `do`), which must never be reported as "controlled by nothing" —
 * the safe direction for a scan whose whole job is to refuse conditions it
 * cannot account for.
 */
typedef struct {
	bool readable;
	bool opaque;
	size_t begin;
	size_t end;
} cond_t;

/** The '(' matching the ')' at @p close, or SIZE_MAX when there is none. */
static size_t match_paren_back(size_t close)
{
	unsigned int depth = 0U;
	size_t i = close;

	for (;;) {
		if (g_code[i] == ')') {
			depth++;
		} else if (g_code[i] == '(') {
			depth--;
			if (depth == 0U) {
				return i;
			}
		}
		if (i == 0U) {
			return SIZE_MAX;
		}
		i--;
	}
}

/**
 * The clause controlling whatever begins at @p at.
 *
 * @p at is the offset of a '{' or of the first token of a statement. The scan
 * runs backwards: whitespace, then a ')' matched back to its '(', then the
 * keyword owning it. `if`/`while`/`for`/`switch` give a readable span; `else`
 * and `do` give `opaque`; anything else — a plain block, an initialiser's
 * `= {`, the start of a body, a call's own argument list — gives neither, i.e.
 * "nothing controls this".
 *
 * Safe on the stripped text: every parenthesis and brace inside a comment or a
 * string literal has already been blanked, so the matching cannot walk into one.
 */
static cond_t cond_of(size_t at)
{
	cond_t c = { false, false, 0U, 0U };
	size_t close;
	size_t open;
	size_t i;
	size_t b;

	if (at == 0U) {
		return c;
	}
	i = skip_space_back(at - 1U);
	if (i == SIZE_MAX) {
		return c;
	}

	if (g_code[i] != ')') {
		if (ident_ending_at(i, &b) &&
		    (span_is(b, i + 1U, "else") || span_is(b, i + 1U, "do"))) {
			c.opaque = true;
		}
		return c;
	}

	close = i;
	open = match_paren_back(close);
	if ((open == SIZE_MAX) || (open == 0U)) {
		/* Unbalanced, or the file starts with it: report control rather
		 * than freedom from it. */
		c.opaque = true;
		return c;
	}

	i = skip_space_back(open - 1U);
	if ((i == SIZE_MAX) || !ident_ending_at(i, &b)) {
		return c; /* a call's argument list, not a control clause */
	}
	if (span_is(b, i + 1U, "if") || span_is(b, i + 1U, "while") ||
	    span_is(b, i + 1U, "for") || span_is(b, i + 1U, "switch")) {
		c.readable = true;
		c.begin = open + 1U;
		c.end = close;
	}
	return c;
}

#define GUARD_MAX 8U

/**
 * Every clause the statement at @p off is nested under, inside @p b.
 *
 * Two sources, because a condition does not need braces to be one:
 *
 *   - the enclosing blocks, found by walking the body with a brace stack;
 *   - a braceless clause applied directly to the statement, e.g.
 *     `if (x) sts_sec_factory_wipe();`, which no brace stack can see.
 *
 * @return the number written to @p out, innermost first.
 */
static size_t guards_of(const body_t *b, size_t off, cond_t *out, size_t max)
{
	size_t stack[64];
	size_t depth = 0U;
	size_t n = 0U;
	size_t i;
	size_t cursor;

	for (i = b->begin; (i < off) && (i < b->end); i++) {
		if (g_code[i] == '{') {
			TEST_ASSERT_TRUE_MESSAGE(depth < (sizeof(stack) /
							  sizeof(stack[0])),
						 "brace nesting past the scan's "
						 "stack");
			stack[depth] = i;
			depth++;
		} else if (g_code[i] == '}') {
			TEST_ASSERT_TRUE_MESSAGE(depth > 0U,
						 "a '}' with no '{' — the brace "
						 "walk lost sync");
			depth--;
		}
	}

	/*
	 * Braceless clauses, innermost first. A condition needs no braces to be
	 * one, and `if (x) rc = sts_sec_factory_wipe();` is invisible to the
	 * brace stack above.
	 *
	 * Walk left from the call through the statement's own text — the call
	 * need not be its first token — and stop at whatever introduces it. A
	 * real statement boundary (';' '{' '}' ':') means nothing braceless
	 * gates it; a ')' is either a control clause, in which case it does and
	 * the walk continues left of it (`if (a) if (b) call();` is two), or
	 * some other call's argument list, which is stepped over.
	 */
	cursor = off;
	while ((n < max) && (cursor > b->begin)) {
		size_t j = cursor - 1U;
		bool boundary = false;
		cond_t c;

		for (;;) {
			char ch = g_code[j];

			if ((ch == ';') || (ch == '{') || (ch == '}') ||
			    (ch == ':')) {
				boundary = true;
				break;
			}
			if (ch == ')') {
				break;
			}
			if (j <= b->begin) {
				boundary = true;
				break;
			}
			j--;
		}
		if (boundary || (g_code[j] != ')')) {
			break;
		}

		c = cond_of(j + 1U);
		if (!c.readable && !c.opaque) {
			/* Another call's ')': step over it and keep looking. */
			size_t open = match_paren_back(j);

			if ((open == SIZE_MAX) || (open <= b->begin)) {
				break;
			}
			cursor = open;
			continue;
		}
		out[n] = c;
		n++;
		if (!c.readable || (c.begin <= b->begin)) {
			break;
		}
		cursor = c.begin - 1U; /* the '(' — keep walking left */
	}

	/* …then the enclosing blocks, innermost first. */
	while ((n < max) && (depth > 0U)) {
		cond_t c;

		depth--;
		c = cond_of(stack[depth]);
		if (c.readable || c.opaque) {
			out[n] = c;
			n++;
		}
	}
	return n;
}

/* The three planes, each named by the source and the anchor that opens it. */
static const struct {
	const char *what;
	const char *file;
	const char *anchor;
} g_planes[] = {
	{ "web", "zephyr/net/sts_web.c", "pv_factory_reset" },
	{ "mcp", "zephyr/console/sts_mcp.c", "mcp_cfg_factory_reset" },
	{ "panel", "zephyr/ui/sts_ui.c", "UI_ACTION_FACTORY_RESET" },
};

#define PLANE_N (sizeof(g_planes) / sizeof(g_planes[0]))

#define CFG_RESET_FN "sts_cfg_factory_reset"
#define SEC_WIPE_FN  "sts_sec_factory_wipe"

/**
 * The only identifiers a guard around the key wipe may mention.
 *
 * The call IS legitimately nested — the symbol is weak, so an image built with
 * CONFIG_STS1000_NET=n resolves it to NULL and both service planes have to ask
 * before calling. That is a question about LINKAGE, and linkage is the only
 * thing the guard is allowed to be about. The moment the condition mentions
 * anything else it has become a short-circuit wearing an `if`: gate the wipe on
 * `st.cfg_reset_ok` and a unit whose config reset failed keeps its TLS identity
 * and every web session, which is the exact defect sts_factory_policy.h exists
 * to record.
 *
 * Member references contribute the MEMBER, not the object: `st.wipe_linked`
 * yields `wipe_linked`. Which record a plane keeps is its own business; which
 * field it consults is not.
 */
static const char *const g_wipe_guard_ok[] = {
	SEC_WIPE_FN,   /* `sts_sec_factory_wipe != NULL` — sts_mcp.c */
	"wipe_linked", /* `st.wipe_linked`              — sts_ui.c  */
	"NULL",
};

#define WIPE_GUARD_OK_N (sizeof(g_wipe_guard_ok) / sizeof(g_wipe_guard_ok[0]))

static bool wipe_guard_ident_ok(size_t begin, size_t end)
{
	size_t i;

	for (i = 0U; i < WIPE_GUARD_OK_N; i++) {
		if (span_is(begin, end, g_wipe_guard_ok[i])) {
			return true;
		}
	}
	return false;
}

/**
 * Assert that the clause g_code[begin, end) mentions nothing but linkage.
 *
 * Numeric literals are skipped (`ident_char()` admits digits, so `0` tokenises
 * as one) and so is the object of a member reference, per g_wipe_guard_ok.
 */
static void assert_guard_is_linkage_only(const char *what, const char *file,
					 size_t begin, size_t end)
{
	size_t i = begin;

	while (i < end) {
		size_t j;
		char msg[256];

		if (!ident_char(g_code[i])) {
			i++;
			continue;
		}
		j = i;
		while ((j < end) && ident_char(g_code[j])) {
			j++;
		}
		/* A numeric literal, not a name. */
		if ((g_code[i] >= '0') && (g_code[i] <= '9')) {
			i = j;
			continue;
		}
		/* The object of a member reference: `st` in `st.wipe_linked`. */
		if ((j < end) && ((g_code[j] == '.') ||
				  ((g_code[j] == '-') && ((j + 1U) < end) &&
				   (g_code[j + 1U] == '>')))) {
			i = j;
			continue;
		}
		(void)snprintf(msg, sizeof(msg),
			       "%s plane (%s): the " SEC_WIPE_FN "() call at "
			       "line %u is gated on `%.*s`. The only thing that "
			       "may gate it is whether the weak symbol is linked "
			       "— any other term makes the key wipe conditional "
			       "on the config reset, and failures must accumulate",
			       what, file, line_of(begin), (int)(j - i),
			       &g_code[i]);
		TEST_ASSERT_TRUE_MESSAGE(wipe_guard_ident_ok(i, j), msg);
		i = j;
	}
}

/**
 * The scan really scanned, and the stripper really stripped.
 *
 * Everything below is meaningless without this: a stripper that blanked the
 * whole file, or a matcher that returned an empty body, would report three
 * beautifully compliant planes.
 */
static void test_the_scan_is_not_vacuous(void)
{
	size_t i;

	for (i = 0U; i < PLANE_N; i++) {
		body_t b;
		char msg[192];

		load(g_planes[i].file);

		(void)snprintf(msg, sizeof(msg), "%s (%s) is implausibly small",
			       g_planes[i].file, g_planes[i].what);
		TEST_ASSERT_TRUE_MESSAGE(g_len > 4096U, msg);

		/* Every one of these files is heavily commented; a stripper
		 * that blanked nothing is broken. */
		(void)snprintf(msg, sizeof(msg),
			       "%s: the comment stripper removed nothing",
			       g_planes[i].file);
		TEST_ASSERT_TRUE_MESSAGE(g_stripped > 512U, msg);

		/* …and it did not blank the code with it. */
		(void)snprintf(msg, sizeof(msg),
			       "%s: the comment stripper removed everything",
			       g_planes[i].file);
		TEST_ASSERT_TRUE_MESSAGE(g_stripped < ((g_len * 9U) / 10U), msg);

		b = body_after(g_planes[i].anchor);
		(void)snprintf(msg, sizeof(msg),
			       "%s: %s body is implausibly small",
			       g_planes[i].file, g_planes[i].anchor);
		TEST_ASSERT_TRUE_MESSAGE((b.end - b.begin) > 64U, msg);
	}

	/*
	 * And the stripping is load-bearing rather than decorative: sts_mcp.c
	 * names both halves in the comment above the function that calls them,
	 * so the raw text has strictly more mentions than the code does.
	 */
	load("zephyr/console/sts_mcp.c");
	{
		size_t code_hits = 0U;
		size_t raw_hits = 0U;
		size_t at = 0U;
		size_t i2;

		for (;;) {
			at = find_token(CFG_RESET_FN, at, g_len);
			if (at == SIZE_MAX) {
				break;
			}
			code_hits++;
			at++;
		}
		for (i2 = 0U; (i2 + strlen(CFG_RESET_FN)) <= g_len; i2++) {
			if (memcmp(&g_raw[i2], CFG_RESET_FN,
				   strlen(CFG_RESET_FN)) == 0) {
				raw_hits++;
			}
		}
		TEST_ASSERT_TRUE_MESSAGE(code_hits > 0U,
					 "sts_mcp.c does not call the config reset");
		TEST_ASSERT_TRUE_MESSAGE(
			raw_hits > code_hits,
			"the stripper did not remove the commented mentions "
			"of " CFG_RESET_FN " in sts_mcp.c");
	}
}

/**
 * The defect this whole header exists for: a plane that runs one half.
 *
 * Both calls, in the documented order — the config reset first, because it is
 * what runs every group's appliers, and those are what push the emptied
 * credential keys into the subsystems still holding RAM copies of them. A wipe
 * that ran first would be undone by the appliers it never saw.
 */
static void test_every_plane_runs_both_halves(void)
{
	size_t i;

	for (i = 0U; i < PLANE_N; i++) {
		body_t b;
		size_t cfg_at;
		size_t wipe_at;
		char msg[192];

		load(g_planes[i].file);
		b = body_after(g_planes[i].anchor);

		cfg_at = find_call(CFG_RESET_FN, b.begin, b.end);
		(void)snprintf(msg, sizeof(msg),
			       "%s plane (%s %s) never calls " CFG_RESET_FN
			       "()",
			       g_planes[i].what, g_planes[i].file,
			       g_planes[i].anchor);
		TEST_ASSERT_TRUE_MESSAGE(cfg_at != SIZE_MAX, msg);

		wipe_at = find_call(SEC_WIPE_FN, b.begin, b.end);
		(void)snprintf(msg, sizeof(msg),
			       "%s plane (%s %s) never calls " SEC_WIPE_FN
			       "(): it would leave the TLS identity and every "
			       "web session behind and report a clean unit",
			       g_planes[i].what, g_planes[i].file,
			       g_planes[i].anchor);
		TEST_ASSERT_TRUE_MESSAGE(wipe_at != SIZE_MAX, msg);

		(void)snprintf(msg, sizeof(msg),
			       "%s plane: " SEC_WIPE_FN "() at line %u runs "
			       "before " CFG_RESET_FN "() at line %u; the "
			       "appliers would undo it",
			       g_planes[i].what, line_of(wipe_at),
			       line_of(cfg_at));
		TEST_ASSERT_TRUE_MESSAGE(cfg_at < wipe_at, msg);
	}
}

/**
 * Failures accumulate in the planes too, not just in the arithmetic.
 *
 * A `return` between the two calls is the short-circuit: it makes a failed
 * config reset skip the key wipe, which is how a unit that kept its TLS identity
 * gets reported with the single -EIO an operator reads as "the config bit
 * failed, retry".
 *
 * A `goto` is the same defect wearing a different keyword — and so is an `if`.
 * `if (st.wipe_linked && st.cfg_reset_ok)` skips the wipe with no jump anywhere
 * between the two calls, so the span scan cannot see it; the third check reads
 * the clauses the call is actually nested under and requires every one of them
 * to be about linkage and nothing else.
 */
static void test_no_plane_short_circuits_between_the_halves(void)
{
	size_t i;

	for (i = 0U; i < PLANE_N; i++) {
		body_t b;
		size_t cfg_at;
		size_t wipe_at;
		size_t ret_at;
		cond_t guards[GUARD_MAX];
		size_t n_guards;
		size_t g;
		char msg[224];

		load(g_planes[i].file);
		b = body_after(g_planes[i].anchor);
		cfg_at = find_call(CFG_RESET_FN, b.begin, b.end);
		wipe_at = find_call(SEC_WIPE_FN, b.begin, b.end);
		TEST_ASSERT_TRUE(cfg_at != SIZE_MAX);
		TEST_ASSERT_TRUE(wipe_at != SIZE_MAX);

		ret_at = find_token("return", cfg_at, wipe_at);
		(void)snprintf(msg, sizeof(msg),
			       "%s plane (%s): return at line %u sits between "
			       "the config reset and the key wipe — failures "
			       "must accumulate, not short-circuit",
			       g_planes[i].what, g_planes[i].file,
			       line_of(ret_at));
		TEST_ASSERT_TRUE_MESSAGE(ret_at == SIZE_MAX, msg);

		/* A `goto` out of the middle is the same defect wearing a
		 * different keyword. */
		ret_at = find_token("goto", cfg_at, wipe_at);
		(void)snprintf(msg, sizeof(msg),
			       "%s plane (%s): goto at line %u sits between the "
			       "config reset and the key wipe",
			       g_planes[i].what, g_planes[i].file,
			       line_of(ret_at));
		TEST_ASSERT_TRUE_MESSAGE(ret_at == SIZE_MAX, msg);

		/* …and an `if` is the third, which neither scan above can see. */
		n_guards = guards_of(&b, wipe_at, guards, GUARD_MAX);
		TEST_ASSERT_TRUE_MESSAGE(n_guards < GUARD_MAX,
					 "the key wipe is nested deeper than "
					 "this scan reads");
		for (g = 0U; g < n_guards; g++) {
			(void)snprintf(msg, sizeof(msg),
				       "%s plane (%s): the " SEC_WIPE_FN "() "
				       "call is inside an `else` or a `do`, "
				       "whose condition is written elsewhere — "
				       "this scan cannot show it is only about "
				       "linkage",
				       g_planes[i].what, g_planes[i].file);
			TEST_ASSERT_FALSE_MESSAGE(guards[g].opaque, msg);
			assert_guard_is_linkage_only(g_planes[i].what,
						     g_planes[i].file,
						     guards[g].begin,
						     guards[g].end);
		}
	}
}

/**
 * The guard reader is not vacuous.
 *
 * Everything above is worth nothing if guards_of() returns zero clauses for a
 * call that plainly has one: the loop would then find nothing to object to and
 * report three compliant planes. Two of the three planes reach the wipe through
 * a linkage test that IS in the sources today, so the scan is required to see
 * it — and the panel's, which is the one the reviewer's mutation targets, is
 * required to be exactly one clause naming exactly one field.
 */
static void test_the_guard_reader_sees_the_linkage_tests_that_exist(void)
{
	cond_t guards[GUARD_MAX];
	size_t n;
	body_t b;
	size_t at;

	/* The web plane calls it flat: no clause at all, and that is correct —
	 * sts_web.c IS the net area, so the symbol is never NULL there. */
	load("zephyr/net/sts_web.c");
	b = body_after("pv_factory_reset");
	at = find_call(SEC_WIPE_FN, b.begin, b.end);
	TEST_ASSERT_TRUE(at != SIZE_MAX);
	TEST_ASSERT_EQUAL_size_t(0U, guards_of(&b, at, guards, GUARD_MAX));

	/* The MCP plane tests the weak symbol directly. */
	load("zephyr/console/sts_mcp.c");
	b = body_after("mcp_cfg_factory_reset");
	at = find_call(SEC_WIPE_FN, b.begin, b.end);
	TEST_ASSERT_TRUE(at != SIZE_MAX);
	n = guards_of(&b, at, guards, GUARD_MAX);
	TEST_ASSERT_EQUAL_size_t_MESSAGE(1U, n,
					 "sts_mcp.c gates the key wipe on its "
					 "weak-symbol test; a reader that sees "
					 "no clause there sees nothing anywhere");
	TEST_ASSERT_TRUE(guards[0].readable);
	TEST_ASSERT_TRUE(find_token(SEC_WIPE_FN, guards[0].begin,
				    guards[0].end) != SIZE_MAX);

	/* The panel plane tests the field it recorded that answer in. */
	load("zephyr/ui/sts_ui.c");
	b = body_after("UI_ACTION_FACTORY_RESET");
	at = find_call(SEC_WIPE_FN, b.begin, b.end);
	TEST_ASSERT_TRUE(at != SIZE_MAX);
	n = guards_of(&b, at, guards, GUARD_MAX);
	TEST_ASSERT_EQUAL_size_t_MESSAGE(1U, n,
					 "sts_ui.c gates the key wipe on "
					 "st.wipe_linked; a reader that sees no "
					 "clause there sees nothing anywhere");
	TEST_ASSERT_TRUE(guards[0].readable);
	TEST_ASSERT_TRUE(find_token("wipe_linked", guards[0].begin,
				    guards[0].end) != SIZE_MAX);
}

/**
 * The reboot is not conditional on what the reset managed to do.
 *
 * The two service planes schedule it straight from the body, at brace depth 1 —
 * i.e. on no condition at all. The panel plane reaches sys_reboot() through
 * exactly one gate, sts_factory_should_reboot(), and
 * test_the_reboot_never_depends_on_the_outcome() above proves that gate is
 * constant over its entire domain. Between them that is the whole invariant: an
 * operator who asked for a factory reset always gets the reboot that clears
 * RAM-only key material, including — especially — when a step failed.
 */
static void test_the_reboot_is_not_conditional_on_the_outcome(void)
{
	body_t b;
	size_t at;

	/* --- web ------------------------------------------------------- */
	load("zephyr/net/sts_web.c");
	b = body_after("pv_factory_reset");
	at = find_token("reboot_work", b.begin, b.end);
	TEST_ASSERT_TRUE_MESSAGE(at != SIZE_MAX,
				 "the web plane schedules no reboot");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, depth_at(&b, at),
		"the web plane's reboot is nested inside a condition");

	/* --- mcp ------------------------------------------------------- */
	load("zephyr/console/sts_mcp.c");
	b = body_after("mcp_cfg_factory_reset");
	at = find_token("factory_reboot_work", b.begin, b.end);
	TEST_ASSERT_TRUE_MESSAGE(at != SIZE_MAX,
				 "the MCP plane schedules no reboot");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, depth_at(&b, at),
		"the MCP plane's reboot is nested inside a condition");

	/* --- panel ----------------------------------------------------- */
	load("zephyr/ui/sts_ui.c");
	b = body_after("UI_ACTION_FACTORY_RESET");
	at = find_call("sts_factory_should_reboot", b.begin, b.end);
	TEST_ASSERT_TRUE_MESSAGE(
		at != SIZE_MAX,
		"the panel plane does not consult sts_factory_should_reboot()");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, depth_at(&b, at),
		"the panel plane's reboot gate is itself nested in a condition");

	at = find_call("sys_reboot", b.begin, b.end);
	TEST_ASSERT_TRUE_MESSAGE(at != SIZE_MAX,
				 "the panel plane never reboots");
	/*
	 * Depth 2: the body, then the single sts_factory_should_reboot() gate
	 * asserted above. Any deeper means a second condition crept in, and a
	 * second condition is one this suite has not proven constant.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		2U, depth_at(&b, at),
		"the panel plane's reboot sits under more than the "
		"sts_factory_should_reboot() gate");
}

/**
 * The panel plane feeds the policy honestly.
 *
 * It is the one plane that uses sts_factory_steps_t rather than accumulating an
 * int, so it is the one that can populate the record and then ignore it. Both
 * fields have to be written from the calls' own results, and the result has to
 * be consulted.
 */
static void test_the_panel_plane_reports_what_it_actually_did(void)
{
	body_t b;

	load("zephyr/ui/sts_ui.c");
	b = body_after("UI_ACTION_FACTORY_RESET");

	TEST_ASSERT_TRUE_MESSAGE(find_token("sts_factory_steps_t", b.begin,
					    b.end) != SIZE_MAX,
				 "the panel plane keeps no step record");
	TEST_ASSERT_TRUE_MESSAGE(find_token("cfg_reset_ok", b.begin, b.end) !=
					 SIZE_MAX,
				 "the panel plane never records the config "
				 "reset's result");
	TEST_ASSERT_TRUE_MESSAGE(find_token("wipe_ok", b.begin, b.end) !=
					 SIZE_MAX,
				 "the panel plane never records the key wipe's "
				 "result");
	TEST_ASSERT_TRUE_MESSAGE(find_token("wipe_linked", b.begin, b.end) !=
					 SIZE_MAX,
				 "the panel plane never records whether a wipe "
				 "is linked into this image");
	TEST_ASSERT_TRUE_MESSAGE(find_call("sts_factory_result", b.begin,
					   b.end) != SIZE_MAX,
				 "the panel plane never asks whether the reset "
				 "actually completed");
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_the_result_over_the_whole_domain);
	RUN_TEST(test_a_failure_in_either_half_is_reported);
	RUN_TEST(test_an_absent_wipe_is_honest_rather_than_failed);
	RUN_TEST(test_wipe_skipped_tracks_only_the_linkage);
	RUN_TEST(test_the_reboot_never_depends_on_the_outcome);

	RUN_TEST(test_the_scan_is_not_vacuous);
	RUN_TEST(test_every_plane_runs_both_halves);
	RUN_TEST(test_no_plane_short_circuits_between_the_halves);
	RUN_TEST(test_the_guard_reader_sees_the_linkage_tests_that_exist);
	RUN_TEST(test_the_reboot_is_not_conditional_on_the_outcome);
	RUN_TEST(test_the_panel_plane_reports_what_it_actually_did);

	return UNITY_END();
}
