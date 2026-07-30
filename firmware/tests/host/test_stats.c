/*
 * STS1000 "Meridian" — core/stats unit tests.
 *
 * WHY THESE EXPECTATIONS ARE NOT SELF-REFERENTIAL (ARCHITECTURE.md §9 wants
 * values derived independently of the code under test). Every number asserted
 * below is a closed-form evaluation of the estimator definition on an input
 * whose statistics are known exactly. None of them was read off the
 * implementation. The derivations, in the order the tests run:
 *
 *  1. sqrt. stats_sqrt() is checked against a double Newton iteration written
 *     here, sharing no seed and no table with the implementation, plus exact
 *     squares. Same approach as test_quality.c takes to quality_sqrtf(), and it
 *     comes first because every later coefficient assertion passes through it.
 *
 *  2. PURE FREQUENCY OFFSET, x(i) = x0 + K*i. The second difference
 *     x[i+2m] - 2x[i+m] + x[i] = (x0 + K(i+2m)) - 2(x0 + K(i+m)) + (x0 + Ki)
 *     = 0 for every i and every m. So ADEV and MDEV are *exactly* zero, and
 *     because adev.c forms that difference in int64 the test asserts == 0.0
 *     with no tolerance at all. This is the sharpest test in the file: a sign
 *     error, an off-by-one in the 2m stride, or a stride of m instead of 1 all
 *     leave a non-zero residue that no tolerance can absorb.
 *
 *  3. PURE FREQUENCY DRIFT, y(t) = D*t, so x(t) = D*t^2/2. Then
 *     x(t+2T) - 2x(t+T) + x(t) = (D/2)[(t+2T)^2 - 2(t+T)^2 + t^2] = D*T^2,
 *     independent of t. Hence sigma_y^2(T) = (D T^2)^2 / (2 T^2) = D^2 T^2 / 2
 *     and ADEV(T) = D*T/sqrt(2) — the coefficient, not just the tau^+1 slope.
 *     For MDEV the inner average of m identical second differences is the same
 *     D*T^2, so MDEV(T) = D*T/sqrt(2) as well: drift does *not* separate the
 *     two estimators, which is why test 4 exists.
 *
 *  4. NYQUIST-RATE PHASE ALTERNATION, x(i) = A*(-1)^i. For odd m,
 *     x[i+2m] - 2x[i+m] + x[i] = A(-1)^i[1 - 2(-1)^m + 1] = 4A(-1)^i, so
 *       ADEV(m) = sqrt(16 A^2 / (2 m^2 tau0^2))   = 2*sqrt(2)*A / (m*tau0)
 *     and the MDEV window sum of m alternating terms is 4A(-1)^j, giving
 *       MDEV(m) = sqrt(16 A^2 / (2 m^4 tau0^2))   = 2*sqrt(2)*A / (m^2*tau0).
 *     MDEV(m)/ADEV(m) = 1/m, *exactly*, on a deterministic input. That is the
 *     test that proves MDEV is MDEV: an implementation that returns ADEV is
 *     wrong by a factor of 3 at m = 3 and 15 at m = 15, with no statistics to
 *     hide behind. For even m the second difference is A(-1)^i[1-2+1] = 0 and
 *     both deviations are exactly zero — asserted too, because it is a second
 *     independent exact-zero case with a *different* cause from test 2.
 *
 *  5. WHITE PM, x(i) i.i.d. Rademacher (+/-A, so variance exactly A^2). The
 *     second difference has variance A^2(1+4+1) = 6A^2, so
 *       ADEV(m) = sqrt(6A^2 / (2 m^2 tau0^2)) = sqrt(3)*A / (m*tau0)   ~ tau^-1
 *     and because the three m-sample windows [j,j+m), [j+m,j+2m), [j+2m,j+3m)
 *     are disjoint, the MDEV window sum has variance 6*m*A^2, so
 *       MDEV(m) = sqrt(6 m A^2 / (2 m^4 tau0^2)) = sqrt(3)*A / (m^1.5*tau0)
 *                                                                  ~ tau^-3/2.
 *     Both coefficients are asserted, not just the slopes.
 *
 *  6. WHITE FM, x a Rademacher random walk with step +/-A. The second
 *     difference is (sum of m steps) - (sum of m independent steps), variance
 *     2mA^2, so ADEV(m) = sqrt(2 m A^2 / (2 m^2 tau0^2)) = A/(tau0*sqrt(m)),
 *     i.e. tau^-1/2 with a known coefficient.
 *
 *  7. HISTOGRAM. Every summary is exact by construction — see the two
 *     histogram tests for the arithmetic. The sawtooth pair reproduces the
 *     §14 comparison: a constant bias plus a symmetric qErr ramp of known
 *     period, where the mean is provably unchanged by the correction and the
 *     standard deviation collapses from Delta*sqrt((P^2-1)/12) to exactly zero.
 *
 * RANDOMNESS. Tests 5 and 6 use xorshift64* with a fixed seed, so the record is
 * byte-identical on every run and every host. There is no rand() anywhere here,
 * seeded or otherwise. Tolerances are set from the estimator's equivalent
 * degrees of freedom rather than from what the code happens to return; each one
 * carries its arithmetic.
 */

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "stats/adev.h"

/* 1 PPS. */
#define TAU0_NS UINT64_C(1000000000)

/* ---------------------------------------------------------------- helpers */

/*
 * Reference square root: Newton–Raphson in double from a deliberately naive
 * seed, iterated far past convergence. Shares no code, no table and no seed
 * trick with stats_sqrt(); it is the definition of a square root (y such that
 * y*y == x) driven to a fixed point.
 */
static double ref_sqrt(double x)
{
	double scale = 1.0;
	double y;
	int i;

	if (!(x > 0.0)) {
		return 0.0;
	}

	/*
	 * Range-reduce x into [0.5, 2) by powers of four, since a factor of 4
	 * in the radicand is a factor of 2 in the root. Needed rather than
	 * merely tidy: Newton converges only linearly (by halving) while the
	 * iterate is far above the root, so an unreduced 2^-1074 would need
	 * ~537 iterations before quadratic convergence begins. All the
	 * multiplies are exact — powers of two, including on subnormals.
	 */
	while (x > 2.0) {
		x *= 0.25;
		scale *= 2.0;
	}
	while (x < 0.5) {
		x *= 4.0;
		scale *= 0.5;
	}

	y = 1.0;
	for (i = 0; i < 200; i++) {
		y = 0.5 * (y + x / y);
	}
	return y * scale;
}

static double ref_abs(double x)
{
	return (x < 0.0) ? -x : x;
}

/*
 * Relative-error assertion with the numbers in the failure message. Written by
 * hand rather than using TEST_ASSERT_DOUBLE_WITHIN so the suite does not depend
 * on Unity's optional double support being configured in.
 */
static void assert_rel(double expect, double got, double tol, const char *what)
{
	static char msg[256];
	double err;

	if (expect == 0.0) {
		err = ref_abs(got);
	} else {
		err = ref_abs((got - expect) / expect);
	}

	if (err <= tol) {
		return;
	}

	(void)snprintf(msg, sizeof(msg),
		       "%s: expected %.12g, got %.12g, rel err %.4g > %.4g",
		       what, expect, got, err, tol);
	TEST_FAIL_MESSAGE(msg);
}

static void assert_exact_zero(double got, const char *what)
{
	static char msg[128];

	if (got == 0.0) {
		return;
	}
	(void)snprintf(msg, sizeof(msg), "%s: expected exactly 0.0, got %.17g",
		       what, got);
	TEST_FAIL_MESSAGE(msg);
}

/* xorshift64* — deterministic, seeded, identical on every host. */
static uint64_t rng_state;

static void rng_seed(uint64_t s)
{
	rng_state = (s != 0u) ? s : UINT64_C(0x9E3779B97F4A7C15);
}

static uint64_t rng_next(void)
{
	uint64_t x = rng_state;

	x ^= x >> 12;
	x ^= x << 25;
	x ^= x >> 27;
	rng_state = x;
	return x * UINT64_C(0x2545F4914F6CDD1D);
}

/* Rademacher draw: +A or -A with probability 1/2, variance exactly A^2. Uses
 * the top bit, which is the strong end of xorshift64*'s output. */
static int64_t rng_pm(int64_t a)
{
	return ((rng_next() >> 63) != 0u) ? a : -a;
}

/* One shared record buffer; the noise tests are the only large ones and they
 * run one at a time. */
#define REC_N 131072u
static int64_t g_x[REC_N];

/* ------------------------------------------------------------------- sqrt */

static void test_sqrt_known_and_reference(void)
{
	static const double exact[][2] = {
		{ 0.0, 0.0 },
		{ 1.0, 1.0 },
		{ 4.0, 2.0 },
		{ 9.0, 3.0 },
		{ 0.25, 0.5 },
		{ 1.0e-24, 1.0e-12 },
		{ 1.0e24, 1.0e12 },
	};
	double x;
	size_t i;

	for (i = 0u; i < sizeof(exact) / sizeof(exact[0]); i++) {
		assert_rel(exact[i][1], stats_sqrt(exact[i][0]), 1.0e-15,
			   "exact square");
	}

	/* Sweep 60 decades against the independent reference. */
	for (x = 1.0e-30; x < 1.0e30; x *= 10.0) {
		assert_rel(ref_sqrt(x), stats_sqrt(x), 1.0e-15, "decade sweep");
		assert_rel(ref_sqrt(x * 3.0), stats_sqrt(x * 3.0), 1.0e-15,
			   "decade sweep x3");
	}

	/* Refusals and edges must not trap or loop. */
	assert_exact_zero(stats_sqrt(0.0), "sqrt(+0)");
	assert_exact_zero(stats_sqrt(-0.0), "sqrt(-0)");
	assert_exact_zero(stats_sqrt(-1.0), "sqrt(-1)");
	assert_exact_zero(stats_sqrt(-1.0e300), "sqrt(-1e300)");

	/* Subnormal: 5e-324 is DBL_TRUE_MIN, whose root is ~2.22e-162. */
	assert_rel(ref_sqrt(5.0e-324), stats_sqrt(5.0e-324), 1.0e-12,
		   "sqrt(subnormal)");
	assert_rel(ref_sqrt(1.0e-310), stats_sqrt(1.0e-310), 1.0e-12,
		   "sqrt(1e-310)");

	/* +infinity is its own square root and must not enter the Newton loop
	 * (inf/inf is NaN, which would poison it). Built at run time from a
	 * volatile so the overflow is not a compile-time constant. */
	{
		volatile double big = 1.0e308;
		double inf = big * 10.0;

		TEST_ASSERT_TRUE_MESSAGE(inf > 1.0e308, "need an infinity");
		TEST_ASSERT_TRUE_MESSAGE(stats_sqrt(inf) == inf,
					 "sqrt(+inf) must be +inf");
	}
}

/* ------------------------------------ 2. pure frequency offset -> exactly 0 */

static void test_freq_offset_is_exactly_zero(void)
{
	/* x(i) = x0 + K*i: a 1.234567 us/s frequency offset on a -0.987 ms
	 * starting phase. Both signs, both large, so a dropped term shows. */
	static const int64_t x0 = -987654321;
	static const int64_t k = 1234567;
	static const uint32_t ms[] = { 1u, 2u, 3u, 7u, 64u, 100u, 1000u, 1365u };
	const size_t n = 4096u;
	size_t i;
	size_t j;

	for (i = 0u; i < n; i++) {
		g_x[i] = x0 + k * (int64_t)i;
	}

	for (j = 0u; j < sizeof(ms) / sizeof(ms[0]); j++) {
		char what[64];
		double v = -1.0;

		(void)snprintf(what, sizeof(what), "ADEV(m=%u) on linear phase",
			       ms[j]);
		TEST_ASSERT_EQUAL_INT(0, stats_adev(g_x, n, TAU0_NS, ms[j], &v));
		assert_exact_zero(v, what);

		v = -1.0;
		(void)snprintf(what, sizeof(what), "MDEV(m=%u) on linear phase",
			       ms[j]);
		TEST_ASSERT_EQUAL_INT(0, stats_mdev(g_x, n, TAU0_NS, ms[j], &v));
		assert_exact_zero(v, what);
	}

	/* A pure phase step with no rate at all is the degenerate case of the
	 * same argument: constant x annihilates too. */
	for (i = 0u; i < n; i++) {
		g_x[i] = 42;
	}
	{
		double v = -1.0;

		TEST_ASSERT_EQUAL_INT(0, stats_adev(g_x, n, TAU0_NS, 5u, &v));
		assert_exact_zero(v, "ADEV on constant phase");
		TEST_ASSERT_EQUAL_INT(0, stats_mdev(g_x, n, TAU0_NS, 5u, &v));
		assert_exact_zero(v, "MDEV on constant phase");
	}
}

/* ------------------------------------- 3. pure drift -> D*tau/sqrt(2) */

static void test_drift_coefficient(void)
{
	/*
	 * D = 1e-10 per second (0.1 ppb/s). x(t) = D t^2 / 2 seconds
	 *   = 0.5e-10 * t^2 s = 0.5e-10 * 1e12 * t^2 ps = 50 * t^2 ps,
	 * so with tau0 = 1 s the record is the exact integer 50*i^2.
	 */
	const double d_per_s = 1.0e-10;
	const size_t n = 4096u;
	const uint32_t ms[] = { 1u, 2u, 5u, 10u, 100u, 1000u };
	size_t i;
	size_t j;
	double root2 = ref_sqrt(2.0);

	for (i = 0u; i < n; i++) {
		g_x[i] = 50 * (int64_t)i * (int64_t)i;
	}

	for (j = 0u; j < sizeof(ms) / sizeof(ms[0]); j++) {
		double tau_s = (double)ms[j];
		double expect = d_per_s * tau_s / root2;
		char what[80];
		double v = -1.0;

		/*
		 * The second difference is exactly 100*m^2 ps for every i, so
		 * the only inexactness in the whole chain is the square root.
		 * 1e-12 is therefore a generous bound and still ~7 orders
		 * tighter than any slope-only check.
		 */
		(void)snprintf(what, sizeof(what),
			       "ADEV(m=%u) = D*tau/sqrt(2)", ms[j]);
		TEST_ASSERT_EQUAL_INT(0, stats_adev(g_x, n, TAU0_NS, ms[j], &v));
		assert_rel(expect, v, 1.0e-12, what);

		/* MDEV's inner average of m identical second differences is
		 * that same value, so the closed form is identical. */
		v = -1.0;
		(void)snprintf(what, sizeof(what),
			       "MDEV(m=%u) = D*tau/sqrt(2)", ms[j]);
		TEST_ASSERT_EQUAL_INT(0, stats_mdev(g_x, n, TAU0_NS, ms[j], &v));
		assert_rel(expect, v, 1.0e-12, what);
	}

	/* tau^+1: a decade of tau must give a decade of ADEV. */
	{
		double a1 = -1.0;
		double a10 = -1.0;

		TEST_ASSERT_EQUAL_INT(0, stats_adev(g_x, n, TAU0_NS, 1u, &a1));
		TEST_ASSERT_EQUAL_INT(0, stats_adev(g_x, n, TAU0_NS, 10u, &a10));
		assert_rel(10.0, a10 / a1, 1.0e-12, "drift slope tau^+1");
	}
}

/* ------------------- 4. alternation -> MDEV/ADEV = 1/m, exactly */

static void test_alternating_phase_separates_mdev_from_adev(void)
{
	/* x(i) = A*(-1)^i with A = 1 ns. */
	const int64_t a_ps = 1000;
	const size_t n = 4096u;
	const uint32_t odd[] = { 1u, 3u, 5u, 15u, 101u, 1001u };
	const uint32_t even[] = { 2u, 4u, 16u, 100u, 1000u };
	double two_root2 = 2.0 * ref_sqrt(2.0);
	size_t i;
	size_t j;

	for (i = 0u; i < n; i++) {
		g_x[i] = ((i % 2u) == 0u) ? a_ps : -a_ps;
	}

	for (j = 0u; j < sizeof(odd) / sizeof(odd[0]); j++) {
		double md = (double)odd[j];
		/* A is in ps, so the dimensionless deviation carries 1e-12. */
		double exp_adev = two_root2 * (double)a_ps * 1.0e-12 / md;
		double exp_mdev = exp_adev / md;
		double adev = -1.0;
		double mdev = -1.0;
		char what[96];

		TEST_ASSERT_EQUAL_INT(0,
			stats_adev(g_x, n, TAU0_NS, odd[j], &adev));
		TEST_ASSERT_EQUAL_INT(0,
			stats_mdev(g_x, n, TAU0_NS, odd[j], &mdev));

		(void)snprintf(what, sizeof(what),
			       "ADEV(m=%u) = 2*sqrt(2)*A/(m*tau0)", odd[j]);
		assert_rel(exp_adev, adev, 1.0e-12, what);

		(void)snprintf(what, sizeof(what),
			       "MDEV(m=%u) = 2*sqrt(2)*A/(m^2*tau0)", odd[j]);
		assert_rel(exp_mdev, mdev, 1.0e-12, what);

		/*
		 * The load-bearing assertion of this file: on a deterministic
		 * input MDEV is smaller than ADEV by exactly m. An
		 * implementation that computes ADEV twice fails this at m = 3.
		 */
		(void)snprintf(what, sizeof(what),
			       "ADEV(m=%u)/MDEV(m=%u) = m", odd[j], odd[j]);
		assert_rel(md, adev / mdev, 1.0e-12, what);
	}

	/* Even m annihilates the alternation entirely: a second exact-zero
	 * case, with a different cause from the linear-phase one. */
	for (j = 0u; j < sizeof(even) / sizeof(even[0]); j++) {
		double v = -1.0;
		char what[64];

		(void)snprintf(what, sizeof(what),
			       "ADEV(m=%u) on alternation", even[j]);
		TEST_ASSERT_EQUAL_INT(0,
			stats_adev(g_x, n, TAU0_NS, even[j], &v));
		assert_exact_zero(v, what);

		v = -1.0;
		(void)snprintf(what, sizeof(what),
			       "MDEV(m=%u) on alternation", even[j]);
		TEST_ASSERT_EQUAL_INT(0,
			stats_mdev(g_x, n, TAU0_NS, even[j], &v));
		assert_exact_zero(v, what);
	}
}

/* --------------- 5. white PM -> ADEV tau^-1, MDEV tau^-3/2 */

static void test_white_pm_adev_and_mdev_slopes(void)
{
	/* sigma_x = A exactly, because the draw is Rademacher. */
	const int64_t a_ps = 1000;
	const uint32_t ms[] = { 1u, 4u, 16u, 64u, 256u };
	/*
	 * Tolerances, per averaging factor, derived from each estimator's
	 * equivalent degrees of freedom — not from what the code returns.
	 *
	 * ADEV. For white PM the overlapping estimator has
	 * edf ~= (N+1)(N-2m)/(2(N-m)), which at N = 131072 is ~65500 for every
	 * factor here. The variance's relative sigma is 1/sqrt(2*edf) = 0.28 %
	 * and the deviation's is half that, ~0.14 %. 2 % is ~14 sigma.
	 *
	 * MDEV. The m-sample phase average is what makes MDEV the useful
	 * estimator and also what costs it confidence: the window sums are
	 * m-long and heavily overlapped, so the count of effectively independent
	 * estimates falls roughly as N/(2m) rather than staying at ~N/2. That
	 * gives a deviation sigma of ~0.14 % at m = 1 rising to ~2.2 % at
	 * m = 256, so a single tolerance for the whole sweep would be either
	 * useless at small tau or fragile at large tau. These bounds are ~5-14
	 * sigma at every point.
	 *
	 * Both columns stay orders of magnitude below what any real defect
	 * costs: a dropped factor of 2 in the normalisation is 41 % and a
	 * wrong tau exponent is a factor of m.
	 */
	static const double tol_a[] = { 0.02, 0.02, 0.02, 0.02, 0.02 };
	static const double tol_m[] = { 0.02, 0.02, 0.03, 0.06, 0.12 };
	double root3 = ref_sqrt(3.0);
	size_t i;
	size_t j;
	double adev[5];
	double mdev[5];

	rng_seed(UINT64_C(0x5153545331303030)); /* "QST1000" */
	for (i = 0u; i < REC_N; i++) {
		g_x[i] = rng_pm(a_ps);
	}

	for (j = 0u; j < sizeof(ms) / sizeof(ms[0]); j++) {
		double md = (double)ms[j];
		double sig = (double)a_ps * 1.0e-12;
		double exp_a = root3 * sig / md;
		double exp_m = root3 * sig / (md * ref_sqrt(md));
		char what[96];

		TEST_ASSERT_EQUAL_INT(0,
			stats_adev(g_x, REC_N, TAU0_NS, ms[j], &adev[j]));
		TEST_ASSERT_EQUAL_INT(0,
			stats_mdev(g_x, REC_N, TAU0_NS, ms[j], &mdev[j]));

		(void)snprintf(what, sizeof(what),
			       "white PM ADEV(m=%u) = sqrt(3)*sigma/tau", ms[j]);
		assert_rel(exp_a, adev[j], tol_a[j], what);

		(void)snprintf(what, sizeof(what),
			       "white PM MDEV(m=%u) = sqrt(3)*sigma/(m^1.5*tau0)",
			       ms[j]);
		assert_rel(exp_m, mdev[j], tol_m[j], what);
	}

	/*
	 * MDEV and ADEV are the same estimator at m = 1 — the m^4 denominator
	 * is 1, the window is one term, and both run over n-2 terms. Not
	 * "close": identical, and asserted as such. This is the structural half
	 * of the MDEV proof; the alternation test is the numerical half.
	 */
	TEST_ASSERT_TRUE_MESSAGE(adev[0] == mdev[0],
				 "MDEV(m=1) must equal ADEV(m=1) exactly");

	/* Slopes over the 1 -> 256 span: ADEV falls by 256, MDEV by 256^1.5 =
	 * 4096. These differ by a factor of 16, which is the whole reason §14
	 * asks for both. */
	assert_rel(256.0, adev[0] / adev[4], tol_a[4], "white PM ADEV tau^-1");
	assert_rel(4096.0, mdev[0] / mdev[4], tol_m[4],
		   "white PM MDEV tau^-3/2");

	/* And MDEV/ADEV = m^-1/2 at each tau. The ratio's uncertainty is the
	 * MDEV term's, so it takes the MDEV tolerance. */
	for (j = 0u; j < sizeof(ms) / sizeof(ms[0]); j++) {
		char what[80];

		(void)snprintf(what, sizeof(what),
			       "white PM ADEV/MDEV at m=%u = sqrt(m)", ms[j]);
		assert_rel(ref_sqrt((double)ms[j]), adev[j] / mdev[j], tol_m[j],
			   what);
	}
}

/* --------------------------- 6. white FM -> ADEV tau^-1/2 */

static void test_white_fm_adev_slope(void)
{
	/* Random walk with Rademacher steps: step std is exactly A. */
	const int64_t a_ps = 1000;
	const uint32_t ms[] = { 1u, 10u, 100u };
	const double tol[] = { 0.03, 0.05, 0.09 };
	int64_t acc = 0;
	size_t i;
	size_t j;
	double adev[3];

	rng_seed(UINT64_C(0x574B464D5F303031)); /* "WKFM_001" */
	for (i = 0u; i < REC_N; i++) {
		g_x[i] = acc;
		acc += rng_pm(a_ps);
	}

	for (j = 0u; j < sizeof(ms) / sizeof(ms[0]); j++) {
		double md = (double)ms[j];
		double expect = (double)a_ps * 1.0e-12 / ref_sqrt(md);
		char what[80];

		TEST_ASSERT_EQUAL_INT(0,
			stats_adev(g_x, REC_N, TAU0_NS, ms[j], &adev[j]));

		/*
		 * Tolerance from the white-FM edf of the overlapping estimator,
		 * edf ~= 3(N-1)/(2m) - 2(N-2)/N: at N = 131072 that is ~2e5 at
		 * m = 1 (deviation sigma 0.11 %), ~2e4 at m = 10 (0.35 %) and
		 * ~2e3 at m = 100 (1.1 %). The bounds below are 27, 14 and 8
		 * sigma respectively — wide enough that the suite cannot flake
		 * on this fixed record, narrow enough that a tau^-1 or tau^0
		 * answer fails by orders of magnitude.
		 */
		(void)snprintf(what, sizeof(what),
			       "white FM ADEV(m=%u) = A/(tau0*sqrt(m))", ms[j]);
		assert_rel(expect, adev[j], tol[j], what);
	}

	/* A decade of tau must give sqrt(10) of ADEV. */
	assert_rel(ref_sqrt(10.0), adev[0] / adev[1], 0.06,
		   "white FM slope 1->10");
	assert_rel(ref_sqrt(10.0), adev[1] / adev[2], 0.10,
		   "white FM slope 10->100");
	assert_rel(10.0, adev[0] / adev[2], 0.10, "white FM decade 1->100");
}

/* ------------------------------------------------ 7a. histogram, exact */

static void test_hist_exact_distribution(void)
{
	/*
	 * Repeating {-3000, -1000, +1000, +3000} ps, 1000 cycles.
	 *   mean = 0 exactly (the set is symmetric)
	 *   mean(x^2) = (9 + 1 + 1 + 9)e6 / 4 = 5e6 ps^2
	 *   rms = sqrt(5e6) = 2236.06797749979 ps
	 *   sdev = rms because the mean is zero
	 *   p2p = 3000 - (-3000) = 6000 ps
	 */
	static const int64_t cycle[4] = { -3000, -1000, 1000, 3000 };
	const size_t n = 4000u;
	uint32_t bins[4];
	stats_hist_t h;
	size_t i;

	for (i = 0u; i < n; i++) {
		g_x[i] = cycle[i % 4u];
	}

	/* Window [-4000, 4000) in four 2000 ps bins holds every sample. */
	TEST_ASSERT_EQUAL_INT(0,
		stats_hist(g_x, n, -4000, 2000, bins, 4u, &h));

	TEST_ASSERT_EQUAL_UINT32(1000u, bins[0]); /* [-4000,-2000) */
	TEST_ASSERT_EQUAL_UINT32(1000u, bins[1]); /* [-2000,    0) */
	TEST_ASSERT_EQUAL_UINT32(1000u, bins[2]); /* [    0, 2000) */
	TEST_ASSERT_EQUAL_UINT32(1000u, bins[3]); /* [ 2000, 4000) */

	TEST_ASSERT_EQUAL_UINT(n, h.n);
	TEST_ASSERT_EQUAL_UINT(n, h.in_range);
	TEST_ASSERT_EQUAL_UINT(0u, h.under);
	TEST_ASSERT_EQUAL_UINT(0u, h.over);
	TEST_ASSERT_EQUAL_INT64(-3000, h.min_ps);
	TEST_ASSERT_EQUAL_INT64(3000, h.max_ps);
	TEST_ASSERT_EQUAL_INT64(6000, h.p2p_ps);
	assert_exact_zero(h.mean_ps, "hist mean of a symmetric set");
	assert_rel(ref_sqrt(5.0e6), h.rms_ps, 1.0e-14, "hist rms");
	assert_rel(ref_sqrt(5.0e6), h.sdev_ps, 1.0e-14, "hist sdev");

	/* rms^2 = mean^2 + sdev^2 must hold identically. */
	assert_rel(h.rms_ps * h.rms_ps,
		   h.mean_ps * h.mean_ps + h.sdev_ps * h.sdev_ps, 1.0e-13,
		   "rms^2 = mean^2 + sdev^2");

	/*
	 * Narrow the window to [-2000, 0) u [0, 2000) so the outer pair falls
	 * outside. The counts must move; the summary must not — it describes
	 * the data, not the window.
	 */
	{
		uint32_t two[2];

		TEST_ASSERT_EQUAL_INT(0,
			stats_hist(g_x, n, -2000, 2000, two, 2u, &h));
		TEST_ASSERT_EQUAL_UINT32(1000u, two[0]);
		TEST_ASSERT_EQUAL_UINT32(1000u, two[1]);
		TEST_ASSERT_EQUAL_UINT(1000u, h.under);
		TEST_ASSERT_EQUAL_UINT(1000u, h.over);
		TEST_ASSERT_EQUAL_UINT(2000u, h.in_range);
		TEST_ASSERT_EQUAL_INT64(6000, h.p2p_ps);
		assert_rel(ref_sqrt(5.0e6), h.rms_ps, 1.0e-14,
			   "rms is window-independent");
	}

	/* Summary-only mode: no bins, same statistics. */
	TEST_ASSERT_EQUAL_INT(0, stats_hist(g_x, n, 0, 0, NULL, 0u, &h));
	TEST_ASSERT_EQUAL_UINT(0u, h.in_range);
	TEST_ASSERT_EQUAL_UINT(0u, h.under);
	TEST_ASSERT_EQUAL_UINT(0u, h.over);
	assert_rel(ref_sqrt(5.0e6), h.rms_ps, 1.0e-14, "summary-only rms");

	/* n = 1 is a legal record: zero spread, and the mean is the sample. */
	g_x[0] = -7500;
	TEST_ASSERT_EQUAL_INT(0, stats_hist(g_x, 1u, 0, 0, NULL, 0u, &h));
	assert_rel(-7500.0, h.mean_ps, 1.0e-15, "n=1 mean");
	assert_rel(7500.0, h.rms_ps, 1.0e-14, "n=1 rms");
	assert_exact_zero(h.sdev_ps, "n=1 sdev");
	TEST_ASSERT_EQUAL_INT64(0, h.p2p_ps);
}

/* -------------------------- 7b. the §14 sawtooth-correction pair */

static void test_hist_sawtooth_pair_is_the_qerr_proof(void)
{
	/*
	 * §14 wants the residual histogram computed with and without sawtooth
	 * correction, and the difference is the evidence the qErr path works.
	 * Model it with exact arithmetic:
	 *
	 *   corrected[i]   = C                     (a pure cable/antenna bias)
	 *   uncorrected[i] = C + Delta*k,  k = (i mod P) - (P-1)/2
	 *
	 * with P = 9 (odd, so k is an integer), Delta = 500 ps, C = 2000 ps and
	 * n a whole number of periods. Then:
	 *
	 *   mean is C in BOTH, exactly, because sum(k) = 0 over a period;
	 *   sdev(corrected)   = 0 exactly;
	 *   sdev(uncorrected) = Delta*sqrt((P^2-1)/12) = 500*sqrt(80/12)
	 *                     = 500*sqrt(20/3) = 1290.9944487358056 ps;
	 *   rms(uncorrected)  = sqrt(C^2 + sdev^2) = sqrt(4e6 + 5e6/3);
	 *   p2p(uncorrected)  = 8*Delta = 4000 ps.
	 *
	 * The shape of that pair is the diagnostic: mean identical, spread
	 * collapsing. A qErr path with an inverted sign would double the spread
	 * instead of removing it, and this is the arithmetic that would show it.
	 */
	const int64_t c_ps = 2000;
	const int64_t delta_ps = 500;
	const size_t period = 9u;
	const size_t n = 9000u; /* 1000 whole periods */
	double sdev_unc = (double)delta_ps * ref_sqrt(20.0 / 3.0);
	stats_hist_t corr;
	stats_hist_t unc;
	size_t i;

	/* Uncorrected first, in the shared buffer. */
	for (i = 0u; i < n; i++) {
		int64_t k = (int64_t)(i % period) - 4;

		g_x[i] = c_ps + delta_ps * k;
	}
	TEST_ASSERT_EQUAL_INT(0, stats_hist(g_x, n, 0, 0, NULL, 0u, &unc));

	for (i = 0u; i < n; i++) {
		g_x[i] = c_ps;
	}
	TEST_ASSERT_EQUAL_INT(0, stats_hist(g_x, n, 0, 0, NULL, 0u, &corr));

	/* The bias survives the correction, exactly and in both. */
	assert_rel((double)c_ps, unc.mean_ps, 1.0e-14, "uncorrected mean = C");
	assert_rel((double)c_ps, corr.mean_ps, 1.0e-15, "corrected mean = C");
	assert_rel(unc.mean_ps, corr.mean_ps, 1.0e-14,
		   "correction must not move the mean");

	/* The spread does not. */
	assert_rel(sdev_unc, unc.sdev_ps, 1.0e-13,
		   "uncorrected sdev = Delta*sqrt((P^2-1)/12)");
	assert_exact_zero(corr.sdev_ps, "corrected sdev");

	assert_rel(ref_sqrt(4.0e6 + 5.0e6 / 3.0), unc.rms_ps, 1.0e-13,
		   "uncorrected rms = sqrt(C^2 + sdev^2)");
	assert_rel((double)c_ps, corr.rms_ps, 1.0e-14,
		   "corrected rms = C (no spread left)");

	TEST_ASSERT_EQUAL_INT64(4000, unc.p2p_ps);
	TEST_ASSERT_EQUAL_INT64(0, corr.p2p_ps);
	TEST_ASSERT_EQUAL_INT64(c_ps - 4 * delta_ps, unc.min_ps);
	TEST_ASSERT_EQUAL_INT64(c_ps + 4 * delta_ps, unc.max_ps);

	/* The identity holds on both, which is what lets an operator read the
	 * bias and the jitter off the same two numbers. */
	assert_rel(unc.rms_ps * unc.rms_ps,
		   unc.mean_ps * unc.mean_ps + unc.sdev_ps * unc.sdev_ps,
		   1.0e-13, "uncorrected rms^2 = mean^2 + sdev^2");

	/* And the histogram itself: nine 500 ps bins centred on C, one per
	 * sawtooth step, 1000 counts each. */
	{
		uint32_t bins[9];
		size_t j;

		for (i = 0u; i < n; i++) {
			int64_t k = (int64_t)(i % period) - 4;

			g_x[i] = c_ps + delta_ps * k;
		}
		TEST_ASSERT_EQUAL_INT(0,
			stats_hist(g_x, n, c_ps - 4 * delta_ps - delta_ps / 2,
				   delta_ps, bins, 9u, &unc));
		for (j = 0u; j < 9u; j++) {
			TEST_ASSERT_EQUAL_UINT32(1000u, bins[j]);
		}
		TEST_ASSERT_EQUAL_UINT(n, unc.in_range);
	}
}

/* ------------------------------------------------ max_m and the tau axis */

static void test_max_m_and_octaves(void)
{
	uint32_t m[8];
	size_t got;

	/* ADEV needs n >= 2m+1, MDEV needs n >= 3m. */
	TEST_ASSERT_EQUAL_UINT32(0u, stats_max_m(0u, STATS_DEV_ADEV));
	TEST_ASSERT_EQUAL_UINT32(0u, stats_max_m(1u, STATS_DEV_ADEV));
	TEST_ASSERT_EQUAL_UINT32(0u, stats_max_m(2u, STATS_DEV_ADEV));
	TEST_ASSERT_EQUAL_UINT32(1u, stats_max_m(3u, STATS_DEV_ADEV));
	TEST_ASSERT_EQUAL_UINT32(1u, stats_max_m(4u, STATS_DEV_ADEV));
	TEST_ASSERT_EQUAL_UINT32(2u, stats_max_m(5u, STATS_DEV_ADEV));
	TEST_ASSERT_EQUAL_UINT32(50u, stats_max_m(101u, STATS_DEV_ADEV));

	TEST_ASSERT_EQUAL_UINT32(0u, stats_max_m(0u, STATS_DEV_MDEV));
	TEST_ASSERT_EQUAL_UINT32(0u, stats_max_m(2u, STATS_DEV_MDEV));
	TEST_ASSERT_EQUAL_UINT32(1u, stats_max_m(3u, STATS_DEV_MDEV));
	TEST_ASSERT_EQUAL_UINT32(1u, stats_max_m(5u, STATS_DEV_MDEV));
	TEST_ASSERT_EQUAL_UINT32(2u, stats_max_m(6u, STATS_DEV_MDEV));
	TEST_ASSERT_EQUAL_UINT32(33u, stats_max_m(99u, STATS_DEV_MDEV));
	TEST_ASSERT_EQUAL_UINT32(33u, stats_max_m(100u, STATS_DEV_MDEV));

	/* The bound is real: the returned m must work and m+1 must not. */
	{
		double v;
		size_t i;

		for (i = 0u; i < 101u; i++) {
			g_x[i] = 50 * (int64_t)i * (int64_t)i;
		}
		TEST_ASSERT_EQUAL_INT(0, stats_adev(g_x, 101u, TAU0_NS, 50u, &v));
		TEST_ASSERT_EQUAL_INT(-ENODATA,
			stats_adev(g_x, 101u, TAU0_NS, 51u, &v));
		TEST_ASSERT_EQUAL_INT(0, stats_mdev(g_x, 99u, TAU0_NS, 33u, &v));
		TEST_ASSERT_EQUAL_INT(-ENODATA,
			stats_mdev(g_x, 99u, TAU0_NS, 34u, &v));
	}

	/* Octaves: 1,2,4,8,16,32 for max_m = 50. */
	got = stats_m_octaves(101u, STATS_DEV_ADEV, m, 8u);
	TEST_ASSERT_EQUAL_UINT(6u, got);
	TEST_ASSERT_EQUAL_UINT32(1u, m[0]);
	TEST_ASSERT_EQUAL_UINT32(2u, m[1]);
	TEST_ASSERT_EQUAL_UINT32(4u, m[2]);
	TEST_ASSERT_EQUAL_UINT32(8u, m[3]);
	TEST_ASSERT_EQUAL_UINT32(16u, m[4]);
	TEST_ASSERT_EQUAL_UINT32(32u, m[5]);

	/* Count-only, and a capacity smaller than the answer. */
	TEST_ASSERT_EQUAL_UINT(6u,
		stats_m_octaves(101u, STATS_DEV_ADEV, NULL, 0u));
	memset(m, 0xFF, sizeof(m));
	TEST_ASSERT_EQUAL_UINT(6u, stats_m_octaves(101u, STATS_DEV_ADEV, m, 2u));
	TEST_ASSERT_EQUAL_UINT32(1u, m[0]);
	TEST_ASSERT_EQUAL_UINT32(2u, m[1]);
	TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, m[2]); /* untouched past cap */

	TEST_ASSERT_EQUAL_UINT(0u, stats_m_octaves(2u, STATS_DEV_ADEV, m, 8u));
	TEST_ASSERT_EQUAL_UINT(1u, stats_m_octaves(3u, STATS_DEV_MDEV, m, 8u));

	/*
	 * The STATS_MAX_M ceiling. stats_max_m() reads only the sample count, so
	 * a record length far past the ceiling can be asked about without
	 * allocating one — which is the only way to reach this clamp, since the
	 * record it describes would be 37 MB.
	 */
	TEST_ASSERT_EQUAL_UINT32(STATS_MAX_M,
		stats_max_m((size_t)STATS_MAX_M * 4u, STATS_DEV_ADEV));
	TEST_ASSERT_EQUAL_UINT32(STATS_MAX_M,
		stats_max_m((size_t)STATS_MAX_M * 4u, STATS_DEV_MDEV));
	/* 2^21 = 2097152 is the last octave at or below STATS_MAX_M. */
	TEST_ASSERT_EQUAL_UINT(22u,
		stats_m_octaves((size_t)STATS_MAX_M * 4u, STATS_DEV_ADEV, NULL,
				0u));
}

/* ------------------------------------------------------------------ sweep */

static void test_sweep_matches_single_calls_and_stops_cleanly(void)
{
	const uint32_t ms[] = { 1u, 2u, 4u, 8u, 16u };
	const size_t n = 4096u;
	double out[5];
	double one;
	size_t got = 99u;
	size_t i;
	size_t j;

	for (i = 0u; i < n; i++) {
		g_x[i] = 50 * (int64_t)i * (int64_t)i; /* drift record */
	}

	TEST_ASSERT_EQUAL_INT(0,
		stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, ms, 5u, out,
				&got));
	TEST_ASSERT_EQUAL_UINT(5u, got);
	for (j = 0u; j < 5u; j++) {
		TEST_ASSERT_EQUAL_INT(0,
			stats_adev(g_x, n, TAU0_NS, ms[j], &one));
		TEST_ASSERT_TRUE_MESSAGE(out[j] == one,
					 "ADEV sweep must equal single calls");
	}

	TEST_ASSERT_EQUAL_INT(0,
		stats_dev_sweep(STATS_DEV_MDEV, g_x, n, TAU0_NS, ms, 5u, out,
				&got));
	TEST_ASSERT_EQUAL_UINT(5u, got);
	for (j = 0u; j < 5u; j++) {
		TEST_ASSERT_EQUAL_INT(0,
			stats_mdev(g_x, n, TAU0_NS, ms[j], &one));
		TEST_ASSERT_TRUE_MESSAGE(out[j] == one,
					 "MDEV sweep must equal single calls");
	}

	/* Stops at the first factor the record cannot support and says so. */
	{
		const uint32_t past[] = { 1u, 2u, 100000u, 200000u };

		got = 99u;
		TEST_ASSERT_EQUAL_INT(0,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, past,
					4u, out, &got));
		TEST_ASSERT_EQUAL_UINT(2u, got);
	}

	/* A record that supports nothing yields zero points, not an error. */
	{
		const uint32_t one_m[] = { 1u };

		got = 99u;
		TEST_ASSERT_EQUAL_INT(0,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, 2u, TAU0_NS, one_m,
					1u, out, &got));
		TEST_ASSERT_EQUAL_UINT(0u, got);
	}
}

/* -------------------------------------------------------------- refusals */

static void test_refusals(void)
{
	const size_t n = 64u;
	double v;
	size_t got;
	stats_hist_t h;
	uint32_t bins[4];
	size_t i;

	for (i = 0u; i < n; i++) {
		g_x[i] = (int64_t)i;
	}

	/* Zero and one point: refuse, do not divide by zero. */
	TEST_ASSERT_EQUAL_INT(-ENODATA, stats_adev(g_x, 0u, TAU0_NS, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-ENODATA, stats_adev(g_x, 1u, TAU0_NS, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-ENODATA, stats_adev(g_x, 2u, TAU0_NS, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-ENODATA, stats_mdev(g_x, 0u, TAU0_NS, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-ENODATA, stats_mdev(g_x, 1u, TAU0_NS, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-ENODATA, stats_mdev(g_x, 2u, TAU0_NS, 1u, &v));

	/* tau beyond what the record supports. */
	TEST_ASSERT_EQUAL_INT(-ENODATA, stats_adev(g_x, n, TAU0_NS, 32u, &v));
	TEST_ASSERT_EQUAL_INT(0, stats_adev(g_x, n, TAU0_NS, 31u, &v));
	TEST_ASSERT_EQUAL_INT(-ENODATA, stats_mdev(g_x, n, TAU0_NS, 22u, &v));
	TEST_ASSERT_EQUAL_INT(0, stats_mdev(g_x, n, TAU0_NS, 21u, &v));

	/* Bad arguments. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_adev(NULL, n, TAU0_NS, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_adev(g_x, n, TAU0_NS, 1u, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_adev(g_x, n, TAU0_NS, 0u, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_adev(g_x, n, 0u, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		stats_adev(g_x, n, TAU0_NS, STATS_MAX_M + 1u, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_mdev(NULL, n, TAU0_NS, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_mdev(g_x, n, TAU0_NS, 1u, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_mdev(g_x, n, TAU0_NS, 0u, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_mdev(g_x, n, 0u, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		stats_mdev(g_x, n, TAU0_NS, STATS_MAX_M + 1u, &v));

	/* Out-of-range phase is refused, not saturated: a clipped sample makes
	 * a plausible-looking ADEV, which is the failure mode worth refusing.
	 * Checked at both ends and exactly on the boundary. */
	g_x[10] = STATS_PHASE_MAX_PS;
	TEST_ASSERT_EQUAL_INT(0, stats_adev(g_x, n, TAU0_NS, 1u, &v));
	g_x[10] = STATS_PHASE_MAX_PS + 1;
	TEST_ASSERT_EQUAL_INT(-ERANGE, stats_adev(g_x, n, TAU0_NS, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-ERANGE, stats_mdev(g_x, n, TAU0_NS, 1u, &v));
	g_x[10] = -STATS_PHASE_MAX_PS - 1;
	TEST_ASSERT_EQUAL_INT(-ERANGE, stats_adev(g_x, n, TAU0_NS, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-ERANGE, stats_mdev(g_x, n, TAU0_NS, 1u, &v));
	TEST_ASSERT_EQUAL_INT(-ERANGE, stats_hist(g_x, n, 0, 0, NULL, 0u, &h));
	g_x[10] = 10;

	/* Sweep argument validation. */
	{
		const uint32_t ok[] = { 1u, 2u };
		const uint32_t desc[] = { 4u, 2u };
		const uint32_t dup[] = { 2u, 2u };
		const uint32_t zero[] = { 0u };
		const uint32_t big[] = { STATS_MAX_M + 1u };
		double out[2];

		TEST_ASSERT_EQUAL_INT(-EINVAL,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, ok, 2u,
					out, NULL));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
			stats_dev_sweep(STATS_DEV_ADEV, NULL, n, TAU0_NS, ok,
					2u, out, &got));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, NULL,
					2u, out, &got));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, ok, 2u,
					NULL, &got));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, ok, 0u,
					out, &got));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, n, 0u, ok, 2u, out,
					&got));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, desc,
					2u, out, &got));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, dup,
					2u, out, &got));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, zero,
					1u, out, &got));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, big,
					1u, out, &got));
		/* An out-of-domain kind must be refused, not silently treated
		 * as one of the two. */
		TEST_ASSERT_EQUAL_INT(-EINVAL,
			stats_dev_sweep((stats_dev_t)2, g_x, n, TAU0_NS, ok, 2u,
					out, &got));

		/* Out-of-range phase is caught before any point is written. */
		g_x[3] = STATS_PHASE_MAX_PS + 1;
		got = 99u;
		TEST_ASSERT_EQUAL_INT(-ERANGE,
			stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, ok, 2u,
					out, &got));
		TEST_ASSERT_EQUAL_UINT(0u, got);
		g_x[3] = 3;

		/* A rejected sweep must leave the count at zero, not at
		 * whatever the caller passed in. */
		got = 99u;
		(void)stats_dev_sweep(STATS_DEV_ADEV, g_x, n, TAU0_NS, desc, 2u,
				      out, &got);
		TEST_ASSERT_EQUAL_UINT(0u, got);
	}

	/* Histogram argument validation. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_hist(NULL, n, 0, 100, bins, 4u, &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_hist(g_x, n, 0, 100, bins, 4u, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_hist(g_x, 0u, 0, 100, bins, 4u, &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_hist(g_x, n, 0, 0, bins, 4u, &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_hist(g_x, n, 0, -100, bins, 4u, &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, stats_hist(g_x, n, 0, 100, bins, 0u, &h));

	/* A bin window whose edges are not representable is refused rather
	 * than wrapped. */
	TEST_ASSERT_EQUAL_INT(-ERANGE,
		stats_hist(g_x, n, STATS_PHASE_MAX_PS + 1, 100, bins, 4u, &h));
	TEST_ASSERT_EQUAL_INT(-ERANGE,
		stats_hist(g_x, n, 0, INT64_MAX / 2, bins, 4u, &h));
	/*
	 * The other overflow edge: 4 bins of INT64_MAX/4 do NOT overflow on
	 * their own (the product is INT64_MAX-3), so the width check passes and
	 * it is the window's *upper edge* that cannot be represented once any
	 * positive lo is added. Both guards are needed; this one reaches the
	 * second.
	 */
	TEST_ASSERT_EQUAL_INT(-ERANGE,
		stats_hist(g_x, n, 1000, INT64_MAX / 4, bins, 4u, &h));
}

/* ------------------------------------------------------------------- main */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_sqrt_known_and_reference);

	RUN_TEST(test_freq_offset_is_exactly_zero);
	RUN_TEST(test_drift_coefficient);
	RUN_TEST(test_alternating_phase_separates_mdev_from_adev);
	RUN_TEST(test_white_pm_adev_and_mdev_slopes);
	RUN_TEST(test_white_fm_adev_slope);

	RUN_TEST(test_hist_exact_distribution);
	RUN_TEST(test_hist_sawtooth_pair_is_the_qerr_proof);

	RUN_TEST(test_max_m_and_octaves);
	RUN_TEST(test_sweep_matches_single_calls_and_stops_cleanly);
	RUN_TEST(test_refusals);

	return UNITY_END();
}
