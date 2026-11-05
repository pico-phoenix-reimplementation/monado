// Copyright 2026, picoreverse contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Error-state Kalman filter for one PICO 4 controller, see pico4_ctrl_eskf.h.
 * @ingroup drv_pico4
 */

#include "pico4_ctrl_eskf.h"

#include <math.h>
#include <string.h>

#define N P4_ESKF_N
#define IT 0  // dtheta
#define IP 3  // dp
#define IV 6  // dv
#define IBG 9 // dbg
#define IBA 12

#define GRAVITY 9.80665

//! Stillness: |gyro - bias| below this (rad/s, ~1.5 deg/s)...
#define STILL_GYRO 0.026
//! ...|accel| within this of 1 g and within this of its 50 ms average (m/s^2)...
#define STILL_ACCEL 0.4
#define STILL_JITTER 0.2
//! ...for this long (s).
#define STILL_S 0.2
//! Pseudo-measurements (stillness, gravity) every this many samples (~50 Hz).
#define PSEUDO_EVERY 10


/*
 *
 * Small math.
 *
 */

void
p4_q_mul(const double a[4], const double b[4], double out[4])
{
	double x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
	double y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
	double z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
	double w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
	out[0] = x;
	out[1] = y;
	out[2] = z;
	out[3] = w;
}

void
p4_q_conj(const double q[4], double out[4])
{
	out[0] = -q[0];
	out[1] = -q[1];
	out[2] = -q[2];
	out[3] = q[3];
}

static void
q_normalize(double q[4])
{
	double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	for (int i = 0; i < 4; i++) {
		q[i] /= n;
	}
	if (q[3] < 0) {
		for (int i = 0; i < 4; i++) {
			q[i] = -q[i];
		}
	}
}

static void
q_exp(const double v[3], double q[4])
{
	double a = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	double s = a < 1e-9 ? 0.5 : sin(0.5 * a) / a;
	q[0] = v[0] * s;
	q[1] = v[1] * s;
	q[2] = v[2] * s;
	q[3] = cos(0.5 * a);
}

static void
q_log(const double q_in[4], double v[3])
{
	double q[4] = {q_in[0], q_in[1], q_in[2], q_in[3]};
	if (q[3] < 0) {
		for (int i = 0; i < 4; i++) {
			q[i] = -q[i];
		}
	}
	double s = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
	double k = s < 1e-9 ? 2.0 : 2.0 * atan2(s, q[3]) / s;
	v[0] = q[0] * k;
	v[1] = q[1] * k;
	v[2] = q[2] * k;
}

static void
q_to_mat(const double q[4], double R[3][3])
{
	double x = q[0], y = q[1], z = q[2], w = q[3];
	R[0][0] = 1 - 2 * (y * y + z * z);
	R[0][1] = 2 * (x * y - z * w);
	R[0][2] = 2 * (x * z + y * w);
	R[1][0] = 2 * (x * y + z * w);
	R[1][1] = 1 - 2 * (x * x + z * z);
	R[1][2] = 2 * (y * z - x * w);
	R[2][0] = 2 * (x * z - y * w);
	R[2][1] = 2 * (y * z + x * w);
	R[2][2] = 1 - 2 * (x * x + y * y);
}

void
p4_q_rot(const double q[4], const double v[3], double out[3])
{
	double R[3][3];
	q_to_mat(q, R);
	for (int i = 0; i < 3; i++) {
		out[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
	}
}

double
p4_q_angle(const double a[4], const double b[4])
{
	double d = fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
	return 2.0 * acos(d > 1.0 ? 1.0 : d);
}

static double
norm3(const double v[3])
{
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

//! Shortest rotation taking unit vector a onto unit vector b.
static void
q_from_two(const double a[3], const double b[3], double q[4])
{
	double c[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
	double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	if (d < -0.999999) {
		// Opposite: any axis orthogonal to a.
		double ax[3] = {1, 0, 0};
		if (fabs(a[0]) > 0.9) {
			ax[0] = 0;
			ax[2] = 1;
		}
		double o[3] = {a[1] * ax[2] - a[2] * ax[1], a[2] * ax[0] - a[0] * ax[2], a[0] * ax[1] - a[1] * ax[0]};
		double n = norm3(o);
		q[0] = o[0] / n;
		q[1] = o[1] / n;
		q[2] = o[2] / n;
		q[3] = 0;
		return;
	}
	q[0] = c[0];
	q[1] = c[1];
	q[2] = c[2];
	q[3] = 1.0 + d;
	q_normalize(q);
}

//! Rotation about +y (yaw) part of q, swing-twist decomposition.
static void
q_twist_y(const double q[4], double out[4])
{
	out[0] = 0;
	out[1] = q[1];
	out[2] = 0;
	out[3] = q[3];
	double n = sqrt(out[1] * out[1] + out[3] * out[3]);
	if (n < 1e-9) {
		out[1] = 0;
		out[3] = 1;
		return;
	}
	out[1] /= n;
	out[3] /= n;
}

static void
skew(const double v[3], double S[3][3])
{
	S[0][0] = 0;
	S[0][1] = -v[2];
	S[0][2] = v[1];
	S[1][0] = v[2];
	S[1][1] = 0;
	S[1][2] = -v[0];
	S[2][0] = -v[1];
	S[2][1] = v[0];
	S[2][2] = 0;
}

//! Cholesky of the m x m SPD matrix A (in place, lower). False if not positive definite.
static bool
chol(double A[6][6], int m)
{
	for (int j = 0; j < m; j++) {
		double s = A[j][j];
		for (int k = 0; k < j; k++) {
			s -= A[j][k] * A[j][k];
		}
		if (s <= 1e-30) {
			return false;
		}
		A[j][j] = sqrt(s);
		for (int i = j + 1; i < m; i++) {
			double t = A[i][j];
			for (int k = 0; k < j; k++) {
				t -= A[i][k] * A[j][k];
			}
			A[i][j] = t / A[j][j];
		}
	}
	return true;
}

//! Solve L L^T x = b in place.
static void
chol_solve(const double L[6][6], int m, double b[6])
{
	for (int i = 0; i < m; i++) {
		double t = b[i];
		for (int k = 0; k < i; k++) {
			t -= L[i][k] * b[k];
		}
		b[i] = t / L[i][i];
	}
	for (int i = m - 1; i >= 0; i--) {
		double t = b[i];
		for (int k = i + 1; k < m; k++) {
			t -= L[k][i] * b[k];
		}
		b[i] = t / L[i][i];
	}
}


/*
 *
 * Filter core.
 *
 */

static void
symmetrize(double P[N][N])
{
	for (int i = 0; i < N; i++) {
		for (int j = i + 1; j < N; j++) {
			double m = 0.5 * (P[i][j] + P[j][i]);
			P[i][j] = m;
			P[j][i] = m;
		}
		if (P[i][i] < 1e-18) {
			P[i][i] = 1e-18;
		}
	}
}

static void
inject(struct p4_eskf_core *c, const double dx[N])
{
	double dq[4], q[4];
	q_exp(&dx[IT], dq);
	p4_q_mul(c->q, dq, q);
	q_normalize(q);
	memcpy(c->q, q, sizeof(q));
	for (int i = 0; i < 3; i++) {
		c->p[i] += dx[IP + i];
		c->v[i] += dx[IV + i];
		c->bg[i] += dx[IBG + i];
		c->ba[i] += dx[IBA + i];
	}
}

/*!
 * Generic update: residual r (m <= 6), H (m x N), R (m x m). Rejected (and
 * nothing changed) when gate > 0 and the NIS exceeds it.
 */
static bool
update(struct p4_eskf_core *c, int m, const double r[6], const double H[6][N], const double R[6][6], double gate, double *out_nis)
{
	double PHt[N][6];
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < m; j++) {
			double s = 0;
			for (int k = 0; k < N; k++) {
				if (H[j][k] != 0.0) {
					s += c->P[i][k] * H[j][k];
				}
			}
			PHt[i][j] = s;
		}
	}
	double S[6][6];
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < m; j++) {
			double s = R[i][j];
			for (int k = 0; k < N; k++) {
				if (H[i][k] != 0.0) {
					s += H[i][k] * PHt[k][j];
				}
			}
			S[i][j] = s;
		}
	}
	if (!chol(S, m)) {
		return false;
	}
	double y[6];
	memcpy(y, r, sizeof(double) * (size_t)m);
	chol_solve(S, m, y);
	double nis = 0;
	for (int i = 0; i < m; i++) {
		nis += r[i] * y[i];
	}
	if (out_nis != NULL) {
		*out_nis = nis;
	}
	if (gate > 0.0 && nis > gate) {
		return false;
	}

	// K = PHt S^-1, row by row.
	double K[N][6];
	for (int i = 0; i < N; i++) {
		double row[6];
		memcpy(row, PHt[i], sizeof(row));
		chol_solve(S, m, row);
		memcpy(K[i], row, sizeof(row));
	}
	double dx[N];
	for (int i = 0; i < N; i++) {
		double s = 0;
		for (int j = 0; j < m; j++) {
			s += K[i][j] * r[j];
		}
		dx[i] = s;
	}
	// P -= K (H P) = K PHt^T
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			double s = 0;
			for (int k = 0; k < m; k++) {
				s += K[i][k] * PHt[j][k];
			}
			c->P[i][j] -= s;
		}
	}
	symmetrize(c->P);
	inject(c, dx);
	return true;
}

static void
propagate(const struct p4_eskf_params *prm, struct p4_eskf_core *c, double dt, const double wm[3], const double am[3])
{
	double w[3], a[3];
	for (int i = 0; i < 3; i++) {
		w[i] = wm[i] - c->bg[i];
		a[i] = am[i] - c->ba[i];
	}
	memcpy(c->w, w, sizeof(w));

	double R[3][3];
	q_to_mat(c->q, R);
	double aw[3];
	for (int i = 0; i < 3; i++) {
		aw[i] = R[i][0] * a[0] + R[i][1] * a[1] + R[i][2] * a[2];
	}
	aw[1] -= GRAVITY;

	// Out of the cameras' view the velocity decays (no open-loop double integration).
	double e = 1.0;
	if (c->last_opt_t == 0 || (double)(c->t - c->last_opt_t) * 1e-9 > prm->coast_s) {
		e = exp(-dt / prm->damp_tau);
	}
	for (int i = 0; i < 3; i++) {
		c->p[i] += c->v[i] * dt + 0.5 * aw[i] * dt * dt;
		c->v[i] = (c->v[i] + aw[i] * dt) * e;
	}
	double wdt[3] = {w[0] * dt, w[1] * dt, w[2] * dt}, dq[4], q[4];
	q_exp(wdt, dq);
	p4_q_mul(c->q, dq, q);
	q_normalize(q);
	memcpy(c->q, q, sizeof(q));

	// F, block by block (zero elsewhere).
	static double F[N][N];
	memset(F, 0, sizeof(F));
	double Rd[3][3], Sa[3][3], RSa[3][3];
	q_to_mat(dq, Rd);
	skew(a, Sa);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			RSa[i][j] = R[i][0] * Sa[0][j] + R[i][1] * Sa[1][j] + R[i][2] * Sa[2][j];
		}
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			F[IT + i][IT + j] = Rd[j][i]; // Exp(w dt)^T
			F[IV + i][IT + j] = -RSa[i][j] * dt;
			F[IV + i][IBA + j] = -R[i][j] * dt;
		}
		F[IT + i][IBG + i] = -dt;
		F[IP + i][IP + i] = 1.0;
		F[IP + i][IV + i] = dt;
		F[IV + i][IV + i] = e;
		F[IBG + i][IBG + i] = 1.0;
		F[IBA + i][IBA + i] = 1.0;
	}

	// P = F P F^T + Q, skipping F's zeros.
	static double FP[N][N];
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			double s = 0;
			for (int k = 0; k < N; k++) {
				if (F[i][k] != 0.0) {
					s += F[i][k] * c->P[k][j];
				}
			}
			FP[i][j] = s;
		}
	}
	for (int i = 0; i < N; i++) {
		for (int j = i; j < N; j++) {
			double s = 0;
			for (int k = 0; k < N; k++) {
				if (F[j][k] != 0.0) {
					s += FP[i][k] * F[j][k];
				}
			}
			c->P[i][j] = s;
			c->P[j][i] = s;
		}
	}
	for (int i = 0; i < 3; i++) {
		c->P[IT + i][IT + i] += prm->sigma_g * prm->sigma_g * dt;
		c->P[IV + i][IV + i] += prm->sigma_a * prm->sigma_a * dt;
		c->P[IBG + i][IBG + i] += prm->sigma_bg * prm->sigma_bg * dt;
		c->P[IBA + i][IBA + i] += prm->sigma_ba * prm->sigma_ba * dt;
	}
}

static void
pseudo_updates(const struct p4_eskf_params *prm, struct p4_eskf_core *c, double dt, const double wm[3], const double am[3])
{
	double a[3] = {am[0] - c->ba[0], am[1] - c->ba[1], am[2] - c->ba[2]};
	double an = norm3(a);
	double k = dt / 0.05 > 1.0 ? 1.0 : dt / 0.05;
	double jit = 0;
	for (int i = 0; i < 3; i++) {
		c->a_ema[i] += (am[i] - c->a_ema[i]) * k;
		jit += (am[i] - c->a_ema[i]) * (am[i] - c->a_ema[i]);
	}
	bool quiet = norm3(c->w) < STILL_GYRO && fabs(an - GRAVITY) < STILL_ACCEL && sqrt(jit) < STILL_JITTER;
	if (quiet) {
		c->still_s += dt;
		for (int i = 0; i < 3; i++) {
			c->gsum[i] += wm[i];
		}
		c->gn++;
	} else {
		c->still_s = 0;
		c->gn = 0;
		c->gsum[0] = c->gsum[1] = c->gsum[2] = 0;
	}

	if (++c->k % PSEUDO_EVERY != 0) {
		return;
	}
	bool still = c->still_s >= STILL_S;
	double H[6][N], R[6][6], r[6];

	if (still) {
		// Zero velocity...
		memset(H, 0, sizeof(H));
		memset(R, 0, sizeof(R));
		for (int i = 0; i < 3; i++) {
			r[i] = -c->v[i];
			H[i][IV + i] = 1.0;
			R[i][i] = 0.01 * 0.01;
		}
		update(c, 3, r, H, R, 0.0, NULL);
		// ...and zero rate: the mean gyro is the bias.
		if (c->gn >= PSEUDO_EVERY) {
			memset(H, 0, sizeof(H));
			memset(R, 0, sizeof(R));
			for (int i = 0; i < 3; i++) {
				r[i] = c->gsum[i] / c->gn - c->bg[i];
				H[i][IBG + i] = 1.0;
				R[i][i] = 1.5e-3 * 1.5e-3;
			}
			update(c, 3, r, H, R, 0.0, NULL);
			c->gn = 0;
			c->gsum[0] = c->gsum[1] = c->gsum[2] = 0;
		}
	}

	// Without optical updates, tilt from gravity (weak: the hand accelerates).
	bool no_opt = c->last_opt_t == 0 || (double)(c->t - c->last_opt_t) * 1e-9 > prm->gravity_after_s;
	if (no_opt && norm3(c->w) < 1.0 && fabs(an - GRAVITY) < 1.0) {
		double qc[4], u[3], up[3] = {0, 1, 0}, S[3][3];
		p4_q_conj(c->q, qc);
		p4_q_rot(qc, up, u); // predicted up in I
		skew(u, S);
		memset(H, 0, sizeof(H));
		memset(R, 0, sizeof(R));
		double sig = still ? 0.05 : 0.3;
		for (int i = 0; i < 3; i++) {
			r[i] = a[i] / an - u[i];
			for (int j = 0; j < 3; j++) {
				H[i][IT + j] = S[i][j];
			}
			R[i][i] = sig * sig;
		}
		update(c, 3, r, H, R, 0.0, NULL);
	}
}

static void
init_from_accel(struct p4_eskf *f, int64_t t, const double a[3])
{
	struct p4_eskf_core *c = &f->cur;
	memset(c, 0, sizeof(*c));
	c->t = t;
	double an = norm3(a), u[3] = {a[0] / an, a[1] / an, a[2] / an}, up[3] = {0, 1, 0};
	q_from_two(u, up, c->q); // body "up" (accel) -> world up
	memcpy(c->bg, f->bg0, sizeof(c->bg));
	memcpy(c->a_ema, a, sizeof(c->a_ema));
	double tilt = 0.05, pos = 1.0, vel = 0.1, bg = 0.5 * M_PI / 180.0, ba = 0.15;
	for (int i = 0; i < 3; i++) {
		c->P[IT + i][IT + i] = tilt * tilt;
		c->P[IP + i][IP + i] = pos * pos;
		c->P[IV + i][IV + i] = vel * vel;
		c->P[IBG + i][IBG + i] = bg * bg;
		c->P[IBA + i][IBA + i] = ba * ba;
	}
}

//! Process ring sample @p idx on f->cur, snapshot when due.
static void
step(struct p4_eskf *f, uint64_t idx)
{
	const struct p4_eskf_sample *s = &f->ring[idx % P4_ESKF_RING];
	double w[3] = {s->w[0], s->w[1], s->w[2]}, a[3] = {s->a[0], s->a[1], s->a[2]};
	double dt = (double)(s->t - f->cur.t) * 1e-9;
	if (dt > 0.0) {
		if (dt > 0.05) {
			dt = 0.05; // a gap in the stream: do not integrate it
		}
		propagate(&f->prm, &f->cur, dt, w, a);
		pseudo_updates(&f->prm, &f->cur, dt, w, a);
	}
	f->cur.t = s->t;
	if (idx % P4_ESKF_SNAP_EVERY == 0) {
		struct p4_eskf_snap *sn = &f->snaps[(idx / P4_ESKF_SNAP_EVERY) % P4_ESKF_SNAPS];
		sn->idx = idx;
		sn->core = f->cur;
	}
}


/*
 *
 * API.
 *
 */

void
p4_eskf_default_params(struct p4_eskf_params *prm)
{
	prm->sigma_g = 3e-4;
	prm->sigma_a = 4e-3;
	prm->sigma_bg = 2e-5;
	prm->sigma_ba = 2e-4;
	prm->coast_s = 0.15;
	prm->damp_tau = 0.3;
	prm->gravity_after_s = 0.5;
}

void
p4_eskf_init(struct p4_eskf *f, const struct p4_eskf_params *prm, const double bg0[3])
{
	memset(f, 0, sizeof(*f));
	f->prm = *prm;
	memcpy(f->bg0, bg0, sizeof(f->bg0));
	f->cur.q[3] = 1.0;
	for (int i = 0; i < P4_ESKF_SNAPS; i++) {
		f->snaps[i].idx = UINT64_MAX;
	}
}

void
p4_eskf_imu(struct p4_eskf *f, int64_t t, const double w[3], const double a[3])
{
	if (f->n > 0 && t <= f->cur.t) {
		return; // repeated or out of order
	}
	if (f->n == 0) {
		init_from_accel(f, t, a);
	}
	struct p4_eskf_sample *s = &f->ring[f->n % P4_ESKF_RING];
	s->t = t;
	for (int i = 0; i < 3; i++) {
		s->w[i] = (float)w[i];
		s->a[i] = (float)a[i];
	}
	step(f, f->n);
	f->n++;
}

static void
reset_block(struct p4_eskf_core *c, int at, double sigma)
{
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < 3; j++) {
			c->P[at + j][i] = 0.0;
			c->P[i][at + j] = 0.0;
		}
	}
	for (int j = 0; j < 3; j++) {
		c->P[at + j][at + j] = sigma * sigma;
	}
}

static enum p4_eskf_result
apply_pose(struct p4_eskf *f,
           const double q[4],
           const double p[3],
           const double Rp[3][3],
           double sig_th,
           double gate,
           enum p4_eskf_pose_mode mode,
           double *out_nis)
{
	struct p4_eskf_core *c = &f->cur;
	if (mode == P4_ESKF_RESET) {
		memcpy(c->q, q, sizeof(c->q));
		q_normalize(c->q);
		memcpy(c->p, p, sizeof(c->p));
		memset(c->v, 0, sizeof(c->v));
		reset_block(c, IT, 5.0 * M_PI / 180.0);
		reset_block(c, IP, 0.02);
		reset_block(c, IV, 0.5);
		c->last_opt_t = c->t;
		return P4_ESKF_APPLIED;
	}

	double H[6][N], R[6][6], r[6], qc[4], dq[4];
	memset(H, 0, sizeof(H));
	memset(R, 0, sizeof(R));
	p4_q_conj(c->q, qc);
	p4_q_mul(qc, q, dq);
	q_log(dq, &r[3]);

	if (mode == P4_ESKF_RESET_POS) {
		memcpy(c->p, p, sizeof(c->p));
		memset(c->v, 0, sizeof(c->v));
		reset_block(c, IP, 0.02);
		reset_block(c, IV, 0.3);
		c->last_opt_t = c->t;
		double r3[6] = {r[3], r[4], r[5]};
		for (int i = 0; i < 3; i++) {
			H[i][IT + i] = 1.0;
			R[i][i] = sig_th * sig_th;
		}
		update(c, 3, r3, H, R, gate, out_nis);
		return P4_ESKF_APPLIED;
	}

	for (int i = 0; i < 3; i++) {
		r[i] = p[i] - c->p[i];
		H[i][IP + i] = 1.0;
		H[3 + i][IT + i] = 1.0;
		for (int j = 0; j < 3; j++) {
			R[i][j] = Rp[i][j];
		}
		R[3 + i][3 + i] = sig_th * sig_th;
	}
	if (!update(c, 6, r, H, R, gate, out_nis)) {
		return P4_ESKF_REJECTED;
	}
	c->last_opt_t = c->t;
	return P4_ESKF_APPLIED;
}

enum p4_eskf_result
p4_eskf_pose(struct p4_eskf *f,
             int64_t t,
             const double q[4],
             const double p[3],
             const double Rp[3][3],
             double sig_th,
             double gate,
             enum p4_eskf_pose_mode mode,
             double *out_nis,
             double q_pred[4],
             double p_pred[3])
{
	if (out_nis != NULL) {
		*out_nis = 0.0;
	}
	if (f->n == 0) {
		return P4_ESKF_NO_IMU;
	}

	// Newer than the newest IMU sample (or within it): apply to the current state.
	if (t >= f->cur.t) {
		if (q_pred != NULL) {
			memcpy(q_pred, f->cur.q, sizeof(double) * 4);
		}
		if (p_pred != NULL) {
			memcpy(p_pred, f->cur.p, sizeof(double) * 3);
		}
		return apply_pose(f, q, p, Rp, sig_th, gate, mode, out_nis);
	}

	// Rewind to the newest snapshot not after t...
	const struct p4_eskf_snap *best = NULL;
	uint64_t oldest_ok = f->n > P4_ESKF_RING ? f->n - P4_ESKF_RING : 0;
	for (int i = 0; i < P4_ESKF_SNAPS; i++) {
		const struct p4_eskf_snap *sn = &f->snaps[i];
		if (sn->idx == UINT64_MAX || sn->idx >= f->n || sn->idx < oldest_ok || sn->core.t > t) {
			continue;
		}
		if (best == NULL || sn->idx > best->idx) {
			best = sn;
		}
	}
	if (best == NULL) {
		f->too_old++;
		return P4_ESKF_TOO_OLD;
	}

	static struct p4_eskf_core backup; // big (P), keep it off the (musl) thread stack
	backup = f->cur;
	f->cur = best->core;
	uint64_t i = best->idx + 1;
	// ...replay up to t...
	while (i < f->n && f->ring[i % P4_ESKF_RING].t <= t) {
		step(f, i++);
	}
	if (q_pred != NULL) {
		memcpy(q_pred, f->cur.q, sizeof(double) * 4);
	}
	if (p_pred != NULL) {
		memcpy(p_pred, f->cur.p, sizeof(double) * 3);
	}
	enum p4_eskf_result res = apply_pose(f, q, p, Rp, sig_th, gate, mode, out_nis);
	if (res != P4_ESKF_APPLIED) {
		f->cur = backup;
		return res;
	}
	// ...and on to the newest sample.
	while (i < f->n) {
		step(f, i++);
	}
	f->replays++;
	return P4_ESKF_APPLIED;
}

static void
turn_heading(struct p4_eskf_core *c, const double q_target[4])
{
	double tw_t[4], tw_c[4], inv[4], d[4], q[4];
	q_twist_y(q_target, tw_t);
	q_twist_y(c->q, tw_c);
	p4_q_conj(tw_c, inv);
	p4_q_mul(tw_t, inv, d);
	p4_q_mul(d, c->q, q);
	q_normalize(q);
	memcpy(c->q, q, sizeof(q));
}

void
p4_eskf_set_heading(struct p4_eskf *f, const double q_target[4])
{
	turn_heading(&f->cur, q_target);
	for (int i = 0; i < P4_ESKF_SNAPS; i++) {
		if (f->snaps[i].idx != UINT64_MAX) {
			turn_heading(&f->snaps[i].core, q_target);
		}
	}
}

void
p4_eskf_set_position(struct p4_eskf *f, const double p[3], double sigma_p, double sigma_v)
{
	memcpy(f->cur.p, p, sizeof(f->cur.p));
	memset(f->cur.v, 0, sizeof(f->cur.v));
	reset_block(&f->cur, IP, sigma_p);
	reset_block(&f->cur, IV, sigma_v);
}

bool
p4_eskf_still(const struct p4_eskf *f)
{
	return f->cur.still_s >= STILL_S;
}

void
p4_eskf_sigmas(const struct p4_eskf *f, double *sig_rot, double *sig_pos)
{
	const struct p4_eskf_core *c = &f->cur;
	*sig_rot = sqrt((c->P[IT][IT] + c->P[IT + 1][IT + 1] + c->P[IT + 2][IT + 2]) / 3.0);
	*sig_pos = sqrt((c->P[IP][IP] + c->P[IP + 1][IP + 1] + c->P[IP + 2][IP + 2]) / 3.0);
}


/*
 *
 * Clock mapping.
 *
 */

//! Window of the arrival - stamp minimum.
#define CLOCK_WINDOW_NS 2000000000LL

void
p4_clock_init(struct p4_clock *c)
{
	memset(c, 0, sizeof(*c));
	c->win_min = INT64_MAX;
	c->prev_min = INT64_MAX;
}

int64_t
p4_clock_ssc(struct p4_clock *c, int64_t ssc_ns, int64_t arrival_ns)
{
	int64_t d = arrival_ns - ssc_ns;
	if (c->win_start == 0 || arrival_ns - c->win_start > CLOCK_WINDOW_NS) {
		c->prev_min = c->win_min;
		c->win_min = d;
		c->win_start = arrival_ns;
	} else if (d < c->win_min) {
		c->win_min = d;
	}
	int64_t off = c->win_min < c->prev_min ? c->win_min : c->prev_min;
	return ssc_ns + off;
}

int64_t
p4_clock_ctrl(struct p4_clock *c, uint32_t ctrl_us, int64_t ssc_mono_ns)
{
	if (ctrl_us == 0) {
		return ssc_mono_ns;
	}
	if (!c->have_ctrl) {
		c->have_ctrl = true;
		c->last_us = ctrl_us;
		c->ctrl_ns = 0;
		c->ctrl_off = (double)ssc_mono_ns;
		return ssc_mono_ns;
	}
	c->ctrl_ns += (int64_t)(uint32_t)(ctrl_us - c->last_us) * 1000;
	c->last_us = ctrl_us;
	double d = (double)ssc_mono_ns - (double)c->ctrl_ns - c->ctrl_off;
	if (fabs(d) > 50e6) {
		// Controller restarted or reconnected: start over from the SSC time.
		c->ctrl_ns = 0;
		c->ctrl_off = (double)ssc_mono_ns;
		return ssc_mono_ns;
	}
	c->ctrl_off += d * 0.002; // ~1 s at 500 Hz
	return c->ctrl_ns + (int64_t)c->ctrl_off;
}


/*
 *
 * Optical fix policy.
 *
 */

//! chi^2 99.9 % for 6 and 3 degrees of freedom.
#define GATE6 22.46
#define GATE3 16.27
//! Relock after this many good fixes in a row that the filter rejects but that agree with each other.
#define RELOCK_COUNT 3

void
p4_track_init(struct p4_ctrl_track *tr, const struct p4_eskf_params *prm, const double bg0[3])
{
	memset(tr, 0, sizeof(*tr));
	p4_eskf_init(&tr->f, prm, bg0);
}

enum p4_track_fix
p4_track_fix(struct p4_ctrl_track *tr,
             int64_t t,
             const double q[4],
             const double p[3],
             const double ray[3],
             int inliers,
             double rms,
             double *out_nis,
             double out_qp[4],
             double out_pp[3])
{
	bool good = inliers >= 10 && rms < 1.0;
	bool weak = inliers < 8 || rms > 1.2;
	double k = weak ? 2.0 : 1.0;
	// Position: 2 mm across, 6 mm along the viewing ray, plus ~3 mm for the head pose.
	double sl = 0.002 * k, sr = 0.006 * k, sh = 0.003;
	double Rp[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Rp[i][j] = (sr * sr - sl * sl) * ray[i] * ray[j] + (i == j ? sl * sl + sh * sh : 0.0);
		}
	}
	double sth = (1.5 * k + 0.3) * M_PI / 180.0;
	double nis = 0, qp[4], pp[3];
	if (out_nis != NULL) {
		*out_nis = 0;
	}

	if (!tr->locked) {
		if (inliers < 6 || rms > 1.8) {
			tr->ignored++;
			return P4_FIX_IGNORED;
		}
		if (p4_eskf_pose(&tr->f, t, q, p, Rp, sth, 0, P4_ESKF_RESET, NULL, NULL, NULL) != P4_ESKF_APPLIED) {
			tr->ignored++;
			return P4_FIX_IGNORED;
		}
		tr->locked = true;
		tr->last_fix_t = t;
		tr->disagree = 0;
		return P4_FIX_LOCKED;
	}

	bool reacq = t - tr->last_fix_t > P4_TRACK_REACQ_NS;
	enum p4_eskf_pose_mode mode = P4_ESKF_UPDATE;
	double gate = GATE6;
	if (reacq) {
		mode = good ? P4_ESKF_RESET : P4_ESKF_RESET_POS;
		gate = 0.0;
	}
	enum p4_eskf_result res = p4_eskf_pose(&tr->f, t, q, p, Rp, sth, gate,
	                                       mode, &nis, qp, pp);
	if (out_nis != NULL) {
		*out_nis = nis;
	}
	if (out_qp != NULL) {
		memcpy(out_qp, qp, sizeof(qp));
	}
	if (out_pp != NULL) {
		memcpy(out_pp, pp, sizeof(pp));
	}
	if (res == P4_ESKF_TOO_OLD || res == P4_ESKF_NO_IMU) {
		tr->ignored++;
		return P4_FIX_IGNORED;
	}
	if (res == P4_ESKF_APPLIED) {
		tr->disagree = 0;
		if (t > tr->last_fix_t) {
			tr->last_fix_t = t;
		}
		if (reacq) {
			tr->held_reacq++;
			return P4_FIX_REACQUIRED;
		}
		tr->applied++;
		tr->nis_sum += nis;
		tr->nis_n++;
		return P4_FIX_APPLIED;
	}

	// Rejected. If good fixes keep disagreeing with the filter but agree with
	// each other (their relative motion matches the IMU's), the filter is wrong.
	tr->rejected++;
	if (!good) {
		return P4_FIX_REJECTED;
	}
	bool agree = false;
	if (tr->disagree > 0 && t - tr->rej_t < 500000000LL) {
		double c1[4], dm[4], c2[4], dp[4];
		p4_q_conj(tr->rej_q, c1);
		p4_q_mul(c1, q, dm);
		p4_q_conj(tr->rej_qp, c2);
		p4_q_mul(c2, qp, dp);
		double ang = p4_q_angle(dm, dp);
		double e = 0;
		for (int i = 0; i < 3; i++) {
			double d = (p[i] - tr->rej_p[i]) - (pp[i] - tr->rej_pp[i]);
			e += d * d;
		}
		agree = ang < 5.0 * M_PI / 180.0 && sqrt(e) < 0.05;
	}
	tr->disagree = agree ? tr->disagree + 1 : 1;
	tr->rej_t = t;
	memcpy(tr->rej_q, q, sizeof(tr->rej_q));
	memcpy(tr->rej_p, p, sizeof(tr->rej_p));
	memcpy(tr->rej_qp, qp, sizeof(tr->rej_qp));
	memcpy(tr->rej_pp, pp, sizeof(tr->rej_pp));
	if (tr->disagree >= RELOCK_COUNT &&
	    p4_eskf_pose(&tr->f, t, q, p, Rp, sth, 0, P4_ESKF_RESET, NULL, NULL, NULL) == P4_ESKF_APPLIED) {
		tr->disagree = 0;
		tr->relocks++;
		tr->last_fix_t = t > tr->last_fix_t ? t : tr->last_fix_t;
		return P4_FIX_RELOCKED;
	}
	return P4_FIX_REJECTED;
}
