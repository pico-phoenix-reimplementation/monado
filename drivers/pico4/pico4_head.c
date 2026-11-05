// Copyright 2026, picoreverse contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Head position filter for the PICO 4, see pico4_head.h.
 * @ingroup drv_pico4
 */

#include "pico4_head.h"

#include <math.h>
#include <string.h>

//! Follow the m_imu_3dof heading every this many IMU samples (~10 Hz).
#define HEADING_EVERY 50

/*!
 * Overwrite the position. The snapshots still hold the old one and a later
 * delayed update would rewind to them and undo this: drop them (updates
 * older than now are then applied at the current time).
 */
static void
set_position(struct p4_head *h, const double p[3], double sigma_p, double sigma_v)
{
	p4_eskf_set_position(&h->f, p, sigma_p, sigma_v);
	for (int i = 0; i < P4_ESKF_SNAPS; i++) {
		h->f.snaps[i].idx = UINT64_MAX;
	}
}

void
p4_head_default_params(struct p4_head_params *prm)
{
	p4_eskf_default_params(&prm->eskf);
	// The head moves less violently than a hand, and the SLPI accel arrives
	// at 100 Hz repeated at the gyro rate: a little more accel noise.
	prm->eskf.sigma_a = 0.1;
	prm->eskf.sigma_ba = 1e-3;
	// Coast on the IMU for a while when SLAM drops a frame or two.
	prm->eskf.coast_s = 0.25;
	prm->eskf.damp_tau = 0.3;
	prm->eskf.gravity_after_s = 0.5;
	prm->sigma_pos = 0.008; // measured 2026-10-26: 5-14 mm per axis frame to frame at rest
	prm->gate = 30.0;       // NIS, 3 position dof (the orientation part is ~0, see below)
	prm->reject_accept = 3;
	prm->hold_after_s = 0.5;
	// Kalibr body frame (= IMU) -> head point (-0.024, 0.051, 0.07) of pico4-slam
	// (PICO4_SLAM_HEAD), in head axes (x = -y_b, y = x_b, z = z_b).
	prm->lever[0] = -0.051;
	prm->lever[1] = -0.024;
	prm->lever[2] = 0.07;
	prm->smooth_tau = 0.05;
}

static double
corr_decay(const struct p4_head *h, int64_t t)
{
	if (h->prm.smooth_tau <= 0.0 || h->corr_t == 0) {
		return 0.0;
	}
	double age = (double)(t - h->corr_t) * 1e-9;
	return age < 0.0 ? 1.0 : exp(-age / h->prm.smooth_tau);
}

void
p4_head_init(struct p4_head *h, const struct p4_head_params *prm)
{
	memset(h, 0, sizeof(*h));
	h->prm = *prm;
	const double bg0[3] = {0, 0, 0};
	p4_eskf_init(&h->f, &prm->eskf, bg0);
}

void
p4_head_imu(struct p4_head *h, int64_t t, const double w[3], const double a[3], const double q_fusion[4])
{
	uint64_t n0 = h->f.n;
	p4_eskf_imu(&h->f, t, w, a);
	if (h->f.n == n0) {
		return; // repeated or out of order
	}
	if (q_fusion != NULL && (h->heading_k++ % HEADING_EVERY) == 0) {
		p4_eskf_set_heading(&h->f, q_fusion);
	}
	// Long without SLAM: hold the position instead of integrating the accelerometer.
	if (h->have_pos && (double)(h->f.cur.t - h->last_upd_t) * 1e-9 > h->prm.hold_after_s) {
		if (!h->holding) {
			h->holding = true;
			h->holds++;
			memcpy(h->hold_p, h->f.cur.p, sizeof(h->hold_p));
		}
		set_position(h, h->hold_p, 0.05, 0.05);
	}
}

static enum p4_eskf_result
update(struct p4_head *h, int64_t t, const double z[3], double gate, double *nis)
{
	double R[3][3] = {{0}};
	for (int i = 0; i < 3; i++) {
		R[i][i] = h->prm.sigma_pos * h->prm.sigma_pos;
	}
	// Position only: the orientation "measurement" is the filter's own with a
	// huge sigma (the pose update is 6-dof), so it carries no information.
	enum p4_eskf_result r = p4_eskf_pose(&h->f, t, h->f.cur.q, z, R, 1e3, gate, P4_ESKF_UPDATE, nis, NULL, NULL);
	if (r == P4_ESKF_TOO_OLD) {
		h->too_old++;
		r = p4_eskf_pose(&h->f, h->f.cur.t, h->f.cur.q, z, R, 1e3, gate, P4_ESKF_UPDATE, nis, NULL, NULL);
	}
	return r;
}

void
p4_head_slam(struct p4_head *h, int64_t t, uint32_t map_id, const double p_head[3], const double q_head[4])
{
	if (h->f.n == 0) {
		return; // no IMU yet
	}
	// Head point -> IMU position.
	double rl[3], p_world[3];
	p4_q_rot(q_head, h->prm.lever, rl);
	for (int i = 0; i < 3; i++) {
		p_world[i] = p_head[i] - rl[i];
	}
	if (!h->have_map || h->map_id != map_id) {
		// New map: continue from where the head is now (or the standing height).
		double here[3] = {0.0, 1.6, 0.0};
		if (h->have_pos) {
			memcpy(here, h->f.cur.p, sizeof(here));
		}
		for (int i = 0; i < 3; i++) {
			h->anchor[i] = p_world[i] - here[i];
		}
		set_position(h, here, 0.005, h->have_pos ? 0.05 : 0.1);
		h->have_pos = h->have_map = true;
		h->map_id = map_id;
		h->last_upd_t = h->f.cur.t;
		h->holding = false;
		h->rejects = 0;
		h->anchors++;
		return;
	}

	double z[3] = {p_world[0] - h->anchor[0], p_world[1] - h->anchor[1], p_world[2] - h->anchor[2]};
	bool resume = h->holding;
	double before[3];
	memcpy(before, h->f.cur.p, sizeof(before));
	double nis = 0;
	enum p4_eskf_result r;
	if (resume) {
		// After a hold the position is whatever SLAM says (as the old filter did).
		set_position(h, z, 0.01, 0.1);
		memset(h->corr, 0, sizeof(h->corr));
		r = P4_ESKF_APPLIED;
		h->holding = false;
	} else {
		r = update(h, t, z, h->prm.gate, &nis);
		if (r == P4_ESKF_REJECTED) {
			h->rejected++;
			if (++h->rejects >= h->prm.reject_accept) {
				// SLAM keeps disagreeing: it jumped (relocalised), follow it.
				r = update(h, t, z, 0.0, &nis);
				h->forced++;
			}
		}
	}
	if (r == P4_ESKF_APPLIED && !resume && h->prm.smooth_tau > 0.0) {
		// Show the jump of this update gradually.
		double k = corr_decay(h, h->f.cur.t);
		for (int i = 0; i < 3; i++) {
			h->corr[i] = h->corr[i] * k + before[i] - h->f.cur.p[i];
		}
		h->corr_t = h->f.cur.t;
	}
	if (r == P4_ESKF_APPLIED) {
		h->applied++;
		h->nis_sum += nis;
		h->rejects = 0;
		h->last_upd_t = h->f.cur.t;
	}
}

void
p4_head_slam_lost(struct p4_head *h, int64_t arrival)
{
	(void)h;
	(void)arrival;
	// Nothing to do: the filter coasts on the IMU, damps the velocity after
	// coast_s and holds the position after hold_after_s.
}

bool
p4_head_position(const struct p4_head *h, int64_t t, const double q_head[4], double p[3], double v[3])
{
	if (!h->have_pos) {
		return false;
	}
	const struct p4_eskf_core *c = &h->f.cur;
	double dt = (double)(t - c->t) * 1e-9;
	dt = dt < 0.0 ? 0.0 : (dt > 0.1 ? 0.1 : dt);
	double rl[3], w_world[3], wxr[3];
	p4_q_rot(q_head, h->prm.lever, rl);
	// Velocity of the head point = IMU velocity + omega x lever (omega in world).
	p4_q_rot(c->q, c->w, w_world);
	wxr[0] = w_world[1] * rl[2] - w_world[2] * rl[1];
	wxr[1] = w_world[2] * rl[0] - w_world[0] * rl[2];
	wxr[2] = w_world[0] * rl[1] - w_world[1] * rl[0];
	double k = corr_decay(h, t);
	for (int i = 0; i < 3; i++) {
		p[i] = c->p[i] + c->v[i] * dt + rl[i] + h->corr[i] * k;
		v[i] = c->v[i] + wxr[i];
	}
	return true;
}

void
p4_head_print_stats(const struct p4_head *h, FILE *out)
{
	fprintf(out, "head eskf: applied %u (mean NIS %.2f), rejected %u, forced %u, too old %u, anchors %u, holds %u\n", h->applied,
	        h->applied ? h->nis_sum / h->applied : 0.0, h->rejected, h->forced, h->too_old, h->anchors, h->holds);
}
