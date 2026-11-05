// Copyright 2026, picoreverse contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Head position filter for the PICO 4: the controller ESKF
 *         (pico4_ctrl_eskf) driven by the head IMU, with SLAM positions as
 *         delayed updates (rewind and replay).
 *
 * The SLAM position comes in already rotated into the head IMU world (the
 * driver's SLAM -> IMU alignment); this filter keeps its own anchor so the
 * position stays continuous across new SLAM maps. Only the position is used
 * from SLAM: the displayed orientation stays with m_imu_3dof, whose heading
 * the filter follows so both share one world.
 *
 * Plain C, built by the driver and by slam/ctrl/replay/head_replay.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include "pico4_ctrl_eskf.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

struct p4_head_params
{
	struct p4_eskf_params eskf;
	double sigma_pos;   //!< SLAM position noise, m (per axis)
	double gate;        //!< NIS gate for SLAM updates (0 = none)
	int reject_accept;  //!< after this many rejected updates in a row, accept the next one
	double hold_after_s; //!< no SLAM update for this long: hold the position (no IMU dead reckoning)
	//! IMU -> SLAM head point in head axes, m. The filter tracks the IMU (which
	//! is what the accelerometer measures); positions in and out are the head point.
	double lever[3];
	//! Each SLAM correction is shown spread over this time constant (s), not as
	//! a step at the 25 Hz update rate (0 = off).
	double smooth_tau;
};

struct p4_head
{
	struct p4_head_params prm;
	struct p4_eskf f;
	bool have_pos, have_map;
	uint32_t map_id;
	double anchor[3];  //!< filter position = aligned SLAM position - anchor
	int64_t last_upd_t; //!< IMU-clock time of the last SLAM update applied
	bool holding;
	double hold_p[3];
	int rejects;
	uint32_t heading_k;
	//! Correction not shown yet (see smooth_tau), as of corr_t.
	double corr[3];
	int64_t corr_t;

	// Statistics.
	uint32_t applied, rejected, forced, too_old, anchors, holds;
	double nis_sum;
};

void
p4_head_default_params(struct p4_head_params *prm);

void
p4_head_init(struct p4_head *h, const struct p4_head_params *prm);

/*!
 * One head IMU sample (gyro rad/s, accel m/s^2, head axes, sample time on
 * CLOCK_MONOTONIC) and the m_imu_3dof orientation after it (IMU world, xyzw).
 */
void
p4_head_imu(struct p4_head *h, int64_t t, const double w[3], const double a[3], const double q_fusion[4]);

/*!
 * A tracked SLAM head position at IMU-clock time @p t, already rotated into the
 * IMU world (before any anchor), with the head orientation at that time (IMU
 * world, xyzw). A new @p map_id re-anchors at the current position.
 */
void
p4_head_slam(struct p4_head *h, int64_t t, uint32_t map_id, const double p_world[3], const double q_head[4]);

//! SLAM reported lost/not tracking.
void
p4_head_slam_lost(struct p4_head *h, int64_t arrival);

/*!
 * Head point position (and velocity) predicted to time @p t (CLOCK_MONOTONIC),
 * for the head orientation @p q_head predicted to that time (IMU world). False
 * before the first SLAM pose.
 */
bool
p4_head_position(const struct p4_head *h, int64_t t, const double q_head[4], double p[3], double v[3]);

void
p4_head_print_stats(const struct p4_head *h, FILE *out);

#ifdef __cplusplus
}
#endif
