// Copyright 2026, picoreverse contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Error-state Kalman filter for one PICO 4 controller: IMU propagation,
 *         delayed optical pose updates (rewind and replay), stillness updates.
 *
 * Plain C with its own small double-precision math, so the offline replay
 * harness (slam/ctrl/replay) builds the same file without Monado.
 *
 * Frames: W = gravity-aligned world (+y up), I = controller IMU (accelerometer
 * axes; the gyro uses the same axes). State: q (I -> W), p, v (IMU origin,
 * world), gyro bias, accel bias. Error state [dtheta dp dv dbg dba] with a
 * right (body) perturbation q = q_hat * Exp(dtheta). After Sola, "Quaternion
 * kinematics for the error-state Kalman filter" (arXiv:1711.02508).
 *
 * @ingroup drv_pico4
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define P4_ESKF_N 15
//! IMU samples kept for replay (~1 s at 500 Hz).
#define P4_ESKF_RING 512
//! A snapshot every this many samples...
#define P4_ESKF_SNAP_EVERY 5
//! ...this many kept (~0.64 s at 500 Hz: the oldest measurement that can still be applied).
#define P4_ESKF_SNAPS 64

struct p4_eskf_params
{
	double sigma_g;  //!< gyro white noise, rad/s/sqrt(Hz)
	double sigma_a;  //!< accel white noise, m/s^2/sqrt(Hz)
	double sigma_bg; //!< gyro bias random walk, rad/s^2/sqrt(Hz)
	double sigma_ba; //!< accel bias random walk, m/s^3/sqrt(Hz)
	double coast_s;  //!< no velocity damping for this long after an optical update
	double damp_tau; //!< velocity damping time constant after that, s
	double gravity_after_s; //!< gravity (tilt) updates once this long without optical updates
};

//! Everything that changes per IMU sample; copied whole into snapshots.
struct p4_eskf_core
{
	int64_t t; //!< ns, time of the last processed sample
	double q[4]; //!< x y z w, I -> W
	double p[3], v[3], bg[3], ba[3];
	double P[P4_ESKF_N][P4_ESKF_N];

	int64_t last_opt_t; //!< time of the last optical update applied, 0 = never
	double w[3];        //!< last angular velocity, bias removed, I frame
	double a_ema[3];    //!< low-passed accel, for the stillness test
	double still_s;     //!< how long the stillness test has held
	double gsum[3];     //!< gyro sum while still, for the bias update
	int gn;
	uint32_t k; //!< sample counter for the 50 Hz pseudo-measurements
};

struct p4_eskf_sample
{
	int64_t t;
	float w[3], a[3];
};

struct p4_eskf_snap
{
	uint64_t idx; //!< index of the sample this state is after
	struct p4_eskf_core core;
};

struct p4_eskf
{
	struct p4_eskf_params prm;
	struct p4_eskf_core cur;
	uint64_t n; //!< samples pushed
	struct p4_eskf_sample ring[P4_ESKF_RING];
	struct p4_eskf_snap snaps[P4_ESKF_SNAPS];
	double bg0[3];

	// Statistics.
	uint32_t replays, too_old;
};

enum p4_eskf_result
{
	P4_ESKF_APPLIED = 0,
	P4_ESKF_REJECTED = 1,
	P4_ESKF_TOO_OLD = 2,
	P4_ESKF_NO_IMU = 3,
};

enum p4_eskf_pose_mode
{
	P4_ESKF_UPDATE = 0,      //!< normal update, gated
	P4_ESKF_RESET = 1,       //!< take the pose as it is (first lock, relock)
	P4_ESKF_RESET_POS = 2,   //!< take the position as it is, update the orientation (gated)
};

void
p4_eskf_default_params(struct p4_eskf_params *prm);

void
p4_eskf_init(struct p4_eskf *f, const struct p4_eskf_params *prm, const double bg0[3]);

//! One IMU sample: gyro (rad/s, raw incl. bias) and accel (m/s^2), I frame. Times must increase.
void
p4_eskf_imu(struct p4_eskf *f, int64_t t, const double w[3], const double a[3]);

/*!
 * Optical pose of the IMU frame at time @p t (same clock as the IMU samples):
 * orientation q (I -> W, xyzw), position p. Rp = position covariance (world),
 * sig_th = orientation sigma (rad, isotropic). gate = NIS limit (0 = none).
 * The pose the filter predicted at @p t is returned in q_pred/p_pred (may be NULL).
 */
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
             double p_pred[3]);

//! Turn the heading (about +y) so that it equals the heading of @p q_target (I -> W), keep the tilt.
void
p4_eskf_set_heading(struct p4_eskf *f, const double q_target[4]);

//! Overwrite position/velocity (out-of-view hold) and widen their uncertainty.
void
p4_eskf_set_position(struct p4_eskf *f, const double p[3], double sigma_p, double sigma_v);

bool
p4_eskf_still(const struct p4_eskf *f);

//! RMS of the orientation (rad) and position (m) sigmas.
void
p4_eskf_sigmas(const struct p4_eskf *f, double *sig_rot, double *sig_pos);

/*!
 * Sample times of one controller's IMU on CLOCK_MONOTONIC. The SSC stamp is
 * the QTimer time the SLPI read the packet (a fixed offset from
 * CLOCK_MONOTONIC: the smallest arrival - stamp over a sliding window); the
 * controller's own microsecond counter gives the exact spacing (the SLPI reads
 * in bursts, +-1.5 ms) and is tied to the SSC time with a slow average.
 */
struct p4_clock
{
	int64_t win_start, win_min, prev_min; //!< arrival - ssc minimum, current and previous window
	bool have_ctrl;
	uint32_t last_us;
	int64_t ctrl_ns;     //!< unwrapped controller counter
	double ctrl_off;     //!< mono - controller, ns, averaged
};

void
p4_clock_init(struct p4_clock *c);

//! Map an SSC stamp to CLOCK_MONOTONIC given its arrival time.
int64_t
p4_clock_ssc(struct p4_clock *c, int64_t ssc_ns, int64_t arrival_ns);

//! Sample time from the controller counter (falls back to the SSC time when ctrl_us is 0).
int64_t
p4_clock_ctrl(struct p4_clock *c, uint32_t ctrl_us, int64_t ssc_mono_ns);

/*!
 * Optical fix policy on top of the filter: first lock, gating, relock when the
 * filter is the one that is wrong, re-acquisition after a long gap.
 */
struct p4_ctrl_track
{
	struct p4_eskf f;
	bool locked;
	int64_t last_fix_t; //!< IMU-clock time of the last optical fix used
	// Rejected good fixes that agree with each other (relock rule).
	int disagree;
	int64_t rej_t;
	double rej_q[4], rej_p[3], rej_qp[4], rej_pp[3];
	// Statistics.
	uint32_t applied, rejected, ignored, relocks, held_reacq;
	double nis_sum;
	uint32_t nis_n;
};

enum p4_track_fix
{
	P4_FIX_IGNORED = 0,  //!< not good enough to lock on, or too old
	P4_FIX_LOCKED = 1,   //!< first lock (filter reset to the fix)
	P4_FIX_APPLIED = 2,  //!< normal update
	P4_FIX_REACQUIRED = 3, //!< after a long gap: position taken, orientation updated
	P4_FIX_REJECTED = 4, //!< failed the gate
	P4_FIX_RELOCKED = 5, //!< filter reset: good fixes kept disagreeing with it
};

//! After this long without a fix the next one re-acquires (position reset).
#define P4_TRACK_REACQ_NS 400000000LL

void
p4_track_init(struct p4_ctrl_track *tr, const struct p4_eskf_params *prm, const double bg0[3]);

/*!
 * Optical pose of the controller IMU frame in the world at IMU-clock time t.
 * ray = unit direction from the cameras to the controller (world), for the
 * depth-elongated position noise. out_qp/out_pp (may be NULL): the filter's
 * prediction at t, before the fix (valid when locked).
 */
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
             double out_pp[3]);

// Small helpers shared with the driver.
void
p4_q_mul(const double a[4], const double b[4], double out[4]);
void
p4_q_rot(const double q[4], const double v[3], double out[3]);
void
p4_q_conj(const double q[4], double out[4]);
double
p4_q_angle(const double a[4], const double b[4]);

#ifdef __cplusplus
}
#endif
