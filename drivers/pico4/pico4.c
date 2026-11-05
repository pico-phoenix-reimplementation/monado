// Copyright 2026, picoreverse contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 HMD (SLPI IMU orientation + SLAM position) and controllers (buttons/axes).
 *
 * Both panels are one 4320x2160 DSI connector, left eye on the left half.
 * Head orientation comes from @ref m_imu_3dof fed with the SSC gyro/accel
 * samples (low latency). Head position comes from the visual SLAM tracker
 * (pico4-slam, ORB-SLAM3 on the tracking cameras) when it runs: its poses are
 * rotated into the IMU's frame with an alignment estimated by comparing both
 * orientations, extrapolated with a filtered velocity, and held when tracking
 * is lost (1.6 m standing height until the first pose). Controllers have no
 * tracking yet and sit at fixed positions. Pressing the system (PICO) button
 * recenters yaw and the horizontal position.
 *
 * @ingroup drv_pico4
 */

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_system.h"

#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_imu_3dof.h"
#include "math/m_mathinclude.h" // IWYU pragma: keep
#include "math/m_predict.h"
#include "math/m_vec3.h"

#include "util/u_builders.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_file.h"
#include "util/u_json.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"
#include "util/u_visibility_mask.h"

#include "pico4_interface.h"
#include "pico4_ctrl_eskf.h"
#include "pico4_head.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>


DEBUG_GET_ONCE_LOG_OPTION(pico4_log, "PICO4_LOG", U_LOGGING_INFO)
DEBUG_GET_ONCE_OPTION(pico4_socket, "PICO4_SOCKET", PICO4_SOCKET_DEFAULT)
DEBUG_GET_ONCE_NUM_OPTION(pico4_refresh, "PICO4_REFRESH", 72)
DEBUG_GET_ONCE_FLOAT_OPTION(pico4_fov, "PICO4_FOV", 0.0f)
DEBUG_GET_ONCE_OPTION(pico4_optical_calib, "PICO4_OPTICAL_CALIB", "/var/lib/pico4/OpticalCalibrationData.json")
DEBUG_GET_ONCE_BOOL_OPTION(pico4_no_distortion, "PICO4_NO_DISTORTION", false)
DEBUG_GET_ONCE_BOOL_OPTION(pico4_no_slam, "PICO4_NO_SLAM", false)
DEBUG_GET_ONCE_FLOAT_OPTION(pico4_slam_align_rate, "PICO4_SLAM_ALIGN_RATE", 0.02f)
// One Euro filter on the SLAM position (Casiez et al. 2012): Hz, Hz per m/s, Hz.
DEBUG_GET_ONCE_FLOAT_OPTION(pico4_slam_mincutoff, "PICO4_SLAM_MINCUTOFF", 1.0f)
// 2026-10-26 VRChat replays (slam/ctrl/replay/head_replay): beta 10 -> 20 and
// dcutoff 2 -> 0.5 Hz cut the 72 Hz frame-to-frame jitter by a third (standing,
// small head motion) and predict the next SLAM sample as well or better.
DEBUG_GET_ONCE_FLOAT_OPTION(pico4_slam_beta, "PICO4_SLAM_BETA", 20.0f)
//! Position cutoff (Hz) while the head is still: SLAM's few-mm frame-to-frame
//! noise read as motion by the One Euro speed term made the world wobble.
DEBUG_GET_ONCE_FLOAT_OPTION(pico4_slam_still_cutoff, "PICO4_SLAM_STILL_CUTOFF", 0.05f)
//! Head still: gyro below this (rad/s, ~1.7 deg/s)...
#define P4_HEAD_STILL_GYRO 0.03f
//! ...accel magnitude within this of the learnt 1 g (m/s^2)...
#define P4_HEAD_STILL_ACCEL 0.3f
//! ...for this long (s).
#define P4_HEAD_STILL_S 0.15f
//! Velocity (derivative) cutoff, Hz. At 2 Hz the SLAM noise (5-14 mm per pose)
//! made a velocity noise of ~0.1 m/s that the extrapolation over latency +
//! prediction (~70-100 ms) showed as cm-level shaking.
DEBUG_GET_ONCE_FLOAT_OPTION(pico4_slam_dcutoff, "PICO4_SLAM_DCUTOFF", 0.5f)
//! Each 25 Hz SLAM correction of the shown position is spread over this time
//! constant (s) instead of appearing as a step (0 = steps).
DEBUG_GET_ONCE_FLOAT_OPTION(pico4_slam_spread, "PICO4_SLAM_SPREAD", 0.02f)
//! Head position from the IMU-driven error-state filter (pico4_head.c) instead
//! of the One Euro filter. Off by default: smoother, but in the replays it
//! predicted fast head motion worse (the head accelerometer adds little).
DEBUG_GET_ONCE_BOOL_OPTION(pico4_head_eskf, "PICO4_HEAD_ESKF", false)
/*!
 * Keep the head heading on SLAM: m_imu_3dof has no yaw reference and the head
 * gyro reads 2-4 % off the SLAM rotation, so within one SLAM map the IMU
 * heading drifted 10-40 deg/min while turning (2026-10-26 VRChat: 20-157 deg
 * per map). The SLAM -> IMU alignment chased it, rotating the position about
 * the map origin. With this on, the alignment is frozen 2 s into a map and the
 * IMU heading is pulled towards SLAM instead (mostly while the head turns).
 */
DEBUG_GET_ONCE_BOOL_OPTION(pico4_head_yaw_fix, "PICO4_HEAD_YAW_FIX", true)
//! Heading correction gain per SLAM pose at >= 1 rad/s head rotation (x0.1 at rest).
DEBUG_GET_ONCE_FLOAT_OPTION(pico4_head_yaw_gain, "PICO4_HEAD_YAW_GAIN", 0.05f)
// Extra rotation of the controller IMU inside the controller, "rx,ry,rz" in
// degrees, applied after the default mapping (see ctrl_mount()).
DEBUG_GET_ONCE_OPTION(pico4_ctrl_mount, "PICO4_CTRL_MOUNT", NULL)
DEBUG_GET_ONCE_OPTION(pico4_ctrl_mount_left, "PICO4_CTRL_MOUNT_LEFT", NULL)
//! Camera capture time -> IMU time, ms: the optical rotation matches the gyro
//! best 6 ms earlier (2026-10-26 recordings, both hands).
DEBUG_GET_ONCE_FLOAT_OPTION(pico4_cam_imu_ms, "PICO4_CAM_IMU_MS", -6.0f)
//! Pitch of the reported aim pose about its x axis, degrees (negative = forward axis down).
DEBUG_GET_ONCE_FLOAT_OPTION(pico4_ctrl_tilt, "PICO4_CTRL_TILT_DEG", -45.0f)
//! Record all tracking inputs to this file (slam/ctrl/replay).
DEBUG_GET_ONCE_OPTION(pico4_ctrl_record, "PICO4_CTRL_RECORD", NULL)

#define P4_INFO(sys, ...) U_LOG_IFL_I(sys->log_level, __VA_ARGS__)
#define P4_WARN(sys, ...) U_LOG_IFL_W(sys->log_level, __VA_ARGS__)
#define P4_ERROR(sys, ...) U_LOG_IFL_E(sys->log_level, __VA_ARGS__)

#define PANEL_W 2160
#define PANEL_H 2160

/*
 * Stock lens model (XRRuntime libpxrcompositor, distortionEqnType 2): for a
 * panel point at r metres from the lens centre, theta = r / F and the eye
 * buffer is sampled at tan angle theta * scale(theta^2), with
 * scale(rsq) = K0 + K1 rsq + ... + K10 rsq^10. Per-unit F and K come from
 * persist pvr/OpticalCalibrationData.json ("distortion_parameter", as
 * configserverservice maps them to metersPerTanAngleAtCenter and
 * distortionCoefficients0-10); the panel width is 0.046 m per eye and each
 * lens sits on its panel centre (FixedLensSeparation 0.046).
 */
#define PANEL_W_METERS 0.046f
#define LENS_K_COUNT 11

//! SDK_Lens_ConfigFile defaults, used when the calibration file is missing.
static const float lens_default_f = 0.02735f;
static const float lens_default_k[LENS_K_COUNT] = {
    1.0f,         0.5068f,       -1.989459f,   24.37384f,     -101.909302f, 81.744944f,
    770.101657f, -2911.542581f, 4519.58911f, -3355.23612f, 977.224696f,
};
//! sdk_lens_chromaticAberration0-3: red 1 + c0 + c1 rsq, blue 1 + c2 + c3 rsq.
static const float lens_ca[4] = {-0.002894f, -0.007057f, 0.006197f, 0.01459f};
//! Stock render frustum: +-0.065 at near 0.0508 on all sides (~104 degrees).
static const float lens_tan_half_fov = 0.065f / 0.0508f;
//! Past this theta^2 the polynomial diverges (panel corners), sample black.
static const float lens_max_rsq = 1.0f;

struct pico4_lens
{
	float f;
	float k[LENS_K_COUNT];
};

enum pico4_touch_input
{
	P4_IN_AX_CLICK,
	P4_IN_AX_TOUCH,
	P4_IN_BY_CLICK,
	P4_IN_BY_TOUCH,
	P4_IN_MENU_SYSTEM_CLICK,
	P4_IN_SQUEEZE_VALUE,
	P4_IN_TRIGGER_TOUCH,
	P4_IN_TRIGGER_VALUE,
	P4_IN_THUMBSTICK_CLICK,
	P4_IN_THUMBSTICK_TOUCH,
	P4_IN_THUMBSTICK,
	P4_IN_THUMBREST_TOUCH,
	P4_IN_GRIP_POSE,
	P4_IN_AIM_POSE,
	P4_IN_COUNT,
};

struct pico4_system;

struct pico4_hmd
{
	struct xrt_device base;
	struct pico4_system *sys;
	struct pico4_lens lens[2];
	float tan_half_fov;
};

struct pico4_controller
{
	struct xrt_device base;
	struct pico4_system *sys;
	int hand;
	struct xrt_pose pose;

	//! Latest state from the bridge, protected by sys->mutex.
	struct pico4_msg_controller state;
	int64_t state_ns;
	bool system_was_down;
	int64_t system_down_ns; //!< When the home button went down.
	bool recentered;        //!< Recentered during this press.

	//! 6DoF filter (controller IMU + optical fixes), its clock; protected by sys->mutex.
	struct p4_ctrl_track tr;
	struct p4_clock clk;
	struct xrt_quat mount; //!< Controller IMU accel axes -> OpenXR grip axes.
	struct xrt_quat tilt;  //!< Extra pitch of the reported aim pose (PICO4_CTRL_TILT_DEG).
	int64_t imu_t;         //!< IMU-clock time of the last sample (= filter time).
	uint64_t imu_count;
	bool heading_set; //!< Heading set from the head (before the first optical lock).

	//! Out of the cameras' view the hand hangs on the elbow it had when last
	//! seen, kept in the head's yaw frame (it follows the body) and turned by
	//! the controller IMU; blended in from the filter's coasting estimate.
	int64_t hold_fix_t; //!< last_fix_t the hold below was taken for
	struct xrt_vec3 hold_elbow_hy;
	int64_t hold_set_ns;

	//! When the cameras pick the controller up again, the position jump is
	//! kept as a correction that decays to zero.
	bool out_optical, have_out;
	struct xrt_vec3 out_pos; //!< Last position handed out (before prediction).
	struct xrt_vec3 corr_pos;
	int64_t corr_ns;

	int64_t shm_t; //!< Last shared-memory entry written (IMU clock).
	int64_t stat_ns;
	uint32_t stat_fix[6]; //!< enum p4_track_fix counts since the last status line
	double stat_nis;
};


//! Filter output counts as optically tracked this long after a fix...
#define P4_CTRL_TRACKED_NS (300 * U_TIME_1MS_IN_NS)
//! ...then blends into the elbow hold over this long.
#define P4_CTRL_BLEND_NS (250 * U_TIME_1MS_IN_NS)
//! Time constant of the position easing when the cameras pick a controller up again.
#define P4_CTRL_SWITCH_TAU_S 0.04
//! Controller IMU samples before its heading is set from the head (~0.1 s).
#define P4_CTRL_ALIGN_AFTER 50
//! Forearm, elbow -> grip, along the grip's -z (m).
#define P4_CTRL_FOREARM 0.30f
//! Hold the home button this long to recenter.
#define P4_RECENTER_HOLD_NS (800 * U_TIME_1MS_IN_NS)
//! The LED tracker gets the heading as a prior only this long after a fix.
#define P4_CTRL_PRIOR_LOCK_NS (8000ULL * U_TIME_1MS_IN_NS)
//! Shared-memory state entries every this long.
#define P4_CTRL_SHM_NS (10 * U_TIME_1MS_IN_NS)
/*!
 * Controller IMU frame -> LED model frame (q_h_imu = q_h_led * this): accel =
 * P * led with P = [[0,-1,0],[0,0,1],[-1,0,0]]; it rotates IMU-axis vectors
 * into LED-model axes.
 */
static const double p4_led_to_imu[4] = {0.5, -0.5, -0.5, 0.5};

//! IMU orientations kept to match SLAM poses by time (~0.5 s at 500 Hz).
#define P4_IMU_HISTORY 256
//! Standing head height used until the first SLAM pose.
#define P4_DEFAULT_HEIGHT 1.6f
//! Alignment learnt over this many poses of a new map, then frozen (PICO4_HEAD_YAW_FIX).
#define P4_YAW_ALIGN_POSES 50
//! Heading errors above this (rad) are not corrected...
#define P4_YAW_MAX (10.0 * M_PI / 180.0)
//! ...and this many in a row re-align (keeping the shown position).
#define P4_YAW_REALIGN 25
//! SLAM poses older than this (no message) count as lost.
#define P4_SLAM_TIMEOUT_NS (500 * U_TIME_1MS_IN_NS)

struct pico4_imu_sample
{
	int64_t ns; //!< Sample time on CLOCK_MONOTONIC (mapped SSC time).
	struct xrt_quat rot;
};

//! SLAM positions kept to place optical controller fixes in the world.
#define P4_SLAM_HISTORY 32

struct pico4_pos_sample
{
	int64_t ns; //!< Capture time.
	struct xrt_vec3 pos, vel;
};

//! Position from pico4-slam, protected by pico4_system::mutex.
struct pico4_slam
{
	bool enabled;
	int32_t state;  //!< enum pico4_slam_state, as last reported
	uint32_t map_id;
	bool have_map;
	int64_t msg_ns;    //!< Monotonic arrival time of the last message.
	int64_t sample_ns; //!< Capture time of the last tracked pose.
	uint64_t poses;

	//! SLAM world -> IMU world, low-pass filtered.
	struct xrt_quat align;
	float align_rate;

	//! Position = align * slam position - anchor (+ the standing height at the start).
	struct xrt_vec3 anchor;
	bool have_pos;
	struct xrt_vec3 pos; //!< At sample_ns, in the IMU world, before recentering.
	struct xrt_vec3 vel;   //!< Filtered velocity (One Euro derivative).
	struct xrt_vec3 blend; //!< Correction not shown yet, fades out with time constant spread.
	int64_t blend_ns;      //!< Monotonic time blend was set.
	float spread;          //!< s, see PICO4_SLAM_SPREAD

	//! PICO4_HEAD_YAW_FIX state.
	bool yaw_fix;
	float yaw_gain;
	uint32_t map_poses; //!< Tracked poses in the current map.
	int yaw_rejects;    //!< Poses in a row with a heading error > P4_YAW_MAX.
	uint32_t realigns;

	//! One Euro filter: SLAM positions jitter by a few mm per frame, which the
	//! velocity extrapolation turned into visible shaking.
	float mincutoff, beta, dcutoff;
	float still_cutoff; //!< Position cutoff while the head is still.
};

struct pico4_system
{
	struct os_mutex mutex;
	struct os_thread_helper oth;
	int fd;
	char path[108];
	enum u_logging_level log_level;

	struct m_imu_3dof fusion;
	//! PICO4_HEAD_YAW_FIX: heading correction (about +y, world) applied on top of the fusion.
	struct xrt_quat yaw_corr;
	//! Head orientation (IMU world) = yaw_corr * fusion.rot; use this, not fusion.rot.
	struct xrt_quat head_rot;
	struct xrt_vec3 last_gyro;
	int64_t last_imu_ns; //!< Monotonic time the last IMU sample arrived.
	//! Head stillness from the IMU (see P4_HEAD_STILL_*): while still, the SLAM
	//! alignment is frozen and the position filtered much harder.
	float head_still_s, head_g_ref;
	int64_t head_imu_ts;
	uint64_t imu_count;
	//! Applied in world space on top of the fusion, set by recentering.
	struct xrt_quat recenter;
	//! Position recentering: shown = recenter * (pos - from) + to.
	struct xrt_vec3 recenter_from, recenter_to;

	struct pico4_imu_sample imu_history[P4_IMU_HISTORY];
	uint32_t imu_history_next;
	//! SSC (QTimer) stamps -> CLOCK_MONOTONIC, shared by the head and controller IMUs.
	struct p4_clock ssc_clk;
	//! Camera capture time -> IMU sample time (from the gyro vs optical rotation fit).
	int64_t cam_to_imu_ns;

	struct pico4_pos_sample slam_history[P4_SLAM_HISTORY];
	uint32_t slam_history_next;

	struct pico4_ctrl_shm *shm; //!< Controller state for the LED tracker, NULL if unavailable.
	FILE *rec;                  //!< PICO4_CTRL_RECORD: all tracking inputs, for offline replay.

	struct pico4_slam slam;

	//! PICO4_HEAD_ESKF: head position from the IMU + SLAM error-state filter.
	bool head_eskf;
	struct p4_head head;
	int64_t head_imu_t; //!< Sample time (CLOCK_MONOTONIC) of the last head IMU sample.

	//! Last head pose handed out, for the controllers' arm model.
	struct xrt_pose head_pose;
	bool have_head_pose;

	struct pico4_hmd *hmd;
	struct pico4_controller *ctrl[2];
};


/*
 *
 * Bridge socket.
 *
 */

static void
quat_to_d(const struct xrt_quat *q, double out[4])
{
	out[0] = q->x;
	out[1] = q->y;
	out[2] = q->z;
	out[3] = q->w;
}

static struct xrt_quat
quat_from_d(const double q[4])
{
	struct xrt_quat r = {(float)q[0], (float)q[1], (float)q[2], (float)q[3]};
	math_quat_normalize(&r);
	return r;
}

//! Before the cameras lock it: point the controller (grip) along the head's heading.
static void
ctrl_set_heading_locked(struct pico4_system *sys, struct pico4_controller *c)
{
	const struct xrt_vec3 up = {0.0f, 1.0f, 0.0f};
	struct xrt_quat q_wi = quat_from_d(c->tr.f.cur.q), mount_inv, q_wg, swing, head_twist, grip_twist, inv, d,
	                target;
	math_quat_invert(&c->mount, &mount_inv);
	math_quat_rotate(&q_wi, &mount_inv, &q_wg);
	math_quat_decompose_swing_twist(&sys->head_rot, &up, &swing, &head_twist);
	math_quat_decompose_swing_twist(&q_wg, &up, &swing, &grip_twist);
	math_quat_invert(&grip_twist, &inv);
	math_quat_rotate(&head_twist, &inv, &d);
	math_quat_rotate(&d, &q_wi, &target);
	double t[4];
	quat_to_d(&target, t);
	p4_eskf_set_heading(&c->tr.f, t);
	c->heading_set = true;
}

static void
recenter_locked(struct pico4_system *sys)
{
	const struct xrt_vec3 up = {0.0f, 1.0f, 0.0f};
	struct xrt_quat swing, twist;
	math_quat_decompose_swing_twist(&sys->head_rot, &up, &swing, &twist);
	math_quat_invert(&twist, &sys->recenter);

	// Keep the height, move the current spot under the origin.
	if (sys->slam.have_pos) {
		sys->recenter_from = sys->slam.pos;
		sys->recenter_to = (struct xrt_vec3){0.0f, sys->slam.pos.y, 0.0f};
	}
	// Controllers the cameras have locked keep their heading (it is the cameras').
	for (int i = 0; i < 2; i++) {
		struct pico4_controller *c = sys->ctrl[i];
		if (c != NULL && c->imu_count > 0 && !c->tr.locked) {
			ctrl_set_heading_locked(sys, c);
		}
	}
	P4_INFO(sys, "Recentered yaw%s", sys->slam.have_pos ? " and position" : "");
}

//! Head orientation (IMU world, before recentering) at IMU-clock time @p ns, interpolated.
static void
imu_history_lookup_locked(struct pico4_system *sys, int64_t ns, struct xrt_quat *out_rot)
{
	const struct pico4_imu_sample *before = NULL, *after = NULL;
	for (uint32_t i = 0; i < P4_IMU_HISTORY; i++) {
		const struct pico4_imu_sample *s = &sys->imu_history[i];
		if (s->ns == 0) {
			continue;
		}
		if (s->ns <= ns && (before == NULL || s->ns > before->ns)) {
			before = s;
		}
		if (s->ns > ns && (after == NULL || s->ns < after->ns)) {
			after = s;
		}
	}
	if (before != NULL && after != NULL) {
		float k = (float)((double)(ns - before->ns) / (double)(after->ns - before->ns));
		math_quat_slerp(&before->rot, &after->rot, k, out_rot);
		math_quat_normalize(out_rot);
	} else if (before != NULL) {
		*out_rot = before->rot; // newer than the history: the newest sample
	} else if (after != NULL) {
		*out_rot = after->rot;
	} else {
		*out_rot = sys->head_rot;
	}
}

//! Head position (IMU world, before recentering) at capture time @p ns.
static struct xrt_vec3
slam_history_lookup_locked(struct pico4_system *sys, int64_t ns)
{
	const struct pico4_pos_sample *before = NULL, *after = NULL;
	for (uint32_t i = 0; i < P4_SLAM_HISTORY; i++) {
		const struct pico4_pos_sample *s = &sys->slam_history[i];
		if (s->ns == 0) {
			continue;
		}
		if (s->ns <= ns && (before == NULL || s->ns > before->ns)) {
			before = s;
		}
		if (s->ns > ns && (after == NULL || s->ns < after->ns)) {
			after = s;
		}
	}
	if (before != NULL && after != NULL) {
		float k = (float)((double)(ns - before->ns) / (double)(after->ns - before->ns));
		return m_vec3_lerp(before->pos, after->pos, k);
	}
	if (before != NULL) {
		double dt = time_ns_to_s(ns - before->ns);
		dt = dt > 0.1 ? 0.1 : dt;
		return m_vec3_add(before->pos, m_vec3_mul_scalar(before->vel, (float)dt));
	}
	if (after != NULL) {
		return after->pos;
	}
	return sys->slam.have_pos ? sys->slam.pos : (struct xrt_vec3){0.0f, P4_DEFAULT_HEIGHT, 0.0f};
}

//! Smoothing factor of a first-order low-pass with cutoff @p hz for a step of @p dt seconds.
static float
one_euro_alpha(double dt, float hz)
{
	double tau = 1.0 / (2.0 * M_PI * hz);
	return (float)(1.0 / (1.0 + tau / dt));
}

static void
handle_head_pose_locked(struct pico4_system *sys, const struct pico4_msg_head_pose *m, int64_t now)
{
	struct pico4_slam *sl = &sys->slam;

	if (sys->rec != NULL) {
		fprintf(sys->rec, "P %" PRId64 " %" PRId64 " %d %u %.5f %.5f %.5f %.6f %.6f %.6f %.6f\n", m->timestamp_ns, now,
		        m->state, m->map_id, m->position[0], m->position[1], m->position[2], m->orientation[0],
		        m->orientation[1], m->orientation[2], m->orientation[3]);
	}
	sl->msg_ns = now;
	if (sl->state != m->state) {
		P4_INFO(sys, "SLAM %s (map %u)",
		        m->state == PICO4_SLAM_TRACKING       ? "tracking"
		        : m->state == PICO4_SLAM_RECENTLY_LOST ? "recently lost"
		                                               : "not tracking",
		        m->map_id);
	}
	sl->state = m->state;
	if (m->state != PICO4_SLAM_TRACKING) {
		// Hold the last position.
		sl->vel = (struct xrt_vec3){0};
		return;
	}

	struct xrt_quat q_slam = {m->orientation[0], m->orientation[1], m->orientation[2], m->orientation[3]};
	struct xrt_vec3 p_slam = {m->position[0], m->position[1], m->position[2]};
	if (!math_quat_validate_within_1_percent(&q_slam) || !math_vec3_validate(&p_slam)) {
		return;
	}
	math_quat_normalize(&q_slam);

	// Both orientations describe the head: align = q_imu * q_slam^-1 maps SLAM world to IMU world.
	struct xrt_quat q_imu, q_slam_inv, meas;
	imu_history_lookup_locked(sys, m->timestamp_ns + sys->cam_to_imu_ns, &q_imu);
	math_quat_invert(&q_slam, &q_slam_inv);
	math_quat_rotate(&q_imu, &q_slam_inv, &meas);
	math_quat_normalize(&meas);

	bool new_map = !sl->have_map || sl->map_id != m->map_id;
	struct xrt_quat align_before = sl->align;
	bool realigned = false;
	if (new_map) {
		sl->align = meas;
		sl->map_poses = 0;
		sl->yaw_rejects = 0;
	} else if (sl->yaw_fix && sl->map_poses >= P4_YAW_ALIGN_POSES) {
		// Alignment frozen: turn the IMU heading towards SLAM (align * q_slam).
		struct xrt_quat q_sw, q_imu_inv, d;
		math_quat_rotate(&sl->align, &q_slam, &q_sw);
		math_quat_invert(&q_imu, &q_imu_inv);
		math_quat_rotate(&q_sw, &q_imu_inv, &d);
		double yaw = remainder(2.0 * atan2(d.y, d.w), 2.0 * M_PI);
		if (fabs(yaw) < P4_YAW_MAX) {
			// Mostly while the head turns, where a correction is not noticed.
			float wn = m_vec3_len(sys->last_gyro);
			float k = sl->yaw_gain * fminf(1.0f, fmaxf(0.1f, wn));
			struct xrt_vec3 up = {0.0f, 1.0f, 0.0f};
			struct xrt_quat dq, yc;
			math_quat_from_angle_vector((float)(k * yaw), &up, &dq);
			math_quat_rotate(&dq, &sys->yaw_corr, &yc);
			math_quat_normalize(&yc);
			sys->yaw_corr = yc;
			math_quat_rotate(&sys->yaw_corr, &sys->fusion.rot, &sys->head_rot);
			math_quat_normalize(&sys->head_rot);
			sl->yaw_rejects = 0;
			if (sys->rec != NULL) {
				fprintf(sys->rec, "Y %" PRId64 " %.6f %.6f %.6f %.6f\n", m->timestamp_ns, yc.x, yc.y, yc.z, yc.w);
			}
		} else if (++sl->yaw_rejects > P4_YAW_REALIGN) {
			// SLAM and IMU keep disagreeing (a bad map or a big IMU error): re-align.
			sl->align = meas;
			sl->yaw_rejects = 0;
			sl->realigns++;
			realigned = true;
			P4_INFO(sys, "SLAM map %u re-aligned (heading off by %.0f deg)", m->map_id, yaw * 180.0 / M_PI);
		}
	} else if (sl->yaw_fix) {
		// Learn the alignment quickly at the start of a map.
		struct xrt_quat a;
		math_quat_slerp(&sl->align, &meas, 0.1f, &a);
		math_quat_normalize(&a);
		sl->align = a;
	} else if (sys->head_still_s < P4_HEAD_STILL_S) {
		// Only while moving: SLAM orientation noise in the alignment rotates the
		// whole position about the map origin (mm of jitter metres away).
		struct xrt_quat a;
		math_quat_slerp(&sl->align, &meas, sl->align_rate, &a);
		math_quat_normalize(&a);
		sl->align = a;
	}

	struct xrt_vec3 p_world;
	math_quat_rotate_vec3(&sl->align, &p_slam, &p_world);
	sl->map_poses++;
	if (realigned) {
		// Keep the shown position: move the anchor with the new alignment.
		struct xrt_vec3 p_old;
		math_quat_rotate_vec3(&align_before, &p_slam, &p_old);
		sl->anchor = m_vec3_add(sl->anchor, m_vec3_sub(p_world, p_old));
	}

	if (sys->head_eskf) {
		const double pw[3] = {p_world.x, p_world.y, p_world.z};
		double qi[4];
		quat_to_d(&q_imu, qi);
		p4_head_slam(&sys->head, m->timestamp_ns + sys->cam_to_imu_ns, m->map_id, pw, qi);
	}

	if (new_map) {
		// Continue from where the head is shown now (or the standing height at the start).
		struct xrt_vec3 here = sl->have_pos ? sl->pos : (struct xrt_vec3){0.0f, P4_DEFAULT_HEIGHT, 0.0f};
		sl->anchor = m_vec3_sub(p_world, here);
		sl->vel = (struct xrt_vec3){0};
		sl->blend = (struct xrt_vec3){0};
		sl->sample_ns = m->timestamp_ns;
		sl->pos = here;
		sl->have_pos = true;
		sl->have_map = true;
		sl->map_id = m->map_id;
		sl->poses++;
		memset(sys->slam_history, 0, sizeof(sys->slam_history));
		P4_INFO(sys, "SLAM map %u anchored", m->map_id);
		return;
	}

	struct xrt_vec3 pos = m_vec3_sub(p_world, sl->anchor);
	double dt = time_ns_to_s(m->timestamp_ns - sl->sample_ns);
	if (dt <= 0.0) {
		return; // stale or repeated frame
	}

	// What hmd_get_tracked_pose shows now, before this pose (see PICO4_SLAM_SPREAD).
	struct xrt_vec3 shown = m_vec3_add(sl->pos, m_vec3_mul_scalar(sl->vel, (float)time_ns_to_s(now - sl->sample_ns)));
	if (sl->spread > 0.0f) {
		float k = (float)exp(-time_ns_to_s(now - sl->blend_ns) / sl->spread);
		shown = m_vec3_add(shown, m_vec3_mul_scalar(sl->blend, k));
	}

	// One Euro filter: smooth hard at rest (low cutoff), follow fast motion
	// (cutoff rises with the filtered speed); its derivative is the velocity
	// used for extrapolation.
	sl->blend = (struct xrt_vec3){0};
	if (dt < 0.2) {
		struct xrt_vec3 dx = m_vec3_mul_scalar(m_vec3_sub(pos, sl->pos), (float)(1.0 / dt));
		if (m_vec3_len(dx) > 5.0f) {
			dx = (struct xrt_vec3){0}; // a jump, not motion
		}
		float ad = one_euro_alpha(dt, sl->dcutoff);
		sl->vel = m_vec3_add(sl->vel, m_vec3_mul_scalar(m_vec3_sub(dx, sl->vel), ad));
		bool still = sys->head_still_s >= P4_HEAD_STILL_S;
		if (still) {
			sl->vel = (struct xrt_vec3){0}; // no extrapolation of noise
		}
		float cutoff = still ? sl->still_cutoff : sl->mincutoff + sl->beta * m_vec3_len(sl->vel);
		float a = one_euro_alpha(dt, cutoff);
		sl->pos = m_vec3_add(sl->pos, m_vec3_mul_scalar(m_vec3_sub(pos, sl->pos), a));
		if (sl->spread > 0.0f) {
			// Show the change gradually instead of as a 25 Hz step.
			struct xrt_vec3 next =
			    m_vec3_add(sl->pos, m_vec3_mul_scalar(sl->vel, (float)time_ns_to_s(now - m->timestamp_ns)));
			sl->blend = m_vec3_sub(shown, next);
			sl->blend_ns = now;
		}
	} else {
		// Too long without a pose: take it as it is.
		sl->vel = (struct xrt_vec3){0};
		sl->pos = pos;
	}
	sl->sample_ns = m->timestamp_ns;
	sl->poses++;
	sys->slam_history[sys->slam_history_next] = (struct pico4_pos_sample){m->timestamp_ns, sl->pos, sl->vel};
	sys->slam_history_next = (sys->slam_history_next + 1) % P4_SLAM_HISTORY;
	if (sys->rec != NULL) {
		fprintf(sys->rec, "S %" PRId64 " %.5f %.5f %.5f\n", m->timestamp_ns, sl->pos.x, sl->pos.y, sl->pos.z);
	}
}

static const char *
hand_name(const struct pico4_controller *c)
{
	return c->hand == 0 ? "Left" : "Right";
}

/*!
 * Optical controller pose (LED model frame -> head frame at capture time) into
 * the world at that time (head orientation from the IMU history, head position
 * from the SLAM history), then into the controller's filter.
 */
static void
handle_controller_pose_locked(struct pico4_system *sys,
                              struct pico4_controller *c,
                              const struct pico4_msg_controller_pose *m,
                              int64_t now)
{
	struct xrt_quat q_h_led = {m->orientation[0], m->orientation[1], m->orientation[2], m->orientation[3]};
	struct xrt_vec3 p_h = {m->position[0], m->position[1], m->position[2]};
	if (!math_quat_validate_within_1_percent(&q_h_led) || !math_vec3_validate(&p_h)) {
		return;
	}
	math_quat_normalize(&q_h_led);
	if (sys->rec != NULL) {
		fprintf(sys->rec, "O %d %" PRId64 " %u %.3f %.5f %.5f %.5f %.6f %.6f %.6f %.6f\n", c->hand, m->timestamp_ns,
		        m->inliers, m->rms, p_h.x, p_h.y, p_h.z, q_h_led.x, q_h_led.y, q_h_led.z, q_h_led.w);
	}
	if (c->imu_count == 0) {
		return;
	}

	int64_t t = m->timestamp_ns + sys->cam_to_imu_ns;
	struct xrt_quat q_wh;
	imu_history_lookup_locked(sys, t, &q_wh);
	struct xrt_vec3 p_wh = slam_history_lookup_locked(sys, m->timestamp_ns);

	double qwh[4], qhl[4], qwl[4], q[4], ph[3], ray[3], p[3];
	quat_to_d(&q_wh, qwh);
	quat_to_d(&q_h_led, qhl);
	p4_q_mul(qwh, qhl, qwl);
	p4_q_mul(qwl, p4_led_to_imu, q);
	ph[0] = p_h.x;
	ph[1] = p_h.y;
	ph[2] = p_h.z;
	p4_q_rot(qwh, ph, ray);
	p[0] = p_wh.x + ray[0];
	p[1] = p_wh.y + ray[1];
	p[2] = p_wh.z + ray[2];
	double rn = sqrt(ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2]);
	for (int i = 0; i < 3; i++) {
		ray[i] /= rn > 1e-6 ? rn : 1.0;
	}

	double nis = 0;
	enum p4_track_fix r = p4_track_fix(&c->tr, t, q, p, ray, m->inliers, m->rms, &nis, NULL, NULL);
	c->stat_fix[r]++;
	if (r == P4_FIX_APPLIED) {
		c->stat_nis += nis;
	}
	switch (r) {
	case P4_FIX_LOCKED:
		P4_INFO(sys, "%s controller: locked to the cameras (%u LEDs, %.2f px)", hand_name(c), m->inliers, m->rms);
		break;
	case P4_FIX_RELOCKED:
		P4_INFO(sys, "%s controller: re-locked, good fixes kept disagreeing with the filter", hand_name(c));
		break;
	case P4_FIX_REACQUIRED:
		P4_INFO(sys, "%s controller: tracking recovered by lights (%u LEDs, %.2f px)", hand_name(c), m->inliers, m->rms);
		break;
	default: break;
	}

	if (now - c->stat_ns > (int64_t)5 * U_TIME_1S_IN_NS) {
		const struct p4_eskf_core *f = &c->tr.f.cur;
		uint32_t *n = c->stat_fix;
		P4_INFO(sys,
		        "%s controller: fixes %u applied, %u rejected (mean NIS %.1f), %u reacquired, %u ignored, "
		        "%u relocks; gyro bias %.2f %.2f %.2f deg/s",
		        hand_name(c), n[P4_FIX_APPLIED], n[P4_FIX_REJECTED],
		        n[P4_FIX_APPLIED] ? c->stat_nis / n[P4_FIX_APPLIED] : 0.0, n[P4_FIX_REACQUIRED], n[P4_FIX_IGNORED],
		        n[P4_FIX_RELOCKED], f->bg[0] * 180.0 / M_PI, f->bg[1] * 180.0 / M_PI, f->bg[2] * 180.0 / M_PI);
		c->stat_ns = now;
		memset(c->stat_fix, 0, sizeof(c->stat_fix));
		c->stat_nis = 0;
	}
}

//! Append the controller's state (as the LED tracker sees it) to the shared memory.
static void
ctrl_shm_write_locked(struct pico4_system *sys, struct pico4_controller *c)
{
	struct pico4_ctrl_shm *shm = sys->shm;
	const struct p4_eskf_core *f = &c->tr.f.cur;
	if (shm == NULL || f->t - c->shm_t < P4_CTRL_SHM_NS) {
		return;
	}
	c->shm_t = f->t;

	// Head pose now (the filter's newest sample is within a few ms of now).
	double qwh[4], qhw[4], tmp[4], q_h_led[4], q_i_l[4], led_q[4], d[3], p_h[3], up_led[3], w_led[3];
	quat_to_d(&sys->head_rot, qwh);
	p4_q_conj(qwh, qhw);
	struct xrt_vec3 p_wh = slam_history_lookup_locked(sys, f->t - sys->cam_to_imu_ns);
	p4_q_conj(p4_led_to_imu, q_i_l); // LED axes -> IMU axes
	p4_q_mul(qhw, f->q, tmp);
	p4_q_mul(tmp, q_i_l, q_h_led);
	d[0] = f->p[0] - p_wh.x;
	d[1] = f->p[1] - p_wh.y;
	d[2] = f->p[2] - p_wh.z;
	p4_q_rot(qhw, d, p_h);
	p4_q_mul(f->q, q_i_l, tmp); // LED -> world
	p4_q_conj(tmp, led_q);
	const double up[3] = {0, 1, 0};
	p4_q_rot(led_q, up, up_led);
	p4_q_rot(p4_led_to_imu, f->w, w_led);

	double sr, sp;
	p4_eskf_sigmas(&c->tr.f, &sr, &sp);
	uint32_t flags = 0;
	if (c->imu_count >= P4_CTRL_ALIGN_AFTER) {
		flags |= PICO4_CTRL_SHM_IMU;
	}
	// The heading is only a full prior shortly after a fix: out of view it
	// drifts, and a stale "locked" heading made the tracker reject every
	// correct fit (no fixes for minutes, 2026-10-26). Tilt stays good (IMU flag).
	if (c->tr.locked && f->t - c->tr.last_fix_t < P4_CTRL_PRIOR_LOCK_NS) {
		flags |= PICO4_CTRL_SHM_LOCKED;
		if (f->t - c->tr.last_fix_t < P4_CTRL_TRACKED_NS) {
			flags |= PICO4_CTRL_SHM_TRACKED;
		}
	}

	struct pico4_ctrl_shm_entry e = {
	    .timestamp_ns = f->t - sys->cam_to_imu_ns,
	    .flags = flags,
	    .sigma_rot = (float)sr,
	    .sigma_pos = (float)sp,
	};
	for (int i = 0; i < 3; i++) {
		e.q_h_led[i] = (float)q_h_led[i];
		e.p_h[i] = (float)p_h[i];
		e.up_led[i] = (float)up_led[i];
		e.omega_led[i] = (float)w_led[i];
	}
	e.q_h_led[3] = (float)q_h_led[3];

	uint32_t h = (uint32_t)c->hand;
	__atomic_add_fetch(&shm->seq, 1, __ATOMIC_ACQ_REL);
	shm->e[h][shm->next[h] % PICO4_CTRL_SHM_ENTRIES] = e;
	shm->next[h] = (shm->next[h] + 1) % PICO4_CTRL_SHM_ENTRIES;
	__atomic_add_fetch(&shm->seq, 1, __ATOMIC_ACQ_REL);
}

static void
handle_controller_imu_locked(struct pico4_system *sys,
                             struct pico4_controller *c,
                             int64_t t,
                             const float gyro[3],
                             const float accel[3])
{
	double w[3] = {gyro[0], gyro[1], gyro[2]}, a[3] = {accel[0], accel[1], accel[2]};
	if (sys->rec != NULL) {
		fprintf(sys->rec, "I %d %" PRId64 " %.5f %.5f %.5f %.4f %.4f %.4f\n", c->hand, t, w[0], w[1], w[2], a[0], a[1],
		        a[2]);
	}
	if (c->imu_count > 0 && t <= c->imu_t) {
		return;
	}
	p4_eskf_imu(&c->tr.f, t, w, a);
	c->imu_t = t;
	if (++c->imu_count == P4_CTRL_ALIGN_AFTER) {
		ctrl_set_heading_locked(sys, c);
		P4_INFO(sys, "%s controller IMU running", hand_name(c));
	}
	ctrl_shm_write_locked(sys, c);
}

static void
handle_msg(struct pico4_system *sys, const uint8_t *buf, ssize_t len)
{
	int64_t now = os_monotonic_get_ns();

	if (len == sizeof(struct pico4_msg_head_imu) && buf[0] == PICO4_MSG_HEAD_IMU) {
		struct pico4_msg_head_imu m;
		memcpy(&m, buf, sizeof(m));
		/*
		 * The IMU is mounted turned 90 degrees about the view axis:
		 * OpenXR x = -imu y, y = imu x (turning left showed up as
		 * pitch, pitching up as yaw right). At rest gravity lands on +y.
		 */
		struct xrt_vec3 gyro = {-m.gyro[1], m.gyro[0], m.gyro[2]};
		struct xrt_vec3 accel = {-m.accel[1], m.accel[0], m.accel[2]};

		os_mutex_lock(&sys->mutex);
		m_imu_3dof_update(&sys->fusion, (uint64_t)m.timestamp_ns, &accel, &gyro);
		math_quat_rotate(&sys->yaw_corr, &sys->fusion.rot, &sys->head_rot);
		math_quat_normalize(&sys->head_rot);
		sys->last_gyro = gyro;
		sys->last_imu_ns = now;
		{
			double idt = time_ns_to_s(m.timestamp_ns - sys->head_imu_ts);
			sys->head_imu_ts = m.timestamp_ns;
			if (sys->head_g_ref == 0.0f) {
				sys->head_g_ref = (float)MATH_GRAVITY_M_S2;
			}
			float an = m_vec3_len(accel);
			bool quiet = m_vec3_len(gyro) < P4_HEAD_STILL_GYRO && fabsf(an - sys->head_g_ref) < P4_HEAD_STILL_ACCEL;
			if (quiet && idt > 0.0 && idt < 0.05) {
				sys->head_still_s += (float)idt;
				sys->head_g_ref += (an - sys->head_g_ref) * 0.01f;
			} else {
				sys->head_still_s = 0.0f;
			}
		}
		int64_t t = p4_clock_ssc(&sys->ssc_clk, m.timestamp_ns, now);
		sys->imu_history[sys->imu_history_next] = (struct pico4_imu_sample){t, sys->head_rot};
		if (sys->head_eskf) {
			const double wd[3] = {gyro.x, gyro.y, gyro.z}, ad[3] = {accel.x, accel.y, accel.z};
			double qf[4];
			quat_to_d(&sys->head_rot, qf);
			p4_head_imu(&sys->head, t, wd, ad, qf);
			sys->head_imu_t = t;
		}
		if (sys->rec != NULL) {
			// Raw m_imu_3dof (before the PICO4_HEAD_YAW_FIX heading correction).
			fprintf(sys->rec, "H %" PRId64 " %.6f %.6f %.6f %.6f\n", t, sys->fusion.rot.x, sys->fusion.rot.y,
			        sys->fusion.rot.z, sys->fusion.rot.w);
			fprintf(sys->rec, "G %" PRId64 " %.5f %.5f %.5f %.4f %.4f %.4f\n", t, gyro.x, gyro.y, gyro.z, accel.x,
			        accel.y, accel.z);
		}
		sys->imu_history_next = (sys->imu_history_next + 1) % P4_IMU_HISTORY;
		if (sys->imu_count++ == 0) {
			P4_INFO(sys, "First head IMU sample");
		}
		os_mutex_unlock(&sys->mutex);
		return;
	}

	if (len == sizeof(struct pico4_msg_controller) && buf[0] == PICO4_MSG_CONTROLLER) {
		struct pico4_msg_controller m;
		memcpy(&m, buf, sizeof(m));
		if (m.hand > 1) {
			return;
		}

		os_mutex_lock(&sys->mutex);
		struct pico4_controller *c = sys->ctrl[m.hand];
		if (c == NULL) {
			os_mutex_unlock(&sys->mutex);
			return;
		}
		// Recenter on a long press of the home button (as stock): a short press
		// is the system button for apps, and recentering by accident moved the
		// whole world (5 times in one VRChat session).
		bool system_down = (m.buttons & PICO4_BTN_SYSTEM) != 0;
		if (system_down && !c->system_was_down) {
			c->system_down_ns = now;
			c->recentered = false;
		}
		if (system_down && !c->recentered && now - c->system_down_ns > P4_RECENTER_HOLD_NS) {
			recenter_locked(sys);
			c->recentered = true;
		}
		c->system_was_down = system_down;
		c->state = m;
		c->state_ns = now;
		os_mutex_unlock(&sys->mutex);
		return;
	}

	if ((len == sizeof(struct pico4_msg_controller_imu) && buf[0] == PICO4_MSG_CONTROLLER_IMU) ||
	    (len == sizeof(struct pico4_msg_controller_imu_raw) && buf[0] == PICO4_MSG_CONTROLLER_IMU_RAW)) {
		// The old message has no controller counter (and its gyro bias is removed by the bridge).
		struct pico4_msg_controller_imu_raw m = {0};
		if (buf[0] == PICO4_MSG_CONTROLLER_IMU) {
			struct pico4_msg_controller_imu o;
			memcpy(&o, buf, sizeof(o));
			m.hand = o.hand;
			m.timestamp_ns = o.timestamp_ns;
			memcpy(m.gyro, o.gyro, sizeof(m.gyro));
			memcpy(m.accel, o.accel, sizeof(m.accel));
		} else {
			memcpy(&m, buf, sizeof(m));
		}
		if (m.hand > 1) {
			return;
		}

		os_mutex_lock(&sys->mutex);
		struct pico4_controller *c = sys->ctrl[m.hand];
		if (c != NULL) {
			int64_t t = p4_clock_ssc(&sys->ssc_clk, m.timestamp_ns, now);
			t = p4_clock_ctrl(&c->clk, m.ctrl_us, t);
			float g[3], a[3];
			memcpy(g, m.gyro, sizeof(g));
			memcpy(a, m.accel, sizeof(a));
			handle_controller_imu_locked(sys, c, t, g, a);
		}
		os_mutex_unlock(&sys->mutex);
		return;
	}

	if (len == sizeof(struct pico4_msg_controller_pose) && buf[0] == PICO4_MSG_CONTROLLER_POSE) {
		struct pico4_msg_controller_pose m;
		memcpy(&m, buf, sizeof(m));
		if (m.hand > 1) {
			return;
		}
		os_mutex_lock(&sys->mutex);
		struct pico4_controller *c = sys->ctrl[m.hand];
		if (c != NULL) {
			handle_controller_pose_locked(sys, c, &m, now);
		}
		os_mutex_unlock(&sys->mutex);
		return;
	}

	if (len == sizeof(struct pico4_msg_head_pose) && buf[0] == PICO4_MSG_HEAD_POSE) {
		struct pico4_msg_head_pose m;
		memcpy(&m, buf, sizeof(m));
		if (!sys->slam.enabled) {
			return;
		}
		os_mutex_lock(&sys->mutex);
		handle_head_pose_locked(sys, &m, now);
		os_mutex_unlock(&sys->mutex);
		return;
	}

	P4_WARN(sys, "Unknown %zd byte message type %u", len, len > 0 ? buf[0] : 0);
}

static void *
socket_thread(void *ptr)
{
	struct pico4_system *sys = ptr;
	uint8_t buf[256];

	os_thread_helper_name(&sys->oth, "PICO 4 bridge");

	while (os_thread_helper_is_running(&sys->oth)) {
		struct pollfd pfd = {.fd = sys->fd, .events = POLLIN};
		int ret = poll(&pfd, 1, 100);
		if (ret <= 0) {
			continue;
		}
		ssize_t len = recv(sys->fd, buf, sizeof(buf), 0);
		if (len < 0) {
			if (errno != EINTR && errno != EAGAIN) {
				P4_ERROR(sys, "recv: %s", strerror(errno));
				break;
			}
			continue;
		}
		handle_msg(sys, buf, len);
	}

	return NULL;
}

static int
open_socket(struct pico4_system *sys)
{
	snprintf(sys->path, sizeof(sys->path), "%s", debug_get_option_pico4_socket());

	int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
	if (fd < 0) {
		return -1;
	}

	struct sockaddr_un addr = {.sun_family = AF_UNIX};
	snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", sys->path);
	unlink(sys->path);
	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		P4_ERROR(sys, "bind %s: %s", sys->path, strerror(errno));
		close(fd);
		return -1;
	}
	// The bridge may run as another user.
	chmod(sys->path, 0666);

	return fd;
}

//! Shared memory for the LED tracker (see pico4_ctrl_shm); NULL if it cannot be made.
static struct pico4_ctrl_shm *
ctrl_shm_open(struct pico4_system *sys)
{
	int fd = open(PICO4_CTRL_SHM_PATH, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
	if (fd < 0 || ftruncate(fd, sizeof(struct pico4_ctrl_shm)) < 0) {
		P4_WARN(sys, "%s: %s", PICO4_CTRL_SHM_PATH, strerror(errno));
		if (fd >= 0) {
			close(fd);
		}
		return NULL;
	}
	struct pico4_ctrl_shm *shm = mmap(NULL, sizeof(*shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (shm == MAP_FAILED) {
		return NULL;
	}
	memset(shm, 0, sizeof(*shm));
	shm->version = PICO4_CTRL_SHM_VERSION;
	__atomic_store_n(&shm->magic, PICO4_CTRL_SHM_MAGIC, __ATOMIC_RELEASE);
	return shm;
}

static void
system_destroy(struct pico4_system *sys)
{
	os_thread_helper_destroy(&sys->oth);
	if (sys->fd >= 0) {
		close(sys->fd);
		unlink(sys->path);
	}
	m_imu_3dof_close(&sys->fusion);
	if (sys->head_eskf) {
		p4_head_print_stats(&sys->head, stderr);
	}
	if (sys->shm != NULL) {
		munmap(sys->shm, sizeof(*sys->shm));
	}
	if (sys->rec != NULL) {
		fclose(sys->rec);
	}
	os_mutex_destroy(&sys->mutex);
	free(sys);
}

static void
system_unref_device(struct pico4_system *sys, struct xrt_device *xdev)
{
	os_mutex_lock(&sys->mutex);
	if (sys->hmd != NULL && &sys->hmd->base == xdev) {
		sys->hmd = NULL;
	}
	for (int i = 0; i < 2; i++) {
		if (sys->ctrl[i] != NULL && &sys->ctrl[i]->base == xdev) {
			sys->ctrl[i] = NULL;
		}
	}
	bool last = sys->hmd == NULL && sys->ctrl[0] == NULL && sys->ctrl[1] == NULL;
	os_mutex_unlock(&sys->mutex);

	if (last) {
		system_destroy(sys);
	}
}


/*
 *
 * HMD.
 *
 */

static xrt_result_t
hmd_get_tracked_pose(struct xrt_device *xdev,
                     enum xrt_input_name name,
                     int64_t at_timestamp_ns,
                     struct xrt_space_relation *out_relation)
{
	struct pico4_hmd *hmd = (struct pico4_hmd *)xdev;
	struct pico4_system *sys = hmd->sys;

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(xdev, sys->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	int64_t now = os_monotonic_get_ns();
	bool position_tracked = false;

	os_mutex_lock(&sys->mutex);
	math_quat_rotate(&sys->recenter, &sys->head_rot, &rel.pose.orientation);
	math_quat_rotate_vec3(&rel.pose.orientation, &sys->last_gyro, &rel.angular_velocity);
	int64_t sample_ns = sys->last_imu_ns;

	rel.pose.position = (struct xrt_vec3){0.0f, P4_DEFAULT_HEIGHT, 0.0f};
	const struct pico4_slam *sl = &sys->slam;
	if (sl->have_pos) {
		struct xrt_vec3 p = sl->pos, v = {0};
		position_tracked = sl->state == PICO4_SLAM_TRACKING && now - sl->msg_ns < P4_SLAM_TIMEOUT_NS;
		if (position_tracked) {
			// Bring the SLAM sample up to the IMU sample time; m_predict_relation does the rest.
			double dt = time_ns_to_s((sample_ns != 0 ? sample_ns : now) - sl->sample_ns);
			dt = dt < 0.0 ? 0.0 : (dt > 0.1 ? 0.1 : dt);
			p = m_vec3_add(p, m_vec3_mul_scalar(sl->vel, (float)dt));
			v = sl->vel;
		}
		if (sl->spread > 0.0f) {
			double age = time_ns_to_s(now - sl->blend_ns);
			p = m_vec3_add(p, m_vec3_mul_scalar(sl->blend, (float)exp(-age / sl->spread)));
		}
		double pe[3], ve[3], qf[4];
		quat_to_d(&sys->head_rot, qf);
		if (sys->head_eskf && p4_head_position(&sys->head, sys->head_imu_t, qf, pe, ve)) {
			// At the last IMU sample time, like the orientation; m_predict_relation does the rest.
			p = (struct xrt_vec3){(float)pe[0], (float)pe[1], (float)pe[2]};
			v = position_tracked ? (struct xrt_vec3){(float)ve[0], (float)ve[1], (float)ve[2]} : (struct xrt_vec3){0};
		}

		struct xrt_vec3 d = m_vec3_sub(p, sys->recenter_from);
		math_quat_rotate_vec3(&sys->recenter, &d, &rel.pose.position);
		rel.pose.position = m_vec3_add(rel.pose.position, sys->recenter_to);
		math_quat_rotate_vec3(&sys->recenter, &v, &rel.linear_velocity);
	}
	os_mutex_unlock(&sys->mutex);

	math_quat_normalize(&rel.pose.orientation);
	rel.relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                     XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
	if (sample_ns != 0) {
		rel.relation_flags |= XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
	}
	if (position_tracked) {
		rel.relation_flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
	}

	double dt = sample_ns != 0 ? time_ns_to_s(at_timestamp_ns - sample_ns) : 0.0;
	if (dt < 0.0) {
		dt = 0.0;
	} else if (dt > 0.1) {
		dt = 0.1;
	}
	m_predict_relation(&rel, dt, out_relation);

	os_mutex_lock(&sys->mutex);
	sys->head_pose = out_relation->pose;
	sys->have_head_pose = true;
	os_mutex_unlock(&sys->mutex);

	return XRT_SUCCESS;
}

static xrt_result_t
hmd_get_visibility_mask(struct xrt_device *xdev,
                        enum xrt_visibility_mask_type type,
                        uint32_t view_index,
                        struct xrt_visibility_mask **out_mask)
{
	struct xrt_fov fov = xdev->hmd->distortion.fov[view_index];
	u_visibility_mask_get_default(type, &fov, out_mask);
	return XRT_SUCCESS;
}

//! The calibration file stores every number as a string.
static bool
json_number(const cJSON *obj, const char *name, float *out)
{
	const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
	char *end = NULL;

	if (cJSON_IsNumber(item)) {
		*out = (float)item->valuedouble;
		return true;
	}
	if (!cJSON_IsString(item)) {
		return false;
	}
	*out = strtof(item->valuestring, &end);
	return end != item->valuestring;
}

static bool
lens_load(struct pico4_hmd *hmd)
{
	const char *path = debug_get_option_pico4_optical_calib();
	char *text = u_file_read_content_from_path(path, NULL);
	if (text == NULL) {
		P4_WARN(hmd->sys, "No %s, using the stock default lens model", path);
		return false;
	}
	cJSON *root = cJSON_Parse(text);
	free(text);

	const cJSON *params = cJSON_GetObjectItemCaseSensitive(root, "distortion_parameter");
	static const char *const side[2] = {"left", "right"};
	struct pico4_lens lens[2];
	bool ok = cJSON_IsObject(params);

	for (int eye = 0; ok && eye < 2; eye++) {
		const cJSON *p = cJSON_GetObjectItemCaseSensitive(params, side[eye]);
		ok = json_number(p, "F", &lens[eye].f) && lens[eye].f > 0.0f;
		for (int i = 0; ok && i < LENS_K_COUNT; i++) {
			char name[8];
			snprintf(name, sizeof(name), "A%d", i);
			ok = json_number(p, name, &lens[eye].k[i]);
		}
	}
	cJSON_Delete(root);

	if (!ok) {
		P4_WARN(hmd->sys, "%s has no usable distortion_parameter, using the stock default", path);
		return false;
	}
	memcpy(hmd->lens, lens, sizeof(lens));
	for (int eye = 0; eye < 2; eye++) {
		const float *k = lens[eye].k;
		P4_INFO(hmd->sys, "%s lens: F %f K %f %f %f %f %f %f %f", side[eye], lens[eye].f, k[0], k[1], k[2], k[3],
		        k[4], k[5], k[6]);
	}
	return true;
}

static float
lens_scale(const struct pico4_lens *lens, float rsq)
{
	float scale = 0.0f;
	for (int i = LENS_K_COUNT - 1; i >= 0; i--) {
		scale = scale * rsq + lens->k[i];
	}
	return scale;
}

static xrt_result_t
hmd_compute_distortion(struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result)
{
	struct pico4_hmd *hmd = (struct pico4_hmd *)xdev;
	const struct pico4_lens *lens = &hmd->lens[view != 0];

	// Panel point relative to the lens centre in tan angles at the centre.
	float tx = (u - 0.5f) * PANEL_W_METERS / lens->f;
	float ty = (v - 0.5f) * PANEL_W_METERS / lens->f;
	float rsq = tx * tx + ty * ty;
	float scale = lens_scale(lens, rsq < lens_max_rsq ? rsq : lens_max_rsq);
	float ca[3] = {
	    1.0f + lens_ca[0] + lens_ca[1] * rsq,
	    1.0f,
	    1.0f + lens_ca[2] + lens_ca[3] * rsq,
	};
	struct xrt_vec2 *out[3] = {&out_result->r, &out_result->g, &out_result->b};

	for (int c = 0; c < 3; c++) {
		// Eye-buffer tan angle to texture coordinates over the render FOV.
		float s = rsq < lens_max_rsq ? scale * ca[c] : 1e3f;
		out[c]->x = 0.5f + tx * s / (2.0f * hmd->tan_half_fov);
		out[c]->y = 0.5f + ty * s / (2.0f * hmd->tan_half_fov);
	}
	return XRT_SUCCESS;
}

static void
hmd_destroy(struct xrt_device *xdev)
{
	struct pico4_hmd *hmd = (struct pico4_hmd *)xdev;
	struct pico4_system *sys = hmd->sys;

	u_var_remove_root(hmd);
	u_device_free(&hmd->base);
	system_unref_device(sys, xdev);
}

static struct pico4_hmd *
hmd_create(struct pico4_system *sys)
{
	enum u_device_alloc_flags flags = U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE;
	struct pico4_hmd *hmd = U_DEVICE_ALLOCATE(struct pico4_hmd, flags, 1, 0);
	if (hmd == NULL) {
		return NULL;
	}
	hmd->sys = sys;

	u_device_populate_function_pointers(&hmd->base, hmd_get_tracked_pose, hmd_destroy);
	hmd->base.get_view_poses = u_device_get_view_poses;
	hmd->base.get_visibility_mask = hmd_get_visibility_mask;

	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	hmd->base.inputs[0].active = true;
	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.position_tracking = true;
	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "PICO 4");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "PICO 4 HMD");

	size_t idx = 0;
	hmd->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = idx;

	for (int eye = 0; eye < 2; eye++) {
		hmd->lens[eye].f = lens_default_f;
		memcpy(hmd->lens[eye].k, lens_default_k, sizeof(lens_default_k));
	}
	lens_load(hmd);

	// Render the stock frustum unless overridden with PICO4_FOV (degrees).
	float fov = 2.0f * atanf(lens_tan_half_fov);
	if (debug_get_float_option_pico4_fov() > 0.0f) {
		fov = DEG_TO_RAD(debug_get_float_option_pico4_fov());
	}
	hmd->tan_half_fov = tanf(fov / 2.0f);

	// 64.5 mm is the stock default IPD; the panels are 2.56" squares.
	struct u_device_simple_info info = {
	    .display = {.w_pixels = PANEL_W * 2, .h_pixels = PANEL_H, .w_meters = 0.130f, .h_meters = 0.065f},
	    .lens_horizontal_separation_meters = 0.0645f,
	    .lens_vertical_position_meters = 0.0325f,
	    .fov = {fov, fov},
	};
	if (!u_device_setup_split_side_by_side(&hmd->base, &info)) {
		P4_ERROR(sys, "Failed to set up display info");
		u_device_free(&hmd->base);
		return NULL;
	}

	hmd->base.hmd->screens[0].nominal_frame_interval_ns =
	    time_s_to_ns(1.0 / (double)debug_get_num_option_pico4_refresh());

	if (debug_get_bool_option_pico4_no_distortion()) {
		u_distortion_mesh_set_none(&hmd->base);
	} else {
		hmd->base.compute_distortion = hmd_compute_distortion;
		u_distortion_mesh_fill_in_compute(&hmd->base);
	}

	u_var_add_root(hmd, "PICO 4 HMD", true);
	u_var_add_log_level(hmd, &sys->log_level, "log_level");
	m_imu_3dof_add_vars(&sys->fusion, hmd, "");
	u_var_add_bool(hmd, &sys->slam.enabled, "slam.enabled");
	u_var_add_ro_i32(hmd, &sys->slam.state, "slam.state");
	u_var_add_ro_u64(hmd, &sys->slam.poses, "slam.poses");
	u_var_add_ro_vec3_f32(hmd, &sys->slam.pos, "slam.pos");
	u_var_add_ro_vec3_f32(hmd, &sys->slam.vel, "slam.vel");
	u_var_add_f32(hmd, &sys->slam.align_rate, "slam.align_rate");
	u_var_add_f32(hmd, &sys->slam.mincutoff, "slam.mincutoff");
	u_var_add_f32(hmd, &sys->slam.beta, "slam.beta");
	u_var_add_f32(hmd, &sys->slam.still_cutoff, "slam.still_cutoff");
	u_var_add_f32(hmd, &sys->slam.dcutoff, "slam.dcutoff");
	u_var_add_f32(hmd, &sys->slam.spread, "slam.spread");
	u_var_add_bool(hmd, &sys->slam.yaw_fix, "slam.yaw_fix");
	u_var_add_f32(hmd, &sys->slam.yaw_gain, "slam.yaw_gain");
	u_var_add_ro_u32(hmd, &sys->slam.realigns, "slam.realigns");

	return hmd;
}


/*
 *
 * Controllers.
 *
 */

static xrt_result_t
ctrl_update_inputs(struct xrt_device *xdev)
{
	struct pico4_controller *c = (struct pico4_controller *)xdev;
	struct pico4_system *sys = c->sys;
	struct xrt_input *in = xdev->inputs;

	os_mutex_lock(&sys->mutex);
	struct pico4_msg_controller s = c->state;
	int64_t ts = c->state_ns;
	os_mutex_unlock(&sys->mutex);

	for (int i = 0; i < P4_IN_COUNT; i++) {
		in[i].timestamp = ts;
		in[i].active = true;
	}

	uint32_t b = s.buttons;
	in[P4_IN_AX_CLICK].value.boolean = (b & PICO4_BTN_AX) != 0;
	in[P4_IN_AX_TOUCH].value.boolean = (b & (PICO4_TOUCH_AX | PICO4_BTN_AX)) != 0;
	in[P4_IN_BY_CLICK].value.boolean = (b & PICO4_BTN_BY) != 0;
	in[P4_IN_BY_TOUCH].value.boolean = (b & (PICO4_TOUCH_BY | PICO4_BTN_BY)) != 0;
	// Left: menu, right: system. PICO's own system button stays with recentering.
	in[P4_IN_MENU_SYSTEM_CLICK].value.boolean = (b & (c->hand == 0 ? PICO4_BTN_MENU : PICO4_BTN_SYSTEM)) != 0;

	float grip = s.grip;
	if (b & PICO4_BTN_GRIP) {
		grip = 1.0f;
	} else if (grip > 0.02f) {
		grip = fminf(1.0f, grip * 1.35f);
	}
	in[P4_IN_SQUEEZE_VALUE].value.vec1.x = grip;

	float trig = s.trigger;
	if (b & PICO4_BTN_TRIGGER) {
		trig = 1.0f;
	} else if (trig > 0.02f) {
		trig = fminf(1.0f, trig * 1.35f);
	}
	in[P4_IN_TRIGGER_TOUCH].value.boolean = (b & PICO4_TOUCH_TRIGGER) != 0 || trig > 0.05f;
	in[P4_IN_TRIGGER_VALUE].value.vec1.x = trig;
	in[P4_IN_THUMBSTICK_CLICK].value.boolean = (b & PICO4_BTN_STICK) != 0;
	in[P4_IN_THUMBSTICK_TOUCH].value.boolean = (b & (PICO4_TOUCH_STICK | PICO4_BTN_STICK)) != 0;
	in[P4_IN_THUMBSTICK].value.vec2.x = s.stick[0];
	in[P4_IN_THUMBSTICK].value.vec2.y = s.stick[1];
	in[P4_IN_THUMBREST_TOUCH].value.boolean = (b & PICO4_TOUCH_THUMBREST) != 0;

	return XRT_SUCCESS;
}

static xrt_result_t
ctrl_get_tracked_pose(struct xrt_device *xdev,
                      enum xrt_input_name name,
                      int64_t at_timestamp_ns,
                      struct xrt_space_relation *out_relation)
{
	struct pico4_controller *c = (struct pico4_controller *)xdev;

	if (name != XRT_INPUT_TOUCH_GRIP_POSE && name != XRT_INPUT_TOUCH_AIM_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(xdev, c->sys->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct pico4_system *sys = c->sys;
	const struct xrt_vec3 up = {0.0f, 1.0f, 0.0f};
	const struct xrt_vec3 forearm_local = {0.0f, 0.0f, -P4_CTRL_FOREARM};
	int64_t now = os_monotonic_get_ns();
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	struct xrt_quat swing, head_yaw, mount_inv;

	os_mutex_lock(&sys->mutex);
	const struct p4_eskf_core *f = &c->tr.f.cur;
	bool have_imu = c->imu_count >= P4_CTRL_ALIGN_AFTER;
	int64_t ref_ns = have_imu ? f->t : now;

	// Head now, in the IMU world (recentering is applied at the end).
	struct xrt_quat q_wh = sys->head_rot;
	struct xrt_vec3 p_wh = slam_history_lookup_locked(sys, ref_ns - sys->cam_to_imu_ns);
	math_quat_decompose_swing_twist(&q_wh, &up, &swing, &head_yaw);

	// Grip orientation: the filter's IMU orientation through the mount.
	struct xrt_quat q_wi = quat_from_d(f->q);
	math_quat_invert(&c->mount, &mount_inv);
	math_quat_rotate(&q_wi, &mount_inv, &rel.pose.orientation);
	struct xrt_vec3 w_i = {(float)f->w[0], (float)f->w[1], (float)f->w[2]};
	math_quat_rotate_vec3(&q_wi, &w_i, &rel.angular_velocity);
	rel.relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT;
	if (have_imu) {
		rel.relation_flags |=
		    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
	} else {
		// No controller IMU: orientation locked, pointing forward along the head's heading.
		rel.pose.orientation = head_yaw;
		rel.angular_velocity = (struct xrt_vec3){0};
	}
	struct xrt_vec3 forearm;
	math_quat_rotate_vec3(&rel.pose.orientation, &forearm_local, &forearm);

	struct xrt_vec3 p_f = {(float)f->p[0], (float)f->p[1], (float)f->p[2]};
	struct xrt_vec3 v_f = {(float)f->v[0], (float)f->v[1], (float)f->v[2]};
	int64_t age = f->t - c->tr.last_fix_t;
	bool optical = have_imu && c->tr.locked && age < P4_CTRL_TRACKED_NS;

	if (optical) {
		// In view (or just left it): the filter, IMU-propagated between fixes.
		rel.pose.position = p_f;
		rel.linear_velocity = v_f;
		rel.relation_flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
	} else if (have_imu && c->tr.locked) {
		// Out of view: the hand hangs on the elbow it had when it left the
		// view, kept in the head's yaw frame (it follows the body), and turns
		// with the IMU. Blended in from the filter's coasting estimate; no
		// open-loop double integration, which made controllers fly off.
		struct xrt_quat yaw_inv;
		math_quat_invert(&head_yaw, &yaw_inv);
		if (c->hold_fix_t != c->tr.last_fix_t) {
			struct xrt_vec3 elbow = m_vec3_sub(m_vec3_sub(p_f, forearm), p_wh);
			math_quat_rotate_vec3(&yaw_inv, &elbow, &c->hold_elbow_hy);
			c->hold_fix_t = c->tr.last_fix_t;
		}
		struct xrt_vec3 elbow;
		math_quat_rotate_vec3(&head_yaw, &c->hold_elbow_hy, &elbow);
		struct xrt_vec3 pivot = m_vec3_add(m_vec3_add(p_wh, elbow), forearm);
		float w = (float)((double)(age - P4_CTRL_TRACKED_NS) / (double)P4_CTRL_BLEND_NS);
		w = w < 0.0f ? 0.0f : (w > 1.0f ? 1.0f : w);
		rel.pose.position = m_vec3_lerp(p_f, pivot, w);
		rel.linear_velocity = m_vec3_mul_scalar(v_f, 1.0f - w);
		rel.relation_flags |= XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
		if (w >= 1.0f && now - c->hold_set_ns > 200 * U_TIME_1MS_IN_NS) {
			// Keep the filter's coasting position on the hold, so its numbers stay sane.
			double pv[3] = {pivot.x, pivot.y, pivot.z};
			p4_eskf_set_position(&c->tr.f, pv, 0.40, 0.8);
			c->hold_set_ns = now;
		}
	} else {
		// Never seen by the cameras: arm model (elbow below and beside the head, forearm along -z).
		struct xrt_vec3 elbow_local = {c->hand == 0 ? -0.20f : 0.20f, -0.50f, -0.05f};
		struct xrt_vec3 elbow;
		math_quat_rotate_vec3(&head_yaw, &elbow_local, &elbow);
		rel.pose.position = m_vec3_add(m_vec3_add(p_wh, elbow), forearm);
	}

	// Position only: when the cameras pick the controller up again, ease out
	// the jump from the held estimate (rotation is never smoothed).
	if (c->have_out && optical && !c->out_optical) {
		c->corr_pos = m_vec3_sub(c->out_pos, rel.pose.position);
		c->corr_ns = now;
	}
	float keep = c->have_out ? (float)exp(-time_ns_to_s(now - c->corr_ns) / P4_CTRL_SWITCH_TAU_S) : 0.0f;
	if (keep > 0.001f) {
		rel.pose.position = m_vec3_add(rel.pose.position, m_vec3_mul_scalar(c->corr_pos, keep));
	}
	c->out_optical = optical;
	c->out_pos = rel.pose.position;
	c->have_out = true;

	// The aim pose (pointer ray) is pitched like other controllers' (held
	// naturally, the IMU-derived forward axis points 10-60 deg up). The grip
	// stays along the handle: xrizer adds its own Touch offset to the grip
	// for OpenVR games, and a tilted grip made the controllers lie flat there.
	if (have_imu && name == XRT_INPUT_TOUCH_AIM_POSE) {
		struct xrt_quat t;
		math_quat_rotate(&rel.pose.orientation, &c->tilt, &t);
		rel.pose.orientation = t;
	}

	// Into the recentered space the head is shown in.
	struct xrt_quat rot;
	struct xrt_vec3 d = m_vec3_sub(rel.pose.position, sys->recenter_from), tmp;
	math_quat_rotate(&sys->recenter, &rel.pose.orientation, &rot);
	rel.pose.orientation = rot;
	math_quat_rotate_vec3(&sys->recenter, &d, &rel.pose.position);
	rel.pose.position = m_vec3_add(rel.pose.position, sys->recenter_to);
	math_quat_rotate_vec3(&sys->recenter, &rel.linear_velocity, &tmp);
	rel.linear_velocity = tmp;
	math_quat_rotate_vec3(&sys->recenter, &rel.angular_velocity, &tmp);
	rel.angular_velocity = tmp;
	os_mutex_unlock(&sys->mutex);

	math_quat_normalize(&rel.pose.orientation);
	double dt = time_ns_to_s(at_timestamp_ns - ref_ns);
	dt = dt < 0.0 ? 0.0 : (dt > 0.1 ? 0.1 : dt);
	m_predict_relation(&rel, dt, out_relation);
	return XRT_SUCCESS;
}

static void
ctrl_destroy(struct xrt_device *xdev)
{
	struct pico4_controller *c = (struct pico4_controller *)xdev;
	struct pico4_system *sys = c->sys;

	u_var_remove_root(c);
	u_device_free(&c->base);
	system_unref_device(sys, xdev);
}

/*!
 * Controller IMU axes -> OpenXR grip axes (x right, y up, z back along the
 * forearm). Held normally, gravity lies mostly on the IMU's -z, tilted towards
 * y (2026-10-21 captures): +90 degrees about x. Confirmed 2026-10-26 from
 * camera poses (head pitched ~40 deg down at the controllers held forward):
 * LED +x = up, LED -z = forward, LED y = left; with accel = P * led that is
 * exactly this mount. (A 180-degree flip tried the same day was wrong.)
 * PICO4_CTRL_MOUNT(_LEFT)="rx,ry,rz" (degrees) adds a rotation.
 */
static struct xrt_quat
ctrl_mount(int hand)
{
	const struct xrt_vec3 ax = {1.0f, 0.0f, 0.0f}, ay = {0.0f, 1.0f, 0.0f}, az = {0.0f, 0.0f, 1.0f};
	struct xrt_quat q;
	math_quat_from_angle_vector((float)(M_PI / 2.0), &ax, &q);

	const char *extra = hand == 0 && debug_get_option_pico4_ctrl_mount_left() != NULL
	                        ? debug_get_option_pico4_ctrl_mount_left()
	                        : debug_get_option_pico4_ctrl_mount();
	float r[3];
	if (extra != NULL && sscanf(extra, "%f,%f,%f", &r[0], &r[1], &r[2]) == 3) {
		struct xrt_quat qx, qy, qz, qyx, e, base = q;
		math_quat_from_angle_vector(r[0] * (float)(M_PI / 180.0), &ax, &qx);
		math_quat_from_angle_vector(r[1] * (float)(M_PI / 180.0), &ay, &qy);
		math_quat_from_angle_vector(r[2] * (float)(M_PI / 180.0), &az, &qz);
		math_quat_rotate(&qy, &qx, &qyx);
		math_quat_rotate(&qz, &qyx, &e);
		math_quat_rotate(&e, &base, &q);
	}
	math_quat_normalize(&q);
	return q;
}

static struct pico4_controller *
ctrl_create(struct pico4_system *sys, int hand)
{
	struct pico4_controller *c =
	    U_DEVICE_ALLOCATE(struct pico4_controller, U_DEVICE_ALLOC_TRACKING_NONE, P4_IN_COUNT, 1);
	if (c == NULL) {
		return NULL;
	}
	c->sys = sys;
	c->hand = hand;

	u_device_populate_function_pointers(&c->base, ctrl_get_tracked_pose, ctrl_destroy);
	c->base.update_inputs = ctrl_update_inputs;
	c->base.name = XRT_DEVICE_TOUCH_CONTROLLER;
	c->base.device_type = hand == 0 ? XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER : XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
	snprintf(c->base.str, XRT_DEVICE_NAME_LEN, "PICO 4 %s Controller", hand == 0 ? "Left" : "Right");
	snprintf(c->base.serial, XRT_DEVICE_NAME_LEN, "PICO 4 %s Controller", hand == 0 ? "Left" : "Right");

	struct xrt_input *in = c->base.inputs;
	in[P4_IN_AX_CLICK].name = hand == 0 ? XRT_INPUT_TOUCH_X_CLICK : XRT_INPUT_TOUCH_A_CLICK;
	in[P4_IN_AX_TOUCH].name = hand == 0 ? XRT_INPUT_TOUCH_X_TOUCH : XRT_INPUT_TOUCH_A_TOUCH;
	in[P4_IN_BY_CLICK].name = hand == 0 ? XRT_INPUT_TOUCH_Y_CLICK : XRT_INPUT_TOUCH_B_CLICK;
	in[P4_IN_BY_TOUCH].name = hand == 0 ? XRT_INPUT_TOUCH_Y_TOUCH : XRT_INPUT_TOUCH_B_TOUCH;
	in[P4_IN_MENU_SYSTEM_CLICK].name = hand == 0 ? XRT_INPUT_TOUCH_MENU_CLICK : XRT_INPUT_TOUCH_SYSTEM_CLICK;
	in[P4_IN_SQUEEZE_VALUE].name = XRT_INPUT_TOUCH_SQUEEZE_VALUE;
	in[P4_IN_TRIGGER_TOUCH].name = XRT_INPUT_TOUCH_TRIGGER_TOUCH;
	in[P4_IN_TRIGGER_VALUE].name = XRT_INPUT_TOUCH_TRIGGER_VALUE;
	in[P4_IN_THUMBSTICK_CLICK].name = XRT_INPUT_TOUCH_THUMBSTICK_CLICK;
	in[P4_IN_THUMBSTICK_TOUCH].name = XRT_INPUT_TOUCH_THUMBSTICK_TOUCH;
	in[P4_IN_THUMBSTICK].name = XRT_INPUT_TOUCH_THUMBSTICK;
	in[P4_IN_THUMBREST_TOUCH].name = XRT_INPUT_TOUCH_THUMBREST_TOUCH;
	in[P4_IN_GRIP_POSE].name = XRT_INPUT_TOUCH_GRIP_POSE;
	in[P4_IN_AIM_POSE].name = XRT_INPUT_TOUCH_AIM_POSE;
	for (int i = 0; i < P4_IN_COUNT; i++) {
		in[i].active = true;
	}
	c->base.outputs[0].name = XRT_OUTPUT_NAME_TOUCH_HAPTIC;
	c->base.supported.orientation_tracking = true;

	// Until the controller IMU runs: hold them in front of the user at hip height.
	c->pose = (struct xrt_pose){XRT_QUAT_IDENTITY, {hand == 0 ? -0.2f : 0.2f, 1.2f, -0.4f}};
	c->mount = ctrl_mount(hand);
	const struct xrt_vec3 tilt_axis = {1.0f, 0.0f, 0.0f};
	math_quat_from_angle_vector(debug_get_float_option_pico4_ctrl_tilt() * (float)(M_PI / 180.0), &tilt_axis,
	                            &c->tilt);
	// Typical gyro bias at rest (raw counts -6, -7, 4 at 8.2 LSB/dps); the filter refines it.
	const double k = M_PI / 180.0 / 8.2;
	const double bg0[3] = {-6.0 * k, -7.0 * k, 4.0 * k};
	struct p4_eskf_params prm;
	p4_eskf_default_params(&prm);
	p4_track_init(&c->tr, &prm, bg0);
	p4_clock_init(&c->clk);

	u_var_add_root(c, c->base.str, true);
	u_var_add_pose(c, &c->pose, "Pose");

	return c;
}


/*
 *
 * Builder.
 *
 */

static const char *driver_list[] = {"pico4"};

static xrt_result_t
pico4_estimate_system(struct xrt_builder *xb, cJSON *config, struct xrt_prober *xp, struct xrt_builder_estimate *estimate)
{
	// No USB ids to probe for: the headset is the machine we run on.
	bool here = access("/dev/fastrpc-sdsp", F_OK) == 0;
	estimate->certain.head = here;
	estimate->certain.left = here;
	estimate->certain.right = here;
	estimate->priority = 0;
	return XRT_SUCCESS;
}

static xrt_result_t
pico4_open_system_impl(struct xrt_builder *xb,
                       cJSON *config,
                       struct xrt_prober *xp,
                       struct xrt_tracking_origin *origin,
                       struct xrt_system_devices *xsysd,
                       struct xrt_frame_context *xfctx,
                       struct u_builder_roles_helper *ubrh)
{
	struct pico4_system *sys = U_TYPED_CALLOC(struct pico4_system);
	sys->log_level = debug_get_log_option_pico4_log();
	sys->recenter = (struct xrt_quat)XRT_QUAT_IDENTITY;
	sys->yaw_corr = (struct xrt_quat)XRT_QUAT_IDENTITY;
	sys->head_rot = (struct xrt_quat)XRT_QUAT_IDENTITY;
	sys->slam.enabled = !debug_get_bool_option_pico4_no_slam();
	sys->slam.align = (struct xrt_quat)XRT_QUAT_IDENTITY;
	sys->slam.align_rate = debug_get_float_option_pico4_slam_align_rate();
	sys->slam.mincutoff = debug_get_float_option_pico4_slam_mincutoff();
	sys->slam.beta = debug_get_float_option_pico4_slam_beta();
	sys->slam.still_cutoff = debug_get_float_option_pico4_slam_still_cutoff();
	sys->slam.dcutoff = debug_get_float_option_pico4_slam_dcutoff();
	sys->slam.spread = debug_get_float_option_pico4_slam_spread();
	sys->slam.yaw_fix = debug_get_bool_option_pico4_head_yaw_fix();
	sys->slam.yaw_gain = debug_get_float_option_pico4_head_yaw_gain();
	sys->head_eskf = debug_get_bool_option_pico4_head_eskf();
	if (sys->head_eskf) {
		struct p4_head_params hp;
		p4_head_default_params(&hp);
		p4_head_init(&sys->head, &hp);
	}
	os_mutex_init(&sys->mutex);
	os_thread_helper_init(&sys->oth);
	m_imu_3dof_init(&sys->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	p4_clock_init(&sys->ssc_clk);
	sys->cam_to_imu_ns = (int64_t)(debug_get_float_option_pico4_cam_imu_ms() * 1e6);
	sys->shm = ctrl_shm_open(sys);
	const char *rec = debug_get_option_pico4_ctrl_record();
	if (rec != NULL && rec[0] != '\0') {
		sys->rec = fopen(rec, "w");
		P4_INFO(sys, "Recording tracking inputs to %s%s", rec, sys->rec == NULL ? " FAILED" : "");
	}

	sys->fd = open_socket(sys);
	if (sys->fd < 0) {
		system_destroy(sys);
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	sys->hmd = hmd_create(sys);
	if (sys->hmd == NULL) {
		system_destroy(sys);
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}
	sys->ctrl[0] = ctrl_create(sys, 0);
	sys->ctrl[1] = ctrl_create(sys, 1);

	struct xrt_device *head = &sys->hmd->base;
	head->tracking_origin->type = XRT_TRACKING_TYPE_OTHER;
	snprintf(head->tracking_origin->name, XRT_TRACKING_NAME_LEN, "PICO 4 IMU");

	xsysd->xdevs[xsysd->xdev_count++] = head;
	ubrh->head = head;
	for (int i = 0; i < 2; i++) {
		if (sys->ctrl[i] == NULL) {
			continue;
		}
		sys->ctrl[i]->base.tracking_origin = head->tracking_origin;
		xsysd->xdevs[xsysd->xdev_count++] = &sys->ctrl[i]->base;
	}
	ubrh->left = sys->ctrl[0] ? &sys->ctrl[0]->base : NULL;
	ubrh->right = sys->ctrl[1] ? &sys->ctrl[1]->base : NULL;

	os_thread_helper_start(&sys->oth, socket_thread, sys);
	P4_INFO(sys, "Listening for pico4-xr-bridge on %s", sys->path);

	return XRT_SUCCESS;
}

static void
pico4_builder_destroy(struct xrt_builder *xb)
{
	free(xb);
}

struct xrt_builder *
pico4_builder_create(void)
{
	struct u_builder *ub = U_TYPED_CALLOC(struct u_builder);

	ub->base.estimate_system = pico4_estimate_system;
	ub->base.open_system = u_builder_open_system_static_roles;
	ub->base.destroy = pico4_builder_destroy;
	ub->base.identifier = "pico4";
	ub->base.name = "PICO 4 (mainline Linux, SLPI IMU + SLAM + controllers)";
	ub->base.driver_identifiers = driver_list;
	ub->base.driver_identifier_count = ARRAY_SIZE(driver_list);
	ub->base.exclude_from_automatic_discovery = false;

	ub->open_system_static_roles = pico4_open_system_impl;

	return &ub->base;
}
