// Copyright 2026, picoreverse contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 driver for mainline Linux: HMD + controllers fed by pico4-xr-bridge.
 * @ingroup drv_pico4
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_pico4 PICO 4 driver
 * @ingroup drv
 *
 * The headset IMU and the controllers are only reachable through the SLPI
 * (SSC sensors "gyro", "accel" and "ble_mcu"). A userspace bridge
 * (tools/nordic/pico-controllers.py --xr) talks to the SSC and forwards the
 * samples as datagrams to the socket below. The 6DoF tracker (pico4-slam)
 * sends head poses to the same socket.
 */

//! Default datagram socket the bridge sends to, override with PICO4_SOCKET.
#define PICO4_SOCKET_DEFAULT "/run/pico4-xr.sock"

enum pico4_msg_type
{
	PICO4_MSG_HEAD_IMU = 1,
	PICO4_MSG_CONTROLLER = 2,
	PICO4_MSG_HEAD_POSE = 3,
	PICO4_MSG_CONTROLLER_IMU = 4,
	PICO4_MSG_CONTROLLER_POSE = 5,
	PICO4_MSG_CONTROLLER_IMU_RAW = 6,
};

//! Head IMU sample: gyro in rad/s, accel in m/s^2, raw SSC sensor axes.
struct pico4_msg_head_imu
{
	uint8_t type;
	uint8_t pad[7];
	int64_t timestamp_ns; //!< Sensor clock, only used for deltas.
	float gyro[3];
	float accel[3];
} __attribute__((packed));

enum pico4_buttons
{
	PICO4_BTN_AX = 1u << 0,
	PICO4_BTN_BY = 1u << 1,
	PICO4_BTN_MENU = 1u << 2,
	PICO4_BTN_STICK = 1u << 3,
	PICO4_BTN_SYSTEM = 1u << 4,
	PICO4_BTN_TRIGGER = 1u << 5,
	PICO4_BTN_GRIP = 1u << 6,
	PICO4_TOUCH_AX = 1u << 8,
	PICO4_TOUCH_BY = 1u << 9,
	PICO4_TOUCH_STICK = 1u << 10,
	PICO4_TOUCH_THUMBREST = 1u << 11,
	PICO4_TOUCH_TRIGGER = 1u << 12,
};

//! Controller state, one per decoded ble_mcu packet.
struct pico4_msg_controller
{
	uint8_t type;
	uint8_t hand; //!< 0 = left, 1 = right
	uint8_t pad[2];
	uint32_t buttons; //!< @ref pico4_buttons
	float trigger;    //!< 0..1
	float grip;       //!< 0..1
	float stick[2];   //!< -1..1, +y up
} __attribute__((packed));

/*!
 * Controller IMU sample, one per valid ble_mcu packet (~27 Hz per controller).
 * Gyro in rad/s and accel in m/s^2, both in the controller IMU's accelerometer
 * axes (the bridge already maps the gyro onto them and removes its bias).
 */
struct pico4_msg_controller_imu
{
	uint8_t type;
	uint8_t hand; //!< 0 = left, 1 = right
	uint8_t pad[6];
	int64_t timestamp_ns; //!< SSC (QTimer) time of the packet
	float gyro[3];
	float accel[3];
} __attribute__((packed));

/*!
 * Controller IMU sample with both time bases, one per valid ble_mcu packet
 * (~500 Hz per controller). Gyro in rad/s *without* bias removal (the driver's
 * filter estimates the bias) and accel in m/s^2, both in the controller IMU's
 * accelerometer axes.
 */
struct pico4_msg_controller_imu_raw
{
	uint8_t type;
	uint8_t hand; //!< 0 = left, 1 = right
	uint8_t pad[2];
	uint32_t ctrl_us;     //!< Controller-side sample counter, microseconds (wraps)
	int64_t timestamp_ns; //!< SSC (QTimer) time the SLPI read the packet
	float gyro[3];
	float accel[3];
} __attribute__((packed));

/*!
 * Optical controller pose from the IR LED tracker in pico4-slam, one per
 * camera frame pair in which the controller was found. The pose maps the
 * controller IMU frame (the frame of the LED calibration and of
 * @ref pico4_msg_controller_imu) to the OpenXR head frame at the capture time.
 */
struct pico4_msg_controller_pose
{
	uint8_t type;
	uint8_t hand;    //!< 0 = left, 1 = right
	uint8_t inliers; //!< LED images used
	uint8_t pad[5];
	int64_t timestamp_ns; //!< CLOCK_MONOTONIC capture time
	float position[3];    //!< metres, head frame
	float orientation[4]; //!< x, y, z, w
	float rms;            //!< reprojection error, pixels
} __attribute__((packed));

enum pico4_slam_state
{
	PICO4_SLAM_NOT_TRACKING = 0, //!< Not initialised yet, or lost.
	PICO4_SLAM_TRACKING = 1,
	PICO4_SLAM_RECENTLY_LOST = 2, //!< Pose is a short-term guess.
};

/*!
 * Head pose from the visual SLAM tracker (pico4-slam: ORB-SLAM3 on the two
 * lower tracking cameras), one per processed camera frame pair.
 *
 * The pose is the OpenXR head frame (x right, y up, z back, origin between
 * the eyes) in the tracker's own world frame, which is arbitrary (the first
 * camera pose of the map) and changes when @ref map_id changes. The driver
 * aligns it to the IMU's gravity-aligned frame itself.
 */
struct pico4_msg_head_pose
{
	uint8_t type;
	uint8_t state; //!< @ref pico4_slam_state
	uint8_t pad[2];
	uint32_t map_id;      //!< Changes when the tracker starts a new map.
	int64_t timestamp_ns; //!< CLOCK_MONOTONIC capture time of the frames.
	float position[3];    //!< Metres.
	float orientation[4]; //!< x, y, z, w.
} __attribute__((packed));

/*
 * Controller state shared with the LED tracker (pico4-slam), so it can predict
 * where each controller is in a camera frame and reject fits that disagree
 * with the IMU. The driver (writer) maps PICO4_CTRL_SHM_PATH and appends one
 * entry per hand about every 10 ms; readers map it read-only.
 *
 * Seqlock: the writer increments @ref pico4_ctrl_shm::seq (odd = writing),
 * updates, increments again. A reader copies the struct and retries while seq
 * is odd or changed during the copy.
 */
#define PICO4_CTRL_SHM_PATH "/dev/shm/pico4-ctrl-state"
#define PICO4_CTRL_SHM_MAGIC 0x50344353u // "SC4P"
#define PICO4_CTRL_SHM_VERSION 1
#define PICO4_CTRL_SHM_ENTRIES 32

enum pico4_ctrl_shm_flags
{
	PICO4_CTRL_SHM_IMU = 1u << 0,     //!< The controller IMU runs: orientation and up_led are good.
	PICO4_CTRL_SHM_LOCKED = 1u << 1,  //!< Heading locked to the cameras: q_h_led is a full prior.
	PICO4_CTRL_SHM_TRACKED = 1u << 2, //!< Optical fix within the last ~150 ms: p_h is a good prior too.
};

struct pico4_ctrl_shm_entry
{
	int64_t timestamp_ns; //!< CLOCK_MONOTONIC, the camera capture clock
	uint32_t flags;       //!< @ref pico4_ctrl_shm_flags
	float q_h_led[4];     //!< LED model frame -> head frame, x y z w
	float p_h[3];         //!< LED model origin in the head frame, metres
	float up_led[3];      //!< World up (against gravity) in the LED model frame, unit
	float sigma_rot;      //!< 1-sigma orientation uncertainty, radians
	float sigma_pos;      //!< 1-sigma position uncertainty, metres
	float omega_led[3];   //!< Angular velocity, LED model frame, rad/s
} __attribute__((packed));

struct pico4_ctrl_shm
{
	uint32_t magic;
	uint32_t version;
	volatile uint32_t seq;
	uint32_t next[2]; //!< Per hand: index the next entry goes to
	struct pico4_ctrl_shm_entry e[2][PICO4_CTRL_SHM_ENTRIES];
} __attribute__((packed));

struct xrt_builder *
pico4_builder_create(void);

#ifdef __cplusplus
}
#endif
