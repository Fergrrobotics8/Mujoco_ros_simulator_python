#!/usr/bin/env python3
"""
Unified MuJoCo + ROS 2 simulation for Unitree G1.

Combines into a SINGLE MuJoCo process:
  - Low-level motor bridge  (29 body + 14 hand DOF)  @ 250 Hz state, real-time cmd
  - RGBD camera bridge      (RealSense D435i)         @ 10 Hz
  - LIDAR bridge            (Livox Mid360)             @ 10 Hz

Performance: single model/data/viewer, single physics loop,
camera & LIDAR synchronized with physics in main thread.

Run (inside Docker /workspace):
    source /opt/ros/humble/setup.bash
    python3 g1_ros2_mujoco_complete_simulation.py
"""

import os
import sys
import time
import threading
from typing import Dict, List

import numpy as np

# GPU-accelerated EGL context for offscreen rendering (mujoco.Renderer).
# Must be set BEFORE "import mujoco".  The passive viewer always uses GLX/X11
# and is NOT affected by this variable.
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from unitree_hg.msg import LowState, LowCmd, MotorState, MotorCmd, IMUState, HandState, HandCmd
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════════════════════
SCENE_XML   = "scene_uni.xml"

# Rates
VIEWER_HZ   = 60.0
STATE_HZ    = 250.0    # Low-level state publish rate
CAMERA_HZ   = 10.0     # Camera publish rate
LIDAR_HZ    = 10.0     # LIDAR publish rate

# Camera
RGB_W, RGB_H   = 320, 240
DEPTH_W, DEPTH_H = 320, 240

#RGB_W, RGB_H   = 1920, 1080
#DEPTH_W, DEPTH_H = 1280, 720

# LIDAR
LIDAR_SITE   = "livox_mid360_site"
MIN_DIST     = 0.15
CUTOFF_DIST  = 40.0
TOPIC_CLOUD    = "/utlidar/cloud_livox_mid360"
LIDAR_FRAME    = "livox_frame"
LIDAR_PARENT   = "torso_link"   # TF parent (fallback: "world")
N_H            = 360
N_V            = 128
PHI_MIN_DEG    = -60
PHI_MAX_DEG    =  60

# Body: 29 DOF
BODY_JOINT_ORDER = [
    "left_hip_pitch_joint",       # 0
    "left_hip_roll_joint",        # 1
    "left_hip_yaw_joint",         # 2
    "left_knee_joint",            # 3
    "left_ankle_pitch_joint",     # 4
    "left_ankle_roll_joint",      # 5
    "right_hip_pitch_joint",      # 6
    "right_hip_roll_joint",       # 7
    "right_hip_yaw_joint",        # 8
    "right_knee_joint",           # 9
    "right_ankle_pitch_joint",    # 10
    "right_ankle_roll_joint",     # 11
    "waist_yaw_joint",            # 12
    "waist_roll_joint",           # 13
    "waist_pitch_joint",          # 14
    "left_shoulder_pitch_joint",  # 15
    "left_shoulder_roll_joint",   # 16
    "left_shoulder_yaw_joint",    # 17
    "left_elbow_joint",           # 18
    "left_wrist_roll_joint",      # 19
    "left_wrist_pitch_joint",     # 20
    "left_wrist_yaw_joint",       # 21
    "right_shoulder_pitch_joint", # 22
    "right_shoulder_roll_joint",  # 23
    "right_shoulder_yaw_joint",   # 24
    "right_elbow_joint",          # 25
    "right_wrist_roll_joint",     # 26
    "right_wrist_pitch_joint",    # 27
    "right_wrist_yaw_joint",      # 28
]

LEFT_HAND_JOINT_ORDER = [
    "left_hand_thumb_0_joint",    # 0
    "left_hand_thumb_1_joint",    # 1
    "left_hand_thumb_2_joint",    # 2
    "left_hand_index_0_joint",    # 3
    "left_hand_index_1_joint",    # 4
    "left_hand_middle_0_joint",   # 5
    "left_hand_middle_1_joint",   # 6
]

RIGHT_HAND_JOINT_ORDER = [
    "right_hand_thumb_0_joint",   # 0
    "right_hand_thumb_1_joint",   # 1
    "right_hand_thumb_2_joint",   # 2
    "right_hand_index_0_joint",   # 3
    "right_hand_index_1_joint",   # 4
    "right_hand_middle_0_joint",  # 5
    "right_hand_middle_1_joint",  # 6
]


# ═══════════════════════════════════════════════════════════════════════════════
#  LOW-LEVEL HELPERS
# ═══════════════════════════════════════════════════════════════════════════════
def safe_float(x) -> float:
    return float(x) if np.isfinite(x) else 0.0


def build_joint_map(model, joint_order: List[str]):
    """Returns (qpos_indices, qvel_indices, actuator_ids, valid_names) for the given joint names."""
    actuator_name_to_id: Dict[str, int] = {}
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if name is not None:
            actuator_name_to_id[name] = i

    qpos_idx, qvel_idx, act_ids, valid_names = [], [], [], []
    for jname in joint_order:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid < 0:
            qpos_idx.append(None); qvel_idx.append(None)
            act_ids.append(None); valid_names.append(None)
            continue
        qpos_idx.append(model.jnt_qposadr[jid])
        qvel_idx.append(model.jnt_dofadr[jid])
        act_ids.append(actuator_name_to_id.get(jname, None))
        valid_names.append(jname)

    return qpos_idx, qvel_idx, act_ids, valid_names


def read_motor_state(data, model, qpos_idx, qvel_idx, act_id, last_dq, dt) -> MotorState:
    st = MotorState()
    if qpos_idx is None:
        return st
    q   = safe_float(data.qpos[qpos_idx])
    dq  = safe_float(data.qvel[qvel_idx])
    ddq = safe_float((dq - last_dq) / dt)
    tau = 0.0
    if act_id is not None and act_id < len(data.actuator_force):
        tau = safe_float(data.actuator_force[act_id])
    st.q = q; st.dq = dq; st.ddq = ddq; st.tau_est = tau
    st.temperature = [25, 25]; st.vol = 24.0
    st.sensor = [0, 0]; st.motorstate = 0; st.reserve = [0, 0, 0, 0]
    return st


# ═══════════════════════════════════════════════════════════════════════════════
#  CAMERA HELPERS
# ═══════════════════════════════════════════════════════════════════════════════
def _image_msg(array, encoding, frame_id, stamp):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = False
    if encoding == "bgr8":
        msg.step = msg.width * 3
    elif encoding == "32FC1":
        msg.step = msg.width * 4
    else:
        msg.step = msg.width * array.dtype.itemsize
    msg.data = array.tobytes()
    return msg


def _camera_info(w, h, fovy_deg, frame_id, stamp):
    fovy = np.deg2rad(fovy_deg)
    fy = (h / 2.0) / np.tan(fovy / 2.0)
    fx = fy
    cx, cy = w / 2.0, h / 2.0
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = w
    msg.height = h
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0] * 5
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


# ═══════════════════════════════════════════════════════════════════════════════
#  LIDAR HELPERS
# ═══════════════════════════════════════════════════════════════════════════════
def generate_mid360_pattern(scan_phase=0.0):
    """Livox MID360 real pattern: 64 vertical channels, non-uniform horizontal density."""
    phi_levels = np.linspace(np.deg2rad(PHI_MIN_DEG), np.deg2rad(PHI_MAX_DEG), N_V, dtype=np.float64)

    h_rays_per_ring = np.ones(N_V, dtype=int) * N_H
    for i in range(N_V):
        normalized_ring = i / (N_V - 1)
        weight = np.sin(np.pi * normalized_ring) ** 1.5
        h_rays = int(N_H * weight * 0.8 + N_H * 0.2)
        h_rays_per_ring[i] = max(h_rays, N_H // 4)

    theta_list, phi_list, ring_list, time_list = [], [], [], []
    for ring_idx in range(N_V):
        n_h = h_rays_per_ring[ring_idx]
        ring_phase = scan_phase + ring_idx * 0.37
        theta_ring = np.linspace(-np.pi, np.pi, n_h, endpoint=False, dtype=np.float64)
        theta_ring = theta_ring + 0.12 * np.sin(3.0 * theta_ring + ring_phase)
        theta_ring = np.mod(theta_ring + np.pi, 2 * np.pi) - np.pi
        phi_ring = phi_levels[ring_idx] + np.deg2rad(0.8) * np.sin(2.0 * theta_ring + ring_phase)
        time_ring = np.linspace(0.0, 1.0, n_h, endpoint=False, dtype=np.float32)

        theta_list.append(theta_ring)
        phi_list.append(phi_ring)
        ring_list.append(np.full(n_h, ring_idx, dtype=np.uint16))
        time_list.append(time_ring)

    return (
        np.concatenate(theta_list),
        np.concatenate(phi_list),
        np.concatenate(ring_list),
        np.concatenate(time_list),
    )


def trace_rays(model, data, site_id, ray_theta, ray_phi, ray_ring, ray_time, cutoff,
               pnt_buf, vec_buf_2d, vec_buf, dist_buf, geomid_buf, geom_group,
               x_local, y_local, z_local):
    """Optimized raytracing — reuses pre-allocated buffers."""
    n_rays = len(ray_theta)
    site_pos = data.site_xpos[site_id]
    site_mat = data.site_xmat[site_id].reshape(3, 3)

    np.cos(ray_phi, out=x_local[:n_rays])
    np.multiply(x_local[:n_rays], np.cos(ray_theta), out=x_local[:n_rays])
    np.cos(ray_phi, out=y_local[:n_rays])
    np.multiply(y_local[:n_rays], np.sin(ray_theta), out=y_local[:n_rays])
    np.sin(ray_phi, out=z_local[:n_rays])

    vec_buf_2d[:n_rays, 0] = x_local[:n_rays]
    vec_buf_2d[:n_rays, 1] = y_local[:n_rays]
    vec_buf_2d[:n_rays, 2] = z_local[:n_rays]

    np.dot(vec_buf_2d[:n_rays], site_mat.T, out=vec_buf_2d[:n_rays])
    norms = np.linalg.norm(vec_buf_2d[:n_rays], axis=1, keepdims=True)
    vec_buf_2d[:n_rays] /= norms

    vec_buf[:n_rays * 3, 0] = vec_buf_2d[:n_rays].ravel()
    pnt_buf[:, 0] = site_pos

    mujoco.mj_multiRay(
        m=model, d=data,
        pnt=pnt_buf, vec=vec_buf[:n_rays * 3, :],
        geomgroup=geom_group, flg_static=1, bodyexclude=-1,
        geomid=geomid_buf, dist=dist_buf, normal=None,
        nray=n_rays, cutoff=cutoff,
    )

    dist = dist_buf[:n_rays].flatten()
    geomid = geomid_buf[:n_rays].flatten()
    mask = (geomid != -1) & (dist >= MIN_DIST) & (dist < cutoff)

    local_vecs = np.empty((n_rays, 3), dtype=np.float64)
    local_vecs[:, 0] = x_local[:n_rays]
    local_vecs[:, 1] = y_local[:n_rays]
    local_vecs[:, 2] = z_local[:n_rays]

    hit_local = local_vecs[mask] * dist[mask, None]
    hit_dist = dist[mask].astype(np.float32)
    hit_ring = ray_ring[mask]
    hit_time = ray_time[mask]

    return (
        hit_local.astype(np.float32),
        hit_dist,
        hit_ring.astype(np.uint16),
        hit_time.astype(np.float32),
    )


def build_pointcloud2(points, distances, ring, time_arr, frame_id, stamp):
    n = points.shape[0]
    if n == 0:
        return None

    fields = [
        PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring",      offset=16, datatype=PointField.UINT16,  count=1),
        PointField(name="time",      offset=18, datatype=PointField.FLOAT32, count=1),
    ]

    intensity = np.clip(1.0 - distances / CUTOFF_DIST, 0.1, 1.0).astype(np.float32)

    dt = np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
        ("intensity", "<f4"), ("ring", "<u2"), ("time", "<f4"),
    ])
    packed = np.empty(n, dtype=dt)
    packed["x"] = points[:, 0]
    packed["y"] = points[:, 1]
    packed["z"] = points[:, 2]
    packed["intensity"] = intensity
    packed["ring"] = ring
    packed["time"] = time_arr

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = n
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 22
    msg.row_step = 22 * n
    msg.is_dense = True
    msg.data = packed.tobytes()
    return msg



# ═══════════════════════════════════════════════════════════════════════════════
#  ASYNC CAMERA WORKER  (EGL context lives in camera thread, not main thread)
# ═══════════════════════════════════════════════════════════════════════════════
class _CameraThread:
    """Renders RGB + Depth cameras in a background EGL thread.

    ``request()`` snapshots MjData in ~0.2 ms and returns immediately.
    Actual GPU rendering happens in parallel, fully decoupling the viewer
    loop from the 5-20 ms camera render cost.

    Requires ``MUJOCO_GL=egl`` (set before importing mujoco) so that each
    ``mujoco.Renderer`` creates its own EGL context bound to *this* thread.
    """

    def __init__(self, model: mujoco.MjModel) -> None:
        self._model   = model
        self._snap    = mujoco.MjData(model)   # private state snapshot
        self._lock    = threading.Lock()        # protects _snap / _stamp / _node
        self._stamp   = None
        self._node    = None
        self._trigger = threading.Event()
        self._ready   = threading.Event()
        self._stop    = False
        self._thread  = threading.Thread(target=self._loop,
                                         daemon=True, name="cam_render")
        self._thread.start()
        if not self._ready.wait(timeout=10.0):
            raise RuntimeError("_CameraThread: renderer init timed out (EGL missing?)")

    # ── Called from main thread ──────────────────────────────────────────────
    def request(self,
                model: mujoco.MjModel,
                data:  mujoco.MjData,
                stamp,
                node) -> None:
        """Non-blocking: copy sim state (~0.2 ms) and wake the render thread."""
        with self._lock:
            mujoco.mj_copyData(self._snap, model, data)
            self._stamp = stamp
            self._node  = node
        self._trigger.set()

    def stop(self) -> None:
        self._stop = True
        self._trigger.set()
        self._thread.join(timeout=3.0)

    # ── Camera thread loop ───────────────────────────────────────────────────
    def _loop(self) -> None:
        # Renderers created HERE so their EGL context belongs to this thread
        rgb_r   = mujoco.Renderer(self._model, width=RGB_W, height=RGB_H)
        depth_r = mujoco.Renderer(self._model, width=DEPTH_W, height=DEPTH_H)
        depth_r.enable_depth_rendering()   # permanent — no per-frame toggling
        self._ready.set()

        while not self._stop:
            if not self._trigger.wait(timeout=0.5):
                continue
            self._trigger.clear()

            with self._lock:
                stamp = self._stamp
                node  = self._node
                if stamp is None or node is None:
                    continue
                # update_scene copies from _snap into renderer's scene buffer
                rgb_r.update_scene(  self._snap, camera="d435i_rgb")
                depth_r.update_scene(self._snap, camera="d435i_depth")
            # Lock released — renderer has its own scene copy, _snap can be updated

            # GPU readback (slow part) — no lock, no shared state
            rgb = rgb_r.render()
            bgr = rgb[:, :, ::-1].copy()
            node.rgb_pub.publish(
                _image_msg(bgr, "bgr8", "d435i_rgb_optical_frame", stamp))
            node.rgb_info_pub.publish(
                _camera_info(RGB_W, RGB_H, 42.5, "d435i_rgb_optical_frame", stamp))

            depth = depth_r.render().astype(np.float32)
            node.depth_pub.publish(
                _image_msg(depth, "32FC1", "d435i_depth_optical_frame", stamp))
            node.depth_info_pub.publish(
                _camera_info(DEPTH_W, DEPTH_H, 57.0, "d435i_depth_optical_frame", stamp))

        rgb_r.close()
        depth_r.close()


# ═══════════════════════════════════════════════════════════════════════════════
#  ASYNC LIDAR WORKER  (raycasting in background thread — read-only MjData snap)
# ═══════════════════════════════════════════════════════════════════════════════
class _LidarThread:
    """Runs LIDAR raycasting + TF publish in a background CPU thread.

    ``request()`` snapshots MjData in ~0.2 ms and returns immediately.
    The 100 ms ``mj_multiRay`` call runs in parallel, fully decoupling the
    viewer loop from raycasting cost.

    ``mj_multiRay`` only READS geometry / body positions → safe to run on a
    private MjData snapshot alongside the live simulation.
    """

    def __init__(self, model: mujoco.MjModel, max_n_rays: int) -> None:
        self._model   = model
        self._snap    = mujoco.MjData(model)   # private state snapshot
        self._lock    = threading.Lock()        # protects _snap / metadata
        self._stamp      = None
        self._node       = None
        self._scan_phase = 0.0
        self._trigger = threading.Event()
        self._stop    = False

        # Thread-local pre-allocated raycasting buffers
        self._pnt_buf    = np.zeros((3, 1), dtype=np.float64)
        self._vec_buf_2d = np.zeros((max_n_rays, 3), dtype=np.float64)
        self._vec_buf    = np.zeros((max_n_rays * 3, 1), dtype=np.float64)
        self._dist_buf   = np.full((max_n_rays, 1), CUTOFF_DIST, dtype=np.float64)
        self._geomid_buf = np.full((max_n_rays, 1), -1, dtype=np.int32)
        self._geom_group = np.ones((6, 1), dtype=np.uint8)
        self._x_local    = np.empty(max_n_rays, dtype=np.float64)
        self._y_local    = np.empty(max_n_rays, dtype=np.float64)
        self._z_local    = np.empty(max_n_rays, dtype=np.float64)

        self._thread = threading.Thread(target=self._loop,
                                        daemon=True, name="lidar_trace")
        self._thread.start()

    # ── Called from main thread ──────────────────────────────────────────────
    def request(self,
                model: mujoco.MjModel,
                data:  mujoco.MjData,
                scan_phase: float,
                stamp,
                node) -> None:
        """Non-blocking: copy sim state (~0.2 ms) and wake the ray thread."""
        with self._lock:
            mujoco.mj_copyData(self._snap, model, data)
            self._scan_phase = scan_phase
            self._stamp      = stamp
            self._node       = node
        self._trigger.set()

    def stop(self) -> None:
        self._stop = True
        self._trigger.set()
        self._thread.join(timeout=5.0)

    # ── Lidar thread loop ────────────────────────────────────────────────────
    def _loop(self) -> None:
        while not self._stop:
            if not self._trigger.wait(timeout=0.5):
                continue
            self._trigger.clear()

            with self._lock:
                stamp      = self._stamp
                node       = self._node
                scan_phase = self._scan_phase
            if stamp is None or node is None:
                continue

            # 1. Generate ray pattern for this scan phase
            ray_theta, ray_phi, ray_ring, ray_time = generate_mid360_pattern(scan_phase)

            # 2. Raycast on snapshot — safe, read-only
            hit_points, hit_dist, hit_ring, hit_time = trace_rays(
                self._model, self._snap, node.lidar_site_id,
                ray_theta, ray_phi, ray_ring, ray_time,
                CUTOFF_DIST,
                self._pnt_buf, self._vec_buf_2d, self._vec_buf,
                self._dist_buf, self._geomid_buf, self._geom_group,
                self._x_local, self._y_local, self._z_local,
            )

            # 3. Publish point cloud
            if hit_points.shape[0] > 0:
                msg = build_pointcloud2(
                    hit_points, hit_dist, hit_ring, hit_time, LIDAR_FRAME, stamp)
                if msg is not None:
                    node.cloud_pub.publish(msg)

            # 4. Publish TF: lidar_parent_frame → livox_frame
            tf = TransformStamped()
            tf.header.stamp    = stamp
            tf.header.frame_id = node.lidar_parent_frame
            tf.child_frame_id  = LIDAR_FRAME

            lidar_pos_w = self._snap.site_xpos[node.lidar_site_id]
            lidar_rot_w = self._snap.site_xmat[node.lidar_site_id].reshape(3, 3)

            if node.torso_body_id is not None:
                torso_pos_w = self._snap.xpos[node.torso_body_id]
                torso_rot_w = self._snap.xmat[node.torso_body_id].reshape(3, 3)
                rel_pos     = torso_rot_w.T @ (lidar_pos_w - torso_pos_w)
                rel_rot     = torso_rot_w.T @ lidar_rot_w
                lidar_pos   = rel_pos
                quat        = Rotation.from_matrix(rel_rot).as_quat()
            else:
                lidar_pos = lidar_pos_w
                quat      = Rotation.from_matrix(lidar_rot_w).as_quat()

            tf.transform.translation.x = float(lidar_pos[0])
            tf.transform.translation.y = float(lidar_pos[1])
            tf.transform.translation.z = float(lidar_pos[2])
            tf.transform.rotation.x    = float(quat[0])
            tf.transform.rotation.y    = float(quat[1])
            tf.transform.rotation.z    = float(quat[2])
            tf.transform.rotation.w    = float(quat[3])

            node.tf_broadcaster.sendTransform(tf)


# ═══════════════════════════════════════════════════════════════════════════════
#  UNIFIED NODE
# ═══════════════════════════════════════════════════════════════════════════════
class G1CompleteSimulation(Node):
    def __init__(self, model, data):
        super().__init__("g1_complete_simulation")
        self.model = model
        self.data  = data

        self.tf_broadcaster = TransformBroadcaster(self)

        # Thread-safe command buffer: ROS2 spin thread writes here,
        # main physics thread applies atomically before each step batch.
        self.ctrl_lock    = threading.Lock()
        self.pending_ctrl: Dict[int, float] = {}

        self._setup_lowlevel()
        self._setup_camera()
        self._setup_lidar()

        self.get_logger().info(
            "=== G1 Complete Simulation Ready ===\n"
            "  Low-level (29+14 DOF): /lowstate, /lowcmd, hands @ {:.0f} Hz\n"
            "  Camera (RGB+Depth): /camera/camera/... @ {:.0f} Hz\n"
            "  LIDAR (Mid360): {} @ {:.0f} Hz".format(
                STATE_HZ, CAMERA_HZ, TOPIC_CLOUD, LIDAR_HZ
            )
        )

    # ── Low-level setup ──────────────────────────────────────────────────────
    def _setup_lowlevel(self):
        model = self.model
        self.body_qpos, self.body_qvel, self.body_act, _ = build_joint_map(model, BODY_JOINT_ORDER)
        self.lh_qpos, self.lh_qvel, self.lh_act, _ = build_joint_map(model, LEFT_HAND_JOINT_ORDER)
        self.rh_qpos, self.rh_qvel, self.rh_act, _ = build_joint_map(model, RIGHT_HAND_JOINT_ORDER)

        self.last_body_dq = np.zeros(len(BODY_JOINT_ORDER), dtype=np.float64)
        self.last_lh_dq   = np.zeros(len(LEFT_HAND_JOINT_ORDER), dtype=np.float64)
        self.last_rh_dq   = np.zeros(len(RIGHT_HAND_JOINT_ORDER), dtype=np.float64)
        self.last_time     = self.get_clock().now()

        # IMU
        self.imu_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu_in_torso")
        try:
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu-torso-angular-velocity")
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu-torso-linear-acceleration")
            self.gyro_adr = model.sensor_adr[gid]
            self.acc_adr  = model.sensor_adr[aid]
        except Exception:
            self.gyro_adr = self.acc_adr = None

        # Publishers
        self.lowstate_pub = self.create_publisher(LowState,  "/lowstate",            10)
        self.lh_state_pub = self.create_publisher(HandState, "/lf/dex3/left/state",  10)
        self.rh_state_pub = self.create_publisher(HandState, "/lf/dex3/right/state", 10)

        # Subscribers
        self.create_subscription(LowCmd,  "/lowcmd",         self._on_lowcmd, 10)
        self.create_subscription(HandCmd, "/dex3/left/cmd",  self._on_lh_cmd, 10)
        self.create_subscription(HandCmd, "/dex3/right/cmd", self._on_rh_cmd, 10)

        self.tick = 0
        # 250 Hz via ROS 2 timer (higher than viewer loop)
        self.create_timer(1.0 / STATE_HZ, self._publish_lowlevel)

    # ── Camera setup ─────────────────────────────────────────────────────────
    def _setup_camera(self):
        self.rgb_pub        = self.create_publisher(Image,      "/camera/camera/color/image_raw",    10)
        self.rgb_info_pub   = self.create_publisher(CameraInfo, "/camera/camera/color/camera_info",  10)
        self.depth_pub      = self.create_publisher(Image,      "/camera/camera/depth/image_rect_raw",    10)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/camera/camera/depth/camera_info",  10)
        # Renderers live inside the camera thread (EGL context is thread-local)
        self._cam_thread = _CameraThread(self.model)
        self.get_logger().info("[Camera] Async render thread ready (EGL)")

    # ── LIDAR setup ──────────────────────────────────────────────────────────
    def _setup_lidar(self):
        self.cloud_pub = self.create_publisher(PointCloud2, TOPIC_CLOUD, 10)
        self.lidar_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, LIDAR_SITE)
        if self.lidar_site_id < 0:
            self.get_logger().error(f"LIDAR site '{LIDAR_SITE}' not found in model!")

        # TF parent frame: torso_link (relative pose) or world (absolute)
        self.torso_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, LIDAR_PARENT)
        if self.torso_body_id < 0:
            self.get_logger().warn(f"Body '{LIDAR_PARENT}' not found, using 'world' as TF parent")
            self.torso_body_id = None
            self.lidar_parent_frame = "world"
        else:
            self.lidar_parent_frame = LIDAR_PARENT
            self.get_logger().info(f"LIDAR TF parent: {self.lidar_parent_frame} -> {LIDAR_FRAME}")

        # Compute max-ray count for pre-allocated buffers in the worker thread
        ray_theta, _, _, _ = generate_mid360_pattern(0.0)
        max_n_rays = len(ray_theta)
        self.scan_phase = 0.0

        # Raycasting runs in background thread — all buffers are thread-local there
        self._lidar_thread = _LidarThread(self.model, max_n_rays)
        self.get_logger().info("[LIDAR] Async raycasting thread ready")

    # ── Low-level command callbacks (→ pending_ctrl, applied before physics) ────
    def _on_lowcmd(self, msg: LowCmd):
        with self.ctrl_lock:
            n = min(len(msg.motor_cmd), len(self.body_act))
            for i in range(n):
                act_id = self.body_act[i]
                if act_id is not None:
                    self.pending_ctrl[act_id] = safe_float(msg.motor_cmd[i].q)

    def _on_lh_cmd(self, msg: HandCmd):
        with self.ctrl_lock:
            n = min(len(msg.motor_cmd), len(self.lh_act))
            for i in range(n):
                act_id = self.lh_act[i]
                if act_id is not None:
                    self.pending_ctrl[act_id] = safe_float(msg.motor_cmd[i].q)

    def _on_rh_cmd(self, msg: HandCmd):
        with self.ctrl_lock:
            n = min(len(msg.motor_cmd), len(self.rh_act))
            for i in range(n):
                act_id = self.rh_act[i]
                if act_id is not None:
                    self.pending_ctrl[act_id] = safe_float(msg.motor_cmd[i].q)

    # ── Low-level publish (timer @ 250 Hz, runs in ROS 2 spin thread) ───────
    def _publish_lowlevel(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1.0 / STATE_HZ

        # IMU
        imu_msg = IMUState()
        if self.imu_site_id >= 0:
            xmat = self.data.site_xmat[self.imu_site_id].reshape(3, 3)
            q_xyzw = Rotation.from_matrix(xmat).as_quat()
            imu_msg.quaternion = [safe_float(q_xyzw[3]), safe_float(q_xyzw[0]),
                                  safe_float(q_xyzw[1]), safe_float(q_xyzw[2])]
            rpy = Rotation.from_matrix(xmat).as_euler("xyz", degrees=False)
            imu_msg.rpy = [safe_float(v) for v in rpy]
        if self.gyro_adr is not None:
            imu_msg.gyroscope     = [safe_float(self.data.sensordata[self.gyro_adr + k]) for k in range(3)]
            imu_msg.accelerometer = [safe_float(self.data.sensordata[self.acc_adr + k])  for k in range(3)]
        else:
            imu_msg.gyroscope = imu_msg.accelerometer = [0.0, 0.0, 0.0]
        imu_msg.temperature = 25

        # /lowstate (body 29 DOF)
        ls = LowState()
        ls.version = [1, 0]; ls.tick = self.tick; self.tick += 1
        ls.imu_state = imu_msg
        body_states = []
        for i, (qp, qv, ac) in enumerate(zip(self.body_qpos, self.body_qvel, self.body_act)):
            st = read_motor_state(self.data, self.model, qp, qv, ac, self.last_body_dq[i], dt)
            if qv is not None:
                self.last_body_dq[i] = safe_float(self.data.qvel[qv])
            body_states.append(st)
        while len(body_states) < 35:
            body_states.append(MotorState())
        ls.motor_state = body_states
        ls.wireless_remote = [0] * 40
        ls.reserve = [0, 0, 0, 0]
        ls.crc = 0
        self.lowstate_pub.publish(ls)

        # Left hand (7 DOF)
        lhs = HandState()
        lh_states = []
        for i, (qp, qv, ac) in enumerate(zip(self.lh_qpos, self.lh_qvel, self.lh_act)):
            st = read_motor_state(self.data, self.model, qp, qv, ac, self.last_lh_dq[i], dt)
            if qv is not None:
                self.last_lh_dq[i] = safe_float(self.data.qvel[qv])
            lh_states.append(st)
        lhs.motor_state = lh_states
        self.lh_state_pub.publish(lhs)

        # Right hand (7 DOF)
        rhs = HandState()
        rh_states = []
        for i, (qp, qv, ac) in enumerate(zip(self.rh_qpos, self.rh_qvel, self.rh_act)):
            st = read_motor_state(self.data, self.model, qp, qv, ac, self.last_rh_dq[i], dt)
            if qv is not None:
                self.last_rh_dq[i] = safe_float(self.data.qvel[qv])
            rh_states.append(st)
        rhs.motor_state = rh_states
        self.rh_state_pub.publish(rhs)

        self.last_time = now

    # ── Camera publish (non-blocking: delegates to _CameraThread) ────────────
    def publish_cameras(self):
        # Snapshots sim state (~0.2 ms) and wakes the render thread.
        # Actual GPU render + ROS publish happen in _CameraThread asynchronously.
        self._cam_thread.request(self.model, self.data,
                                 self.get_clock().now().to_msg(), self)

    # ── LIDAR publish (non-blocking: delegates to _LidarThread) ─────────────
    def publish_lidar(self):
        if self.lidar_site_id < 0:
            return
        # Snapshots sim state (~0.2 ms) and wakes the raycasting thread.
        # Actual mj_multiRay + TF publish happen in _LidarThread asynchronously.
        self._lidar_thread.request(
            self.model, self.data,
            self.scan_phase,
            self.get_clock().now().to_msg(),
            self,
        )
        self.scan_phase += 0.35   # advance phase for next scan frame

    # ── Cleanup ──────────────────────────────────────────────────────────────
    def cleanup(self):
        self._cam_thread.stop()
        self._lidar_thread.stop()


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    scene_xml = os.path.join(os.path.dirname(os.path.abspath(__file__)), SCENE_XML)
    if not os.path.exists(scene_xml):
        print(f"[ERROR] Scene not found: {scene_xml}", flush=True)
        sys.exit(1)

    print(f"[SIM] Loading scene: {scene_xml}", flush=True)
    model = mujoco.MjModel.from_xml_path(scene_xml)
    data  = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    rclpy.init()
    node = G1CompleteSimulation(model, data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    viewer = mujoco.viewer.launch_passive(model, data)

    physics_dt     = model.opt.timestep
    camera_interval = max(1, round(VIEWER_HZ / CAMERA_HZ))
    lidar_interval  = max(1, round(VIEWER_HZ / LIDAR_HZ))
    camera_offset   = 0
    lidar_offset    = lidar_interval // 2
    frame_count     = 0

    # Adaptive: track actual wall time of last frame to compute steps needed.
    # viewer.sync() over X11/Docker takes ~40-50 ms irrespective of VIEWER_HZ.
    # With fixed 8 steps (=16 ms sim) per 50 ms real frame, sim runs at 0.32x.
    # Measuring last_wall_dt and doing int(last_wall_dt / physics_dt) steps
    # keeps the simulation at exactly 1.0x real-time regardless of rendering cost.
    last_wall_dt = 1.0 / VIEWER_HZ   # first-frame seed

    perf_t0     = time.perf_counter()
    perf_steps  = 0
    perf_syncs  = 0
    # Per-section timing accumulators
    t_ctrl_acc = t_step_acc = t_sync_acc = t_pub_acc = 0.0

    print(
        f"[SIM] Running simulation (adaptive real-time loop):\n"
        f"  Physics      : auto steps per frame (tracks wall clock, target 1.0x RT)\n"
        f"  Viewer       : {VIEWER_HZ:.0f} Hz target (X11 paces the loop naturally)\n"
        f"  Motor state  : {STATE_HZ:.0f} Hz (ROS 2 timer)\n"
        f"  Camera       : {CAMERA_HZ:.0f} Hz (every {camera_interval} frames)\n"
        f"  LIDAR        : {LIDAR_HZ:.0f} Hz (every {lidar_interval} frames, staggered)",
        flush=True,
    )

    try:
        while viewer.is_running():
            t_frame = time.perf_counter()

            # 1. Apply pending motor commands atomically before physics
            _t0 = time.perf_counter()
            with node.ctrl_lock:
                if node.pending_ctrl:
                    for act_id, val in node.pending_ctrl.items():
                        data.ctrl[act_id] = val
                    node.pending_ctrl.clear()
            _t1 = time.perf_counter()

            # 2. Physics: run exactly as many steps as real time elapsed
            #    If viewer.sync took 50 ms, we run 50/2 = 25 steps -> stays 1:1 RT
            #    If camera blocked 200 ms, next frame runs 100 steps -> recovers
            steps = max(1, int(last_wall_dt / physics_dt))
            for _ in range(steps):
                mujoco.mj_step(model, data)
            perf_steps += steps
            _t2 = time.perf_counter()

            # 3. Viewer sync (X11 latency is the natural loop pacer)
            viewer.sync()
            perf_syncs += 1
            frame_count += 1
            _t3 = time.perf_counter()

            # 4. Camera (every N frames)
            if (frame_count - camera_offset) % camera_interval == 0:
                node.publish_cameras()

            # 5. LIDAR (every N frames, staggered vs camera)
            if (frame_count - lidar_offset) % lidar_interval == 0:
                node.publish_lidar()
            _t4 = time.perf_counter()

            # Accumulate per-section times
            t_ctrl_acc += _t1 - _t0
            t_step_acc += _t2 - _t1
            t_sync_acc += _t3 - _t2
            t_pub_acc  += _t4 - _t3

            # 6. Measure actual wall time: used by next iteration for step count
            last_wall_dt = time.perf_counter() - t_frame

            # 7. Performance report every 10 s
            perf_elapsed = time.perf_counter() - perf_t0
            if perf_elapsed >= 10.0:
                rt  = (perf_steps * physics_dt) / perf_elapsed
                fps = perf_syncs / perf_elapsed
                n   = max(1, perf_syncs)
                print(
                    f"[PERF] {fps:.1f} FPS | {perf_steps/perf_elapsed:.0f} Hz physics "
                    f"| RT {rt:.2f}x | last_frame {last_wall_dt*1000:.0f} ms\n"
                    f"       avg/frame — ctrl:{t_ctrl_acc/n*1000:.1f} ms  "
                    f"step:{t_step_acc/n*1000:.1f} ms  "
                    f"sync:{t_sync_acc/n*1000:.1f} ms  "
                    f"pub:{t_pub_acc/n*1000:.1f} ms",
                    flush=True,
                )
                perf_t0    = time.perf_counter()
                perf_steps = 0
                perf_syncs = 0
                t_ctrl_acc = t_step_acc = t_sync_acc = t_pub_acc = 0.0

    finally:
        viewer.close()
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        print("[SIM] Exited successfully.", flush=True)


if __name__ == "__main__":
    main()
