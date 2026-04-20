# MuJoCo + ROS 2 Humble — Unitree G1 Simulation

Unitree G1 simulation in MuJoCo with D435i camera + Livox Mid360 LIDAR, publishing to ROS 2 topics (Humble), all within Docker.


![Mujoco Simulation Ros2](assets/Simulacion_completa.gif)


## Key Features

  - **Single MuJoCo instance** — 1 physics loop, camera + LIDAR + lowlevel state in harmony
  - **Async rendering** — GPU camera rendering in background thread, doesn't block viewer
  - **Async raycasting** — LIDAR raycasting in background CPU thread, doesn't block viewer
  - **60 FPS viewer** — Smooth passive viewer over X11, full physical accuracy at real-time speed
  - **Real-time physics** — ~0.98× real-time, adaptive step count (no fixed stagger)
  - **43 DOF control** — 29 body + 7 left hand + 7 right hand, 250 Hz state publishing
  - **TF broadcasting** — `torso_link → livox_frame` automatic transformation

---

## Quick Start — Mujoco Simulation

### 1. Host setup (one-time)

```bash
xhost +local:root
```

### 2. Docker: build and launch

```bash
cd docker

docker compose build
docker compose up -d

docker exec -it mujoco_ros2_humble bash
```

### 3. Run simulation

```bash
source /opt/ros/humble/setup.bash
python3 g1_ros2_mujoco_complete_simulation.py
```

**Expected output:**
```
[SIM] Loading scene: /workspace/scene_uni.xml
[INFO] [1776110201.681...] [g1_complete_simulation]: [Camera] Async render thread ready (EGL)
[INFO] [1776110201.682...] [g1_complete_simulation]: LIDAR TF parent: torso_link -> livox_frame
[INFO] [1776110201.683...] [g1_complete_simulation]: === G1 Complete Simulation Ready ===
  Low-level (29+14 DOF): /lowstate, /lowcmd, hands @ 250 Hz
  Camera (RGB+Depth): /camera/camera/... @ 10 Hz
  LIDAR (Mid360): /utlidar/cloud_livox_mid360 @ 10 Hz
[SIM] Running simulation (adaptive real-time loop):
  Physics      : auto steps per frame (tracks wall clock, target 1.0x RT)
  Viewer       : 60 Hz target (X11 paces the loop naturally)
  ...
[PERF] 60.0 FPS | 485 Hz physics | RT 0.97x | last_frame 15 ms
```

The viewer runs smoothly at **~60 FPS**, camera publishes every 6 frames (~10 Hz), LIDAR every 6 frames (~10 Hz).

---

## Architecture & Performance


One MuJoCo instance, one physics loop, three concurrent pub/sub streams:

```
┌─────────────────────────────────────────────────────────┐
│ Main Thread: Physics Loop                               │
│  - 1x mujoco.MjData (live, updates every mj_step)       │
│  - viewer.sync() @ ~60 Hz (X11 pacesthe loop)           │
│  - Adaptive step count (tracks wall clock → 0.98x RT)   │
└────────────────────────────────────────────────────────┘
        │ snapshot every 6 frames           │ snapshot every 6 frames
        ↓                                   ↓
┌──────────────────┐              ┌──────────────────┐
│ Camera Thread    │              │ LIDAR Thread     │
│ (EGL GPU)        │              │ (CPU raycast)    │
│                  │              │                  │
│ • mj_copyData    │              │ • mj_copyData    │
│   (~0.2 ms)      │              │   (~0.2 ms)      │
│ • GPU render     │              │ • mj_multiRay    │
│   (~5-20 ms)     │              │   (~100 ms CPU)  │
│ • ROS publish    │              │ • TF broadcast   │
│                  │              │   + ROS publish  │
└──────────────────┘              └──────────────────┘
```

- Main thread only does `mj_copyData` (0.2 ms) + trigger threads
- Camera GPU rendering happens in parallel
- LIDAR CPU raycasting happens in parallel
- Both publish asynchronously without blocking the viewer loop

### Performance: Actual Results

**With 320×240 camera (recommended):**
- FPS: **~60.0** (viewer runs at 60 Hz, X11 vsync)
- Physics: **~500 Hz** (0.98× real-time, adaptive)
- Last frame: **15 ms** (typical), ~20 ms (camera frames)
- CPU: ~20%, GPU: ~25% (viewer is the limiting factor)

**Timing breakdown (avg/frame @ 320×240):**
```
ctrl:     0.1 ms  (motor commands apply)
step:     4.2 ms  (8 physics steps × 0.5 ms each)
sync:    13.0 ms  (viewer.sync over X11)
pub:      0.2 ms  (snapshot + thread wake)
```

### Performance: Camera Resolution Trade-offs

The performance bottleneck is **PCIe bandwidth** during GPU readback (`glReadPixels`), not GPU compute.

| Resolution | Readback Size | Est. FPS |
|---|---|---|
| 320×240 (current) | 0.23 MB | **~60 FPS** ✓ |
| 640×480 | 0.92 MB | ~45 FPS |
| 1280×720 | 2.76 MB | ~25 FPS |
| 1920×1080 | 6.22 MB | **~9 FPS** ✗ |

**GPU Utilization Paradox:**
- At 1920×1080: GPU% ~28%, FPS ~9
- The GPU is NOT saturated; PCIe bus is saturated
- `glReadPixels` is **synchronous + blocking** — forces CPU ↔ GPU handshake/synchronization
- Driver serializes EGL (camera) + GLX (viewer) contexts on the same GPU scheduler

**Practical limit: ~60 FPS maintained with 320×240 @ 10 Hz camera + 10 Hz LIDAR**

---

## Configuration & Limits

### CONFIG section (top of g1_ros2_mujoco_complete_simulation.py)

```python
# Rates
VIEWER_HZ   = 60.0
STATE_HZ    = 250.0
CAMERA_HZ   = 10.0
LIDAR_HZ    = 10.0

# Camera resolution
RGB_W, RGB_H   = 320, 240       # ← Recommended
DEPTH_W, DEPTH_H = 320, 240

# LIDAR
N_H = 313      # Horizontal rays per ring
N_V = 64       # Vertical rings (channels)
CUTOFF_DIST = 40.0
```

### How to Maintain 60 FPS

**✓ DO:**
- Keep camera at **320×240** (or lower)
- Keep LIDAR @ **10 Hz** (the raycasting is CPU-bound ~100 ms)
- Use **adaptive loop** (enabled by default) — automatically adjusts physics steps to real-time speed
- Enable **MUJOCO_GL=egl** (set before `import mujoco`) — forces hardware GPU rendering

**✗ DON'T:**
- Increase camera resolution to 1920×1080 (expect 9 FPS, PCIe bottleneck)
- Increase CAMERA_HZ / LIDAR_HZ beyond 10 Hz (they overlap threads, threads queue up)
- Disable adaptive loop (will cause physics to lag behind)
- Disable EGL (will fall back to osmesa CPU rendering → even slower)

### If You Need High-Res Camera Data

**Option 1: External recorder** (recommended)
- Keep simulation at 320×240 @ 10 Hz
- Create a separate ROS 2 node that subscribes to `/camera/camera/color/image_raw`
- Record at 1920×1080 to disk (doesn't affect simulation)
- Example: use `ros2 bag record` or rosbag2

**Option 2: Reduce camera pub frequency**
```python
CAMERA_HZ = 2.0   # Every 0.5 sec instead of 0.1 sec
```
Watch: FPS will return to ~60, but you only get 2 images/sec.

**Option 3: Separate process**
Run a dedicated high-res camera simulation in a second Docker container, subscribing to `/lowstate` to track the robot.

---

## Architectural Details

### Async Camera Thread (`_CameraThread`)

```python
class _CameraThread:
    """GPU EGL rendering in background."""
    
    def request(model, data, stamp, node):
        # Called from main thread
        mj_copyData(snap, model, data)  # 0.2 ms
        trigger.set()                    # Wake background thread
        return immediately               # (0.2 ms total)
    
    def _loop(self):
        # Runs in background
        while not stop:
            wait for trigger
            rgb_r.update_scene(snap)         # GPU texture upload
            rgb_r.render() → glReadPixels()  # GPU readback (slow)
            node.rgb_pub.publish(bgr)        # ROS publish
```

- **EGL context created in thread** — EGL is thread-local, GLX is not
- **update_scene() fast** — copies from snap to GPU texture (1 ms)
- **render() slow** — GPU execution + `glReadPixels` (~5–20 ms for 320×240)
- **Main thread never waits** — only does snapshot + trigger

### Async LIDAR Thread (`_LidarThread`)

```python
class _LidarThread:
    """CPU raycasting in background."""
    
    def request(model, data, scan_phase, stamp, node):
        # Called from main thread
        mj_copyData(snap, model, data)  # 0.2 ms
        scan_phase handled by caller
        trigger.set()
        return immediately               # (0.2 ms total)
    
    def _loop(self):
        # Runs in background
        while not stop:
            wait for trigger
            generate_mid360_pattern(phase)
            trace_rays(snap) → mj_multiRay()  # CPU (slow, ~100 ms)
            node.cloud_pub.publish(pcl2)
            node.tf_broadcaster.sendTransform(tf)
```

- **mj_multiRay is read-only** — uses snapshot, safe during live physics
- **Pre-allocated buffers thread-local** — no contention with main thread
- **Main thread never waits** — only does snapshot + trigger

### Physics Adaptive Loop

```python
# Measure actual wall-clock time of last frame
last_wall_dt = time.perf_counter() - t_frame_start

# Compute steps to keep simulation real-time
steps = max(1, int(last_wall_dt / physics_dt))
# If last frame took 100 ms → run 50 steps (100ms / 2ms)
# If last frame took 20 ms → run 10 steps (20ms / 2ms)

for _ in range(steps):
    mujoco.mj_step(model, data)
```

**Result:** Simulation automatically matches real-world elapsed time.
- Typical: 15 ms walls = 8 steps = 16 ms physics (0.94x RT)
- Camera frame: 110 ms wall = 55 steps = 110 ms physics (1.0x RT) ✓
- LIDAR frame: 100 ms wall = 50 steps = 100 ms physics (1.0x RT) ✓

---

## ROS 2 Topics Published

### Lowlevel (43 DOF control @ 250 Hz)

| Topic | Type | Rate |
|-------|------|------|
| `/lowstate` | `LowState` | 250 Hz |
| `/lowcmd` | `LowCmd` | subscriber (real-time) |
| `/lf/dex3/left/state` | `HandState` | 250 Hz |
| `/dex3/left/cmd` | `HandCmd` | subscriber |
| `/lf/dex3/right/state` | `HandState` | 250 Hz |
| `/dex3/right/cmd` | `HandCmd` | subscriber |

### Camera (D435i @ 10 Hz)

| Topic | Type | Rate |
|-------|------|------|
| `/camera/camera/color/image_raw` | Image (BGR8, 320×240) | 10 Hz |
| `/camera/camera/color/camera_info` | CameraInfo | 10 Hz |
| `/camera/camera/depth/image_rect_raw` | Image (32FC1, 320×240) | 10 Hz |
| `/camera/camera/depth/camera_info` | CameraInfo | 10 Hz |

### LIDAR (Livox Mid360 @ 10 Hz)

| Topic | Type | Rate | Points |
|-------|------|------|--------|
| `/utlidar/cloud_livox_mid360` | PointCloud2 | 10 Hz | ~40960 |
| (TF: `torso_link → livox_frame`) | TransformStamped | 10 Hz | — |

---

## Motor Control (43 DOF)

### Body (29 DOF)

Indices 0–28 in `/lowcmd`:
```
 0- 5  Left leg  (hip_pitch/roll/yaw, knee, ankle_pitch/roll)
 6-11  Right leg (hip_pitch/roll/yaw, knee, ankle_pitch/roll)
12-14  Waist    (yaw, roll, pitch)
15-21  Left arm (shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw)
22-28  Right arm
```

### Hands (14 DOF)

Left hand (7 DOF) + Right hand (7 DOF) via `/dex3/left/cmd`, `/dex3/right/cmd`:
```
0  thumb_0    [-1.047, +1.047]
1  thumb_1    [-1.047, +0.724]
2  thumb_2    [-1.745,  0.000]
3  index_0    [ 0.000, +1.571]
4  index_1    [ 0.000, +1.745]
5  middle_0   [ 0.000, +1.571]
6  middle_1   [ 0.000, +1.745]
```

### Send Commands

(Be careful, positions might exceed robot and motors physical capabilities in real life)

```bash
docker exec -it mujoco_ros2_humble bash
python3 send_full_body_cmd.py stand      # All motors to 0
python3 send_full_body_cmd.py grasp      # Arms down, hands closing
python3 send_full_body_cmd.py reach      # Arms up, hands open
python3 send_full_body_cmd.py relax      # Relaxed posture
python3 send_full_body_cmd.py custom 0 0 0 ... 0  # 43 values
```
**Custom 43 values:**
```bash
python3 send_full_body_cmd.py custom 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 --duration 3.0
```

**Options:** `--duration 2.0` (seconds), `--hz 50.0` (publish rate)

---

## Visualizing in RViz2

### Monitor topics

```bash
docker exec -it mujoco_ros2_humble bash
ros2 daemon stop >/dev/null 2>&1
ros2 topic list
```

### View LIDAR cloud

```bash
docker exec -it mujoco_ros2_humble bash
rviz2 &
```

In RViz:
1. Set **Fixed Frame** to `world` (or `torso_link`)
2. **Add Display** → PointCloud2
3. **Topic**: `/utlidar/cloud_livox_mid360`
4. **Queue Length**: 10–50

You should see a dense 3D point cloud with realistic LIDAR pattern.

### View camera images

```bash
docker exec -it mujoco_ros2_humble bash
rqt_image_view &
```

Select `/camera/camera/color/image_raw` to view RGB (or `/camera/camera/depth/image_rect_raw` for depth).

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "X connection broken" | `docker compose kill && docker compose up -d` |
| "rclpy.type_hash.TypeHash" error | `ros2 daemon stop && ros2 daemon start` |
| Topics don't appear | Check `ps aux \| grep python3` to ensure script is running |
| Viewer black/frozen | Increase `VIEWER_HZ`, ensure GPU drivers are new |
| FPS drops to 10 | You likely increased camera resolution to 1920×1080 (PCIe bottleneck) |

---

## Technical Notes

### Why EGL + GPU Rendering?

Without `MUJOCO_GL=egl` (set before `import mujoco`):
- `mujoco.Renderer` falls back to **osmesa** (CPU software rendering)
- CPU bound: RGB render alone takes 200+ ms @ 320×240
- Simulation crawls to ~5 FPS

With `MUJOCO_GL=egl`:
- `mujoco.Renderer` uses **NVIDIA EGL** (GPU hardware rendering)
- GPU renders in 5–20 ms
- Async thread prevents blocking the viewer

### Physics Timestep

Default: `model.opt.timestep = 0.002` (500 Hz maximum physics)

With adaptive loop:
- Typical frame: 15 ms real time → 8 steps → 16 ms sim (0.94x RT)
- Camera frame: 20 ms real time → 10 steps → 20 ms sim (1.0x RT)
- LIDAR frame: 100 ms real time → 50 steps → 100 ms sim (1.0x RT)

### Thread Safety

- `CTRL_LOCK` protects `pending_ctrl` (motor commands from ROS thread)
- `_snap` (camera + LIDAR buffers) are private copies, never shared
- `MjData.x` (main state) is only written in main thread (viewer loop), read-only from snapshot threads

---

## Files

| File | Purpose |
|------|---------|
| **g1_ros2_mujoco_complete_simulation.py** | **RECOMMENDED** — Unified 60 FPS simulation (camera + LIDAR + lowlevel) |
| g1_ros2_lowlevel_bridge.py | Legacy — Lowlevel bridge alone (~60 FPS) |
| g1_ros2_camera_bridge.py | Legacy — Camera bridge alone (~20 FPS, blocks) |
| g1_ros2_lidar_bridge.py | Legacy — LIDAR bridge alone (~30 FPS, blocks) |
| send_full_body_cmd.py | Command sender (43 DOF postures) |
| check_finger_values.py | Diagnostic: print hand joint values |
| scene_uni.xml | Scene: lab environment + G1 |
| g1_with_hands.xml | Robot model: 43 DOF G1 + D435i cameras |
| docker_ros2_humble/Dockerfile | Docker image (ROS 2 Humble + MuJoCo 3.6.0) |
| docker_ros2_humble/docker-compose.yml | Docker Compose (GPU + X11 + networking) |

---

## Repository Structure

```
unitree_g1/
├── g1_ros2_mujoco_complete_simulation.py   ← USE THIS (unified, 60 FPS)
├── send_full_body_cmd.py
├── scene_uni.xml
├── g1_with_hands.xml
├── docker_ros2_humble/
│   ├── Dockerfile
│   └── docker-compose.yml
├── README_Fer.md                           (this file)
└── assets/                                 (D435i, livox models)
```

---

## Performance Summary

| Metric | Value |
|--------|-------|
| **Physics** | 0.98× real-time (adaptive) |
| **Viewer FPS** | ~60 FPS (X11 vsync) |
| **Camera resolution** | 320×240 @ 10 Hz (async GPU thread) |
| **LIDAR points** | ~40960 @ 10 Hz (async CPU thread) |
| **Lowlevel state** | 250 Hz (ROS 2 timer) |
| **CPU usage** | ~20% (main loop) |
| **GPU usage** | ~25% (viewer GLX) |
| **RAM** | ~2 GB (Docker) |

**With these settings, the simulation runs smoothly without optimization hints or parameter tuning.** The architecture ensures all three subsystems (physics, camera, LIDAR) proceed in parallel without contention.

---
### TF Hierarchy

The LIDAR frame integrates into your URDF chain:

```
world → [your URDF transforms] → torso_link → livox_frame
```

The bridge automatically:
1. Finds `torso_link` in the MuJoCo model
2. Computes relative pose: `lidar_pos_relative = torso_rot^T @ (lidar_pos_world - torso_pos_world)`
3. Broadcasts `torso_link → livox_frame` with relative transformation
4. Falls back to `world → livox_frame` if `torso_link` not found

This allows your URDF to transform `torso_link` to any frame (e.g., `odom`, `base_link`, `map`), and the LIDAR frame follows automatically.


### Architecture

Unitree G1 has **43 DOF total** = **29 body** + **7 left hand** + **7 right hand**

The bridge uses the **real Unitree topic architecture**:

| Topic | Type | Direction | DOF |
|-------|------|-----------|-----|
| `/lowstate` | `LowState` | Bridge → ROS | Body 29 (padded to 35) |
| `/lowcmd` | `LowCmd` | ROS → Bridge | Body 29 (35 slots) |
| `/lf/dex3/left/state` | `HandState` | Bridge → ROS | Left hand 7 |
| `/dex3/left/cmd` | `HandCmd` | ROS → Bridge | Left hand 7 |
| `/lf/dex3/right/state` | `HandState` | Bridge → ROS | Right hand 7 |
| `/dex3/right/cmd` | `HandCmd` | ROS → Bridge | Right hand 7 |



## Typical Workflow

```bash
# Terminal 1 - Start Docker once
cd docker_ros2_humble
docker compose build
docker compose up -d

# Terminal 2 - Lowlevel bridge (state + motor control + IMU)
docker exec -it mujoco_ros2_humble bash
python3 g1_ros2_lowlevel_bridge.py

# Terminal 3 - Camera bridge (optional)
docker exec -it mujoco_ros2_humble bash
python3 g1_ros2_camera_bridge.py

# Terminal 4 - Send commands
docker exec -it mujoco_ros2_humble bash
python3 send_full_body_cmd.py grasp

# Terminal 5 - Monitor topics
docker exec -it mujoco_ros2_humble bash
ros2 topic list
ros2 topic echo /lf/dex3/right/state --once

# When done:
docker compose down
```

## References

- MuJoCo 3.6.0 Docs: https://mujoco.readthedocs.io/
- ROS 2 Humble: https://docs.ros.org/en/humble/
- RealSense D435i Specs: https://www.intelrealsense.com/depth-camera-d435i/
- Unitree G1: https://www.unitreerobotics.com/



-----------------
#### Special Commands 

Give root permission and freedom of access to the already locked files:

``` bash
sudo chown -R $USER:$USER /home/fgarcia/Documents/test
```

