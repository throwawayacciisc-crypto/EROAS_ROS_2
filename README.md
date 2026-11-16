# EROAS on ROS 2 Jazzy / Gazebo Harmonic

A port of [AIRLabIISc/EROAS](https://github.com/AIRLabIISc/EROAS) (underwater
obstacle avoidance from forward-looking sonar images) from ROS 1 / Gazebo
Classic to ROS 2 Jazzy / Gazebo Harmonic. By default it runs the **original
EROAS algorithm** on the **DAVE multibeam sonar**, wired as in the ROS 1
`start_EROAS_demo.launch`:

```
DAVE sonar image ──► ros1_only_gap.py (SPD2C) ──► /rexrov2/cmd_vel_1 ──► ros1_velocity_cbf.py (ST-CBF) ──► /rexrov2/cmd_vel
        └────────► ros1_sonar_reconstruction.py (SCG) ──► /rexrov2/point_cloud ──┘
```

The `ros1_*` scripts in `navigator_auv/scripts/` are line-for-line ROS 2 ports
of the original `only_gap.py`, `velocity_cbf.py` and `sonar_reconstruction.py`;
every deviation is marked `ADAPTED` in the code.

## Quick start (Docker)

Needs Ubuntu with an NVIDIA GPU (GTX 10xx or newer). [SETUP.md](SETUP.md) covers
installing the driver, Docker and the NVIDIA Container Toolkit. Then:

```bash
git clone https://github.com/throwawayacciisc-crypto/EROAS_ROS_2.git
cd EROAS_ROS_2
./run_eroas.sh setup     # once: builds the Docker image and the workspace
./run_eroas.sh start     # simulation + Gazebo window + RViz, World A
./run_eroas.sh teleop    # optional: stop EROAS and drive by keyboard
./run_eroas.sh stop
```

`./run_eroas.sh help` lists all commands and options.

## Building without Docker

On Ubuntu 24.04 with ROS 2 Jazzy, Gazebo Harmonic (`ros-jazzy-ros-gz`) and the
CUDA toolkit (`nvcc`, cuFFT, cuBLAS; the full package list is in
`docker/Dockerfile`), clone this repo into a workspace's `src/` and build:

```bash
mkdir -p ~/eroas_ws/src && cd ~/eroas_ws/src
git clone https://github.com/throwawayacciisc-crypto/EROAS_ROS_2.git
cd ~/eroas_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/local_setup.bash
```

With Ubuntu's own `nvidia-cuda-toolkit` (CUDA 12.0), which does not support
GCC 13, add `--cmake-args -DCMAKE_CUDA_HOST_COMPILER=g++-12`. Without `nvcc`
the DAVE sonar package builds empty and only `sonar_source:=lidar` works.

## Running

Start the simulation with `gui:=false`, wait until the log shows
`Entity creation successful` (RexROV2 spawned, about 20 s), then open the
Gazebo window and RViz. A Gazebo window opened before the spawn never shows the
vehicle, so avoid the launch file's own GUI (`gui:=true`). `run_eroas.sh start`
does this sequence for you.

```bash
ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py gui:=false world_name:=world_a
gz sim -g --force-version 8          # after the vehicle has spawned
rviz2 -d tools/sonar.rviz            # sonar fan image
python3 tools/teleop.py              # manual driving + sonar tilt (stop the EROAS nodes first)
```

`tools/teleop.py` keys: `w/s` surge, `a/d` yaw, `q/e` sway, `r/f` heave,
`space` stop, `i/k` sonar tilt ±5°, `o` tilt 0°, `x` quit.

The fan image is also available without RViz:
`ros2 run rqt_image_view rqt_image_view /rexrov2/blueview_p900/sonar_image`.

### Launch arguments

| Argument | Default | Other values |
|---|---|---|
| `world_name` | `blue_blocks` | `world_a` (paper Figure 8), `world_a_figure5` (paper Figure 5 SPD2C path), `world_b` |
| `navigation` | `ros1` (original EROAS nodes) | `port` (this repo's reworked `only_gap.py` / `velocity_cbf.py`, see below) |
| `sonar_source` | `dave` (DAVE acoustic sonar, GPU) | `lidar` (synthetic sonar image from a gz GPU lidar; no CUDA) |
| `physics_engine` | `gz-physics-bullet-featherstone-plugin` | `gz-physics-dartsim-plugin` (fails with LCP NaN errors in World A) |
| `gui` | `true` | `false` (see above) |
| `waypoints` | per world | `"x,y,z;x,y,z;..."` |

### Checking it works

```bash
ros2 node list     # includes sonar_heading_node (SPD2C), obstacle_avoidance_node (ST-CBF),
                   # sonar_reconstruction, multibeam_sonar_node and thruster_ros_plugin_0..5
ros2 topic hz /rexrov2/blueview_p900/sonar_image_raw   # about 5 Hz on a GTX 1050 Ti
ros2 topic echo /rexrov2/cmd_vel                       # non-zero once the vehicle moves
```

In World A the vehicle searches for a gap, leaves the start area and heads for
waypoint 1 at (29, 97, −50).

## Harmonic port fixes (2026-10-30)

With these fixes the original logic leaves the start area by searching for
gaps, as it did in ROS 1, and reaches waypoint 1 in World A.

- **Sonar beam order** (`navigator_auv/scripts/ros1_sonar_common.py`): DAVE's
  beam 0 is the vehicle's left, but the EROAS code assumes beam 0 is the
  right. FLOAT32 frames are now mirrored across the beams. Before this, SPD2C
  turned toward obstacles and dithered at the spawn point.
- **Zero world gravity** (`obstacle_avoidance.world`): bullet-featherstone
  ignores per-link gravity off, so the vehicle sank about 0.02 m/s, left the
  ±5 m depth band and pushed `only_gap` into its 3-D mode.
- **Obstacle collisions** (`CubeSoft_{5,6,9,13}`): restores the original mesh
  collision shapes (convex decomposition, 16 hulls) in place of the bounding
  boxes, which left invisible walls inside the shapes' notches.
- **DAVE fan image** (`dave_multibeam_sonar/src/MultibeamSonarSensor.cc`):
  - The per-bin `cv::ellipse` drawing (~0.3 s a frame) is replaced with a
    pixel → (beam, bin) lookup table built once. The rate with RViz open went
    from 3.1 Hz to about 4.1 Hz.
  - Frames with no returns, e.g. with the sonar tilted up into open water,
    now draw black instead of a uniform bright fan.

## Repository layout

| Folder | Contents |
|---|---|
| `navigator_auv/` | EROAS navigation: `ros1_*` ports of the original nodes, and the reworked `port` stack |
| `dave_multibeam_sonar/` | DAVE multibeam sonar ported to Harmonic (CUDA); see its README |
| `rexrov2/` | RexROV2 description, control and the `start_EROAS_demo.launch.py` launch file |
| `uuv_simulator/` | UUV Simulator packages, worlds and obstacle models |
| `nps_uw_multibeam_sonar/` | NPS multibeam sonar (not used by the default demo) |
| `tf_transformations/` | ROS 2 `tf_transformations` |
| `tools/` | `teleop.py`, `sonar.rviz` |
| `docker/`, `run_eroas.sh`, `SETUP.md` | Docker environment and launcher |

## The `navigation:=port` stack

Before the original nodes were ported, this repo carried a reworked navigator
(`navigator_auv/scripts/only_gap.py`, `velocity_cbf.py`) that works from the
sonar point cloud, with contour-based navigation, waypoint hovering and
stuck-recovery (fully boxed-in recovery, closing-corridor detection, dead-end
heading exclusion). It is still available with `navigation:=port`. Its known
limitations:

1. In World A it can get trapped in a winding multi-pocket region around
   `y ≈ 60–68`.
2. The `known_bad_headings` dead-end memory has no time-based expiry or
   positional scoping, so a heading can stay excluded after the vehicle has
   moved on.
3. Its own sonar-image decoding (`use_raw_sonar`, off by default) does not
   have the beam-order fix above; the default point-cloud input is not
   affected.

Sonar rays for this stack can be shown in RViz with
`rviz2 -d navigator_auv/rviz/sonar_rays.rviz` (topic
`/rexrov2/sonar_ray_markers`, published by `sonar_ray_markers.py`).

## Troubleshooting

**`ros2: command not found` / package not found.** Source both environments
in every terminal: `source /opt/ros/jazzy/setup.bash` and
`source ~/eroas_ws/install/local_setup.bash`. Check with
`ros2 pkg prefix rexrov2_gazebo`.

**No RexROV2 in the Gazebo window.** The window was opened before the spawn;
see [Running](#running).

**`Failed to initialize NVML: Unknown Error` in Docker.** The container lost
the GPU: `docker restart eroas`, then start again.

**Sonar slow or dropping frames.** The DAVE sonar is GPU heavy; close other
GPU programs. RViz uses Reliable QoS for the ~0.9 MB fan images, since
best-effort drops fragments.

**Changes do not appear.** Rebuild (`colcon build`, or `./run_eroas.sh build`)
and source `install/local_setup.bash` again. Python scripts are installed as
copies, so they need a rebuild too.

More in [SETUP.md](SETUP.md#troubleshooting).

## References

- EROAS (original ROS 1): https://github.com/AIRLabIISc/EROAS
- Gazebo: https://github.com/gazebosim/gz-sim
- ROS–Gazebo integration: https://github.com/gazebosim/ros_gz
- DAVE underwater simulator: https://github.com/Field-Robotics-Lab/dave (sonar from https://github.com/IOES-Lab/dave)
- NPS multibeam sonar: https://github.com/uuvsimulator/nps_uw_multibeam_sonar
