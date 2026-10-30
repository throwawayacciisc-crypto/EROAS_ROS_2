# EROAS Underwater Vehicle Simulation

## Quick start (ROS 2 Jazzy / Gazebo Harmonic, original EROAS logic)

By default the demo now runs the **original AIRLabIISc/EROAS algorithm** on the
**DAVE multibeam sonar**, wired as in the ROS 1 `start_EROAS_demo.launch`:

```
DAVE sonar image ──► ros1_only_gap.py (SPD2C) ──► /rexrov2/cmd_vel_1 ──► ros1_velocity_cbf.py (ST-CBF) ──► /rexrov2/cmd_vel
        └────────► ros1_sonar_reconstruction.py (SCG) ──► /rexrov2/point_cloud ──┘
```

The `ros1_*` scripts in `navigator_auv/scripts/` are line-for-line ROS 2 ports of
the original `only_gap.py`, `velocity_cbf.py` and `sonar_reconstruction.py`;
every deviation is marked `ADAPTED` in the code. Needs an NVIDIA GPU and the
CUDA toolkit (for `dave_multibeam_sonar`, see its README).

```bash
source /opt/ros/jazzy/setup.bash
cd ~/eroas_ws && colcon build --cmake-args -DCMAKE_CUDA_HOST_COMPILER=g++-12
source install/local_setup.bash
ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py world_name:=world_a
# sonar fan image:
ros2 run rqt_image_view rqt_image_view /rexrov2/blueview_p900/sonar_image
```

Options:

| Argument | Default | Other values |
|---|---|---|
| `navigation` | `ros1` (original EROAS nodes) | `port` (this repo's reworked `only_gap.py` / `velocity_cbf.py`) |
| `sonar_source` | `dave` (DAVE acoustic sonar, GPU) | `lidar` (synthetic sonar image from a gz GPU lidar; no CUDA) |
| `physics_engine` | `gz-physics-bullet-featherstone-plugin` | `gz-physics-dartsim-plugin` (fails with LCP NaN errors in World A) |

### Watching it run (Gazebo window + RViz)

Start the server with `gui:=false`, wait for `Entity creation successful`
(RexROV2 spawned), then open the GUI and RViz. A GUI started before the spawn
never shows the vehicle.

```bash
ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py gui:=false world_name:=world_a
gz sim -g --force-version 8          # after the vehicle has spawned
rviz2 -d tools/sonar.rviz            # sonar fan image, Reliable QoS
python3 tools/teleop.py              # manual driving + sonar tilt (stop the EROAS nodes first)
```

`tools/teleop.py` keys: `w/s` surge, `a/d` yaw, `q/e` sway, `r/f` heave,
`space` stop, `i/k` sonar tilt ±5°, `o` tilt 0°, `x` quit.

### Harmonic port fixes (2026-10-30)

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

The rest of this README describes the earlier WSL setup and the `navigation:=port` stack.

A step-by-step guide to building, launching, visualizing, and testing the EROAS underwater vehicle simulation using **ROS 2 Jazzy, Gazebo Harmonic, and Ubuntu 24.04 on WSL**.

## 1. Project Overview

EROAS is an underwater autonomous vehicle navigation stack that combines global and local path planning, forward-looking sonar perception, and safe control using Control Barrier Functions (CBF).

This project migrates the original ROS 1 / Gazebo Classic implementation to ROS 2 Jazzy / Gazebo Harmonic.

The simulation includes:

- Underwater vehicle simulation using RexROV2.
- Sonar-based obstacle perception.
- Contour-based navigation in World A.
- SPD2C and Control Barrier Function (CBF) control.
- Waypoint hovering.
- Sonar-ray visualization through ROS 2 markers and RViz2.
- Obstacle avoidance and stuck-recovery mechanisms.

**Note:** World A still has a documented navigation reliability issue in a winding multi-pocket region around `y ≈ 60–68`. A successful launch does not guarantee that every navigation run will finish successfully.

## 2. Prerequisites

The following environment is assumed to be installed and configured:

- Windows Subsystem for Linux (WSL).
- Ubuntu 24.04 distribution named `Ubuntu-24.04`.
- ROS 2 Jazzy.
- Gazebo Harmonic.
- Required ROS 2 and Gazebo dependencies.
- EROAS source repository.
- Workspace located at `~/eroas_ws`.

The following instructions assume that the workspace has already been created and built at least once.

## 3. Open Ubuntu 24.04 in WSL

Open Windows PowerShell and run:
wsl -d Ubuntu-24.04

Run all subsequent Linux commands inside this Ubuntu terminal.

Load the ROS 2 and EROAS environments:
source /opt/ros/jazzy/setup.bash
source ~/eroas_ws/install/local_setup.bash

Run these commands in every new terminal before executing ROS 2 commands.

## 4. Update the EROAS Repository

Navigate to the repository:
cd ~/eroas_ws/src/uuv_simulator

Check the current repository state:
git status
git branch --show-current

Pull the project branch containing the documented World A fixes:
git pull origin fix/eroas-world-a-navigation

If your repository is stored in another location, replace the path with your actual repository path.

## 5. Build the Workspace

Navigate to the workspace root:
cd ~/eroas_ws

Source ROS 2:
source /opt/ros/jazzy/setup.bash

Build the workspace:
colcon build

After the build completes, source the newly built workspace:
source install/local_setup.bash

For small changes limited to the `uuv_assistants` package, the following targeted build can be used instead:
colcon build --packages-select uuv_assistants
source install/local_setup.bash

If the build fails, resolve the reported dependency or compilation errors before attempting to launch the simulation.

## 6. Launch the Simulation in World A

With the environment sourced, run:
ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py world_name:=world_a

This launches the simulation with the Gazebo graphical interface.

World A is the Figure-8 navigation scenario described in the project report.

### Run in headless mode

For diagnostic runs without a Gazebo GUI, execute:
ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py gui:=false world_name:=world_a start_navigator:=true start_cbf:=true

Headless mode is useful for testing, debugging, and capturing logs without rendering the Gazebo window.

## 7. Verify the Simulation

Open a second Ubuntu-24.04 WSL terminal and run:
source /opt/ros/jazzy/setup.bash
source ~/eroas_ws/install/local_setup.bash

### 7.1 Check running ROS 2 nodes
ros2 node list

A healthy launch typically shows approximately 10–19 nodes, including the six thruster plugin nodes:
thruster_ros_plugin_0
thruster_ros_plugin_1
thruster_ros_plugin_2
thruster_ros_plugin_3
thruster_ros_plugin_4
thruster_ros_plugin_5

The exact node count may vary.

### 7.2 Verify sonar data publishing

Run:
ros2 topic hz /rexrov2/blueview_p900_point_cloud

The topic should publish continuously at a steady rate.

You can inspect the topic information with:
ros2 topic info /rexrov2/blueview_p900_point_cloud

List all available topics:
ros2 topic list

A continuous point-cloud stream confirms that sonar data is being produced. It does not independently confirm that sonar visualization, obstacle avoidance, or navigation is functioning correctly.

## 8. Visualize Sonar Rays Using RViz2

The project uses a custom marker-publishing bridge to visualize sonar rays under Gazebo Harmonic.

Open another Ubuntu-24.04 WSL terminal and source the environments:
source /opt/ros/jazzy/setup.bash
source ~/eroas_ws/install/local_setup.bash

Launch RViz2 using the project's configuration:
rviz2 -d ~/eroas_ws/src/navigator_auv/rviz/sonar_rays.rviz

The configuration subscribes to:
/rexrov2/sonar_ray_markers

The sonar-ray marker publisher is implemented in `sonar_ray_markers.py`.

### If sonar rays are not visible

Check whether the marker topic exists:
ros2 topic list | grep sonar_ray_markers

Inspect the topic:
ros2 topic info /rexrov2/sonar_ray_markers

Inspect published marker messages:
ros2 topic echo /rexrov2/sonar_ray_markers

If the topic is missing or no messages are published:

1. Confirm that the simulation is running.
2. Check the simulation terminal for plugin or bridge errors.
3. Confirm that the marker-publishing script is present and running.
4. Check the topic configured in RViz2.
5. Verify that RViz2 uses the correct fixed frame.

Gazebo Harmonic does not automatically display sonar rays from the world file in the same way Gazebo Classic did. The custom marker bridge and RViz2 configuration provide the documented visualization method.

Gazebo's native Visualize Lidar panel is another option, but it may need to be configured manually for each session.

## 9. Troubleshooting

### Problem 1: `ros2: command not found`

Source the ROS 2 environment:
source /opt/ros/jazzy/setup.bash

Then source the workspace:
source ~/eroas_ws/install/local_setup.bash

Ensure you are using the correct Ubuntu-24.04 WSL distribution.

### Problem 2: Package or launch file not found

Check whether the package is discoverable:
ros2 pkg list | grep rexrov2_gazebo

Check its installation prefix:
ros2 pkg prefix rexrov2_gazebo

If the package is missing, check the source repository, rebuild the workspace, and source the environment again.

### Problem 3: Gazebo plugin-loading errors

Gazebo Harmonic uses a different plugin architecture from Gazebo Classic.

Check the following:

- Whether the required plugin packages are installed.
- Whether the world and model files use compatible plugin declarations.
- Whether the SDF files are compatible with the installed Gazebo version.
- Whether the simulation terminal reports missing libraries or plugin-loading failures.

The migration report identifies legacy ROS 1 plugin declarations and older SDF versions as causes encountered during development.

### Problem 4: Sonar data works but rays are invisible

Sonar sensing and sonar visualization are separate components.

Check that:

- `/rexrov2/blueview_p900_point_cloud` is publishing.
- `/rexrov2/sonar_ray_markers` exists.
- `sonar_ray_markers.py` is running.
- RViz2 is subscribed to the correct marker topic.
- The RViz2 fixed frame is configured correctly.

### Problem 5: Vehicle oscillation or incorrect pivot movement

The migration report traced the pivot issue to a difference in the thruster-mixing matrix and oscillation to control tuning.

Inspect the thruster allocator and controller logs before changing parameters. Validate any changes across multiple test worlds.

### Problem 6: Obstacle avoidance or CBF does not behave correctly

The project report identified an integration issue in which `velocity_cbf.py` was not consuming sonar-derived obstacle data published by `only_gap.py`.

Verify that the obstacle data is being published and consumed correctly. Inspect the controller logs and relevant ROS 2 topics.

The existence of a node does not prove that its input data is connected correctly.

### Problem 7: Changes do not appear after rebuilding

Rebuild the affected package and source the workspace again:

```bash
cd ~/eroas_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select uuv_assistants
source install/local_setup.bash
```

Use a full workspace build if changes affect multiple packages.

## 10. Simulation Testing Checklist

Use this checklist to verify the simulation step by step.

- [ ] Ubuntu 24.04 WSL environment opens successfully.
- [ ] ROS 2 Jazzy is sourced.
- [ ] EROAS workspace is sourced.
- [ ] The workspace builds without errors.
- [ ] The World A launch command starts successfully.
- [ ] Expected ROS 2 nodes are running.
- [ ] The sonar point-cloud topic publishes continuously.
- [ ] The sonar-ray marker topic is available.
- [ ] RViz2 displays the sonar rays.
- [ ] The vehicle responds to navigation and control commands.
- [ ] Vehicle motion is stable rather than excessively oscillatory.
- [ ] Obstacle detection and avoidance are tested.
- [ ] Recovery behavior is tested in constrained spaces.
- [ ] Relevant tests are repeated in multiple worlds.
- [ ] Logs are reviewed for failures or unexpected behavior.

A successful launch, a healthy sonar stream, stable vehicle control, and successful completion of the navigation route are separate validation outcomes.

## 11. Known Limitations

The project report documents the following remaining issues:

1. **World A navigation reliability:** The vehicle can still become trapped in a winding multi-pocket region around `y ≈ 60–68`.
2. **Dead-end heading memory:** The `known_bad_headings` mechanism does not yet have time-based expiry or positional scoping. A heading can remain excluded after the vehicle moves away from the original location.
3. **Regression testing:** Automated regression testing across all test worlds remains a future enhancement.

The project includes improvements to fully boxed-in recovery, closing-corridor detection, and dead-end exclusion in both stuck-recovery and `gap_follow`. These improvements address specific failure modes but do not guarantee successful completion of every World A run.

## 12. Recommended Validation Workflow

For repeatable testing:

1. Open a fresh Ubuntu-24.04 WSL terminal.
2. Source ROS 2 Jazzy and the EROAS workspace.
3. Build the workspace after making code changes.
4. Launch World A in Gazebo.
5. Check the ROS 2 node list.
6. Verify that the sonar point-cloud topic publishes continuously.
7. Launch RViz2 and inspect the sonar-ray visualization.
8. Observe vehicle motion, waypoint behavior, and obstacle avoidance.
9. Capture logs when unexpected behavior occurs.
10. Repeat relevant tests in the corridor, open-tank, and cluttered-obstacle worlds.
11. Record launch success, sensor health, control stability, and route completion separately.

## 13. References

- EROAS repository: https://github.com/AIRLabIISc/EROAS
- Gazebo simulation engine: https://github.com/gazebosim/gz-sim
- ROS–Gazebo integration: https://github.com/gazebosim/ros_gz
- DAVE underwater simulator: https://github.com/Field-Robotics-Lab/dave
- Original multibeam sonar package: https://github.com/uuvsimulator/nps_uw_multibeam_sonar

## 14. Quick Start

Run the following commands inside the Ubuntu-24.04 WSL shell:
# Source ROS 2 and the workspace
source /opt/ros/jazzy/setup.bash
source ~/eroas_ws/install/local_setup.bash

# Update the repository
cd ~/eroas_ws/src/uuv_simulator

git pull origin fix/eroas-world-a-navigation

# Build the workspace
cd ~/eroas_ws

source /opt/ros/jazzy/setup.bash

colcon build

source install/local_setup.bash

# Launch World A
ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py world_name:=world_a

In a second sourced terminal, verify the sonar data:
ros2 topic hz /rexrov2/blueview_p900_point_cloud

In another sourced terminal, display the sonar rays:
rviz2 -d ~/eroas_ws/src/navigator_auv/rviz/sonar_rays.rviz

For small changes limited to `uuv_assistants`, use:
colcon build --packages-select uuv_assistants

source install/local_setup.bash

