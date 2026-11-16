# dave_multibeam_sonar

The DAVE multibeam forward-looking sonar, built for **ROS 2 Jazzy / Gazebo Harmonic**.

Source: [IOES-Lab/dave](https://github.com/IOES-Lab/dave) `gazebo/dave_gz_multibeam_sonar`
(`multibeam_sonar` + `multibeam_sonar_system`), commit `c1c2fba`, Apache-2.0 (see `LICENSE`).
Upstream targets ROS 2 Lyrical / Gazebo Jetty.

Requires an NVIDIA GPU and the CUDA toolkit. Without `nvcc` the package builds
empty, so the rest of the workspace still builds.

## What it provides

- `libmultibeam_sonar.so`: a custom gz-sensors `RenderingSensor`
  (`<sensor type="custom" gz:type="multibeam_sonar">`) that owns its GPU rays
  and runs the CUDA acoustic model on each frame.
- `libmultibeam_sonar_system.so`: the world system
  (`<plugin filename="multibeam_sonar_system" name="custom::MultibeamSonarSystem"/>`)
  that creates and updates those sensors.

Outputs, for a sensor with `<topic>T</topic>`:

| Topic | Transport | Type |
|---|---|---|
| `T/sonar_image_raw` | ROS 2 | `marine_acoustic_msgs/ProjectedSonarImage`, FLOAT32 dB |
| `T/sonar_image` | ROS 2 | `sensor_msgs/Image` fan display (built only when subscribed) |
| `T/normal_image` | ROS 2 | `sensor_msgs/Image` (built only when subscribed) |
| `T/point_cloud` | gz-transport | `gz.msgs.PointCloudPacked` |

Note: each acoustic beam's aperture is the whole configured vertical span, and
rays are cast over the union of beam footprints, so the rays cover **twice** the
configured `vertical` `min_angle`..`max_angle`.

## Changes from upstream

The acoustic model (`sonar_calculation_cuda.cu`) and the messages are unchanged.

1. **Build:** Harmonic package versions (`gz-sim8`, `gz-sensors8`, `gz-rendering8`,
   `gz-transport13`, `gz-msgs10`, `gz-common5`, `gz-plugin2`, `gz-math7`) and
   CUDAToolkit targets; both libraries in one package.
2. **Sensor discovery** (`MultibeamSonarSystem.cc`): upstream found sensors with
   `EachNew`, which misses models spawned at runtime through the `/create`
   service (they are created in UserCommands' PreUpdate, after this system ran).
   It now visits every custom sensor and skips ones already handled.
3. **Sonar model off the render thread** (`MultibeamSonarSensor.cc/.hh`): gz-sim
   waits for the render thread, so computing the sonar inside the GPU-rays frame
   callback slowed the whole simulation to the sonar's frame time. The callback
   now hands the latest depth image to a worker thread (dropping frames it can't
   keep up with), like Gazebo Classic ran sensors off the physics thread.
4. **Display images only when subscribed**, and drawn through a pixel → (beam, bin)
   lookup table built once (upstream drew one `cv::ellipse` per beam per range bin,
   ~0.3 s per frame for 513 beams x 598 bins). Frames with no returns draw black
   (upstream normalised them into a uniform bright fan).
5. **Small speed-ups with identical output:** point-cloud field offsets looked up
   once per frame instead of per point, and the dB image written straight into
   the message buffer.

Measured in the EROAS World A demo on a GTX 1050 Ti (513 beams, 501 vertical
rays, 8 Hz): real-time factor 0.81 (lidar-only baseline 0.95); sonar images
arrive in ROS at about 5 Hz.
