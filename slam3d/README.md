# GLIM 3D SLAM Integration

This directory contains the native 3D SLAM backend and its MuJoCo sensor adapter.
Autonomous logistics and obstacle navigation use fresh GLIM poses. The older
2D localizer remains available for comparison telemetry, but it is not a silent
control fallback: autonomous motion stops until GLIM initializes and whenever
its latest pose is more than 0.75 simulated seconds old.

## Build

GLIM upstream source remains unchanged outside this repository. Place a GLIM
checkout in `build/glim-upstream/`; the build script copies it into the ignored
`build/glim-cpu-source/`, applies the API compatibility patch in
`slam3d/patches/`, builds CPU odometry, passthrough submapping, GICP pose-graph
modules and the native sensor bridge, and writes generated files to
`build/glim-cpu-runtime-no-cuda/`:

```bash
./slam3d/build_glim.sh
```

Override source, patched-source, or output locations with `GLIM_SOURCE`,
`GLIM_PATCHED_SOURCE_DIR`, `GTSAM_POINTS_DIR`, and `GLIM_BUILD_DIR`. The build sets
`BUILD_WITH_CUDA=OFF`; it requires CMake and the GLIM native dependencies.
It disables the optional GLIM viewer and OpenCV.

Run the CPU module and runtime-linker smoke checks inside the same environment
after building:

```bash
./slam3d/validate_glim.sh
```

GLIM's native build requires `gtsam_points` 1.2.2+, GTSAM 4.2+,
Eigen3, Boost serialization, OpenMP, and spdlog. ROS 2 is not required to
build the GLIM core; a ROS 2 Jazzy adapter can be added independently if the
simulator is later moved to ROS message transport.

## Runtime Sensor Bridge

MuJoCo emits a 16-beam, 10 Hz 3D ray-cast scan with per-point acquisition
times, plus 500 Hz accelerometer/gyro samples from a shared sensor site. The
default `balanced` profile uses 20,000 rays per frame and the current field
produces about 8,000 returned points per frame. `high_fidelity` raises the
budget to 70,400 rays per frame for sensor-load tests. The ray budget is
higher than the returned point count because many rays miss the field
geometry. Azimuths use a deterministic low-repetition sequence to approximate
the Mid-360 non-repetitive scan pattern.
The Python adapter batches them into a bounded queue;
`mujoco_glim_bridge` performs
GLIM preprocessing, CPU GICP/IMU odometry, asynchronous submap generation, and
CPU GICP pose-graph loop matching outside the physics thread. GLIM poses and ground-truth
errors are published separately; ground truth is never sent into the estimator.

Navigation consumes the aligned GLIM XY/yaw pose. GLIM's arbitrary initial map
frame is aligned to the selected MuJoCo spawn pose; ground truth is used only
for error telemetry and is never sent to the estimator. Reset restarts GLIM and
isolates old asynchronous pose records by generation. The existing path-stall
detection and bounded replanning remain enabled.

## Verification

From the repository root, run the CPU module smoke check and sensor replay
with environments that provide the native GLIM libraries and MuJoCo:

```bash
./slam3d/validate_glim.sh
uv run python slam3d/replay_mujoco.py --duration 24
```

The replay advances simulation faster than real time. Use it to check the
sensor format and estimator output, not as a real-time throughput benchmark.
The full environment setup, run procedure, telemetry checks, and most recent
end-to-end result are documented in [the reproduction guide](../sim/docs/reproduction.md).
