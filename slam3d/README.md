# GLIM 3D SLAM Integration

This directory contains the native 3D SLAM backend and its MuJoCo sensor adapter.
Autonomous logistics and obstacle navigation use fresh GLIM poses. The older
2D localizer remains available for comparison telemetry, but it is not a silent
control fallback: autonomous motion stops until GLIM initializes and whenever
its latest pose is more than 0.75 simulated seconds old.

## Build

GLIM upstream source remains unchanged outside this repository. Place a GLIM
checkout in `build/glim-upstream/`; the build script copies it into the ignored
`build/glim-source/`, applies the API compatibility patch in
`slam3d/patches/`, builds a native sensor bridge, and writes generated files to
`build/glim-runtime/`:

```bash
./slam3d/build_glim.sh
```

Override source, patched-source, or output locations with `GLIM_SOURCE`,
`GLIM_PATCHED_SOURCE_DIR`, and `GLIM_BUILD_DIR`. The script deliberately
requires both CMake and `nvcc`,
targets the RTX 4090 compute capability (8.9), and fails rather than silently
building a CPU-only backend. It disables the optional GLIM viewer and OpenCV to
keep the first integration build focused on odometry and mapping.

Run the CUDA device/memory and runtime-linker smoke checks inside the same
environment after building:

```bash
./slam3d/validate_glim.sh
```

GLIM's native build also requires CUDA-enabled `gtsam_points` 1.2.2+, GTSAM
4.2+, Eigen3, Boost serialization, OpenMP, and spdlog. Install the CUDA toolkit
and libraries matching it before running the script. ROS 2 is not required to
build the GLIM core; a ROS 2 Jazzy adapter can be added independently if the
simulator is later moved to ROS message transport.

## Runtime Sensor Bridge

MuJoCo emits a 16-beam, 3.33 Hz spinning 3D ray-cast scan with 300 azimuth
samples per revolution and per-point acquisition times, plus 500 Hz
accelerometer/gyro samples from a shared sensor site. The reduced scan cadence keeps the estimator
queue current on the available runtime while retaining full per-scan angular
sampling. The Python adapter batches them into a bounded queue;
`mujoco_glim_bridge` performs
GLIM preprocessing, GPU VGICP/IMU odometry, asynchronous submapping, and global
pose-graph loop matching outside the physics thread. GLIM poses and ground-truth
errors are published separately; ground truth is never sent into the estimator.

Navigation consumes the aligned GLIM XY/yaw pose. GLIM's arbitrary initial map
frame is aligned to the selected MuJoCo spawn pose; ground truth is used only
for error telemetry and is never sent to the estimator. Reset restarts GLIM and
isolates old asynchronous pose records by generation. The existing path-stall
detection and bounded replanning remain enabled.

## Verification

From the repository root, run the CUDA/runtime smoke check and sensor replay
with environments that provide the native GLIM libraries and MuJoCo:

```bash
./slam3d/validate_glim.sh
python slam3d/replay_mujoco.py --duration 24
```

The replay advances simulation faster than real time. Use it to check the
sensor format and estimator output, not as a real-time throughput benchmark.
The full environment setup, run procedure, telemetry checks, and most recent
end-to-end result are documented in [the reproduction guide](../sim/docs/reproduction.md).
