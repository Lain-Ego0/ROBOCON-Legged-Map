# GLIM 3D SLAM Integration

This directory contains the native 3D SLAM backend and its MuJoCo sensor adapter.
Autonomous logistics and obstacle navigation use fresh GLIM poses. The older
2D localizer remains available for comparison telemetry, but it is not a silent
control fallback: autonomous motion stops until GLIM initializes and whenever
its latest pose is more than 0.75 simulated seconds old.

## Build

The setup workflow creates a single Conda environment under `build/robocon-env/`,
fetches fixed GLIM and `gtsam_points` revisions, builds the CPU modules, checks all
runtime libraries, and runs a sensor replay:

```bash
./scripts/setup_env.sh
```

Build output stays under the ignored `build/` directory. To build individual
components, activate the single Conda environment through the project wrapper:

```bash
./scripts/with_project_env.sh ./scripts/build_gtsam_points_cpu.sh
GTSAM_POINTS_DIR=build/glim-conda-deps/lib/cmake/gtsam_points \
  ./scripts/with_project_env.sh ./slam3d/build_glim.sh
GLIM_BUILD_DIR=build/glim-conda-runtime \
  ./scripts/with_project_env.sh ./slam3d/validate_glim.sh
```

The complete Conda package set is described by `native-environment.yml`. The
compiler uses the glibc 2.28 sysroot for broad Linux binary compatibility. The build sets `BUILD_WITH_CUDA=OFF` and
disables the optional GLIM viewer and OpenCV. ROS 2 is not required to build
the GLIM core.

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
with the project Conda environment activated:

```bash
./scripts/with_project_env.sh ./slam3d/validate_glim.sh
./scripts/with_project_env.sh python slam3d/replay_mujoco.py --duration 24
```

The replay advances simulation faster than real time. Use it to check the
sensor format and estimator output, not as a real-time throughput benchmark.
The full environment setup, run procedure, telemetry checks, and most recent
end-to-end result are documented in [the reproduction guide](../sim/docs/reproduction.md).
