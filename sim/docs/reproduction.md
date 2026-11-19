# 环境搭建与运行复现

本文记录从克隆仓库到运行仿真的环境准备方法。Python、CPU PyTorch、MuJoCo、GLIM 构建工具和 C++ 依赖统一安装到 `build/robocon-env/` 中的单一 Conda 环境。仿真和 GLIM 通过子进程标准输入输出传递传感器数据和位姿，不需要 ROS。

## 1. 目录与依赖

克隆项目后进入仓库根目录，安装 Miniforge 或 Miniconda，再运行一键配置脚本。脚本会创建位于 `build/robocon-env/` 的固定 Conda 环境，并将 GLIM 源码、构建缓存和安装文件保存在仓库构建目录中：

```bash
./scripts/setup_env.sh
```

环境清单固定 Python 3.12、CPU-only PyTorch 和 MuJoCo。完整模式会自动获取固定版本源码，构建 CPU 版 GLIM 里程计与点云匹配模块，并运行动态库检查和传感器回放。只需要运行仿真而暂不构建 GLIM 时运行：

```bash
./scripts/setup_env.sh --python-only
```

配置完成后启动仿真：

```bash
./run_sim.sh --port 8731 --no-open
```

`run_sim.sh` 会自动激活仓库中的 Conda 环境。GLIM 启动前会检查桥接程序的动态依赖，缺少库时会在页面状态和 `/tmp/robocon-glim-bridge.log` 中列出未解析项，避免仿真持续排队丢弃扫描。高级用法可通过 `ROBOCON_ENV_PREFIX`、`GTSAM_POINTS_INSTALL_PREFIX` 和 `GLIM_BUILD_DIR` 覆盖默认构建路径。

策略模型和 GLIM 均运行在 CPU 上，不要求 NVIDIA GPU 或 CUDA Toolkit。环境定义位于 `slam3d/native-environment.yml`。策略推理固定使用 PyTorch CPU wheel，MuJoCo 使用与传感器接口匹配的 3.17.0 wheel。整个运行流程不安装或调用 CUDA。

C++ 构建依赖也由 `slam3d/native-environment.yml` 固定，当前支持 Linux x86-64。构建工具链以 glibc 2.28 为目标，覆盖 glibc 2.28 及以上的常见 Linux 发行版。`gtsam_points` 使用上游 v1.2.2，GLIM 使用固定提交 `1a197c999e4865f5cab7c9cd25474128413ebbca`。首次运行配置脚本会自动获取源码、构建 CPU 版本并执行动态库验收及传感器回放。构建缓存、依赖和运行库都保存在仓库的 `build/` 目录中。GLIM CPU 后端使用 GICP 做帧间匹配、直通子图生成和 GICP 位姿图回环优化。整个运行链使用 CPU 版本 PyTorch 和 CPU-only `gtsam_points`，无需 NVIDIA 驱动或 CUDA Toolkit。

若要手动获取 GLIM 源码，可检出项目固定提交：

```bash
git clone https://github.com/koide3/glim.git build/glim-upstream
git -C build/glim-upstream checkout --detach 1a197c999e4865f5cab7c9cd25474128413ebbca
```

构建脚本会复制源码到忽略的 `build/glim-conda-source/`，再应用本仓库的 API 兼容补丁，不会修改 GLIM 源码目录。

可通过以下变量选择源码、补丁工作副本和构建输出目录：

```bash
GLIM_SOURCE=build/glim-upstream \
GLIM_PATCHED_SOURCE_DIR=build/glim-conda-source \
GTSAM_POINTS_INSTALL_PREFIX=build/glim-conda-deps \
GLIM_BUILD_DIR=build/glim-conda-runtime \
GTSAM_POINTS_DIR=build/glim-conda-deps/lib/cmake/gtsam_points \
./scripts/with_project_env.sh ./slam3d/build_glim.sh
```

仿真应用默认从 `build/glim-conda-runtime/` 加载桥接程序和动态库，也可通过 `ROBOCON_GLIM_BUILD_DIR` 指定其他构建目录。配置脚本会将 `gtsam_points` 安装到 `build/glim-conda-deps/`，并将 GLIM 输出到上述目录。

## 2. 构建 GLIM

手动构建时通过仓库脚本激活统一 Conda 环境：

```bash
./scripts/with_project_env.sh ./scripts/build_gtsam_points_cpu.sh
GTSAM_POINTS_DIR=build/glim-conda-deps/lib/cmake/gtsam_points \
  ./scripts/with_project_env.sh ./slam3d/build_glim.sh
```

成功后应至少生成以下文件：

```text
build/glim-conda-runtime/mujoco_glim_bridge
build/glim-conda-runtime/libglim.so
build/glim-conda-runtime/libodometry_estimation_cpu.so
build/glim-conda-runtime/libsub_mapping_passthrough.so
build/glim-conda-runtime/libglobal_mapping_pose_graph.so
```

构建失败时先检查：

```bash
cmake --version
test -f build/glim-upstream/CMakeLists.txt
```

构建脚本要求 GLIM 源码能匹配仓库中的兼容补丁。若提示补丁上下文不匹配，确认下载的 GLIM 版本，不要直接在外部源码目录手工覆盖文件。

## 3. 检查 CPU 模块与动态库

```bash
./slam3d/validate_glim.sh
```

脚本检查 GLIM 核心库、CPU 里程计、直通子图和全局位姿图模块能否加载。看到以下提示表示动态库检查通过：

```text
GLIM CPU odometry, passthrough submapping, and pose-graph modules loaded successfully
```

标准启动脚本会自动激活同一 Conda 环境并检查动态库。若手动指定了不同的构建目录，请在对应环境中运行验证脚本。

## 4. 启动仿真

```bash
ROBOCON_GLIM_BUILD_DIR=build/glim-conda-runtime ./run_sim.sh --port 8731 --no-open
```

浏览器打开 <http://127.0.0.1:8731/>。如端口被占用，换用空闲端口。启动日志出现 `simulator ready` 表示 HTTP 服务已启动，但不代表 GLIM 已经给出定位结果。应用会在任务启动前持续采集传感器数据并初始化估计器。

打开 `/api/state` 查看运行数据。重点检查：

- `loc.glim_status.ready`：GLIM 子进程正在运行。
- `loc.glim_valid`：最新估计存在且年龄不超过 0.75 个仿真秒。
- `loc.glim_latency_ms`：当前位姿年龄，越大表示估计越滞后。
- `loc.glim_status.processed_scans`：已处理扫描数，应持续增加。
- `loc.glim_status.dropped_scans`：输入队列溢出丢弃数，稳定运行时应保持不增长。
- `loc.control_source`：自主模式下是 `glim` 或 `stopped`；手动模式为 `manual`。
- `error`、`render_error`：仿真和渲染错误。

任务赛须先在小地图编辑并确认物资布置，再启动任务。验收自主导航时，等 `glim_valid` 为真后再开始；如果位姿超时，机器人停车是预期的安全行为。可以用 `/api/mission` 的 `reset` 动作验证任务和 GLIM 重启：

```bash
python - <<'PY'
import json
from urllib.request import Request, urlopen

url = "http://127.0.0.1:8731/api/mission"
request = Request(
    url,
    data=json.dumps({"action": "reset"}).encode(),
    headers={"Content-Type": "application/json"},
)
print(urlopen(request).read().decode())
PY
```

结束服务应在启动它的终端按 `Ctrl+C`。应用收到退出信号后会关闭它自己启动的 GLIM 子进程。可检查进程是否退出：

```bash
ps -eo pid,ppid,cmd | grep -E 'sim\.app|mujoco_glim_bridge' | grep -v grep
```

不要对来源不明的仿真进程直接发送终止信号；先根据 PID、父进程、启动时间和监听端口确认它属于哪个会话。

## 5. 传感器回放

回放使用 MuJoCo Python 环境生成传感器数据，并将数据送入原生 GLIM：

```bash
ROBOCON_GLIM_BUILD_DIR=build/glim-conda-runtime ./scripts/with_project_env.sh python slam3d/replay_mujoco.py --duration 24
```

当前仿真传感器参数按 Mid-360 的在线时序设置：3D LiDAR 为 10 Hz，IMU
在 MuJoCo 物理步上以 500 Hz 采样。默认 `balanced` 模式每帧使用 20,000
条几何射线，当前场地约返回 8,000 个有效点；`high_fidelity` 模式使用
70,400 条射线，适合测量高密度传感器压力。射线使用批量投射和确定性的
低重复方位序列，因此不会再使用旧的约 3.33 Hz、300 方位采样配置。参数
集中在 `sim/config.py` 的 `LIDAR3D_*` 常量中。

选择传感器负载模式：

```bash
ROBOCON_LIDAR_PROFILE=realtime ./run_sim.sh
ROBOCON_LIDAR_PROFILE=balanced ./run_sim.sh
ROBOCON_LIDAR_PROFILE=high_fidelity ./run_sim.sh
```

该脚本会比实时速度更快地推进仿真，适合检查点云/IMU 数据格式、GLIM 输出和估计误差，不是实时吞吐验收。若加速回放报告丢帧，不应据此直接判断实时应用同样会丢帧；实时运行应观察 `/api/state` 中处理数、丢帧数和位姿年龄。

## 6. 完整流程验收

1. 启动服务并等待 `glim_valid=true`。
2. 在任务赛中确认物资布置并启动自主任务。机器人只能在箱体进入左右侧机械臂的侧面中段工作区时抓取，归位区接触检测以同一夹持位置为准。
3. 检查机器人能够取箱、携带、到达归位区并释放物资；同类第二件物资应叠放。
4. 任务结束时确认 8 件物资均处于 `home` 或 `stack`，任务状态为完成，机器人返回启动区。
5. 确认 GLIM 处理数持续增加、丢帧数稳定、机器人没有跌倒。
6. 执行 reset，确认物资回到存放位置、任务回到未开始状态，且新 GLIM 位姿时间戳属于重置后的仿真时间。

## 最近一次验证

2026-11-18，使用 10 Hz 扫描、CPU-only GLIM 和新的双侧夹持工作区运行任务赛至 180 秒时限。机器人没有跌倒，GLIM 处理约 2,100 帧，丢帧为 0、同步警告为 0。6 件物资完成归位或叠放，另 2 件已抓取并由机械臂夹持，任务因比赛时间到结束，机器人未返回启动区。最终得分为 1550/1900。此次运行验证了中段工作区抓取、双侧携带及归位区释放，但完整 8 件物资流程仍未通过。

此前在旧的约 3.33 Hz 扫描设置下，CPU 版 GLIM 曾完成 8 件物资处理和返回启动区，用时 140.44 秒，得分 1800/1900。该历史记录没有覆盖当前扫描和机械臂工作区设置。

较早的 GPU 版测试记录如下：

2026-11-18，在约 3.33 Hz 扫描设置下完成了任务赛全流程：8 件物资全部处理，4 件放入归位区、4 件同类叠放；机器人返回启动区，任务用时 156.72 个任务秒。GLIM 处理 745 帧、输入队列丢帧为 0，机器人没有跌倒。复位后物资和任务状态清空，GLIM 重新输出位姿。

该次机器在仿真服务之外仍有另一个仿真进程运行，因此不能把它当作空载性能基准。监测期间位姿年龄大多在约 0.5 至 0.7 秒，曾有短暂过期样本，过期时自主控制按设计停车。复现实验时应先记录同机其他 GPU/CPU 重负载进程，并以本次运行的遥测为准。

## 常见问题

| 现象 | 检查方向 |
| --- | --- |
| 页面能打开但定位状态一直是初始化 | 查看 `glim_status.error`、系统临时目录中的 GLIM 桥接日志和动态库路径。 |
| 自主模式命令为零 | 查看 `glim_valid` 和位姿年龄；过期停车是安全守卫动作。若处理数不增加，再检查 GLIM 子进程及日志。 |
| 丢帧数持续增加 | 降低其他 CPU 负载，检查扫描处理速度；不要增大队列来掩盖长期吞吐不足。 |
| reset 后短暂停车 | GLIM 进程重启并重新初始化期间没有可用位姿，等待新的有效估计即可。 |
| 启动时报端口占用 | 用 `ss -ltnp` 查明监听进程，再更换端口或正常关闭所属仿真。 |

## 其他检查

```bash
python -m compileall -q sim slam3d
node --check sim/web/app.js
git diff --check
```
