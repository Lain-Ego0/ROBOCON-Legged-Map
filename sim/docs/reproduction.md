# 环境搭建与运行复现

本文记录从克隆仓库到运行仿真的环境准备方法。Python 依赖由 `pyproject.toml` 和 `uv.lock` 固定；CUDA 驱动、Toolkit 及 GLIM C++ 依赖属于系统原生环境，不由 Python 锁文件管理。仿真和 GLIM 通过子进程标准输入输出传递传感器数据和位姿，不需要 ROS。

## 1. 目录与依赖

克隆项目后进入仓库根目录，运行一键配置脚本。若系统中没有 `uv`，脚本会将其安装到当前用户目录：

```bash
./scripts/setup_env.sh
```

脚本使用 `.python-version` 选择 Python 3.12，并按 `uv.lock` 安装仿真依赖。Linux 上的策略模型固定使用 CPU 版 PyTorch，这能避免为策略推理下载并安装另一套 CUDA 运行时；GLIM 是否使用 GPU 是独立的原生构建选项。完整模式还会检查 CUDA Toolkit 与 GLIM 源码，构建当前仓库集成的 CUDA 后端并运行动态库检查。它不会使用系统包管理器安装 CUDA、驱动或 C++ 库；这些依赖尚未安装时，脚本会列出缺项并退出。只需要 Python 环境时运行：

```bash
./scripts/setup_env.sh --python-only
```

随后启动仿真：

```bash
uv run python -m sim.app --port 8731 --no-open
```

PyTorch 可在没有 CUDA 的机器上执行 CPU 推理；GLIM 的 3D 定位需要可用的 NVIDIA GPU、驱动和 CUDA 原生依赖。若 `uv.lock` 有意更新，应使用 `uv lock` 生成并提交新的锁文件；复现环境时配置脚本使用 `uv sync --locked`，检测到清单与锁文件不一致会停止。

还需要 NVIDIA 驱动、CUDA Toolkit、`nvcc`、CMake，以及 GLIM 所需的 CUDA 版 `gtsam_points`、GTSAM、Eigen3、Boost serialization、OpenMP 和 spdlog。依赖版本与安装步骤请遵循 [GLIM 上游说明](https://github.com/koide3/glim) 和 [gtsam_points 上游说明](https://github.com/koide3/gtsam_points)。当前构建脚本按 CUDA 架构 8.9 配置，适用于 RTX 4090；其他 GPU 应先检查并调整 `slam3d/build_glim.sh` 中的 `CMAKE_CUDA_ARCHITECTURES`。

从 GLIM 官方仓库获取源码并放在构建目录：

```bash
mkdir -p build
git clone https://github.com/koide3/glim.git build/glim-upstream
```

也可以把源码放在其他目录，并用 `GLIM_SOURCE` 指定。构建脚本会把源码复制到忽略的 `build/glim-source/`，再应用本仓库的 API 兼容补丁；不会直接修改原始 GLIM 源码。

可通过以下变量选择源码、补丁工作副本和构建输出目录：

```bash
GLIM_SOURCE=build/glim-upstream \
GLIM_PATCHED_SOURCE_DIR=build/glim-source \
GLIM_BUILD_DIR=build/glim-runtime \
./slam3d/build_glim.sh
```

仿真应用目前从仓库内的 `build/glim-runtime/` 加载桥接程序和动态库，因此复现应用请使用默认 `GLIM_BUILD_DIR`。若构建到其他目录，需要同步修改 `sim/app.py` 中的桥接路径。

## 2. 构建 GLIM

构建时先激活包含 CUDA、CMake 和 GLIM C++ 依赖的环境，然后运行：

```bash
./slam3d/build_glim.sh
```

成功后应至少生成以下文件：

```text
build/glim-runtime/mujoco_glim_bridge
build/glim-runtime/libglim.so
build/glim-runtime/libodometry_estimation_gpu.so
```

构建失败时先检查：

```bash
cmake --version
nvcc --version
test -f build/glim-upstream/CMakeLists.txt
```

构建脚本要求 GLIM 源码能匹配仓库中的兼容补丁。若提示补丁上下文不匹配，确认下载的 GLIM 版本，不要直接在外部源码目录手工覆盖文件。

## 3. 检查 CUDA 与动态库

```bash
./slam3d/validate_glim.sh
```

脚本检查 CUDA 设备发现、显存分配与释放，以及 GLIM 核心库和 GPU 里程计模块能否加载。只有看到以下两行才表示该项检查通过：

```text
CUDA runtime passed on ... device(s)
GLIM core and GPU odometry plugin loaded successfully
```

若提示找不到 CUDA runtime 或动态库，先使用与构建时相同的 Conda 环境运行检查；不要只在系统 Python 中验证。

## 4. 启动仿真

```bash
uv run python -m sim.app --port 8731 --no-open
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
uv run python - <<'PY'
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
uv run python slam3d/replay_mujoco.py --duration 24
```

该脚本会比实时速度更快地推进仿真，适合检查点云/IMU 数据格式、GLIM 输出和估计误差，不是实时吞吐验收。若加速回放报告丢帧，不应据此直接判断实时应用同样会丢帧；实时运行应观察 `/api/state` 中处理数、丢帧数和位姿年龄。

## 6. 完整流程验收

1. 启动服务并等待 `glim_valid=true`。
2. 在任务赛中确认物资布置并启动自主任务。
3. 检查机器人能够取箱、携带、到达归位区并释放物资；同类第二件物资应叠放。
4. 任务结束时确认 8 件物资均处于 `home` 或 `stack`，任务状态为完成，机器人返回启动区。
5. 确认 GLIM 处理数持续增加、丢帧数稳定、机器人没有跌倒。
6. 执行 reset，确认物资回到存放位置、任务回到未开始状态，且新 GLIM 位姿时间戳属于重置后的仿真时间。

## 最近一次完整运行

2026-11-18，在约 3.33 Hz 扫描设置下完成了任务赛全流程：8 件物资全部处理，4 件放入归位区、4 件同类叠放；机器人返回启动区，任务用时 156.72 个任务秒。GLIM 处理 745 帧、输入队列丢帧为 0，机器人没有跌倒。复位后物资和任务状态清空，GLIM 重新输出位姿。

该次机器在仿真服务之外仍有另一个仿真进程运行，因此不能把它当作空载性能基准。监测期间位姿年龄大多在约 0.5 至 0.7 秒，曾有短暂过期样本，过期时自主控制按设计停车。复现实验时应先记录同机其他 GPU/CPU 重负载进程，并以本次运行的遥测为准。

## 常见问题

| 现象 | 检查方向 |
| --- | --- |
| 页面能打开但定位状态一直是初始化 | 查看 `glim_status.error`、系统临时目录中的 GLIM 桥接日志、CUDA 可见设备和动态库路径。 |
| 自主模式命令为零 | 查看 `glim_valid` 和位姿年龄；过期停车是安全守卫动作。若处理数不增加，再检查 GLIM 子进程及日志。 |
| 丢帧数持续增加 | 降低其他 GPU/CPU 负载，检查扫描处理速度；不要增大队列来掩盖长期吞吐不足。 |
| reset 后短暂停车 | GLIM 进程重启并重新初始化期间没有可用位姿，等待新的有效估计即可。 |
| 启动时报端口占用 | 用 `ss -ltnp` 查明监听进程，再更换端口或正常关闭所属仿真。 |

## 其他检查

```bash
uv run python -m compileall -q sim slam3d
node --check sim/web/app.js
git diff --check
```
