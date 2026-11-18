# ROBOCON 足式机器人仿真

这是一个基于 MuJoCo 的 ROBOCON 场地仿真器。项目加载 S-Dog 混合地形策略，提供任务赛物资搬运、障碍赛检查点航线和手动驾驶三种模式。自主导航使用 GLIM 3D LiDAR/IMU 后端；物资搬运、路径规划和规则计分由 Python 仿真程序处理。

## 启动

需要 Linux、可用的图形设备与 EGL。首次配置 Python 依赖；若未安装 `uv`，脚本会安装到当前用户目录：

```bash
./scripts/setup_env.sh --python-only
```

需要 3D 定位时，先按[复现说明](sim/docs/reproduction.md)准备 GLIM CPU 原生依赖和源码，再运行 `./scripts/setup_env.sh` 完成检查、构建和验证。启动仿真：

```bash
uv run python -m sim.app --no-open
```

浏览器访问 <http://127.0.0.1:8731/>。若端口已占用，可以添加 `--port 8740`。完整环境准备、构建与验收步骤见[复现说明](sim/docs/reproduction.md)。仿真服务退出时会一并关闭它启动的 GLIM 子进程；结束服务可在启动终端按 `Ctrl+C`。

## 操作

- 任务赛：先在小地图编辑物资位置，确认布置后启动任务。机器人按规则取箱、搬运、归位或叠放，最后返回启动区。
- 障碍赛：沿场地检查点运行演示航线，不等同于裁判规则的足端越障判定。
- 手动驾驶：`W/S` 前后、`A/D` 横移、`Q/E` 转向，`Shift` 加速，`Space` 停车。
- 3D 视角：拖拽旋转，滚轮缩放；可选跟随、全景、俯视和轨道视角。
- 定位：遥测显示 GLIM 状态、位姿年龄和误差。自主模式只使用新鲜的 GLIM 位姿；初始化期间或位姿超过 0.75 个仿真秒未更新时停车。手动模式仍可驾驶。

## 系统组成

`sim/app.py` 运行 MuJoCo 主循环和 HTTP 服务；`sim/robot.py` 执行策略推理；`sim/cargo.py` 管理物资接触、携带、释放和叠放；`sim/mission.py` 管理任务状态机与计分；`sim/planning.py` 提供占据栅格、A* 和路径跟踪。

`sim/localization.py` 中的 `LidarScanner3D` 由场景几何射线生成 Mid-360 形状的 16 线点云，扫描频率为 10 Hz，并采集 500 Hz IMU。默认 `balanced` 模式每帧使用 20,000 条射线，当前场地约返回 8,000 个有效点；`high_fidelity` 模式使用 70,400 条射线，适合测量高密度传感器压力。方位采用确定性的低重复序列，模拟非重复扫描。`sim/glim_bridge.py` 将传感器数据送入独立原生进程；`slam3d/native/glim_bridge.cpp` 调用 GLIM 的 CPU 里程计、GICP、直通子图和全局位姿图模块。地面真值只用于误差遥测，不传入估计器。定位延迟过高时自主控制会停车，不会用地面真值或旧二维定位器代替导航位姿。

## 常用检查

```bash
uv run python -m compileall -q sim slam3d
node --check sim/web/app.js
git diff --check
```

GLIM CPU 动态库检查、MuJoCo 传感器回放和完整运行记录见[复现说明](sim/docs/reproduction.md)。回放脚本会加速仿真，不应用它单独判断实时吞吐能力。

## 当前边界

- 障碍赛是检查点路线演示，尚未实现规则书中的裁判级越障检测。
- 搬运使用侧面接触吸附和区域释放，不模拟机械臂抓取。
- GLIM 的吞吐取决于 CPU 型号、负载和其他进程。应在目标机器的实际并发条件下检查位姿年龄与丢帧数；位姿失效时系统会停车。
- 任务得分取决于场景、策略和运行状态。历史单次测试成绩不代表每次运行结果。

## 文件位置

- 场景与策略：`sim/assets/`
- 仿真和控制：`sim/`
- 3D SLAM 构建与配置：`slam3d/`
- 从环境准备到运行验证的步骤：`sim/docs/reproduction.md`
