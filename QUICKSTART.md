# 北极熊仿真 — 快速启动指南

## 前置条件

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo Ignition Fortress (gz6 >= 6.18, sdformat >= 12.9)
- 工作空间已编译 (`~/ros_ws`, 29 个包全部通过)

## 启动步骤

### 终端 1：启动仿真器

```bash
cd ~/ros_ws && source install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

**关键：点击 Gazebo 窗口左下角橙色"开始"按钮**，等待 5 秒让机器人模型和 joint_states 完全加载。

> 也可用命令行启动物理仿真：
> ```bash
> ign service -s /world/default/control --reqtype ignition.msgs.WorldControl \
>   --reptype ignition.msgs.Boolean --timeout 2000 --req 'pause: false'
> ```

### 终端 2：启动导航 + RViz + map→odom 桥接

```bash
cd ~/ros_ws && source install/setup.bash

# 启动导航系统 (不使用 composition 避免 GraphicsMagick 崩溃)
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2025 slam:=False use_composition:=False &

# 等待导航栈初始化 (8 秒)
sleep 8

# 手动发布 map→odom 静态 TF (small_gicp 需要 PCD 文件，暂不可用)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom \
  --ros-args -r /tf_static:=/red_standard_robot1/tf_static \
  -r __ns:=/red_standard_robot1
```

### 在 RViz 中发送导航目标

1. RViz 窗口出现后，点击顶部工具栏 **"2D Goal Pose"** 按钮
2. 在地图上点击目标位置并拖拽设定方向
3. 机器人开始自主导航

## 常用变体

| 场景 | 导航命令变化 |
|------|-------------|
| SLAM 建图 | `slam:=True` |
| 不带 RViz | `use_rviz:=False` |
| 其他场地 | `world:=rmuc_2024` / `rmul_2024` / `rmul_2025` |
| 多机器人 | `ros2 launch pb2025_nav_bringup rm_multi_navigation_simulation_launch.py world:=rmul_2024 robots:="..."` |

## 手柄控制

```bash
ros2 launch pb_teleop_twist_joy teleop_twist_joy.launch.py
```

- **L1 (按钮 4)**: 启用/禁用遥控
- **R1 (按钮 5)**: Turbo 加速
- **左摇杆**: 平移 (X/Y)
- **右摇杆**: 旋转 (Yaw)

## 网页控制面板

```bash
# 操作手界面 → http://localhost:5000/
python3 src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/player_web/main_no_vision.py

# 裁判系统界面 → http://localhost:2350/
python3 src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/referee_web/main.py
```

## 已知问题及解决方案

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 点击"开始" IMU 崩溃 (segfault) | gz6 6.16 bug, SDF -string 模式下 sensor DOM 树不完整 | 升级 gz6 到 6.18+ |
| GraphicsMagick SIGSEGV | libGraphicsMagick 线程不安全，component_container 中并发调用 | `use_composition:=False` |
| small_gicp 崩溃 | PCD 先验点云文件缺失 (需从 FlowUs 下载) | 手动发布 map→odom 静态 TF |
| 导航目标被拒绝 | 启动时序：Gazebo joint_states 未就绪就启动导航 | 先启仿真 → 点击开始 → 等 5s → 再启导航 |

## 目录结构

```
~/ros_ws/
  src/
    rmu_gazebo_simulator/       # 仿真环境
    pb2025_sentry_nav/           # 导航系统 (哨兵)
    rmoss_gz_base/               # ROS↔Gazebo 桥接
    rmoss_gz_resources/          # 机器人模型资源
    ...
```

## 参考

- CLAUDE.md — 完整项目文档与开发工作流
- `pb2025_nav_bringup/launch/` — 启动文件
- `pb2025_nav_bringup/config/simulation/` — 仿真参数配置
