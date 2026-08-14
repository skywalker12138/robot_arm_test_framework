# ROS 2 Robot Arm Trajectory Test Framework

[中文](#中文说明) | [English](#english)

## 中文说明

### 项目简介

这是一个基于 ROS 2 Jazzy 的六关节机械臂轨迹自动化测试项目。测试节点通过 `FollowJointTrajectory` Action 向 `arm_controller` 依次发送 10 组目标关节角，同时订阅 `/joint_states` 获取实际关节位置，自动计算误差、判断测试结果，并输出 CSV 报告和轨迹曲线。

当前版本面向 ROS 2 控制接口、测试流程和数据分析的学习与演示。配套机械臂项目使用 `mock_components/GenericSystem`，因此当前结果主要验证 ROS 2、MoveIt 2 和 ros2_control 的接口与流程，不代表真实机械臂的动力学性能。

### 已实现功能

- 通过 `/arm_controller/follow_joint_trajectory` 发送关节轨迹目标
- 顺序执行 10 组合法六关节目标
- 订阅 `/joint_states` 并按关节名称匹配数据
- 处理 Goal 接受、拒绝和最终 Result
- 检测执行超时并请求取消目标
- 计算每个关节的最终绝对误差
- 计算每组平均误差、最大误差和总体成功率
- 根据误差阈值自动输出 PASS / FAIL
- 保存测试汇总、最终关节结果和逐帧轨迹数据
- 自动生成每组测试的目标值与实际值曲线

### 系统数据流

```text
test_config.py
    │ 10组目标关节角
    ▼
trajectory_test_node.py
    │ FollowJointTrajectory Goal
    ▼
arm_controller
    │
    ├── Action Result ──────────────┐
    └── /joint_states ──────────────┤
                                    ▼
                         误差计算与PASS/FAIL
                                    │
                  ┌─────────────────┴─────────────────┐
                  ▼                                   ▼
              CSV报告                           PNG轨迹曲线
```

### 项目结构

```text
robot_arm_test_framework/
├── README.md
├── LICENSE
├── .gitignore
└── arm_test_runner/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    ├── test/
    └── arm_test_runner/
        ├── __init__.py
        ├── test_config.py
        ├── trajectory_test_node.py
        ├── csv_report.py
        └── plot_results.py
```

主要模块：

- `test_config.py`：关节名称、测试目标、误差阈值和时间参数
- `trajectory_test_node.py`：Action通信、状态订阅和测试流程
- `csv_report.py`：生成CSV报告
- `plot_results.py`：读取逐帧数据并生成PNG曲线

### 环境要求

- Ubuntu 24.04（本项目在 WSL2 中开发）
- ROS 2 Jazzy
- ros2_control
- `joint_trajectory_controller`
- Python 3
- Matplotlib
- 一个提供以下接口的机械臂系统：
  - Action：`/arm_controller/follow_joint_trajectory`
  - Topic：`/joint_states`
  - 关节：`joint1` 至 `joint6`

### 工作空间布局

测试仓库和机械臂仓库可以作为两个独立 Git 仓库，共享同一个 ROS 2 工作空间：

```text
~/ros2_ws/src/
├── six_axis_robotic_arm/
└── robot_arm_test_framework/
```

### 编译

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arm_test_runner --symlink-install
source install/setup.bash
```

### 运行

先启动配套机械臂系统：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_robot_bringup my_robot.launch.xml
```

然后在另一个终端运行测试：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run arm_test_runner trajectory_test_node
```

### 输出文件

每次运行会创建一个带时间戳的目录：

```text
~/ros2_ws/test_results/run_YYYYMMDD_HHMMSS/
├── test_summary.csv
├── joint_results.csv
├── trajectory_samples.csv
├── trajectory_test_01.png
├── ...
└── trajectory_test_10.png
```

- `test_summary.csv`：每组测试的通过状态、平均误差、最大误差和原因
- `joint_results.csv`：每组结束时每个关节的目标值、实际值和误差
- `trajectory_samples.csv`：测试期间逐帧记录的目标值和实际值
- `trajectory_test_XX.png`：六个关节的目标/实际位置曲线

已有CSV时，也可以单独重新生成图片：

```bash
ros2 run arm_test_runner plot_results \
  ~/ros2_ws/test_results/run_YYYYMMDD_HHMMSS
```

### 测试指标

当前版本使用以下指标：

```text
绝对误差 = |目标关节角 - 实际关节角|
```

- 单关节误差阈值默认是 `0.01 rad`
- 所有关节误差不超过阈值，且控制器返回成功时，该组测试为 PASS
- 总体成功率为通过组数除以总测试组数

### 修改测试参数

编辑：

```text
arm_test_runner/arm_test_runner/test_config.py
```

可以修改：

- `TEST_CASES`：测试目标
- `ERROR_TOLERANCE`：误差阈值
- `TRAJECTORY_DURATION_SECONDS`：轨迹时间
- `EXECUTION_TIMEOUT_SECONDS`：超时时间

修改目标前请先确认机器人真实关节限制。

### 当前限制

- 配套机械臂当前使用 Mock Hardware，不包含真实质量、摩擦、惯性和电机动力学
- 当前主要评价最终位置误差，不将曲线中的超调解释为真实控制器性能
- 尚未加入独立的超限目标、非法消息和主动取消测试套件
- 尚未提供一键启动机械臂和测试节点的 Launch 文件
- 本项目选择 CSV 和 PNG 作为测试产物，不集成 rosbag

## English

### Overview

This project is a ROS 2 Jazzy automated joint-trajectory test framework for a six-axis robot arm. It sends ten joint targets to `arm_controller` through the `FollowJointTrajectory` Action, subscribes to `/joint_states`, evaluates final position errors, and generates CSV reports and trajectory plots.

The current version is intended for learning and demonstrating ROS 2 control interfaces, automated test flow, and result analysis. The companion robot project uses `mock_components/GenericSystem`, so the results validate the ROS 2, MoveIt 2, and ros2_control integration rather than real robot dynamics.

### Features

- Sends goals to `/arm_controller/follow_joint_trajectory`
- Runs ten valid six-joint target poses sequentially
- Subscribes to `/joint_states` and matches positions by joint name
- Handles goal acceptance, rejection, and final results
- Detects execution timeouts and requests goal cancellation
- Calculates final absolute error for every joint
- Calculates per-test mean error, maximum error, and overall success rate
- Produces automatic PASS / FAIL decisions
- Saves summary, final-position, and time-series CSV files
- Generates target-versus-actual trajectory plots for every test

### Build

Place this repository and the robot repository in the same ROS 2 workspace:

```text
~/ros2_ws/src/
├── six_axis_robotic_arm/
└── robot_arm_test_framework/
```

Then build the test package:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arm_test_runner --symlink-install
source install/setup.bash
```

### Run

Start the companion robot system:

```bash
ros2 launch my_robot_bringup my_robot.launch.xml
```

Run the automated test in another terminal:

```bash
ros2 run arm_test_runner trajectory_test_node
```

### Results

Each run creates:

```text
~/ros2_ws/test_results/run_YYYYMMDD_HHMMSS/
├── test_summary.csv
├── joint_results.csv
├── trajectory_samples.csv
└── trajectory_test_XX.png
```

Use the plotting executable to regenerate plots from an existing run:

```bash
ros2 run arm_test_runner plot_results \
  ~/ros2_ws/test_results/run_YYYYMMDD_HHMMSS
```

### Configuration

Edit `arm_test_runner/arm_test_runner/test_config.py` to change target poses, tolerances, trajectory duration, or timeout settings. Always verify the robot's joint limits before changing targets.

### Limitations

- The companion robot currently uses mock hardware rather than a physics simulator or real hardware
- Final position errors are meaningful for interface testing, but the plots should not be treated as real control-dynamics measurements
- Dedicated out-of-range, malformed-goal, and active-cancellation test suites are not included yet
- A one-command Launch workflow is not included yet
- This project intentionally uses CSV and PNG artifacts instead of rosbag

## License

Apache License 2.0.
