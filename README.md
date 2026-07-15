# human_ros2

CoDroid 人形机器人 ROS 2 Humble 描述与双臂控制包。

## 功能

- 双 7 自由度机械臂、头部、灵巧手和夹爪 URDF/RViz 模型
- 机器人 UDP 实时接口与 ROS 2 `JointState` 桥接
- Pose 目标到 7 轴关节角的逆运动学
- 控制器 IK、本地阻尼雅可比 IK 和自动回退模式
- 从实际关节位置开始的三次平滑插值
- 离散 Pose 路点的连续本地 IK 与三次 Hermite 轨迹拟合
- 100 Hz 双臂关节位置下发
- 不连接真机的 RViz 右臂挥动演示
- RViz 内嵌双臂控制面板（使能、控制权、实时控制和 Pose 目标）

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

## 构建

```bash
git clone https://github.com/Guybod/human_ros2.git
cd human_ros2

colcon build --base-paths codroid_description \
  --packages-select codroid_description --symlink-install

source install/setup.bash
```

## RViz 模型显示

```bash
# 仅双臂
ros2 launch codroid_description display_arms.launch.py

# 双臂与夹爪
ros2 launch codroid_description display_gripper.launch.py

# 双臂与灵巧手
ros2 launch codroid_description display_hand.launch.py
```

## 纯显示挥臂测试

以下命令不会连接或控制真机：

```bash
ros2 launch codroid_description wave_demo.launch.py
```

## 真机双臂接口

```bash
ros2 launch codroid_description realtime_arms.launch.py \
  robot_ip:=192.168.2.16 \
  ik_solver:=controller_then_local
```

RViz 会自动加载 `CoDroid Arm Control` 面板，可直接操作 UDP 连接、机器人使能、
控制权、左右臂实时控制、错误复位、轨迹取消和末端 Pose 目标。危险操作带二次确认。
模型默认通过 `world → base_link` 固定变换抬高 `1.5 m`，可用
`base_height:=<高度>` 覆盖；该变换只影响场景放置，不改变 `base_link` 下的 IK 输入。

启动不会自动上使能、获取控制权或开启运动控制。首次上机前请阅读接口文档和安全说明。

## 离散位置轨迹快速操作

下面以右臂为例。先确认机器人周围无人、急停可触达，再启动接口：

```bash
ros2 launch codroid_description realtime_arms.launch.py \
  robot_ip:=192.168.2.16 ik_solver:=controller_then_local
```

打开另一个已经执行 `source install/setup.bash` 的终端，确认关节反馈正常：

```bash
ros2 topic hz /joint_states
ros2 topic echo /codroid/trajectory_status
```

按照现场安全流程使能、获取控制权并打开右臂实时控制：

```bash
ros2 service call /codroid_arm_bridge/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /codroid_arm_bridge/lock std_srvs/srv/SetBool "{data: true}"
ros2 service call /codroid_arm_bridge/right_control std_srvs/srv/SetBool "{data: true}"
```

将多个离散末端位置一次发布到右臂路点接口：

```bash
ros2 topic pub --once /codroid/right_arm/pose_waypoints \
  geometry_msgs/msg/PoseArray \
  "{header: {frame_id: base_link}, poses: [
    {position: {x: 0.30, y: -0.30, z: 0.20}, orientation: {w: 1.0}},
    {position: {x: 0.32, y: -0.25, z: 0.23}, orientation: {w: 1.0}},
    {position: {x: 0.30, y: -0.20, z: 0.20}, orientation: {w: 1.0}}
  ]}"
```

该接口始终使用本地数值 IK：以真实关节角为第一点初值，后续点使用上一点的解，
再进行关节空间三次 Hermite 拟合、限位和速度检查，最后以 100 Hz 下发。所有 Pose
必须位于 `base_link` 坐标系；任一点无解时整条轨迹都会拒绝。离散点之间的末端轨迹
不保证为直线。

需要中止时先取消轨迹，再关闭右臂控制：

```bash
ros2 service call /codroid_arm_trajectory/cancel std_srvs/srv/Trigger "{}"
ros2 service call /codroid_arm_bridge/right_control std_srvs/srv/SetBool "{data: false}"
ros2 service call /codroid_arm_bridge/lock std_srvs/srv/SetBool "{data: false}"
ros2 service call /codroid_arm_bridge/enable std_srvs/srv/SetBool "{data: false}"
```

详细状态、参数、左右臂接口及故障排查见[双臂控制接口文档](codroid_description/docs/双臂控制接口.md)。

## 文档

- [ROS 包说明](codroid_description/README.md)
- [双臂控制接口文档](codroid_description/docs/双臂控制接口.md)

## 目录

```text
human_ros2/
├── codroid_description/    # ROS 2 包
├── gripper/                # 原始夹爪模型资源
├── hand/                   # 原始灵巧手模型资源
└── 双臂实时接口文档0526.pdf # 控制器原始接口文档
```

## 安全提示

真机控制前应确认关节反馈、模型方向和限位一致，使用保守速度与小位移测试，并保证急停可触达。
本地 IK 目前不包含自碰撞或环境碰撞检测。
