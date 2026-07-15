# CoDroid Description

双臂 ROS 2 控制接口详见 [docs/双臂控制接口.md](docs/双臂控制接口.md)。

CoDroid 人形机器人 URDF 模型描述包，包含**灵巧手（hand）** 和**夹爪（gripper）** 两种末端执行器变体。

## 目录结构

```
codroid_description/
├── CMakeLists.txt                  # ament_cmake 构建文件
├── package.xml                     # ROS2 Humble 包清单
├── launch/
│   ├── display_hand.launch.py      # 灵巧手版本启动文件
│   └── display_gripper.launch.py   # 夹爪版本启动文件
├── rviz/
│   ├── display_hand.rviz           # 灵巧手 RViz2 配置
│   └── display_gripper.rviz        # 夹爪 RViz2 配置
├── urdf/
│   ├── hand.urdf                   # 灵巧手 URDF 模型
│   └── gripper.urdf                # 夹爪 URDF 模型
└── meshes/                         # 网格文件（符号链接到源目录）
    ├── base_link.STL
    ├── link_arm_l_01.STL ~ link_arm_l_07.STL
    ├── link_arm_r_01.STL ~ link_arm_r_07.STL
    ├── link_head_yaw.STL
    ├── link_head_pitch.STL
    ├── hand/                       # 灵巧手网格 (left/right)
    └── gripper/                    # 夹爪网格 (left/right)
```

## 依赖

| 依赖 | 说明 |
|------|------|
| `robot_state_publisher` | 解析 URDF 并发布 TF |
| `joint_state_publisher_gui` | 图形化关节滑块控制 |
| `rviz2` | 3D 可视化 |
| `urdf` | URDF 解析库 |

## 模型概述

### 共同部分：机器人本体

| 组件 | 自由度 | 说明 |
|------|--------|------|
| `base_link` | — | 机器人基座（固定） |
| 左臂 (J_arm_l_01 ~ J_arm_l_07) | 7 DOF | revolute 关节 |
| 右臂 (J_arm_r_01 ~ J_arm_r_07) | 7 DOF | revolute 关节（镜像） |
| 头部 Yaw (J_head_yaw) | 1 DOF | 水平旋转，±1.53 rad |
| 头部 Pitch (J_head_pitch) | 1 DOF | 俯仰，±0.241 rad |

### 变体一：灵巧手 (hand.urdf)

| 端部 | 关节 | 说明 |
|------|------|------|
| 左手 (`left_base_link`) | fixed → link_arm_l_07 | 左手基座 |
| 右手 (`right_base_link`) | fixed → link_arm_r_07 | 右手基座 |
| 五指手指 | thumb/index/middle/ring/little | 各 2-4 个 revolute + mimic 关节 |
| 力传感器 | palm + 每指 3-4 个 | fixed 关节上的传感器链 |

### 变体二：夹爪 (gripper.urdf)

| 端部 | 关节 | 说明 |
|------|------|------|
| 左夹爪 (`left_gripper_link`) | fixed → link_arm_l_07 | 左夹爪基座 |
| 右夹爪 (`right_gripper_link`) | fixed → link_arm_r_07 | 右夹爪基座 |
| 夹指 1 (`left/right_gripper_finger_link1`) | prismatic | 平移 0 ~ 0.05 m |
| 夹指 2 (`left/right_gripper_finger_link2`) | prismatic | 平移 -0.05 ~ 0 m |
| RealSense 相机 | fixed | 固定在夹爪上 |
| 抓取帧 (`gripper_left/right_grasp_frame`) | fixed | 抓取参考坐标系 |

## 构建与安装

```bash
cd /home/ym/workspace/Human02
colcon build --packages-select codroid_description
source install/setup.bash
```

## 使用方法

### 启动灵巧手版本

```bash
ros2 launch codroid_description display_hand.launch.py
```

### 启动夹爪版本

```bash
ros2 launch codroid_description display_gripper.launch.py
```

### 连接真机双臂实时接口

该入口只适配左右双臂的 14 个关节，不控制头部、灵巧手或夹爪。控制器反馈的
`AP/AV/AT` 发布到 `/joint_states`，发送到 `/codroid/arm_command` 的
`sensor_msgs/msg/JointState` 会转换成文档规定的 UDP 关节位置指令。

```bash
ros2 launch codroid_description realtime_arms.launch.py robot_ip:=192.168.2.16
```

上电和取得控制权属于真实机器人危险操作，launch 不会自动执行。按现场安全流程依次调用：

```bash
ros2 service call /codroid_arm_bridge/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /codroid_arm_bridge/lock std_srvs/srv/SetBool "{data: true}"
ros2 service call /codroid_arm_bridge/left_control std_srvs/srv/SetBool "{data: true}"
ros2 service call /codroid_arm_bridge/right_control std_srvs/srv/SetBool "{data: true}"
```

关节命令可只包含完整的左臂或右臂，也可同时包含双臂；关节名必须与 URDF 一致。例如：

```bash
ros2 topic pub --once /codroid/arm_command sensor_msgs/msg/JointState \
  "{name: [J_arm_l_01, J_arm_l_02, J_arm_l_03, J_arm_l_04, J_arm_l_05, J_arm_l_06, J_arm_l_07], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

停止时先关闭左右臂控制，再释放控制权和下使能：

```bash
ros2 service call /codroid_arm_bridge/left_control std_srvs/srv/SetBool "{data: false}"
ros2 service call /codroid_arm_bridge/right_control std_srvs/srv/SetBool "{data: false}"
ros2 service call /codroid_arm_bridge/lock std_srvs/srv/SetBool "{data: false}"
ros2 service call /codroid_arm_bridge/enable std_srvs/srv/SetBool "{data: false}"
```

原始控制器状态同时发布到 `/codroid/status`（`std_msgs/msg/String`）。可用
`feedback_port`、`command_port`、`rviz` 和 `auto_connect` launch 参数覆盖默认值。

#### Pose 目标逆解与三次插值

左右臂分别接受 `geometry_msgs/msg/PoseStamped`：

- `/codroid/left_arm/pose_target`
- `/codroid/right_arm/pose_target`

节点把 ROS 四元数转换为控制器 IK 接口要求的 `[x,y,z,roll,pitch,yaw]`，通过
WebSocket 9000 请求机器人控制器逆解。逆解得到 7 个关节角后，从最新真实关节反馈开始，
使用 `3u²-2u³` 三次平滑插值，以 100 Hz 向 UDP 实时接口下发。

IK 后端通过 launch 参数选择：

- `ik_solver:=controller`：使用控制器 WebSocket IK，默认值。
- `ik_solver:=local`：使用本地 URDF 阻尼雅可比数值解。
- `ik_solver:=controller_then_local`：先调用控制器 IK，失败时自动使用本地数值解。

```bash
ros2 launch codroid_description realtime_arms.launch.py \
  robot_ip:=192.168.2.16 ik_solver:=controller_then_local
```

```bash
ros2 topic pub --once /codroid/right_arm/pose_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.30, y: -0.25, z: 0.20}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

轨迹状态发布到 `/codroid/trajectory_status`。取消当前插值：

```bash
ros2 service call /codroid_arm_trajectory/cancel std_srvs/srv/Trigger "{}"
```

#### 离散 Pose 路点规划与下发

左右臂分别接受 `geometry_msgs/msg/PoseArray`：

- `/codroid/left_arm/pose_waypoints`
- `/codroid/right_arm/pose_waypoints`

该接口使用最新实际关节角作为第一个 IK 初值，后续点连续使用上一点的关节解，随后
自动分段计时并进行关节空间三次 Hermite 拟合。整条轨迹通过关节限位和速度检查后，
才会以 100 Hz 下发；任意路点逆解失败都会拒绝整条路径。

```bash
ros2 topic pub --once /codroid/right_arm/pose_waypoints \
  geometry_msgs/msg/PoseArray \
  "{header: {frame_id: base_link}, poses: [
    {position: {x: 0.30, y: -0.30, z: 0.20}, orientation: {w: 1.0}},
    {position: {x: 0.32, y: -0.25, z: 0.23}, orientation: {w: 1.0}},
    {position: {x: 0.30, y: -0.20, z: 0.20}, orientation: {w: 1.0}}
  ]}"
```

所有 Pose 必须位于 `base_link` 坐标系。该接口使用本地数值 IK，不受单点 Pose 的
`ik_solver` 选择影响；它保证关节轨迹连续通过路点，但不保证路点之间的末端轨迹为直线。
完整上机顺序、状态说明和故障排查见[双臂控制接口文档](docs/双臂控制接口.md)。

### 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gui` | `true` | 是否启动 `joint_state_publisher_gui`（滑块控制） |
| `rviz` | `true` | 是否启动 RViz2 |

示例：

```bash
# 仅发布 robot_description，不启动 GUI 和 RViz
ros2 launch codroid_description display_hand.launch.py gui:=false rviz:=false

# 使用 headless joint_state_publisher（无 GUI）
ros2 launch codroid_description display_gripper.launch.py gui:=false
```

### 在 RViz 中查看

启动后，RViz 会显示：
- **RobotModel**：完整的 3D 机器人模型（灰色配色）
- **TF**：关键坐标系（`base_link`、末端执行器基座、抓取帧）
- **Grid**：地面参考网格

通过 `joint_state_publisher_gui` 窗口拖动滑块，可以实时控制各关节角度。

## 网格路径

原始 URDF 采用相对路径 `../meshes/...` 引用网格。启动文件在运行时自动将其替换为 `package://codroid_description/meshes/...`，确保 `robot_state_publisher` 能正确加载。网格文件通过符号链接指向源目录，避免重复存储。

## 坐标系

- `base_link` — 全局固定坐标系（机器人基座）
- `link_arm_l_07` / `link_arm_r_07` — 腕部坐标系（末端执行器挂载点）
- `left_base_link` / `right_base_link` — 灵巧手基座
- `left_gripper_link` / `right_gripper_link` — 夹爪基座
- `hand_left_grasp_frame` / `hand_right_grasp_frame` — 灵巧手抓取帧
- `gripper_left_grasp_frame` / `gripper_right_grasp_frame` — 夹爪抓取帧
