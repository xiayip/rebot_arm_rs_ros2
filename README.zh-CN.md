# rebot_arm_rs_ros2

[English](README.md) | [简体中文](README.zh-CN.md)

面向 Seeed reBot Arm B601-RS 六轴机械臂的原生 ROS 2 Jazzy 控制栈。

本项目是 Seeed Studio 官方开源机械臂项目
[reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) 的 ROS 2 扩展。官方主仓库提供硬件设计、
BOM、装配资料和产品文档；本仓库是独立维护的 B601-RS ROS 2 集成，主要包含
`ros2_control`、RobStride 私有 CAN 通信、控制器配置、机器人描述和仿真启动文件。

驱动基于可复用的
[`shared_can_hardware`](https://github.com/xiayip/shared_can_hardware) 和 Linux SocketCAN，并由本仓库
提供 RobStride 协议插件。运行时不依赖 MotorBridge；开发和真机验证过程中，MotorBridge 仅作为独立的
协议参考实现使用。

## 主要功能

- 原生 `ros2_control` 硬件接口和标准 `JointTrajectoryController`。
- 基于 MIT 指令的轨迹位置、阻尼和阻抗控制链，并支持 Pinocchio 重力补偿。
- 面向带载机械臂的保守启动检查和预载保持初始化。
- 支持 mock hardware、RViz 和 Isaac Sim。
- 法兰可组合替换，并提供可选的 Seeed 官方平行夹爪模型。
- 官方平行夹爪使用标准 `ParallelGripperCommand` action。

## 当前支持范围

| 项目 | 当前目标 |
| --- | --- |
| 机械臂 | 六轴 reBot Arm B601-RS |
| 电机 | J1-J3：RobStride RS-06；J4-J6：RobStride RS-00 |
| ROS | Ubuntu 24.04 上的 ROS 2 Jazzy |
| 总线 | Linux SocketCAN，launch 默认使用 `can1` |
| 仿真 | ros2_control mock hardware 和 Isaac Sim |

除非另有说明，本仓库尚未验证其他 ROS 发行版、电机组合或固件版本。在不同机械臂上运行前，必须核对
电机 ID、关节方向、零位、限位、CAN 接口和控制器参数。

各包目前使用预发布版本号 `0.1.0`；在首个带 tag 的稳定版本发布前，配置和 API 仍可能调整。

## 包结构

| 包 | 用途 |
| --- | --- |
| `rebot_arm_rs` | 聚合包 |
| `rebot_arm_rs_description` | 六轴 URDF/Xacro、可选官方平行夹爪和 ros2_control 描述 |
| `rebot_robstride_protocol` | RobStride 私有 CAN 编解码器、机械臂及官方夹爪协议插件 |
| `rebot_arm_rs_bringup` | 控制器配置和 launch 文件 |

六轴机械臂的惯量和组合 mesh 来自 Seeed Studio 官方 RS 模型。固定的上游提交、校验和以及夹爪组合策略
记录在 [`UPSTREAM.md`](rebot_arm_rs_description/urdf/UPSTREAM.md) 中。

## 机器人描述的组合方式

可复用的 `rebot_arm_rs` Xacro 宏只描述六轴机械臂本体，并以稳定的 `link6` 输出坐标系结束。原装物理
法兰由独立的 `rebot_arm_rs_stock_flange` 宏描述，包含 visual、collision、质量、惯量、固定连接和
`tool0`。默认入口 `rebot_arm_rs.urdf.xacro` 会组合机械臂本体和原装法兰，因此默认模型仍具有完整的
原装外观和动力学参数。

下游机器人包也可以只引用机械臂本体，然后在 `link6` 上安装自定义法兰。这样可以在不改变 `joint6`、
TF 名称和 ros2_control 接口的情况下替换完整的法兰模型，也不会让本仓库反向依赖具体机器人项目。

`rebot_arm_rs_with_official_parallel_gripper.urdf.xacro` 会将同一机械臂本体、原装法兰和 Seeed 官方
平行夹爪组合起来。可用以下命令预览：

```bash
ros2 launch rebot_arm_rs_description view_robot.launch.py \
  description_file:=rebot_arm_rs_with_official_parallel_gripper.urdf.xacro
```

官方夹爪模型对外暴露一个归一化执行关节 `gripper_command_joint`：`0.0` 表示完全打开，`1.0` 表示完全
关闭。两个用于显示的指爪关节 mimic 该命令，因此 RViz、TF 和 ros2_control 使用同一状态。真机描述
将夹爪配置为电机 ID `7`；协议适配器将归一化行程映射到官方电机端点（`5.0 rad` 打开，`0.0 rad`
关闭），并复用机械臂驱动的 MIT 编解码、启动检查、反馈和退出行为。

在 mock 或真机启动时显式选择夹爪描述，夹爪控制器会自动加载并激活：

```bash
ros2 launch rebot_arm_rs_bringup mock.launch.py \
  description_file:=rebot_arm_rs_with_official_parallel_gripper.urdf.xacro

ros2 launch rebot_arm_rs_bringup real_world.launch.py \
  description_file:=rebot_arm_rs_with_official_parallel_gripper.urdf.xacro
```

默认仍是不带夹爪的原装法兰版本，因此不会改变现有 launch 行为和下游自定义法兰的组合方式。

官方夹爪使用 ros2_controllers 的
`parallel_gripper_action_controller/GripperActionController`，对外提供标准 ROS 2
`ParallelGripperCommand` action：

```bash
# 以 25% 的归一化速度完全打开。
ros2 action send_goal \
  /rebot_arm_rs_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: [gripper_command_joint], position: [0.0], velocity: [0.25]}}"

# 以 25% 的归一化速度完全关闭。
ros2 action send_goal \
  /rebot_arm_rs_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: [gripper_command_joint], position: [1.0], velocity: [0.25]}}"
```

速度的含义是 `[0, 1]` 每秒的归一化开合速率。当前 RS00 夹爪适配器不导出 effort command，因为在
没有单独标定夹持力模型的情况下，不能把归一化夹持力安全地等同于 MIT 前馈力矩。夹爪反馈由
`joint_state_broadcaster` 发布到常规 `/joint_states`；本控制器不会提供旧的
`/rebot_arm_rs_gripper/gripper_command` 或 `/rebot_arm_rs_gripper/joint_states` 兼容话题。由于没有
effort command interface，不应依赖 Jazzy 控制器 action 结果中的 `state.effort`，应使用
`reached_goal`、`stalled` 以及位置/速度反馈判断结果。

## 仓库依赖关系

源码有意拆分为三个单向依赖的仓库：

```text
rebot_arm_rs_ros2
├── zephyr_controllers       （通用 MIT 控制器栈）
└── shared_can_hardware      （共享 CAN ABI、总线管理和 SocketCAN 传输）
```

[`zephyr_controllers`](https://github.com/xiayip/zephyr_controllers) 和
[`shared_can_hardware`](https://github.com/xiayip/shared_can_hardware) 均不依赖本仓库，彼此之间也没有依赖。
它们通过标准 ros2_control command/state interface 连接，因此控制器层和硬件层都可以被其他机器人复用。

本仓库只维护 reBot Arm RS 相关的集成：

- `rebot_arm_rs_description`：机器人模型和 ros2_control 硬件描述。
- `rebot_robstride_protocol`：供 `shared_can_interface` 使用的 RobStride 协议插件。
- `rebot_arm_rs_bringup`：控制器实例、标定后的增益、重力/摩擦参数和 launch 文件。

聚合包 `rebot_arm_rs` 只依赖实际需要的 MIT 控制器子集，而不是整个 `zephyr_controllers` 元包，避免
引入与机械臂无关的底盘、手部、夹爪和诊断控制器。

创建源码工作区时，先克隆本仓库，再导入两个项目依赖：

```bash
mkdir -p ~/rebot_ws/src
cd ~/rebot_ws/src
git clone https://github.com/xiayip/rebot_arm_rs_ros2.git
cd ~/rebot_ws
vcs import src < src/rebot_arm_rs_ros2/rebot_arm_rs_jazzy.repos
```

[`rebot_arm_rs_jazzy.repos`](rebot_arm_rs_jazzy.repos) 记录了依赖仓库地址，当前跟随各仓库的 `main`
分支。制作发布版本时，建议改为兼容的固定 tag 或 commit，保证依赖可复现。

## 真机安全须知

该软件能够下发足以让机械臂突然运动的力矩。首次启动真机前：

1. 可靠托稳整条机械臂和末端负载，清空运动范围内的人、物体和线缆，并保证能够随时切断电源。
2. 对照实物检查关节零位、电机 ID、方向、限位、负载描述和 CAN 接口。
3. 关闭 MotorBridge、CAN 测试脚本以及所有可能向同一总线发帧的进程。同一条机械臂 CAN 总线只能有
   一个控制端。
4. 从保守的小幅轨迹开始；确认所有关节均已使能并稳定保持后，才能移除支撑。

正常 Ctrl-C 退出时，驱动有意不发送失能帧，电机会继续执行最后一条电机侧命令，避免机械臂立即失去
保持力。进程退出不等于安全断电。电源中断、电机故障或另行配置的 CAN 超时仍可能导致力矩消失；只要
掉力可能造成损坏，就必须使用独立的机械支撑。

## 控制器架构

真机控制通过链式控制器完成，轨迹不会直接进入电机协议：

```text
JointTrajectoryController
  -> MIT 位置适配器（增益 + 重力前馈）
  -> MIT 输出控制器
  -> ros2_control 硬件 -> SocketCAN -> RobStride 电机
```

| 模式 | 激活的上层控制器 | 用途 |
| --- | --- | --- |
| 位置轨迹 | `rebot_arm_rs_trajectory_controller` + `rebot_arm_rs_position_adapter` | 默认模式，通过 MIT 指令进行较硬的轨迹跟踪 |
| 阻尼 | `rebot_arm_rs_damping_controller` | 可反驱、带重力补偿的手动拖动 |
| 阻抗轨迹 | `rebot_arm_rs_impedance_trajectory_controller` + `rebot_arm_rs_impedance_controller` | 柔顺轨迹跟踪和受阻处理 |
| MIT 输出 | `rebot_arm_rs_mit_controller` | 三种模式共用的末级链式输出控制器，通常保持激活 |

位置轨迹模式仍然是 MIT 位置控制链，不是电机原生 CSP 位置环。`controllers.yaml` 中的增益和前馈是针对
已调试机械臂的起始参数；更换法兰、工具、负载、安装方向或电机固件后必须重新验证。

## 真机启动行为

- 通用硬件插件只打开一次 CAN 总线，并在激活时读取各电机的位置、`run_mode` 和滤波相电流。
- 初始位置命令会保留已有的负载预紧力。驱动读取滤波相电流 `iqf`，使用对应型号的力矩常数换算为输出
  力矩，再将有界的 `torque / mit_kp` 偏移加到测量位置，避免 ros2_control 启动时将带载保持力矩突然
  清零。
- 激活时始终尝试使能机械臂。驱动先发送保留预载的 MIT HOLD 探测命令：已处于 Motor 模式的电机不会
  被重复使能；停止状态的电机会连续收到使能帧和保持帧。已处于 MIT `run_mode` 的电机不会被重复切换
  模式。
- 对已经处于 MIT 模式的电机，共享 CAN 运行时会约每 20 ms 重复一次不带使能的探测，最长等待 3 s。
  已停止电机必须收到跨越至少 30 ms 的三次无故障状态反馈，并同时满足：位置跨度不超过 0.003 rad、
  温度跨度不超过 1 °C、绝对速度不超过 0.5 rad/s、最新反馈不超过 100 ms。若电机相对最初被支撑姿态
  移动超过 0.025 rad，本次激活失败，等待重新尝试。发送使能前，停止电机的保持目标和导出命令基线会
  更新为最新测量位置。热启动且已经处于 MIT 运行状态的电机保留原预载，只要求新的无故障 Motor 模式
  反馈，不重复使能，也不等待冷启动稳定时间。
- 以上逻辑是针对已观察到的 RS 固件启动竞争所做的保守保护，**不是固件文档定义的 ready 位，也不是
  力矩自检**。真实低温工作需要单独验证，不能直接绕过检查。非 MIT 模式转换仍使用原有流程，不属于
  这套 MIT 专用 readiness 检查范围。
- 需要同时构建 `shared_can_ros2` 和 `rebot_robstride_protocol` 才能使用该检查。readiness 超时会报告具体
  关节和原因，并在发送使能/失能帧之前失败。冷启动期间仍必须托稳整臂，因为这些检查不能证明机械臂已
  经具备承载负载的能力。
- 默认以三段链激活轨迹控制器：`JointTrajectoryController -> MIT position adapter -> MIT output
  controller`。传入 `activate_trajectory_controller:=false` 时，MIT 输出控制器保持安全 HOLD，轨迹控制器
  和位置适配器只加载、不激活。
- 每个关节导出完整 MIT 命令组：`position`、`velocity`、`kp`、`kd`、`t_ff`。MIT 输出控制器独占这些
  硬件接口并提供链式 reference interface。位置适配器把 JTC 的位置和速度 reference 与配置的增益及
  可选 Pinocchio 重力前馈组合成 MIT 命令。`position_kp` 和 `position_kd` 支持运行时调整，并通过实时
  安全方式传入控制循环。
- 默认真机配置使用 joint2-joint4 已标定的仿射重力补偿，`gravity_compensation.factor=1.0`，其余关节
  scale 为 0。重力模型来自 ros2_control 使用的同一份 `robot_description`。
- 控制器激活时保留上一组 MIT 命令的等效力矩，再通过限速和平滑过渡进入 `g(q)`，避免前馈阶跃。
  运行时 factor 限制为 `[0, 1]`，并在 `factor_transition_duration` 内平滑变化。
- 位置适配器按配置频率在 `~/mit_state` 发布 `control_msgs/msg/DynamicJointState`，包含位置/速度 reference、
  原始和缩放后的重力力矩、重力 factor、前馈力矩、P/D 反馈力矩和最终 MIT 命令力矩。
- 阻抗控制器通过电机侧 MIT `kp`/`kd` 提供刚度和阻尼，ROS 实时循环通过整形后的 `t_ff` 提供已标定
  重力和 URDF 软限位力矩。激活时将当前位置作为平衡点，并接收专用阻抗 JTC 的限速 reference。逐关节
  弹簧力矩限制通过减小电机平衡点误差来约束 `kp * (q_ref - q)`，不会把位置反馈环搬回 ROS 循环。
- 阻抗轨迹保护会监控跟踪误差和实测速度。如果某个关节在近似静止的情况下持续超过误差阈值达到配置
  时间，会取消当前阻抗 JTC goal；取消时以实测姿态建立零速度 HOLD。JTC 更宽的 path tolerance 是更大
  误差下的第二层保护。
- 运行时反馈只有一个来源：每个 operation-status 响应都会更新位置、速度、力矩、温度和故障状态。
  `0x7019` 只在激活时读取一次，用于建立安全初始保持位置，不会轮询，也不会覆盖运行时状态。
- 所有 MIT 命令都会限制在对应电机型号范围内；位置还会受到关节限位和最大单周期步长的约束。
- J1-J3 配置为 RS-06，J4-J6 配置为 RS-00，host ID 均为 `0xFD`。

MotorBridge Gateway 和本硬件插件不能同时控制同一条 CAN 总线。

## 构建与首次 mock 启动

```bash
source /opt/ros/jazzy/setup.bash
cd ~/rebot_ws
vcs import src < src/rebot_arm_rs_ros2/rebot_arm_rs_jazzy.repos
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-up-to rebot_arm_rs
source install/setup.bash

# 无需真机，在 RViz 中验证描述和控制器栈。
ros2 launch rebot_arm_rs_bringup mock.launch.py
```

请从 ROS 2 工作区根目录运行 `colcon`，不要在工作区的上层目录运行，以免包发现过程扫描无关的 Python
源码。

## 启动与操作

### 模型与仿真

```bash
# 只查看原装机械臂描述，不启动 ros2_control。
ros2 launch rebot_arm_rs_description view_robot.launch.py

# 打开仓库内的 Isaac 资产和 ros2_control 后端。
ros2 launch rebot_arm_rs_bringup isaac_sim.launch.py

# 无界面启动，用于 launch 和 bridge 验证。
ros2 launch rebot_arm_rs_bringup isaac_sim.launch.py headless:=true
```

Isaac launch 使用仿真专用位置轨迹控制器和 `topic_based_ros2_control/TopicBasedSystem`，不会修改真机
MIT 控制链。运行时会创建 Action Graph：发布 `/rebot_arm_rs/isaac_joint_states`、订阅
`/rebot_arm_rs/isaac_joint_commands`、驱动 articulation 并发布 `/clock`。传入
`start_ros2_control:=false` 可只打开 Isaac stage 和 Graph。显式的 URDF 到 USD 重新生成流程见
[`rebot_arm_rs_description/usd/README.md`](rebot_arm_rs_description/usd/README.md)。

仿真轨迹示例：

```bash
ros2 action send_goal \
  /rebot_arm_rs_isaac_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], \
  points: [{positions: [0.1, 0.2, 0.2, 0.0, 0.0, 0.0], \
  time_from_start: {sec: 3}}]}}"
```

### 真机

完成前述安全检查并配置、启用 SocketCAN 接口后启动：

```bash
# 停止状态的电机会被使能；Ctrl-C 保留最后一条电机侧保持命令。
ros2 launch rebot_arm_rs_bringup real_world.launch.py can_interface:=can1

# 可选：只加载轨迹控制器但不激活；硬件驱动仍会使能并 HOLD 当前姿态。
ros2 launch rebot_arm_rs_bringup real_world.launch.py \
  can_interface:=can1 activate_trajectory_controller:=false
```

运动前检查状态：

```bash
ros2 control list_controllers -c /controller_manager
ros2 topic echo --once /joint_states
```

确认实物方向和工作空间安全后，再发送小幅位置轨迹：

```bash
ros2 action send_goal \
  /rebot_arm_rs_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1], points: [{positions: [0.1], \
  time_from_start: {sec: 3, nanosec: 0}}]}}"
```

### 切换控制模式

`--strict` 会拒绝“停用一个当前并未激活的控制器”。切换前先运行 `ros2 control list_controllers`，只在
`--deactivate` 中填写当前 active 的源控制器。

从默认位置轨迹链切换到阻尼模式：

```bash
ros2 control switch_controllers \
  --deactivate rebot_arm_rs_trajectory_controller rebot_arm_rs_position_adapter \
  --activate rebot_arm_rs_damping_controller \
  --strict -c /controller_manager
```

从阻尼模式返回默认位置轨迹链：

```bash
ros2 control switch_controllers \
  --deactivate rebot_arm_rs_damping_controller \
  --activate rebot_arm_rs_position_adapter rebot_arm_rs_trajectory_controller \
  --strict -c /controller_manager
```

从默认位置轨迹链切换到柔顺阻抗轨迹链：

```bash
ros2 control switch_controllers \
  --deactivate rebot_arm_rs_trajectory_controller rebot_arm_rs_position_adapter \
  --activate rebot_arm_rs_impedance_controller \
             rebot_arm_rs_impedance_trajectory_controller \
  --strict -c /controller_manager
```

通过阻抗 JTC 发送轨迹：

```bash
ros2 action send_goal \
  /rebot_arm_rs_impedance_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint2], points: [{positions: [0.3], \
  time_from_start: {sec: 3, nanosec: 0}}]}}"
```

返回默认位置轨迹链：

```bash
ros2 control switch_controllers \
  --deactivate rebot_arm_rs_impedance_trajectory_controller \
               rebot_arm_rs_impedance_controller \
  --activate rebot_arm_rs_position_adapter rebot_arm_rs_trajectory_controller \
  --strict -c /controller_manager
```

### 诊断与运行时调参

```bash
# 查看位置适配器的实时 MIT 命令分解。
ros2 topic echo /rebot_arm_rs_position_adapter/mit_state

# 查看阻抗目标、限速后平衡点、MIT 增益和力矩分解。
ros2 topic echo /rebot_arm_rs_impedance_controller/state

# 在线修改位置模式增益。每个数组必须包含六个非负值。
# 仅在托稳机械臂并记录当前值之后操作。
ros2 param set /rebot_arm_rs_position_adapter \
  position_kp "[300.0, 300.0, 300.0, 80.0, 80.0, 80.0]"
ros2 param set /rebot_arm_rs_position_adapter \
  position_kd "[6.0, 12.0, 12.0, 5.0, 4.0, 4.0]"
```

## 上游与资料来源

- reBot Arm 硬件、BOM 和社区主仓库：
  [Seeed-Projects/reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm)
- B601-RS URDF 运动学和机械臂 mesh：
  [Seeed-Projects/reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2)
- Seeed 官方平行夹爪 URDF 和 mesh：
  [Seeed-Projects/reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm)
- RobStride 私有 CAN 协议行为：RobStride 手册和 MotorBridge 0.5.1

导入资产的精确版本和校验和记录在
[`rebot_arm_rs_description/urdf/UPSTREAM.md`](rebot_arm_rs_description/urdf/UPSTREAM.md)。

## 许可证

ROS 集成包在各自 `package.xml` 中声明为 `Apache-2.0`。导入的 Seeed 硬件描述资产保留其上游许可证和
来源信息，其中官方平行夹爪硬件资产使用 `CERN-OHL-W-2.0`。重新分发修改后的资产前，请阅读
[`rebot_arm_rs_description/LICENSES`](rebot_arm_rs_description/LICENSES) 和
[`UPSTREAM.md`](rebot_arm_rs_description/urdf/UPSTREAM.md)。

## 贡献

欢迎提交 issue 和 pull request。reBot 专用协议和描述修改应放在本仓库；可复用 CAN 传输修改应放在
[`shared_can_hardware`](https://github.com/xiayip/shared_can_hardware)；通用控制器修改应放在
[`zephyr_controllers`](https://github.com/xiayip/zephyr_controllers)。代码修改后请构建受影响的包并运行：

```bash
colcon test --packages-up-to rebot_arm_rs
colcon test-result --verbose
```

涉及真机的修改应同时提供：被测机械臂配置、固件版本、CAN 抓包或相关控制器遥测、精确测试命令，以及
测试是在 mock、仿真还是真机上完成。未实际执行的真机测试不得报告为成功。
