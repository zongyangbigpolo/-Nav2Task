# HW-P3：真实建图、定位与 Nav2 路径规划

[总览](README.md) · 前置：[HW-P2](phase-02-base-and-safety.md) · 下一阶段：[HW-P4](phase-04-voice.md)

状态：待执行。目标：在自己的场地内完成真实导航，不套用第一阶段 sandbox 地图。

## 1. 场地准备与数据检查

选择约 3×4 m 或更大的单层区域，地面平整，通道宽度初始 ≥0.6 m，
用纸箱代替障碍物，人不进入行驶路径。屏蔽楼梯入口、低矮易碰撞物和镜面干扰区。
LDS-02 只有一个扫描平面，不能可靠识别悬空桌沿、低矮电线、玻璃和落差；
本方案不允许无人看护的自由巡游。

Pi 的**已实现且通过 P2 的最终 bringup**持续运行，包含稳定设备路径、
gate 与驱动侧超时保护。此自研 launch 不存在于当前仓库，不能用“上游原始
robot.launch.py 直接驱动”绕过它。边缘机确认：

```bash
# [边缘机] 每个新终端执行 P2 环境设置。
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

确认 `base_footprint -> base_link -> base_scan` 变换有效、方向正确，
所有实体节点 `use_sim_time=false`。无 `/clock` 不代表实体系统坏了。
相同 `odom -> base_footprint` 不能由两个节点同时发布。

## 2. 建图：手动遥控 + slam_toolbox

边缘机上执行现有上游命令：

```bash
# [边缘机 A] SLAM；此时不运行 AMCL 或第二个 map->odom 发布者。
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

# [边缘机 B] 仅在 gate 已切 MANUAL 后；核对遥控发布类型为 TwistStamped。
ros2 run turtlebot3_teleop teleop_keyboard \
  --ros-args -r cmd_vel:=/teleop/cmd_vel

# [边缘机 C]
rviz2
```

该上游键盘程序是“增减目标速度”，不是松键即停的 dead-man 遥控。
按空格/`s`、确认速度为零，再切 DISARMED 和退出；
已核对固定版本的 Jazzy `finally` 分支会发布保存的速度变量，
因此**不能依赖 Ctrl-C 必然发送零速**。
P2 的候选/驱动超时仍必须启用；最终手动控制器建议另实现显式 dead-man，
该键盘程序仅用于有急停人员在场的低速建图，不用于无人看护控制。

为 slam_toolbox 保存现场配置：`odom_frame=odom`、`map_frame=map`、
`base_frame=base_footprint`、`scan_topic=/scan`；若发行版默认不同，复制其默认配置
到自研配置目录再改，并以 `--show-args` 确认 launch 的 `slam_params_file` 参数。
不能拿 topic 拼错而没有数据的 SLAM 输出当作地图。

RViz Fixed Frame 设 `map`，添加 `/map`、`/scan`、TF 和 RobotModel。
以 ≤0.08 m/s 慢速绕场地，转角停顿采样，回到起点完成闭环。
重复观察：墙线不能重影、封闭边界合理、可通行路径已被观测。
不要拉着运行中的车移动，也不要把地图保存后继续改变场地障碍布局。

```bash
# [边缘机] 路径为本机实际路径。
mkdir -p ~/robot-data/maps/lab-v1
ros2 run nav2_map_server map_saver_cli \
  -f ~/robot-data/maps/lab-v1/map
```

预期生成 `map.yaml` 与其引用的图像文件，核对 `image` 相对路径、
resolution、origin 和 free/occupied thresholds。
地图第一次保存后还要重载检查，不能只看到文件名。
若需要以后续建图，另使用 slam_toolbox 的序列化功能保存 pose graph；
普通 `.yaml/.pgm` 栅格图不包含完整 SLAM 图优化状态。

先发零速并让 gate 回 DISARMED，再终止遥控、停止 SLAM，最后启动 AMCL 导航。
`map -> odom` 的唯一发布者从 SLAM 切换为 AMCL，不能两者同时发布。

## 3. 配置 Nav2：从 Burger 参数复制，不从仿真 Waffle 参数复制

在边缘机新建 `~/robot-data/config/burger-nav.yaml`，完整复制固定 Jazzy 分支的
Burger 参数作为基线。下面是**需要修改/核对的字段，不是完整参数文件**：

| 部分 | 基线要求 |
| --- | --- |
| AMCL | 差速模型，map/odom/base 与实机一致；初始定位由操作者按真实位置设置 |
| Planner `GridBased` | `nav2_navfn_planner::NavfnPlanner`；未知区域初始禁止规划 `allow_unknown=false` |
| Controller `FollowPath` | `dwb_core::DWBLocalPlanner`；不用第一阶段 Waffle MPPI 参数 |
| 初始速度 | `max_vel_x=0.10`、`max_speed_xy=0.10`，`max_vel_theta=0.40`，与 Pi gate 一致 |
| Goal checker | 位置容差 0.25 m、朝向容差 0.25 rad；DWB 相关阈值同步核对 |
| 实体时钟 | 所有节点 `use_sim_time=false` |
| footprint | 实测车体含音频支架/急停突出物；本体资料尺寸不能代替最终外形 |
| costmap | global=map，local=odom，LaserScan marking/clearing；初始 resolution 0.05 m |
| 速度格式 | controller/behavior/smoother/collision_monitor/gate/driver 都核对 stamped 设置 |
| smoother | max_velocity `[0.10,0.0,0.40]`；制动参数由低速测试确定，禁止透传仿真高加速度 |
| Collision Monitor | 输入链路正确，最终候选输出 `/nav/cmd_vel`；source_timeout 目标 0.5 s |

固定上游 Burger 样例里 `max_vel_x=0.3`，超过官方 Burger 最大平移速度 0.22 m/s；
故不能不经检查就把样例速度直接用于实机。现场降速和 gate 限幅必须两层都存在。
不能只改 controller 而漏掉 recovery 的旋转/后退速度。

膨胀半径、停止区、减速区由实测外形和 P2 制动距离决定：
从 footprint 外沿向外，停止余量 ≥最大已测停止距离 +0.10 m；
减速区再向外扩展约 0.15 m，现场以纸箱测量修正。
例如最终外形外接半径 0.15 m、低速最坏停止距离 0.10 m，则停止圆初值
≥0.35 m。示例不代表已测到这两个数值。
上游默认 stop 圆半径 0.1 m 不能被误认为覆盖所有新增外形。

安全策略：初期不启用未单独验证的自动后退/快速旋转恢复；
对恢复行为的关闭/替换同步修改 Behavior Tree，不能移除 plugin 后留着 BT 调用它。
无通路时允许失败和等待，不让模型或人工参数强行穿过障碍。
Collision Monitor min_points 和雷达盲区需用实际障碍物验证，不能当成必然挡住细杆。

## 4. 上游导航命令与最终速度链

仅在 P2 软件安全前置与本节完整参数已完成时执行：

```bash
# [边缘机] 先关闭 SLAM，使用真实地图。
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=false \
  map:=$HOME/robot-data/maps/lab-v1/map.yaml \
  params_file:=$HOME/robot-data/config/burger-nav.yaml
```

本命令会启动 Nav2 与 RViz，但最后输出 remap 必须已在完整配置/自研 launch
落到 `/nav/cmd_vel`，Pi gate 才能进入 AUTO。核对
`ros2 topic info /cmd_vel --verbose` 的最终发布者只有 gate，
`/nav/cmd_vel` 的候选来源是 Collision Monitor；
遥控退出且不直连最终 `/cmd_vel`。

先 DISARMED 完成 AMCL 初始化，再人工 AUTO 使能。
RViz “2D Pose Estimate” 用真实站位与方向，激光点应叠在墙体；
定位未就绪、地图版本错误或 TF 太旧时禁止运动。不要把初始化坐标当作瞬移。
首次 Nav2 Goal 选 1 m 左右的可见空地，旁边有人负责急停。

## 5. 待实现：命名地点与单任务管理

后续 `SW-*` 包必须提供地点表与任务控制；当前没有对应 `ros2 run` 可执行程序，
因此不编造一条“运行即可成功”的任务命令。

地点配置示例，**坐标必须由本场地现场采集，以下数值仅为格式示例**：

```yaml
schema_version: 1
map_id: lab-v1
frame: map
waypoints:
  start: {label: 起点, x: 0.0, y: 0.0, yaw: 0.0}
  desk: {label: 实验台, x: 1.2, y: 0.5, yaw: 1.57}
  door: {label: 门口等候点, x: 2.0, y: 1.0, yaw: 3.14}
```

在 RViz 采集目标 pose，记录 x/y/yaw；转换为 quaternion 时
`z=sin(yaw/2)`、`w=cos(yaw/2)`。目标不能落在占据区、未知区或膨胀禁行区域。
将地图与地点表绑定同一 map_id 与 SHA256；更换地图使旧地点表失效。

任务服务内部只发 `/navigate_to_pose`，白名单地点由本地查表转换，
不接受任意外部坐标。状态机：

```text
DISARMED -> READY -> PENDING_CONFIRMATION -> NAVIGATING
                  -> CANCELED / FAILED / SUCCEEDED -> READY
任何阶段遇到安全故障 -> FAULT -> 人工复位 -> DISARMED
```

一个活动目标；相同 command_id 重试不重复执行；新目标默认拒绝并提示先取消，
不默默抢占。到达由 Nav2 结果 +停稳后位姿判断，播报不能提前说“已经到达”。
取消先锁存 Pi 零速，再异步 cancel；超时 60 秒或不可达应失败，不能无限重试。
“回起点”只是到命名地点 `start`，不是自动充电、复位里程计或重新建图。

## 6. 验收表

先选择三个实测地点，每次包括重启后的人工定位和确认：

| 测试 | 重复次数 | 通过门槛 |
| --- | --- | --- |
| 重载地图并定位 | 3 次冷启动 | 激光叠图、TF 唯一、Nav2 active，未定位不运动 |
| A→B→C 的命名导航 | 各段 5 次，共 15 次 | ≥14 次成功，零碰撞；失败有具体结果 |
| 终点精度 | 每次导航 | 停稳后 TF 与尺量/地面标记都记录，误差 ≤0.25 m /0.25 rad |
| 纸箱挡路 | 3 次 | 有安全替代路则重新规划，否则停车/失败；不能碰撞 |
| 不可达或未知地点 | 各 3 次 | 不执行未知地点；不可达明确失败，不继续挤向障碍 |
| 导航时取消 | 3 次 | 零速锁存，Action 最终取消；无旧目标恢复 |
| 无公网 | 1 次完整路线 | 地图、Nav2、局域网不依赖云，仍能完成 |
| 断局域网 | 3 次 | 复测 P2 停车，恢复网络不自行继续 |

Action 的成功不是外部真值证明；实体精度必须用地面标记和测量补充，
不能仅依赖可能漂移的 odom/AMCL。这也不是厘米级定位系统。

证据：真实地图/地点表/完整参数、SHA256、RViz 截图、连续行驶视频、
`ros2 bag record` 的 `/scan /odom /tf /tf_static /plan /cmd_vel /nav/cmd_vel /amcl_pose`，
自研任务状态与 gate 状态，逐次成功/失败 CSV。bag 先确认存储空间和录制时长，
不要把无限录制当作验收。
