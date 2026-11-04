# HW-P2：驱动、局域网与本地安全控制

[总览](README.md) · 前置：[HW-P1](phase-01-hardware.md) · 下一阶段：[HW-P3](phase-03-mapping-and-navigation.md)

状态：待执行。目标：先证明底盘、里程计和故障停车可信，再允许导航上车。

## 1. Ubuntu 与 ROS 安装

Pi 和边缘机均按[ROS 2 Jazzy 官方 Ubuntu deb 安装](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
配置 Universe、locale、ROS apt 签名与软件源。不要复制过期 apt key 或未审查的安装脚本。
在完成官方软件源配置后执行：

```bash
# [Pi]
sudo apt update
sudo apt install ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-hls-lfcd-lds-driver ros-jazzy-xacro libudev-dev \
  build-essential python3-colcon-common-extensions

# [边缘机]
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  python3-colcon-common-extensions
```

两台机器均创建独立 `~/turtlebot3_ws/src`。主方案以源码固定 TurtleBot3、消息、
SDK 和 LDS-02 驱动，避免依赖已经变化的 apt TurtleBot3 包：

```bash
# [两台机器] 新工作空间；目录已存在时先检查，勿重复 clone。
source /opt/ros/jazzy/setup.bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/ld08_driver.git
git -C DynamixelSDK checkout --detach 516ae8632c593d7c84afd2bdee2a2fac965305cc
git -C turtlebot3_msgs checkout --detach 9209e01b4708cf3ca5610b01dfc9586d3c7889d8
git -C turtlebot3 checkout --detach 0c0be84e3f5c3194fb2adea8426a58a96060eab5
git -C ld08_driver checkout --detach 00dba810a0fe5579b5062c71030c1e3c79a4c5f4
```

依赖解析前按官方 ROS 工具说明初始化 rosdep；若已初始化，不重复 `rosdep init`。
Pi 仅构建驱动闭包，边缘机仅构建本方案需要的导航/描述/遥控闭包，不安装 Gazebo：

```bash
# [Pi]
cd ~/turtlebot3_ws
rosdep update
rosdep install --from-paths src/turtlebot3/turtlebot3_bringup \
  src/turtlebot3/turtlebot3_node src/turtlebot3/turtlebot3_description \
  src/turtlebot3_msgs src/DynamixelSDK src/ld08_driver --ignore-src -r -y
colcon build --symlink-install --parallel-workers 1 \
  --packages-up-to turtlebot3_bringup ld08_driver

# [边缘机]
cd ~/turtlebot3_ws
rosdep update
rosdep install --from-paths src/turtlebot3/turtlebot3_navigation2 \
  src/turtlebot3/turtlebot3_description src/turtlebot3/turtlebot3_teleop \
  src/turtlebot3_msgs src/DynamixelSDK --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to turtlebot3_navigation2 turtlebot3_description turtlebot3_teleop
```

如出现缺少 transitive package，按错误补充同发行版依赖后重跑，保留错误。
不能用 `--continue-on-error` 把部分构建伪装成完成；不需要删除上游源码目录。
保存每台机器 `dpkg-query -W 'ros-jazzy-*'`、Git SHA、内核和 Ubuntu 小版本，
本设计固定了源码参考，不声称锁定了 apt 全部传递依赖。

## 2. 网络、时钟与串口

路由器固定租约示例：边缘机 `192.168.50.10`，Pi `192.168.50.20`。
若现有网段不同，记录实际值，不照抄示例静态地址。关闭 AP client isolation，
边缘机用网线；只在受控 LAN 放行 ROS DDS/音频服务，不公开端口、不开路由器公网映射。
时钟用同一 NTP 源；无公网时由边缘机提供 LAN 时间，目标偏差 <50 ms，记录实测。

每个 ROS 终端先执行：

```bash
# [两台机器]
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
```

Pi 串口优先记录 `/dev/serial/by-id/`。上游 `robot.launch.py` 支持 `usb_port`，
雷达端口却在 launch 内固定为 `/dev/ttyUSB0`。最终自研 bringup 必须显式传入
稳定设备路径，不能只在文档里写“用 by-id”却让 launch 继续读错串口。
缺少序列号时用确认过的 USB 物理路径/设备特征建立 udev 规则，验证重插后不串线。
仅向运行账号授予所需设备权限，不用全局 `chmod 777`。

OpenCR 固件按[官方刷机流程](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/)：
确认 `OPENCR_MODEL=burger`、实际串口、固件来源；保存下载包 SHA256。
`latest` 固件并非版本锁，不应每次启动自动刷入。刷机用稳定供电且轮子架空，
不把其他板型的二进制烧到 OpenCR。

## 3. 初步驱动检查：仅架空执行

上游命令：

```bash
# [Pi] OpenCR 和 LDS-02 已配置，轮子架空。
ros2 launch turtlebot3_bringup robot.launch.py \
  usb_port:=/dev/ttyACM0 use_sim_time:=false

# [边缘机] 新终端执行相同环境设置后。
ros2 node list --no-daemon
ros2 topic list -t
ros2 topic info /cmd_vel --verbose
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

现场确认 `/scan` 为 LaserScan，`/odom` 为 Odometry，最终 `/cmd_vel` 为 TwistStamped。
`/odom` 原点不是世界原点，`map` TF 还没有建立是正常的。
目标值：scan 连续频率 ≥5 Hz，odom ≥10 Hz，无持续 USB 重连，静止 60 秒里程计
漂移 <0.05 m；这些是工程验收门槛，低于门槛应诊断型号、QoS、串口和负载。

## 4. 待实现的软件前置：本地 safety_gate

这是后续独立 `SW-*` 实现要求，**当前没有可执行 `safety_gate` 包**。
完成它与自研 bringup 前，仅做架空诊断，不进行地面连续遥控。

唯一速度链路：

```text
MANUAL: /teleop/cmd_vel -----------+
                                 +-> Pi safety_gate -> /cmd_vel -> turtlebot3_node
AUTO: Nav2 -> smoother ->         |
collision_monitor -> /nav/cmd_vel-+
```

所有候选和最终速度统一 TwistStamped。Nav2 内部 remap 必须使 collision_monitor
最后输出到 `/nav/cmd_vel`；其余内部 `cmd_vel_nav`、`cmd_vel_smoothed` 保持一致。
仅 remap controller 而漏掉 recovery/behavior 的速度输出会绕过保护，必须检查全链路。
上游原始遥控命令会直接发布 `/cmd_vel`，最终模式应将其 remap 到 `/teleop/cmd_vel`。

实现契约：

| 项目 | 固定要求 |
| --- | --- |
| 部署 | Pi 本地进程，与云端/边缘机分离，不依赖 ASR/LLM 线程 |
| 输出 | 20 Hz；`/cmd_vel` 唯一发布者；每条消息新时间戳，frame `base_footprint` |
| 数值 | 仅 `linear.x`、`angular.z` 可非零；拒绝 NaN/Inf；限幅初始 0.10 m/s、0.40 rad/s |
| 超时 | 候选速度收到后 300 ms 无更新即输出零；用 monotonic 判断到达时间 |
| 防陈旧 | QoS depth 1、volatile；发送 stamp 年龄 >300 ms 或未来偏差 >100 ms 拒绝；不重新盖章后接受旧数据 |
| 模式 | DISARMED / MANUAL / AUTO / FAULT；同一时刻只有一个来源；禁止旧消息跨模式复用 |
| 激光 | AUTO 中 scan 到达年龄 >500 ms 或未有效覆盖监控区域即零速并 FAULT |
| 边缘心跳 | AUTO 中任务监督 5 Hz 心跳，超过 1 秒丢失即 FAULT；与速度超时独立 |
| 停止优先 | `/mission/stop` 使零速锁存，不等 Action cancel 完成；随后取消目标 |
| 恢复 | 故障/急停释放不自动回 AUTO；操作者现场复位并确认才能重新使能 |
| 退出 | 正常退出发送零速；崩溃仍由驱动/固件层兜底，不依赖 finally |
| 状态 | 发布结构化 fault code、mode、候选年龄、传感器年龄、限幅与输出；不得静默拒绝 |

已核对上游 teleop 在 Jazzy 使用 TwistStamped，但 frame_id 留空。
最终接入层只对已知 teleop 来源将空 frame 按差速本体命令解释，
保留原发送时间戳；其他未知 frame 不接受。不能把速度当作地图坐标变换。
模式控制与复位应是 Pi 上受认证的操作接口，携带 robot_id、mode、
单调递增序号和请求有效期；启动/急停/服务重启后 DISARMED，
不允许 LLM 或录音文本直接调用 AUTO 使能/故障复位。
任务监督心跳必须绑定本次启动的 session_id 与序号，不能由一个与任务管理无关的
空定时器持续伪造“健康”。

Nav2 的 Collision Monitor 是软件碰撞保护，不是合规安全扫描器。
Pi 本地 gate 不重做全局规划；保留它是因为边缘机崩溃/网络中断时，OpenCR 的
驱动心跳可能仍然在运行。上游 `turtlebot3_node` 源码持续写心跳，并没有在速度回调
中提供本方案的网络候选超时，不能把固件心跳误认为“任何上游失联都会停”。
上游 OpenCR 源码有 500 ms 心跳超时，但必须用实际刷入版本和断线实验核对。

## 5. 架空故障测试，然后低速落地

两人在场：一人操作，另一人在急停旁。先架空完成以下每种 3 次，不允许人用手挡轮：

| 故障注入 | 预期 |
| --- | --- |
| 停掉候选发布进程，Pi driver 仍运行 | gate 在 300 ms 后零速，不能继续保留旧速度 |
| 切断 Pi 与边缘机 LAN | 300 ms 候选超时先停车，心跳超时锁存；恢复网络不自动运动 |
| 停掉 gate | 验证车载驱动/固件最终停止；若 driver 心跳仍让车持续运动，必须补上驱动级命令超时 |
| 停掉底盘 driver / 拔通信 USB | 固件心跳保护触发；记录实测停止时间，不只看 ROS 日志 |
| 输出 NaN / 过限 / 旧时间戳 / 同时两来源 | 拒绝异常、限幅或保持唯一模式，不产生危险速度 |
| AUTO 中断激光数据 | ≤500 ms 进入 FAULT，恢复激光仍需人工复位 |
| 急停，随后释放 | 不依赖软件即可失去驱动力；不自行恢复任务 |

特别是 gate 崩溃而 driver 继续发心跳，可能不触发 OpenCR 看门狗。
必须在底盘驱动侧实现并测试“最终速度输入超时置零”，或使用经验证的等效固件策略；
**仅写一个转发节点不足以完成本阶段**。这是待实现 SW 前置，不能假设上游已有。
驱动级实现目标：收到最终速度后以本地 monotonic 更新时刻，
超过 300 ms 就在独立 wall timer 中主动向 OpenCR 写零速度；
正常更新周期 ≤50 ms，拒绝非有限值、陈旧输入。写串口失败必须报告 FAULT，
不继续输出健康状态。串口写与回调串行化，避免超时零速被旧回调覆盖。
驱动层故障恢复同样不能直接重放旧的非零目标。

架空测试通过后，在平整封闭场地以 0.05 m/s、0.2 rad/s 开始遥控，
无 Nav2 自动任务，测前进 1 m 和原地转一圈。
先测自然停止/急停后的最大滑行距离，再设置人员隔离区，禁止以真人测试碰撞。
上限目标：故障后停止时间 ≤1 秒、低速滑行 ≤0.15 m，重复 3 次；不达标就整改。
不将这些数值当成认证安全等级。

## 6. 通过条件与证据

硬件急停、唯一最终发布者、网络候选超时、gate 崩溃兜底、driver/USB 断线停车
全部通过才允许 AUTO。`/cmd_vel` 的 publisher 必须确认为 Pi gate，Nav2/遥控不得直连。

提交：网络拓扑/时钟偏差、设备路径、驱动与固件版本、TF/频率日志、所有故障视频、
1 m 直线的尺量与 odom 对比（误差 ≤10%）、转向方向与角度测试。
未通过项目写明失败，不进入 P3。
