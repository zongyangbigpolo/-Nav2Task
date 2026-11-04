# 参考项目、版本与兼容性核实

[返回五阶段总览](README.md)

检索日期：2026-11-04。以下为当日实际读取的 GitHub 元数据、README、手册或源码，
不是仅按 star 数推荐。未实测硬件、模型性能或第三方项目部署。

## 1. 项目选型

| 项目 | 核实内容与借鉴范围 | 不照搬的边界 |
| --- | --- | --- |
| [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) | Jazzy 分支有实体 bringup、Burger 参数、Nav2 launch，官方成套软硬件路线 | 不混用仿真 Waffle 参数；原始 bringup 无本方案本地 gate |
| [ROBOTIS-GIT/OpenCR](https://github.com/ROBOTIS-GIT/OpenCR) | ROS2 固件有 500 ms heartbeat timeout，车载 driver 每 100 ms 写心跳 | 不等于网络速度失联保护；固件实物版本须测试 |
| [ROBOTIS-GIT/ld08_driver](https://github.com/ROBOTIS-GIT/ld08_driver) | Jazzy LDS-02 驱动 | LDS-01、LDS-03 使用不同驱动，不因 USB 串口相同就通用 |
| [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | Nav2、AMCL、costmap、Simple Commander、Collision Monitor | 使用发行版配套接口，不把 main 全部功能认为在 Jazzy 自动存在 |
| [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | 2D 建图、地图与 pose graph 管理 | 栅格图不等于序列化 SLAM 图；与 AMCL 不同时发布 map→odom |
| [k2-fsa/sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx) | 离线 ASR/TTS、Linux x64/ARM64 支持、中文 SenseVoice 与 VITS 示例 | 库许可证不覆盖所有模型；示例录音不是车载识别 |
| [ggml-org/whisper.cpp](https://github.com/ggml-org/whisper.cpp) | 备选 CPU ASR，MIT 代码许可证 | 本五阶段不同时维护第二语音栈，不假定 Pi 4 实时性能 |
| [aniskoubaa/rosgpt](https://github.com/aniskoubaa/rosgpt) | README 有自然语言→JSON→TurtleBot3 Nav2 parser 的结构 | 不采用任意速度/直接执行范式；API 元数据未识别 SPDX，未复制源码 |
| [RobotecAI/rai](https://github.com/RobotecAI/rai) | README 明列 Jazzy/Humble、ASR/TTS、机器人 agent 与 ROSbot XL 演示 | 参考职责拆分和受控工具，不引入完整重型多agent框架，也不将演示当作Burger认证 |
| [openai/openai-python](https://github.com/openai/openai-python) | README 给出 Python SDK、Responses 和 Chat Completions 用法 | 未执行API；兼容第三方端点需要各自验证，模型ID/额度后续选定 |

相比从自组电机+编码器+MCU开始，TurtleBot3 提供已知差速参数、雷达、驱动、
固件与导航入口，因此本项目将开发重点放在集成、安全、语音与复现。
没有选择 TurtleBot4/ROSbot 作为主方案，避免预算与另一套固件/厂商驱动依赖；
不代表这些平台能力不足。

## 2. 固定的源码参考

以下 SHA 由 GitHub API 当日查询取得；P2 的核心驱动按这些版本固定。
Jazzy 的 Nav2 与 slam_toolbox 主方案通过 apt 安装，部署时另记录 deb 实际版本，
不把下表当作所有运行依赖的完整 lock。

| 仓库 | 分支/版本 | 查询得到的 SHA |
| --- | --- | --- |
| ROBOTIS-GIT/turtlebot3 | jazzy | `0c0be84e3f5c3194fb2adea8426a58a96060eab5` |
| ROBOTIS-GIT/turtlebot3_msgs | jazzy | `9209e01b4708cf3ca5610b01dfc9586d3c7889d8` |
| ROBOTIS-GIT/DynamixelSDK | jazzy | `516ae8632c593d7c84afd2bdee2a2fac965305cc` |
| ROBOTIS-GIT/ld08_driver | jazzy | `00dba810a0fe5579b5062c71030c1e3c79a4c5f4` |
| ROBOTIS-GIT/coin_d4_driver | jazzy，仅参考LDS-03 | `79d183324729f0c1881f11ef431b6384c1aef957` |
| k2-fsa/sherpa-onnx | master 调研快照 | `8c092a193f7878d827a26068e18f587576e9888d` |
| RobotecAI/rai | main 调研快照 | `b018729833742c6f71e1e9f332ee2b435676fb4f` |
| aniskoubaa/rosgpt | main 调研快照 | `1e7d57b3bb5a7e30df6988cab97bd8a27fed7b44` |

语音安装基线另选当日 latest release **sherpa-onnx v2.0.1**，
不使用上表滚动 master 示例。固定 tag 后仍需保存实际 tag SHA、
pip freeze 与模型 hash，才能复现自己完成的版本。

## 3. 关键证据与链接

### 实体硬件与 Jazzy

- [官方 Quick Start](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)：
  包含 Ubuntu 24.04 / Jazzy PC 部分；顶部旧摘要尚未同步，不仅依据摘要。
- [官方 Pi Jazzy 章节源码](https://github.com/ROBOTIS-GIT/emanual/blob/master/_includes/en/platform/turtlebot3/quick_start/sbc_setup_jazzy.md)：
  Ubuntu Server 24.04、Jazzy、LDS 驱动与设备配置。
- [硬件规格](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)：
  Burger 官方最大平移速度 0.22 m/s、Pi 4、OpenCR、原厂电源与电池；
  不据此推定新增USB音频供电余量或完整急停方案。
- [官方导航手册](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/)：
  Jazzy 起 cmd_vel 使用 TwistStamped，AMCL 需真实初始定位。
- [固定 bringup launch](https://github.com/ROBOTIS-GIT/turtlebot3/blob/0c0be84e3f5c3194fb2adea8426a58a96060eab5/turtlebot3_bringup/launch/robot.launch.py)：
  雷达按 LDS_MODEL 选择，OpenCR 支持 usb_port，雷达 launch 传 `/dev/ttyUSB0`。
- [固定 Burger Nav2 参数](https://github.com/ROBOTIS-GIT/turtlebot3/blob/0c0be84e3f5c3194fb2adea8426a58a96060eab5/turtlebot3_navigation2/param/burger.yaml)：
  DWB、Navfn、stamped 配置；样例 max_vel_x=0.3 不是本方案可直接采用的实体限速。
- [OpenCR 固件源码](https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp)：
  `HEARTBEAT_TIMEOUT_MS=500`；实际刷入包要核对并实测。
- [车载 driver 源码](https://github.com/ROBOTIS-GIT/turtlebot3/blob/0c0be84e3f5c3194fb2adea8426a58a96060eab5/turtlebot3_node/src/turtlebot3.cpp)：
  100 ms driver 心跳与速度回调独立，必须补上上游命令失联保护。
- [固定版本键盘遥控](https://github.com/ROBOTIS-GIT/turtlebot3/blob/0c0be84e3f5c3194fb2adea8426a58a96060eab5/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py)：
  Jazzy 为 TwistStamped、空 frame_id；退出分支保留速度变量，不能靠 Ctrl-C 保证停止。

### 语音

- [sherpa-onnx 官方说明](https://k2-fsa.github.io/sherpa/onnx/index.html)：
  本地离线推理，支持 Linux/macOS/嵌入式，性能仍需目标机实测。
- [v2.0.1 SenseVoice 文件示例](https://github.com/k2-fsa/sherpa-onnx/blob/v2.0.1/python-api-examples/offline-sense-voice-ctc-decode-files.py)：
  使用 model.int8.onnx 与 tokens，默认测试模型附带 WAV。
- [v2.0.1 TTS 示例](https://github.com/k2-fsa/sherpa-onnx/blob/v2.0.1/python-api-examples/offline-tts.py)：
  P4 采用其中中文 VITS 模型配置，不复制示例长段文学文本。
- [PyPI v2.0.1 元数据](https://pypi.org/project/sherpa-onnx/2.0.1/)：
  当日已核对存在 CPython 3.12、Linux x86_64 wheel；未安装执行，不据此保证模型性能。
- [ASR 模型 Release](https://github.com/k2-fsa/sherpa-onnx/releases/tag/asr-models)：
  当日确认 `sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17.tar.bz2`
  资产存在，约163 MB；解压占用另算。
- [TTS 模型 Release](https://github.com/k2-fsa/sherpa-onnx/releases/tag/tts-models)：
  当日确认 `vits-icefall-zh-aishell3.tar.bz2` 存在，需单独检查模型授权。

## 4. 许可证与使用范围

GitHub 元数据识别 TurtleBot3、OpenCR、sherpa-onnx、RAI 为 Apache-2.0，
whisper.cpp 为 MIT。ROSGPT 元数据返回 license=null，**不等于允许任意复制**；
本次仅引用架构思想和公开链接，不导入其源码。
其他仓库/模型/音色/数据集授权实施前逐项核实，教学使用不自动豁免版权。
本次提交只新增原创实施文档，未复制上游大段实现，也未上传模型或硬件固件包。

部分旧 Nav2 网页路径检索返回 404，OpenAI 平台页面返回 403；
没有将无法读取的页面作为已验证内容。协议兼容与语音实时性能仍是后续实测门槛。
