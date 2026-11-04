# HW-P4：中文离线语音与任务对话

[总览](README.md) · 前置：[HW-P3](phase-03-mapping-and-navigation.md) · 下一阶段：[HW-P5](phase-05-llm-and-delivery.md)

状态：待执行。目标：先把录音、识别、规则意图、确认、导航和播报全部接通，
**不用大模型也能完成固定对话**。开放领域问答放在 P5，不能提前宣称完成。

## 1. 输入方式与部署

第一版采用车上 USB HID 按键 **按住说话，松开结束**，最长 8 秒。
不先做远场唤醒、持续监听和全双工打断，这些会显著增加回声与误触发风险。
音频设备在车上，不能用边缘机桌面麦克风伪装成车载语音。

Pi 负责 PCM 采集/播放和 HID 事件；边缘机做 ASR/TTS、规则解析与任务管理。
电机控制/gate 与音频进程分离，语音阻塞不能拖住停车定时器。
按键触发新录音时允许打断当前播报；不通过扬声器播放来构造自动确认。

## 2. 先验证 Linux 音频，不先写 ROS 节点

```bash
# [Pi]
sudo apt update
sudo apt install alsa-utils ffmpeg
arecord -l
aplay -l
```

根据枚举结果选择设备名字，以下 `USB` 仅为 ALSA card ID 示例，实际可能不同：

```bash
# [Pi] 录 5 秒，设备必须支持格式；不支持时查看能力并显式转换。
arecord -D plughw:CARD=USB,DEV=0 -f S16_LE -r 16000 -c 1 \
  -d 5 /tmp/voice-check.wav
aplay -D plughw:CARD=USB,DEV=0 /tmp/voice-check.wav
```

验收自己说话可听清、左右/单声道无误、最大音量无削波、重复插拔设备编号变化后
仍能找到同一硬件。使用 `plughw` 允许 ALSA 格式转换；若录制实际格式不同，
显式转换为 16 kHz mono PCM，不只修改 WAV header。
将一段经同意的测试音频传到边缘机，先跑离线文件识别。

## 3. 上游 sherpa-onnx 离线文件基线

边缘机使用独立 Python venv，不向系统 ROS Python 环境强装依赖：

```bash
# [边缘机]
sudo apt install python3-venv libsndfile1
mkdir -p ~/robot-speech
cd ~/robot-speech
python3 -m venv .venv
source .venv/bin/activate
python -m pip install sherpa-onnx==2.0.1 soundfile
git clone --branch v2.0.1 --depth 1 https://github.com/k2-fsa/sherpa-onnx.git upstream
curl -fLO https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17.tar.bz2
tar -xjf sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17.tar.bz2
python upstream/python-api-examples/offline-sense-voice-ctc-decode-files.py
```

上游示例默认识别模型附带 `test_wavs/zh.wav`，**不是自动使用 Pi 录音**。
第二步用同一 API `OfflineRecognizer.from_sense_voice()`，
以 `soundfile.read(..., dtype="float32")` 读取 Pi 文件并通过
`stream.accept_waveform(sample_rate, samples)` 解码。
保留真实 sample_rate、归一化和声道转换，不能将模型示例输出算作实机识别结果。
SenseVoice 输出可能带语言/情感标签，解析时移除已知标签，拒绝空识别，
不要将标签作为导航关键词。

中文 TTS 基线使用上游提供的 Aishell3 VITS 模型：

```bash
# [边缘机] 同一 robot-speech 目录与 venv。
curl -fLO https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/vits-icefall-zh-aishell3.tar.bz2
tar -xjf vits-icefall-zh-aishell3.tar.bz2
python upstream/python-api-examples/offline-tts.py \
  --vits-model=./vits-icefall-zh-aishell3/model.onnx \
  --vits-lexicon=./vits-icefall-zh-aishell3/lexicon.txt \
  --vits-tokens=./vits-icefall-zh-aishell3/tokens.txt \
  --tts-rule-fsts=./vits-icefall-zh-aishell3/phone.fst,./vits-icefall-zh-aishell3/date.fst,./vits-icefall-zh-aishell3/number.fst \
  --sid=21 --num-threads=2 --output-filename=./reply.wav \
  "已到达实验台。"
```

先在边缘机检查 reply.wav，再传回 Pi 用 `aplay` 播放。
合成 WAV 采样率由模型输出决定，播放网关读取 WAV 信息后播放/重采样，
不能假设一定是 16 kHz。
记录 pip freeze、示例 Git SHA、模型压缩包/解压文件 SHA256；
下载后按模型卡核对模型/数据许可证，库的 Apache-2.0 不等于模型许可证。
主方案为教学实验；未明确模型授权前不声称可商业再分发，模型不上传本仓库。

以上命令来自已核对的上游文件，**本机未安装/测试这些语音模型**。
如果 v2.0.1 wheel 或模型依赖在目标机不可用，记录错误后按对应版本源码说明构建，
不能在报告中补写不存在的成功率。

## 4. 待实现：音频网关与规则任务对话

为独立 `SW-*` 任务定义以下局域网接口。当前无运行服务，URL 是待实现契约：

| 接口 | 行为 |
| --- | --- |
| Pi→边缘 `POST /v1/speech/recognize` | multipart WAV + request_id；mono、16 kHz、≤8 s、≤300 KB |
| 返回 | `{request_id,text,language,processing_ms,status}`；空音频、解码失败显式错误 |
| 边缘→Pi 播放 | Pi 已建立的受认证连接下发 reply_id + WAV；确认接收/播放完成，可被 PTT 打断 |
| HID 事件 | 本地按下/释放启动与停止采集；异常断开结束录音并提示，不持续录到磁盘 |
| 传输 | 受控 LAN +认证，合理超时、限制文件大小；远程访问需 TLS/VPN，不开放公网 |

先用文件 + SSH 手工链路验证模型，再实现受认证 HTTP/WebSocket 网关，
不要把每句语音都用 shell 拼接 scp/ssh 命令来执行。
ASR/TTS 用 CPU，小主机最多两个推理线程起步；导航并行时测延迟，
延迟太大先做进程资源隔离或升级边缘机，不挤占 gate。

规则对话只支持：

| 输入 | 规则行为 |
| --- | --- |
| “去实验台”“到门口等候点” | 按受控别名查地点表，产生待确认任务，不立即行驶 |
| “确认”“取消” | 必须绑定当前 pending_id，确认未过期且 mode READY，才执行/取消 |
| “停止”“停下” | 高优先级本地规则，触发 P2 的 stop 路径，不调用大模型 |
| “你在哪里”“导航到哪了” | 查询本地任务/地图状态，不编造位置 |
| “还有多少电” | 读 `/battery_state`；字段未知/无 percentage 就说无法获得百分比 |
| 其他闲聊 | 固定提示“当前仅支持导航、状态与取消；自由对话将在下一阶段启用” |

“去实验台”→“将前往实验台，请确认”→**新一次按键说“确认”**→导航。
pending_id 由程序生成，10 秒过期；无确认/识别模糊/未知地点不移动。
重说去别处作废旧 pending，不允许迟到的“确认”执行前一任务。
ASR 通常不提供可比较的统一置信度，不能编造一个 `confidence>0.8` 阈值；
用地点唯一匹配、复述和显式确认限制误识别。

TTS 播放期间不自行录音，PTT 可以打断播报后开始录音。
语音“停止”依赖识别与网络，仅作为交互取消；危险情况下仍按硬件急停。
“停止”高优先级必须能打断正在等待的 ASR/任务队列，不能排在长回复后面。
确认语义只由当前用户 PTT 请求产生，不从 LLM 内容或扬声器回声生成。

## 5. 性能与验收

小主机性能目标：5 秒以内语音，释放按键后至 ASR 文本 p95 ≤3 秒；
固定回复首次出声 p95 ≤2 秒。这是待测目标，不保证 N100 在任意模型下达到。
先做 30 条录音语料（不含隐私），记录正确文本、意图与误差；不能只选成功样本。

| 测试 | 通过条件 |
| --- | --- |
| 静止环境固定指令 30 条 | 意图与地点正确 ≥27/30；错误不得触发错误地点行驶 |
| 电机低速运行时 20 条 | 明确记录噪声影响；若未达 90%，退回停车说话模式，不硬宣称远场可用 |
| 确认/过期/未知地点/模糊地点 | 各 5 次，全部零误动作 |
| 扬声器播报“确认”/“去实验台” | 不产生自身导航任务 |
| 车上录音、边缘 ASR、车上播报 | 连续 10 次通过；不是桌面音频替代 |
| 已定位条件下语音目标 | 5 次到目标、Action 成功且车上播报，保留视频与任务 trace |
| 取消、服务挂掉、音频设备拔出 | 明确提示，无新动作；已有任务依安全/取消策略处理 |
| 断公网但保留 LAN | 固定语音、TTS 和导航完整通过，不访问云模型 |

提交：音频设备/格式配置、模型与示例版本、匿名测试语料与统计、
每轮 request_id/pending_id/command_id、真实导航结果、时延分位数、
回声/断网/确认超时视频。测试录音上传前须获同意。
