# Ubuntu 端 Codex：ROS2 遥测工程构建任务书

更新：2026-10-30。

请将本文件与 **《地面电脑_MAVLink遥测接收说明.md》一起交给 Ubuntu 端 Codex**。前者规定要实现的 ROS2 工程和验收方法，后者是飞控已经确定的发送协议。

## 1. 给 Ubuntu 端 Codex 的任务

请在 Ubuntu 上实现、构建并验证一个 ROS2 工程：监听本机 **UDP 14550**，解析飞控主动发送的 MAVLink v2 数据，按完整批次发布飞行器姿态、机体水平速度、离地高度和有效状态，使 ROS2 下游节点、绘图工具与 rosbag 可以使用。

当前提供的仓库是 STM32 固件工程，没有现成的 ROS2 包。本任务书默认新建地面端工作区 `~/fly_telemetry_ws`；如果 Ubuntu 上已经有用户指定的 ROS2 工作区或相关包，先阅读其 `AGENTS.md`、README 和接口，沿用现有结构与命名，避免创建重复工程或覆盖已有修改。

请直接完成代码、依赖配置、launch、参数文件、离线模拟器和必要测试。交付时报告实际执行的构建/测试命令及结果，区分模拟器验证与真实飞控收包验证。

第一版的交付范围是**只读遥测接入**。保持已确定的飞控协议，不要求飞控增加消息，不向飞控发送心跳、参数请求、频率请求、控制或解锁命令。收到的高度、速度已经在飞控端解算，地面端按协议接收使用。

## 2. 先检查 Ubuntu 环境

先确认实际系统、ROS2 发行版、Python 和现有工作区，不要把以下示例中的发行版当成机器的既定环境：

```bash
cat /etc/os-release
ls /opt/ros
printenv ROS_DISTRO
command -v ros2
command -v colcon
/usr/bin/python3 --version
```

使用实际安装的 ROS2 underlay。后面的示例以 `jazzy` 为例；机器安装其他发行版时替换为实际路径，不能在同一个 shell 中混用多个 ROS2 发行版。

```bash
# 示例：实际是 Humble 等版本时，替换此路径。
source /opt/ros/jazzy/setup.bash
```

优先适配机器已经可用的 ROS2，不在任务中自动升级整个 ROS 环境。若 ROS2 未安装，先根据 Ubuntu 版本确定对应官方安装方式，并将缺少的环境说明清楚。

## 3. 已确定的输入协议

必须以《地面电脑_MAVLink遥测接收说明.md》为依据，下面是实现所需的摘要：

| 项目 | 规定 |
|------|------|
| 监听地址 | 默认 `0.0.0.0:14550`，端口可参数化 |
| UDP 载荷 | 原始 MAVLink 二进制字节 |
| 协议 | MAVLink v2，`common` dialect |
| MAVLink 来源 | SYSID=1、COMPID=1，均可参数化过滤 |
| 发送频率 | 每 50ms 一批，完整输入时每个字段均为 20Hz |
| 心跳 | 没有 HEARTBEAT，不得调用 `wait_heartbeat()` 等待连接 |
| 每批内容 | ATTITUDE + vx_body + vy_body + height + telem_ok，共五帧 |
| 批次关联键 | SYSID、COMPID、`time_boot_ms`；不同 UDP 来源还需隔离 |
| 时间含义 | 飞控启动后的打包毫秒时间，不是 Unix 时间或同步采样时间 |

字段映射：

| MAVLink 消息 | 字段 / 名称 | ROS2 原始字段 | 单位 |
|--------------|-------------|--------------|------|
| `ATTITUDE` (30) | `roll`、`pitch`、`yaw` | 同名 | rad |
| `ATTITUDE` (30) | `rollspeed`、`pitchspeed`、`yawspeed` | 同名 | rad/s |
| `NAMED_VALUE_FLOAT` (251) | `name="vx_body"` 的 `value` | `vx_body` | m/s |
| `NAMED_VALUE_FLOAT` (251) | `name="vy_body"` 的 `value` | `vy_body` | m/s |
| `NAMED_VALUE_FLOAT` (251) | `name="height"` 的 `value` | `height` | m，向上为正 |
| `NAMED_VALUE_INT` (252) | `name="telem_ok"` 的 `value` | `source_valid_flags` | 位掩码 |

`telem_ok`：bit0=姿态/角速度有效，bit1=XY 速度有效，bit2=高度有效。7=全部有效，5=姿态和高度有效但速度无效。无效浮点量为 NaN，不能替换成 0。

高度已经计算 `distance × cos(roll) × cos(pitch)`，不要重复补偿。速度已经以 m/s 输出并做近似旋转补偿，不要再次乘距离、除积分时间或减角速度。

## 4. 工程结构

采用两个包：`fly_telemetry_interfaces` 定义完整遥测消息，`fly_telemetry_bridge` 实现 Python 接收节点。自定义接口使用 `ament_cmake`，Python 节点使用 `ament_python`；这是支持 Python 使用自定义消息的一种标准组织方式，参见 [ROS2 官方自定义接口教程源码](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.rst)。

```text
~/fly_telemetry_ws/
├── requirements.txt
├── docs/
│   ├── 地面电脑_MAVLink遥测接收说明.md
│   └── Ubuntu_Codex_ROS2工程构建说明.md
├── src/
│   ├── fly_telemetry_interfaces/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── msg/FlightTelemetry.msg
│   └── fly_telemetry_bridge/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/fly_telemetry_bridge
│       ├── fly_telemetry_bridge/
│       │   ├── __init__.py
│       │   ├── udp_receiver.py
│       │   ├── batch_assembler.py
│       │   ├── bridge_node.py
│       │   └── telemetry_simulator.py
│       ├── launch/telemetry.launch.py
│       ├── config/telemetry.yaml
│       ├── test/
│       └── README.md
├── .venv/                   # 使用系统 Python 的虚拟环境
├── build/                   # colcon 生成
├── install/                 # colcon 生成
└── log/                     # colcon 生成
```

纯 Python 的 UDP 接收/批次拼装代码与 ROS 节点分开，便于不启动 ROS 图时测试解析和超时行为。Python 包必须正确安装 resource marker、package.xml、launch 和 config，并通过 console_scripts 导出：

```text
telemetry_bridge = fly_telemetry_bridge.bridge_node:main
telemetry_simulator = fly_telemetry_bridge.telemetry_simulator:main
```

`setup.cfg` 中脚本安装目录应为 `$base/lib/fly_telemetry_bridge`，使 `ros2 run` 能找到节点。相关包结构要求见 [ROS2 官方 Python 包教程源码](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.rst)。

## 5. 消息和话题定义

### 5.1 `FlightTelemetry.msg`

第一版使用下列完整批次消息，数值与飞控字段保持同一单位和方向：

```text
# 飞控原始机体系：X前/Y右/Z下。height 单独约定向上为正。
# Euler 角沿用飞控自身姿态参考，不声称为绝对 ENU/NED 航向。
uint8 ATTITUDE_VALID=1
uint8 VELOCITY_VALID=2
uint8 HEIGHT_VALID=4

# stamp 为地面端完整批次接收时的 ROS 时间；不是飞控测量时间。
# frame_id 默认 base_link_frd。
std_msgs/Header header
uint32 time_boot_ms
uint8 sysid
uint8 compid

float32 roll
float32 pitch
float32 yaw
float32 rollspeed
float32 pitchspeed
float32 yawspeed
float32 vx_body
float32 vy_body
float32 height

# 固件 telem_ok 原始低三位。
uint8 source_valid_flags
# 经地面端有限数检查后的有效位，不能增加固件未给出的有效位。
uint8 valid_flags
```

接口包需声明 `ament_cmake`、`std_msgs`、`rosidl_default_generators`、`rosidl_default_runtime` 及 `rosidl_interface_packages` 组，并导出运行时依赖。`CMakeLists.txt` 至少包含：

```cmake
cmake_minimum_required(VERSION 3.8)
project(fly_telemetry_interfaces)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlightTelemetry.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

### 5.2 话题

| 话题 | 类型 | 发布时机 |
|------|------|----------|
| `/fly/telemetry` | `fly_telemetry_interfaces/msg/FlightTelemetry` | 每收到一份完整的新批次发布一次；正常约 20Hz |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 2Hz，并及时反映链路状态变化 |

完整批次即使 `telem_ok=0` 也应发布，因为“有数据但测量无效”与“链路断开”不同。链路超时后不要定时重复旧遥测冒充新采样；在 diagnostics 中报告断流，下游同时根据本地接收时间判断旧数据失效。

第一版发布者统一采用 **KEEP_LAST、depth=10、RELIABLE、VOLATILE**，便于终端、绘图和录包工具接入；下游需要匹配 QoS。DDS 的可靠传输不能补回进入桥接节点之前丢失的 UDP 数据。QoS 兼容性见 [ROS2 官方 QoS 文档源码](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)。

### 5.3 坐标和时间约束

原始话题使用 `base_link_frd`，明确 X前/Y右/Z下。ROS 常用机体系为 X前/Y左/Z上，参见 [REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)。因此不要把原始字段直接标记为标准 `base_link`。

本阶段只发布上述自定义完整数据，不自动生成 TF、全局 Pose 或 Odometry。当前输入没有水平位置、机体 Z 速度，也没有绝对北向参考。后续若增加 FLU 速度接口，应明确 `vx_flu=vx_body`、`vy_flu=-vy_body`，未测得的 Z 速度不能默认为有效的 0；姿态参考转换需要单独定义，不能仅凭名称把 Yaw 当作 ENU 航向。

ROS 消息角度保留 rad，显示层才转为度。`height` 始终向上为正，它是地形相对高度，不是机体系 Z 坐标分量。

`header.stamp` 使用节点 ROS 时钟记录该批在地面端完整接收的时间。使用 `time.monotonic()` 计算缓存寿命和链路超时，避免系统校时或 ROS 时钟暂停影响网络健康判断。飞控 `time_boot_ms` 单独保留，不能直接换成 Unix 时间。支持 32 位回绕和飞控重启，不得因时间戳变小而永久拒收；不要仅凭单个迟到包就判断飞控重启。

## 6. UDP 接收和拼批实现要求

建议使用 Python 标准库 `socket` 配合 pymavlink 的显式 v2 common 解析器：

```python
from pymavlink.dialects.v20 import common as mavlink2

# 每个 UDP 来源独立保留一个 parser，不能每个数据报重新创建。
parser = mavlink2.MAVLink(None)
parser.robust_parsing = True
messages = parser.parse_buffer(datagram) or []
```

显式导入 v2 dialect 的用法见 [MAVLink 官方 Pymavlink 文档](https://mavlink.io/en/mavgen_python/#choosing-the-dialectmavlink-version)。Codex 应根据实际安装版本核对解析 API，并跳过 `BAD_DATA`、处理解析异常，不要自行重写 MAVLink CRC。

实现细则：

1. 默认绑定 `0.0.0.0:14550`，将 socket 设为 non-blocking。在约 5ms 的 ROS timer 中排空已到数据；每轮设最大数据报数或运行时间预算，避免异常大流量饿死其他回调。
2. 按 UDP 源 IP/端口维护独立解析器，设置来源数量上限并清理闲置项；过滤 MAVLink SYSID/COMPID。不要用一个 parser 混合多个来源的半帧。
3. 一次 UDP 可包含多帧，也可只含半帧。持续保存解析状态，解析所有完整消息；流中丢字节或坏 CRC 后允许重新同步。
4. 对 `name` 去除末尾 NUL，兼容库返回字符串或 bytes，再精确比较。三个 ID=251 消息必须独立存储，不能互相覆盖。
5. 按 `(UDP来源, sysid, compid, time_boot_ms)` 收集五个组成部分；允许完整消息间乱序、忽略重复字段、忽略不相关消息。缺一个组成部分就不能发布为完整批次。
6. 每来源待配对批次最多 32 份，默认超过 250ms 清除，记录不完整批次计数。对已发布批次做有限容量去重，恢复/来源会话重建时清理旧缓存。
7. `source_valid_flags=telem_ok & 0x07`；`valid_flags` 从该值开始，经 `math.isfinite` 校验只清位。任一姿态或角速度值非法时清 bit0，并清依赖姿态的 bit1/2；任一 XY 速度非法时清 bit1；高度非法时清 bit2。无效组的输出浮点值统一置 NaN。
8. 完整批次接收即刷新链路时间，即使其测量全部无效；默认超过 500ms 无完整批次为链路超时，默认超过 100ms 无原始 UDP 数据可另作调试信息，不能混用为源传感器有效标记。
9. 链路正常、测量全有效时 diagnostics 为 OK；完整数据仍在到达但部分测量无效时为 WARN；从未收到或完整批次超时为 ERROR。输出 UDP 数据报数、解析消息数、坏帧数、不完整批次数、最后接收间隔和完整批次频率等统计。
10. 端口被占用时报告明确错误并退出；不要用端口复用掩盖两个接收进程争用。退出时关闭 socket，恢复收包不要求重启节点。

建议默认参数：

```yaml
telemetry_bridge:
  ros__parameters:
    bind_address: "0.0.0.0"
    udp_port: 14550
    target_sysid: 1
    target_compid: 1
    frame_id: "base_link_frd"
    receive_poll_ms: 5
    batch_timeout_ms: 250
    link_timeout_ms: 500
    max_pending_batches: 32
    max_sources: 8
    diagnostics_period_ms: 500
```

如果采用其他线程/队列结构，保持上述接口与行为，队列也必须有容量限制。此阶段参数可在启动时读取，修改监听端口时重启节点即可。

## 7. 安装依赖与构建

以下命令是 Ubuntu 端 Codex 应执行并核实的流程，当前 Windows 固件环境没有执行这些 ROS2 命令。

创建目录并把两份协议文档复制到 `docs/` 后，在 `src/` 实现前述两个包。构建工具和 ROS 依赖可通过 apt/rosdep 管理：

```bash
source /opt/ros/jazzy/setup.bash  # 按实际 ROS2 发行版替换
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-venv
mkdir -p ~/fly_telemetry_ws/src ~/fly_telemetry_ws/docs
cd ~/fly_telemetry_ws
```

为 pymavlink 创建使用系统 Python 的虚拟环境，使其可读取 apt 安装的 ROS2 Python 包；不使用与 ROS 二进制不匹配的 Conda Python。依据见 [ROS2 官方第三方 Python 包说明](https://github.com/ros2/ros2_documentation/blob/jazzy/source/How-To-Guides/Using-Python-Packages.rst)。

```bash
/usr/bin/python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE
source .venv/bin/activate
python3 -m pip install pymavlink pytest
python3 -c "import rclpy, pymavlink, colcon_core; print('Python dependencies OK')"
```

Codex 应把验证通过的 pymavlink 版本记录到 `requirements.txt`，并在 README 记下 Ubuntu、ROS2 和 Python 版本。Python 桥接包在 package.xml 声明 `rclpy`、`std_msgs`、`diagnostic_msgs`、`fly_telemetry_interfaces`、`launch`、`launch_ros`、`ament_index_python` 等实际用到的依赖及对应测试依赖。

仅首次、且 rosdep 尚未初始化时执行 `sudo rosdep init`。不要把已初始化的报错当作工程构建失败。

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro "$ROS_DISTRO"

# 显式用当前虚拟环境 Python 解释 colcon 入口，避免节点脚本被绑定到缺少 pymavlink 的 Python。
python3 "$(command -v colcon)" build --symlink-install \
  --packages-up-to fly_telemetry_bridge

source install/setup.bash
python3 -c "from fly_telemetry_interfaces.msg import FlightTelemetry; print('ROS interface OK')"
ros2 pkg executables fly_telemetry_bridge
```

`--packages-up-to` 会构建 bridge 及其工作区依赖；接口包必须被 bridge 的 package.xml 正确声明。若模块导入失败，检查当前 Python、console_scripts 的解释器路径、underlay/overlay 和虚拟环境，不用 `sudo pip` 或强行修改系统 Python 绕过问题。

## 8. 启动和查看数据

每个新终端按实际环境 source underlay、虚拟环境和 overlay：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/fly_telemetry_ws
source .venv/bin/activate
source install/setup.bash

# 实际 launch 必须加载包内 config/telemetry.yaml，并创建 telemetry_bridge 节点。
ros2 launch fly_telemetry_bridge telemetry.launch.py
```

检查端口与 ROS 话题：

```bash
ss -lunp | rg ':14550\b'
ros2 topic list -t
ros2 topic echo /fly/telemetry
ros2 topic hz /fly/telemetry
ros2 topic echo /diagnostics
```

若未安装 `rg`，用 `grep` 检查端口即可。有数据时完整遥测话题应接近 20Hz；无数据时节点仍应存活、diagnostics 应持续报告状态。检查 UDP 是否到达可以使用 `tcpdump -ni any udp port 14550`，但抓包需要相应权限，且不表示消息已经成功解码。

实际接收源为串口/网络转发设备：将其目的 IP 设为 Ubuntu 地面电脑可达 IP，目的 UDP 端口设为 14550。模拟器和实板不要同时使用相同 SYSID/COMPID 向同一个接收入口发送；离线测试使用 loopback 和测试端口隔离。

可用 `rqt_plot` 查看 `/fly/telemetry/roll`、`pitch`、`yaw`、`vx_body`、`vy_body`、`height` 的变化，具体命令格式按安装版本核实；角度曲线单位为 rad。录包：

```bash
ros2 bag record /fly/telemetry /diagnostics
```

## 9. 离线模拟器与测试要求

实现 `telemetry_simulator` 命令行程序，用 pymavlink v2 common 打包，通过 UDP 主动发送与飞控相同的五条消息，不发送心跳。默认目的地址为 `127.0.0.1:14550`、20Hz、SYSID=1/COMPID=1；提供 `--host`、`--port`、`--rate`、`--seconds` 和 `--scenario` 参数。

示例场景：

- `normal`：每批同一 time_boot_ms，姿态/速度/高度有已知数值或可预测变化，telem_ok=7。
- `invalid_flow`：姿态和高度有效，XY 为 NaN、telem_ok=5。
- `all_invalid`：数值全 NaN、telem_ok=0，但链路仍持续到达。
- `drop_height`：缺 height 帧，不得拼接上一批高度并发布完整批次。
- `fragmented`：把 MAVLink 帧拆到多个 UDP 数据报，并合并多帧；要求解析状态可持续。
- `corrupt`：注入坏 CRC 或随机字节后继续发正常帧，接收器能恢复。
- `wrap_restart`：覆盖 time_boot_ms 回绕及模拟飞控重启，不永久拒收后续数据。

测试应验证：

1. 正常五帧一批只发布一份 ROS 消息，单位/轴方向不被改变；不需要心跳即可收到。
2. 按 name 区分三个 251 消息；不同批次、不同 UDP 来源、不同 SYSID/COMPID 不串数据。
3. NaN、Infinity、有效位冲突时仅清除相应有效位，不把无效量变成 0。
4. 丢帧、重复帧、完整消息乱序、半帧和多帧数据报的处理符合拼批规则，缓存有界。
5. 网络断流后约 500ms 报超时，恢复数据后自动恢复；持续收到全无效完整批次时报告测量无效而非链路断开。
6. 本地计时不依赖电脑与飞控时钟同步；回绕和重启后仍能继续收数。
7. UDP 端口占用错误明确、退出释放端口；收数正常时日志不过量刷屏。

构建完成后执行：

```bash
python3 "$(command -v colcon)" test \
  --packages-select fly_telemetry_interfaces fly_telemetry_bridge
python3 "$(command -v colcon)" test-result --verbose
```

桥接节点保持运行，另一终端运行 10 秒正常模拟输入：

```bash
ros2 run fly_telemetry_bridge telemetry_simulator \
  --host 127.0.0.1 --port 14550 --rate 20 --seconds 10 --scenario normal
```

随后检查话题频率和字段，停止模拟器验证超时，再重新发送验证恢复。测试自动化可以改用空闲端口，避免与实板或其他程序抢占 14550。

## 10. Ubuntu 端 Codex 的最终交付

完成后提供：

- 两个 ROS2 包、消息定义、依赖文件、launch/YAML、模拟器和测试。
- README：实际环境版本、从空工作区到运行的命令、话题字段和坐标约定、常见故障排查。
- 构建及测试结果；模拟器下的实际接收/发布频率、断流与恢复结果。
- 实板数据若已到达，报告 UDP 来源、收到的消息类型/名称与有效位；未接通实板时清楚标注仅完成离线验证。

本文件是跨机器实施说明，不表示 ROS2 工程已经在当前 Windows 工作区生成或在 Ubuntu 上构建成功。Ubuntu 端 Codex 应依据本机实际情况完成实现与验证。
