# 2026-11-18 Go2 现场操作参考手册

编写日期：2026-11-17；适用日期：2026-11-18（周三）；代码核对基线：`6e26e52`。

> 本文按用户要求单独新增，供现场参考，不覆盖、不改写同学编写的 [PLAN.md](PLAN.md)，也不代表下列项目已经实现或通过验收。原计划中的“明天”是历史日期，请勿直接套用其 10/25 输出目录。
>
> 本手册列出的工作可以跨天完成。先获得正常机器狗的可靠数据，再扩展 WSL、双狗与无线；完成一层就保存成果。现场运动必须提前告知，并由用户启动；助手不得自行发送运动请求或启动会让狗运动的程序。

## 快速导航

- [1. 已知条件与明天目标](#1-已知条件与明天目标)
- [2. 分工、物品与时间](#2-分工物品与时间)
- [3. 到场登记和环境检查](#3-到场登记和环境检查)
- [4. 正常机器狗的静止基线](#4-正常机器狗的静止基线)
- [4.5 换狗后静止点云仍偏移（11/18 现场补充）](#45-换狗后静止点云仍偏移)
- [5. 测量与人工遥控建图](#5-测量与人工遥控建图)
- [6. 保存、回放与质量检查](#6-保存回放与质量检查)
- [7. WSL 接入对照与排障](#7-wsl-接入对照与排障)
- [8. 双狗双机的分层操作](#8-双狗双机的分层操作)
- [9. 无线连接的可选验证](#9-无线连接的可选验证)
- [10. 故障处理和停止条件](#10-故障处理和停止条件)
- [11. 收尾、备份与记录表](#11-收尾备份与记录表)
- [12. 证据、来源与命令限制](#12-证据来源与命令限制)

## 1. 已知条件与明天目标

### 1.1 本次与以前的区别

| 项目 | 已知情况 | 明天如何处理 |
|---|---|---|
| 旧机器狗 | 用户确认之前的狗有故障；具体故障部件和维修结论尚未提供 | 保留旧记录并注明设备故障背景；不把旧狗表现作为正常 Go2 的结论，也不直接认定全部异常都由同一故障造成 |
| 新机器狗 | 用户计划换正常的狗 | 重新登记编号、固件、电量和姿态，重新做静止基线 |
| 同学的电脑 | 原生 Ubuntu，之前能接入机器狗，明天仍可用 | 作为主要采集与建图设备 |
| 用户的电脑 | Windows 11；本次查到 WSL2 中已注册 Ubuntu-22.04，查询时处于停止状态 | ROS 2 安装和实机通信未检查；之前连接曾出问题，作为待验证设备 |
| 第二只正常机器狗 | 是否可同时到场尚未确认 | 一只狗也能完成主流程；双狗分支以实到设备为前提 |
| 现有软件 | 单狗时间对齐、扫描生成、SLAM、保存与回放工具已经存在 | 复用这些工具，先固定现有参数 |
| 双狗/导航/无线 | 没有当前项目的完整验收证据 | 不把设计方案写成现成功能，不临时复制单狗启动命令冒充双狗系统 |
| 数据 | 原始 rosbag 在 Elements；仓库只有源码、索引和小型证据 | 先检查外盘，再生成录制、地图和大体积诊断产物 |

10/25 的补录证据显示：静止 A 连续性通过，A2 有约 5.89 s 源缺口；里程计偏航变化较大，地图独立几何测量仍未完成。它们是旧设备记录，不是新狗的测量结果，详见 [实验结果](field_results_2026-10-25.md)。

### 1.2 优先级与当天可接受的成果

| 层级 | 内容 | 完成后可以得到什么 |
|---|---|---|
| P0 必做 | 身份/环境登记、正常狗短录制、十分钟静止 A、十分钟静止 B＋SLAM、正常保存 | 正常设备的输入基线和建图链路证据 |
| P1 优先 | 两段独立测量、用户遥控短路线、地图保存/重载、视频 | 一组可展示、可复现且有几何参照的单狗成果 |
| P2 可选 | 同一只狗在原生 Ubuntu 与 WSL 下顺序对照 | 判断 WSL 是否能成为第二台接入机 |
| P3 可选 | 两狗各自验收、隔离接入、独立录制、初始相对位姿登记 | 双狗数据准备；融合程序未完成时只保留可融合的数据 |
| P4 后续 | 无线接入、地图融合、导航、任务分配与探索 | 依赖新增实现与验收，不要求明天做完 |

完成 P0/P1 就有实质成果。没有完成 WSL、双狗或无线，不影响正常机器狗基线的价值。对尚未运行的项目填写“未做”，对证据不足的项目填写“待验证”。

## 2. 分工、物品与时间

### 2.1 人员与设备命名

- **操作者**：保管遥控器，确认场地和线缆，启动/停止实际运动。
- **记录员**：操作电脑、核对话题、录制、拍照、记录事件和文件路径。
- 测量时两人共同确认端点；运动时不要让操作者同时排查电脑网络。
- 本文称两台电脑为“原生 Ubuntu 机”和“Windows/WSL 机”；机器人记为 R1、R2。
- **静止阶段 A/B 是试验阶段，不是两只狗。** A＝仅采集；B＝采集＋服务器 SLAM。

### 2.2 出发前清单

- [ ] 正常机器狗、对应遥控器、充电器；确认可用电量和现场充电安排。
- [ ] 原生 Ubuntu 电脑及充电器；确认之前的 ROS 2 工作区可用。
- [ ] Windows 电脑及充电器；WSL 只作为可选分支。
- [ ] 已知可用网线，条件允许带一根备用线和 USB 有线网卡。
- [ ] Elements 外盘和连接线；另备独立存储设备用于备份。
- [ ] 卷尺、胶带、地面方向标记、手机/相机、纸质记录。
- [ ] 若测试双机联网：可管理的路由器、电源；先确认其允许局域网设备互通。
- [ ] 若测试完全无线：确认机载开发电脑/外接接入设备及供电，不凭“狗有 Wi-Fi”假定 DDS 已可用。

不要为明天临时同时升级机器人固件、ROS 发行版、SDK 和算法参数。必须修改时记录前后版本，并从短时静止检查重新开始。

### 2.3 时间建议

以下是预算，不是必须完成的时间表；始终保留最后 30–45 分钟收尾。

| 可用时间 | 建议完成范围 |
|---|---|
| 约 2 小时 | 登记、短录制、静止 A/B、实时快照与文件检查；有余量做现场测量，离线回放可后做 |
| 约 4 小时 | 上述内容＋两段测量＋一条用户遥控路线＋一次保存/重载；WSL只做短时排查 |
| 约 6–8 小时 | 在已有成果基础上做重复路线、WSL对照、第二狗独立验证；无线/融合视条件选做 |

建议主流程：接入 30 分钟 → 短录制 15 分钟 → A/B 各约 25–35 分钟 → 测量 20–30 分钟 → 遥控路线 30–60 分钟 → 保存/回放/备份。WSL 初次现场排查建议先限定 30–45 分钟；遇到环境重装、依赖大下载或持续无 DDS 数据，记录后回到已验证主流程。

## 3. 到场登记和环境检查

### 3.1 先登记，不混用新旧狗

建立一页现场记录，至少填写：日期、R1/R2 对应设备编号、型号、固件、可用开发电脑、开始电量、人员、场景、主机、网卡、代码提交、修改项、测试起止时间。

记录旧狗故障信息时区分“用户确认有故障”“维修方诊断”“本次实测”，不要补写未知部件。给新狗使用新的 `GO2_ROBOT_ID`；即使网络地址与旧狗相同，也不能沿用旧设备身份。

### 3.2 原生 Ubuntu 机：加载环境

以下 `bash` 命令用于**同学的原生 Ubuntu 机**。当前 Windows 的 `D:\SiZuJiQiRen` 只是源码副本，不能在 PowerShell 直接运行这些命令。

命令逐块执行，每块先检查结果；报错就停在当前步骤。下列路径来自已有项目记录，若同学实际用户名或工作区不同，先核对真实路径后统一替换，不创建假挂载或路径链接来绕过存储检查。

每个现场终端都先执行：

```bash
cd /home/yaorz/unitree_go2_ws
source /home/yaorz/unitree_ros2/setup.sh
source /home/yaorz/unitree_go2_ws/install/setup.bash

export GO2_SESSION_ID=2026-11-18_native
export GO2_ROBOT_ID=go2_replacement_01
export GO2_DATA_ROOT=/media/yaorz/Elements/unitree_go2
export GO2_RECORD_POWER_STATE=1
export SESSION_ROOT="$GO2_DATA_ROOT/experiments/${GO2_SESSION_ID}_${GO2_ROBOT_ID}"
```

`go2_replacement_01` 是本手册建议的登记标签，须在记录中对应实际设备编号；第二狗另设新标签。后续日期复用时修改日期。重试使用新的运行目录，不能覆盖上一轮。

核对版本和包：

```bash
git status --short --branch
git rev-parse HEAD
printenv ROS_DISTRO RMW_IMPLEMENTATION ROS_DOMAIN_ID
ros2 pkg prefix go2_slam_bringup
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_map_server
ros2 pkg prefix unitree_go
ros2 pkg executables go2_slam_bringup
```

当前手册按 `6e26e52` 的脚本核对。若 Ubuntu 机代码不同，先核对差异；本地有改动时不要 `reset --hard` 或覆盖同学文件。若安装空间没有包含当前源码，先在离线准备阶段构建并重新加载环境，避免“录制的源码快照”和“实际运行的旧安装包”不一致。现场不为采集而启动 `go2_sport_test`、`go2_safe_move` 或官方运动示例。

### 3.3 外盘检查：通过之后才能录制

```bash
lsblk -o NAME,LABEL,FSTYPE,SIZE,MOUNTPOINTS
findmnt --mountpoint /media/yaorz/Elements
df -h /media/yaorz/Elements
python3 scripts/check_external_storage.py --root "$GO2_DATA_ROOT" --probe
```

必须确认挂载的是实际 `Elements` 设备，并看到写入/刷新测试通过。脚本目前设有 5 GiB 最低剩余空间门槛；该值不是一天实验容量保证，仍要根据计划录制数量留余量。旧十分钟包约 0.85 GiB，仅可作粗略容量参考，不能预言新狗数据量。

没有挂载、只剩一个同名空目录、写入失败或空间不足时，停止大型输出。不要改到 Ubuntu 主目录、WSL 的虚拟硬盘或 Windows 内盘作为隐式替代。

检查通过后，创建本轮小型现场证据目录：

```bash
PRECHECK="$SESSION_ROOT/precheck_$(date +%H%M%S)_$$"
mkdir -p "$PRECHECK"
git rev-parse HEAD > "$PRECHECK/git_revision.txt"
git status --short > "$PRECHECK/git_status.txt"
git diff -- src scripts > "$PRECHECK/local_changes.patch"
ip -br link > "$PRECHECK/interfaces.txt"
ip -br addr > "$PRECHECK/addresses.txt"
ip route > "$PRECHECK/routes.txt"
```

手工补充设备编号、固件、操作者和场景；不把密码或登录令牌写进记录。

### 3.4 有线网络与话题

```bash
ip -br link
ip -br addr
ip route
printenv CYCLONEDDS_URI
ros2 topic list --spin-time 3 --no-daemon -t
ros2 topic info --spin-time 2 --no-daemon --verbose /utlidar/cloud_base
ros2 topic info --spin-time 2 --no-daemon --verbose /utlidar/robot_odom
```

确认连接的是当前登记的狗；网卡和 CycloneDDS 配置一致，地址与该狗实际网络匹配。不能把网上示例 IP 当成新狗实测 IP。`ping` 成功只证明部分网络可达，不等于 ROS/DDS 数据可用。

本仓库的录制入口要求：

- `/utlidar/cloud_base`：`sensor_msgs/msg/PointCloud2`；
- `/utlidar/robot_odom`：`nav_msgs/msg/Odometry`；
- `/sportmodestate` 或 `/lf/sportmodestate`：`unitree_go/msg/SportModeState`。

再核对 `/utlidar/imu`、`/utlidar/lidar_state` 和 `/lf/lowstate`。本手册默认记录 `/lf/lowstate`；若该话题无数据，记录缺失。选择不采电源数据时，A/B 都设 `GO2_RECORD_POWER_STATE=0`，并从观察命令移除 `--power-state-topic`，不把“订阅已建立”写成“数据已采到”。

用现有被动脚本看实际消息：

```bash
python3 scripts/inspect_go2_streams.py --seconds 30 --light \
  --output "$PRECHECK/streams_$(date +%H%M%S).json"
```

检查消息数、实际接收频率、点数、frame、里程计的 `child_frame_id`、时间回退和消息中的状态。状态错误码和电源原值先原样记录，不猜未经确认的厂商含义。

**当前脚本的适用条件：**

```bash
test -r /sys/class/net/enp2s0/carrier
cat /sys/class/net/enp2s0/carrier
```

`check_go2_continuity.py` 使用固定的 `enp2s0` 载波和计数器；`assess_continuity_report.py` 使用对应字段；`save_live_slam.py` 退出阶段也读取这一网卡。实际网卡不同就需要配套适配后再运行这些工具，不能通过创建假文件、随意改网卡名或假定 `--interface` 参数存在来绕过。网卡名相符但实际接狗的是另一个接口，同样不适用。

## 4. 正常机器狗的静止基线

### 4.1 短录制：先验证能开始、能结束、能读取

让狗保持固定姿态，周围尽量不走人，地面标记机身位置和朝向，拍一张起始照片。所有阶段保持同一场景和姿态；用户完成姿态调整后再开始计时。

终端 T1：

```bash
bash scripts/record_d1.sh smoke_native_01
```

脚本先做存储和话题检查，随后启动 `ros2 bag record`。**等待录制器实际进入录制状态之后**再计时；记录屏幕打印的完整 `Recording directory`。约 60–90 秒后手动 `Ctrl+C`，等待命令正常退出及 `bag/metadata.yaml` 生成。

本手册不使用从脚本启动就开始倒计时的 `timeout 660s`：发现话题和准备信息会占用时间，容易造成真正录制段不足。录制时不打开 SQLite，也不运行摘要、数据库检查或回放。

关闭后，在 T1 设置刚才打印的目录。下面故意使用占位路径，必须替换为本轮实际目录，不用“最新目录”自动猜：

```bash
RUN_DIR='/REPLACE_WITH_ACTUAL_RUN_DIRECTORY'
ros2 bag info "$RUN_DIR/bag"
python3 scripts/summarize_motion_bag.py "$RUN_DIR"
```

如果 `metadata.yaml` 缺失或关闭异常，保留原始文件，登记“不完整”，另建一次运行；不要现场对唯一原始包执行修复或删除 `.db3-wal` 等文件。

### 4.2 静止 A：只采集，不运行服务器 SLAM

先正常关闭自己启动的 SLAM 进程，确认没有遗留运动程序；不要用宽泛的 `pkill` 清理他人的进程。已存在的原厂 TF 或内部服务只做记录，不随意关闭。

T1 开始录制：

```bash
bash scripts/record_d1.sh static_A_native
```

等录制已真正开始后，在 T2 开始 620 秒观察（T2 必须也执行 §3.2 的环境块）：

```bash
OBS_DIR="$SESSION_ROOT/continuity_A_$(date +%H%M%S)_$$"
python3 scripts/check_go2_continuity.py --seconds 620 --phase static_A_native \
  --power-state-topic /lf/lowstate --output "$OBS_DIR"
```

观察程序结束并保存 `report.json` 后，让录制再保持约 10 秒，在 T1 按 `Ctrl+C` 正常停止。若链路、电量或机器人异常则提前结束，按不完整窗口记录，不为了凑时长维持异常状态。

T2 评估自己的观察目录：

```bash
python3 scripts/assess_continuity_report.py "$OBS_DIR"
```

T1 按 §4.1 用本轮真实 `RUN_DIR` 执行包信息与运动摘要。填好 `run_metadata.yaml` 中的 `operator`、`observer`、`scene`、`initial_pose_reference`、`robot_firmware`；未知时间戳语义保留待确认。记录 `recording_completed_normally: true/false`，异常中止不能填 true。

### 4.3 静止 B：采集＋服务器 SLAM

保持相同设备、场景、姿态和采集选项。T1 启动新录制：

```bash
bash scripts/record_d1.sh static_B_native_slam
```

等录制开始后，在 T3 启动单狗建图：

```bash
ros2 launch go2_slam_bringup go2_slam.launch.py
```

确认启动无持续报错，检查输出：

```bash
ros2 topic info /scan
ros2 topic info /map
timeout 10s ros2 run tf2_ros tf2_echo map base_link
```

上面的 `timeout` 只限制只读 TF 观察，不是录制计时器；到时退出状态可能是 124。点云和里程计的时钟偏移由时间桥每轮重新估计，不复用旧狗的固定偏移。

静止时 `/scan` 有发布门限和约 10 秒刷新机制，不能要求静止扫描始终高频；应结合扫描诊断、地图和 TF 判断。新狗的身高/姿态不同，核对高度切片是否仍合理，不能直接假定旧狗地面高度有效。

T2 开始 B 的观察：

```bash
OBS_DIR="$SESSION_ROOT/continuity_B_$(date +%H%M%S)_$$"
python3 scripts/check_go2_continuity.py --seconds 620 --phase static_B_native_slam \
  --power-state-topic /lf/lowstate --output "$OBS_DIR"
```

观察结束后，**保持 SLAM 和录制器运行，狗继续静止**，在另一个已加载环境的终端保存地图：

```bash
python3 scripts/save_live_slam.py \
  --output-root "$SESSION_ROOT/live_static_B_$(date +%H%M%S)_$$"
```

确认 `live_verification.json` 中 `status` 为 `success`，`map_reload` 的逐格、尺寸与几何检查通过。记录工具实际打印的 `verified_snapshot_*` 子目录。拍结束照片。

然后依次：T1 停止录制并等待关闭 → T3 停止 SLAM → T2 评估报告 → 读取已关闭的包。不要先停止 SLAM 再期待保存服务仍可用。

### 4.4 怎么判定

| 项目 | 项目内部目标/检查方式 | 不能据此声称什么 |
|---|---|---|
| 输入连续性 G1 | 完整在线至少 600 s，建议观察 620 s；平均点云 ≥12 Hz，>0.25 s 源间隔比例 <1%，最大源间隔 ≤1 s；报告还检查时间递增、非空云及接收静默 | 平均频率高不代表没有长断流；这些不是厂商规格 |
| 原厂里程计 | 记录净平移、净偏航、方向及漂移随时间变化 | 不能把里程计变化直接当成机身真实移动 |
| 物理静止 | 地面标记、开始/结束照片；可离线用 ICP 辅助检查 | ICP 是机载传感器一致性证据，不是独立外部真值 |
| 估计稳定 G2b2 | 沿用计划：10 分钟地图位姿偏航变化 <1°、`map→odom` 平移变化 <5 cm；分别检查 | 合成地图位姿净平移不能替代 `map→odom` 平移；TF 时间不对应时先记待验证 |
| 保存/重载 | `live_verification.json` 实际成功，地图和位姿图文件存在 | 地图能保存不代表地图几何准确，更不代表可自主导航 |

A、B 都通过再进入短运动采集。如果 A 通过、B 失败，记录 SLAM 开关、CPU/网络负载、电量和运行时间，优先做静止重试；不能仅凭顺序试验断言 SLAM 是根因。后半天有余量时再重复一轮静止观察，检查是否出现运行一段时间后才发生的断流。

### 4.5 换狗后静止点云仍偏移

11/18 操作者反馈，换狗后在原地仍看见 LiDAR 画面偏移。新狗的观察界面、话题、坐标系、偏移量和持续时间待记录。10/25 旧狗两次静止约十分钟，原厂里程计偏航分别变化 −20.64°、+12.41°，同时段机身点云前后各 60 秒的配准转角约为 −0.03°、−0.04°。因此优先核对新狗的里程计和显示坐标。新狗的故障归因与 Go2 型号层面的普遍性都还需要证据。[旧狗实测](field_results_2026-10-25.md)

**先做两分钟的静止观察。** 只接入目标狗，记下机身编号，保持当前姿态和软件参数。选一个两面实墙相交的墙角，用固定机位拍到狗与地面朝向标记。观察者记下是单个点在刷新、整段墙边持续平移、转动，还是地图出现双墙。若最初在 Unitree App 发现，保留原界面的短视频和时刻，再用原生 Ubuntu 上的 RViz 对照。

1. 在 §3.2 的 Ubuntu 环境中读取两路点云各自的坐标名。下面两条命令分别读取一条消息的头部。

   ```bash
   timeout 10s ros2 topic echo /utlidar/cloud_base --qos-reliability best_effort --once --field header
   timeout 10s ros2 topic echo /utlidar/cloud --qos-reliability best_effort --once --field header
   ```

2. RViz 中单独显示 `PointCloud2` 的 `/utlidar/cloud_base`，把 `Global Options → Fixed Frame` 设成第一条消息的 `frame_id`（原配置为 `base_link`），把 `Decay Time` 设成 `0`。保持视角不动，对比开始和结束的墙角位置。若墙边整体移动，切换到 `/utlidar/cloud`，同时把 Fixed Frame 改成第二条消息的 `frame_id`，在同一场景重复观察。若消息超时或 RViz 数据为空，先按 §3.4 核对接收、数据源和 QoS。
3. 在 §4.3 的 B 阶段，已有时间桥和 SLAM 时，可在两个终端分别运行下面两行，比较同一时间窗的平移和偏航角（度）。记录起止时刻和读数。若要在 RViz 把 Fixed Frame 切到 `odom` 或 `map`，选经时间桥对齐的 `/go2/slam/cloud_base`。A 阶段维持仅采集，保留原始里程计供离线分析。

   ```bash
   timeout 125s ros2 run tf2_ros tf2_echo odom base_link
   timeout 125s ros2 run tf2_ros tf2_echo map odom
   ```

| 同一时间窗内的观察 | 下一步处理 |
|---|---|
| 单帧点位在更新，墙边和墙角位置保持稳定 | 留下相同视角的起止图，继续按 A/B 采满十分钟。用固定结构的位置变化量描述漂移。 |
| 机身点云中的墙角稳定，`odom → base_link` 偏航或平移持续变化 | 与同段 `/utlidar/robot_odom`、固定机位影像对照。若里程计漂移重现，用同一包离线比较扫描匹配能否纠正漂移，并用已有转向包复核真实转动是否保留。旧狗两次偏航方向相反，固定角度补偿缺乏依据。 |
| 机身点云和里程计稳定，只在 `map` 中出现墙体重影 | 核对扫描与里程计时间、同一 TF 父子关系的发布源，以及 `map → odom` 的变化是否对应重影时刻。该变换本身也用于正常定位修正；结合地图几何和回放的 `trajectory_timing` 判断，再决定是否调整扫描匹配。 |
| 雷达自身坐标下墙角稳定，只有机身点云中的墙角持续偏移 | 对照同段两路点云的时间戳、坐标定义和机身点云的生成过程。先检查变换和预处理配置。 |
| 雷达自身坐标下的实墙也持续偏移 | 先从固定视频核实机身是否微动，核对传感器固定情况、行人和反射表面。在静态、有清楚墙角的场景复测。若仍可重复，保留设备编号、固件、原始点云、IMU、雷达状态和视频，交设备负责人或宇树支持排查。 |

**用已关闭的录制包复核。** 按 §3.3 确认 Elements 挂载可写，按 §4.2–4.3 分别保存 A/B 的包、连续性报告和 B 阶段地图。按 §6.2 对同一新狗静止包运行漂移与 ICP 脚本，后者默认比较 `cloud_base`。记录里程计净偏航、ICP 转角、平移、内点率，以及所用时间窗；墙角被遮挡或多个配准解同样合理时，保留不确定性。按 §6.3 回放 B 包，再按 §4.4 分别判定输入连续性、静止估计稳定性及地图重载。原始点云在自身坐标下也异常时，另外保留该话题的对照结果。

每轮可直接填写：`设备编号 ____｜主机/固件 ____｜包目录 ____｜界面/话题/Fixed Frame ____｜观察时段 ____｜实墙位移或转角 ____｜里程计偏航变化 ____｜ICP 转角/平移/内点率 ____｜map→odom 变化 ____｜照片/视频 ____｜下一步 ____`。

## 5. 测量与人工遥控建图

### 5.1 先把独立测量做实

选择至少两段约 1–3 m、能在二维地图上定位的固定结构，例如两处墙角之间的距离、门框两侧间距。尽量选择方向不同的线段。记录：

| 编号 | 实物两端 | 实测长度 m | 不确定度 m | 测量者/工具/方法 | 照片编号 | 对应地图与端点 |
|---|---|---|---|---|---|---|
| M1 | 待填 | 待填 | 待填 | 待填 | 待填 | 待选点 |
| M2 | 待填 | 待填 | 待填 | 待填 | 待填 | 待选点 |

不确定度按实际卷尺读数和端点重复定位情况估计，不固定抄一个好看的值。地图端点必须对应同一实物，记录在该地图的米制坐标中；不使用截图像素坐标。现场至少拍清端点和卷尺，地图选点可以离线完成。

现有模板 `scripts/config/map_reference_template.json` 是空测量列表。填报字段包括 `id`、`independent`、`measured_by`、`method`、`measured_length_m`、`uncertainty_m`、`allowed_absolute_error_m`、`map_points_m`；另保留照片编号和被测地图路径。未测、端点未知或在未知栅格时保持待验证，不填虚构数值来让程序通过。

### 5.2 路线由短到长，每条单独录制

先确认操作者持有遥控器、起点/朝向有地面标记、线缆不会绊住人与狗。助手先告知采集准备情况，用户决定并启动动作。本文没有任何自动运动命令。

| 轮次 | 用户遥控动作建议 | 目的 |
|---|---|---|
| straight_01 | 条件允许时缓慢直行约 1–2 m，随后停稳 | 看平移过程中点云、TF、地图是否连续 |
| turn_01 | 在标注的方向之间做约 45°–90° 的缓慢转向，再停稳 | 看纯转向时地图是否更新；记录实际标记角度，不把目标角度当测量结果 |
| loop_01 | 在小范围内走一条清楚可复现的路线，回到起点附近并恢复初始朝向 | 获得闭合路线数据和累计误差参照 |
| loop_02 | 条件相同下重复上一条路线 | 检查结果是否可重复 |

“回到起点”只是闭合路线采集。默认 `do_loop_closing: false`，不能把走了一圈写成 SLAM 闭环已验收。首次新狗现场不临时打开闭环；留原始包供之后离线比较。

每条路线采用同样顺序：

1. 停稳，记录起点照片、场景和电量；确认上一条 SLAM 已正常停止。
2. T1 启动新的录制，例如 `bash scripts/record_d1.sh loop_01`。
3. 录制开始后，T3 启动 §4.3 的 SLAM，确认地图/TF/扫描正常。
4. 记录员说“录制已开始，地图正常，等待用户启动”；用户开始遥控。
5. 每次停顿、碰线、人员遮挡、接管和异常都写入事件记录，手机视频尽量包含起点和终点标记。
6. 回到终点后静止，先运行 `save_live_slam.py` 保存本轮新目录，再结束录制与 SLAM。
7. 读取已关闭的包，确认不是只保存了开始或结束的一小段。

独立重复运行的地图可比较，但未经对齐不得直接拼接。不以“画面更丰满”替代几何检查。A/B 之后电量、姿态或配置有变，必须记入事件表。

## 6. 保存、回放与质量检查

本节可在现场空余时间或返回后进行。不要在正式采集中同时进行高负载回放；一次只运行一个回放实例。

### 6.1 文件检查

对每轮已结束的录制，确认至少存在：

```text
运行目录/
  run_metadata.yaml
  events.csv
  topics.txt
  config/
  source/
  bag/metadata.yaml
  bag/*.db3
  motion_validation.json       # 摘要脚本生成后
```

摘要脚本会生成分析文件并更新运行元数据；原始 `.db3` 不做编辑。录制中不读取 SQLite。实时地图输出另在 `live_* / verified_snapshot_*` 下，不要误认为一定在 rosbag 目录里。

### 6.2 静止漂移分析

先重新确认外盘仍挂载并可写，`RUN_DIR` 指向完整的静止 A 或 B 运行目录：

```bash
python3 scripts/check_external_storage.py --root "$GO2_DATA_ROOT" --probe
ANALYSIS_ROOT="$GO2_DATA_ROOT/analysis/2026-11-18/$GO2_ROBOT_ID"
python3 scripts/analyze_stationary_drift.py "$RUN_DIR/bag" \
  --output "$ANALYSIS_ROOT/drift_$(basename "$RUN_DIR")_$(date +%H%M%S)_$$"
python3 scripts/analyze_stationary_icp.py "$RUN_DIR/bag" \
  --output "$ANALYSIS_ROOT/icp_$(basename "$RUN_DIR")_$(date +%H%M%S)_$$"
```

这两个分析脚本本身没有完整的外盘挂载保护，因此前面的外盘检查不能省略；输出目录要求不存在。原始陀螺积分不直接作为角度真值，配准失败时保留失败记录。

### 6.3 隔离回放与质量报告

```bash
python3 scripts/run_slam_replay.py "$RUN_DIR/bag" --rate 1 \
  --output-root "$GO2_DATA_ROOT/slam_validation/2026-11-18/$GO2_ROBOT_ID"
```

现有回放器在脚本内部使用回环网卡 `lo` 和 DDS 域 87，只回放点云与里程计，不发布运动命令。使用这个入口，不在实机通信域直接播放包含全部原始话题的包。

回放完成后，抄录工具打印的真实输出路径：

```bash
REPLAY_DIR='/REPLACE_WITH_ACTUAL_REPLAY_DIRECTORY'
python3 scripts/evaluate_slam_run.py "$REPLAY_DIR" \
  --output-root "$GO2_DATA_ROOT/slam_quality/2026-11-18/$GO2_ROBOT_ID"
```

有已经填写、且端点对应这一张回放地图的独立测量文件时，再执行：

```bash
REFERENCE_FILE='/REPLACE_WITH_THIS_MAP_REFERENCE_JSON'
python3 scripts/evaluate_slam_run.py "$REPLAY_DIR" --reference "$REFERENCE_FILE" \
  --output-root "$GO2_DATA_ROOT/slam_quality/2026-11-18/$GO2_ROBOT_ID"
```

注意：

- `evaluate_slam_run.py` 需要回放目录的 `metrics.json`、`metadata.yaml` 和地图，不能直接把实时快照目录当回放目录传入。
- 独立参考文件只对应一张地图，不跨轮复用地图端点。
- 几段距离全部合格时脚本会给出 `partially_checked`；这不代表整张地图已全面验收。
- 短运动包不足十分钟时，源连续性时长项可为 `pending`；保留同设备当天 A/B 的长时证据，不伪造短包时长。
- TF 组合轨迹使用最近收到的地图变换，若 `trajectory_timing` 有限制，不能用逐点轨迹当同步真值。
- 当前回放器**没有 `--slam-params` 参数**；参数比较需要单独准备并记录实际配置，不照抄不存在的开关。第一轮保持仓库当前默认，不直接推广旧故障机上的调参结果。

## 7. WSL 接入对照与排障

### 7.1 定位：可尝试，尚未作为本项目正式接入机

公开资料中存在 WSL＋Go2 的个人 DDS 接入报告，也有 Windows/WSL 通过 WebRTC 的项目记录；它们不证明当前电脑、当前仓库和双狗场景已经稳定。普通 ROS 2 talker/listener 成功、`ping` 成功、能列出话题，均不能替代实机点云连续性验证。

主流程优先在同学原生 Ubuntu 机完成。WSL 分支只在有剩余时间时推进；如果 ROS 2/消息包都未安装，先记录为“软件环境待准备”，不在有限现场时间里默认重装全部环境。

### 7.2 Windows 只读检查

在 **Windows PowerShell** 运行：

```powershell
wsl --list --verbose
wsl --version
Get-NetAdapter | Select-Object Name,InterfaceDescription,Status,LinkSpeed
Get-NetIPConfiguration
Get-Content -LiteralPath "$env:USERPROFILE\.wslconfig" -ErrorAction SilentlyContinue
```

`.wslconfig` 不存在并不表示系统损坏。记录网卡、Windows/WSL 版本、已有网络模式、VPN和虚拟网卡情况；不要先关闭所有防火墙或卸载网络软件。

进入现有发行版（先确认其中没有自动启动机器人动作的自定义启动脚本）：

```powershell
wsl -d Ubuntu-22.04
```

之后在 **WSL Bash** 做只读环境检查：

```bash
cat /etc/os-release
ip -br addr
ip route
test -f /opt/ros/humble/setup.bash
command -v ros2
printenv ROS_DISTRO RMW_IMPLEMENTATION ROS_DOMAIN_ID CYCLONEDDS_URI
```

新终端找不到 `ros2`，先区分“未加载环境”和“未安装”。有 Humble 时再加载 `/opt/ros/humble/setup.bash`；Unitree 消息包和工作区须加载这台 WSL 上的实际安装路径，不能直接套用同学的 `/home/yaorz/...`。

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix unitree_go
ros2 pkg prefix rmw_cyclonedds_cpp
```

找不到包时停止实机接入验收。可以记录缺项后离线准备。现有源码在 Windows `D:\SiZuJiQiRen`，WSL 常见可见路径为 `/mnt/d/SiZuJiQiRen`，但应先检查实际挂载；看到源码不代表依赖已安装或已构建。

### 7.3 网络配置调整参考，不是自动修复脚本

只有在保存当前配置、确认没有其他重要 WSL 作业后才调整网络。微软文档中的镜像模式配置为：

```ini
[wsl2]
networkingMode=mirrored
```

该片段是合并到用户目录 `.wslconfig` 的配置参考，**不能用它覆盖整个文件**；已有内存、代理或其他设置应保留。修改生效通常需要重启 WSL；`wsl --shutdown` 会终止所有 WSL 发行版及其中任务，不能在录制、构建或同学任务进行时执行。

调整顺序：

1. 确认 Windows 物理网卡有载波、地址与机器狗实际网络匹配。
2. 确认 WSL 能看到对应网络和正确路由，而不是只看见与机器人不可达的默认 NAT 路径。
3. 核对 CycloneDDS 绑定实际 WSL 接口，DDS 域与狗匹配；不要给电脑随便换一个域号期望它仍能看见原厂狗。
4. 检查 Windows 与 Hyper-V 防火墙是否阻断实测 DDS 流量。按实际网卡、对端和端口做最小范围调整并保留回退记录；本手册不提供全局关闭防火墙命令。
5. 若默认发现仍失败，可根据原生 Ubuntu 的已知对端和抓包证据研究显式 DDS peers；不复制网上固定 IP、接口名、旧版 XML 字段或狭窄端口范围当通用配置。

改环境后可正常停止当前终端启动的 ROS CLI daemon，再重新发现；不要把看到旧缓存的话题当成新连接成功。

```bash
ros2 daemon stop
ros2 topic list --spin-time 3 --no-daemon -t
ros2 topic info --spin-time 2 --no-daemon --verbose /utlidar/cloud_base
ros2 topic info --spin-time 2 --no-daemon --verbose /utlidar/robot_odom
```

### 7.4 同狗、同线、两台主机顺序对照

比较时保持狗的姿态、场景、配置尽量一致，记录电量与时刻；顺序对照仍有运行时间、温度等混杂，不能当严格因果证明。

1. 原生 Ubuntu：同一只正常狗静止，短观察通过，保留主机和网络信息。
2. 录制器正常关闭后再换线到 Windows 机；不要拔走仍在录制的主机网线。
3. WSL：先看 30 秒实际消息，再延长到 620 秒；只做传感器订阅，不运行运动示例。
4. 若 WSL 失败，再把同一根线接回原生 Ubuntu 复核，判断问题是否只在 WSL 路径出现。

WSL 中已具备依赖且源码可访问时，可以使用不依赖固定网卡的轻量检查：

```bash
cd /mnt/d/SiZuJiQiRen
python3 scripts/inspect_go2_streams.py --seconds 30 --light
```

有余量延长观察：

```bash
python3 scripts/inspect_go2_streams.py --seconds 620 --light
```

这些命令只打印报告，不写原始包或大型点云。脚本保存的是接收频率、最大内部接收间隔等有限统计，**没有完整 G1 所需的全部源间隔、首尾静默和主机物理网卡证据**，因此长观察完成只能标“初步收数通过”，不能直接填正式 G1 通过。

### 7.5 WSL 成为正式第二接入机之前的门槛

- [ ] 话题类型、真实点云/里程计、时间与 frame 正确，重启终端后能复现连接。
- [ ] 连续性监控已适配实际网卡，并与评估器字段一致；没有伪造 `enp2s0`。
- [ ] WSL 内虚拟网卡/UDP 计数与 Windows 物理网卡证据分开，不混称同一指标。
- [ ] 原始录制的真实外盘目标已经验证，能正常关闭、读取和回放。
- [ ] 完成不少于十分钟的正式输入检查，再验证 SLAM 和保存流程。
- [ ] 原生 Ubuntu 对照与 WSL 结果明确标注主机、配置和差异。

**存储限制很重要：** Windows 的 `E:`/`F:` 不能仅凭盘符认定就是 Elements，WSL 的 `/mnt/e` 也不等于项目已验证的外盘挂载。若唯一 Elements 正插在同学电脑，WSL 暂时只做不落盘大流的订阅检查。要同时录两狗，需先建立并验证指向实际外盘的共享存储或明确另一独立外盘方案，记录路径映射；未完成前不向 WSL 内部磁盘偷偷录制。

当前 `check_external_storage.py` 对默认 `/media/yaorz/Elements` 有挂载检查，但对随意传入的其他根目录主要检查空间和写入。把 `GO2_DATA_ROOT` 改成 `/mnt/...` 并看到“可写”，**不能单独证明目标是外盘**。

## 8. 双狗双机的分层操作

### 8.1 只有一只正常狗时

完成这只狗的主流程，然后顺序做两台主机接入对照。可以验证电脑兼容性，不能称为“双狗同时运行”。旧故障狗不因凑数量而自动作为第二台有效实验设备。

### 8.2 两只正常狗都到场时

先分别用已验证的原生 Ubuntu 机检查 R1、R2。两只狗各有自己的编号、固件记录和至少一次静止基线；某只不通过时先保留该狗结果，另一只继续单机流程。

第一层采用独立连接：

```text
R1 ──网线── 原生 Ubuntu 机：R1 采集/单狗 SLAM
R2 ──网线── Windows/WSL 机：仅在接入验收后运行 R2 采集/单狗 SLAM
```

正式接入没有完成的 WSL，不能仅凭第二条网线插上就标记双机通过。替代选项是另一台已经配置并验证的原生 Ubuntu 机或机载开发电脑，不要求当天必须得到。

### 8.3 先隔离，再共享

原始话题如 `/utlidar/cloud_base`、`/api/sport/request` 可能同名，内部 IP 也可能重复。各狗原始 DDS 网络应只经过自己的接入端，不直接透明桥接到公共 Wi-Fi，不把两狗直接放进同一个未做隔离的广播域。

双机同时运行前确认：

- [ ] 各狗身份与物理连线可对应；IP、网卡、发布端信息已登记。
- [ ] 各接入进程绑定正确接口，原始 DDS 不从另一台电脑的公共网络泄漏。
- [ ] 暂不共享原始 `/tf`、`/tf_static` 或运动请求。
- [ ] 双机和回放的数据目录、机器人标签、主机标签独立。
- [ ] 在停止各自记录后做链路断开/恢复检查，确认断开 R1 不导致 R2 的数据身份变化；不以制造录制损坏来检查身份。
- [ ] 同一条命令不会被两个机器人接收；控制出口尚未完成时，只做被动数据验证。

**仅增加 namespace 不够。** 向公共协同网络发布前需要显式桥接、话题重命名与消息内 frame 改写，TF 的父子 frame 也要一致。下面是待实现的接口示例，不是当前仓库已经存在的话题：

| 内容 | R1 示例 | R2 示例 |
|---|---|---|
| 点云 | `/robot_a/cloud` | `/robot_b/cloud` |
| 里程计 | `/robot_a/odom` | `/robot_b/odom` |
| 地图 | `/robot_a/map` | `/robot_b/map` |
| 机身坐标 | `robot_a/base_link` | `robot_b/base_link` |
| 局部坐标 | `robot_a/odom`、`robot_a/map` | `robot_b/odom`、`robot_b/map` |
| 任务 | 发给 R1 的独立目标接口 | 发给 R2 的独立目标接口 |

原生 Ubuntu 可以兼任总控。原计划是集中式 SLAM；若未来为降低无线带宽，改成每台电脑各跑 SLAM、共享地图与位姿，应作为明确的架构选择记录，不能混淆两种实验配置。

### 8.4 明天适合采集的双狗证据

1. 两狗分别建立单机地图，记录起点位置和朝向；先让一只运动、另一只静止，均由用户操作，便于核对归属。
2. 用同一地面坐标基准测两狗起点的相对位置、朝向，并拍包含两狗起点标记的照片。记录单位、坐标轴方向和测量误差。
3. 选择两条有一部分重叠观测区域的小路线，各自独立录制，记录布局是否被改变。
4. 保存两份原始包、各自地图、初始相对位姿和视频。融合程序没完成时，成果写为“双狗独立采集与对齐资料准备”。
5. 只有在桥接、唯一坐标系与地图融合已经实现并验证后，才显示共同地图；两张图同时出现在 RViz 中不自动等于融合成功。

### 8.5 自主控制的后续顺序

每狗先验证速度桥接、限速、命令有效期、超时停车、人工接管与控制归属，再做单狗到点导航，最后才由总控分配不同目标。服务器离线时不能依靠已经断开的无线链路补发停车；接入端看门狗及机器人侧实际停车行为都需要实测。

本仓库已有的 `go2_safe_move` 和 `go2_sport_test` 包含运动动作，不是完成上述能力的通用控制桥。本手册不提供“一键双狗运动”命令。线缆连接期间即使软件准备完成，任何动作仍由用户启动。

## 9. 无线连接的可选验证

先区分两个阶段：

- **电脑间无线**：两台电脑共享地图/任务，狗与电脑仍接网线。
- **狗端无线**：接入程序放到机载开发电脑或随狗携带的接入设备，狗不用拖到笔记本的长网线。

推荐的最终网络结构为：

```text
R1 的独立机器人网络 ── 机载接入端 A ──Wi-Fi──┐
                                            路由器 ── 总控电脑
R2 的独立机器人网络 ── 机载接入端 B ──Wi-Fi──┘
```

按顺序推进：

1. 登记实际型号、机载开发电脑、登录权限和无线网卡；不修改未获授权的系统服务。
2. 检查该设备的无线接口能否访问所需 SDK/DDS 数据。App 能连 Wi-Fi，只说明 App 链路工作，不能证明当前 ROS 2 接口可以无线访问。
3. 先让一只狗静止，通过接入端向总控转发明确选定的数据；不要把两个原厂 DDS 网络透明合并。
4. 对照有线基线，测点云有效频率、源间隔、接收静默、延迟波动和链路恢复；一只通过后再加入第二只。
5. 原始点云带宽不足时，评估降低共享数据量或机载建图、共享地图/位姿；保留采样策略和限制，不能把缺失帧补造成真实观测。
6. 无线移动覆盖与断联停车在后续由用户启动的专门测试中验证，静止网络通过不等于运动覆盖已通过。

无线阶段未准备好就结束在“资源与接口已核对”。不要把明天正常狗采集的时间全部用来配置路由器、刷固件或更换协议。第三方 WebRTC 方案与现有有线 DDS 的话题、时序和频率可能不同，应单独作为新链路验证。

## 10. 故障处理和停止条件

| 现象 | 先检查/记录 | 当天处理 |
|---|---|---|
| 网卡无载波 | 当前接口、线缆、USB 网卡、设备电源 | 换已知可用线/口做对照；不先改算法 |
| `ping` 通但无话题 | 实际 DDS 网卡、域、消息包、Windows/Hyper-V 防火墙、CLI 缓存 | 记录配置和有界抓包，不以 ping 宣布接入成功 |
| 有话题但无真实消息 | 发布端/QoS、接收计数、实际源 IP、启动模式 | 先用被动检查脚本；列出话题名不算采集通过 |
| 点云持续多秒缺口 | A/B 阶段、源/接收间隔、主机错误、电量、时间 | 停止扩展运动，保留原始包与观察报告 |
| A 正常、B 异常 | 是否同时跑回放、大量可视化、其他订阅或录盘任务 | 静止条件下一次改变一个因素复查；不直接归因 |
| 地图没有自由/占据格、TF 持续报错 | 输入 frame、时间桥、扫描诊断、是否启动重复节点 | 保存日志，先修单机链路，不继续扩大路线 |
| 地图保存失败 | SLAM 是否仍在运行、服务是否存在、外盘、固定网卡假设 | 保留失败日志及原始包；不要把 PGM 文件存在当成功 |
| WSL 卡在网络或依赖 | 当前版本、安装缺项、配置与失败阶段 | 到时间预算后回到原生 Ubuntu，后续再排查 |
| SQLite 锁/录制崩溃 | 是否录制中读库、存储错误、进程退出信息 | 保留原件；恢复只能在另一个副本上进行 |
| 电量不足、线缆拉扯、用户要求停止 | 事件发生时刻 | 用户先停止机器人；记录员正常关闭录制，保存现场情况 |

禁止通过以下方式“消除问题”：删除失败报告、修改时间戳伪装连续、把同盘副本标为独立备份、把未知状态字段解释成已确认原因、把故障机结果直接混入正常机统计。

## 11. 收尾、备份与记录表

### 11.1 离开前顺序

1. 用户让狗停稳，确认没有仍在发送运动请求的程序。
2. 若本轮 SLAM 正常，先保存地图和位姿图并检查重载结果。
3. 正常停止录制器，等待退出及 `metadata.yaml` 落盘；再停止 SLAM/观察进程。
4. 检查每个已关闭包的信息，生成摘要；不对录制中的数据库做检查。
5. 拍最后设备姿态、线缆和场景照片，补齐所有路径及事件。
6. 复制完整实验目录到另一实际设备或独立存储。先确认目标身份、空间和可写；复制完核对文件数量、大小和 SHA-256，才登记备份完成。
7. 若没有第二设备，如实写“独立备份未完成”，不要把 Elements 内另存目录/ZIP 或 GitHub 小型证据算成原始包备份。
8. 等复制/校验完成，按系统正常卸载外盘；不要直接拔盘。

一次实验的独立备份应包含 bag、元数据、参数、源码快照、日志和关联地图/照片。这里不预填备份盘路径，避免复制到错误磁盘。现有盘点清单对应 11/17 的历史数据，新实验需新建自己的文件校验记录，不能声称已被旧清单覆盖。

### 11.2 运行索引模板

填入本次现场记录；小型文字结果可在事后另建带日期的实验记录，但不要改写同学原计划。

| 运行 | 狗编号 | 主机/网卡 | SLAM 开关 | 包目录 | 观察目录 | 地图目录 | 时长/结束状态 | 结果 |
|---|---|---|---|---|---|---|---|---|
| smoke | 待填 | 待填 | 关 | 待填 | 待填 | — | 待填 | 未做 |
| static_A | 待填 | 待填 | 关 | 待填 | 待填 | — | 待填 | 未做 |
| static_B | 待填 | 待填 | 开 | 待填 | 待填 | 待填 | 待填 | 未做 |
| straight_01 | 待填 | 待填 | 开 | 待填 | 按实际填写 | 待填 | 待填 | 未做 |
| turn_01 | 待填 | 待填 | 开 | 待填 | 按实际填写 | 待填 | 待填 | 未做 |
| loop_01 | 待填 | 待填 | 开 | 待填 | 按实际填写 | 待填 | 待填 | 未做 |
| WSL_compare | 待填 | 待填 | 关 | 无/实际路径 | 待填 | — | 待填 | 未做 |
| R2_static | 待填 | 待填 | 按实际填写 | 待填 | 待填 | 待填 | 待填 | 未做 |

### 11.3 事件记录模板

| 本地时间（Asia/Shanghai） | 运行/设备 | 事件 | 用户动作或配置变化 | 照片/视频/日志编号 |
|---|---|---|---|---|
| 待填 | 待填 | 录制实际开始 | 无动作/实际说明 | 待填 |
| 待填 | 待填 | 用户启动运动/停稳 | 实际说明 | 待填 |
| 待填 | 待填 | 异常、掉线或遮挡 | 实际说明 | 待填 |
| 待填 | 待填 | 录制正常关闭 | 是否正常 | 待填 |

### 11.4 当天结论模板

```text
日期：2026-11-18
实到正常机器狗数量、编号：
使用主机与代码版本：
新狗静止 A/B：通过 / 失败 / 待验证 / 未做；对应证据：
地图保存/重载：
独立测量：完成几段；照片与地图端点是否齐全：
人工遥控路线：完成哪些；是否有起终点标记：
WSL：未准备 / 无法接入 / 初步收数 / 正式连续性通过；限制：
双狗：未做 / 各自接入 / 同时隔离采集 / 已融合；证据：
无线：未做 / 接口检查 / 静止收数 / 其他；证据：
原始数据位置：
独立备份目标与校验结果：
最关键的新发现：
未完成事项与需要的设备/软件：
```

### 11.5 最低带回清单

- [ ] 正常狗的身份、固件、姿态、时间与人员记录。
- [ ] 完整 A/B 原始包和连续性报告；未完整的轮次明确标注。
- [ ] 至少一份经过保存/重载检查的地图；未通过时保留失败信息。
- [ ] 两段独立距离、测量方法、照片；端点尚未选取时记录为待补。
- [ ] 至少一条用户遥控路线及对应视频，前提条件未满足则明确未做。
- [ ] 每次运行真实参数、版本、目录及事件记录。
- [ ] WSL/双狗/无线的实际推进层级，不扩大结论。
- [ ] 独立备份状态；未备份的原因及原始盘所在位置。

## 12. 证据、来源与命令限制

### 12.1 项目材料

- [原项目计划](PLAN.md)：保留原文，本文只提供 11/18 操作参考。
- [10/25 结果](field_results_2026-10-25.md)：11/17 整理的历史实验事实，包含未通过项。
- [建图包说明](../src/go2_slam_bringup/README.md)：时间桥、扫描、SLAM与保存链路。
- [录制脚本](../scripts/record_d1.sh)、[连续性观察](../scripts/check_go2_continuity.py)、[观察评估](../scripts/assess_continuity_report.py)。
- [轻量流检查](../scripts/inspect_go2_streams.py)、[实时快照](../scripts/save_live_slam.py)、[隔离回放](../scripts/run_slam_replay.py)、[质量评估](../scripts/evaluate_slam_run.py)。
- [数据提交与备份约定](data_publication.md)：原始数据不随代码仓库下载，GitHub 不等于原始数据备份。

### 12.2 WSL 和无线资料（2026-11-17 查阅）

- [微软 WSL 网络说明](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking)：镜像网络支持组播/局域网，仍需处理 Hyper-V 防火墙；这是网络能力说明，不是 Go2 实机保证。
- [宇树 ROS 2 官方说明](https://github.com/unitreerobotics/unitree_ros2/blob/master/README.md)：Ubuntu 22.04/Humble、网络接口与状态/传感器接入。
- [Leonardo Stringhetta 的 Go2＋WSL 个人实机记录](https://pt.linkedin.com/pulse/colocando-o-unitree-go2-do-unisalesiano-ara%C3%A7atuba-de-stringhetta-q682f)：作者报告通过镜像网络、防火墙和 DDS peers 解决接入；不是本项目长时间 SLAM 验收。
- [go2-fusion 的 Windows/WSL 页面](https://github.com/maxhawksby/go2-fusion/wiki/WSL-and-Windows)：记录 WebRTC 传感器测试，同时明确 Windows Docker 下有线 CycloneDDS 发现未测试；两种路径不可混称。
- [Go2 官方规格](https://www.unitree.com/go2/)：无线硬件能力与具体开放接口应分开核对。

### 12.3 本手册的验证边界

命令参数及输出格式根据当前仓库源码核对；编写时没有连接新机器狗，没有启动实机程序，没有在用户 WSL 中安装依赖、修改网络或执行运动。Ubuntu 实机执行、WSL 接入、第二狗资源、无线链路与双狗协同均以明天实际记录为准。
