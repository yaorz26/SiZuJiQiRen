# 下次单机实机验证：先稳定输入，再测地图

适用条件：一台 Go2、两人、网线连接、室内空间有限。机器人动作只由操作员用遥控器启动，本机脚本均为接收、记录与建图。继续使用已验证的 enp2s0 接入，不在同一轮同时换无线网络或驱动。

## 进场前完成的准备

- 电池具备充足电量，记录实际开始电量；有备用电池时记录更换时间。不通过故意耗到保护关机验证供电。
- 固定场景和姿态，记录位置、朝向、开机时间、机器人身份和雷达版本。姿态改变后重新核对高度切片。
- 带卷尺、地面标记与相机；选两段可辨认的固定墙边或门框，测量真实距离及测量不确定度。
- Elements 已挂载且可写；使用固定代码/配置版本，记录每阶段目录。独立备份设备可用时再做独立副本。

## 现场顺序与停止条件

| 阶段 | 操作 | 时长预算 | 通过条件/停止处理 |
|---|---|---:|---|
| 接入与电源状态 | 固定姿态，核对载波、实际电量与电池消息 | 10–15 min | 有点云/里程计；电源字段存在且有效；无载波先查链路 |
| A：仅采集 | 完整静止录制与诊断，无服务器 SLAM | 约 11 min | 至少十分钟在线数据，点云平均 ≥12 Hz，源间隔 >0.25 s 占比 <1%，最大源间隔 ≤1 s |
| B：采集与建图 | 使用同样订阅/录制设置，开启服务器 SLAM | 约 11 min | 输入同样通过，TF 与地图正常，无进程退出 |
| 文件检查 | 结束录制、等 metadata 完整，检查原始包并保存地图 | 10–15 min | 文件可读取、非空；逐格重载及几何元数据一致 |
| 独立几何 | 两段真实距离，登记对应地图特征；条件允许才遥控短路线 | 20–40 min | 被测距离在约定误差范围内；不以里程计替代卷尺 |
| 小闭环 | 操作员按网线余量选择短路线，回到起点标记 | 20–30 min | 起终点位置/朝向有独立标记；闭环精度另行报告 |

上述输入门槛为项目暂定值，不是厂商规格。任一静止阶段再次出现多秒缺口，结束并保留该组，转为驱动/雷达/供电诊断，不继续扩展运动。主动断开后的无数据窗口不能当作雷达断流证据。

## 两个终端的静止采集入口

在两个终端均加载环境：

```bash
cd /home/yaorz/unitree_go2_ws
source /home/yaorz/unitree_ros2/setup.sh
source /home/yaorz/unitree_go2_ws/install/setup.bash
```

终端一启动约十一分钟的原始记录（启动检查会占十余秒，预留 660 秒以获得至少十分钟数据）：

```bash
GO2_RECORD_POWER_STATE=1 timeout --signal=INT --kill-after=10s 660s bash scripts/record_d1.sh charged_static_A
```

脚本输出目录按当天日期与 robot ID 建立；需要明确现场编号时设置 `GO2_SESSION_ID`、`GO2_ROBOT_ID`。限时完成后的外层退出码 124 是预设 timeout 的结果，以录制正常关闭和 metadata 完整为依据，不将其与锁冲突退出 134 混淆。

确认输出 Recording 及核心话题已订阅后，在终端二启动相同阶段的诊断。路径必须是新目录：

```bash
python3 scripts/check_go2_continuity.py --seconds 620 --phase charged_static_A \
  --power-state-topic /lf/lowstate \
  --output /media/yaorz/Elements/unitree_go2/experiments/charged_static_A_unique
```

电源话题存在但始终零值/异常值时，先核实固件字段含义，保留原值，不用未经确认的单位换算。电池状态与温度主要按接收时间关联，原始 tick 同时保留；并非独立高精度时钟同步。

A 结束后两端都停止，等录制写完，再检查。A 通过才运行 B；B 使用相同命令与订阅，仅将标签/输出目录改为 B，并在第三终端增加：

```bash
ros2 launch go2_slam_bringup go2_slam.launch.py
```

不在录制时执行数据库查询或 bag info/reindex。观测脚本断网会保存并退出，原始录制仍应先按 Ctrl+C 等待正常结束，拔线不能代替结束录制。

## 离线检查与地图质量

录制已正常结束后，按真实输出目录执行：

```bash
python3 scripts/summarize_motion_bag.py /absolute/path/to/completed_run
python3 scripts/assess_continuity_report.py /absolute/path/to/completed_observation
python3 scripts/run_slam_replay.py /absolute/path/to/completed_run/bag --rate 1
python3 scripts/evaluate_slam_run.py /absolute/path/to/replay_output
```

回放仅使用环回 DDS 域和传感器输入，输出源码/配置校验值、地图、位姿图、估计轨迹和检查结果。链路失败时保存失败报告，不能继续判定地图可用。轨迹是实时估计，使用最新收到的 map→odom 变换，并记录其时间差；不属于离线全局优化轨迹或独立真值。

连续性评估只读取已结束的观察，包含接收起末端的无消息时段。已确认断开连接的离线时间剔除，未完成时长保留待验证，同时汇总原始电源字段。不要只看某个一分钟窗口频率就判定长时稳定。

评估输出包括占据/自由/未知面积、四邻接自由连通区域、源连续性和独立距离状态。已观测面积不是场地覆盖率，自由单元连通也未计机身宽度和余量，不能直接作为导航通过证据。

## 独立距离的填写方式

复制 [参考模板](/home/yaorz/unitree_go2_ws/scripts/config/map_reference_template.json) 到该次记录目录。每项填写方法、测量者、真实长度、误差范围和同一物理特征在对应地图中的两端点。例如下列仅为格式说明，数值均为占位，必须替换成实测值：

```json
{
  "measurements": [{
    "id": "measured_wall_segment_1",
    "independent": true,
    "measured_by": "现场测量者",
    "method": "卷尺测量；端点有照片及标记",
    "measured_length_m": 1.0,
    "uncertainty_m": 0.01,
    "allowed_absolute_error_m": 0.10,
    "map_points_m": [[0.0, 0.0], [1.0, 0.0]]
  }]
}
```

端点以保存地图的米制坐标填写，图像行从顶部向下，地图 y 从底部向上；使用评估生成的地图坐标预览选点。坐标原点旋转也会被计算。端点在未知区域、缺少独立测量或误差范围无法区分通过/失败时，评估保留待验证。

```bash
python3 scripts/evaluate_slam_run.py /absolute/path/to/replay_output \
  --reference /absolute/path/to/independent_reference.json
```

两段距离通过仍只是部分几何检查；完整地图、闭环精度与导航需要额外证据。每阶段结束写清真实通过项和未通过项，不把“生成了图”作为全部验收。
