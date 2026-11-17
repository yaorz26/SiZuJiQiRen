# 2026-10-25 实验结果：静止漂移与输入连续性

> 2026-11-17 根据外盘中已经存在的报告、元数据和参数快照补录。
> 本次整理没有新采集、重新回放或机器人运动测试；本文是结果记录，不是新计划。
> 现行计划的基线时间为 10/24；10/25 的新增证据以本文和对应原始报告为准。

## 结论

1. **A 的十分钟输入连续性通过，后续 A2 未通过。** 两次均完成约 641 s 录制和 620 s 连续性观察。
   A2 虽然平均点云频率仍高于 14 Hz，但出现 5.89 s 源时间缺口，超过项目 1 s 门限。
2. **静止时原厂里程计偏航变化明显，且两次方向不同。** A 的摘要为 −20.64°，A2 为 +12.41°。
   同期点云前后窗口配准只得到约 −0.03°/−0.04° 的转动，支持机器人基本静止；不能把里程计偏航当作真实机身转动。
3. **A 包的一次离线回放完成保存和逐格重载。** 保存图位姿净变化为 0.0361 m、−1.0393°，
   但实际回放参数不同于仓库默认，且存在 TF 时间对应限制。不能据此宣布默认配置已通过静止稳定性验收。
4. **没有找到已填写的独立距离测量。** 现存测量模板的 `measurements` 仍为空。
   本次整理未找到新的 B 实时 SLAM、闭环或导航验收证据，不补写为已完成。

## 录制与观察窗口

| 指标 | A | A2 |
|---|---:|---:|
| 录制目录后缀 | `charged_static_A_110553_5696` | `charged_static_A2_113502_11761` |
| 录制时长 | 641.17 s | 641.24 s |
| 总消息数 | 494,158 | 491,951 |
| 原始数据库 | 875.17 MiB | 851.58 MiB |
| SQLite 检查（历史报告） | ok | ok |
| 观察时长 | 620.006 s | 620.007 s |
| 观察点云频率 | 14.647 Hz | 14.197 Hz |
| 最大源时间间隔 | 0.694 s | **5.890 s** |
| 源间隔 >0.25 s 占比 | 0.0441% | 0.5227% |
| 最大接收静默 | 0.672 s | **5.858 s** |
| 十分钟连续性评估 | passed | **failed** |
| 观察窗口主机 UDP / 网卡接收错误增量 | 0 / 0 | 0 / 0 |

频率和缺口占比来自各自 `continuity_*/assessment.json` 的 620 s 窗口；
完整录制的 `motion_validation.json` 统计窗口稍长，数值可能略有不同。
两次观察期间没有主机接收错误增量，不能据此定位或排除机器人端、驱动、链路或调度问题。
驱动自报丢包字段的单位未确认，不将其直接换算成百分比。

证据：[A 连续性](evidence/elements/experiments/2026-10-25_go2_1/continuity_A_110630/assessment.json)、
[A2 连续性](evidence/elements/experiments/2026-10-25_go2_1/continuity_A2_113540/assessment.json)、
[A 录制摘要](evidence/elements/experiments/2026-10-25_go2_1/charged_static_A_110553_5696/motion_validation.json)、
[A2 录制摘要](evidence/elements/experiments/2026-10-25_go2_1/charged_static_A2_113502_11761/motion_validation.json)。

## 静止位姿的两路对照

| 指标 | A | A2 |
|---|---:|---:|
| 里程计净偏航（录制摘要） | −20.6365° | +12.4098° |
| 里程计净平移 | 75.0 mm | 24.2 mm |
| `cloud_base` 前后 60 s 窗口 ICP 转角 | −0.0267° | −0.0389° |
| ICP 平移模长 | 28.6 mm | 5.8 mm |
| ICP 10 cm 内点率 | 99.19% | 94.68% |

A 另外保留了 `/utlidar/cloud` 原始点云对照：转角 −0.1028°、平移 5.0 mm、内点率 99.62%。
两种点云的统计结果均支持场景在机身坐标系中变化较小。
这属于机载传感器一致性检查，依赖场景静止和配准条件，不替代地面标记、照片或外部测量。

A 的专门漂移脚本采用不同取样方式，得到 −20.6621°，拟合斜率约 −1.948°/min；
其陀螺原始积分为 +87.92°、扣除初始偏置后仍为 +63.29°。
这些结果不能作为独立角度真值，也不应把不同窗口的数值混用。
A2 偏航方向发生反转，不能把此前 A/B 的负向漂移外推为固定补偿量；本次证据不足以确定根因。

证据：[A ICP](evidence/elements/analysis/2026-10-25/icp_A/stationary_icp.json)、
[A2 ICP](evidence/elements/analysis/2026-10-25/icp_A2/stationary_icp.json)、
[原始点云 ICP](evidence/elements/analysis/2026-10-25/icp_A_raw/stationary_icp.json)、
[A 漂移分析](evidence/elements/analysis/2026-10-25/drift_A/stationary_drift.json)。

## A 离线回放与实际参数

回放目录为 `slam_validation/2026-10-25_angle_default/charged_static_A_110553_5696_112335_891052`。
元数据标记源码提交 `96483b2d9d1014903e82559ac28fe5d7190cf4a0`，但参数快照与该提交默认配置存在差异：

| 参数 | 仓库默认 / 采集时配置快照 | 本次回放配置快照 |
|---|---:|---:|
| `angle_variance_penalty` | 0.1 | **1.0** |
| `minimum_angle_penalty` | 0.3 | **0.9** |
| `distance_variance_penalty` | 0.1 | **0.5** |
| `minimum_distance_penalty` | 0.2 | **0.5** |

回放参数文件 SHA-256 为 `c0b31fd5d19545e638b2211e1e16266e964e431285e03c95ad13005a288f7738`，
与回放元数据记录一致。应按该快照复现，不能仅凭 `angle_default` 目录名认定使用了仓库默认值。
本次只归档该实验参数，没有将其推广为新默认。

| 回放结果 | 数值 / 状态 |
|---|---|
| 保存链路、位姿图序列化、地图逐格重载 | passed |
| 地图位姿净平移 / 偏航 | 0.03605 m / −1.03931° |
| 输入里程计净平移 / 偏航 | 0.07503 m / −20.63654° |
| 地图尺寸 / 图节点 | 148×144 / 63 |
| 已观测面积 | 8.3538 m² |
| 最大自由连通面积 | 6.0912 m² |
| 独立距离 / 导航 | pending / not_verified |
| 轨迹 TF 时间检查 | timing_caveat；6,170 个样本中 5,750 个超出时间门限 |

1.03931° 仍略高于项目静止偏航 <1° 的门限；以上平移是合成地图位姿的净变化，
不能直接代替计划要求的 `map→odom` 平移变化。
轨迹使用最近收到的地图 TF 合成，时间戳间隔最大约 9.88 s，
因此逐点轨迹不能视为独立校验过的同步真值。
保存图面积也不是完整场地覆盖率，自由格连通性没有考虑机器人机身尺寸。

证据：[回放元数据](evidence/elements/slam_validation/2026-10-25_angle_default/charged_static_A_110553_5696_112335_891052/metadata.yaml)、
[实际参数](evidence/elements/slam_validation/2026-10-25_angle_default/charged_static_A_110553_5696_112335_891052/config/slam_toolbox.yaml)、
[回放指标](evidence/elements/slam_validation/2026-10-25_angle_default/charged_static_A_110553_5696_112335_891052/metrics.json)、
[质量报告](evidence/elements/slam_quality/2026-10-25/113442_095163/quality.json)。

## 其余记录与缺失证据

- `loadtest_no_record_1150` 保留了约 300 s 的非录制观察报告，点云平均约 14.49 Hz。
  它短于十分钟，不能替代 G1，也不足以独立证明录盘负载是缺口根因。
- [独立距离模板](evidence/elements/analysis/2026-10-24/measurement_2026-10-25.json)仍是空列表；
  人员、场景、初始姿态和部分固件元数据仍为 `TO_FILL`，不推断填补。
- 没有把原始状态中的错误码、电量原值或驱动字段解释成未经确认的厂商含义。
- 原始数据未上传 GitHub；来源、大小和校验值见[完整盘点清单](data_inventory_2026-11-17.csv)，
  实验对应关系见[10/25 数据索引](data_index_2026-10-25.yaml)。
