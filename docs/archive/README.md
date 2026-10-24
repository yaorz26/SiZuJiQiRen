# 历史计划与指导归档

2026-10-24 整理：项目此前存在多份计划、测试卡和路线指导，内容重叠，部分日期与前提已被 10/10 实机结果改变。为使 [PLAN.md](../PLAN.md) 成为唯一有效的计划，以下文件从 `docs/` 和仓库根目录移入本目录，**原样保留作历史追溯，不再作为行动依据**。

归档时仅机械修正了因移动而失效的相对链接；正文内容未改写。归档文件里的“下一步”“必须完成”等表述均为当时快照。当前该做什么，只看 [PLAN.md](../PLAN.md)。

## 归档清单

| 文件 | 原用途 | 归档原因 | 当前替代 |
|---|---|---|---|
| [successful_progress_plan.md](2026-10-10/successful_progress_plan.md) | 7 个周末的总体推进规划（10/10 版） | 日期、阶段编号和 D1 结论已过时；与后续实机结果不一致 | [PLAN §5](../PLAN.md#5-阶段计划六个现场窗口) |
| [today_tasks_2026-10-10.md](2026-10-10/today_tasks_2026-10-10.md) | 10/10 当天任务单 | 当天已结束；任务与结果已转入记录和索引 | [PLAN §8](../PLAN.md#8-下一次现场2026-10-25s1) |
| [next_robot_session.md](2026-10-10/next_robot_session.md) | 下一次现场的测试卡与操作步骤 | 面向 10/11 前后的场景；部分命令/目录已变化 | [PLAN §8](../PLAN.md#8-下一次现场2026-10-25s1) |
| [slam_integration_plan_2026-10-10.md](2026-10-10/slam_integration_plan_2026-10-10.md) | 从采集转入单机 SLAM 的集成计划 | 时间对齐、TF、扫描、保存链路已完成 | [PLAN §4](../PLAN.md#4-技术主线与当前默认配置) |
| [field_session_checklist.md](2026-10-10/field_session_checklist.md) | 可复用的现场测试卡 | 与唯一计划并存会造成两套流程 | [PLAN §8](../PLAN.md#8-下一次现场2026-10-25s1) |
| [map_view_interpretation.md](2026-10-10/map_view_interpretation.md) | 三维点云与二维地图的对比解释 | 结论已并入计划与 bringup 文档 | [PLAN §4](../PLAN.md#4-技术主线与当前默认配置) |
| [server_mapping_architecture.md](2026-10-10/server_mapping_architecture.md) | 双机集中式建图架构说明 | 架构已在计划中概述；双机仍未开始 | [PLAN §1](../PLAN.md#1-目标与边界) |
| [git_management.md](2026-10-10/git_management.md) | Git 管理约定 | 关键规则并入计划，避免额外指导文件 | [PLAN §12](../PLAN.md#12-数据版本与文档规则) |
| [KISS_ICP_NEXT_STEPS.md](2026-10-10/KISS_ICP_NEXT_STEPS.md) | KISS-ICP 的构建/运行说明 | KISS-ICP 是候选路线，不是当前主线；构建命令保留作参考 | [PLAN §4](../PLAN.md#4-技术主线与当前默认配置) |
| [UBUNTU_CODEX_HANDOFF.md](2026-10-10/UBUNTU_CODEX_HANDOFF.md) | 5 月的环境交接与 ROS 入门 | 话题、环境和项目阶段均已变化 | [PLAN](../PLAN.md)、[README](../../README.md) |

## 仍然有效的记录（不在归档内）

以下是事实记录或索引，不是计划；保留在 `docs/` 供追溯与复现：

- [requirement.md](../requirement.md)：原始需求。
- [stationary_test_2026-10-10.md](../stationary_test_2026-10-10.md)：10/10 实机测试记录。
- [diagnosis_2026-10-10.md](../diagnosis_2026-10-10.md)：当时的问题与原因分析。
- [continuity_investigation_2026-10-10.md](../continuity_investigation_2026-10-10.md)：A/B/C 连续性对照。
- [charged_validation_2026-10-10.md](../charged_validation_2026-10-10.md)：充电后 A/B 与实时地图复测。
- [slam_status_2026-10-10.md](../slam_status_2026-10-10.md)：当时 SLAM 状态与质量限制。
- [offline_quality_2026-10-10.md](../offline_quality_2026-10-10.md)：离线质量评估结果。
- [data_index_2026-10-10.yaml](../data_index_2026-10-10.yaml)：原始包、地图与报告的索引。
- [legacy_artifact_index.csv](../legacy_artifact_index.csv)：移出 Git 跟踪的旧文件索引。

这些记录中的“下一次优先……”是当时的建议，已被 [PLAN.md](../PLAN.md) 取代；记录本身不更新为新的计划，以免改写证据。

## 归档规则

1. 只有 [PLAN.md](../PLAN.md) 是计划；不新建 `plan_*`、`next_steps`、`todolist`、`*_plan.md`。
2. 现场/离线结果写入带日期的记录文件，只写结果、证据、未通过项和复现入口，不写替代计划。
3. 阶段结束后可把被取代的记录/指导移入 `docs/archive/<日期>/`，并在本文件登记。
4. 归档文件保持只读语义，不随新结论改写；需要更正时写新的记录或在 PLAN 中说明。
