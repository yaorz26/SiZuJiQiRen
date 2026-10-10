# Unitree Go2 ROS 2 工作区

面向 Ubuntu / ROS 2 Humble 的 Go2 实验工作区，包含状态记录、点云处理、
TF 桥接、SLAM 启动配置和现场数据采集脚本。

## 获取代码

```bash
git clone --recurse-submodules git@github.com:yaorz26/SiZuJiQiRen.git
cd SiZuJiQiRen
```

已有工作区需要初始化固定版本的 KISS-ICP：

```bash
git submodule update --init --recursive
```

`src/kiss-icp` 来自上游 PRBonn/KISS-ICP，父仓库固定在提交
`1ffa7d7512f10bfc8b1185095011fa31184019e3`。本地依赖来源和使用方式见
[vendor/README.md](vendor/README.md)。

## 主要内容

| 路径 | 用途 |
|---|---|
| `src/go2_tf_bridge` | 里程计与 TF 桥接 |
| `src/go2_lidar_snapshot`、`src/go2_lidar_filter` | 点云保存和过滤 |
| `src/go2_state_logger` | 机器人状态记录 |
| `src/go2_slam_bringup` | 二维 SLAM 与三维点云地图投影启动配置 |
| `src/go2_cloud_map_projector` | 将三维点云投影为二维占据栅格 |
| `src/go2_kiss_icp_bringup` | Go2 KISS-ICP 里程计与地图投影配置 |
| `scripts/record_d1.sh` | 单机现场原始数据录制及元数据保存 |

## 构建与运行

需安装 ROS 2 Humble、colcon、相应 ROS 消息/启动依赖及 Eigen3、TBB。
运动和状态接口包还依赖本机已安装的 Unitree ROS 2 消息与 API；先加载
当前已经验证可用的 Go2 环境。

建图包的具体构建参数、启动方式和输入话题说明见
[KISS-ICP 使用说明](KISS_ICP_NEXT_STEPS.md)。首次构建后加载
`install/setup.bash`。构建产物会在本地生成，由 Git 忽略。

## 现场任务与数据管理

- [项目需求](docs/requirement.md)
- [每周推进规划](docs/successful_progress_plan.md)
- [现场测试卡与检查表](docs/field_session_checklist.md)
- [2026-10-10 单机采集任务](docs/today_tasks_2026-10-10.md)
- [2026-10-10 单机静止测试记录](docs/stationary_test_2026-10-10.md)
- [已有环境与进展交接](helper/UBUNTU_CODEX_HANDOFF.md)
- [Git 管理约定与历史数据索引](docs/git_management.md)

录制、点云、地图、视频和大规模实验数据优先保存到
`/media/yaorz/Elements/unitree_go2`。默认采集入口检查外盘挂载、可用空间
和写入条件，失败时停止。每次运行独立保存元数据和配置快照。
源码、配置、脚本、文档及小型数据索引保留在工作区。
