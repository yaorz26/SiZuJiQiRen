# Unitree Go2 ROS 2 工作区

面向 Ubuntu / ROS 2 Humble 的 Go2 实验工作区，包含状态记录、点云处理、
TF 桥接、SLAM 启动配置和现场数据采集脚本。

项目计划只维护 [docs/PLAN.md](docs/PLAN.md)；历史计划与指导见
[归档目录](docs/archive/README.md)，不再作为行动依据。

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
| `src/go2_slam_bringup` | 时钟对齐、运动补偿扫描与二维 SLAM 启动配置 |
| `src/go2_cloud_map_projector` | 将三维点云投影为二维占据栅格 |
| `src/go2_kiss_icp_bringup` | Go2 KISS-ICP 里程计与地图投影配置（候选路线） |
| `scripts/` | 现场录制、连续性观察、回放、地图保存与质量评估 |

## 构建与运行

需安装 ROS 2 Humble、colcon、相应 ROS 消息/启动依赖及 Eigen3、TBB。
运动和状态接口包还依赖本机已安装的 Unitree ROS 2 消息与 API；先加载
当前已经验证可用的 Go2 环境。

建图包的启动方式、输入话题和当前限制见
[go2_slam_bringup README](src/go2_slam_bringup/README.md)。首次构建后加载
`install/setup.bash`。构建产物会在本地生成，由 Git 忽略。

KISS-ICP 是候选里程计路线，其构建与运行说明已归档：
[KISS_ICP_NEXT_STEPS.md](docs/archive/2026-10-10/KISS_ICP_NEXT_STEPS.md)。

## 计划、记录与数据管理

- [项目计划（唯一计划）](docs/PLAN.md)
- [原始需求](docs/requirement.md)
- [2026-10-10 实机测试记录](docs/stationary_test_2026-10-10.md)
- [2026-10-10 SLAM 状态与质量限制](docs/slam_status_2026-10-10.md)
- [2026-10-10 数据索引](docs/data_index_2026-10-10.yaml)
- [历史计划与指导（归档）](docs/archive/README.md)

录制、点云、地图、视频和大规模实验数据优先保存到
`/media/yaorz/Elements/unitree_go2`。默认采集入口检查外盘挂载、可用空间
和写入条件，失败时停止。每次运行独立保存元数据和配置快照。
源码、配置、脚本、文档及小型数据索引保留在工作区。
