# Go2 ROS2 实机开发交接文档

本文档用于在 Ubuntu 主机上重新启动 Codex 或交给新成员时快速接手。假设新 Codex 没有任何上下文，但 Ubuntu 主机上已有 `unitree_go2_ws` 工作空间，并且今天从 `helloworld` 示例扩展出来的代码已经存在或可从 Windows 工作区复制过去。

## 1. 当前项目主线

项目原本在 `unitree_rl_mjlab` 中完成了 Go2 的仿真验证，包括：

- MuJoCo 中 Go2 高层速度控制；
- Viser 可视化；
- 航点跟踪；
- 覆盖路径；
- 虚拟雷达；
- 占据栅格建图；
- 多智能体覆盖规划展示。

现在真机路线已经从早期的 `unitree_sdk2 / go2_ctrl` 调整为：

```text
Ubuntu 22.04 原生环境
  -> ROS2 Humble
  -> unitree_ros2 / unitree_api / unitree_go
  -> 直接发布 /api/sport/request
  -> 调用 Go2 官方 Sport API
  -> 读取 /sportmodestate、/utlidar/robot_odom、/utlidar/cloud_deskewed 等 topic
```

重要结论：

- 不再优先走 WSL2 + SDK2 控制程序路线；
- 当前实机主线是 ROS2 直连 Unitree API；
- 控制、状态读取、雷达点云采集已经验证成功；
- 下一阶段准备部署或接入 SLAM。

## 2. ROS2 基础理解

### 2.1 topic

`topic` 可以理解成 ROS2 的消息频道。

例如：

```text
/api/sport/request
```

就是给 Go2 官方 Sport API 发请求的频道。

### 2.2 publisher

`publisher` 是消息发布者。代码里：

```cpp
publisher_ = this->create_publisher<unitree_api::msg::Request>(
  "/api/sport/request",
  10
);
```

表示创建一个发布者，往 `/api/sport/request` 发布 `unitree_api::msg::Request` 类型消息。

### 2.3 request

`request` 是发给 Go2 官方 API 的请求消息，不是 HTTP request。

核心字段：

```cpp
request.header.identity.api_id = 1016;
```

表示调用 Sport API 中编号为 `1016` 的功能。已验证 `1016` 是 `Hello` 打招呼动作。

`Move` 动作还需要参数：

```cpp
request.header.identity.api_id = 1008;
request.parameter = "{\"x\":0.10,\"y\":0.0,\"z\":0.0}";
```

其中：

```text
x = vx    前后速度，单位 m/s
y = vy    左右速度，单位 m/s
z = vyaw  转向角速度，单位 rad/s
```

## 3. 已验证的关键接口

### 3.1 Sport API 编号

已根据 Unitree 官方 `unitree_ros2` 示例确认：

```text
1002  BalanceStand
1003  StopMove
1008  Move
1016  Hello
```

含义：

- `BalanceStand`：平衡站立；
- `StopMove`：停止运动；
- `Move(vx, vy, vyaw)`：按给定速度运动；
- `Hello`：打招呼。

### 3.2 已验证 topic

控制：

```text
/api/sport/request
```

状态：

```text
/sportmodestate
/lf/sportmodestate
```

里程计：

```text
/utlidar/robot_odom
```

雷达点云：

```text
/utlidar/cloud_deskewed
/utlidar/cloud
/utlidar/voxel_map
```

注意：

- `/utlidar/cloud_deskewed` 已确认类型为 `sensor_msgs/msg/PointCloud2`，且 `frame_id: odom`；
- `/utlidar/cloud` 已确认类型为 `sensor_msgs/msg/PointCloud2`；
- `/utlidar/voxel_map` 已确认类型为 `sensor_msgs/msg/PointCloud2`，但测试时 3 秒内没有收到点，可能需要建图模块启动或特定模式。

### 3.3 `/sportmodestate` 输出内容

已验证类型：

```text
unitree_go/msg/SportModeState
```

包含：

- `stamp.sec / stamp.nanosec`
- `imu_state.quaternion`
- `imu_state.gyroscope`
- `imu_state.accelerometer`
- `imu_state.rpy`
- `mode`
- `gait_type`
- `position`
- `body_height`
- `velocity`
- `yaw_speed`
- `range_obstacle`
- `foot_force`

### 3.4 `/utlidar/robot_odom` 输出内容

已验证类型：

```text
nav_msgs/msg/Odometry
```

包含：

- `pose.pose.position`
- `pose.pose.orientation`
- `twist.twist.linear`
- `twist.twist.angular`

这个 topic 后续可用于航点闭环控制。

## 4. 今天新增的 ROS2 包

Windows 工作区根目录：

```text
D:\multi_agent_env_sensing_envcheck
```

Ubuntu ROS2 工作空间：

```text
~/unitree_go2_ws
```

复制包到 Ubuntu 的通用方式：

```bash
cd ~/unitree_go2_ws/src
cp -r /mnt/d/multi_agent_env_sensing_envcheck/<package_folder>/<package_name> .
cd ~/unitree_go2_ws
colcon build --packages-select <package_name>
source install/setup.bash
```

### 4.1 `helloworld`

原始参考示例。核心功能：

```text
发布 /api/sport/request
api_id = 1016
让 Go2 打招呼
```

它证明 ROS2 直连 Unitree API 可用。

### 4.2 `go2_safe_move`

路径：

```text
D:\multi_agent_env_sensing_envcheck\go2_safe_move\go2_safe_move
```

功能：

```text
BalanceStand
低速前进 Move(0.10, 0.0, 0.0)
StopMove
低速转向 Move(0.0, 0.0, 0.20)
StopMove
```

已实机验证成功。

复制、编译、运行：

```bash
cd ~/unitree_go2_ws/src
rm -rf go2_safe_move
cp -r /mnt/d/multi_agent_env_sensing_envcheck/go2_safe_move/go2_safe_move .
cd ~/unitree_go2_ws
colcon build --packages-select go2_safe_move
source install/setup.bash
ros2 run go2_safe_move go2_safe_move
```

安全要求：

- 空旷地面；
- 遥控器在手；
- 人员在侧后方；
- 随时准备接管或急停；
- 当前节点没有避障功能。

### 4.3 `go2_state_logger`

路径：

```text
D:\multi_agent_env_sensing_envcheck\go2_state_logger\go2_state_logger
```

功能：

```text
订阅 /sportmodestate
订阅 /utlidar/robot_odom
保存 CSV 日志
```

输出字段：

```text
time_s
sport_x, sport_y, sport_z
sport_roll, sport_pitch, sport_yaw
sport_vx, sport_vy, sport_vz
sport_yaw_speed
body_height
mode, gait_type, error_code
odom_x, odom_y, odom_z
odom_roll, odom_pitch, odom_yaw
odom_vx, odom_vy, odom_vz
odom_wx, odom_wy, odom_wz
```

注意：`SportModeState.stamp` 是 Unitree 自己的 `TimeSpec` 类型，不是 `builtin_interfaces/msg/Time`。代码里已用模板函数兼容：

```cpp
template <typename Stamp>
static double stamp_to_seconds(const Stamp & stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}
```

复制、编译、运行：

```bash
cd ~/unitree_go2_ws/src
rm -rf go2_state_logger
cp -r /mnt/d/multi_agent_env_sensing_envcheck/go2_state_logger/go2_state_logger .
cd ~/unitree_go2_ws
colcon build --packages-select go2_state_logger
source install/setup.bash
ros2 run go2_state_logger go2_state_logger --ros-args -p output_path:=safe_move_log.csv
```

和 `go2_safe_move` 同时运行：

终端 1：

```bash
cd ~/unitree_go2_ws
source install/setup.bash
ros2 run go2_state_logger go2_state_logger --ros-args -p output_path:=safe_move_log.csv
```

终端 2：

```bash
cd ~/unitree_go2_ws
source install/setup.bash
ros2 run go2_safe_move go2_safe_move
```

已实机验证成功。日志里能看到：

- 前进时 `x/y` 变化；
- 转向时 `yaw` 和 `wz` 变化；
- `StopMove` 后速度接近 0。

### 4.4 `go2_lidar_snapshot`

路径：

```text
D:\multi_agent_env_sensing_envcheck\go2_lidar_snapshot\go2_lidar_snapshot
```

当前版本为 C++，功能：

```text
订阅 /utlidar/cloud_deskewed
读取 PointCloud2 中的 x/y/z
采集一定时间或直到达到 max_points
保存 CSV
保存 PLY
```

输出：

```text
go2_lidar_points.csv
go2_lidar_points.ply
```

默认：

```text
topic = /utlidar/cloud_deskewed
duration_s = 3.0
max_points = 120000
sample_stride = 1
output_prefix = go2_lidar
```

复制、编译、运行：

```bash
cd ~/unitree_go2_ws/src
rm -rf go2_lidar_snapshot
cp -r /mnt/d/multi_agent_env_sensing_envcheck/go2_lidar_snapshot/go2_lidar_snapshot .
cd ~/unitree_go2_ws
colcon build --packages-select go2_lidar_snapshot
source install/setup.bash
ros2 run go2_lidar_snapshot go2_lidar_snapshot
```

采集 20 秒、160 万点：

```bash
ros2 run go2_lidar_snapshot go2_lidar_snapshot --ros-args -p topic:=/utlidar/cloud_deskewed -p duration_s:=20.0 -p max_points:=1600000 -p output_prefix:=go2_dense
```

注意：

- 程序达到 `max_points` 会提前保存退出；
- 若采集时转一圈，点云更完整；
- `/utlidar/cloud_deskewed` 的 `frame_id` 已确认是 `odom`。

查看点云：

```bash
CloudCompare go2_dense_points.ply
```

Ubuntu 命令是大写：

```bash
CloudCompare
```

不是：

```bash
cloudcompare
```

如果没装：

```bash
sudo apt update
sudo apt install cloudcompare
```

已测试结果：

- 20 秒；
- 160 万点；
- 转一圈；
- CloudCompare 可打开；
- 能看到比较完整的室内局部环境点云；
- 中央黑洞是机器人/雷达盲区；
- 周围结构可见。

### 4.5 `go2_lidar_filter`

路径：

```text
D:\multi_agent_env_sensing_envcheck\go2_lidar_filter\go2_lidar_filter
```

功能：

```text
读取 go2_dense_points.csv
过滤距离和高度
分离地面点和障碍物点
体素下采样
输出三类 PLY
投影障碍物到 2D 占据栅格
```

输出：

```text
go2_filtered_filtered.ply
go2_filtered_ground.ply
go2_filtered_obstacles.ply
go2_filtered_obstacles.csv
go2_filtered_occupancy.csv
go2_filtered_occupancy.pgm
go2_filtered_occupancy_meta.txt
```

三个 PLY 含义：

```text
filtered.ply   过滤后的整体有效点云
ground.ply     地面或低矮点
obstacles.ply  障碍物点
```

复制、编译：

```bash
cd ~/unitree_go2_ws/src
rm -rf go2_lidar_filter
cp -r /mnt/d/multi_agent_env_sensing_envcheck/go2_lidar_filter/go2_lidar_filter .
cd ~/unitree_go2_ws
colcon build --packages-select go2_lidar_filter
source install/setup.bash
```

运行：

```bash
ros2 run go2_lidar_filter go2_lidar_filter --ros-args -p input_csv:=go2_dense_points.csv -p output_prefix:=go2_filtered
```

如果输入是 `go2_dense_160w_points.csv`：

```bash
ros2 run go2_lidar_filter go2_lidar_filter --ros-args -p input_csv:=go2_dense_160w_points.csv -p output_prefix:=go2_filtered
```

提高 2D 占据栅格分辨率：

```bash
ros2 run go2_lidar_filter go2_lidar_filter --ros-args -p input_csv:=go2_dense_points.csv -p output_prefix:=go2_filtered_5cm -p grid_resolution_m:=0.05 -p voxel_size_m:=0.02
```

更细：

```bash
ros2 run go2_lidar_filter go2_lidar_filter --ros-args -p input_csv:=go2_dense_points.csv -p output_prefix:=go2_filtered_3cm -p grid_resolution_m:=0.03 -p voxel_size_m:=0.02
```

如果障碍物过少，降低障碍物高度阈值：

```bash
-p obstacle_min_z_m:=0.08
```

如果噪声太多，提高阈值：

```bash
-p obstacle_min_z_m:=0.25
```

目前用户反馈：

- 障碍物点云生成效果比较好；
- 2D 占据图默认 10cm 分辨率偏糊；
- 建议用 `grid_resolution_m:=0.05` 或 `0.03`。

## 5. 已完成实机里程碑

### 5.1 控制链路

已完成：

```text
ROS2 -> /api/sport/request -> Go2 Sport API
```

验证动作：

- Hello；
- BalanceStand；
- StopMove；
- Move 低速前进；
- Move 低速转向。

### 5.2 状态反馈

已完成：

```text
/sportmodestate
/utlidar/robot_odom
```

可以读取：

- 位置；
- 姿态；
- 速度；
- yaw；
- yaw_speed；
- body_height；
- mode；
- error_code。

### 5.3 运动日志

已完成：

```text
一边运行 go2_safe_move
一边运行 go2_state_logger
保存 safe_move_log.csv
```

日志中能清楚看到前进和转向。

### 5.4 雷达点云

已完成：

```text
/utlidar/cloud_deskewed -> PointCloud2 -> CSV / PLY
```

已在 CloudCompare 中打开 `.ply`，效果可用。

### 5.5 点云后处理

已完成：

```text
原始点云 -> 过滤 -> 地面/障碍物分离 -> 2D 占据栅格
```

障碍物点云效果好，2D 栅格需要调分辨率。

## 6. 常见问题和坑

### 6.1 `ros2 run go2_sport_test test` 报 No executable found

原因：可执行文件名和包名不一致。

正确命令应使用 `CMakeLists.txt` 中 `add_executable` 的名字。

例如：

```bash
ros2 run go2_safe_move go2_safe_move
```

可以查：

```bash
ros2 pkg executables <package_name>
```

### 6.2 `SportModeState.stamp` 类型错误

错误：

```text
cannot convert unitree_go::msg::TimeSpec to builtin_interfaces::msg::Time
```

原因：Unitree 的时间戳类型不是 ROS 标准时间，但字段也有 `sec/nanosec`。

解决：用模板函数读取。

### 6.3 `std::max(int, long int)` 编译错误

错误点：

```cpp
std::max(1, declare_parameter<int>(...))
```

解决：

```cpp
const int declared_sample_stride =
  static_cast<int>(this->declare_parameter<int>("sample_stride", 1));
sample_stride_ = std::max(1, declared_sample_stride);
```

### 6.4 `cloudcompare` 命令找不到

安装包叫 `cloudcompare`，可执行命令通常是：

```bash
CloudCompare
```

打开文件：

```bash
CloudCompare go2_dense_points.ply
```

### 6.5 `/utlidar/voxel_map` 没点

现象：

```text
Publisher count: 1
但采集 3 秒 No points captured
```

可能原因：

- topic 存在但当前不发布；
- 建图模块没启动；
- 需要特定 mapping 模式；
- 没有新地图更新。

排查：

```bash
ros2 topic hz /utlidar/voxel_map
```

如果没有频率，先不用它。

## 7. 建议下一步：SLAM

用户现在希望下一步部署 SLAM。

通俗解释：

```text
SLAM = 一边走一边画地图，同时知道自己在地图里的位置。
```

当前已经具备 SLAM 前置条件：

- 可控制 Go2 运动；
- 可读取 odom；
- 可读取雷达点云；
- 可保存点云；
- 可分离障碍物点；
- 可生成占据栅格。

### 7.1 第一优先级：先看 Go2 自带 `/uslam/*`

topic 列表中已有：

```text
/uslam/client_command
/uslam/cloud_map
/uslam/frontend/cloud_world_ds
/uslam/frontend/odom
/uslam/localization/cloud_world
/uslam/localization/odom
/uslam/navigation/global_path
/uslam/server_log
```

下一步不要立刻从零部署 LIO-SAM，先检查 Go2 自带 SLAM 是否已经在发数据。

建议命令：

```bash
ros2 topic info /uslam/cloud_map
ros2 topic hz /uslam/cloud_map
ros2 topic echo /uslam/cloud_map --once
```

```bash
ros2 topic info /uslam/localization/cloud_world
ros2 topic hz /uslam/localization/cloud_world
ros2 topic echo /uslam/localization/cloud_world --once
```

```bash
ros2 topic info /uslam/localization/odom
ros2 topic hz /uslam/localization/odom
ros2 topic echo /uslam/localization/odom --once
```

如果这些是：

```text
sensor_msgs/msg/PointCloud2
nav_msgs/msg/Odometry
```

并且有频率，那么下一步写：

```text
go2_slam_map_saver
```

功能：

```text
订阅 /uslam/cloud_map 或 /uslam/localization/cloud_world
保存 slam_map.ply
订阅 /uslam/localization/odom
保存轨迹 CSV
```

### 7.2 第二优先级：开源 SLAM 参考

如果 Go2 自带 SLAM 不够用，再考虑：

```text
LIO-SAM
FAST-LIO / FAST-LIO2
slam_toolbox
```

推荐顺序：

1. Go2 自带 `/uslam/*`
2. LIO-SAM / FAST-LIO 这类 3D LiDAR + IMU SLAM
3. slam_toolbox 只适合 2D LaserScan，除非先把点云转 2D scan

不要自己从零写 SLAM。

## 8. 当前最建议让新 Codex 做的任务

给新 Codex 的推荐 prompt：

```text
我们已经完成 Go2 ROS2 直连控制、状态读取、雷达点云采集和点云后处理。请先读取 UBUNTU_CODEX_HANDOFF.md，然后继续检查 /uslam/* 相关 topic，判断 Go2 自带 SLAM 是否可用。如果 /uslam/cloud_map 或 /uslam/localization/cloud_world 有 PointCloud2 数据，请写一个 C++ ROS2 包 go2_slam_map_saver，保存 SLAM 地图为 PLY，并保存 /uslam/localization/odom 轨迹为 CSV。代码风格参考已有的 go2_lidar_snapshot 和 go2_state_logger。
```

## 9. 推荐现场命令清单

### 9.1 查看 topic 类型和频率

```bash
ros2 topic info /uslam/cloud_map
ros2 topic hz /uslam/cloud_map
```

```bash
ros2 topic info /uslam/localization/cloud_world
ros2 topic hz /uslam/localization/cloud_world
```

```bash
ros2 topic info /uslam/localization/odom
ros2 topic hz /uslam/localization/odom
```

### 9.2 运行安全运动

```bash
cd ~/unitree_go2_ws
source install/setup.bash
ros2 run go2_safe_move go2_safe_move
```

### 9.3 记录状态

```bash
cd ~/unitree_go2_ws
source install/setup.bash
ros2 run go2_state_logger go2_state_logger --ros-args -p output_path:=safe_move_log.csv
```

### 9.4 采集雷达点云

```bash
cd ~/unitree_go2_ws
source install/setup.bash
ros2 run go2_lidar_snapshot go2_lidar_snapshot --ros-args -p topic:=/utlidar/cloud_deskewed -p duration_s:=20.0 -p max_points:=1600000 -p output_prefix:=go2_dense
```

### 9.5 过滤点云并生成障碍物和占据图

```bash
cd ~/unitree_go2_ws
source install/setup.bash
ros2 run go2_lidar_filter go2_lidar_filter --ros-args -p input_csv:=go2_dense_points.csv -p output_prefix:=go2_filtered_5cm -p grid_resolution_m:=0.05 -p voxel_size_m:=0.02
```

### 9.6 打开点云

```bash
CloudCompare go2_dense_points.ply
CloudCompare go2_filtered_obstacles.ply
```

## 10. 项目当前一句话总结

当前项目已经从仿真推进到 Go2 实机 ROS2 控制与感知验证阶段：已实现 ROS2 直连 Unitree API 控制运动、读取状态与里程计、采集真实雷达三维点云、提取障碍物点云并生成二维占据栅格。下一步应优先检查并接入 Go2 自带 `/uslam/*` SLAM 输出，若可用则保存 SLAM 地图和轨迹；若不可用，再考虑部署 LIO-SAM 或 FAST-LIO。

