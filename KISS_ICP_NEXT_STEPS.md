# Go2 KISS-ICP 构建与运行

KISS-ICP 已作为子模块登记在 `src/kiss-icp`，固定上游提交
`1ffa7d7512f10bfc8b1185095011fa31184019e3`。新克隆使用
`--recurse-submodules`；已有工作区执行：

```bash
git submodule update --init --recursive
```

## 构建建图相关包

在工作区根目录运行。先加载 ROS 2 Humble；如需其他 Unitree 接口包，
另行加载本机已验证的 Go2 环境。系统需要 ROS 依赖、Eigen3 与 TBB。
Sophus 和 robin-map 使用仓库内的头文件及 CMake 配置：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  kiss_icp go2_cloud_map_projector go2_kiss_icp_bringup \
  go2_tf_bridge go2_slam_bringup \
  --cmake-args \
  "-DSophus_DIR=$PWD/vendor/ros_humble_sophus/opt/ros/humble/share/sophus/cmake" \
  "-Dtsl-robin-map_DIR=$PWD/vendor/robin_map/usr/share/cmake/tsl-robin-map"
source install/setup.bash
```

## 输入与启动

当前 Go2 KISS-ICP 启动配置默认订阅 `/utlidar/cloud_base`。运行前确认
该话题确实发布、点云有效且坐标关系适合里程计输入；默认配置不表示
该话题已存在。使用其他已验证话题时通过 `cloud_topic` 显式指定。

```bash
ros2 launch go2_kiss_icp_bringup go2_kiss_icp.launch.py
```

例如，在已确认 `/utlidar/cloud_base` 有效时，可显式写成：

```bash
ros2 launch go2_kiss_icp_bringup go2_kiss_icp.launch.py \
  cloud_topic:=/utlidar/cloud_base
```

RViz 的 Fixed Frame 使用 `kiss_odom`，添加：

- `TF`
- `PointCloud2`：`/kiss/local_map`
- `Odometry`：`/kiss/odometry`
- `Map`：`/kiss/map_from_cloud`

源码中的可执行程序为 `kiss_icp_node`。如果安装后找不到程序或话题，
先检查 `ros2 pkg executables kiss_icp`、当前加载的工作区和实际发布话题。
实机输入、坐标、时间与建图质量仍需按现场检查表验证。
