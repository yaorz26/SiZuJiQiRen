# 本地构建依赖

这里保存从 Debian 软件包解出的头文件、CMake 配置及原有版权说明，
用于当前 KISS-ICP 构建。下载的 `.deb` 缓存由 Git 忽略。

| 目录 | 原软件包 |
|---|---|
| `ros_humble_sophus` | `ros-humble-sophus_1.22.9102-2jammy.20260226.005142_amd64.deb` |
| `robin_map` | `robin-map-dev_0.6.3-1_all.deb` |

版权和许可证分别位于：

- `ros_humble_sophus/usr/share/doc/ros-humble-sophus/copyright`
- `robin_map/usr/share/doc/robin-map-dev/copyright`

指定这两份本地 CMake 配置即可构建；系统仍需安装 ROS 2 Humble、Eigen3
和 TBB。完整命令见根目录 [KISS-ICP 使用说明](../KISS_ICP_NEXT_STEPS.md)。
