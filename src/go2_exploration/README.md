# 单狗自主探索与在线建图

从未知室内环境起步。虚拟 Go2 根据速度指令移动并生成新扫描，`slam_toolbox` 根据扫描更新地图。探索节点读取地图和当前位置，在可达的已知空闲区选择能看到未知区域的观察点。Nav2 负责规划、跟随路径和避障，到点后再选择目标。场景几何只供仿真器生成雷达和检查碰撞，探索和导航节点读取的都是 ROS 话题。

## 在 Ubuntu 22.04 / ROS 2 Humble 上复现

需要 `ros-humble-navigation2`、`ros-humble-nav2-bringup`、`ros-humble-slam-toolbox`、NumPy、SciPy 和 PyYAML。可在配好这些依赖的终端执行：

```bash
cd /home/yaorz/unitree_go2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_exploration
source install/setup.bash
ros2 run go2_exploration run_sim.py --goals 3 --seconds 180
```

入口使用独立的 ROS 域 83 和回环网卡。仿真器订阅该域的 `/cmd_vel`，此入口仅启动虚拟底盘、建图、导航和探索节点。请为此试验保留域 83。输出写到 `/media/yaorz/Elements/unitree_go2/simulation/frontier/<时间>/`，启动时打印完整路径。默认路径要求 Elements 已挂载。

- `result.json` 包含每个目标、Nav2 结果、仿真位置、终点误差、移动距离、碰撞拦截次数和建图面积。
- `initial_grid.npz`、`last_grid.npz`、`map.pgm`、`map.yaml` 用于比较前后地图。面积按所有已知栅格（空闲和占据）的数量乘以单格面积计算。
- `source/`、`slam.yaml`、`nav2.yaml` 和各节点日志记录本轮实际代码与参数。

通过条件为达到指定数量的目标，按首帧地图与仿真坐标的对应关系逐个核对真实终点，误差各不超过 0.30 m，模型移动至少 0.5 m，已知面积增长至少 1 m²，碰撞拦截为零并成功保存地图。超时、无可达前沿等会写入结果。目标数是本次试验的停止条件，完整房间覆盖率需另行测量。

## 在项目地图上预览下一个目标

已有的单狗 SLAM 提供 `/map` 和 `map → odom → base_link`。在加载项目与本包环境的终端启动：

```bash
ros2 run go2_exploration frontier_explorer.py
```

在 RViz 中添加 Pose，选择 `/frontier_explorer/preview`，Fixed Frame 设为地图的 frame。默认只发布预览。节点会检验地图和 TF 的时效，考虑 `clearance_m`（默认 0.36 m），排除不通的区域和窄缝。实际 Go2 的机身、载荷与停稳距离需要单独测量，再决定余量。

仿真运行器会在 Nav2 就绪后调用 `/frontier_explorer/enable`（`std_srvs/srv/SetBool`）。启用期间同一时刻只有一个 `NavigateToPose` 目标。禁用会请求撤销当前目标；超时或丢失地图/TF 也会请求撤销，收到终态后才允许另选。已尝试目标按地图米制坐标记忆。没有候选只表示按照当前地图与余量找不到可达观察点。

## 实机接入边界

探索闭环已在二维虚拟底盘、理想里程计和模拟雷达上测试。独立的 `guarded_velocity_bridge.py` 默认处于未使能状态，默认输出为 `/guarded_velocity_bridge/preview_sport_request`。隔离消息测试验证了限速，以及命令超时、手柄输入、正常进程退出时生成停车请求。实机停车距离、断链后的行为和实际控制权仍待测量。

首次接狗先保持静止，使用已确认配对的手柄，摇杆回中。只读检查 `/wirelesscontroller` 和 `/utlidar/robot_odom` 的实际消息、各自发布端及消息间隔。用于局部避障的数据源还要保持及时更新：现有累积扫描在静止时默认约十秒刷新一次，短暂回波过滤也只用于建图。

11/18 的 Go2 能从 `/lf/lowstate.wireless_remote` 读到手柄数据。加载同学电脑的 Unitree 消息包和本包后，可选择以下预览输入：

```bash
ros2 run go2_exploration guarded_velocity_bridge.py --ros-args \
  -p handset_source:=lowstate_preview
```

此模式在启动时拒绝 `hardware_output:=true`。节点按这台设备实测的 `55 51` 包头解析 40 字节手柄字段。按键按下或任一摇杆/扳机绝对值达到 0.10 时，已使能的预览会生成一次 StopMove 并撤销使能；摇杆回中后需要重新显式使能。包头异常、轴值无效、LowState 的 tick 重复或倒退、状态流超时也会撤销使能。位布局见[宇树 SDK 的手柄结构](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/dds_wrapper/common/unitree_joystick.hpp)。

LowState 的 tick 来自机器狗。11/18 在狗安全停放、摇杆回中的条件下，现场人员关闭同一只手柄后，手柄字段连续 1,101 帧为全零，LowState 仍然更新；重新开机并确认 DL 常亮后，包头恢复为 `55 51`。使用实测过渡帧的隔离预览产生一次 StopMove，连接恢复后保持撤销使能。这一记录适用于本次设备和手柄关机场景。其他固件、遮挡或异常掉线时仍需检查是否会保持旧值。实际停稳距离和独立接管手段须由现场人员在低速试验中验证。
