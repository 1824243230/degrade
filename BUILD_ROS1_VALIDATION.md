# ROS1 迁移构建验收

验收日期：2026-10-18。
环境：Ubuntu 20.04.6 amd64、ROS1 Noetic、GCC 9.4、C++17。

## 结果

- `bash scripts/build_ros1.sh`：通过；生成 `fastlivo_mapping` 和全部 8 个 SA-LIVO 库。
- `bash scripts/build_ros1.sh install`：通过；安装到工作区 `install/`。
- Livox ROS1 驱动可执行文件、自定义消息、vikit_common 和 vikit_ros：编译、链接通过。
- `ctest --test-dir build/sa_livo --output-on-failure`：2/2 通过。
- 自适应体素测试编译参数包含 `-UNDEBUG`，Release 下 assert 有效。
- 对主程序、8 个算法/工具库、2 个 vikit 库和 Livox 驱动执行 `ldd -r`：
  在 Noetic 环境中没有缺失库或未解析符号，没有 ROS2 动态库；
  OpenCV 为系统 4.2，PCL 为系统 1.10。
- 45 条工程编译命令均使用 GCC 9，无 ROS2 或 `/usr/local/include` 头文件路径。
- 两个 ROS1 launch 均通过 `roslaunch --dump-params` 离线解析。
- 14 个 YAML 的参数值与迁移前一致，仅移除 ROS2 参数外层和 QoS 参数。
- 132 个节点参数及 66 个地图参数已迁移至 ROS1 的分组路径。
- SAIF、SA-VIO、自适应体素实现及其两个测试文件与迁移前内容一致。

## 复现

在工作区根目录执行：

```bash
bash scripts/prepare_ros1_deps.sh
bash scripts/build_ros1.sh
ctest --test-dir build/sa_livo --output-on-failure
```

动态依赖检查需要与正常 ROS1 使用方式一样加载 Noetic 和工作区：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
ldd -r devel/lib/sa_livo/fastlivo_mapping
roslaunch --dump-params sa_livo mapping_avia.launch
roslaunch --dump-params sa_livo mapping_avia_marslvig.launch
```

主要产物：

- `devel/lib/sa_livo/fastlivo_mapping`
- `devel/lib/liblaser_mapping.so` 及其余算法库
- `devel/lib/livox_ros_driver2/livox_ros_driver2_node`
- `install/lib/sa_livo/fastlivo_mapping`

本机检查日志保存在 `.deps/catkin-build.log`、`.deps/catkin-install.log`、
`.deps/ldd-validation.log` 和 `.deps/launch-params.yaml`。

本次没有回放 rosbag、连接传感器或评估轨迹/地图。实时订阅和 ROS1 .bag
直读已完成源码迁移，实际运行效果需要在数据和标定齐备后验证。
