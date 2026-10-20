# KISS-Matcher ROS1 重定位验证记录

日期：2026-10-20。目标环境：Ubuntu 20.04、ROS1 Noetic、GCC 9.4、C++17，系统 Eigen 3.3.7、PCL 1.10、OpenCV 4.2、TBB 2020.1。

## 1. 构建与安装

- 本地 ROBIN/PMC/xenium、KISS-Matcher 和 small_gicp 已构建安装到 `.deps/install`。
- 第三方 987 个源码/构建文件的 SHA256 校验通过，清单为 `src/kiss_matcher/ROS1_SOURCES.sha256`。
- 常规构建及 `bash scripts/build_ros1.sh install` 通过。
- 在独立目录 `.deps/clean-catkin-build` 首次全量构建通过，并安装到 `.deps/clean-catkin-install`；该目录未使用原 `build/devel` 的包产物。
- 单独创建 CMake 工程，通过已安装的 `kiss_matcher::kiss_matcher_core` 与 `small_gicp::small_gicp` 编译、链接、运行成功，验证安装配置的传递依赖。
- 安装后的 `fastlivo_mapping` 执行 `ldd -r` 无缺失库或未解析符号，依赖系统 PCL 1.10、OpenCV 4.2、TBB `libtbb.so.2`，没有 ROS2 动态库。
- 安装后的 `liblocalization.so` 包含指向 `.deps/install/lib` 的 RUNPATH，可以找到 small_gicp。
- 新定位 launch 可通过 `roslaunch --dump-params` 解析；地图、标定和运行参数均位于 `/laserMapping` 命名空间。
- SA-LIVO 的 19 条编译命令检查通过，无 `-march=native`、ROS2 或意外的 `/usr/local/include` 路径。

独立构建复现命令：

```bash
bash scripts/build_ros1.sh --build .deps/clean-catkin-build \
  -DCATKIN_DEVEL_PREFIX="$PWD/.deps/clean-catkin-devel" \
  -DCMAKE_INSTALL_PREFIX="$PWD/.deps/clean-catkin-install" install
```

## 2. 自动化测试

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
ctest --test-dir build/sa_livo --output-on-failure
```

完整套件包含 5 个 CTest 检查项，其中 ROS 集成检查项包含两个独立场景：

| 检查项 | 覆盖内容 | 结果 |
|---|---|---|
| ROS1 pipeline | 进程内 bag 直读，无相机图像；实际 mapper 启动、失效及恢复 | 通过 |
| 同一 ROS 检查项的第二个场景 | 外部 `rosbag play --clock`、图像及联合更新路径 | 通过 |
| localization_test | 配准、元信息、歧义、失效判定、异步任务、重试、50 次恢复周期 | 通过 |
| localization_state_test | 非零外参、非 Z 轴重力、19 维协方差、恢复后 IMU 传播、两种地图后端只读 | 通过 |
| adaptive_voxel_map_v2_test | 原有自适应体素地图回归 | 通过 |
| saif_test | 原有 SAIF 回归 | 通过 |

### 2.1 配准与状态机

- 已知六自由度变换的配准结果满足平移误差 `<0.03 m`、旋转误差 `<0.01 rad` 的断言。
- 此合成案例的验证 RMSE 约 `0.057 m`、重叠率为 `1.0`；这是合成点云配准指标，不能替代真实场景测量。
- 平面退化、无有效输入和低重叠被拒绝。
- 同一子图内重复房间返回 `ambiguous global candidates`，不提交错误定位。
- 一次成功求解后传入零对应点，返回无效结果，没有复用上一次位姿。
- 支持 XYZ、XYZI、XYZRGB、FLOAT64 XYZ 以及地图 NaN 过滤；标定不匹配、元信息错误和空地图被拒绝。
- 视觉单独退化不会触发重定位；短暂 LiDAR 下降可恢复，持续无观测和持续地图支持不足会触发失效。
- 过期任务、时钟重置后的旧结果及超时结果不会提交；连续匹配失败达到上限后进入 FAILED。
- 连续 50 次状态机恢复周期通过。该项使用替身全局求解器及真实点云验证，验证任务隔离和状态复用，不等同于 50 次真实场景恢复验收。

### 2.2 实际 ROS1 节点回放

两个场景都调用实际 `fastlivo_mapping`，使用测试生成的 PCD、元信息、点云/IMU bag，以及第二场景的图像：

- 使用非零 LiDAR–IMU 外参和非 Z 轴地图重力。
- 每帧加入无效点，确认进入定位前会过滤。
- 启动时完成第一次重定位；中途注入错误点云使跟踪失效，正常点云返回后完成第二次恢复。
- 两个场景均完成两次恢复，并输出多帧有效地图位姿。
- 有效位姿的时间戳均有对应的 `status.valid=true`；输出父坐标系为 `map`。
- 最终 IMU 位置满足距已知合成真值 `<0.15 m` 的断言。
- 无相机图像的场景能够继续使用 LiDAR/IMU；带图像的场景执行联合更新。

## 3. 日志与产物

- `.deps/kiss-build.log`：匹配依赖构建。
- `.deps/localization-final-install.log`：主工作区构建及安装。
- `.deps/localization-clean-install.log`：独立构建目录及安装更新。
- `.deps/localization-ctest.log`：完整测试结果。
- `build/test_results/sa_livo/`：ROS 集成测试 XML。
- `.deps/localization-consumer.log`：独立安装库调用验证。
- `.deps/localization-installed-check.log`：安装后的动态库及 launch 检查。

## 4. 真实数据验收尚未执行

当前未提供配套真实 PCD、独立查询 bag、标定和参考真值。因此尚未测量：

- 20 个启动位置及 20 次运行中恢复的成功率；
- 真实位置/姿态误差与误接受率；
- 目标硬件上的耗时分布；
- 连续 50 次真实恢复及长时间运行的内存趋势。

这些仍按实施文档中的目标验收，不将合成测试结果表述为实机性能。
