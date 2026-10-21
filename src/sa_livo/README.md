# SA-LIVO（ROS1 Noetic）

这是 SA-LIVO 的独立复现及工程扩展，基于 FAST-LIVO2，保留 SAIF 信息融合、
直接法视觉前端、IMU 处理和自适应体素地图。它不是论文作者的官方实现。
本工程现在使用 Ubuntu 20.04 + ROS1 Noetic，不再提供 ROS2 构建入口。

## 构建

工作区布局为 `src/sa_livo`、`src/rpg_vikit` 和 `src/livox_ros_driver2`。
从工作区根目录执行：

```bash
# 系统依赖，使用 Ubuntu 20.04 / ROS Noetic 的软件包。
sudo apt-get install gcc-9 g++-9 cmake libeigen3-dev libopencv-dev libpcl-dev \
  libboost-thread-dev libapr1-dev \
  ros-noetic-roscpp ros-noetic-rosbag ros-noetic-cv-bridge \
  ros-noetic-image-transport ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-rviz

./build.sh --deps
```

之后直接运行 `./build.sh` 增量编译；可用 `./build.sh -j4 -l4` 调整并行度，
`./build.sh --help` 查看完整用法。也可从其他目录通过绝对路径调用。

准备脚本将固定版本的 Sophus 和 Livox SDK2 安装到工作区 `.deps/install`。
构建脚本从干净环境加载 Noetic，固定 GCC 9、C++17、系统 Eigen 3.3.7、
OpenCV 4.2、PCL 1.10，并避免 `/usr/local` 的旧 Sophus 和其他 PCL/OpenCV。
不需要运行 Livox 设备或启动 ROS master 即可编译。依赖版本见工作区
`DEPENDENCIES_ROS1.md`。

```bash
# 已有算法测试；自适应体素测试在 Release 下仍启用 assert。
ctest --test-dir build/sa_livo --output-on-failure
```

## 后续运行

运行前需要对应数据的传感器话题和标定。以下命令用于数据准备完成后的验证；
编译迁移验收不包含 bag 回放、实机、轨迹精度或建图效果。

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 实机订阅；默认 AVIA、IMU、左相机话题保持原配置。
roslaunch sa_livo mapping_avia.launch use_rviz:=true

# ROS1 .bag 进程内直读；不同时创建传感器订阅。
roslaunch sa_livo mapping_avia.launch \
  direct_bag_path:=/absolute/path/to/recording.bag

# 使用 ROS1 rosbag play，分别在两个终端执行：
roslaunch sa_livo mapping_avia.launch use_sim_time:=true
rosbag play /absolute/path/to/recording.bag --clock
```

通过 `avia_params_file:=...` 和 `camera_params_file:=...` 指定其他配置。
所有 YAML 加载到 `/laserMapping` 私有命名空间；例如
`/laserMapping/common/lid_topic`。离线路径参数由 launch 的
`direct_bag_path` 参数覆盖，默认空字符串选择实时订阅。

输入支持标准 `sensor_msgs/PointCloud2` 或 ROS1
`livox_ros_driver2/CustomMsg`、`sensor_msgs/Imu`、原始或压缩主相机图像，
以及可选第二路原始相机图像。主相机压缩图像通过
`common/img_compressed_en` 选择，无需外部解压节点。
原始雷达 UDP packet 消息需要在本工程外先解码成点云。

保留输出话题 `/cloud_registered`、`/aft_mapped_to_init`、`/path`、`/rgb_img`
等，以及 `camera_init` 和 `aft_mapped` 坐标系。轨迹和 PCD 仍写入包目录的
`Log/result` 和 `Log/PCD`，相应开关由原配置控制。

## 算法与来源

- SAIF：联合信息矩阵分解、退化方向衰减和联合 InEKF 更新。
- SA-VIO：直接光度信息融合、滑窗、亮度模型和观测门限。
- 地图：平面统计、自适应邻域、多尺度支持及可选旧版地图路径。
- 现有实验参数、标定数值及算法更新顺序保留；配置文件仅迁移 ROS 参数层次。
- 原 ROS2 来源：integralrobotics/FAST-LIVO2，commit d4ad051（2025-04-09）；
  本仓库随后增加 SA-LIVO 复现模块。本次仅转换 ROS 平台及必要依赖接口。

原 README 引用的 `datasets/` 运行和评估脚本不在当前工作区中，本次未重建。
`RESULTS.md` 的历史结果不是本次 ROS1 迁移后的重新测量结果。

继承代码遵循 GPL-2.0-only；外部依赖保留各自许可。
vikit 声明 GPLv3，组合二进制的再分发需要另行确认许可兼容性。

## 固定地图定位与失效恢复

已增加 ROS1 KISS-Matcher 启动重定位和运行中恢复。使用已有 PCD 及其
`.yaml` 元信息，设备在启动和恢复时停稳；质量不可靠期间暂停有效位姿输出。
相机缺失时允许 LiDAR/IMU 定位，固定地图禁止实时写入。

```bash
bash scripts/prepare_ros1_deps.sh
bash scripts/build_ros1.sh
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sa_livo localization.launch map_path:=/absolute/path/map.pcd \
  calibration_id:=default sensor_params_file:=/absolute/path/sensors.yaml
rostopic echo /localization/status
```

本次导出的地图会附带实际重力方向、地图版本和标定标识。旧 PCD 需补齐可信
元信息；不能从默认坐标轴推断重力。完整构建、接口、回放和验收说明见工作区
`KISS_MATCHER_ROS1实施方案.md`。

## Long Corridor 无图像回放

`mapping_long_corridor.launch` 使用 `config/long_corridor.yaml`，强制 `img_en: 0`。
工作区脚本 `bash scripts/validate_long_corridor.sh` 将原始 `/velodyne_packets`
解码为标准点云，并对整段 LiDAR/IMU 回放输出轨迹及参考误差。初始运动期间等待
IMU 稳定窗口后再开始建图，参数及实测结果见工作区 `Long_Corridor_ROS1验证记录.md`。

该 launch 的扩展评估文件 `raw_trajectory.tum` 和 `livo_quality.csv` 默认写入
`Log/result/<seq_name>/<本次运行的唯一标识>/`，允许重复启动。也可传入
`output_dir:=/absolute/path/to/new_run` 指定目录；若其中已有 `raw_trajectory.tum`，
程序会拒绝覆盖，需换用新目录。

## LIVO 与 BTC 回环

新增单次联合更新后的 BTC 子图回环、Ceres 位姿图优化、`map → camera_init`
校正、历史轨迹和全局地图重建。默认 `loop_closure/enabled: false`。
后端使用独立快照和线程；不会将全局修正写入前端状态及局部地图。

```bash
bash scripts/prepare_ceres_ros1.sh
bash scripts/build_ros1.sh
# 默认使用用户确认的原 Long Corridor 数据及 databag 中的 MEI 标定和参考。
bash scripts/run_livo_btc.sh --repetitions 3
```

脚本会先解码并保留图像，将 MEI 图像校正为针孔模型，按 header 采样为 10 Hz，
再依次运行三次 LIVO 和三次 LIVO＋BTC，分别输出到
`validation/long_corridor_livo_btc/livo/` 与 `livo_btc/`。
输入、输出和复现说明见工作区 `LIVO_BTC_ROS1实施与验证.md`。

## Corridor01：Avia＋IMU＋RGB 的 LIVO 回放

用户已确认 `databag/Corridor01.bag` 使用 `/livox/avia/lidar`、`/livox/avia/imu`
及 `/camera/color/image_raw/compressed`，并确认 `databag/传感器标定参数.md`
同样适用于这套安装关系。专用 profile 为 `config/corridor01_avia_livo_profile.yaml`。
当前数据和标定文件已移到工作区旁的 `../lidarbag/其他包/`，profile 中的路径
相对于工作区根目录。
专用脚本默认启用 BTC 回环；可传入 `loop_enabled:=false` 运行单独的 LIVO。

```bash
cd /home/hyh/salivo
bash scripts/run_corridor01_livo.sh use_rviz:=true
# 同时启用已接入的 BTC 后端：
bash scripts/run_corridor01_livo.sh use_rviz:=true loop_enabled:=true
```

首次自动从原 bag 生成三传感器派生输入，完成 MEI 校正、10 Hz 图像采样及 Livox
消息类型统一，之后复用经校验的缓存。脚本直接启动 LIVO 并在进程内读取该数据，
不需要另开 `rosbag play`；`Ctrl+C` 停止。无界面时省略 `use_rviz:=true`。
结果保存在 `validation/corridor01_avia_livo/runs/` 的独立目录。
如需另开终端播包，使用 `bash scripts/run_corridor01_livo.sh --external-bag use_rviz:=true
loop_enabled:=true`；等待节点启动后，在另一终端执行
`rosbag play --clock --delay=2 validation/corridor01_avia_livo/input/normalized_livo.bag`。
该模式自动启用模拟时钟，播放的是准备后的派生输入。完整命令见工作区 `运行指南.md`。
建图结束会导出带重力及标定元信息的 `optimized_map.pcd`；使用
`bash scripts/run_corridor01_livo.sh --localize /absolute/path/optimized_map.pcd use_rviz:=true`
可对同一输入运行固定地图启动重定位和失效恢复。该模式与在线回环建图分别运行。
Corridor01 完整回放已接受 19 条回环约束；实际包头的全局重定位尚未得到有效位姿，
结果与限制见 `文档/Corridor01回环与重定位验证.md`，定位是否可用以状态的 `valid` 为准。
Corridor01 没有已确认的匹配参考轨迹，此入口不读取旧 Long Corridor 的 CSV。
专用静止初始化加速度标准差门限为 0.30 m/s²，依据开头静止段约 0.19 m/s² 的实测
向量噪声设置；原 Long Corridor 的 0.10 门限保持不变。已对前 30 s 完成启动验证，
输出 288 帧，80 帧有非零视觉融合贡献，无非有限状态。记录位于
`validation/corridor01_avia_livo/smoke_30s_initialized/smoke_summary.json`；不代表全包精度验收。
