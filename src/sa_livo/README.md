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

bash scripts/prepare_ros1_deps.sh
bash scripts/build_ros1.sh
```

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
