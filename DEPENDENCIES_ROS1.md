# ROS1 构建依赖记录

目标：Ubuntu 20.04 amd64、Noetic、GCC 9.4、C++17。

| 组件 | 固定来源 / 版本 |
| --- | --- |
| Eigen | Ubuntu `libeigen3-dev` 3.3.7，`/usr/lib/cmake/eigen3` |
| OpenCV | Ubuntu 4.2，`/usr/lib/x86_64-linux-gnu/cmake/opencv4` |
| PCL | Ubuntu 1.10，`/usr/lib/x86_64-linux-gnu/cmake/pcl` |
| Sophus | https://github.com/strasdat/Sophus ，1.22.10，`de0f8d3d92bf776271e16de56d1803940ebccab9` |
| Livox SDK2 | https://github.com/Livox-SDK/Livox-SDK2 ，`c0796f04c143143899c87a773d9f6b7136453c0b` |
| Livox ROS driver2 | https://github.com/Livox-SDK/livox_ros_driver2 ，`21445540f0d100dc86a7e6df312dd70bbdb4afdf` |
| vikit | 工作区已有 https://github.com/xuankuzcr/rpg_vikit ，`6c886c8e5d83997806e00294826d528cea3581dd` |

本地修改：Livox 驱动保留上游 ROS1 构建段，使用工作区 SDK 安装前缀；
vikit 保留已有 Pinhole、EquidistantCamera 实现，迁移至现代 Sophus 和
C++17，去除主机特定 SIMD 编译选项。SA-LIVO 不依赖 vikit_ros 的相机加载器。

Sophus 和 SDK 的准备过程由 `scripts/prepare_ros1_deps.sh` 固定，
安装前缀 `.deps/install` 不覆盖系统库。SDK 使用 GCC 9；Sophus 为头文件库。
`src/livox_ros_driver2` 和 `src/rpg_vikit` 源码及其本地补丁应随工作区一起保留。

重定位扩展：本地 KISS-Matcher 0.3.0、ROBIN 1.2.7、PMC、xenium 和
small_gicp 1.0.1 使用 `src/kiss_matcher/ROS1_SOURCES.sha256` 校验；通过
`scripts/prepare_kiss_matcher_ros1.sh` 安装至同一工作区前缀。新增系统依赖为
TBB 2020.1、FLANN 1.9.1、LZ4、yaml-cpp，以及 ROS1 std_srvs/message_generation。
第三方源码中的自动下载在本工作区构建入口禁用。KISS 修复低对应点时复用旧解的路径，
CMake 安装配置补齐 TBB/ROBIN/Eigen/OpenMP 传递依赖；不启用 `-march=native`。

缺失的 Livox 驱动可通过 `scripts/prepare_livox_ros1.sh` 从上述固定提交恢复。
下载归档 SHA256 为
`b0f25c87ce8be7bc2edcf5f81fd0e2c3a4c0f55a5f8a1a7f44bbc7ffe7fda664`，
已有完整源码目录不会被覆盖。

## Long Corridor 原始包离线验证

新增 `decode_velodyne_bag` 使用 Noetic 的 `velodyne_pointcloud`/`velodyne_msgs`，
以及其 `VLP16db.yaml` 标定文件，不依赖 ROS2 或 Python 的 LZ4 扩展。
轨迹评估使用系统 Python3 的 numpy、scipy、matplotlib。
缺少时安装 `ros-noetic-velodyne-pointcloud python3-numpy python3-scipy python3-matplotlib`。
构建只运行 `bash scripts/build_ros1.sh`，不运行 `catkin_make install`。
工作区输出为 `build/`、`devel/`；`.deps/install/` 是已有第三方依赖前缀，运行重定位库仍需保留。
