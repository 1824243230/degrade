# ROS1 构建依赖记录

目标：Ubuntu 20.04 amd64、Noetic、GCC 9.4、C++17。

| 组件 | 固定来源 / 版本 |
| --- | --- |
| Eigen | Ubuntu `libeigen3-dev` 3.3.7，`/usr/share/eigen3/cmake` |
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
