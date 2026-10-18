cmake_minimum_required(VERSION 3.10)
project(sa_livo LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp rosbag sensor_msgs geometry_msgs nav_msgs visualization_msgs std_msgs
  tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions cv_bridge image_transport
  livox_ros_driver2 vikit_common)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs calib3d highgui)
find_package(Sophus 1.22.10 REQUIRED CONFIG)
get_target_property(Eigen3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(Sophus_INCLUDE_DIRS Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(mimalloc QUIET)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vio lio pre imu_proc laser_mapping utils saif sa_vio
  CATKIN_DEPENDS roscpp rosbag sensor_msgs geometry_msgs nav_msgs visualization_msgs
    std_msgs tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions cv_bridge
    image_transport livox_ros_driver2 vikit_common
  DEPENDS Eigen3 PCL OpenCV Sophus)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_compile_definitions(ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
# Keep the Eigen/PCL alignment ABI used by Ubuntu binary packages (no -march=native).
include(ProcessorCount)
ProcessorCount(SA_LIVO_CPU_COUNT)
if(SA_LIVO_CPU_COUNT GREATER 4)
  set(SA_LIVO_CPU_COUNT 4)
endif()
if(SA_LIVO_CPU_COUNT GREATER 1)
  add_compile_definitions(MP_EN MP_PROC_NUM=${SA_LIVO_CPU_COUNT})
else()
  add_compile_definitions(MP_PROC_NUM=1)
endif()

add_library(utils src/utils.cpp)
add_library(saif src/saif.cpp)
add_library(lio src/voxel_map.cpp)
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(sa_vio src/sa_vio.cpp)
add_library(laser_mapping src/LIVMapper.cpp)
target_link_libraries(saif PUBLIC Eigen3::Eigen)
foreach(target lio vio pre imu_proc sa_vio laser_mapping)
  target_link_libraries(${target} PUBLIC ${roscpp_LIBRARIES} ${rosbag_LIBRARIES}
    ${tf2_ros_LIBRARIES} ${cv_bridge_LIBRARIES} ${image_transport_LIBRARIES}
    ${vikit_common_LIBRARIES} ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES} Eigen3::Eigen Sophus::Sophus OpenMP::OpenMP_CXX Threads::Threads)
  add_dependencies(${target} ${catkin_EXPORTED_TARGETS})
endforeach()
target_link_libraries(lio PUBLIC utils saif)
target_link_libraries(vio PUBLIC lio)
target_link_libraries(pre PUBLIC utils)
target_link_libraries(imu_proc PUBLIC utils)
target_link_libraries(sa_vio PUBLIC saif)
target_link_libraries(laser_mapping PUBLIC vio lio sa_vio pre imu_proc ${Boost_LIBRARIES})

add_executable(fastlivo_mapping src/main.cpp)
target_link_libraries(fastlivo_mapping PRIVATE laser_mapping)
add_dependencies(fastlivo_mapping ${catkin_EXPORTED_TARGETS})
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping PRIVATE mimalloc)
endif()

install(TARGETS fastlivo_mapping utils saif lio vio pre imu_proc sa_vio laser_mapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config launch rviz_cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_executable(adaptive_voxel_map_v2_test tests/adaptive_voxel_map_v2_test.cpp)
  target_include_directories(adaptive_voxel_map_v2_test PRIVATE include)
  target_link_libraries(adaptive_voxel_map_v2_test PRIVATE Eigen3::Eigen OpenMP::OpenMP_CXX)
  target_compile_options(adaptive_voxel_map_v2_test PRIVATE -UNDEBUG)
  add_test(NAME adaptive_voxel_map_v2_test COMMAND adaptive_voxel_map_v2_test)
  add_executable(saif_test tests/saif_test.cpp)
  target_link_libraries(saif_test PRIVATE saif)
  target_compile_options(saif_test PRIVATE -UNDEBUG)
  add_test(NAME saif_test COMMAND saif_test)
endif()
