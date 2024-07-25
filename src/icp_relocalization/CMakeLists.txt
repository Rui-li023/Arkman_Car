cmake_minimum_required(VERSION 2.8.3)
project(icp_relocalization)

# 查找catkin的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
)

find_package(Eigen3 REQUIRED)

## 设置 C++ 标准为 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包的编译设置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES icp_relocalization icp_relocalization
  CATKIN_DEPENDS roscpp std_msgs pcl_ros pcl_conversions
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(icp_node src/icp_relocalization.cpp)

# 链接库
target_link_libraries(icp_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)