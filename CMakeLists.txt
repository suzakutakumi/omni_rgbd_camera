cmake_minimum_required(VERSION 3.5)
project(omni_rgbd_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common io)
find_package(PCL 1.13 REQUIRED)
# find_package(Eigen REQUIRED)

find_package(Boost REQUIRED)
find_package(choreonoid REQUIRED)

include_directories(
  include
)
include_directories(${PCL_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

add_subdirectory(src)

choreonoid_add_simple_controller(OmniRGBDCameraNode src/omni_rgbd_camera_node.cpp src/PointCloud.cpp)
ament_target_dependencies(OmniRGBDCameraNode
    rclcpp
    sensor_msgs
    pcl_conversions
    PCL
)
target_link_libraries(OmniRGBDCameraNode ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
