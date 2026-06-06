cmake_minimum_required(VERSION 3.8)
project(orbslam3_ros)

# -----------------------------------------------------------------------------
# 1. Global build policy
# -----------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(OpenGL_GL_PREFERENCE GLVND)

include(GNUInstallDirs)

# -----------------------------------------------------------------------------
# 2. ROS 2 and system dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# Sophus is shipped with the local ORB_SLAM3 tree.
set(ORB_SLAM3_ROOT "$ENV{HOME}/tools/ORB_SLAM3" CACHE PATH "ORB_SLAM3 root directory")
set(ORB_SLAM3_PANGOLIN_BUILD_DIR "$ENV{HOME}/tools/Pangolin/build/src" CACHE PATH "Pangolin build/src directory")
set(ORB_SLAM3_SOPHUS_BUILD_DIR "${ORB_SLAM3_ROOT}/Thirdparty/Sophus/build" CACHE PATH "Sophus build directory")

find_package(Sophus CONFIG REQUIRED
  PATHS "${ORB_SLAM3_SOPHUS_BUILD_DIR}"
  NO_DEFAULT_PATH
)

# -----------------------------------------------------------------------------
# 3. ORB_SLAM3 local installation layout
# -----------------------------------------------------------------------------
set(ORB_SLAM3_ALIAS_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

find_path(ORB_SLAM3_INCLUDE_DIR
  NAMES System.h
  PATHS "${ORB_SLAM3_ROOT}/include"
  REQUIRED
  NO_DEFAULT_PATH
)

find_path(ORB_SLAM3_DBOW2_INCLUDE_DIR
  NAMES DBoW2/BowVector.h
  PATHS "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2"
  REQUIRED
  NO_DEFAULT_PATH
)

find_path(ORB_SLAM3_G2O_INCLUDE_DIR
  NAMES g2o/core/base_vertex.h
  PATHS "${ORB_SLAM3_ROOT}/Thirdparty/g2o"
  REQUIRED
  NO_DEFAULT_PATH
)

find_path(ORB_SLAM3_CAMERA_MODELS_INCLUDE_DIR
  NAMES GeometricCamera.h
  PATHS "${ORB_SLAM3_ROOT}/include/CameraModels"
  REQUIRED
  NO_DEFAULT_PATH
)

find_library(ORB_SLAM3_LIBRARY
  NAMES ORB_SLAM3
  PATHS "${ORB_SLAM3_ROOT}/lib"
  REQUIRED
  NO_DEFAULT_PATH
)

find_library(ORB_SLAM3_PANGOLIN_LIBRARY
  NAMES pangolin
  PATHS "${ORB_SLAM3_PANGOLIN_BUILD_DIR}"
  REQUIRED
  NO_DEFAULT_PATH
)

find_library(ORB_SLAM3_DBOW2_LIBRARY
  NAMES DBoW2
  PATHS "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib"
  REQUIRED
  NO_DEFAULT_PATH
)

find_library(ORB_SLAM3_G2O_LIBRARY
  NAMES g2o
  PATHS "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"
  REQUIRED
  NO_DEFAULT_PATH
)

set(ORB_SLAM3_REALSENSE2_LIBRARY
  "/opt/ros/humble/lib/x86_64-linux-gnu/librealsense2.so.2.57"
  CACHE FILEPATH "realsense2 shared library"
)
if(NOT EXISTS "${ORB_SLAM3_REALSENSE2_LIBRARY}")
  message(FATAL_ERROR "realsense2 library not found: ${ORB_SLAM3_REALSENSE2_LIBRARY}")
endif()

find_library(ORB_SLAM3_CRYPTO_LIBRARY
  NAMES crypto
  REQUIRED
)

# -----------------------------------------------------------------------------
# 4. Imported ORB_SLAM3 target
# -----------------------------------------------------------------------------
add_library(ORB_SLAM3 SHARED IMPORTED GLOBAL)
set_target_properties(ORB_SLAM3 PROPERTIES
  IMPORTED_LOCATION "${ORB_SLAM3_LIBRARY}"
  INTERFACE_LINK_LIBRARIES
    "Sophus::Sophus;${ORB_SLAM3_PANGOLIN_LIBRARY};${ORB_SLAM3_DBOW2_LIBRARY};${ORB_SLAM3_G2O_LIBRARY};${ORB_SLAM3_REALSENSE2_LIBRARY};${ORB_SLAM3_CRYPTO_LIBRARY};${OpenCV_LIBS};Boost::serialization;OpenGL::OpenGL;GLEW::GLEW"
)

# -----------------------------------------------------------------------------
# 5. Project target helper
#    - 后续新增节点，只要把目标名传进来即可。
#    - 统一使用 #include "ORB_SLAM3/System.h"。
# -----------------------------------------------------------------------------
function(orbslam3_ros_link_target target_name)
  target_compile_features(${target_name} PRIVATE cxx_std_17)
  target_include_directories(${target_name} PRIVATE
    "${ORB_SLAM3_ALIAS_INCLUDE_DIR}"
    "${ORB_SLAM3_ROOT}"
    "${ORB_SLAM3_INCLUDE_DIR}"
    "${ORB_SLAM3_CAMERA_MODELS_INCLUDE_DIR}"
    "${ORB_SLAM3_DBOW2_INCLUDE_DIR}"
    "${ORB_SLAM3_G2O_INCLUDE_DIR}"
  )
  target_link_libraries(${target_name} ORB_SLAM3)
  set_target_properties(${target_name} PROPERTIES
    BUILD_RPATH
      "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_PANGOLIN_BUILD_DIR};${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib;/opt/ros/humble/lib/x86_64-linux-gnu"
    INSTALL_RPATH
      "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_PANGOLIN_BUILD_DIR};${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib;/opt/ros/humble/lib/x86_64-linux-gnu"
    INSTALL_RPATH_USE_LINK_PATH TRUE
  )
  ament_target_dependencies(${target_name}
    rclcpp
    rcl_interfaces
    sensor_msgs
    std_msgs
    message_filters
    image_transport
    cv_bridge
    geometry_msgs
    tf2
    tf2_ros
    nav_msgs
  )
endfunction()

# -----------------------------------------------------------------------------
# 6. Executable targets
#    - realtime 与 dataset 入口分离，避免把不同运行模式混在同一个节点里。
# -----------------------------------------------------------------------------
set(ORB_SLAM3_ROS_REALTIME_NODE_SOURCES
  src/node/node_base.cpp
  src/node/main.cpp
  src/node/rgbd_node.cpp
  src/node/publishers.cpp
  src/slam/rgbd_slam.cpp
)

add_executable(${PROJECT_NAME}_realtime_node
  ${ORB_SLAM3_ROS_REALTIME_NODE_SOURCES}
)

orbslam3_ros_link_target(${PROJECT_NAME}_realtime_node)

set(ORB_SLAM3_ROS_MULTIMODE_NODE_SOURCES
  src/node/node_base.cpp
  src/node/main_multimode.cpp
  src/node/multimode_node.cpp
  src/node/publishers.cpp
  src/slam/system_slam.cpp
)

add_executable(${PROJECT_NAME}_multimode_node
  ${ORB_SLAM3_ROS_MULTIMODE_NODE_SOURCES}
)

orbslam3_ros_link_target(${PROJECT_NAME}_multimode_node)

set(ORB_SLAM3_ROS_DATASET_NODE_SOURCES
  src/node/node_base.cpp
  src/node/main_dataset.cpp
  src/node/rgbd_dataset_node.cpp
  src/node/publishers.cpp
  src/slam/rgbd_slam.cpp
  src/dataset/tum_dataset_loader.cpp
)

add_executable(${PROJECT_NAME}_dataset_node
  ${ORB_SLAM3_ROS_DATASET_NODE_SOURCES}
)

orbslam3_ros_link_target(${PROJECT_NAME}_dataset_node)

install(TARGETS
  ${PROJECT_NAME}_realtime_node
  ${PROJECT_NAME}_dataset_node
  ${PROJECT_NAME}_multimode_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  PATTERN ".gitkeep" EXCLUDE
)

install(DIRECTORY vocabulary/
  DESTINATION share/${PROJECT_NAME}/vocabulary
  PATTERN ".gitkeep" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN ".gitkeep" EXCLUDE
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  PATTERN ".gitkeep" EXCLUDE
)

# -----------------------------------------------------------------------------
# 7. Future layout suggestions
#    - include/orbslam3_ros/      public headers
#    - src/node/                  ROS node entry and callbacks
#    - src/slam/                  ORB_SLAM3 wrapper / adapter
#    - src/utils/                 timestamp / transform / image helpers
#    - config/                    yaml params
#    - launch/                    launch files
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# 8. Testing
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
