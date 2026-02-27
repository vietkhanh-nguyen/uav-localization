cmake_minimum_required(VERSION 3.5)
project(ros2_orb_slam3)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-w -O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# Sửa lại dòng này để lấy đường dẫn động dựa trên vị trí hiện tại
set(ORB_SLAM3_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3")

include_directories(
    ${ORB_SLAM3_ROOT_DIR}
    ${ORB_SLAM3_ROOT_DIR}/include
    ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
    include
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(mono_node_cpp src/common.cpp src/mono_node.cpp)

ament_target_dependencies(mono_node_cpp 
    rclcpp 
    std_msgs 
    sensor_msgs 
    geometry_msgs
    cv_bridge
    message_filters
)

target_link_libraries(mono_node_cpp
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib/libg2o.so
)

install(TARGETS
  mono_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
