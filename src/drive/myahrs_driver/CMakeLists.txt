cmake_minimum_required(VERSION 3.8)
project(myahrs_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 드라이버 실행 파일 빌드
add_executable(myahrs_driver_node src/myahrs_driver_node.cpp)
target_include_directories(myahrs_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(myahrs_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)

# 인스톨 규칙 설정
install(TARGETS myahrs_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()