cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

# ==============================================================================
# 1. 패키지 의존성 선언
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)  # stability_monitor_node의 /speed_limit 발행용
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)        # IMU 쿼터니언 변환 계산용
find_package(diagnostic_msgs REQUIRED)  # /wheel/motor_status 구독용

# ==============================================================================
# 2. 빌드 타겟 정의 (Executables & Dependencies)
# ==============================================================================

# [노드 1] 복합 경사 및 전류 안전 보상형 안정성 감시 노드
add_executable(stability_monitor_node
  src/stability_monitor_node.cpp
)
ament_target_dependencies(stability_monitor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav2_msgs
  tf2
  diagnostic_msgs
)

# [노드 2] 수동 모드 조이스틱 멀티플렉서 노드
add_executable(joy_mux_node
  src/joy_mux_node.cpp
)
ament_target_dependencies(joy_mux_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# [노드 3] Manual-drive 전용 소프트 전류 램프 노드 (22A 연속 감쇠, 45A/3초 하드컷오프와는 별개 레이어)
add_executable(current_ramp_node
  src/current_ramp_node.cpp
)
ament_target_dependencies(current_ramp_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

# ==============================================================================
# 3. 인스톨 및 배포 규칙 설정 (Install Rules)
# ==============================================================================

# 생성된 세 개의 노드 실행 바이너리를 모두 install 타겟에 등록
install(TARGETS
  stability_monitor_node
  joy_mux_node
  current_ramp_node
  DESTINATION lib/${PROJECT_NAME}
)

# 디렉토리 설치 규칙 유지 (launch 폴더 패키지 공유 폴더 배포)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()