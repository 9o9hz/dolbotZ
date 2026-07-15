cmake_minimum_required(VERSION 3.8)
project(robot_arm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
# 필요한 의존성이 늘어나면 아래 find_package를 추가한다.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/manual_total_position_node.py
    scripts/gamepad_position_controller.py
    scripts/safety_manager.py
    scripts/semiauto_placeholder.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 모든 소스 파일에 저작권/라이선스가 정리될 때까지 해당 linter를 건너뛴다.
  set(ament_cmake_copyright_FOUND TRUE)
  # 모든 소스 파일에 저작권/라이선스가 정리될 때까지 cpplint를 건너뛴다.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
