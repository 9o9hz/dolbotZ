cmake_minimum_required(VERSION 3.8)
project(rmd_x8_driver)

# 의존성 패키지 로드
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# 1. 내부 파이썬 패키지(모듈) 설치 (rmd_x8_driver/__init__.py 등이 있는 폴더)
ament_python_install_package(${PROJECT_NAME})

# 2. 실행 가능한 메인 노드 스크립트 등록 및 설치 (프로토콜 의존성 파일 추가 완료)
# 인코더 및 속도 지령 통신을 담당하는 메인 파일입니다.
install(PROGRAMS
  rmd_x8_driver/rmd_x8_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rmd_x8_driver_node
)

# ★ [핵심 추가] 메인 노드가 바로 옆방에서 참조할 수 있도록 프로토콜 스크립트도 같은 실행 디렉토리에 배포
install(PROGRAMS
  rmd_x8_driver/rmd_x8_protocol.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. 설정 및 런치 파일 배포 (config, launch 폴더 통째로 share에 설치)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 4. ROS 2 패키지 인덱스에 최종 등록 (Package not found 버그 완벽 예방)
ament_package()