cmake_minimum_required(VERSION 3.8)
project(hik_camera)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

# MVS SDK（系统级安装）
include_directories(/opt/MVS/include)

ament_auto_add_executable(hik_camera_node src/hik_camera_node.cpp)
target_link_libraries(hik_camera_node
  ${OpenCV_LIBS}
  /opt/MVS/lib/64/libMvCameraControl.so
)

install(TARGETS hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
