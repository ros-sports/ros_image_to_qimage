cmake_minimum_required(VERSION 3.8)
project(ros_image_to_qimage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(ros_image_to_qimage src/ros_image_to_qimage.cpp)
target_include_directories(ros_image_to_qimage PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ros_image_to_qimage
  sensor_msgs
  Qt5Widgets
  cv_bridge)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ros_image_to_qimage
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ros_image_to_qimage
)
ament_export_dependencies(Qt5Widgets)
ament_export_dependencies(sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
