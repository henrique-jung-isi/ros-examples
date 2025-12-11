cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
        "msg/BoundedImage.msg"
        DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(msgs_linker ${PROJECT_NAME}_msgs rosidl_typesupport_cpp)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp sensor_msgs cv_bridge)
target_link_libraries(talker ${msgs_linker})

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp sensor_msgs cv_bridge)
target_link_libraries(listener ${msgs_linker})


target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# export CYCLONEDDS_URI="file:///home/jung/Documents/ros-examples/cyclonedds.xml"
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
