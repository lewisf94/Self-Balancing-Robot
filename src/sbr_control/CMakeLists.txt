cmake_minimum_required(VERSION 3.8)
project(sbr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sbr_msgs REQUIRED)

# --- Pure (ROS-free) control logic -------------------------------------------
add_library(balance_lib STATIC
  src/pid.cpp
  src/balance_controller.cpp
)
target_include_directories(balance_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(balance_lib PUBLIC cxx_std_17)

# --- ROS node ----------------------------------------------------------------
add_executable(balance_controller_node src/balance_controller_node.cpp)
target_link_libraries(balance_controller_node balance_lib)
ament_target_dependencies(balance_controller_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs sbr_msgs)

install(TARGETS balance_controller_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

# --- Tests -------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_balance test/test_balance_controller.cpp)
  target_link_libraries(test_balance balance_lib)
  ament_add_gtest(test_pid test/test_pid.cpp)
  target_link_libraries(test_pid balance_lib)
endif()

ament_package()
