cmake_minimum_required(VERSION 3.8)
project(value_iteration3)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(planner STATIC
  src/planner.cpp src/actions.cpp src/global_planner.cpp src/local_planner.cpp)
target_include_directories(planner PUBLIC include)
target_link_libraries(planner PUBLIC Threads::Threads)
target_compile_features(planner PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_executable(vi_node src/vi_node.cpp)
target_link_libraries(vi_node planner)
ament_target_dependencies(vi_node
  rclcpp ament_index_cpp nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vi_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(planner_test src/test_planner.cpp)
target_link_libraries(planner_test planner)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

install(TARGETS vi_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_test(NAME planner_test COMMAND planner_test)
endif()

ament_package()
