cmake_minimum_required(VERSION 3.5)
project(pr_storage)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)

find_package(pr_msgs REQUIRED)

include_directories(include)

set(node_plugins "")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


### Recorder node ###

add_library(single_recorder SHARED
  src/single_recorder.cpp)

target_compile_definitions(single_recorder
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(single_recorder
  "rclcpp"
  "rclcpp_components"
  "pr_msgs")

rclcpp_components_register_nodes(single_recorder "pr_storage::SingleRecorder")
set(node_plugins "${node_plugins}pr_storage::SingleRecorder;$<TARGET_FILE:single_recorder>\n")


### Mocap Recorder node ###

add_library(mocap_recorder SHARED
  src/mocap_recorder.cpp)

target_compile_definitions(mocap_recorder
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(mocap_recorder
  "rclcpp"
  "rclcpp_components"
  "pr_msgs")

rclcpp_components_register_nodes(mocap_recorder "pr_storage::MocapRecorder")
set(node_plugins "${node_plugins}pr_storage::MocapRecorder;$<TARGET_FILE:mocap_recorder>\n")


### OTS Recorder node ###

add_library(ots_recorder SHARED
  src/ots_recorder.cpp)

target_compile_definitions(ots_recorder
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(ots_recorder
  "rclcpp"
  "rclcpp_components"
  "pr_msgs")

rclcpp_components_register_nodes(ots_recorder "pr_storage::OTSRecorder")
set(node_plugins "${node_plugins}pr_storage::OTSRecorder;$<TARGET_FILE:ots_recorder>\n")


### Number Recorder node ###

add_library(num_recorder SHARED
  src/number_recorder.cpp)

target_compile_definitions(num_recorder
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(num_recorder
  "rclcpp"
  "rclcpp_components"
  "pr_msgs")

rclcpp_components_register_nodes(num_recorder "pr_storage::NumberRecorder")
set(node_plugins "${node_plugins}pr_storage::NumberRecorder;$<TARGET_FILE:num_recorder>\n")


### Installation ###

install(TARGETS
  single_recorder
  mocap_recorder
  ots_recorder
  num_recorder
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


### Install launch files ###
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
