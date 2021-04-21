cmake_minimum_required(VERSION 3.5)
project(pr_3dof_storage)

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

find_package(pr_3dof_msgs REQUIRED)

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

### 3DOF Recorder node ###

add_library(single_recorder SHARED
  src/single_recorder.cpp)

target_compile_definitions(single_recorder
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(single_recorder
  "rclcpp"
  "rclcpp_components"
  "pr_3dof_msgs")

rclcpp_components_register_nodes(single_recorder "pr_3dof_storage::SingleRecorder")
set(node_plugins "${node_plugins}pr_3dof_storage::SingleRecorder;$<TARGET_FILE:single_recorder>\n")

### 3DOF Long Array Recorder node ###

add_library(long_array_recorder SHARED
  src/long_array_recorder.cpp)

target_compile_definitions(long_array_recorder
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(long_array_recorder
  "rclcpp"
  "rclcpp_components"
  "pr_3dof_msgs")

rclcpp_components_register_nodes(long_array_recorder "pr_3dof_storage::LongArrayRecorder")
set(node_plugins "${node_plugins}pr_3dof_storage::LongArrayRecorder;$<TARGET_FILE:long_array_recorder>\n")


### Installation ###

install(TARGETS
  single_recorder
  long_array_recorder
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


### Install launch files ###
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
