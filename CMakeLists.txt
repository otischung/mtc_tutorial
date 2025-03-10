cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Install pre-commit hook
execute_process(
  COMMAND find ${CMAKE_CURRENT_SOURCE_DIR}/.git/hooks/ -type l -name "pre-commit" -delete || true
  COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/pre-commit.hook ${CMAKE_CURRENT_SOURCE_DIR}/.git/hooks/pre-commit
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE HOOK_INSTALL_RESULT
  OUTPUT_QUIET ERROR_QUIET
)

if (HOOK_INSTALL_RESULT)
  message(WARNING "Failed to install pre-commit hook in ${CMAKE_CURRENT_SOURCE_DIR}.")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mtc_node src/mtc_node.cpp src/mtc_task_node.cpp)
add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(hello_moveit_2 src/hello_moveit_2.cpp)
add_executable(moving_obstacle src/moving_obstacle.cpp)
add_executable(moving_obstacle_2 src/moving_obstacle_2.cpp)
add_executable(mtc src/mtc.cpp)
target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(hello_moveit_2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(moving_obstacle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(moving_obstacle_2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(mtc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mtc_node
  "moveit_task_constructor_core"
  "rclcpp"
)
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)
ament_target_dependencies(
  hello_moveit_2
  "moveit_ros_planning_interface"
  "rclcpp"
)
ament_target_dependencies(
  moving_obstacle
  rclcpp
  moveit_msgs
  geometry_msgs
  shape_msgs
  std_msgs
)
ament_target_dependencies(
  moving_obstacle_2
  rclcpp
  moveit_msgs
  geometry_msgs
  shape_msgs
  std_msgs
)
ament_target_dependencies(
  mtc
  moveit_task_constructor_core
  moveit_ros_planning_interface
  rclcpp
  moveit_msgs
  geometry_msgs
  shape_msgs
  std_msgs
)

install(TARGETS mtc_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS hello_moveit_2
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS moving_obstacle
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS moving_obstacle_2
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS mtc
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
