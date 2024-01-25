cmake_minimum_required(VERSION 3.8)
project(gen3_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)


include_directories(
  include
)

add_executable(robot_controller 
src/main_loop.cpp 
src/gen3_interface.cpp 
src/controller.cpp
src/util.cpp)

ament_target_dependencies(robot_controller rclcpp geometry_msgs sensor_msgs std_msgs controller_manager_msgs Eigen3)
target_link_libraries(robot_controller rbdl rbdl_urdfreader)

add_executable(robot_test 
src/main_test.cpp 
)

ament_target_dependencies(robot_test rclcpp geometry_msgs sensor_msgs std_msgs controller_manager_msgs Eigen3)
target_link_libraries(robot_test rbdl rbdl_urdfreader)




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

install(TARGETS robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS robot_test
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
