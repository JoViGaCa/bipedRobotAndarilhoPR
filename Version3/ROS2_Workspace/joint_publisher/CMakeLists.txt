cmake_minimum_required(VERSION 3.8)
project(joint_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sensor_filter REQUIRED)
find_package(stimate_state REQUIRED)
find_package(state_space_controller REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(jointPublisher src/jointPublisher.cpp)
ament_target_dependencies(jointPublisher rclcpp trajectory_msgs)
add_executable(stateSpaceTest src/stateSpaceTest.cpp)
ament_target_dependencies(stateSpaceTest rclcpp std_msgs sensor_msgs trajectory_msgs)
add_executable(getSensorValues src/getSensorValues.cpp)
ament_target_dependencies(getSensorValues rclcpp std_msgs sensor_msgs)
add_executable(stateSpaceNav src/stateSpaceNav.cpp)
ament_target_dependencies(stateSpaceNav rclcpp std_msgs sensor_msgs trajectory_msgs)
add_executable(stateSpaceNavV2 src/stateSpaceNavV2.cpp)
ament_target_dependencies(stateSpaceNavV2 rclcpp std_msgs sensor_msgs trajectory_msgs sensor_filter stimate_state state_space_controller)
add_executable(modelKasaei2019 src/modelKasaei2019.cpp)
ament_target_dependencies(modelKasaei2019 rclcpp std_msgs sensor_msgs trajectory_msgs sensor_filter stimate_state state_space_controller)
add_executable(modelKajitaWithKalman src/modelKajitaWithKalman.cpp)
ament_target_dependencies(modelKajitaWithKalman rclcpp std_msgs sensor_msgs trajectory_msgs sensor_filter stimate_state state_space_controller)




install(TARGETS
  jointPublisher
  stateSpaceTest
  getSensorValues
  stateSpaceNav
  stateSpaceNavV2
  modelKasaei2019
  modelKajitaWithKalman
  DESTINATION lib/${PROJECT_NAME}
)

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
