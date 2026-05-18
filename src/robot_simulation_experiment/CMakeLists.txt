cmake_minimum_required(VERSION 3.8)
project(robot_simulation_experiment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

install(PROGRAMS
  scripts/control_eye_in_hand_ros2.py
  scripts/human_walking_stable_step_ros2.py
  scripts/humanoid_walk_only_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/asimo_walker/main.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME asimo_style_zmp_walker
)

install(CODE "execute_process(COMMAND /usr/bin/sed -i \"1s|.*|#!/usr/bin/python3|\" \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/asimo_style_zmp_walker\")")

install(DIRECTORY
  scripts/asimo_walker/
  DESTINATION lib/${PROJECT_NAME}/asimo_walker
  FILES_MATCHING PATTERN "*.py"
  PATTERN "main.py" EXCLUDE
)

ament_package()
