# Install script for directory: /home/astondky/Desktop/robot_simulation_experiment_ws/src/robot_simulation_experiment

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/astondky/Desktop/robot_simulation_experiment_ws/install/robot_simulation_experiment")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_simulation_experiment" TYPE PROGRAM FILES
    "/home/astondky/Desktop/robot_simulation_experiment_ws/src/robot_simulation_experiment/scripts/control_eye_in_hand_ros2.py"
    "/home/astondky/Desktop/robot_simulation_experiment_ws/src/robot_simulation_experiment/scripts/human_walking_stable_step_ros2.py"
    "/home/astondky/Desktop/robot_simulation_experiment_ws/src/robot_simulation_experiment/scripts/humanoid_walk_only_ros2.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_simulation_experiment" TYPE PROGRAM RENAME "asimo_style_zmp_walker" FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/src/robot_simulation_experiment/scripts/asimo_walker/main.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND /usr/bin/sed -i "1s|.*|#!/usr/bin/python3|" "$ENV{DESTDIR}/home/astondky/Desktop/robot_simulation_experiment_ws/install/robot_simulation_experiment/lib/robot_simulation_experiment/asimo_style_zmp_walker")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_simulation_experiment/asimo_walker" TYPE DIRECTORY FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/src/robot_simulation_experiment/scripts/asimo_walker/" FILES_MATCHING REGEX "/[^/]*\\.py$" REGEX "/main\\.py$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/robot_simulation_experiment")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/robot_simulation_experiment")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_simulation_experiment/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_simulation_experiment/environment" TYPE FILE FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_simulation_experiment/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_simulation_experiment/environment" TYPE FILE FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_simulation_experiment" TYPE FILE FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_simulation_experiment" TYPE FILE FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_simulation_experiment" TYPE FILE FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_simulation_experiment" TYPE FILE FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_simulation_experiment" TYPE FILE FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_index/share/ament_index/resource_index/packages/robot_simulation_experiment")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_simulation_experiment/cmake" TYPE FILE FILES
    "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_core/robot_simulation_experimentConfig.cmake"
    "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/ament_cmake_core/robot_simulation_experimentConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_simulation_experiment" TYPE FILE FILES "/home/astondky/Desktop/robot_simulation_experiment_ws/src/robot_simulation_experiment/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/astondky/Desktop/robot_simulation_experiment_ws/build/robot_simulation_experiment/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
