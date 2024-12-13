# CMake generated Testfile for 
# Source directory: /home/luis-a/panda_ws/src/moveit2/moveit_ros/planning/planning_scene_monitor
# Build directory: /home/luis-a/panda_ws/build/moveit_ros_planning/planning_scene_monitor
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[current_state_monitor_tests]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/luis-a/panda_ws/build/moveit_ros_planning/test_results/moveit_ros_planning/current_state_monitor_tests.gtest.xml" "--package-name" "moveit_ros_planning" "--output-file" "/home/luis-a/panda_ws/build/moveit_ros_planning/ament_cmake_gmock/current_state_monitor_tests.txt" "--command" "/home/luis-a/panda_ws/build/moveit_ros_planning/planning_scene_monitor/current_state_monitor_tests" "--gtest_output=xml:/home/luis-a/panda_ws/build/moveit_ros_planning/test_results/moveit_ros_planning/current_state_monitor_tests.gtest.xml")
set_tests_properties([=[current_state_monitor_tests]=] PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/luis-a/panda_ws/build/moveit_ros_planning/planning_scene_monitor/current_state_monitor_tests" TIMEOUT "60" WORKING_DIRECTORY "/home/luis-a/panda_ws/build/moveit_ros_planning/planning_scene_monitor" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;106;ament_add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;52;_ament_add_gmock;/home/luis-a/panda_ws/src/moveit2/moveit_ros/planning/planning_scene_monitor/CMakeLists.txt;51;ament_add_gmock;/home/luis-a/panda_ws/src/moveit2/moveit_ros/planning/planning_scene_monitor/CMakeLists.txt;0;")
add_test([=[trajectory_monitor_tests]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/luis-a/panda_ws/build/moveit_ros_planning/test_results/moveit_ros_planning/trajectory_monitor_tests.gtest.xml" "--package-name" "moveit_ros_planning" "--output-file" "/home/luis-a/panda_ws/build/moveit_ros_planning/ament_cmake_gmock/trajectory_monitor_tests.txt" "--command" "/home/luis-a/panda_ws/build/moveit_ros_planning/planning_scene_monitor/trajectory_monitor_tests" "--gtest_output=xml:/home/luis-a/panda_ws/build/moveit_ros_planning/test_results/moveit_ros_planning/trajectory_monitor_tests.gtest.xml")
set_tests_properties([=[trajectory_monitor_tests]=] PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/luis-a/panda_ws/build/moveit_ros_planning/planning_scene_monitor/trajectory_monitor_tests" TIMEOUT "60" WORKING_DIRECTORY "/home/luis-a/panda_ws/build/moveit_ros_planning/planning_scene_monitor" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;106;ament_add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;52;_ament_add_gmock;/home/luis-a/panda_ws/src/moveit2/moveit_ros/planning/planning_scene_monitor/CMakeLists.txt;57;ament_add_gmock;/home/luis-a/panda_ws/src/moveit2/moveit_ros/planning/planning_scene_monitor/CMakeLists.txt;0;")
add_test([=[planning_scene_monitor_test_launch_planning_scene_monitor.test.py]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/luis-a/panda_ws/build/moveit_ros_planning/test_results/moveit_ros_planning/planning_scene_monitor_test_launch_planning_scene_monitor.test.py.xunit.xml" "--package-name" "moveit_ros_planning" "--output-file" "/home/luis-a/panda_ws/build/moveit_ros_planning/ros_test/planning_scene_monitor_test_launch_planning_scene_monitor.test.py.txt" "--command" "ros2" "test" "/home/luis-a/panda_ws/src/moveit2/moveit_ros/planning/planning_scene_monitor/test/launch/planning_scene_monitor.test.py" "test_binary_dir:=/home/luis-a/panda_ws/build/moveit_ros_planning/planning_scene_monitor" "--junit-xml=/home/luis-a/panda_ws/build/moveit_ros_planning/test_results/moveit_ros_planning/planning_scene_monitor_test_launch_planning_scene_monitor.test.py.xunit.xml" "--package-name=moveit_ros_planning")
set_tests_properties([=[planning_scene_monitor_test_launch_planning_scene_monitor.test.py]=] PROPERTIES  TIMEOUT "30" WORKING_DIRECTORY "/home/luis-a/panda_ws/build/moveit_ros_planning/planning_scene_monitor" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ros_testing/cmake/add_ros_test.cmake;73;ament_add_test;/home/luis-a/panda_ws/src/moveit2/moveit_ros/planning/planning_scene_monitor/CMakeLists.txt;67;add_ros_test;/home/luis-a/panda_ws/src/moveit2/moveit_ros/planning/planning_scene_monitor/CMakeLists.txt;0;")
subdirs("../gmock")
subdirs("../gtest")
