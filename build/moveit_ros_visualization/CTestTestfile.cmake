# CMake generated Testfile for 
# Source directory: /home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization
# Build directory: /home/luis-a/panda_ws/build/moveit_ros_visualization
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[cppcheck]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/luis-a/panda_ws/build/moveit_ros_visualization/test_results/moveit_ros_visualization/cppcheck.xunit.xml" "--package-name" "moveit_ros_visualization" "--output-file" "/home/luis-a/panda_ws/build/moveit_ros_visualization/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/humble/bin/ament_cppcheck" "--xunit-file" "/home/luis-a/panda_ws/build/moveit_ros_visualization/test_results/moveit_ros_visualization/cppcheck.xunit.xml" "--include_dirs" "/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/rviz_plugin_render_tools/include" "/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/robot_state_rviz_plugin/include" "/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/planning_scene_rviz_plugin/include" "/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/motion_planning_rviz_plugin/include" "/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/trajectory_rviz_plugin/include")
set_tests_properties([=[cppcheck]=] PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;66;ament_add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;87;ament_cppcheck;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/CMakeLists.txt;140;ament_package;/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/CMakeLists.txt;0;")
add_test([=[lint_cmake]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/luis-a/panda_ws/build/moveit_ros_visualization/test_results/moveit_ros_visualization/lint_cmake.xunit.xml" "--package-name" "moveit_ros_visualization" "--output-file" "/home/luis-a/panda_ws/build/moveit_ros_visualization/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/luis-a/panda_ws/build/moveit_ros_visualization/test_results/moveit_ros_visualization/lint_cmake.xunit.xml")
set_tests_properties([=[lint_cmake]=] PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/CMakeLists.txt;140;ament_package;/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/CMakeLists.txt;0;")
add_test([=[pep257]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/luis-a/panda_ws/build/moveit_ros_visualization/test_results/moveit_ros_visualization/pep257.xunit.xml" "--package-name" "moveit_ros_visualization" "--output-file" "/home/luis-a/panda_ws/build/moveit_ros_visualization/ament_pep257/pep257.txt" "--command" "/opt/ros/humble/bin/ament_pep257" "--xunit-file" "/home/luis-a/panda_ws/build/moveit_ros_visualization/test_results/moveit_ros_visualization/pep257.xunit.xml")
set_tests_properties([=[pep257]=] PROPERTIES  LABELS "pep257;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_pep257.cmake;41;ament_add_test;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;18;ament_pep257;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/CMakeLists.txt;140;ament_package;/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/CMakeLists.txt;0;")
add_test([=[xmllint]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/luis-a/panda_ws/build/moveit_ros_visualization/test_results/moveit_ros_visualization/xmllint.xunit.xml" "--package-name" "moveit_ros_visualization" "--output-file" "/home/luis-a/panda_ws/build/moveit_ros_visualization/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/luis-a/panda_ws/build/moveit_ros_visualization/test_results/moveit_ros_visualization/xmllint.xunit.xml")
set_tests_properties([=[xmllint]=] PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/CMakeLists.txt;140;ament_package;/home/luis-a/panda_ws/src/moveit2/moveit_ros/visualization/CMakeLists.txt;0;")
subdirs("rviz_plugin_render_tools")
subdirs("robot_state_rviz_plugin")
subdirs("planning_scene_rviz_plugin")
subdirs("motion_planning_rviz_plugin")
subdirs("trajectory_rviz_plugin")
