# CMake generated Testfile for 
# Source directory: /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libCharon
# Build directory: /home/tiao/git_ws/moveo_moveit_ws/src/Cura/libCharon/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(pytest-main "/usr/bin/python3" "-m" "pytest" "--junitxml=/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libCharon/build/junit-pytest-main.xml" "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libCharon/tests")
set_tests_properties(pytest-main PROPERTIES  ENVIRONMENT "PYTHONPATH=/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libCharon:/home/tiao/cura/Uranium/:/home/tiao/git_ws/moveo_moveit_ws/devel/lib/python2.7/dist-packages:/home/tiao/ws_moveit/devel/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages")
