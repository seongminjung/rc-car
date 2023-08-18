# CMakeLists.txt
#
# Copyright (c) 2015 Wunderkammer Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.3)
project(ackermann_vehicle_gazebo)
find_package(catkin REQUIRED COMPONENTS roscpp rviz)
catkin_package(INCLUDE_DIRS include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(l2p src/l2p.cpp)
target_link_libraries(l2p
  ${catkin_LIBRARIES}
  
)

catkin_install_python(PROGRAMS scripts/ackermann_controller.py scripts/cmd_vel_to_ackermann_drive.py scripts/keyboard_driver_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
