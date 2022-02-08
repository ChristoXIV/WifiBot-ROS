### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(roswifibot)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 3 - Find other CMake/Catkin packages needed for build
SET(ROSDEPS message_generation roscpp std_msgs tf
)
find_package(catkin REQUIRED COMPONENTS ${ROSDEPS})
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
add_message_files(DIRECTORY msg FILES Status.msg )
### 5 - Invoke message/service/action generation (generate_messages())
generate_messages(
  DEPENDENCIES std_msgs
)
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${ROSDEPS}
  DEPENDS
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#common commands for building c++ executables and libraries
add_library(libwifibot               src/libwifibot.cpp)
add_executable(wifibot_node          src/wifibot.cpp)
target_link_libraries(wifibot_node   ${catkin_LIBRARIES} libwifibot)
add_dependencies     (wifibot_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)
### 8 - Tests to build (catkin_add_gtest())
# Nothing in this package
### 9 - Install rules (install())
# Nothing in this package
