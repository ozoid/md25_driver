cmake_minimum_required(VERSION 2.8.3)
project(md25_driver)

## Compile as C++17
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(${PROJECT_NAME}_node
  src/md25_driver/md25.cpp
  src/md25_driver/main.cpp
)

catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
