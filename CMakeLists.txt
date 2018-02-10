cmake_minimum_required(VERSION 2.8.3)
project(xtion2_ros)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
  cmake_modules
  roscpp
  roslint
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenNI2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS Boost
)

roslint_cpp()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENNI2_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} src/io_interface.cxx)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OPENNI2_LIBRARY} ${catkin_LIBRARIES})

set(NODE ${PROJECT_NAME}_node)
add_executable(${NODE} src/${NODE}.cxx)
add_dependencies(${NODE} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE} ${PROJECT_NAME} ${OPENNI2_LIBRARY} ${catkin_LIBRARIES})
