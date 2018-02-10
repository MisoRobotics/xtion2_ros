cmake_minimum_required(VERSION 2.8.3)
project(xtion2_ros)

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
  cmake_modules
  cv_bridge
  roscpp
  roslint
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3.2 REQUIRED)
find_package(OpenNI2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge roscpp
  DEPENDS Boost OpenCV OpenNI2
)

roslint_cpp()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENNI2_INCLUDE_DIR}
)

set(LINK_LIBS  ${OPENNI2_LIBRARY} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
set(NODE ${PROJECT_NAME}_node)

add_library(${PROJECT_NAME}
  src/camera_publisher.cxx
  src/io_interface.cxx
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

add_executable(${NODE} src/${NODE}.cxx)
add_dependencies(${NODE} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE} ${PROJECT_NAME} ${LINK_LIBS})
