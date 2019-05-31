cmake_minimum_required(VERSION 2.8.3)
project(v9_goalpost_global)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   roslib
   std_msgs
   sensor_msgs
   geometry_msgs
   image_transport
   cv_bridge
)

find_package(OpenCV 3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS
   roscpp
   roslib
   std_msgs
   geometry_msgs
   sensor_msgs
   image_transport
   cv_bridge
)

include_directories(
   include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
   ${PROJECT_NAME}_node 
      #src/v9_goalpost_detector.cpp
      src/v9_goalpost_detector_node.cpp
      src/v9_goalpost_global.cpp
      )

add_dependencies(
   ${PROJECT_NAME}_node 
      ${${PROJECT_NAME}_EXPORTED_TARGETS} 
      ${catkin_EXPORTED_TARGETS}
      ${OpenCV_EXPORTED_TARGETS}
   )

target_link_libraries(
   ${PROJECT_NAME}_node
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
