cmake_minimum_required(VERSION 2.8.3)
project(youbot_interface)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#add_compile_options(-DCMAT_STATIC_BUFFERS -DCMAT_STANDALONE)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pitt_msgs
  pcl_conversions
  pcl_ros
  rrtstar_msgs
  youbotcmd_msgs
   
)
find_package(PCL REQUIRED)
catkin_python_setup()
include_directories(
	INCLUDE_DIRS
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
	include/
)
link_directories(
#  /usr/local/lib/cmat
)

#Is for add the .o files to devel/lib
catkin_package(
 CATKIN_DEPENDS message_generation roscpp rospy std_msgs pitt_msgs pcl_msgs pcl_conversions sensor_msgs   geometry_msgs rrtstar_msgs youbotcmd_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

#ADD_LIBRARY(endor /home/nasa/Documents/Endor/endor-master/aograph.cpp)
add_library(youbotCallback include/youbotCallback.cpp)


#add_library(aograph include/aograph.cpp)
#add_library(CallBackClass src/CallBackClass.cpp)
#add_library(endorAction src/endorAction.cpp)
target_link_libraries(youbotCallback  ${catkin_LIBRARIES} ${PCL_LIBRARIES} )


add_dependencies(youbotCallback geometry_msgs_generate_messages_cpp youbotcmd_msgs_generate_messages_cpp)

## Declare a C++ executable
# name  name.cpp
add_executable(youbotInterface src/youbotInterface.cpp)

add_dependencies(youbotInterface geometry_msgs_generate_messages_cpp rrtstar_msgs_generate_messages_cpp youbotcmd_msgs_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(youbotInterface
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   youbotCallback

#   linuxcmat
 )
#target_link_libraries(aograph aonode)

