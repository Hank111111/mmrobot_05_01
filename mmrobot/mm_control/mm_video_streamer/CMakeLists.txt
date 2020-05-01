cmake_minimum_required(VERSION 2.8.3)

project(mm_video_streamer)
set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
set(ZED_DIR /usr/local/zed)
find_package(ZED 2 REQUIRED)
SET(SPECIAL_OS_LIBS "pthread")
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  nodelet
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nodelet message_runtime
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3 -std=c++11 -pthread  ")


#set ( CMAKE_EXE_LINKER_FLAGS_RELEASE  "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -std=c++11 -Wall ")
#set ( CMAKE_EXE_LINKER_FLAGS_DEBUG  "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fsanitize=address -fsanitize=undefined ")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -std=c++11 -Wall -O0 -fno-omit-frame-pointer -fsanitize=address -fsanitize=undefined -D__FORTIFY_SOURCE=2 -rdynamic -DDEBUG")

#set ( CMAKE_EXE_LINKER_FLAGS_DEBUG  "${CMAKE_EXE_LINKER_FLAGS_DEBUG}  ")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -std=c++11 -Wall -O0 -fno-omit-frame-pointer ")


include_directories(/usr/local/include)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(/usr/lib/x86_64-linux-gnu)
#link_directories(/usr/local/lib)
#link_directories(/usr/lib/)




add_library(streamer_nodelet src/streamer_nodelet.cpp src/streamer/streamer.cpp)
#add_dependencies(streamer_nodelet  ${catkin_EXPORTED_TARGETS})
#target_link_libraries(streamer_nodelet   ${OpenCV_LIBS} ${catkin_LIBRARIES} avformat swscale avcodec swresample avutil)
target_link_libraries(streamer_nodelet   ${OpenCV_LIBS} ${catkin_LIBRARIES}  -lavformat -lbz2 -lswscale -lavcodec -llzma -lz -lswresample  -lavutil -pthread -lm -ldl)

add_executable(streamer_camera_node src/streamer_camera_node.cpp src/streamer/streamer.cpp)
target_link_libraries(streamer_camera_node   ${OpenCV_LIBS} ${catkin_LIBRARIES} ${ZED_LIBRARIES})
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -L/usr/local/lib -lavformat -lbz2 -lswscale -lavcodec -llzma -lz -lswresample  -lavutil -pthread -lm -ldl")

#target_include_directories(streamer_nodelet PRIVATE ${AVCODEC_INCLUDE_DIR} ${AVFILTER_INCLUDE_DIR} ${AVFORMAT_INCLUDE_DIR} ${AVUTIL_INCLUDE_DIR} ${AVDEVICE_INCLUDE_DIR})
#set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -L/usr/local/lib -lavformat -lbz2 -lswscale -lavcodec -llzma -lz -lswresample  -lavutil -pthread -lm -ldl")
#set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} `pkg-config --cflags libavformat libswscale libswresample libavutil libavcodec opencv` `pkg-config --libs libavformat libswscale libswresample libavutil libavcodec opencv` -std=gnu++11")
 