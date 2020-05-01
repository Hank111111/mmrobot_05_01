cmake_minimum_required(VERSION 2.8.3)
project(mm_visual_postion)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
set(ZED_DIR /usr/local/zed)



## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g -O0 -Wall)




## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mm_robot_decision
  actionlib
  diagnostic_msgs
  geometry_msgs
  sensor_msgs
  shape_msgs
  std_srvs
  tf
  message_generation
  OpenCV
  roslib
  tf_conversions
  cv_bridge
  image_transport
  camera_info_manager
)
find_package(Eigen3 REQUIRED)
find_package(VISP REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${VISP_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})



## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
add_message_files(
  FILES
  ROI.msg
  AppInnerRequest.msg
  EquipmentPose.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs   
   mm_robot_decision
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mm_visual_postion
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mm_robot_decision
  CATKIN_DEPENDS message_runtime 
#  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


## Declare a C++ library



add_library(hough_v3_4_5
  src/utils/hough_v3_4_5.cpp
)
add_library(particularFilter
  src/switch/particularFilter.cpp
)
add_library(coordinate_transformer
  src/utils/CoordinateTransformer.cpp
)
add_library(check_create_path
  src/utils/CheckCreatePath.cpp
)

add_library(ransac_ellipse2d
  src/hand_trunk/ransac_ellipse2d.cpp
)

add_library(utils
  src/utils/utils.cpp
)
add_dependencies(utils mm_visual_postion_generate_messages_cpp)

add_library(rough_circle_solver
  src/hand_trunk/RoughCircleSolver.cpp
)
add_dependencies(rough_circle_solver utils mm_visual_postion_generate_messages_cpp)
add_library(precise_circle_solver
  src/hand_trunk/PreciseCircleSolver.cpp 
)
add_dependencies(precise_circle_solver utils mm_visual_postion_generate_messages_cpp)

target_link_libraries(ransac_ellipse2d utils ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_library(visual_cabinet src/visual_wrapper/VisualCabinet.cpp)
add_dependencies(visual_cabinet  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
target_link_libraries(visual_cabinet ${catkin_LIBRARIES} ${OpenCV_LIBS} check_create_path utils mysqlcppconn)


add_library(visual_safe_wrapper src/visual_wrapper/VisualSafeWrapper.cpp)
add_dependencies(visual_safe_wrapper  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
target_link_libraries(visual_safe_wrapper  ${catkin_LIBRARIES} ${OpenCV_LIBS} check_create_path utils visual_cabinet)

add_library(rect_switch_finder src/switch/RectSwitchFinder.cpp)
add_dependencies(rect_switch_finder  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
target_link_libraries(rect_switch_finder  utils ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} check_create_path utils hough_v3_4_5)

add_library(libdigitalMeter src/digitalmeter/digitalDeal.cpp)
target_link_libraries(libdigitalMeter ${catkin_LIBRARIES} ${OpenCV_LIBS})

link_directories(${OpenCV_LIB_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/mm_visual_postion_node.cpp)



#add_executable(SwitchDataProcessor src/experiment/SwitchDataProcessor.cpp)
#add_dependencies(SwitchDataProcessor ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
#target_link_libraries(SwitchDataProcessor utils check_create_path coordinate_calculation coordinate_transformer particularFilter switch_finder_by_hough hough_v3_4_5 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})

#add_executable(SwitchDataGenerator src/experiment/SwitchDataGenerator.cpp)
#add_dependencies(SwitchDataGenerator  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)

#target_link_libraries(SwitchDataGenerator utils check_create_path coordinate_transformer hough_v3_4_5 switch_finder_by_hough ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(AutoCalib src/AutoCalib.cpp)
add_dependencies(AutoCalib  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)

target_link_libraries(AutoCalib check_create_path utils ${catkin_LIBRARIES} ${OpenCV_LIBS} ${VISP_LIBRARIES})

#add_executable(TrunkDataGenerator src/experiment/TrunkDataGenerator.cpp )
#add_dependencies(TrunkDataGenerator  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
#target_link_libraries(TrunkDataGenerator utils switch_finder_by_hough precise_circle_solver rough_circle_solver check_create_path ransac_ellipse2d ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


#add_executable(TrunkDataProcessor src/experiment/TrunkDataProcessor.cpp)
#add_dependencies(TrunkDataProcessor  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
#target_link_libraries(TrunkDataProcessor  utils switch_finder_by_hough precise_circle_solver rough_circle_solver check_create_path ransac_ellipse2d ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


add_executable(VisualSafeWrapperNode src/visual_wrapper/VisualSafeWrapperNode.cpp)
add_dependencies(VisualSafeWrapperNode  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
target_link_libraries(VisualSafeWrapperNode  visual_safe_wrapper)


add_executable(VisualSafeWrapperTeachNode src/visual_wrapper/VisualSafeWrapperTeachNode.cpp)
add_dependencies(VisualSafeWrapperTeachNode  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
target_link_libraries(VisualSafeWrapperTeachNode  visual_safe_wrapper )

#catkin_add_gtest()
catkin_add_executable_with_gtest(visual_cabinet_test test/visual_wrapper/VisualCabinetTest.cpp)
target_link_libraries(visual_cabinet_test mysqlcppconn  ${catkin_LIBRARIES} ${OpenCV_LIBS}  utils visual_cabinet)
add_dependencies(visual_cabinet_test mm_visual_postion_generate_messages_cpp)

catkin_add_executable_with_gtest(hand_trunk_solver_test test/hand_trunk_solver/HandTrunkSolverTest.cpp)
target_link_libraries(hand_trunk_solver_test  utils precise_circle_solver rough_circle_solver check_create_path ransac_ellipse2d hough_v3_4_5 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
add_dependencies(hand_trunk_solver_test mm_visual_postion_generate_messages_cpp )

catkin_add_executable_with_gtest(rect_switch_finder_test test/rect_switch_finder/RectSwitchFinderTest.cpp)
target_link_libraries(rect_switch_finder_test  ${catkin_LIBRARIES} ${OpenCV_LIBS}    utils rect_switch_finder)
add_dependencies(rect_switch_finder_test mm_visual_postion_generate_messages_cpp)



add_executable(RectSwitchFinderNode src/switch/RectSwitchFinderNode.cpp)
add_dependencies(RectSwitchFinderNode  ${catkin_EXPORTED_TARGETS}  mm_visual_postion_generate_messages_cpp)
target_link_libraries(RectSwitchFinderNode   ${catkin_LIBRARIES} ${OpenCV_LIBS}   rect_switch_finder check_create_path)


add_executable(HandTrunkSolverNode src/hand_trunk/HandTrunkSolverNode.cpp)
add_dependencies(HandTrunkSolverNode  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
target_link_libraries(HandTrunkSolverNode   utils hough_v3_4_5 precise_circle_solver rough_circle_solver check_create_path ransac_ellipse2d ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(MeterRecognizerNode src/switch/MeterRecognizer.cpp)
add_dependencies(MeterRecognizerNode  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
target_link_libraries(MeterRecognizerNode   utils hough_v3_4_5 rect_switch_finder precise_circle_solver rough_circle_solver check_create_path ransac_ellipse2d ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(digitalMeterNode src/digitalmeter/checkDigits.cpp)
add_dependencies(digitalMeterNode  ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
target_link_libraries(digitalMeterNode  utils libdigitalMeter hough_v3_4_5 rect_switch_finder check_create_path  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

#FILE(GLOB_RECURSE SRC_FILES src/01ZedCapture.cpp)
#add_executable(ZedCapture src/01ZedCapture.cpp)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

#target_link_libraries(ZedCapture ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED}) 

#add_executable(02ZedView src/02ZedView.cpp)
#add_dependencies(02ZedView ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
#target_link_libraries(02ZedView  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})                    

# add_executable(light src/light.cpp)
# add_dependencies(light ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
# target_link_libraries(light utils ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})

#add_executable(rigid_transform_test src/RigidTransformSolverTest.cpp)
#add_dependencies(rigid_transform_test ${catkin_EXPORTED_TARGETS} mm_visual_postion_generate_messages_cpp)
#target_link_libraries(rigid_transform_test ${catkin_LIBRARIES} ${OpenCV_LIBS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mm_visual_postion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
