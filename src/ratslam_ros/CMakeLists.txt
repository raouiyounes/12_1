cmake_minimum_required(VERSION 2.8.3)
project(ratslam_ros)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS "-std=c++0x")
find_package(catkin REQUIRED COMPONENTS libviso2   pcl_conversions pcl_msgs message_generation std_msgs roscpp sensor_msgs nav_msgs tf visualization_msgs image_transport nav_msgs cv_bridge)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(Irrlicht REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   TopologicalAction.msg
   TopologicalEdge.msg
   TopologicalMap.msg
   TopologicalNode.msg
   ViewTemplate.msg
   TransAndRot.msg
   toute.msg
   scalarMess.msg
   poses_robot.msg
)

## Generate actions in the 'msg' folder
#add_action_files(DIRECTORY action FILES DoDishes.action)
#generate_messages(DEPENDENCIES actionlib_msgs)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
CATKIN_DEPENDS message_runtime geometry_msgs std_msgs sensor_msgs cv_bridge image_transport 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${libviso2_INCLUDE_DIRS}
)

#ratslam library 

add_library(ratslam src/ratslam/experience_map.cpp src/ratslam/posecell_network.cpp  src/ratslam/local_view_match.cpp src/ratslam/visual_odometry.cpp)
add_library(ratslam2  src/ratslam/FPVision.cpp src/ratslam/Filter.h  src/ratslam/Filter.cpp)
add_library(ratslam3  src/ratslam/include_fs/loMeasurement.h  src/ratslam/include_fs/loMeasurement.cpp src/ratslam/include_fs/particle.cpp src/ratslam/include_fs/particle.h  src/ratslam/include_fs/sitRobotMM.h src/ratslam/include_fs/sitRobotMM.cpp)

# uncomment is you don't have irrlicht installed
add_definitions("-DHAVE_IRRLICHT")

add_executable(ratslam_lv src/main_lv.cpp)
target_link_libraries(ratslam_lv 
  ${catkin_LIBRARIES}
  ratslam
ratslam2
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(ratslam_pc src/main_pc.cpp)
target_link_libraries(ratslam_pc
  ${catkin_LIBRARIES} 
  ratslam
ratslam2
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES})

add_executable(ratslam_em src/main_em.cpp)
target_link_libraries(ratslam_em
  ${catkin_LIBRARIES}
  ratslam
ratslam2
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES})

add_executable(ratslam_vo src/main_vo.cpp)
target_link_libraries(ratslam_vo
  ${catkin_LIBRARIES}
  ratslam
ratslam2
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(ratslam_vland src/main_vland.cpp)
target_link_libraries(ratslam_vland
  ${catkin_LIBRARIES}
 ratslam
ratslam2
  ${OpenCV_LIBRARIES})


add_executable(ratslam_pcl src/main_pcl.cpp)
target_link_libraries(ratslam_pcl
  ${catkin_LIBRARIES}
 ratslam
ratslam2
  ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
  )

add_executable(ratslam_slam src/main_slam.cpp)
target_link_libraries(ratslam_slam
  ${catkin_LIBRARIES}
 ratslam
ratslam2
ratslam3
  ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  )
  
  
  
add_executable(ratslam_odo src/main_odo.cpp)
target_link_libraries(ratslam_odo
  ${catkin_LIBRARIES}
  ratslam
  ratslam2
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBRARIES})
  
  
#config files for devel
set(MEDIA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/media) #devel use the files in the source dir
file(GLOB infiles "config/*.in")
foreach(infile ${infiles})
  #MESSAGE(STATUS "Process file: ${infile}")

  # Generate output file name
  string(REGEX REPLACE ".in\$" "" outfile "${infile}")
  get_filename_component(outfile "${outfile}" NAME)
  set(outfile "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/config/${outfile}")
  #MESSAGE(STATUS "Output file: ${outfile}")

  configure_file("${infile}" "${outfile}")
endforeach()

#config files for install
set(MEDIA_PATH ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/media) #install use the installed media files
file(GLOB infiles "config/*.in")
foreach(infile ${infiles})
  #MESSAGE(STATUS "Process file: ${infile}")

  # Generate output file name
  string(REGEX REPLACE ".in\$" "" outfile "${infile}")
  get_filename_component(outfile "${outfile}" NAME)
  set(outfile "${CMAKE_CURRENT_BINARY_DIR}/config/${outfile}")
  #MESSAGE(STATUS "Output file: ${outfile}")

  configure_file("${infile}" "${outfile}")
endforeach()

add_dependencies(ratslam_lv ratslam_ros_generate_messages_cpp)
add_dependencies(ratslam_pc ratslam_ros_generate_messages_cpp)
add_dependencies(ratslam_em ratslam_ros_generate_messages_cpp)
add_dependencies(ratslam_vo ratslam_ros_generate_messages_cpp)
add_dependencies(ratslam_slam ratslam_ros_generate_messages_cpp)
add_dependencies(ratslam_odo ratslam_ros_generate_messages_cpp)

add_dependencies(ratslam_vland ${catkin_EXPORTED_TARGETS})
add_dependencies(ratslam_vland ${PROJECT_NAME}_gencpp)


install(TARGETS ratslam ratslam2 ratslam3 ratslam_slam ratslam_odo ratslam_lv ratslam_pc ratslam_em ratslam_vo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY src/media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
