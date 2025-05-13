cmake_minimum_required(VERSION 3.0.2)
project(adi_3dtof_floor_detector)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std = c++ 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_view
  image_geometry
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  eigen_conversions
  tf2
  tf2_ros
  pcl_ros
  dynamic_reconfigure
)

find_package(Eigen3 QUIET)
find_package(OpenCV REQUIRED)

# Variable to indicate that the sensor is connected to the platform
set(SENSOR_CONNECTED TRUE CACHE BOOL "Indicates if the sensor is connected to the platform")

if(CMAKE_BUILD_TYPE)
  string(TOLOWER ${CMAKE_BUILD_TYPE} BUILD_TYPE)
  if ((${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64|AARCH64") AND (${BUILD_TYPE} STREQUAL "release"))
    #To enable NEON optimizations on device
    add_compile_definitions(ENABLE_NEON_OPTIMIZATION)
    #To verify NEON optimizations
    #add_compile_definitions(ENABLE_NEON_OPTIMIZATION_VERIFICATION)
  endif()
endif()

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
cfg/FloorDetectorParams.cfg
#cfg / DynReconf2.cfg
)

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
  INCLUDE_DIRS include
  LIBRARIES adi_3dtof_floor_detector
  CATKIN_DEPENDS roscpp std_msgs image_geometry sensor_msgs pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# The macro should be passed using cmake command
# ex: catkin_make -DHOST_BUILD=TRUE,
# By default it is set to FALSE
if(SENSOR_CONNECTED)
  find_package(aditof REQUIRED)
  add_compile_definitions(ENABLE_ADTF31XX_SENSOR)
  add_executable(${PROJECT_NAME}_node src/adi_3dtof_floor_detector_main.cpp src/adi_3dtof_floor_detector_algorithm.cpp src/adi_3dtof_floor_detector_application.cpp src/input_sensor_adtf31xx.cpp src/image_proc_utils.cpp src/output_sensor_file.cpp src/adi_3dtof_floor_detector_input_thread.cpp src/adi_3dtof_floor_detector_output_thread.cpp src/input_sensor_file_rosbagbin.cpp src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp)
else()
  add_executable(${PROJECT_NAME}_node src/adi_3dtof_floor_detector_main.cpp src/adi_3dtof_floor_detector_algorithm.cpp src/adi_3dtof_floor_detector_application.cpp src/image_proc_utils.cpp src/output_sensor_file.cpp src/adi_3dtof_floor_detector_input_thread.cpp src/adi_3dtof_floor_detector_output_thread.cpp src/module_profile.cpp src/input_sensor_file_rosbagbin.cpp src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp)
endif()

#Example Node
add_executable(adi_3dtof_floor_detector_example_node src/adi_3dtof_floor_detector_example.cpp src/image_proc_utils.cpp )

#Dynamic Reconfigure
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64|AARCH64")
  set(FLOOR_DETECTOR_CORE_LIB ${PROJECT_SOURCE_DIR}/libs/libfloor_plane_detection.a)
elseif (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64|X86_64")
  set(FLOOR_DETECTOR_CORE_LIB ${PROJECT_SOURCE_DIR}/libs_linux_x86/libfloor_plane_detection.a)
endif()

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${aditof_LIBRARIES}
  ${FLOOR_DETECTOR_CORE_LIB})

target_link_libraries(${PROJECT_NAME}_example_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
        ${aditof_LIBRARIES}
)

#############
## Install ##
#############

#Copy the lib to devel / lib

set(RQT_RVIZ_ORIGINAL_FILE ${PROJECT_SOURCE_DIR}/rqt_config/adi_3dtof_floor_detector_rqt_rviz.rviz)
set(RQT_RVIZ_DEST_FILE ~/.ros/adi_3dtof_floor_detector_rqt_rviz.rviz)
add_custom_command( TARGET ${PROJECT_NAME}_node POST_BUILD COMMAND cp ${RQT_RVIZ_ORIGINAL_FILE} ${RQT_RVIZ_DEST_FILE} )

# Copy the ToF sensor config folder
add_custom_command( TARGET ${PROJECT_NAME}_node POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/config 
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/config)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
 install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 
 install(TARGETS ${PROJECT_NAME}_example_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

