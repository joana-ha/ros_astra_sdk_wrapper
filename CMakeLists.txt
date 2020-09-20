cmake_minimum_required(VERSION 2.8.3)
project(ros_astra_sdk_wrapper)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Release)

###################
## Find Packages ##
###################

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  cmake_modules
  #visualization_msgs
  std_msgs
  image_transport
  sensor_msgs
  message_generation
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)

set(CMAKE_MODULE_PATH "/home/turtlebot/lib/AstraSDK/samples/cmake/")
#list(APPEND CMAKE_MODULE_PATH "/home/turtlebot/lib/AstraSDK/samples/cmake/")
set(ASTRA_SAMPLES_COMMON_DIR "/home/turtlebot/lib/AstraSDK/samples/common/")

MESSAGE( STATUS "AFTER:" )
foreach(path ${CMAKE_MODULE_PATH})
  message("     Path = " ${path})
endforeach(path)

find_package(Astra REQUIRED)

MESSAGE( STATUS "CMAKE_MODULE_PATH:         " ${CMAKE_MODULE_PATH} )
MESSAGE( STATUS "ASTRA_SAMPLES_COMMON_DIR:  " ${ASTRA_SAMPLES_COMMON_DIR} )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        BodyTracking.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sdk_stream_wrapper
        CATKIN_DEPENDS roscpp nodelet std_msgs image_transport sensor_msgs message_generation geometry_msgs message_runtime
        DEPENDS ${ASTRA_ROOT}
)

###########
## Build ##
###########

include_directories(
  include
  ${ASTRA_INCLUDE_DIR}
  ${ASTRA_INCLUDE_DIR}/astra/capi
  ${ASTRA_SAMPLES_COMMON_DIR}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${Boost_INCLUDE_DIRS}
)

add_library(sdk_stream_wrapper
        src/astra_device_manager.cpp
        src/astra_stream_manager.cpp
        src/stream_wrapper.cpp
        )
## Generate messages before compiling the lib
add_dependencies(sdk_stream_wrapper ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(sdk_stream_wrapper ${catkin_LIBRARIES} ${ASTRA_LIBRARIES}
        ${Boost_LIBRARIES})


add_library(astra_device_manager_lib
        src/astra_device_manager.cpp
        )

add_dependencies(astra_device_manager_lib ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(astra_device_manager_lib sdk_stream_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(ros_astra_sdk_nodelet
        ros/ros_astra_sdk_nodelet.cpp
        )
target_link_libraries(ros_astra_sdk_nodelet astra_device_manager_lib sdk_stream_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${ASTRA_LIBRARIES})

add_executable(ros_astra_sdk_node
        ros/ros_astra_sdk_node.cpp
        )
target_link_libraries(ros_astra_sdk_node astra_device_manager_lib sdk_stream_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${ASTRA_LIBRARIES} )
add_dependencies(ros_astra_sdk_node ${catkin_EXPORTED_TARGETS})
#add_dependencies(astra_camera_node)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS sdk_stream_wrapper astra_device_manager_lib ros_astra_sdk_nodelet ros_astra_sdk_node ${ADDITIONAL_EXECUTABLES}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

# add xml file
install(FILES astra_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )


