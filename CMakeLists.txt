cmake_minimum_required(VERSION 3.14)

project(
  ADCCollisionSensor
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required to build this project")
endif()

if(NOT DEFINED ENV{ROS_VERSION})
  message(FATAL_ERROR "ROS_VERSION is not set in the environment")
endif()

message("=========== ROS_VERSION : $ENV{ROS_VERSION}")
if($ENV{ROS_VERSION} EQUAL 2)
  set(ROS_VERSION 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
else()
  message(FATAL_ERROR "Unknown ROS version: $ENV{ROS_VERSION}")
endif()
 
# Configure and install the plugin configuration
configure_file(etc/ADCCollisionSensor.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/ADCCollisionSensor.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ADCCollisionSensor.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")
 
set(plugin_SRC
  src/ADCCollisionSensor.cpp
  src/LpfThreshold.cpp
)
 
set(plugin_HDR
  src/ADCCollisionSensor.h
  src/LpfThreshold.h
  src/ROS2Subscriber.h
)
 
# The add_plugin macro is provided by mc_rtc
add_plugin(ADCCollisionSensor "${plugin_SRC}" "${plugin_HDR}")
target_link_libraries(ADCCollisionSensor PUBLIC mc_rtc::mc_rtc_ros)
 
# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()
 

