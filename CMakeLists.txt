cmake_minimum_required(VERSION 3.1)
project(volumetric_plugin)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(AMBF REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options filesystem)

# ROS1 check (ROS2 won't have roscpp / ros/* headers)
find_package(roscpp QUIET)

include_directories(${AMBF_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${AMBF_LIBRARY_DIRS})
add_definitions(${AMBF_DEFINITIONS})

# Keep this if you want the msgs subdir configured/built as part of this tree.
# (Note: if you already converted volumetric_drilling_msgs to ROS2/ament,
# it won't integrate via add_subdirectory the same way; but this matches your current structure.)
add_subdirectory(volumetric_drilling_msgs)

set(vol_plugin_prefix "plugin/volumetric_drilling")

# ---- ROS1-only plugin (skip in ROS2) ----
if(NOT roscpp_FOUND)
  message(WARNING
    "ROS1 (roscpp) not found -> skipping volumetric_drilling (ROS1-only: includes ros/ros.h, ros/subscriber.h)."
  )
else()
  add_library(volumetric_drilling SHARED
    ${vol_plugin_prefix}/common.h
    ${vol_plugin_prefix}/volumetric_drilling.cpp
    ${vol_plugin_prefix}/volumetric_drilling.h
    ${vol_plugin_prefix}/ros_interface.cpp
    ${vol_plugin_prefix}/ros_interface.h
    ${vol_plugin_prefix}/joystick.cpp
    ${vol_plugin_prefix}/joystick.h
    ${vol_plugin_prefix}/footpedal.cpp
    ${vol_plugin_prefix}/footpedal.h
    ${vol_plugin_prefix}/wave_generator.cpp
    ${vol_plugin_prefix}/wave_generator.h
    ${vol_plugin_prefix}/drill_manager.cpp
    ${vol_plugin_prefix}/drill_manager.h
    ${vol_plugin_prefix}/endo_manager.cpp
    ${vol_plugin_prefix}/endo_manager.h
    ${vol_plugin_prefix}/camera_panel_manager.h
    ${vol_plugin_prefix}/camera_panel_manager.cpp
    ${vol_plugin_prefix}/gaze_marker_controller.h
    ${vol_plugin_prefix}/gaze_marker_controller.cpp
  )

  target_link_libraries(volumetric_drilling
    ${Boost_LIBRARIES}
    ${AMBF_LIBRARIES}
  )

  set_property(TARGET volumetric_drilling PROPERTY POSITION_INDEPENDENT_CODE TRUE)
endif()

# ---- This one doesn't depend on ROS headers, so always build it ----
add_library(camera_hmd SHARED
  plugin/camera_hmd/hmd.cpp
  plugin/camera_hmd/hmd.h
)
target_link_libraries(camera_hmd ${AMBF_LIBRARIES})
set_property(TARGET camera_hmd PROPERTY POSITION_INDEPENDENT_CODE TRUE)
