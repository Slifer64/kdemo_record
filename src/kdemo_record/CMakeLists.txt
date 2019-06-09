cmake_minimum_required(VERSION 3.2)
project(kdemo_record)

add_definitions(-DLINUX)

set(CMAKE_CXX_STANDARD 11) # Tell CMake to compile with C++11
set(CMAKE_AUTOMOC ON) # Tell CMake to run moc when needed.
set(CMAKE_AUTOUIC ON) # Tell CMake to run uic when needed.
set(CMAKE_AUTORCC ON) # Tell CMake to run rcc when needed

# Moc generated files are located in the current dir so we need to tell CMake to look for them.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  #OpenCV REQUIRED
  #PCL REQUIRED
  roscpp
  roslib
  lwr4p
  ati_sensor
)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets Core Gui PrintSupport REQUIRED)
# Add Qt5 definitions and includes to build libraries.
# Widgets add Widgets Core and Gui
add_definitions(${Qt5Widgets_DEFINITIONS})
include_directories(${Qt5Widgets_INCLUDES})
SET(Qt_LIBRARIES
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::PrintSupport
)

# BHand need to know that we are on Linux
add_definitions( -DLINUX )

#add_message_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

set(SRC_LIST

  src/source_list/robot/robot.cpp
  src/source_list/robot/lwr4p_robot.cpp
  src/source_list/robot/dummy_robot.cpp
  src/source_list/utils.cpp
  src/source_list/kdemo_record.cpp

  include/kdemo_record/gui/data_plot_dialog.cpp
  include/kdemo_record/gui/data_rec_dialog.cpp
  include/kdemo_record/gui/mainwindow.cpp
  include/kdemo_record/gui/qcustomplot.cpp
  include/kdemo_record/gui/rec_data.cpp
  include/kdemo_record/gui/utils.cpp
  include/kdemo_record/gui/view_jpos_dialog.cpp
  include/kdemo_record/gui/view_pose_dialog.cpp
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/kdemo_record/gui
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  /usr/include/eigen3
)

# *************************************************
# *************************************************

add_executable(kdemo_record
  src/main.cpp ${INCLUDE_LIST} ${SRC_LIST}
  resources/resource.qrc
)

add_dependencies(kdemo_record
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(kdemo_record
  ${catkin_LIBRARIES}
  armadillo
  ${Qt_LIBRARIES}
  lwr4p
  ati_sensor
)


# *************************************************
# *************************************************
