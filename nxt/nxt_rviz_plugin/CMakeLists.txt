cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

# Use Qt signals and slots to avoid defining "emit", "slots", etc.
# because they can conflict with Boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

# Specify which header files need to be run through "moc",
# Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  src/nxt_color_display.h
  src/nxt_ultrasonic_display.h
)

set(SOURCE_FILES
  src/nxt_color_display.cpp
  src/nxt_color_visual.cpp
  src/nxt_ultrasonic_display.cpp
  src/nxt_ultrasonic_visual.cpp
  ${MOC_FILES}
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# An rviz plugin is just a shared library, so here we declare the
# library to be called PROJECT_NAME
rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

