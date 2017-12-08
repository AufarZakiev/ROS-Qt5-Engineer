cmake_minimum_required(VERSION 2.8.12)
project(ros_qt_controller)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)
# find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5Widgets REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS system-lib
)

# qt5_add_resources(QT_RESOURCES_CPP qt_resources/arrows.qrc)
# qt5_wrap_cpp(QT_MOC_HPP src/ControlWindow.h
#						src/RobotThread.h)

include_directories(src ${catkin_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Network_INCLUDE_DIRS})
include_directories(${Qt5Core_INCLUDE_DIRS})

add_executable(twist_control_node src/main.cpp)
                                   #${QT_MOC_HPP})

target_link_libraries(twist_control_node ${QT_LIBRARIES}
                          ${catkin_LIBRARIES} Qt5::Core Qt5::Network)

install(TARGETS twist_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
