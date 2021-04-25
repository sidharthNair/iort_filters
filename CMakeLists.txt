cmake_minimum_required(VERSION 3.0.2)
project(iort_filters)

# # specify the C++ standard
add_compile_options(-fPIC -shared)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    qt_build 
    nodelet
    pluginlib
    roscpp
    insitu
    iort_lib
)
find_package(jsoncpp REQUIRED)
find_package(OpenCV 3.4.5 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Qt5 COMPONENTS Core Gui Widgets)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iort_filters
#  CATKIN_DEPENDS nodelet pluginlib roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    inc
    ${catkin_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

set(iort_filters_HDRS
    inc/QRFilter/QRFilter.hpp
    inc/QRFilter/QRFilter_dialog.hpp

    inc/IoRT_Plot/IoRT_Plot.hpp
    inc/IoRT_Plot/IoRT_Plot_dialog.hpp
)

qt5_wrap_cpp(iort_filters_MOCS ${iort_filters_HDRS})

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/QRFilter/QRFilter.cpp
    src/QRFilter/QRFilter_dialog.cpp

    src/IoRT_Plot/IoRT_Plot.cpp
    src/IoRT_Plot/IoRT_Plot_dialog.cpp

    ${iort_filters_MOCS}
)

target_link_libraries(${PROJECT_NAME}
    zbar
    jsoncpp_lib
    Qt5::Core
    Qt5::Widgets
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

## Install plugins
install(FILES plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
        