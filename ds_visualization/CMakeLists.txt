cmake_minimum_required(VERSION 2.8.3)
project(ds_visualization)

if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  ds_av_msgs
)

catkin_package(
  DEPENDS
    rviz
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  src
)

find_package(Qt5Widgets REQUIRED)
add_definitions(-DQT_NO_KEYWORDS -g)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(HEADER_FILES
  src/dataspeedObject_array_display.h
  src/dataspeedObject_display.h
)

qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})

set(SOURCE_FILES
   src/dataspeedObject_array_display.cpp
   src/dataspeedObject_display.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME}
  ${SOURCE_FILES}
  ${UIC_FILES}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
  ${rviz_DEFAULT_PLUGIN_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
