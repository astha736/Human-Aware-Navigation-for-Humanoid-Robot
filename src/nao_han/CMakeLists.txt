cmake_minimum_required(VERSION 2.8.3)
project(nao_han)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  message_filters
  sensor_msgs
)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(base_footprint_NAO_HAN src/base_footprint_NAO_HAN.cpp)
target_link_libraries(base_footprint_NAO_HAN  ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS ./nodes/new_nao_walker.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
