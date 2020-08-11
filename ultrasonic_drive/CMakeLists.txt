cmake_minimum_required(VERSION 3.0.2)
project(ultrasonic_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/
)

add_executable(ultrasonic_drive
    src/main.cpp
    src/ultrasonic.cpp
    )

target_link_libraries(ultrasonic_drive
    ${catkin_LIBRARIES})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
    )

install(TARGETS ultrasonic_drive
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

