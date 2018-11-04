cmake_minimum_required(VERSION 2.8.3)
project(turtle_rv)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror -ggdb ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp roslib message_generation geometry_msgs sensor_msgs i2r_common nusmv)

add_message_files(FILES ObstaclePoint.msg ObstaclePoints.msg ObstaclePosition.msg ObstaclePositions.msg SetMode.msg)
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS roscpp roslib message_runtime geometry_msgs sensor_msgs i2r_common nusmv)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(turtle_rv_gazebo src/gazebo.cpp)
add_dependencies(turtle_rv_gazebo turtle_rv_generate_messages_cpp)
target_link_libraries(turtle_rv_gazebo ${catkin_LIBRARIES})

add_executable(turtle_rv_data_logger src/data_logger.cpp)
add_dependencies(turtle_rv_data_logger turtle_rv_generate_messages_cpp)
target_link_libraries(turtle_rv_data_logger ${catkin_LIBRARIES})


add_executable(turtle_rv_obstacles src/obstacles.cpp)
add_dependencies(turtle_rv_obstacles turtle_rv_generate_messages_cpp)
target_link_libraries(turtle_rv_obstacles ${catkin_LIBRARIES})

add_executable(turtle_rv_control src/control.cpp)
add_dependencies(turtle_rv_control turtle_rv_generate_messages_cpp)
target_link_libraries(turtle_rv_control ${catkin_LIBRARIES})

add_executable(turtle_rv_trust src/trust.cpp)
add_dependencies(turtle_rv_trust turtle_rv_generate_messages_cpp)
target_link_libraries(turtle_rv_trust ${catkin_LIBRARIES})

install(TARGETS turtle_rv_gazebo turtle_rv_obstacles turtle_rv_trust ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY monitors/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY msg/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY node/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
