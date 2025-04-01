if(CMAKE_VERSION VERSION_GREATER_EQUAL 4.0.0)
  cmake_minimum_required(VERSION 3.5)
else()
  cmake_minimum_required(VERSION 3.13.4)
endif()

project(rosbag_snapshot_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation rosgraph_msgs)
add_message_files(FILES SnapshotStatus.msg)
add_service_files(FILES TriggerSnapshot.srv)
generate_messages(DEPENDENCIES rosgraph_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime rosgraph_msgs
)

