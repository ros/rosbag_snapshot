cmake_minimum_required(VERSION 2.8.3)
project(rosbag_snapshot_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation rosgraph_msgs)
add_message_files(FILES SnapshotStatus.msg)
add_service_files(FILES TriggerSnapshot.srv)
generate_messages(DEPENDENCIES rosgraph_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime rosgraph_msgs
)

