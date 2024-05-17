cmake_minimum_required(VERSION 3.0.2)
project(rosbag_snapshot)

set(PACKAGE_DEPENDENCIES rosbag rosbag_snapshot_msgs roscpp rosgraph_msgs std_srvs topic_tools)
set(PACKAGE_CATKIN_COMPONENTS ${PACKAGE_DEPENDENCIES})
if(CATKIN_ENABLE_TESTING)
  list(APPEND PACKAGE_CATKIN_COMPONENTS roslint rostest)
endif()

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_CATKIN_COMPONENTS})

find_package(Boost REQUIRED COMPONENTS program_options regex)

catkin_package(
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    INCLUDE_DIRS include
    LIBRARIES rosbag_snapshot
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(rosbag_snapshot src/snapshotter.cpp)
add_dependencies(rosbag_snapshot ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosbag_snapshot ${catkin_LIBRARIES})

add_executable(snapshot src/snapshot.cpp)
add_dependencies(snapshot ${catkin_EXPORTED_TARGETS})
target_link_libraries(snapshot rosbag_snapshot ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  roslint_python(test)
  roslint_cpp()
  roslint_add_test()

  add_rostest(test/snapshot.test)
endif()

install(TARGETS snapshot
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS rosbag_snapshot
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
