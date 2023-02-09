# rosbag_snapshot

Solution for [this ros_comm issue](https://github.com/ros/ros_comm/issues/1399) which acts similarly to the deprecated `rosrecord -s` command but with additional features. It is added as a new package here rather than patching `rosbag` based on [the discussion here](https://github.com/ros/ros_comm/pull/1414).

It subscribes to topics and maintains a buffer of recent messages like a dash cam. This is useful in live testing where unexpected events can occur which would be useful to have data on but the opportunity is missed if `rosbag record` was not running (disk space limits make always running `rosbag record` impracticable). Instead, users may run snapshot in the background and save data from the recent past to disk as needed.


## Usage

`rosbag_snapshot` can be configured through command line flags and with ROS params for more granular control. By default, the command will run in server mode (buffering data). When certain flags are used, program will act as a client by requesting that the server write data to disk or freezing the buffer to preserve interesting data until a user can decide what to write.

### CLI usage

```
$ rosrun rosbag_snapshot snapshot -h
Usage: snapshot [options] [topic1 topic2 ...]

Buffer recent messages until triggered to write or trigger an already running instance.

Options:
  -h [ --help ]                produce help message
  -t [ --trigger-write ]       Write buffer of selected topics to a bag file
  -p [ --pause ]               Stop buffering new messages until resumed or
                               write is triggered
  -r [ --resume ]              Resume buffering new messages, writing over
                               older messages as needed
  -s [ --size ] arg (=-1)      Maximum memory per topic to use in buffering in
                               MB. Default: no limit
  -d [ --duration ] arg (=30)  Maximum difference between newest and oldest
                               buffered message per topic in seconds. Default:
                               30
  -o [ --output-prefix ] arg   When in trigger write mode, prepend PREFIX to
                               name of writing bag file
  -O [ --output-filename ] arg When in trigger write mode, exact name of
                               written bag file
  --topic arg                  Topic to buffer. If triggering write, write only
                               these topics instead of all buffered topics.
  -c [ --compression ] arg     Compression type. Options are: LZ4, BZ2.
                               Default: uncompressed.
```

###### Hold a buffer of the last 30 seconds of data from selected topics until triggered to write
`rosrun rosbag_snapshot snapshot -d 30 /tf /odom /camera/image_color /camera/camera_info /velodyne_points`

###### Buffer the most recent gigabyte of the following topics in the camera namespace
`ROS_NAMESPACE=camera rosrun rosbag_snapshot snapshot -s 1000 image_rect_color camera_info`


### Example launch file
```
<launch>
  <node name="snapshot" pkg="rosbag_snapshot" type="snapshot" args="">
    <rosparam>
        default_duration_limit: 1  # Maximum time difference between newest and oldest message, seconds, overrides -d flag
        default_memory_limit: 0.1  # Maximum memory used by messages in each topic's buffer, MB, override -s flag
        compression: LZ4           # LZ4, BZ2, uncompressed
        topics:
            - test1                # Inherit defaults
            - test2:               # Override duration limit, inherit memory limit
                duration: 2
            - test3:               # Override both limits
                duration: -1       # Negative means no duration limit
                memory: 0.00
    </rosparam>
  </node>
</launch>
```

## Client examples

###### Write all buffered data to `<datetime>.bag`
`rosrun rosbag_snapshot snapshot -t`

###### Write buffered data from selected topics to `new_lighting<datetime>.bag`
`rosrun rosbag_snapshot snapshot -t -o new_lighting /camera/image_raw /camera/camera_info`

###### Write all buffered data to `/home/user/crashed_into_wall.bag`
`rosrun rosbag_snapshot snapshot -t -O /home/user/crashed_into_wall`

###### Pause buffering of new data, holding current buffer in memory until -t or -r is used
`rosrun rosbag_snapshot snapshot -p`

###### Resume buffering new data
`rosrun rosbag_snapshot snapshot -r`

###### Call trigger service manually, specifying absolute window start and stop time for written data

```
$ rosservice call /trigger_snapshot "filename: 'short_control_instability.bag'
topics:
- '/tf'
- '/odom'
- '/wrench'
- '/motors/cmd'
start_time:
  secs: 62516
  nsecs: 0
stop_time:
  secs: 62528
  nsecs: 0"
```

###### View status of buffered data (useful for future tools/GUI)

```
$ rostopic echo /snapshot_status
topics:
  -
    topic: "/test"
    node_pub: ''
    node_sub: "/snapshot_1527185221120605085"
    window_start:
      secs: 62516
      nsecs: 761000000
    window_stop:
      secs: 62521
      nsecs: 984000000
    delivered_msgs: 524
    dropped_msgs: 0
    traffic: 2096
    period_mean:
      secs: 0
      nsecs:         0
    period_stddev:
      secs: 0
      nsecs:         0
    period_max:
      secs: 0
      nsecs:         0
    stamp_age_mean:
      secs: 0
      nsecs:         0
    stamp_age_stddev:
      secs: 0
      nsecs:         0
    stamp_age_max:
      secs: 0
      nsecs:         0
enabled: True
---
```
