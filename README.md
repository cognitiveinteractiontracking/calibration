# calibration

## one_shot_calib_node

tbd

## dynamic_transform_publisher

tbd


## yaml_handler.py

Load calibration.yaml (sensor pose, path rosbag tracking data) via rosparam. The script will read the rosbags, transform the poses to the same frame as the sensors und output the data to a gm2dl file, which is readable for the g2o library.

## inverse_transform.py

Calculates the inverse transformation and prints it to std::out so that it can be used by tf static_transform_publisher

### Examples

```
$ ./inverse_transform.py -x 100. -y 2 -z 0 -R 0 -P 0 -Y 0
-100.0000 -2.0000 0.0000 0.0000 0.0000 0.0000 1.0000
```

`static_inverse_transform_publisher` does the following in a generic way:
```
$ rosrun tf static_transform_publisher $(./inverse_transform.py -x 100. -y 2 -z 0 -R 0 -P 0 -Y 0) source_frame target_frame 100
```

## odom_to_tf

This node updates a transformation between two coordinate frames with a given ros::nav_msgs::Odometry.

### Parameter

|          Name           |  Type  | Default |                                                            Description                                                            |
| ----------------------- | ------ | ------- | --------------------------------------------------------------------------------------------------------------------------------- |
| ros_listener_odom_topic | string | /topic  | Listenertopic for the ros::nav_msgs::Odometry to update the tf.                                                                   |
| parent_frame            | string | /parent | Parent frame id                                                                                                                   |
| child_frame             | string | /child  | Child frame id                                                                                                                    |
| rostimenow              | bool   | false   | If this is set to true, ros::now as headertimestamp will be used otherwhise the timestamp from the input ros::nav_msgs::Odometry. | 

