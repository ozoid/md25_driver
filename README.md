# md25_driver
A ROS driver for the md25 motor controller - v0.7.0

Parameters:
* publish_motor_status_frequency = 1.0
* publish_odom_frequency = 10.0

Subscriptions:
* cmd_vel - geometry_msgs::Twist

Advertisements:
* motor_status - vector<diagnostic_msgs::KeyValue>[3] - motor current left/right, battery volts
* odom - nav_msgs::Odometry
* tf - geometry_msgs::TransformStamped

Services:
* md25_driver/stop_motor - bool
* md25_driver/reset_encoders - bool

```
<launch>
    <node name="md25_driver" pkg="md25_driver" type="md25_driver_node" launch-prefix="" output="screen">
        <param name="publish_motor_status_frequency" type="double" value="0.1" />
        <param name="publish_odom_frequency" type="double" value="20.0">
    </node>
</launch>
```

Requires read/write access to /dev/i2c-x - Ubuntu requires permission changes
