# md25_driver
A ROS driver for the md25 motor controller

Added functions to set speed etc. - ported from Arduino MD25 driver - Josh Villbrandt (http://javconcepts.com/), July 7, 2012.

Wrapped MD25 ROS Driver - v0.5

Parameters:
* publish_motor_status_frequency = 1.0
* publish_odom_frequency = 10.0

Subscriptions:
* speed_command - std_msgs::ByteMultiArray[2] - left/right

Advertisements:
* motor_status - vector<diagnostic_msgs::KeyValue>[3] - motor current left/right, batteryvolts
* odom - nav_msgs::Odometry

Services:
* md25_driver/stop_motor - bool
* md25_driver/reset_encoders - bool

```
<launch>
    <node name="md25_driver" pkg="md25_driver" type="md25_driver_node" launch-prefix="" output="screen">
        <param name="max_speed" type="int" value="5" />
        <param name="publish_motor_status_frequency" type="double" value="1.0" />
        <param name="publish_odom_frequency" type="double" value="10.0">
    </node>
</launch>
```

Requires read/write access to /dev/i2c-x - Ubuntu requires permission changes
