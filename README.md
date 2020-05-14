# md25_driver
A ROS driver for the md25 motor controller

Added functions to set speed etc. - ported from Arduino MD25 driver - Josh Villbrandt (http://javconcepts.com/), July 7, 2012.

Wrapped MD25 ROS Driver - v0.5.2

Parameters:
* publish_motor_status_frequency = 1.0
* publish_motor_encoders_frequency = 10.0

Subscriptions:
* speed_command - std_msgs::ByteMultiArray[2] - left/right

Advertisements:
* motor_status - vector<diagnostic_msgs::KeyValue>[3] - motor current left/right, batteryvolts
* motor_encoders - std_msgs::Int16MultiArray> - left/right wheel encoders

Services:
* md25_driver/stop_motor - bool
* md25_driver/reset_encoders - bool

```
<launch>
    <node name="md25_driver" pkg="md25_driver" type="md25_driver_node" launch-prefix="" output="screen">
        <param name="publish_motor_status_frequency" type="double" value="0.1" />
        <param name="publish_motor_encoders_frequency" type="double" value="10.0">
    </node>
</launch>
```

Requires read/write access to /dev/i2c-x - Ubuntu requires permission changes
