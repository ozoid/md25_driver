# md25_driver
A ROS driver for the md25 motor controller

Added functions to set speed etc. - ported from Arduino MD25 driver - Josh Villbrandt (http://javconcepts.com/), July 7, 2012.

Wrapped MD25 ROS Driver - v0.4

Parameters:
* publish_current_speed_frequency = 5.0
* publish_motor_status_frequency = 5.0
* publish_motor_encoders_frequency = 1.0
* publish_odom_frequency = 1.0

Subscriptions:
* speed_command - std_msgs::ByteMultiArray[2] - left/right

Advertisements:
* current_speed - std_msgs::ByteMultiArray[2] - left/right
* motor_status - vector<diagnostic_msgs::KeyValue>[3] - motor current left/right, batteryvolts
* motor_encoders - std_msgs::Int32MultiArray[2] - left/right
* odom - nav_msgs::Odometry

Services:
* md25_driver/stop_motor - bool
* md25_driver/reset_encoders - bool

```
<launch>
    <node name="md25_driver" pkg="md25_driver" type="md25_driver_node" launch-prefix="" output="screen">
        <param name="max_speed" type="int" value="5" />
        <param name="publish_current_speed_frequency" type="double" value="10.0" />
        <param name="publish_motor_status_frequency" type="double" value="2.0" />
        <param name="publish_motor_encoders_frequency" type="double" value="1.0"/>
        <param name="publish_odom_frequency" type="double" value="1.0">
    </node>
<<<<<<< HEAD
</launch>```

Requires read/write access to /dev/i2c-x - Ubuntu requires permission changes

=======
</launch>
```
>>>>>>> 25139c6eef3f095d40672313534fc39b31ea8feb
