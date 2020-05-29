# md25_driver
A ROS driver for the md25 motor controller with PID control - v0.9.2

Parameters:
* publish_motor_status_frequency = 1.0
* publish_odom_frequency = 10.0
* publish_current_speed_frequency = 0.0
* publish_motor_encoders_frequency = 0.0
* pid_frequency = 20.0
* enable_twist = true
* enable_pid = true
* motor_mode = 1
* pid_p = 2.0
* pid_i = 0.5
* pid_d = 0.0
* pid_o = 100.0
* debug_mode = false

Subscriptions:
* cmd_vel - geometry_msgs::Twist

Advertisements:
* motor_status - std_msgs::ByteMultiArray - motor current left/right, battery volts, motor speed left/right
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
