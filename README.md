# md25_driver
A ROS driver for the Devantech MD25 motor controller with PID control - v0.9.5

Parameters:
* publish_motor_status_frequency = 1.0
* publish_odom_frequency = 10.0
* publish_current_speed_frequency = 0.0
* publish_motor_encoders_frequency = 0.0
* pid_frequency = 40.0
* enable_twist = true
* enable_pid = true
* motor_mode = 1 (-127|0|+127)
* max_speed = 127
* pid_p = 20.0
* pid_i = 12.0
* pid_d = 0.0
* pid_o = 50.0
* debug_mode = false

Subscriptions:
* cmd_vel - geometry_msgs::Twist

Advertisements:
* motor_status - std_msgs::ByteMultiArray - motor current left/right, battery volts, motor speed left/right
* odom - nav_msgs::Odometry
* tf - geometry_msgs::TransformStamped

Services:
* md25_driver/stop_motors - bool
* md25_driver/reset_encoders - bool

```
<launch>
<node name="md25_driver" pkg="md25_driver" type="md25_driver_node" launch-prefix="" output="screen">
        <param name="max_speed" type="int" value="80" /> 
        <param name="publish_current_speed_frequency" type="double" value="0.0" />
        <param name="publish_motor_status_frequency" type="double" value="0.0" />
        <param name="debug_mode" type="bool" value="false" />
        <param name="pid_frequency" type="double" value="40.0" />
        <param name="publish_motor_encoders_frequency" type="double" value="0.0"/>
        <param name="publish_odom_frequency" type="double" value="10.0"/>
        <param name="enable_twist" type="bool" value="true"/>
        <param name="enable_pid" type="bool" value="true"/>
        <param name="motor_mode" type="int" value="1"/>
        <param name="pid_p" type="double" value="20"/>
        <param name="pid_i" type="double" value="12"/>
        <param name="pid_d" type="double" value="0.0"/>
        <param name="pid_o" type="double" value="50.0"/>
 </node>
 </launch>
```

Requires read/write access to /dev/i2c-x - Ubuntu requires permission changes
