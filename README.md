# md25_driver
A ROS driver for the md25 motor controller

Added functions to set speed etc. - ported from Arduino MD25 driver - Josh Villbrandt (http://javconcepts.com/), July 7, 2012.

Wrapped MD25 ROS Driver - v0.1

Parameters:
* publish_current_speed_frequency = 5.0
* publish_motor_status_frequency = 5.0
* publish_motor_encoders_frequency = 1.0

Subscriptions:
* speed_command - std_msgs::ByteMultiArray[2] - left/right

Advertisements:
* current_speed - std_msgs::ByteMultiArray[2] - left/right
* motor_status - vector<diagnostic_msgs::KeyValue>[3] - motor current left/right, batteryvolts
* motor_encoders - std_msgs::Int32MultiArray[2] - left/right

Services:
* stop_motor - bool
* reset_encoders - bool

