# md25_driver
A ROS driver for the md25 motor controller

Added functions to set speed etc. - ported from Arduino MD25 driver - Josh Villbrandt (http://javconcepts.com/), July 7, 2012.

Wrapped MD25 ROS Driver - v0.1

Parameters:
   publish_current_speed_frequency = 5.0
   publish_motor_status_frequency = 1.0
   publish_motor_encoders_frequency = 1.0

Subscriptions:
   speed_command

Advertisements:
   current_speed
   motor_status
   motor_encoders

Services:
   stop_motor
   reset_encoders

