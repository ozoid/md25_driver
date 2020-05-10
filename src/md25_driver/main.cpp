#include <map>
#include <string>
#include <vector>
#include <memory>
#include <md25_driver/md25.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <tf/transform_broadcaster.h>
//#include <tf2_ros/transform_broadcaster.h>

class MD25MotorDriverROSWrapper{
private:
  std::unique_ptr<md25_driver> motor;
  ros::Subscriber speed_command_subscriber_;

  ros::Publisher current_speed_publisher_;
  ros::Publisher motor_status_publisher_;
  ros::Publisher motor_encoders_publisher_;
  ros::Publisher odom_publisher_;

  ros::ServiceServer stop_motor_server_;
  ros::ServiceServer reset_encoders_server_;

  ros::Timer current_speed_timer_;
  ros::Timer motor_status_timer_;
  ros::Timer motor_encoders_timer_;
  ros::Timer odom_timer_;

  tf::TransformBroadcaster transform_broadcaster_;
  
  double publish_current_speed_frequency_;
  double publish_motor_status_frequency_;
  double publish_motor_encoders_frequency_;
  double publish_odom_frequency_;

  long _PreviousLeftEncoderCounts = 0;
  long _PreviousRightEncoderCounts =0;
  double DistancePerCount = (3.14159265 * 0.13) / 2626; 
  double lengthBetweenTwoWheels = 0.25;

public:
  double x = 0.0;
  double y = 0.0; 
  double th_angle = 0.0;
  ros::Time last_time;

 MD25MotorDriverROSWrapper(ros::NodeHandle *nh){
    motor.reset(new md25_driver("/dev/i2c-1"));  
    bool setup = motor->setup();
    if(!setup){
      ROS_ERROR("failed to setup motor driver!");
    }
    motor->resetEncoders();
    int max_speed;
    if(!ros::param::get("~max_speed",max_speed)){
      max_speed = 120;
    }
    //--------
    if(!ros::param::get("~publish_current_speed_frequency",publish_current_speed_frequency_)){
      publish_current_speed_frequency_ = 5.0;
    }
    if(!ros::param::get("~publish_motor_status_frequency",publish_motor_status_frequency_)){
      publish_motor_status_frequency_ = 5.0;
    }
    if(!ros::param::get("~publish_motor_encoders_frequency",publish_motor_encoders_frequency_)){
      publish_motor_encoders_frequency_ = 1.0;
    }
     if(!ros::param::get("~publish_odom_frequency",publish_motor_encoders_frequency_)){
      publish_motor_encoders_frequency_ = 1.0;
    }
    //--------
    
    speed_command_subscriber_ = nh->subscribe("speed_command",10,&MD25MotorDriverROSWrapper::callbackSpeedCommand, this);
    stop_motor_server_ = nh->advertiseService("md25_driver/stop_motor",&MD25MotorDriverROSWrapper::callbackStop, this);
    reset_encoders_server_ = nh->advertiseService("md25_driver/reset_encoders",&MD25MotorDriverROSWrapper::callbackReset,this);

    current_speed_publisher_ = nh->advertise<std_msgs::ByteMultiArray>("current_speed",10);
    motor_status_publisher_ = nh->advertise<diagnostic_msgs::DiagnosticStatus>("motor_status",10);
    motor_encoders_publisher_ = nh->advertise<std_msgs::Int32MultiArray>("motor_encoders",10);
    odom_publisher_ = nh->advertise<nav_msgs::Odometry>("odom",10);


    current_speed_timer_ = nh->createTimer(ros::Duration(1.0 / publish_current_speed_frequency_),&MD25MotorDriverROSWrapper::publishCurrentSpeed,this);
    motor_status_timer_ = nh->createTimer(ros::Duration(1.0 / publish_current_speed_frequency_),&MD25MotorDriverROSWrapper::publishMotorStatus,this);
    motor_encoders_timer_ = nh->createTimer(ros::Duration(1.0 / publish_motor_encoders_frequency_),&MD25MotorDriverROSWrapper::publishEncoders,this);
    odom_timer_ = nh->createTimer(ros::Duration(1.0 / publish_odom_frequency_),&MD25MotorDriverROSWrapper::publishOdom,this);
  }
//---------------------------------------
void callbackSpeedCommand(const std_msgs::ByteMultiArray &msg){
      motor->setMotor1Speed(msg.data[0]);
      motor->setMotor2Speed(msg.data[1]);
  }
//---------------------------------------
bool callbackReset(std_srvs::Trigger::Request &req, std_srvs::TriggerResponse &res){
    motor->resetEncoders();
    res.success = true;
    res.message = "Encoders Reset";
    return true;
 }
//---------------------------------------
bool callbackStop(std_srvs::Trigger::Request &req, std_srvs::TriggerResponse &res){
    stop();
    res.success = true;
    res.message = "Stopped Motors";
    return true;
  }
//---------------------------------------
void publishCurrentSpeed(const ros::TimerEvent &event){
   std_msgs::ByteMultiArray barry;
   barry.data.clear();
   barry.data.push_back(motor->getMotor1Speed());
   barry.data.push_back(motor->getMotor2Speed());
   current_speed_publisher_.publish(barry);
 }
//---------------------------------------
void publishEncoders(const ros::TimerEvent &event){
   std_msgs::Int32MultiArray irry;
   irry.data.clear();
   irry.data.push_back(motor->getEncoder1());
   irry.data.push_back(motor->getEncoder2());
   motor_encoders_publisher_.publish(irry);
 }
//---------------------------------------
void publishMotorStatus(const ros::TimerEvent &event){
  diagnostic_msgs::DiagnosticStatus msg;
  //std::map<std::string,std::string>status = motor->getMotor1Current();
  std::vector<diagnostic_msgs::KeyValue>values;
  diagnostic_msgs::KeyValue value1;
  value1.key = "M1";
  value1.value = motor->getMotor1Current();
  values.push_back(value1);
  diagnostic_msgs::KeyValue value2;
  value2.key = "M2";
  value2.value = motor->getMotor2Current();
  values.push_back(value2);
  diagnostic_msgs::KeyValue value3;
  value3.key = "B1";
  value3.value = motor->getBatteryVolts();
  values.push_back(value3);
  msg.values = values;
  motor_status_publisher_.publish(msg);
 }
//---------------------------------------
void stop(){
    motor->stop_motors();
 }
//---------------------------------------
void publishOdom(const ros::TimerEvent &event){
  ros::Time current_time = ros::Time::now();
  long tick_l;
  long tick_r;
  std::tie(tick_l, tick_r) = motor->get_encoders();
  //extract the wheel velocities from the tick signals count
  long deltaLeft = tick_l - _PreviousLeftEncoderCounts;
  long deltaRight = tick_r - _PreviousRightEncoderCounts;
  double v_left = (deltaLeft * DistancePerCount) / (current_time - last_time).toSec();
  double v_right = (deltaRight * DistancePerCount) / (current_time - last_time).toSec();
  double v = ((v_right + v_left) / 2); //directional velocity
  double w = ((v_right - v_left)/lengthBetweenTwoWheels); //angularvelocity

  double dt = (current_time - last_time).toSec();
  double delta_x = (v * cos(w)) * dt;
  double delta_y = (v * sin(w)) * dt;
  double delta_th = w * dt;

  x += delta_x;
  y += delta_y;
  th_angle += delta_th;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_angle);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  transform_broadcaster_.sendTransform(odom_trans);

  //Odometry message
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

   //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = v;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = w;

  //publish the message
  odom_publisher_.publish(odom);
  _PreviousLeftEncoderCounts = tick_l;
  _PreviousRightEncoderCounts = tick_r;

  last_time = current_time;

}


};
//---------------------------------------
int main(int argc,char **argv){
  ros::init(argc,argv,"md25_driver");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  MD25MotorDriverROSWrapper motor_wrapper(&nh);
  ROS_INFO("MD25 Motor Driver Started");
  ros::waitForShutdown();
  motor_wrapper.stop();
 }
