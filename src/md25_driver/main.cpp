#include <map>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <cmath>
#include <md25_driver/md25.hpp>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <tf/transform_broadcaster.h>
//#include <tf2_ros/transform_broadcaster.h>

class MD25MotorDriverROSWrapper{
private:
  
  //ros::Subscriber speed_command_subscriber_;
  //ros::Subscriber lmotor_subscriber_;
  //ros::Subscriber rmotor_subscriber_;
  ros::Subscriber motor_twist_subscriber_;
  ros::Publisher current_speed_publisher_;
  ros::Publisher motor_status_publisher_;
  ros::Publisher motor_encoders_publisher_;
  ros::Publisher odom_publisher_;
  
  ros::ServiceServer stop_motor_server_;
  ros::ServiceServer reset_encoders_server_;

  ros::Timer current_speed_timer_;
  ros::Timer motor_speed_timer_;
  ros::Timer motor_status_timer_;
  ros::Timer motor_encoders_timer_;
  ros::Timer odom_timer_;
  ros::Timer pid_timer_;

  tf::TransformBroadcaster transform_broadcaster_;
  //-----------------------------------------------
  const long MAXPIDOUTPUT      = 127;
  const double PI = 3.14159265358979323846;
  double publish_current_speed_frequency_;
  double publish_motor_status_frequency_;
  double publish_motor_encoders_frequency_;
  double publish_odom_frequency_;
  double pid_frequency_;
  bool debug_mode_ = false;
  bool enable_speed_ = false;
  bool enable_encoders_ = false;
  bool enable_odom_ = false;
  bool enable_pid_ = false;
  bool enable_twist_ = false;
  bool enable_status_ = false;
  double wheelDiameter = 0.210;
  double wheelTrack = 0.345;
  double cpr = 1080;
  double ticksPerMeter = cpr / (PI * wheelDiameter);
  int motor_mode_ = 1;
  int moving = 0;
  //double DistancePerCount = (3.14159265 * 0.13) / cpr; 
//-------------------------------------------------
double Kp = 2.0;
double Ki = 0.5;
double Kd = 0.0;
double Ko = 20.0;

//-------------------------------------------------
/* Setpoint Info For a Motor */
typedef struct {
  double TargetSpeed=0;            // target speed in m/s
  double TargetTicksPerFrame=0;    // target speed in ticks per frame
  long Encoder=0;                  // encoder count
  long PrevEnc=0;                  // last encoder count
  int PrevErr=0;                   // last error
  int Ierror=0;                    // integrated error
  int output=0;                    // last motor setting
}
SetPointInfo;
SetPointInfo leftPID, rightPID;
typedef struct {
  ros::Time lastOdom;            // last ROS time odometry was calculated
  ros::Time encoderStamp;	 // last ROS time encoders were read
  unsigned long encoderTime;     // most recent millis() time encoders were read
  unsigned long lastEncoderTime; // last millis() time encoders were read
  unsigned long lastOdomTime;    // last millis() time odometry was calculated
  long prevLeftEnc;              // last left encoder reading used for odometry
  long prevRightEnc;             // last right encoder reading used for odometry
  float linearX;	         // total linear x distance traveled
  float linearY;	         // total linear y distance traveled
  float angularZ;		 // total angular distance traveled
}
OdomInfo;
OdomInfo odomInfo;
//---------------------------------------
public:
  double RobotX = 0.0;
  double RobotY = 0.0; 
  double RobotTH = 0.0;
  //ros::Time last_time;
  std::unique_ptr<md25_driver> motor;
//---------------------------------------
 MD25MotorDriverROSWrapper(ros::NodeHandle *nh){
    odomInfo.encoderTime = Millis();
    odomInfo.encoderStamp = ros::Time::now();
    setParams();
    //------------------------------------------
    motor.reset(new md25_driver("/dev/i2c-1"));  
    bool setup = motor->setup();
    bool mode1 = motor->setMode(motor_mode_);
    if(!setup){
      ROS_ERROR("failed to setup motor driver!");
    }
    if(!mode1){
      ROS_ERROR("failed to set motor to mode %d!",motor_mode_);
    }else{
      ROS_INFO("Motor Mode set to %d",motor_mode_);
    }
    bool resetted  = motor->resetEncoders();
    if(!resetted){
      ROS_ERROR("failed to reset encoders!");
    }
    //------------------------------------------
    if(enable_speed_){
      current_speed_timer_ = nh->createTimer(ros::Duration(1.0 / publish_current_speed_frequency_),&MD25MotorDriverROSWrapper::publishCurrentSpeed,this);
      current_speed_publisher_ = nh->advertise<std_msgs::ByteMultiArray>("current_speed",10);
    }
    if(enable_encoders_){
      motor_encoders_timer_ = nh->createTimer(ros::Duration(1.0 / publish_motor_encoders_frequency_),&MD25MotorDriverROSWrapper::publishEncoders,this);
      motor_encoders_publisher_ = nh->advertise<std_msgs::Int16MultiArray>("motor_encoders",10);
    }
    if(enable_twist_){
      motor_twist_subscriber_ = nh->subscribe("cmd_vel",2,&MD25MotorDriverROSWrapper::twistToMotors, this);
    }
    if(enable_status_){
      motor_status_timer_ = nh->createTimer(ros::Duration(1.0 / publish_motor_status_frequency_),&MD25MotorDriverROSWrapper::publishMotorStatus,this);
      motor_status_publisher_ = nh->advertise<std_msgs::ByteMultiArray>("motor_status",1);
    }
    if(enable_odom_){
      odom_timer_ = nh->createTimer(ros::Duration(1.0 / publish_odom_frequency_),&MD25MotorDriverROSWrapper::publishOdom,this);
      odom_publisher_ = nh->advertise<nav_msgs::Odometry>("odom",1);
    }
    if(enable_pid_){
      pid_timer_ = nh->createTimer(ros::Duration(1.0/pid_frequency_), &MD25MotorDriverROSWrapper::UpdatePID,this);
    }
     stop_motor_server_ = nh->advertiseService("md25_driver/stop_motor",&MD25MotorDriverROSWrapper::callbackStop, this);
     reset_encoders_server_ = nh->advertiseService("md25_driver/reset_encoders",&MD25MotorDriverROSWrapper::callbackReset,this);
    }
//---------------------------------------
void setParams(){
 if(!ros::param::get("~publish_current_speed_frequency",publish_current_speed_frequency_)){
      publish_current_speed_frequency_ = 0.0;
    }
    if(!ros::param::get("~publish_motor_status_frequency",publish_motor_status_frequency_)){
      publish_motor_status_frequency_ = 0.0;
    }
    if(!ros::param::get("~publish_motor_encoders_frequency",publish_motor_encoders_frequency_)){
       publish_motor_encoders_frequency_ = 0.0;
     }
     if(!ros::param::get("~publish_odom_frequency",publish_odom_frequency_)){
      publish_odom_frequency_ = 10.0;
    }
    if(!ros::param::get("~motor_mode",motor_mode_)){
      motor_mode_ = 1;
    }
    if(!ros::param::get("~pid_p",Kp)){
      Kp = 2.0;
    }
    if(!ros::param::get("~pid_i",Ki)){
      Ki = 0.5;
    }
    if(!ros::param::get("~pid_d",Kd)){
      Kd = 0.0;
    }
    
    if(!ros::param::get("~pid_frequency",pid_frequency_)){
      pid_frequency_ = 10.0;
    }
    if(!ros::param::get("~pid_o",Ko)){
      Ko = 2.0 * pid_frequency_;
    }
    if(!ros::param::get("~debug_mode",debug_mode_)){
      debug_mode_ = false;
    }
    if(publish_current_speed_frequency_ == 0.0){
      enable_speed_ = false;
    }else{
      enable_speed_ = true;
    }
    if(publish_motor_encoders_frequency_ == 0.0){
      enable_encoders_ = false;
    }else{
      enable_encoders_ = true;
    }
    if(!ros::param::get("~enable_twist",enable_twist_)){
      enable_twist_ = false;
    }
    if(publish_motor_status_frequency_ == 0.0){
      enable_status_ = false;
    }else{
      enable_status_ = true;
    }
    if(publish_odom_frequency_ == 0.0){
      enable_odom_ = false;
    }else{
      enable_odom_ = true;
    }
    if(!ros::param::get("~enable_pid",enable_pid_)){
      enable_pid_ = false;
    }
    ROS_INFO("Parameters Set");
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
  std::pair<int,int> speeds = motor->getMotorsSpeed();
  std_msgs::ByteMultiArray barry;
  barry.data.clear();
  barry.data.push_back(speeds.first);
  barry.data.push_back(speeds.second);
  current_speed_publisher_.publish(barry);
 }
//---------------------------------------
void publishEncoders(const ros::TimerEvent &event){
  std::pair<int,int> ticks = motor->readEncoders();
  std_msgs::ByteMultiArray barry;
  barry.data.clear();
  barry.data.push_back(ticks.first);
  barry.data.push_back(ticks.second);
  motor_encoders_publisher_.publish(barry);
 }
//---------------------------------------
void publishMotorStatus(const ros::TimerEvent &event){
  std::pair<int,int> currents = motor->getMotorsCurrent();
  std_msgs::ByteMultiArray barry;
  barry.data.clear();
  barry.data.push_back(currents.first);
  barry.data.push_back(currents.second);
  motor_status_publisher_.publish(barry);
 }
//---------------------------------------
void stop(){
    motor->stopMotors();
 }
//---------------------------------------
 void shutdown(){
    
 }
//---------------------------------------
void publishOdom(const ros::TimerEvent &event){
  //ros::Time current_time = leftPID.EncoderStamp;//ros::Time::now();
  double dt = (odomInfo.encoderTime - odomInfo.lastEncoderTime);
  odomInfo.lastEncoderTime = odomInfo.encoderTime;
  //double dt = (current_time - last_time).toSec();
  long deltaLeft = leftPID.Encoder - odomInfo.prevLeftEnc;
  long deltaRight = rightPID.Encoder - odomInfo.prevRightEnc;
  odomInfo.prevLeftEnc = leftPID.Encoder;
  odomInfo.prevRightEnc = rightPID.Encoder;
  
  double v_left = (deltaLeft * ticksPerMeter) / dt;
  double v_right = (deltaRight * ticksPerMeter) / dt;
  double v = ((v_right + v_left) / 2); //directional velocity
  double w = ((v_right - v_left)/wheelTrack); //angularvelocity
  double delta_x = (v * cos(w)) * dt;
  double delta_y = (v * sin(w)) * dt;
  double delta_th = w * dt;

  RobotX += delta_x;
  RobotY += delta_y;
  RobotTH += delta_th;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(RobotTH);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = odomInfo.encoderStamp;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = RobotX;
  odom_trans.transform.translation.y = RobotY;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  transform_broadcaster_.sendTransform(odom_trans);

  //Odometry message
  nav_msgs::Odometry odom;
  odom.header.stamp = odomInfo.encoderStamp;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = RobotX;
  odom.pose.pose.position.y = RobotY;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

   //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = v;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = w;

  odom_publisher_.publish(odom);
}
//--------------------------------------------------------
void twistToMotors(const geometry_msgs::Twist &msg){
  double x = msg.linear.x;
  double th = msg.angular.z;
  double spd_left,spd_right;
  if(x==0 && th ==0){
    moving = 0;
    motor->stopMotors();
    return;
  }
  moving = 1;
  if(x==0){
    //turn in place
    spd_right = th * wheelTrack / 2.0;
    spd_left = -spd_right;
  }else if(th == 0){
    //rotate around position
    spd_left = x - th * wheelTrack /2.0;
    spd_right = x + th * wheelTrack /2.0;
  }

  if(enable_pid_){
    /* Set the target speeds in meters per second */
    leftPID.TargetSpeed = spd_left;
    rightPID.TargetSpeed = spd_right;
    /* Convert speeds to encoder ticks per frame */
    leftPID.TargetTicksPerFrame = SpeedToTicks(leftPID.TargetSpeed);
    rightPID.TargetTicksPerFrame = SpeedToTicks(rightPID.TargetSpeed);
  }else{
    motor->writeSpeed(spd_left, spd_right);
  }
}
//------------------------------------------------------------------
/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  // Derivative error is the delta Perror
  output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  p->PrevErr = Perror;
  p->PrevEnc = p->Encoder;
  output += p->output;
  if (output >= MAXPIDOUTPUT)
    output = MAXPIDOUTPUT;
  else if (output <= -MAXPIDOUTPUT)
    output = -MAXPIDOUTPUT;
  else
    p->Ierror += Perror;
  p->output = output;
}
//---------------------------------------------------------------
/* Read the encoder values and call the PID routine */
void UpdatePID(const ros::TimerEvent &event) {
  long ticks_l;
  long ticks_r;
  //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  std::tie(ticks_l, ticks_r) = motor->readEncoders();
  long deltaLeft = ticks_l - leftPID.PrevEnc;
  long deltaRight = ticks_r - rightPID.PrevEnc;
  if(abs(deltaLeft) >1000|| abs(deltaRight) > 1000){
    return; // if error in reading value (noise/collision)
  }
  leftPID.Encoder = ticks_l;
  rightPID.Encoder = ticks_r;
  odomInfo.encoderTime = Millis();
  odomInfo.encoderStamp = ros::Time::now();
  
  if (!moving) return;
  /* Compute PID update for each motor */
  doPID(&leftPID);
  doPID(&rightPID);
  /* Set the motor speeds accordingly */
    motor->writeSpeed(leftPID.output, rightPID.output);
  if(debug_mode_){
    ROS_INFO("request: L=%ld, R=%ld", (long int)leftPID.output, (long int)rightPID.output);
  }
}
//---------------------------------------
int SpeedToTicks(double v) {
  return int(v * cpr / (pid_frequency_ * PI * wheelDiameter));
}
long Millis(){
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch()).count();
 }
};
//---------------------------------------
int main(int argc,char **argv){
  ros::init(argc,argv,"md25_driver");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  MD25MotorDriverROSWrapper motor_wrapper(&nh);
  
  ROS_INFO("MD25 Motor Driver v0.9.0 Started");
  ros::waitForShutdown();
  //motor_wrapper.stop();
  motor_wrapper.shutdown();
  return 0;
 }
