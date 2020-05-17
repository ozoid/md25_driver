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
  //ros::Publisher current_speed_publisher_;
  ros::Publisher motor_status_publisher_;
  //ros::Publisher motor_encoders_publisher_;
  ros::Publisher odom_publisher_;
  //ros::Publisher lmotor_encoder_publisher_;
  //ros::Publisher rmotor_encoder_publisher_;

  ros::ServiceServer stop_motor_server_;
  ros::ServiceServer reset_encoders_server_;

  //ros::Timer current_speed_timer_;
  ros::Timer motor_status_timer_;
  //ros::Timer motor_encoders_timer_;
  ros::Timer odom_timer_;
  ros::Timer pid_timer_;

  tf::TransformBroadcaster transform_broadcaster_;
  //-----------------------------------------------
  const long MAXPIDOUTPUT      = 100;
  const double PI = 3.14159265358979323846;
  //double publish_current_speed_frequency_;
  double publish_motor_status_frequency_;
  //double publish_motor_encoders_frequency_;
  double publish_odom_frequency_;
  double pid_frequency_;
  
  double wheelDiameter = 0.210;
  double wheelTrack = 0.345;
  double cpr = 1080;
  double ticksPerMeter = cpr / (PI * wheelDiameter);

  int moving = 0;
  //double DistancePerCount = (3.14159265 * 0.13) / cpr; 
//-------------------------------------------------
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

//-------------------------------------------------
/* Setpoint Info For a Motor */
typedef struct {
  double TargetSpeed;            // target speed in m/s
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  int PrevErr;                   // last error
  int Ierror;                    // integrated error
  int output;                    // last motor setting
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
    motor.reset(new md25_driver("/dev/i2c-1"));  
    bool setup = motor->setup();
    if(!setup){
      ROS_ERROR("failed to setup motor driver!");
    }
    motor->resetEncoders();
    // if(!ros::param::get("~publish_current_speed_frequency",publish_current_speed_frequency_)){
    //   publish_current_speed_frequency_ = 5.0;
    // }
    if(!ros::param::get("~publish_motor_status_frequency",publish_motor_status_frequency_)){
      publish_motor_status_frequency_ = 1.0;
    }
    // if(!ros::param::get("~publish_motor_encoders_frequency",publish_motor_encoders_frequency_)){
    //   publish_motor_encoders_frequency_ = 1.0;
    // }
     if(!ros::param::get("~publish_odom_frequency",publish_odom_frequency_)){
      publish_odom_frequency_ = 10.0;
    }
     if(!ros::param::get("~pid_frequency",publish_odom_frequency_)){
      pid_frequency_ = 10.0;
    }
    //--------
    
    //lmotor_subscriber_ = nh->subscribe("lmotor",2,&MD25MotorDriverROSWrapper::callbackLSpeedCommand, this);
    //rmotor_subscriber_ = nh->subscribe("rmotor",2,&MD25MotorDriverROSWrapper::callbackRSpeedCommand, this);
    motor_twist_subscriber_ = nh->subscribe("cmd_vel",2,&MD25MotorDriverROSWrapper::twistToMotors, this);
      
    stop_motor_server_ = nh->advertiseService("md25_driver/stop_motor",&MD25MotorDriverROSWrapper::callbackStop, this);
    reset_encoders_server_ = nh->advertiseService("md25_driver/reset_encoders",&MD25MotorDriverROSWrapper::callbackReset,this);

    //current_speed_publisher_ = nh->advertise<std_msgs::ByteMultiArray>("current_speed",10);
    motor_status_publisher_ = nh->advertise<diagnostic_msgs::DiagnosticStatus>("motor_status",10);
    //motor_encoders_publisher_ = nh->advertise<std_msgs::Int16MultiArray>("motor_encoders",10);
    //lmotor_encoder_publisher_= nh->advertise<std_msgs::Int16>("lwheel",10);
    //rmotor_encoder_publisher_= nh->advertise<std_msgs::Int16>("rwheel",10);
    odom_publisher_ = nh->advertise<nav_msgs::Odometry>("odom",10);

    //current_speed_timer_ = nh->createTimer(ros::Duration(1.0 / publish_current_speed_frequency_),&MD25MotorDriverROSWrapper::publishCurrentSpeed,this);
    motor_status_timer_ = nh->createTimer(ros::Duration(1.0 / publish_motor_status_frequency_),&MD25MotorDriverROSWrapper::publishMotorStatus,this);
    //motor_encoders_timer_ = nh->createTimer(ros::Duration(1.0 / publish_motor_encoders_frequency_),&MD25MotorDriverROSWrapper::publishEncoders,this);

    odom_timer_ = nh->createTimer(ros::Duration(1.0 / publish_odom_frequency_),&MD25MotorDriverROSWrapper::publishOdom,this);
    pid_timer_ = nh->createTimer(ros::Duration(1.0/pid_frequency_), &MD25MotorDriverROSWrapper::UpdatePID,this);
  }
//---------------------------------------
void callbackLSpeedCommand(const std_msgs::Int16 &msg){
      motor->setMotor1Speed(msg.data);
  }
//---------------------------------------
void callbackRSpeedCommand(const std_msgs::Int16 &msg){
      motor->setMotor2Speed(msg.data);
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
   //current_speed_publisher_.publish(barry);
 }
//---------------------------------------
void publishEncoders(const ros::TimerEvent &event){
   std::pair<int,int> ticks = motor->readEncoders();
    std_msgs::Int16 lwheel;
    lwheel.data = ticks.first;
    std_msgs::Int16 rwheel;
    rwheel.data = ticks.second;
  // lmotor_encoder_publisher_.publish(lwheel);
  // rmotor_encoder_publisher_.publish(rwheel);
 }
//---------------------------------------
void publishMotorStatus(const ros::TimerEvent &event){
  diagnostic_msgs::DiagnosticStatus msg;
  std::vector<diagnostic_msgs::KeyValue>values;
  diagnostic_msgs::KeyValue value1;
  value1.key = "M1";
  value1.value = (int)motor->getMotor1Current();
  values.push_back(value1);
  diagnostic_msgs::KeyValue value2;
  value2.key = "M2";
  value2.value = (int)motor->getMotor2Current();
  values.push_back(value2);
  diagnostic_msgs::KeyValue value3;
  value3.key = "B1";
  value3.value = (int)motor->getBatteryVolts();
  values.push_back(value3);
  msg.values = values;
  motor_status_publisher_.publish(msg);
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
   /* Set the target speeds in meters per second */
  leftPID.TargetSpeed = spd_left;
  rightPID.TargetSpeed = spd_right;
  /* Convert speeds to encoder ticks per frame */
  leftPID.TargetTicksPerFrame = SpeedToTicks(leftPID.TargetSpeed);
  rightPID.TargetTicksPerFrame = SpeedToTicks(rightPID.TargetSpeed);

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
  ROS_INFO("MD25 Motor Driver v0.7.0 Started");
  ros::waitForShutdown();
  motor_wrapper.stop();
  motor_wrapper.shutdown();
  return 0;
 }
