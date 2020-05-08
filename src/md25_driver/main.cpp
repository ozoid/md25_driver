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
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

class MD25MotorDriverROSWrapper{
private:
  std::unique_ptr<md25_driver> motor;
  ros::Subscriber speed_command_subscriber_;
  ros::Publisher current_speed_publisher_;
  ros::Publisher motor_status_publisher_;
  ros::Publisher motor_encoders_publisher_;
  ros::ServiceServer stop_motor_server_;
  ros::ServiceServer reset_encoders_server_;
  ros::Timer current_speed_timer_;
  ros::Timer motor_status_timer_;
  ros::Timer motor_encoders_timer_;
  double publish_current_speed_frequency_;
  double publish_motor_status_frequency_;
  double publish_motor_encoders_frequency_;
public:
 MD25MotorDriverROSWrapper(ros::NodeHandle *nh){

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
    //--------
    speed_command_subscriber_ = nh->subscribe("speed_command",10,&MD25MotorDriverROSWrapper::callbackSpeedCommand, this);
    stop_motor_server_ = nh->advertiseService("stop_motor",&MD25MotorDriverROSWrapper::callbackStop, this);
    reset_encoders_server_ = nh->advertiseService("reset_encoders",&MD25MotorDriverROSWrapper::callbackReset,this);
    current_speed_publisher_ = nh->advertise<std_msgs::ByteMultiArray>("current_speed",10);
    motor_status_publisher_ = nh->advertise<diagnostic_msgs::DiagnosticStatus>("motor_status",10);
    motor_encoders_publisher_ = nh->advertise<std_msgs::Int32MultiArray>("motor_encoders",10);

    current_speed_timer_ = nh->createTimer(ros::Duration(1.0 / publish_current_speed_frequency_),&MD25MotorDriverROSWrapper::publishCurrentSpeed,this);
    motor_status_timer_ = nh->createTimer(ros::Duration(1.0/publish_current_speed_frequency_),&MD25MotorDriverROSWrapper::publishMotorStatus,this);
    motor_encoders_timer_ = nh->createTimer(ros::Duration(1.0/publish_motor_encoders_frequency_),&MD25MotorDriverROSWrapper::publishEncoders,this);
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
  value1.key = "M1"
  value1.value = motor->getMotor1Current();
  values.push_back(value1);
  diagnostic_msgs::KeyValue value2;
  value2.key = "M2"
  value2.value = motor->getMotor2Current();
  values.push_back(value2);
  diagnostic_msgs::KeyValue value3;
  value3.key = "B1"
  value3.value = motor->getBatteryVolts();
  values.push_back(value3);
  msg.values = values;
  motor_status_publisher_.publish(msg);
 }
//---------------------------------------
void stop(){
    motor->stop_motors();
  }
 }
//---------------------------------------
int main(int argc,char **argv){
  if (argc == 1) {
    MD25MotorDriverROSWrapper::motor = std::make_unique<md25_driver>(argv[1]);
  } else MD25MotorDriverROSWrapper::motor = std::make_unique<md25_driver>("/dev/i2c/1");
  if (!motor->setup()) return 1; 

  ros::init(argc,argv,"motor_driver");
  ros::NodeHandle nh;
  ros:AsyncSpinner spinner(4);
  spinner.start();
  MD25MotorDriverROSWrapper motor_wrapper(&nh);
  ROS_INFO("Motor Driver Started");
  ros::waitForShutdown();
  motor_wrapper.stop();
 }
}
