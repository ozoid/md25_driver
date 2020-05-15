// Copyright 2017 Hunter L. Allen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __MD25_HPP__
#define __MD25_HPP__

#include <functional>
#include <iostream>
#include <cstring>
#include <memory>
#include <tuple>
#include <mutex>
#include <linux/i2c-dev.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <unistd.h>
#include <fcntl.h>

#include <ros/ros.h>

#define BUF_LEN 10

class md25_driver
{
public:
  md25_driver(const char * _i2c_file = "/dev/i2c-1");
  ~md25_driver();
  //-------------------- 
  bool setup();
  int getEncoder1();
  int getEncoder2();
  std::pair<int, int> readEncoders();
  int getSoftwareVersion();
  int getBatteryVolts();
  int getAccelerationRate();
  int getMotor1Current();
  int getMotor2Current();
  int getMotor1Speed();
  int getMotor2Speed();
  int getMode();

  void setMotorsSpeed(int speed);
  void setMotor1Speed(int speed);
  void setMotor2Speed(int speed);
  void setMode(int mode);
  void setAccelerationRate(int rate);
  void stopMotor1();
  void stopMotor2();
  void stopMotors();
  void haltMotors();

  bool resetEncoders();
  void enableSpeedRegulation();
  void disableSpeedRegulation();
  void enableTimeout();
  void disableTimeout();
  void changeAddress(int newAddress);
  //-------------
private:
  std::mutex lock;
  int readEncoderArray(int reg);
  int readRegisterByte(int reg);
  void setMotorSpeed(int motor, int speed);
  bool sendCommand(int command,int reg);

  int m_fd = -1;
  int address = 0x58;
  int m_software_version = 0;
  int m_encoder_1_ticks = 0;
  int m_encoder_2_ticks = 0;
  
  const char * m_i2c_file = nullptr;
    
  static int const SPD1		            = 0x00;  // speed to first motor
  static int const SPD2		            = 0x01;  // speed to second motor
  static int const ENC1	              = 0x02;  // motor encoder 1 (first byte)
  static int const ENC2	              = 0x06;  // motor encoder 2 (first byte)
  static int const VOLT		            = 0x0A;  // battery volts
  static int const I1	                = 0x0B;  // motor 1 current
  static int const I2	                = 0x0C;  // motor 2 current
  static int const SW_VER             = 0x0D;  // software version
  static int const ACC_RATE	          = 0x0E;  // acceleration rate
  static int const MODE		            = 0x0F;  // mode of operation
  static int const CMD		            = 0x10;  // command register
  static int const ENCODER_RESET      = 0x20; // 
  static int const DISABLE_SPEED_REG  = 0x30; //
  static int const ENABLE_SPEED_REG   = 0x31; //
  static int const DISABLE_TIMEOUT    = 0x32; //  
  static int const ENABLE_TIMEOUT     = 0x33; //
  static int const STOP_SPEED	      	= 0x80;  // 0 velocity  
};

#endif
