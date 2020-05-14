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
  std::pair<long, long> readEncoders();
  uint8_t getSoftwareVersion();
  uint8_t getBatteryVolts();
  uint8_t getAccelerationRate();
  uint8_t getMotor1Current();
  uint8_t getMotor2Current();
  uint8_t getMotor1Speed();
  uint8_t getMotor2Speed();
  uint8_t getMode();

  void setMotorsSpeed(uint8_t speed);
  void setMotor1Speed(uint8_t speed);
  void setMotor2Speed(uint8_t speed);
  void setMode(uint8_t mode);
  void setAccelerationRate(uint8_t rate);
  void stopMotor1();
  void stopMotor2();
  void stopMotors();

  void resetEncoders();
  void enableSpeedRegulation();
  void disableSpeedRegulation();
  void enableTimeout();
  void disableTimeout();
  void changeAddress(uint8_t newAddress);
  //-------------
private:
  std::mutex lock;
  int32_t readEncoderArray(uint8_t reg);
  uint8_t readRegisterByte(uint8_t reg);
  void setMotorSpeed(uint8_t motor, uint8_t speed);
  bool sendCommand(uint8_t command,int reg);

  int m_fd = -1;
  int address = 0x58;
  unsigned short m_software_version = 0;
  long m_encoder_1_ticks = 0;
  long m_encoder_2_ticks = 0;
  
  const char * m_i2c_file = nullptr;
    
  static uint8_t const SPD1		            = 0x00;  // speed to first motor
  static uint8_t const SPD2		            = 0x01;  // speed to second motor
  static uint8_t const ENC1	              = 0x02;  // motor encoder 1 (first byte)
  static uint8_t const ENC2	              = 0x06;  // motor encoder 2 (first byte)
  static uint8_t const VOLT		            = 0x0A;  // battery volts
  static uint8_t const I1	                = 0x0B;  // motor 1 current
  static uint8_t const I2	                = 0x0C;  // motor 2 current
  static uint8_t const SW_VER             = 0x0D;  // software version
  static uint8_t const ACC_RATE	          = 0x0E;  // acceleration rate
  static uint8_t const MODE		            = 0x0F;  // mode of operation
  static uint8_t const CMD		            = 0x10;  // command register
  static uint8_t const ENCODER_RESET      = 0x20; // 
  static uint8_t const DISABLE_SPEED_REG  = 0x30; //
  static uint8_t const ENABLE_SPEED_REG   = 0x31; //
  static uint8_t const DISABLE_TIMEOUT    = 0x32; //  
  static uint8_t const ENABLE_TIMEOUT     = 0x33; //
  static uint8_t const STOP_SPEED	      	= 0x80;  // 0 velocity  
};

#endif
