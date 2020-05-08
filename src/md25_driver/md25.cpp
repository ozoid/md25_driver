#include <md25_driver/md25.hpp>

md25_driver::md25_driver(const char * _i2c_file)
: m_i2c_file(_i2c_file)
{ clear_buffer(); }

md25_driver::~md25_driver()
{ /* destructor */ }

bool md25_driver::setup()
{
  /*
   * For information, see this link:
   *  http://www.robot-electronics.co.uk/htm/md25i2c.htm
   *
   * (hopefully it sticks around).
   */
  m_buff[0] = softwareVerReg;  /* get software version */
  if ((m_fd = open(m_i2c_file, O_RDWR)) < 0) {
    ROS_ERROR("Failed to open i2c file");
    return false;
  } else if (ioctl(m_fd, I2C_SLAVE, address) < 0) {
    ROS_ERROR("Could not speak to I2C bus!");
    return false;
  } else if (write(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("Could not write to i2c slave");
    return false;
  } else if (read(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("Could not read from i2c slave");
    return false;
  }

  /* at this point, we have obtained the software version! */
  ROS_INFO("MD25 Motors initialized with software verison '%u'", m_buff[0]);
  m_software_version = static_cast<unsigned short>(m_buff[0]);

  /* set encoders to zero, return */
  return reset_encoders();
}

bool md25_driver::reset_encoders()
{
  bool result = sendCommand(resetEncoders,cmdReg);
  if (!result) {
    ROS_ERROR("could not reset encoders");
    return false;
  }
  return true;
}

void md25_driver::clear_buffer()
{
  for (int x = BUF_LEN; x --> 0; m_buff[x] = 0x0);
}

bool md25_driver::read_encoders()
{
  /* encoder 1 is stored in registers 2 - 5 */
  m_buff[0] = encoderOneReg;

  if (write(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("Could not write to i2c");
    return false;
  } else if (read(m_fd, m_buff, 8) != 8) {
    ROS_ERROR("Could not read encoder values");
    return false;
  }

  /* assemble the encoder readings */
  m_encoder_1_ticks =
    (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
  m_encoder_2_ticks =
    (m_buff[4] << 24) + (m_buff[5] << 16) + (m_buff[6] << 8) + m_buff[7];

  return true;
}

std::pair<long, long> md25_driver::get_encoders()
{
  if (!read_encoders()) {
    /* error already reported, just return {-1, -1}  */
    return std::make_pair(-1, -1);
  }
  return std::make_pair(m_encoder_1_ticks, m_encoder_2_ticks);
}

void md25_driver::stop_motors()
{
  ROS_INFO("HALT received, stopping motors");
  m_buff[0] = speed1Reg;
  m_buff[1] = 128;  /* this speed stops the motors */

  if (write(m_fd, m_buff, 2) != 2) {
    ROS_ERROR("failed to stop robot, better go catch it!");
    return;
  }

  m_buff[0] = speed2Reg;
  m_buff[1] = 128;

  if (write(m_fd, m_buff, 2) != 2) {
    ROS_ERROR("failed to stop robot, better go catch it!");
    return;  
  }

  ROS_INFO("motors stopped");
}
//----------------------------------------------
int md25_driver::getEncoder1()
{
   return readEncoderArray(encoderOneReg);
}

int md25_driver::getEncoder2()
{
   return readEncoderArray(encoderTwoReg);
}
long md25_driver::getSoftwareVersion()
{
   return readRegisterByte(softwareVerReg);
}

float md25_driver::getBatteryVolts()
{
   return readRegisterByte(voltReg)/10.0;
}

byte md25_driver::getAccelerationRate()
{
   return readRegisterByte(accRateReg);
}

byte md25_driver::getMotor1Speed()
{
   return readRegisterByte(speed1Reg);
}

byte md25_driver::getMotor2Speed()
{
   return readRegisterByte(speed2Reg);
}

byte md25_driver::getMotor1Current()
{
   return readRegisterByte(current1Reg);
}

byte md25_driver::getMotor2Current()
{
   return readRegisterByte(current2Reg);
}

byte md25_driver::getMode()
{
   return readRegisterByte(modeReg);
}

void md25_driver::resetEncoders()
{
   sendCommand(resetEncoders,cmdReg);
}

void md25_driver::enableSpeedRegulation()
{
   sendCommand(enableSpeedReg,cmdReg);
}

void md25_driver::disableSpeedRegulation()
{
   sendCommand(disableSpeedReg,cmdReg);
}

void md25_driver::enableTimeout()
{
   sendCommand(enableTimeout,cmdReg);
}

void md25_driver::disableTimeout()
{
   sendCommand(disableTimeout,cmdReg);
}

void md25_driver::setMotorsSpeed(byte speed)
{
   setMotor1Speed(speed);
   setMotor2Speed(speed);
}

void md25_driver::setMotor1Speed(byte speed)
{
   setMotorSpeed(speed1Reg, speed);
}

void md25_driver::setMotor2Speed(byte speed)
{
   setMotorSpeed(speed2Reg, speed);
}

void md25_driver::stopMotor1()
{
   setMotor1Speed(stopSpeed);
}

void md25_driver::stopMotor2()
{
   setMotor2Speed(stopSpeed);
}

void md25_driver::stopMotors()
{
   stopMotor1();
   stopMotor2();
}
void md_25_driver::setMode(byte mode)
{
   sendCommand(mode,modeReg);
}
void md25_driver::setAccelerationRate(byte rate)
{
   sendCommand(rate,accRateReg);
}
void md25_driver::setMotorSpeed(byte motor, byte speed)
{
   sendCommand(speed,motor);
}


int md25_driver::readEncoderArray(byte reg){
m_buff[0] = reg;
  if (write(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("Could not write to i2c");
    return false;
  } else if (read(m_fd, m_buff, 4) != 4) {
    ROS_ERROR("Could not read register value");
    return false;
  }
  int result = (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
  return result;
}

byte md25_driver::readRegisterByte(byte reg){
  m_buff[0] = reg;
  if (write(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("Could not write to i2c");
    return false;
  } else if (read(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("Could not read register value");
    return false;
  }
  return m_buff[0];
}

bool md25_driver::sendCommand(byte command,int reg){
  m_buff[0] = reg;
  m_buff[1] = command;
  if (write(m_fd, m_buff, 2) != 2) {
    ROS_ERROR("failed to send command!");
    return false;  
  }
  return true;
}
