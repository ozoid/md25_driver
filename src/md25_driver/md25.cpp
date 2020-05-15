#include <md25_driver/md25.hpp>
md25_driver::md25_driver(const char * _i2c_file) : m_i2c_file(_i2c_file){  }
md25_driver::~md25_driver(){ close(m_fd);}
//---------------------------------------------
bool md25_driver::setup(){
  /*
   * For information, see this link:
   *  http://www.robot-electronics.co.uk/htm/md25i2c.htm
   *
   * (hopefully it sticks around).
   */
  int m_buff[BUF_LEN] = {};
  m_buff[0] = SW_VER;  /* get software version */
  if ((m_fd = open(m_i2c_file, O_RDWR)) < 0) {
    ROS_ERROR("Failed to open i2c file");
    ROS_ERROR(m_i2c_file);
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

  ROS_INFO("MD25 Motors initialized with software verison '%u'", m_buff[0]);
  m_software_version = static_cast<int>(m_buff[0]);
  return resetEncoders();
}
//---------------------------------------------
bool md25_driver::resetEncoders(){
  bool result = sendCommand(ENCODER_RESET,CMD);
  if (!result) {
    ROS_ERROR("could not reset encoders");
    return false;
  }
  return true;
}
//---------------------------------------------
void md25_driver::haltMotors(){
  int m_buff[BUF_LEN] = {0};
  ROS_INFO("HALT received, stopping motors");
  m_buff[0] = SPD1;
  m_buff[1] = 128;  /* this speed stops the motors */

  if (write(m_fd, m_buff, 2) != 2) {
    ROS_ERROR("failed to stop robot, better go catch it!");
    return;
  }

  m_buff[0] = SPD2;
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
   return readEncoderArray(ENC1);
}
int md25_driver::getEncoder2()
{
   return readEncoderArray(ENC2);
}
int md25_driver::getSoftwareVersion()
{
   return readRegisterByte(SW_VER);
}
int md25_driver::getBatteryVolts()
{
   return readRegisterByte(VOLT);
}
int md25_driver::getAccelerationRate()
{
   return readRegisterByte(ACC_RATE);
}
int md25_driver::getMotor1Speed()
{
   return readRegisterByte(SPD1);
}
int md25_driver::getMotor2Speed()
{
   return readRegisterByte(SPD2);
}
int md25_driver::getMotor1Current()
{
   return readRegisterByte(I1);
}
int md25_driver::getMotor2Current()
{
   return readRegisterByte(I2);
}
int md25_driver::getMode()
{
   return readRegisterByte(MODE);
}
void md25_driver::enableSpeedRegulation()
{
   sendCommand(ENABLE_SPEED_REG,CMD);
}
void md25_driver::disableSpeedRegulation()
{
   sendCommand(DISABLE_SPEED_REG,CMD);
}
void md25_driver::enableTimeout()
{
   sendCommand(ENABLE_TIMEOUT,CMD);
}
void md25_driver::disableTimeout()
{
   sendCommand(DISABLE_TIMEOUT,CMD);
}
void md25_driver::setMotorsSpeed(int speed)
{
   setMotor1Speed(speed);
   setMotor2Speed(speed);
}
void md25_driver::setMotorSpeed(int motor, int speed)
{
   sendCommand(speed,motor); //motor = 0|1 left|right
}
void md25_driver::setMotor1Speed(int speed)
{
   setMotorSpeed(SPD1, speed);
}
void md25_driver::setMotor2Speed(int speed)
{
   setMotorSpeed(SPD2, speed);
}
void md25_driver::stopMotor1()
{
   setMotor1Speed(STOP_SPEED);
}
void md25_driver::stopMotor2()
{
   setMotor2Speed(STOP_SPEED);
}
void md25_driver::stopMotors()
{
   stopMotor1();
   stopMotor2();
}
void md25_driver::setMode(int mode)
{
   sendCommand(mode,MODE);
}
void md25_driver::setAccelerationRate(int rate)
{
   sendCommand(rate,ACC_RATE);
}
//----------------------------------------------------
std::pair<int, int> md25_driver::readEncoders(){
  int m_buff[BUF_LEN] = {0};
  /* encoder 1 is stored in registers 2 - 5 */
  m_buff[0] = ENC1;
  lock.lock();
  if (write(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("Could not write to i2c");
    lock.unlock();
    return std::make_pair(0, 0);
  } else if (read(m_fd, m_buff, 8) != 8) {
    ROS_ERROR("Could not read encoder values");
    lock.unlock();
    return std::make_pair(0, 0);;
  }
  lock.unlock();
  /* assemble the encoder readings */
  m_encoder_1_ticks =
    (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
  m_encoder_2_ticks =
    (m_buff[4] << 24) + (m_buff[5] << 16) + (m_buff[6] << 8) + m_buff[7];

  return std::make_pair(m_encoder_1_ticks, m_encoder_2_ticks);
}
//-------------------------------------------------------
int md25_driver::readEncoderArray(int reg){
  int m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
  lock.lock();
  if (write(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("Could not write to i2c");
    lock.unlock();
    return false;
  } else if (read(m_fd, m_buff, 4) != 4) {
    ROS_ERROR("Could not read register value");
    lock.unlock();
    return false;
  }
  lock.unlock();
  return (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
}

int md25_driver::readRegisterByte(int reg){
  int m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
  lock.lock();
  if (write(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("Could not write to i2c");
    lock.unlock();
    return false;
  } else if (read(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("Could not read register value");
    lock.unlock();
    return false;
  }
  lock.unlock();
  return m_buff[0];
}

bool md25_driver::sendCommand(int command,int reg){
  int m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
  m_buff[1] = command;
  lock.lock();
  if (write(m_fd, m_buff, 2) != 2) {
    ROS_ERROR("failed to send command!");
    lock.unlock();
    return false;  
  }
  lock.unlock();
  return true;
}
