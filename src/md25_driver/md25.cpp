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
  uint8_t m_buff[BUF_LEN] = {};
  lastReadEncoders = false;
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

  ROS_INFO("MD25 Motors initialized with software version '%u'", m_buff[0]);
  m_software_version = static_cast<int>(m_buff[0]);
  return true;
}
//---------------------------------------------
int md25_driver::getSoftwareVersion()
{
   return readByte(SW_VER);
}
int md25_driver::getBatteryVolts()
{
   return readByte(VOLT);
}
int md25_driver::getAccelerationRate()
{
   return readByte(ACC_RATE);
}
int md25_driver::getMode()
{
   return readByte(MODE);
}
std::pair<int, int> md25_driver::getMotorsSpeed()
{
   return readTwoBytes(SPD1);
}
std::pair<int, int> md25_driver::getMotorsCurrent()
{
   return readTwoBytes(I1);
}
bool md25_driver::enableSpeedRegulation()
{
  return sendCommand(ENABLE_SPEED_REG,CMD);
}
bool md25_driver::disableSpeedRegulation()
{
  return sendCommand(DISABLE_SPEED_REG,CMD);
}
bool md25_driver::enableTimeout()
{
  return sendCommand(ENABLE_TIMEOUT,CMD);
}
bool md25_driver::disableTimeout()
{
  return sendCommand(DISABLE_TIMEOUT,CMD);
}
bool md25_driver::setMotorsSpeed(int speed1,int speed2)
{
  return writeSpeed(speed1,speed2);
}

bool md25_driver::stopMotors()
{
  return writeSpeed(STOP_SPEED,STOP_SPEED);
}
bool md25_driver::haltMotors(){
  uint8_t m_buff[BUF_LEN] = {0};
  lastReadEncoders = false;
  //ROS_INFO("HALT received, stopping motors");
  m_buff[0] = SPD1;
  m_buff[1] = STOP_SPEED;  /* this speed stops the motors */
  //lock.lock();
  if (write(m_fd, m_buff, 2) != 2) {
    ROS_ERROR("HALT: failed to stop robot, better go catch it!");
    //lock.unlock();
    return false;
  }
  m_buff[0] = SPD2;
  m_buff[1] = STOP_SPEED;
  if (write(m_fd, m_buff, 2) != 2) {
    ROS_ERROR("HALT: failed to stop robot, better go catch it!");
    //lock.unlock();
    return false;  
  }
  //lock.unlock();
  //ROS_INFO("motors stopped");
  return true;
}
bool md25_driver::setMode(int mode)
{
  return sendCommand(mode,MODE);
}
bool md25_driver::setAccelerationRate(int rate)
{
  return sendCommand(rate,ACC_RATE);
}
//---------------------------------------------
bool md25_driver::resetEncoders(){
  bool result = sendCommand(ENCODER_RESET,CMD);
  if (!result) {
    ROS_ERROR("SND: Could not reset encoders");
    return false;
  }
  m_encoder_1_ticks = 0;
  m_encoder_2_ticks = 0;
  return true;
}
//----------------------------------------------------
std::pair<int, int> md25_driver::readEncoders2(){
  //lock.lock();
  m_encoder_1_ticks = readEncoder(ENC1);
  m_encoder_2_ticks = readEncoder(ENC2);
  //lock.unlock();
  return std::make_pair(m_encoder_1_ticks,m_encoder_2_ticks);
}
//-------------------------------------------------------
std::pair<int, int> md25_driver::readEncoders(){
  uint8_t m_buff[BUF_LEN] = {0};
  if(!lastReadEncoders){
    m_buff[0] = ENC1;
    if (write(m_fd, m_buff, 1) != 1) {
      ROS_ERROR("RE: Could not write to i2c");
      return std::make_pair(m_encoder_1_ticks,m_encoder_2_ticks);
    }
    lastReadEncoders = true;
    ros::Duration(0.005).sleep();
  }
  if (read(m_fd, m_buff, 8) != 8) {
    ROS_ERROR("RE: Could not read encoder values");
    return std::make_pair(m_encoder_1_ticks,m_encoder_2_ticks);
  }
  int LT = (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
  int RT = (m_buff[4] << 24) + (m_buff[5] << 16) + (m_buff[6] << 8) + m_buff[7];

  if((RT - m_encoder_2_ticks) > 1000 || (RT - m_encoder_2_ticks) < -1000){
     ROS_ERROR("RE: Right encoder Jump > 1000 - %d",(RT - m_encoder_2_ticks));
  }else{
    m_encoder_2_ticks = RT;
  }
  if((LT - m_encoder_1_ticks) > 1000 || (LT - m_encoder_1_ticks) < -1000){
    ROS_ERROR("RE: Left encoder Jump > 1000 - %d",(LT - m_encoder_1_ticks));
  }else{
    m_encoder_1_ticks = LT;
  }
  
  return std::make_pair(m_encoder_1_ticks,m_encoder_2_ticks);
}
//----------------------------------------------------------
int md25_driver::readEncoder(int LR){
  uint8_t m_buff[BUF_LEN] = {0};
  int ticks = 0;
  if(LR == ENC1){ticks = m_encoder_1_ticks; }else{ticks = m_encoder_2_ticks;}
  m_buff[0] = LR;
  if (write(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("RE: Could not write to i2c");
    return ticks;
  } else if (read(m_fd, m_buff, 4) != 4) {
    ROS_ERROR("RE: Could not read encoder values %d ",LR);
    return ticks;
  }
  ticks = (m_buff[0] << 24) + (m_buff[1] << 16) + (m_buff[2] << 8) + m_buff[3];
  return ticks;
}
//-------------------------------------------------------
bool md25_driver::writeSpeed(int left,int right){
  lastReadEncoders = false;
  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = SPD1;
  m_buff[1] = left;
  m_buff[2] = right;
  //lock.lock();
  if (write(m_fd, m_buff, 3) != 3) {
    ROS_ERROR("WS: failed to send  speed command!");
  //  lock.unlock();
    return false;  
  }
  //lock.unlock();
  return true;
}
//-------------------------------------------------------
//-------------------------------------------------
 std::pair<int, int> md25_driver::readTwoBytes(int reg){
  lastReadEncoders = false;
  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
 // lock.lock();
  if (write(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("R2: Could not write to i2c");
 //   lock.unlock();
    return std::make_pair(0 ,0);
  } else if (read(m_fd, m_buff, 2) != 2) {
    ROS_ERROR("R2: Could not read register value");
  //  lock.unlock();
    return std::make_pair(0 ,0);
  }
 // lock.unlock();
  return std::make_pair(m_buff[0] ,m_buff[1]); 
}
//-------------------------------------------------

int md25_driver::readByte(int reg){
  lastReadEncoders = false;
  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
 // lock.lock();
  if (write(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("R1: Could not write to i2c");
  //  lock.unlock();
    return 0;
  } else if (read(m_fd, m_buff, 1) != 1) {
    ROS_ERROR("R1: Could not read register value");
  //  lock.unlock();
    return 0;
  }
 // lock.unlock();
  return m_buff[0];
}

bool md25_driver::sendCommand(int value,int reg){
  lastReadEncoders = false;
  uint8_t m_buff[BUF_LEN] = {0};
  m_buff[0] = reg;
  m_buff[1] = value;
//  lock.lock();
  if (write(m_fd, m_buff, 2) != 2) {
    ROS_ERROR("failed to send command!");
 //   lock.unlock();
    return false;  
  }
//  lock.unlock();
  return true;
}
