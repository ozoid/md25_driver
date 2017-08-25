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
  m_buff[0] = 13;  /* get software version */
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
  m_buff[0] = 0x10;  /* command register */
  m_buff[1] = 0x20;  /* command to zero out encoders */

  if (write(m_fd, m_buff, 2) != 2) {
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
  /* enoder 1 is stored in registers 2 - 5 */
  m_buff[0] = 0x2;

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
  m_buff[0] = 0;
  m_buff[1] = 128;  /* this speed stops the motors */

  if (write(m_fd, m_buff, 2) != 2) {
    ROS_ERROR("failed to stop robot, better go catch it!");
    return;
  }

  m_buff[0] = 1;
  m_buff[1] = 128;

  if (write(m_fd, m_buff, 2) != 2) {
    ROS_ERROR("failed to stop robot, better go catch it!");
    return;  
  }

  ROS_INFO("motors stopped");
}
