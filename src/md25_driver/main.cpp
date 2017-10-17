#include <md25_driver/md25.hpp>

int main(int argc, char ** argv)
{
  std::unique_ptr<md25_driver> driver;
  if (argc == 1) {
    driver = std::make_unique<md25_driver>(argv[1]);
  } else driver = std::make_unique<md25_driver>();

  if (!driver->setup()) return 1;
  for (long lval, rval; ros::ok();) {
    std::tie(lval, rval) = driver->get_encoders();
    std::cout<<"left: "<<lval<<", right: "<<rval<<std::endl;
  }
}
