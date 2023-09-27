#include <cstdio>
#include "trajectory_server/trajectory_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryServer>());
  rclcpp::shutdown();
  
  return 0;
}
