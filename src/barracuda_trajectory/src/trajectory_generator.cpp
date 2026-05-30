#include "barracuda_trajectory/trajectory_generator_node.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGenerator>());
  rclcpp::shutdown();
  return 0;
}
