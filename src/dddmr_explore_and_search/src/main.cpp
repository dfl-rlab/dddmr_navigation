#include <rclcpp/rclcpp.hpp>
#include "dddmr_explore_and_search/dddmr_explore_and_search.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DddmrExploreAndSearch>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
