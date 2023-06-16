#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <chrono>
#include <string>
#include <atomic>
#include <tuple>
#include <iostream>

// #include "PID.hpp"
// #include "DummyRxTx.hpp"
#include "proxy_holder_node.hpp"


int main(int argc, char *argv[])
{
  std::cout << "Hello from main";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProxyHolderNode>());
  rclcpp::shutdown();
  return 0;
}