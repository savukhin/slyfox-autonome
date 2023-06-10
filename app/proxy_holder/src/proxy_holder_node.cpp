#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <memory>
#include <chrono>
#include <string>
#include <atomic>

#include "crsf/CrsfSerial.hpp"
#include "RxTx/DummyRxTx.hpp"

using std::placeholders::_1;

class ProxyHolderNode : public rclcpp::Node
{
private:
  // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  unsigned int holdChannel_;
  int holdThreshold_;

  BaseRxTx rx_;
  BaseRxTx tx_;

  CrsfSerial crsfRx_;
  CrsfSerial crsfFCU_;

  std::atomic_bool hold_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void readRx()
  {
    auto bytes = this->rx_.ReadBytes();
    crsfRx_.handleBytes(bytes);

    std_msgs::msg::String message;
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);
  }

  void onPacket()
  {
    int holdValue = this->crsfRx_.getChannel(holdChannel_);
    this->hold_ = holdValue > holdThreshold_;
  }

  void onLinkUp()
  {
    RCLCPP_INFO(this->get_logger(), "Link up");
  }

  void onLinkDown()
  {
    RCLCPP_INFO(this->get_logger(), "Link down");
  }

  
public:
  explicit ProxyHolderNode()
      : Node("proxy_holder_node"), count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing proxy holder node");

    // /rtabmap/localization_pose
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/rtabmap/localization_pose", 10, std::bind(&ProxyHolderNode::poseGet, this, _1));

    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic", 10, std::bind(&ProxyHolderNode::topic_callback, this, _1));

    this->rx_ = DummyRxTx();
    this->tx_ = DummyRxTx();

    this->crsfRx_.onPacketChannels = std::bind(&ProxyHolderNode::onPacket, this);
    this->crsfRx_.onLinkUp = std::bind(&ProxyHolderNode::onLinkUp, this);
    this->crsfRx_.onLinkDown = std::bind(&ProxyHolderNode::onLinkDown, this);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ProxyHolderNode::readRx, this));
  }

  void poseGet(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Link down");
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }


  virtual ~ProxyHolderNode() = default;
};

// } // namespace proxy_holder

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProxyHolderNode>());
  rclcpp::shutdown();
  return 0;
}