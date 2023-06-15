#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <chrono>
#include <string>
#include <atomic>
#include <tuple>

#include "crsf/CrsfSerial.hpp"
#include "RxTx/DummyRxTx.hpp"
#include "pid/PID.hpp"
#include "arrow.hpp"
#include "holder.hpp"

using std::placeholders::_1;

class ProxyHolderNode : public rclcpp::Node
{
private:
  // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  BaseRxTx rx_;
  BaseRxTx tx_;

  PIDController<Arrow> pid;

  CrsfSerial crsfRx_;
  CrsfSerial crsfFCU_;

  Holder holder_;
  unsigned int hold_channel_;

  using _pose_type = geometry_msgs::msg::PoseWithCovarianceStamped;

  rclcpp::Subscription<_pose_type>::SharedPtr pose_sub_;
  _pose_type last_pose_;

  std::tuple<double, double, double> static calculatePitchRollDistance(const _pose_type &from, const _pose_type &to) {
    tf2::Quaternion fromQuat;
    tf2::fromMsg(from.pose.pose.orientation, fromQuat);

    // tf2::Vector3 yawDir = tf2::quatRotate(fromQuat, tf2::Vector3(0, 0, 1)); // up
    tf2::Vector3 pitchDir = tf2::quatRotate(fromQuat, tf2::Vector3(0, 1, 0)); // side
    tf2::Vector3 rollDir = tf2::quatRotate(fromQuat, tf2::Vector3(1, 0, 0)); // forward

    tf2::Vector3 fromVector;
    tf2::fromMsg(from.pose.pose.position, fromVector);

    tf2::Vector3 toVector;
    tf2::fromMsg(to.pose.pose.position, toVector);

    tf2::Vector3 direction = toVector - fromVector;
    
    // because proj(n, L) = (L * n)
    // proj(n, L) * n + L' = L; where L' - vector we seek
    // So L' = L - proj(n, L) * n
    // And Pitch is n To Yaw-Pitch plane; L - direction; L' - vector we seek
    tf2::Vector3 yawRollProjection = direction - (direction * pitchDir) * pitchDir;
    tf2::Vector3 yawPitchProjection = direction - (direction * rollDir) * rollDir;

    double pitch = yawRollProjection.angle(rollDir); 
    double roll = yawPitchProjection.angle(pitchDir);

    return {pitch, roll, direction.length()};
  }

  void readCrsfs()
  {
    auto bytes = this->rx_.ReadBytes();
    crsfRx_.handleBytes(bytes);

    bytes = this->tx_.ReadBytes();
    crsfFCU_.handleBytes(bytes);

    std_msgs::msg::String message;
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);
  }

  // void onPacketRX()
  // {
  //   int hold_value = this->crsfRx_.getChannel(hold_channel_);

  //   this->holder_.processValue(hold_value, last_pose_);

  //   if this->holder_.
  // }

  void onPacketFCU()
  {
    crsfRx_.queuePacket(
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
        crsfFCU_.getChannels(),
        CRSF_NUM_CHANNELS);
  }

  void onLinkUp()
  {
    RCLCPP_INFO(this->get_logger(), "Link up");
  }

  void onLinkDown()
  {
    RCLCPP_INFO(this->get_logger(), "Link down");
  }

  Arrow getPid()
  {
  }

  void setPid(Arrow a)
  {
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

    this->pid = PIDController<Arrow>(1.0, 1.0, 1.0,
                                     std::bind(&ProxyHolderNode::getPid, this),
                                     std::bind(&ProxyHolderNode::setPid, this, _1));

    this->rx_ = DummyRxTx();
    this->tx_ = DummyRxTx(0, 0);

    this->crsfFCU_.onPacketChannels = std::bind(&ProxyHolderNode::onPacketFCU, this);

    this->crsfRx_.onPacketChannels = std::bind(&ProxyHolderNode::onPacketRX, this);
    this->crsfRx_.onLinkUp = std::bind(&ProxyHolderNode::onLinkUp, this);
    this->crsfRx_.onLinkDown = std::bind(&ProxyHolderNode::onLinkDown, this);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ProxyHolderNode::readCrsfs, this));
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