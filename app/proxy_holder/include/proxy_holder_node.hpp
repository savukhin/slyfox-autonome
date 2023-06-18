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
#include <map>

#include "CrsfSerial.hpp"
#include "DummyRxTx.hpp"
#include "UsbRxTx.hpp"
#include "PID.hpp"
#include "holder.hpp"

using std::placeholders::_1;

// enum class SerialType {
//   Dummy = "dummy", 
//   USB = "usb",
// };

class ProxyHolderNode : public rclcpp::Node
{
private:
  // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  BaseRxTx *rx_;
  BaseRxTx *fcu_;

  PIDController<double> pidThrottle;
  PIDController<double> pidPitch;
  PIDController<double> pidRoll;

  CrsfSerial crsfRx_;
  CrsfSerial crsfFCU_;

  Holder holder_;

  unsigned int hold_channel_;
  unsigned int yaw_channel_;
  unsigned int pitch_channel_;
  unsigned int roll_channel_;
  unsigned int throttle_channel_;

  using _pose_type = geometry_msgs::msg::PoseWithCovarianceStamped;
  using _pose_ptr_type = geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr;

  rclcpp::Subscription<_pose_type>::SharedPtr pose_sub_;
  _pose_ptr_type last_pose_;

  OnSetParametersCallbackHandle::SharedPtr param_cb_hndl_;
  std::map<std::string, std::function<void(double)>> param_funcs_;
  
  // geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_pose_;

  std::tuple<double, double, double> static calculatePitchRollDistance(_pose_ptr_type from, _pose_ptr_type to) {
    auto fromPose = from.get()->pose.pose;
    auto toPose = to.get()->pose.pose;
    tf2::Quaternion fromQuat;
    tf2::fromMsg(fromPose.orientation, fromQuat);

    // tf2::Vector3 yawDir = tf2::quatRotate(fromQuat, tf2::Vector3(0, 0, 1)); // up
    tf2::Vector3 pitchDir = tf2::quatRotate(fromQuat, tf2::Vector3(0, 1, 0)); // side
    tf2::Vector3 rollDir = tf2::quatRotate(fromQuat, tf2::Vector3(1, 0, 0)); // forward

    tf2::Vector3 fromVector;
    tf2::fromMsg(fromPose.position, fromVector);

    tf2::Vector3 toVector;
    tf2::fromMsg(toPose.position, toVector);

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
    if (this->rx_) {
      auto bytes = this->rx_->ReadBytes();
      crsfRx_.handleBytes(bytes);
    }

    if (this->fcu_) {
      auto bytes = this->fcu_->ReadBytes();
      crsfFCU_.handleBytes(bytes);
    }
  }

  void static sendMessage(CrsfSerial &crsf, BaseRxTx *tx, int* data) {
    auto package = crsf.queuePacket(
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
        data,
        CRSF_NUM_CHANNELS);

    uint8_t *bytes = package.first;
    size_t len = package.second;

    if (tx)
      tx->WriteBytes(bytes, len);

    free(bytes);
  }

  void onPacketRX()
  {
    int hold_value = this->crsfRx_.getChannel(hold_channel_);

    this->holder_.processValue(hold_value, last_pose_);

    if (!this->holder_.isHold()) {
      this->sendMessage(crsfFCU_, fcu_, crsfRx_.getChannels());    
      return;
    }

    double pitch, roll, dist;
    std::tie(pitch, roll, dist) = this->calculatePitchRollDistance(last_pose_, this->holder_.getHoldedPose());

    
    double pitch_speed = this->pidPitch.tick(pitch, 0);
    double roll_speed = roll = this->pidRoll.tick(roll, 0);
    double throttle = this->pidThrottle.tick(dist, 0);

    int* channels = new int[CRSF_NUM_CHANNELS];
    memcpy(channels, crsfRx_.getChannels(), CRSF_NUM_CHANNELS);

    channels[this->yaw_channel_] = 0;
    channels[this->throttle_channel_] = throttle;
    channels[this->pitch_channel_] = pitch_speed;
    channels[this->roll_channel_] = roll_speed;

    this->sendMessage(crsfFCU_, fcu_, channels);  
  }

  void onPacketFCU()
  {
    this->sendMessage(crsfRx_, rx_, crsfFCU_.getChannels());
  }

  void onLinkUp()
  {
    RCLCPP_INFO(this->get_logger(), "Link up");
  }

  void onLinkDown()
  {
    RCLCPP_INFO(this->get_logger(), "Link down");
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
        const std::string name = parameter.get_name();
      
        auto iter = this->param_funcs_.find(name);
        if (iter == this->param_funcs_.end()) {
          RCLCPP_ERROR(this->get_logger(), ("No such parameter " + name).c_str());
          continue;
        }

        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), ("Parameter " + name + " must be double").c_str());
          continue;
      }


        iter->second(parameter.as_double());
    }

    return result;
  }

public:
  explicit ProxyHolderNode()
      : Node("proxy_holder_node"), count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing proxy holder node");

    double pid_p_throttle = this->declare_parameter<double>("pid_p_throttle", 1.0);
    double pid_i_throttle = this->declare_parameter<double>("pid_i_throttle", 1.0);
    double pid_d_throttle = this->declare_parameter<double>("pid_d_throttle", 1.0);
    double pid_p_pitch = this->declare_parameter<double>("pid_p_pitch", 1.0);
    double pid_i_pitch = this->declare_parameter<double>("pid_i_pitch", 1.0);
    double pid_d_pitch = this->declare_parameter<double>("pid_d_pitch", 1.0);
    double pid_p_roll = this->declare_parameter<double>("pid_p_roll", 1.0);
    double pid_i_roll = this->declare_parameter<double>("pid_i_roll", 1.0);
    double pid_d_roll = this->declare_parameter<double>("pid_d_roll", 1.0);

    this->param_funcs_["pid_p_throttle"] = [&](double val) { this->pidThrottle.setP(val); };
    this->param_funcs_["pid_i_throttle"] = [&](double val) { this->pidThrottle.setI(val); };
    this->param_funcs_["pid_d_throttle"] = [&](double val) { this->pidThrottle.setD(val); };
    this->param_funcs_["pid_p_pitch"] = [&](double val) { this->pidPitch.setP(val); };
    this->param_funcs_["pid_i_pitch"] = [&](double val) { this->pidPitch.setI(val); };
    this->param_funcs_["pid_d_pitch"] = [&](double val) { this->pidPitch.setD(val); };
    this->param_funcs_["pid_p_roll"] = [&](double val) { this->pidRoll.setP(val); };
    this->param_funcs_["pid_i_roll"] = [&](double val) { this->pidRoll.setI(val); };
    this->param_funcs_["pid_d_roll"] = [&](double val) { this->pidRoll.setD(val); };

    // dummy | usb
    std::string rx_serial_type = this->declare_parameter("rx_serial_type", "dummy");
    std::string rx_serial = this->declare_parameter("rx_serial", "/dev/USB0");
    uint rx_serial_baud = this->declare_parameter<int>("rx_serial_baud", 115200);
    std::string fcu_serial_type = this->declare_parameter("fcu_serial_type", "dummy");
    std::string fcu_serial = this->declare_parameter("fcu_serial", "/dev/USB1");
    uint fcu_serial_baud = this->declare_parameter<int>("fcu_serial_baud", 115200);

    this->param_cb_hndl_ = add_on_set_parameters_callback(
      std::bind(&ProxyHolderNode::onParameterChange, this, std::placeholders::_1));

    if (rx_serial_type != "dummy" && rx_serial_type != "usb") {
      const char* msg = "rx_serial_type type must be \"dummy\" or \"usb\"";
      RCLCPP_ERROR(this->get_logger(), msg);
      throw new std::runtime_error(msg);
      return;
    }

     if (fcu_serial_type != "dummy" && fcu_serial_type != "usb") {
      const char* msg = "fcu_serial_type type must be \"dummy\" or \"usb\"";
      RCLCPP_ERROR(this->get_logger(), msg);
      throw new std::runtime_error(msg);
      return;
    }

    // /rtabmap/localization_pose
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/rtabmap/localization_pose", 10, 
      std::bind(&ProxyHolderNode::poseGet, this, _1)
    );

    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic", 10, std::bind(&ProxyHolderNode::topic_callback, this, _1));

    this->pidThrottle = PIDController<double>(pid_p_throttle, pid_i_throttle, pid_d_throttle);
    this->pidPitch = PIDController<double>(pid_p_pitch, pid_i_pitch, pid_d_pitch);
    this->pidRoll = PIDController<double>(pid_p_roll, pid_i_roll, pid_d_roll);

    if (rx_serial_type == "dummy") {
      this->rx_ = new DummyRxTx();
    } else {
      this->rx_ = new UsbRxTx(rx_serial.c_str(), rx_serial_baud);
    }
    if (fcu_serial_type == "dummy") {
      this->fcu_ = new DummyRxTx(0, 0);
    } else {
      this->fcu_ = new UsbRxTx(fcu_serial.c_str(), fcu_serial_baud);
    }

    this->crsfFCU_.onPacketChannels = std::bind(&ProxyHolderNode::onPacketFCU, this);

    this->crsfRx_.onPacketChannels = std::bind(&ProxyHolderNode::onPacketRX, this);
    this->crsfRx_.onLinkUp = std::bind(&ProxyHolderNode::onLinkUp, this);
    this->crsfRx_.onLinkDown = std::bind(&ProxyHolderNode::onLinkDown, this);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ProxyHolderNode::readCrsfs, this));
  }

  void poseGet(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    this->last_pose_ = msg;
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  virtual ~ProxyHolderNode() = default;
};

// } // namespace proxy_holder
