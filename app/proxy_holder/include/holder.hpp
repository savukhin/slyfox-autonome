#pragma once

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <memory>
#include <chrono>
#include <string>
#include <atomic>

enum class HolderState
{
    Hold,
    TransientFromHold,
    Free,
    TransientFromFree,
};

class Holder
{
private:
    std::atomic_bool is_holding_;
    int threshold_;

    std::atomic_bool is_transienting_;
    std::chrono::system_clock::time_point start_transient_time_;
    std::chrono::duration<uint64_t, std::milli> transient_duration_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr holded_pose_;

public:
    Holder(std::chrono::duration<uint64_t, std::milli> transient_duration = std::chrono::milliseconds(100)) : transient_duration_(transient_duration){};

    // return true if switched
    bool processValue(int hold_value, geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
    {
        if ((hold_value > threshold_ && this->is_holding_) || (hold_value < threshold_ && !this->is_holding_))
        {
            this->is_transienting_ = false;
            return false;
        }

        if (!this->is_transienting_)
        {
            this->is_transienting_ = true;
            this->start_transient_time_ = std::chrono::system_clock::now();
        }

        auto delta = std::chrono::system_clock::now() - this->start_transient_time_;
        if (delta > this->transient_duration_)
        {
            this->is_holding_ = true;
            this->is_transienting_ = false;
            this->holded_pose_ = pose;
            return true;
        }
        return false;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr getHoldedPose() const { return this->holded_pose_; }

    bool isHold() const { return this->is_holding_; }

    void setTransientDuration(std::chrono::duration<uint64_t, std::milli> transient_duration) {
        this->transient_duration_ = transient_duration;
    }
};
