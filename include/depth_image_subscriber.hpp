#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "unitree/robot/channel/channel_subscriber.hpp"

#include "msg/DepthImage_.hpp"

namespace unitree::common
{

// Subscribes to MuJoCo's depth image on `rt/depthimage`
// (unitree_go::msg::dds_::DepthImage_ : width, height, normalized_value [0,1]).
// The callback runs on the DDS reader thread; the frame is copied out under a
// mutex. The publisher (Task 7) outputs exactly 53x30 already-normalized
// frames, so no denormalize/resize happens here.
class DepthImageSubscriber
{
public:
    DepthImageSubscriber(int domain_id, const std::string &interface,
                         const std::string &topic = "rt/depthimage")
    {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, interface);
        sub_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::DepthImage_>(topic));
        sub_->InitChannel(
            std::bind(&DepthImageSubscriber::onDepthImage, this, std::placeholders::_1), 1);
    }

    bool latest_frame(std::vector<float> &out, uint16_t &width, uint16_t &height)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!have_frame_) return false;
        out = frame_;
        width = width_;
        height = height_;
        return true;
    }

private:
    void onDepthImage(const void *data)
    {
        const auto *msg = static_cast<const unitree_go::msg::dds_::DepthImage_ *>(data);
        std::lock_guard<std::mutex> lock(mtx_);
        width_ = msg->width();
        height_ = msg->height();
        frame_.assign(msg->normalized_value().begin(), msg->normalized_value().end());
        have_frame_ = true;
    }

    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::DepthImage_> sub_;
    std::mutex mtx_;
    std::vector<float> frame_;
    uint16_t width_ = 0;
    uint16_t height_ = 0;
    bool have_frame_ = false;
};

} // namespace unitree::common
