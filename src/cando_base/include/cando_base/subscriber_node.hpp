#ifndef SIMPLE_SUBSCRIBER_HPP
#define SIMPLE_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rpi_stereo_cam
{

  // Node that subscribes to a topic, used for testing composition and IPC
  class ImageSubscriberNode : public rclcpp::Node
  {
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2_;

  public:

    explicit ImageSubscriberNode(const rclcpp::NodeOptions &options);
  };

} // namespace rpi_stereo_cam

#endif //SIMPLE_SUBSCRIBER_HPP
