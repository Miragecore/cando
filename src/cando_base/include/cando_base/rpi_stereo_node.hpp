#ifndef RPI_STEREO_NODE_HPP
#define RPI_STEREO_NODE_HPP

#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cando_base/rpi_stereo_driver.hpp"
#include "cando_base/rpi_stereo_cam_context.hpp"

namespace rpi_stereo_cam 
{

  class RpiStereoCamNode : public rclcpp::Node
  {
    RpiStereoCameraContext cxt_;

    std::thread thread_;
    std::atomic<bool> canceled_;

    //std::shared_ptr<cv::VideoCapture> capture_;
    std::shared_ptr<RpiStereoCamDriver> stereo_cam_;
    //sensor_msgs::msg::CameraInfo camera_info_msg_;
    sensor_msgs::msg::CameraInfo rcamera_info_msg_;
    sensor_msgs::msg::CameraInfo lcamera_info_msg_;

    int publish_fps_;
    //rclcpp::Time next_stamp_;
    rclcpp::Time rcam_next_stamp_;
    rclcpp::Time lcam_next_stamp_;

    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    //rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rcam_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lcam_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rcamera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr lcamera_info_pub_;

  public:

    explicit RpiStereoCamNode(const rclcpp::NodeOptions &options);

    ~RpiStereoCamNode() override;

  private:

    void validate_parameters();

    void loop();
  };

} // namespace rpi_stereo_cam

#endif // RPI_STEREO_NODE_HPP
