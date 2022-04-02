// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef RPI_STEREO__RPI_STEREO_HPP_
#define RPI_STEREO__RPI_STEREO_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rpi_stereo/rpi_stereo_driver.hpp"
#include <image_geometry/stereo_camera_model.h>
#include <string>

namespace rpi_stereo
{

class RpiStereo : public rclcpp::Node
{
public:
  explicit RpiStereo(const rclcpp::NodeOptions & options);

protected:
  void onInit();
  void doWork();
  void reconfigureCallback();
	bool read_camera_info(std::string file_, sensor_msgs::msg::CameraInfo & info);

private:
  image_transport::CameraPublisher l_pub_;
  image_transport::CameraPublisher r_pub_;

	std::string l_frame_id_;
	std::string r_frame_id_;

  rclcpp::TimerBase::SharedPtr timer_;

  image_geometry::StereoCameraModel cam_model_;
  std::shared_ptr<rpi_stereo_cam::RpiStereoCamDriver> stereo_cam_;

  cv::Mat l_frame;
  cv::Mat r_frame;

	std::string l_camera_info_file_;
	std::string r_camera_info_file_;

	bool rectify_;

	double publish_rate_;

  sensor_msgs::msg::CameraInfo l_cam_info_;
  sensor_msgs::msg::CameraInfo r_cam_info_;
};

}  // namespace rpi_stereo

#endif  // RPI_STEREO__RPI_STEREO_HPP_
