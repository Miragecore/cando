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

//#include <camera_info_manager/camera_info_manager.hpp>
#include "camera_calibration_parsers/parse.hpp"

#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rpi_stereo/rpi_stereo.hpp"

namespace rpi_stereo
{

using namespace std::chrono_literals;

RpiStereo::RpiStereo(const rclcpp::NodeOptions & options)
: Node("RpiStereo", options)
{
  rectify_ = this->declare_parameter("rectify", false);
  publish_rate_ = this->declare_parameter("publish_rate", static_cast<double>(10));

  l_frame_id_ = this->declare_parameter("l_frame_id", std::string("l_camera"));
  r_frame_id_ = this->declare_parameter("r_frame_id", std::string("r_camera"));
 
  l_camera_info_file_ = this->declare_parameter("l_camera_info_file", std::string("left.yaml"));
  r_camera_info_file_ = this->declare_parameter("r_camera_info_file", std::string("right.yaml"));

	bool l_info_loaded_ = read_camera_info(l_camera_info_file_, l_cam_info_);
	bool r_info_loaded_ = read_camera_info(r_camera_info_file_, r_cam_info_);

	if(l_info_loaded_ && r_info_loaded_ && rectify_)
  {
		//create rectify publsher
    l_pub_ = image_transport::create_camera_publisher(this, "/left/image_rect");
    r_pub_ = image_transport::create_camera_publisher(this, "/right/image_rect");
  } else {
		//create raw pub	
    l_pub_ = image_transport::create_camera_publisher(this, "/left/image_raw");
    r_pub_ = image_transport::create_camera_publisher(this, "/right/image_raw");
		rectify_ = false;
  }

	/*
  flip_vertical_ = this->declare_parameter("flip_vertical", false);
  frame_id_ = this->declare_parameter("frame_id", std::string("camera"));
  publish_rate_ = this->declare_parameter("publish_rate", static_cast<double>(10));
  camera_info_url_ = this->declare_parameter("camera_info_url", std::string(""));
  */

  auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      RCLCPP_INFO(get_logger(), "param_change_callback");

      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name() == "rectify") {
          //filename_ = parameter.as_string();
          rectify_ = parameter.as_bool();
          RCLCPP_INFO(get_logger(), "Reset rectify as '%d'", rectify_);
          //RpiStereo::onInit();
          //return result;
        } else if (parameter.get_name() == "publish_rate") {
          //flip_horizontal_ = parameter.as_bool();
          publish_rate_ = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Reset publish_rate as '%lf'", publish_rate_);
          //RpiStereo::onInit();
          //return result;
        } else if (parameter.get_name() == "l_camera_info_file") {
          l_camera_info_file_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "Reset l_camera_info_file as '%s'", l_camera_info_file_.c_str());
          //RpiStereo::onInit();
          //return result;
        } else if (parameter.get_name() == "r_camera_info_file") {
          r_camera_info_file_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "Reset r_camera_info_file as '%s'", r_camera_info_file_.c_str());
        } else if (parameter.get_name() == "l_frame_id") {
          l_frame_id_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "Reset l_frame_id as '%s'", l_frame_id_.c_str());
        } else if (parameter.get_name() == "r_frame_id") {
          r_frame_id_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "Reset r_frame_id as '%s'",r_frame_id_.c_str());
        }
      }
      RpiStereo::reconfigureCallback();
      return result;
    };
	try {
	  auto result = this->add_on_set_parameters_callback(param_change_callback);
		if(result != NULL)
			onInit();
	} catch(std::bad_alloc& e) {
      RCLCPP_ERROR(
        this->get_logger(), "set_parameters_callback alloc failed to load: %s",
        e.what());
  }
  //this->set_on_parameters_set_callback(param_change_callback);
}

bool RpiStereo::read_camera_info(std::string file_, sensor_msgs::msg::CameraInfo & info) 
{
  std::string camera_name;
  if (camera_calibration_parsers::readCalibration(file_, 
                                                    camera_name,
                                                    info)) {
    RCLCPP_INFO(get_logger(), "got camera info for '%s'", camera_name.c_str());
		return true;
  } else {
    RCLCPP_ERROR(get_logger(), "cannot get camera info, will not publish");
		return false;
  }
/*
  camera_info_manager::CameraInfoManager c(this);
  if (!file_.empty()) {
    RCLCPP_INFO(get_logger(), "camera_info_url exist");
    try {
      c.validateURL(file_);
      c.loadCameraInfo(file_);
      info = c.getCameraInfo();
			return true;
    } catch (cv::Exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "camera calibration failed to load: %s %s %s %i",
        e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  } else {
    RCLCPP_INFO(get_logger(), "no camera_info_url exist");
  }
	return false;
*/
}

void RpiStereo::reconfigureCallback()
{
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / publish_rate_)),
    std::bind(&RpiStereo::doWork, this));

	bool l_info_loaded_ = read_camera_info(l_camera_info_file_, l_cam_info_);
	bool r_info_loaded_ = read_camera_info(r_camera_info_file_, r_cam_info_);

	if(l_info_loaded_ && r_info_loaded_ && rectify_)
  {
		//create rectify publsher
    l_pub_ = image_transport::create_camera_publisher(this, "/left/image_rect");
    r_pub_ = image_transport::create_camera_publisher(this, "/right/image_rect");
  } else {
		//create raw pub	
    l_pub_ = image_transport::create_camera_publisher(this, "/left/image_raw");
    r_pub_ = image_transport::create_camera_publisher(this, "/right/image_raw");
		rectify_ = false;
  }
}

/*
  camera_info_manager::CameraInfoManager c(this);
  if (!camera_info_url_.empty()) {
    RCLCPP_INFO(get_logger(), "camera_info_url exist");
    try {
      c.validateURL(camera_info_url_);
      c.loadCameraInfo(camera_info_url_);
      camera_info_ = c.getCameraInfo();
    } catch (cv::Exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "camera calibration failed to load: %s %s %s %i",
        e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  } else {
    RCLCPP_INFO(get_logger(), "no camera_info_url exist");
  }
}
*/
void RpiStereo::doWork()
{
  RCLCPP_INFO(
    this->get_logger(),
    "do Work");
  // Transform the image.
  try {
		stereo_cam_->read_left(l_frame);
		stereo_cam_->read_right(r_frame);
		auto stamp = now();	
		
		if(rectify_)
		{
			cam_model_.fromCameraInfo(l_cam_info_, r_cam_info_);
			
			cam_model_.left().rectifyImage(l_frame, l_frame);
			cam_model_.right().rectifyImage(r_frame, r_frame);
		}	

    sensor_msgs::msg::Image::SharedPtr l_img =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", l_frame).toImageMsg();
    l_img->header.frame_id = l_frame_id_;
    l_img->header.stamp = stamp;
    l_cam_info_.header.frame_id = l_img->header.frame_id;
    l_cam_info_.header.stamp = l_img->header.stamp;

    l_pub_.publish(*l_img, l_cam_info_);

    sensor_msgs::msg::Image::SharedPtr r_img =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", r_frame).toImageMsg();
    r_img->header.frame_id = r_frame_id_;
    r_img->header.stamp = stamp;
    r_cam_info_.header.frame_id = r_img->header.frame_id;
    r_cam_info_.header.stamp = r_img->header.stamp;

    r_pub_.publish(*r_img, r_cam_info_);
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Image processing error: %s %s %s %i",
      e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
  }
}

void RpiStereo::onInit()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Start Cam init");
  try {
  	stereo_cam_ = std::make_shared<rpi_stereo_cam::RpiStereoCamDriver>();
  	stereo_cam_->init();
/* 
   image_ = cv::imread(filename_, cv::IMREAD_COLOR);
    if (image_.empty()) {  // if filename not exist, open video device
      try {  // if filename is number
        int num = std::stoi(filename_);  // num is 1234798797
        cap_.open(num);
      } catch (const std::invalid_argument &) {  // if file name is string
        cap_.open(filename_);
      }
      CV_Assert(cap_.isOpened());
      cap_.read(image_);
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    }
    CV_Assert(!image_.empty());
*/
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to load image : %s %s %s %i",
       e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    return;
  }

/*
  RCLCPP_INFO(
    this->get_logger(),
    "Flip horizontal image is : %s", ((flip_horizontal_) ? "true" : "false"));
  RCLCPP_INFO(
    this->get_logger(),
    "Flip flip_vertical image is : %s", ((flip_vertical_) ? "true" : "false"));
*/
  // From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html
  // #void flip(InputArray src, OutputArray dst, int flipCode)
  // FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
/*
  flip_image_ = true;
  if (flip_horizontal_ && flip_vertical_) {
    flip_value_ = 0;  // flip both, horizontal and vertical
  } else if (flip_horizontal_) {
    flip_value_ = 1;
  } else if (flip_vertical_) {
    flip_value_ = -1;
  } else {
    flip_image_ = false;
  }
*/
/*
  camera_info_.width = image_.cols;
  camera_info_.height = image_.rows;
  camera_info_.distortion_model = "plumb_bob";
  camera_info_.d = {0, 0, 0, 0, 0};
  camera_info_.k = {1, 0, static_cast<float>(camera_info_.width / 2), 0, 1,
    static_cast<float>(camera_info_.height / 2), 0, 0, 1};
  camera_info_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  camera_info_.p = {1, 0, static_cast<float>(camera_info_.width / 2), 0, 0, 1,
    static_cast<float>(camera_info_.height / 2), 0, 0, 0, 1, 0};
*/
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / publish_rate_)),
    std::bind(&RpiStereo::doWork, this));
}

}  // namespace rpi_stereo

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rpi_stereo::RpiStereo)
