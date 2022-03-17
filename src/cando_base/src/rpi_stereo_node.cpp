#include "cando_base/rpi_stereo_node.hpp"

#include <iostream>

#include "camera_calibration_parsers/parse.hpp"

namespace rpi_stereo_cam
{
  std::string mat_type2encoding(int mat_type)
  {
    switch (mat_type) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        throw std::runtime_error("unsupported encoding type");
    }
  }

  RpiStereoCamNode::RpiStereoCamNode(const rclcpp::NodeOptions &options) :
    Node("rpi_stereo_cam", options),
    canceled_(false)
  {
    RCLCPP_INFO(get_logger(), "use_intra_process_comms=%d", options.use_intra_process_comms());

    // Initialize parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(RPI_STEREO_CAM_ALL_PARAMS, validate_parameters)

    // Register for parameter changed. NOTE at this point nothing is done when parameters change.
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, RPI_STEREO_CAM_ALL_PARAMS, 
                                                   validate_parameters)

    // Log the current parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(cxt_, n, t, d)
    CXT_MACRO_LOG_SORTED_PARAMETERS(RCLCPP_INFO, get_logger(), 
                                    "rpi_stereo_cam Parameters", RPI_STEREO_CAM_ALL_PARAMS)

    // Check that all command line parameters are registered
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), RPI_STEREO_CAM_ALL_PARAMS)

    RCLCPP_INFO(get_logger(), "OpenCV version %d", CV_VERSION_MAJOR);

    //rpi stereo does not treat file
    // Open file or device
    /*
    if (cxt_.file_) {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.filename_);

      if (!capture_->isOpened()) {
        RCLCPP_ERROR(get_logger(), "cannot open file %s", cxt_.filename_.c_str());
        return;
      }

      if (cxt_.fps_ > 0) {
        // Publish at the specified rate
        publish_fps_ = cxt_.fps_;
      } else {
        // Publish at the recorded rate
        publish_fps_ = static_cast<int>(capture_->get(cv::CAP_PROP_FPS));
      }

      double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
      double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
      RCLCPP_INFO(get_logger(), "file %s open, width %g, height %g, publish fps %d",
                  cxt_.filename_.c_str(), width, height, publish_fps_);

      next_stamp_ = now();

    } else { 
      //capture_ = std::make_shared<cv::VideoCapture>(cxt_.index_);

      if (!capture_->isOpened()) {
        RCLCPP_ERROR(get_logger(), "cannot open device %d", cxt_.index_);
        return;
      }

      if (cxt_.height_ > 0) {
        capture_->set(cv::CAP_PROP_FRAME_HEIGHT, cxt_.height_);
      }

      if (cxt_.width_ > 0) {
        capture_->set(cv::CAP_PROP_FRAME_WIDTH, cxt_.width_);
      }

      if (cxt_.fps_ > 0) {
        capture_->set(cv::CAP_PROP_FPS, cxt_.fps_);
      }

      double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
      double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
      double fps = capture_->get(cv::CAP_PROP_FPS);
      RCLCPP_INFO(get_logger(), "device %d open, width %g, height %g, device fps %g",
                  cxt_.index_, width, height, fps);
    }*/

    stereo_cam_ = std::make_shared<RpiStereoCamDriver>();
    stereo_cam_->init(); 

    assert(!cxt_.lcamera_info_path_.empty()); // readCalibration will crash if file_name is ""
    std::string camera_name;
    if (camera_calibration_parsers::readCalibration(cxt_.lcamera_info_path_, 
                                                    camera_name,
                                                    lcam_info_msg_)) {
      RCLCPP_INFO(get_logger(), "got camera info for '%s'", camera_name.c_str());
      lcam_info_msg_.header.frame_id = cxt_.lcamera_frame_id_;
      lcamera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
    } else {
      RCLCPP_ERROR(get_logger(), "cannot get camera info, will not publish");
      lcamera_info_pub_ = nullptr;
    }

    l_raw_pub_ = create_publisher<sensor_msgs::msg::Image>("left/image_raw", 10);
    l_rect_pub_ = create_publisher<sensor_msgs::msg::Image>("left/image_rect", 10);

    assert(!cxt_.rcamera_info_path_.empty()); // readCalibration will crash if file_name is ""
    if (camera_calibration_parsers::readCalibration(cxt_.rcamera_info_path_, 
                                                    camera_name,
                                                    rcam_info_msg_)) {
      RCLCPP_INFO(get_logger(), "got camera info for '%s'", camera_name.c_str());
      rcam_info_msg_.header.frame_id = cxt_.rcamera_frame_id_;
      rcamera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
    } else {
      RCLCPP_ERROR(get_logger(), "cannot get camera info, will not publish");
      rcamera_info_pub_ = nullptr;
    }

		cam_model_.fromCameraInfo(lcam_info_msg_, rcam_info_msg_);

    r_raw_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_raw", 10);
    r_rect_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_rect", 10);

    // Run loop on it's own thread
    thread_ = std::thread(std::bind(&RpiStereoCamNode::loop, this));

    RCLCPP_INFO(get_logger(), "start publishing");
  }

  RpiStereoCamNode::~RpiStereoCamNode()
  {
    // Stop loop
    canceled_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void RpiStereoCamNode::validate_parameters()
  {}

  sensor_msgs::msg::Image* RpiStereoCamNode::make_msg(rclcpp::Time stamp, 
                                                      std::string id, 
                                                      cv::Mat &frame)
  {

    sensor_msgs::msg::Image *msg = new sensor_msgs::msg::Image();

    msg->header.stamp = stamp;
    msg->header.frame_id = id;
    msg->height = frame.rows;
    msg->width = frame.cols;
    msg->encoding = mat_type2encoding(frame.type());
    msg->is_bigendian = false;
    msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    msg->data.assign(frame.datastart, frame.dataend);

    return msg;
  }

  void RpiStereoCamNode::loop()
  {
    cv::Mat rframe, lframe;
		cv::Mat r_rect, l_rect;

    while (rclcpp::ok() && !canceled_.load()) {
      // Read a frame, if this is a device block until a frame is available

      stereo_cam_->read_left(lframe);
      auto l_stamp = now();
      stereo_cam_->read_right(rframe);
      auto r_stamp = now();

			cam_model_.right().rectifyImage(rframe, r_rect);
			cam_model_.left().rectifyImage(lframe, l_rect);
      /*
      if (!capture_->read(frame)) {
        RCLCPP_INFO(get_logger(), "EOF, stop publishing");
        break;
      }
      */

      //auto stamp = now();

      // Avoid copying image message if possible
      //sensor_msgs::msg::Image::UniquePtr l_img_raw_msg(new sensor_msgs::msg::Image());
      sensor_msgs::msg::Image::UniquePtr l_img_raw_msg(make_msg(l_stamp,
                                                                 cxt_.lcamera_frame_id_,
                                                                 lframe));
      
      sensor_msgs::msg::Image::UniquePtr l_img_rect_msg(make_msg(l_stamp,
                                                                 cxt_.lcamera_frame_id_,
                                                                 l_rect));

      // Convert OpenCV Mat to ROS Image
      /*
      l_img_raw_msg->header.stamp = l_stamp;
      l_img_raw_msg->header.frame_id = cxt_.lcamera_frame_id_;
      l_img_raw_msg->height = lframe.rows;
      l_img_raw_msg->width = lframe.cols;
      l_img_raw_msg->encoding = mat_type2encoding(lframe.type());
      l_img_raw_msg->is_bigendian = false;
      l_img_raw_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(lframe.step);
      l_img_raw_msg->data.assign(lframe.datastart, lframe.dataend);
      */

      // Avoid copying image message if possible
      //sensor_msgs::msg::Image::UniquePtr r_img_raw_msg(new sensor_msgs::msg::Image());
      sensor_msgs::msg::Image::UniquePtr r_img_raw_msg(make_msg(l_stamp,
                                                                cxt_.rcamera_frame_id_,
                                                                rframe));
      
      sensor_msgs::msg::Image::UniquePtr r_img_rect_msg(make_msg(l_stamp,
                                                                  cxt_.rcamera_frame_id_,
                                                                  r_rect));

      // Convert OpenCV Mat to ROS Image
      /*
      r_img_raw_msg->header.stamp = l_stamp;
      r_img_raw_msg->header.frame_id = cxt_.rcamera_frame_id_;
      r_img_raw_msg->height = rframe.rows;
      r_img_raw_msg->width = rframe.cols;
      r_img_raw_msg->encoding = mat_type2encoding(rframe.type());
      r_img_raw_msg->is_bigendian = false;
      r_img_raw_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(rframe.step);
      r_img_raw_msg->data.assign(rframe.datastart, rframe.dataend);
      */
#undef SHOW_ADDRESS
#ifdef SHOW_ADDRESS
      static int count = 0;
      RCLCPP_INFO(get_logger(), "%d, %p", count++, 
                                reinterpret_cast<std::uintptr_t>(l_img_raw_msg.get()));
      RCLCPP_INFO(get_logger(), "%d, %p", count++, 
                                reinterpret_cast<std::uintptr_t>(r_img_raw_msg.get()));
#endif

      // Publish
      l_raw_pub_->publish(std::move(l_img_raw_msg));
      l_rect_pub_->publish(std::move(l_img_rect_msg));
      if (lcamera_info_pub_) {
        lcam_info_msg_.header.stamp = l_stamp;
        lcamera_info_pub_->publish(lcam_info_msg_);
      }

      // Publish
      r_raw_pub_->publish(std::move(r_img_raw_msg));
      r_rect_pub_->publish(std::move(r_img_rect_msg));
      if (rcamera_info_pub_) {
        rcam_info_msg_.header.stamp = l_stamp;
        rcamera_info_pub_->publish(rcam_info_msg_);
      }
      // Sleep if required
      // not support file
      /*
      if (cxt_.file_) {
        using namespace std::chrono_literals;
        next_stamp_ = next_stamp_ + rclcpp::Duration{1000000000ns / publish_fps_};
        auto wait = next_stamp_ - stamp;
        if (wait.nanoseconds() > 0) {
          std::this_thread::sleep_for(static_cast<std::chrono::nanoseconds>(wait.nanoseconds()));
        }
      }
      */
    }
  }

} // namespace rpi_stereo_cam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rpi_stereo_cam::RpiStereoCamNode)
