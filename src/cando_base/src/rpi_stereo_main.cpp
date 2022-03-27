#include "cando_base/rpi_stereo_node.hpp"

// Launch RpiStereoCamNode with use_intra_process_comms=true

void quit(int sig)
{
				(void)sig;
				rclcpp::shutdown();
}

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

	signal(SIGINT, quit);

  // Create single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Create and add camera node
  rclcpp::NodeOptions options{};
  options.use_intra_process_comms(true);
  auto node = std::make_shared<rpi_stereo_cam::RpiStereoCamNode>(options);
  executor.add_node(node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
