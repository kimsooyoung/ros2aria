#include <stdio.h>
#include <math.h>
#ifdef ADEPT_PKG
  #include <Aria.h>
  #include <ArRobotConfigPacketReader.h> // todo remove after ArRobotConfig implemented in AriaCoda
#else
  #include <Aria/Aria.h>
  #include <Aria/ArRobotConfigPacketReader.h> // todo remove after ArRobotConfig implemented in AriaCoda
#endif

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <sensor_msgs/msg/point_cloud.hpp>  //for sonar data
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp> // can optionally publish sonar as new type pointcloud2
#include "nav_msgs/msg/odometry.hpp"
#include "ros2aria/msg/bumper_state.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include <tf2/transform_datatypes.h>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"

#include <sstream>

class RosAriaNode : public rclcpp::Node
{
public:
  RosAriaNode()
    : Node("ros_aria_node")
  {
    // Create a publisher for a std_msgs/String topic
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    // Create a timer to publish messages at a specified rate
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&RosAriaNode::publishMessage, this));
  }

  ~RosAriaNode() {
    
  }

private:
  void publishMessage()
  {
    // Create a message
    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2!";

    // Publish the message
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);
  Aria::init();

  // Create a ROS 2 node
  auto node = std::make_shared<RosAriaNode>();

  // Spin the node to start processing messages
  rclcpp::spin(node);

  // Shutdown the ROS 2 system
  rclcpp::shutdown();

  return 0;
}
