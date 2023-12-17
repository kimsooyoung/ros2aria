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

/** @brief Node that interfaces between ROS and mobile robot base features via ARIA library. 

    RosAriaNode will use ARIA to connect to a robot controller (configure via
    ~port parameter), either direct serial connection or over the network.  It 
    runs ARIA's robot communications cycle in a background thread, and
    as part of that cycle (a sensor interpretation task which calls RosAriaNode::publish()),
    it  publishes various topics with newly received robot
    data.  It also sends velocity commands to the robot when received in the
    cmd_vel topic, and handles dynamic_reconfigure and Service requests.

    For more information about ARIA see
    http://robots.mobilerobots.com/wiki/Aria.

    RosAria uses the roscpp client library, see http://www.ros.org/wiki/roscpp for
    information, tutorials and documentation.
*/

class RosAriaNode : public rclcpp::Node
{
public:
  RosAriaNode() : Node("ros_aria_node")
  {
    // // Create a publisher for a std_msgs/String topic
    // publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    // // Create a timer to publish messages at a specified rate
    // timer_ = this->create_wall_timer(
    //   std::chrono::seconds(1), std::bind(&RosAriaNode::publishMessage, this));
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

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
  rclcpp::Publisher<ros2aria::msg::BumperState>::SharedPtr bumpers_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr sonar_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_pointcloud2_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr recharge_state_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_of_charge_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_state_pub;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub;
  
  // class variables related to topics
  std_msgs::msg::Int8 recharge_state;
  std_msgs::msg::Bool motors_state;
  bool published_motors_state;

  // Service Servers
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enable_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disable_srv;

  rclcpp::Time veltime;
  rclcpp::TimerBase::SharedPtr cmdvel_watchdog_timer;
  // rclcpp::Duration cmdvel_timeout;

  std::string serial_port;
  int serial_baud;

  ArRobotConnector *conn;
  ArLaserConnector *laserConnector;
  ArRobot *robot;
  nav_msgs::msg::Odometry position;
  ros2aria::msg::BumperState bumpers;
  ArPose pos;
  ArFunctorC<RosAriaNode> myPublishCB;

  //for odom->base_link transform
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
  geometry_msgs::msg::TransformStamped odom_trans;

  std::string frame_id_odom;
  std::string frame_id_base_link;
  std::string frame_id_bumper;
  std::string frame_id_sonar;

  // flag indicating whether sonar was enabled or disabled on the robot
  bool sonar_enabled; 

  // enable and publish sonar topics. set to true when first subscriber connects, set to false when last subscriber disconnects. 
  bool publish_sonar; 
  bool publish_sonar_pointcloud2;

  // Debug Aria
  bool debug_aria;
  std::string aria_log_filename;
  
  // Robot Calibration Parameters (see readParameters() function)
  int TicksMM, DriftFactor, RevCount;  //If TicksMM or RevCount are <0, don't use. If DriftFactor is -99999, don't use (DriftFactor could be 0 or negative).

  // TODO: dynamic_reconfigure into parameter callback
  // dynamic_reconfigure::Server<rosaria::RosAriaConfig> *dynamic_reconfigure_server;

  // whether to publish aria lasers
  bool publish_aria_lasers;
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
