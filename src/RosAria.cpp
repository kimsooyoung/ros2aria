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
    // port and baud
    serial_port = this->declare_parameter("serial_port", "/dev/ttyUSB0");
    serial_baud = this->declare_parameter("baud", 0);

    // handle debugging more elegantly
    debug_aria = this->declare_parameter("debug_aria", false);
    aria_log_filename = this->declare_parameter("aria_log_filename", "Aria.log");

    // whether to connect to lasers using aria
    publish_aria_lasers = this->declare_parameter("publish_aria_lasers", false);

    // Get frame_ids to use.
    frame_id_odom = this->declare_parameter("odom_frame", "odom");
    frame_id_base_link = this->declare_parameter("base_link_frame", "base_link");
    frame_id_bumper = this->declare_parameter("bumpers_frame", "bumpers");
    frame_id_sonar = this->declare_parameter("sonar_frame", "sonar");

    RCLCPP_WARN(this->get_logger(), "serial_port : %s", serial_port);
    RCLCPP_WARN(this->get_logger(), "serial_baud : %d", serial_baud);

    RCLCPP_WARN(this->get_logger(), "odom_frame : %s", frame_id_odom);
    RCLCPP_WARN(this->get_logger(), "base_link_frame : %s", frame_id_base_link);
    RCLCPP_WARN(this->get_logger(), "bumpers_frame : %s", frame_id_bumper);
    RCLCPP_WARN(this->get_logger(), "sonar_frame : %s", frame_id_sonar);

    // advertise services for data topics
    // second argument to advertise() is queue size.
    // other argmuments (optional) are callbacks, or a boolean "latch" flag (whether to send current data to new
    // subscribers when they subscribe).
    // See ros::NodeHandle API docs.
    pose_pub = this->create_publisher<nav_msgs::msg::Odometry>("pose", 1000);
    bumpers_pub = this->create_publisher<ros2aria::msg::BumperState>("bumper_state", 1000);
    
    sonar_pub = this->create_publisher<sensor_msgs::msg::PointCloud>("sonar", 50);
    sonar_pointcloud2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_pointcloud2", 50);
    voltage_pub = this->create_publisher<std_msgs::msg::Float64>("battery_voltage", 1000);
    recharge_state_pub = this->create_publisher<std_msgs::msg::Int8>("battery_recharge_state", 5);
    state_of_charge_pub = this->create_publisher<std_msgs::msg::Float32>("battery_state_of_charge", 100);
    motors_state_pub = this->create_publisher<std_msgs::msg::Bool>("motors_state", 5);
    published_motors_state = true;

    cmdvel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&RosAriaNode::cmdvel_cb, this, std::placeholders::_1));

    // advertise enable/disable services
    enable_srv = create_service<std_srvs::srv::Empty>(
      "enable_motors", 
      std::bind(&RosAriaNode::enable_srv_response, this, std::placeholders::_1, std::placeholders::_2));
    disable_srv = create_service<std_srvs::srv::Empty>(
      "disable_motors", 
      std::bind(&RosAriaNode::disable_srv_response, this, std::placeholders::_1, std::placeholders::_2));

    veltime = rclcpp::Clock().now();
  }

  void enable_srv_response(std::shared_ptr<std_srvs::srv::Empty::Request> request,
                          std::shared_ptr<std_srvs::srv::Empty::Response> response){
    RCLCPP_INFO(this->get_logger(), "RosAria: Enable motors request.");
    robot->lock();

    if(robot->isEStopPressed())
      RCLCPP_WARN(this->get_logger(), "RosAria: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");

    robot->enableMotors();
    robot->unlock();
  }

  void disable_srv_response(std::shared_ptr<std_srvs::srv::Empty::Request> request,
                          std::shared_ptr<std_srvs::srv::Empty::Response> response){

    RCLCPP_INFO(this->get_logger(), "RosAria: Disable motors request.");
    robot->lock();
    robot->disableMotors();
    robot->unlock();         
  }

  void cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg){
    veltime = rclcpp::Clock().now();
    RCLCPP_INFO(get_logger(), "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.seconds());

    robot->lock();
    robot->setVel(msg->linear.x*1e3);
    if(robot->hasLatVel())
      robot->setLatVel(msg->linear.y*1e3);
    robot->setRotVel(msg->angular.z*180/M_PI);
    robot->unlock();

    RCLCPP_DEBUG(get_logger(), "RosAria: sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.seconds(),
      (double) msg->linear.x * 1e3, (double) msg->linear.y * 1e3, (double) msg->angular.z * 180/M_PI);
  }

  ~RosAriaNode() {
    // disable motors and sonar.
    robot->disableMotors();
    robot->disableSonar();

    robot->stopRunning();
    robot->waitForRunExit();
    Aria::shutdown();
  }

  void sonarConnectCb()
  {
    publish_sonar = true;
    publish_sonar_pointcloud2 = true;
    robot->lock();
    if (publish_sonar || publish_sonar_pointcloud2)
    {
      robot->enableSonar();
      sonar_enabled = false;
    }
    else if(!publish_sonar && !publish_sonar_pointcloud2)
    {
      robot->disableSonar();
      sonar_enabled = true;
    }
    robot->unlock();
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
