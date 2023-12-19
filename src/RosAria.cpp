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
  RosAriaNode() : Node("ros_aria_node"), myPublishCB(this, &RosAriaNode::publish)
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
      std::bind(&RosAriaNode::enable_motors_cb, this, std::placeholders::_1, std::placeholders::_2));
    disable_srv = create_service<std_srvs::srv::Empty>(
      "disable_motors", 
      std::bind(&RosAriaNode::disable_motors_cb, this, std::placeholders::_1, std::placeholders::_2));

    veltime = rclcpp::Clock().now();
    
    serial_port = "";
    serial_baud = 0; 
    conn = NULL;
    laserConnector = NULL;
    robot = NULL;
    
    sonar_enabled = false;
    publish_sonar = false;
    publish_sonar_pointcloud2 = false;
    debug_aria = false;

    TicksMM = -1;
    DriftFactor = -99999;
    RevCount = -1;
    publish_aria_lasers = false;
  }

  int Setup(){
    // Note, various objects are allocated here which are never deleted (freed), since Setup() is only supposed to be
    // called once per instance, and these objects need to persist until the process terminates.

    robot = new ArRobot();
    ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
    ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
    argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)

    // Now add any parameters given via ros params (see RosAriaNode constructor):

    // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
    // for wireless serial connection. Otherwise, interpret it as a serial port name.
    size_t colon_pos = serial_port.find(":");
    if (colon_pos != std::string::npos)
    {
      args->add("-remoteHost"); // pass robot's hostname/IP address to Aria
      args->add(serial_port.substr(0, colon_pos).c_str());
      args->add("-remoteRobotTcpPort"); // pass robot's TCP port to Aria
      args->add(serial_port.substr(colon_pos+1).c_str());
    }
    else
    {
      args->add("-robotPort %s", serial_port.c_str()); // pass robot's serial port to Aria
    }

    // if a baud rate was specified in baud parameter
    if(serial_baud != 0)
    {
      args->add("-robotBaud %d", serial_baud);
    }
    
    if( debug_aria )
    {
      // turn on all ARIA debugging
      args->add("-robotLogPacketsReceived"); // log received packets
      args->add("-robotLogPacketsSent"); // log sent packets
      args->add("-robotLogVelocitiesReceived"); // log received velocities
      args->add("-robotLogMovementSent");
      args->add("-robotLogMovementReceived");
      ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename.c_str(), true);
    }

    // Connect to the robot
    conn = new ArRobotConnector(argparser, robot); // warning never freed
    if (!conn->connectRobot()) {
      RCLCPP_ERROR(this->get_logger(), "RosAria: ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
      return 1;
    }

    if(publish_aria_lasers)
      laserConnector = new ArLaserConnector(argparser, robot, conn);

    // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
    if(!Aria::parseArgs())
    {
      RCLCPP_ERROR(this->get_logger(), "RosAria: ARIA error parsing ARIA startup parameters!");
      return 1;
    }

    // readParameters();

    // Start dynamic_reconfigure server
    // dynamic_reconfigure_server = new dynamic_reconfigure::Server<rosaria::RosAriaConfig>;

    // Setup Parameter Minimums and maximums
    // rosaria::RosAriaConfig dynConf_min;
    // rosaria::RosAriaConfig dynConf_max;
    
    // dynConf_max.trans_accel = robot->getAbsoluteMaxTransAccel() / 1000;
    // dynConf_max.trans_decel = robot->getAbsoluteMaxTransDecel() / 1000;
    
    // TODO: Fix rqt dynamic_reconfigure gui to handle empty intervals
    // Until then, set unit length interval.
    // dynConf_max.lat_accel = ((robot->getAbsoluteMaxLatAccel() > 0.0) ? robot->getAbsoluteMaxLatAccel() : 0.1) / 1000;
    // dynConf_max.lat_decel = ((robot->getAbsoluteMaxLatDecel() > 0.0) ? robot->getAbsoluteMaxLatDecel() : 0.1) / 1000;
    // dynConf_max.rot_accel = robot->getAbsoluteMaxRotAccel() * M_PI/180;
    // dynConf_max.rot_decel = robot->getAbsoluteMaxRotDecel() * M_PI/180;

    // dynConf_min.trans_accel = 0;
    // dynConf_min.trans_decel = 0;
    // dynConf_min.lat_accel = 0;
    // dynConf_min.lat_decel = 0;
    // dynConf_min.rot_accel = 0;
    // dynConf_min.rot_decel = 0;
    
    // dynConf_min.TicksMM     = 0;
    // dynConf_max.TicksMM     = 200;
    // dynConf_min.DriftFactor = -99999;
    // dynConf_max.DriftFactor = 32767;
    // dynConf_min.RevCount    = 0;
    // dynConf_max.RevCount    = 65535;

    // dynamic_reconfigure_server->setConfigMax(dynConf_max);
    // dynamic_reconfigure_server->setConfigMin(dynConf_min);
    
    // rosaria::RosAriaConfig dynConf_default;
    // dynConf_default.trans_accel = robot->getTransAccel() / 1000;
    // dynConf_default.trans_decel = robot->getTransDecel() / 1000;
    // dynConf_default.lat_accel   = robot->getLatAccel() / 1000;
    // dynConf_default.lat_decel   = robot->getLatDecel() / 1000;
    // dynConf_default.rot_accel   = robot->getRotAccel() * M_PI/180;
    // dynConf_default.rot_decel   = robot->getRotDecel() * M_PI/180;

    // dynConf_default.TicksMM     = 0;
    // dynConf_default.DriftFactor = -99999;
    // dynConf_default.RevCount    = 0;

    // dynamic_reconfigure_server->setConfigDefault(dynConf_default);
    // dynamic_reconfigure_server->setCallback(boost::bind(&RosAriaNode::dynamic_reconfigureCB, this, _1, _2));

    // Enable the motors
    robot->enableMotors();

    // disable sonars on startup
    robot->disableSonar();

    // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
    // robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);

    // Initialize bumpers with robot number of bumpers
    bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
    bumpers.rear_bumpers.resize(robot->getNumRearBumpers());

    // Run ArRobot background processing thread
    robot->runAsync(true);

    // connect to lasers and create publishers
    if(publish_aria_lasers)
    {
      RCLCPP_INFO(this->get_logger(), "rosaria: Connecting to laser(s) configured in ARIA parameter file(s)...");
      if (!laserConnector->connectLasers())
      {
        RCLCPP_FATAL(this->get_logger(), "rosaria: Error connecting to laser(s)...");
        return 1;
      }

      robot->lock();
      const std::map<int, ArLaser*> *lasers = robot->getLaserMap();
      RCLCPP_INFO(this->get_logger(), "rosaria: there are %lu connected lasers", lasers->size());

      for(std::map<int, ArLaser*>::const_iterator i = lasers->begin(); i != lasers->end(); ++i)
      {
        ArLaser *l = i->second;
        int ln = i->first;
        std::string tfname("laser");
        if(lasers->size() > 1 || ln > 1) // no number if only one laser which is also laser 1
          tfname += ln; 
        tfname += "_frame";
        RCLCPP_INFO(this->get_logger(), "rosaria: Creating publisher for laser #%d named %s with tf frame name %s", ln, l->getName(), tfname.c_str());
        // TODO: LaserPublisher
        // new LaserPublisher(l, n, true, tfname);
      }
      robot->unlock();
      RCLCPP_INFO(this->get_logger(), "rosaria: Done creating laser publishers");
    }

    // register a watchdog for cmd_vel timeout

    // double cmdvel_timeout_param = 0.6;
    // n.param("cmd_vel_timeout", cmdvel_timeout_param, 0.6);
    // cmdvel_timeout = ros::Duration(cmdvel_timeout_param);
    // if (cmdvel_timeout_param > 0.0)
    //   cmdvel_watchdog_timer = n.createTimer(ros::Duration(0.1), 
    //   &RosAriaNode::cmdvel_watchdog, this);

    cmdvel_watchdog_timer = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RosAriaNode::cmdvel_watchdog, this)
    );

    RCLCPP_INFO(this->get_logger(), "rosaria: Setup complete");
    return 0;
  }

  void publish(){
    // Note, this is called via SensorInterpTask callback (myPublishCB, named "ROSPublishingTask"). ArRobot object 'robot' sholud not be locked or unlocked.
    pos = robot->getPose();

    tf2::Quaternion quat;
    quat.setRPY(0, 0, pos.getTh() * M_PI / 180.0); // Convert degrees to radians
    position.pose.pose.orientation.x = quat.x();
    position.pose.pose.orientation.y = quat.y();
    position.pose.pose.orientation.z = quat.z();
    position.pose.pose.orientation.w = quat.w();
    //Aria returns pose in mm.
    position.pose.pose.position.x = pos.getX() / 1000.0; // Convert millimeters to meters
    position.pose.pose.position.y = pos.getY() / 1000.0; // Convert millimeters to meters
    position.pose.pose.position.z = 0.0; // Assuming the Z position is 0

    // tf::poseTFToMsg(
    //   tf::Transform(
    //     tf::createQuaternionFromYaw(pos.getTh() * M_PI/180), 
    //     tf::Vector3(pos.getX()/1000,
    //     pos.getY()/1000, 0)
    //   ),
    //   position.pose.pose
    // ); //Aria returns pose in mm.
    
    position.twist.twist.linear.x = robot->getVel()/1000.0; //Aria returns velocity in mm/s.
    position.twist.twist.linear.y = robot->getLatVel()/1000.0;
    position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;
    
    position.header.frame_id = frame_id_odom;
    position.child_frame_id = frame_id_base_link;
    position.header.stamp = rclcpp::Clock().now();
    pose_pub->publish(position);

    RCLCPP_DEBUG(this->get_logger(), 
      "RosAria: publish: (time %d) pose x: %f, pose y: %f, pose angle: %f; linear vel x: %f, vel y: %f; angular vel z: %f", 
      position.header.stamp.sec,
      (double)position.pose.pose.position.x,
      (double)position.pose.pose.position.y,
      (double)position.pose.pose.orientation.w,
      (double)position.twist.twist.linear.x,
      (double)position.twist.twist.linear.y,
      (double)position.twist.twist.angular.z
    );

  }

  void cmdvel_watchdog(){
    // stop robot if no cmd_vel message was received for 0.6 seconds
    auto time_diff = rclcpp::Clock().now() - veltime;
    if (time_diff.seconds() > 0.6)
    {
      robot->lock();
      robot->setVel(0.0);
      if(robot->hasLatVel())
        robot->setLatVel(0.0);
      robot->setRotVel(0.0);
      robot->unlock();
    }
  }

  void enable_motors_cb(std::shared_ptr<std_srvs::srv::Empty::Request> request,
                          std::shared_ptr<std_srvs::srv::Empty::Response> response){
    RCLCPP_INFO(this->get_logger(), "RosAria: Enable motors request.");
    robot->lock();

    if(robot->isEStopPressed())
      RCLCPP_WARN(this->get_logger(), "RosAria: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");

    robot->enableMotors();
    robot->unlock();
  }

  void disable_motors_cb(std::shared_ptr<std_srvs::srv::Empty::Request> request,
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
