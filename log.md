[x] ROS 2 parameter server 복습
=> 기본은 디폴트 값으로 매긴 다음 cb을 통해 예외처리 / 값 변경하도록 하자
=> 

TicksMM
DriftFactor
RevCount
trans_accel
trans_decel
lat_accel
lat_decel
rot_accel
rot_decel

readParameters

TicksMM => 0
DriftFactor => 0
RevCount => 0


dynamic_reconfigure_server

trans_accel
lat_accel
rot_accel

dynamic_reconfigureCB

    robot->comInt(93, TicksMM);
    robot->comInt(89, DriftFactor);
    robot->comInt(88, RevCount);
    robot->setTransAccel(value);
    robot->setTransDecel(value);
    robot->setLatAccel(value);
    robot->setLatDecel(value);
    robot->setRotAccel(value);
    robot->setRotDecel(value);


```
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
class TestParams : public rclcpp::Node
{
public:
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &parameter : parameters)
        {
            if (parameter.get_name() == "my_str" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                my_str_ = parameter.as_string();
                RCLCPP_INFO(this->get_logger(), "Parameter 'my_str' changed: %s", my_str_.c_str());
            }
        }
        return result;
    }
    TestParams() : Node("test_params_rclcpp")
    {
        this->declare_parameter("my_str", "default value");
        my_str_ = this->get_parameter("my_str").as_string();
        this->set_on_parameters_set_callback(
            std::bind(&TestParams::parametersCallback, this, std::placeholders::_1));
    }
private:
    std::string my_str_;
};
```

colcon build --symlink-install --packages-select rosaria_msgs && source install/local_setup.bash

ros2 interface show rosaria_msgs/msg/BumperState 
std_msgs/Header header
bool[] front_bumpers
bool[] rear_bumpers

colcon build --symlink-install --packages-select rosaria