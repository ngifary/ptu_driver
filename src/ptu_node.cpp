#include <chrono>
#include <functional>
#include <memory>
#include <string>

extern "C" {
  #include "ptu_driver/PCA9685.h"
}

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class PTUNode : public rclcpp::Node
{
public:
  PTUNode()
      : Node("ptu_node"), count_(0)
  {
    char i2c_dev[32];
    snprintf(i2c_dev, sizeof(i2c_dev), "/dev/i2c-1");
    i2c_init(i2c_dev, I2C_ADDR);
    i2c_writeReg(MODE1, 0x80);
    usleep(10000);
    PCA9685_setPWMFreq(1000); // Set the default frequency
    PCA9685_setPWMFreq(60);
    setServoDegree(SERVO_UP_CH, ServoUpDegree);
    setServoDegree(SERVO_DOWN_CH, ServoDownDegree);

    // publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
    subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_cmd", 10, std::bind(&PTUNode::setState, this, _1));
    // timer_ = this->create_wall_timer(
    //     500ms, std::bind(&PTUNode::timer_callback, this));
  }

private:
  void setState(const sensor_msgs::msg::JointState::ConstSharedPtr &joint_state_msg)
  {
    RCLCPP_INFO(get_logger(), "STATE RECEIVED");
    RCLCPP_DEBUG(get_logger(), "SETSTATE REQUEST - START");
    auto pan = static_cast<int>(joint_state_msg->position[0]);
    auto tilt = static_cast<int>(joint_state_msg->position[1]);
    auto pan_speed = static_cast<int>(joint_state_msg->velocity[0]);
    auto tilt_speed = static_cast<int>(joint_state_msg->velocity[1]);
    RCLCPP_DEBUG_STREAM(get_logger(), "Node received state_cmd message: (p,t)=(" << pan << "," << tilt << ") and (v_p,v_t)=(" << pan_speed << "," << tilt_speed << ").");
    setServoDegree(SERVO_UP_CH, tilt);
    setServoDegree(SERVO_DOWN_CH, pan);
    RCLCPP_DEBUG(get_logger(), "SETSTATE REQUEST - END");
  }
  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState();
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PTUNode>());
  rclcpp::shutdown();
  return 0;
}