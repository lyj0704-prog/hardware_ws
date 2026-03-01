#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

class JoyToAckermannNode : public rclcpp::Node
{
public:
  JoyToAckermannNode()
  : Node("joy_to_ackermann_node")
  {
    joy_left_topic_ = declare_parameter<std::string>("joy_left_topic", "/joy_left");
    joy_right_topic_ = declare_parameter<std::string>("joy_right_topic", "/joy_right");
    ackermann_topic_ = declare_parameter<std::string>("ackermann_topic", "/ackermann_cmd");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    max_steer_rad_ = declare_parameter<double>("max_steer_rad", 0.24);
    forward_speed_mps_ = declare_parameter<double>("forward_speed_mps", 0.2);
    reverse_speed_mps_ = declare_parameter<double>("reverse_speed_mps", 0.5);

    steering_axis_index_ = declare_parameter<int>("steering_axis_index", 0);
    forward_button_index_ = declare_parameter<int>("forward_button_index", 1);
    reverse_button_index_ = declare_parameter<int>("reverse_button_index", -1);

    steering_deadzone_ = declare_parameter<double>("steering_deadzone", 0.05);
    invert_steering_ = declare_parameter<bool>("invert_steering", false);

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    joy_timeout_ms_ = declare_parameter<int>("joy_timeout_ms", 300);
    hold_last_steering_on_timeout_ =
      declare_parameter<bool>("hold_last_steering_on_timeout", false);

    ackermann_pub_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic_, 20);
    joy_left_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_left_topic_, 20,
      std::bind(&JoyToAckermannNode::onLeftJoy, this, std::placeholders::_1));
    joy_right_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_right_topic_, 20,
      std::bind(&JoyToAckermannNode::onRightJoy, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&JoyToAckermannNode::onPublishTimer, this));

    const auto t0 = now();
    last_left_joy_time_ = t0;
    last_right_joy_time_ = t0;

    RCLCPP_INFO(
      get_logger(),
      "joy_to_ackermann_node started. joy_left_topic=%s joy_right_topic=%s ackermann_topic=%s "
      "steer_axis=%d forward_button=%d",
      joy_left_topic_.c_str(), joy_right_topic_.c_str(), ackermann_topic_.c_str(),
      steering_axis_index_, forward_button_index_);
  }

private:
  static bool getButtonValue(const sensor_msgs::msg::Joy & msg, int button_index)
  {
    if (button_index < 0) {
      return false;
    }
    const size_t idx = static_cast<size_t>(button_index);
    if (idx >= msg.buttons.size()) {
      return false;
    }
    return msg.buttons[idx] != 0;
  }

  static double getAxisValue(const sensor_msgs::msg::Joy & msg, int axis_index)
  {
    if (axis_index < 0) {
      return 0.0;
    }
    const size_t idx = static_cast<size_t>(axis_index);
    if (idx >= msg.axes.size()) {
      return 0.0;
    }
    return static_cast<double>(msg.axes[idx]);
  }

  void onLeftJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    double steer_axis = getAxisValue(*msg, steering_axis_index_);
    if (invert_steering_) {
      steer_axis *= -1.0;
    }
    if (std::abs(steer_axis) < steering_deadzone_) {
      steer_axis = 0.0;
    }

    latest_steer_rad_ = std::clamp(steer_axis, -1.0, 1.0) * max_steer_rad_;
    has_left_joy_ = true;
    last_left_joy_time_ = now();
  }

  void onRightJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const bool forward_pressed = getButtonValue(*msg, forward_button_index_);
    const bool reverse_pressed = getButtonValue(*msg, reverse_button_index_);

    if (forward_pressed && !reverse_pressed) {
      latest_speed_mps_ = std::abs(forward_speed_mps_);
    } else if (reverse_pressed && !forward_pressed) {
      latest_speed_mps_ = -std::abs(reverse_speed_mps_);
    } else {
      latest_speed_mps_ = 0.0;
    }
    has_right_joy_ = true;
    last_right_joy_time_ = now();
  }

  void onPublishTimer()
  {
    const auto t_now = now();
    double speed_cmd = 0.0;
    double steer_cmd = 0.0;

    if (has_right_joy_) {
      const bool right_timed_out =
        (t_now - last_right_joy_time_).nanoseconds() / 1000000LL > joy_timeout_ms_;
      if (!right_timed_out) {
        speed_cmd = latest_speed_mps_;
      }
    }

    if (has_left_joy_) {
      const bool left_timed_out =
        (t_now - last_left_joy_time_).nanoseconds() / 1000000LL > joy_timeout_ms_;
      if (!left_timed_out) {
        steer_cmd = latest_steer_rad_;
      } else if (hold_last_steering_on_timeout_) {
        steer_cmd = latest_steer_rad_;
      }
    }

    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = t_now;
    msg.header.frame_id = frame_id_;
    msg.drive.speed = speed_cmd;
    msg.drive.steering_angle = steer_cmd;
    ackermann_pub_->publish(msg);
  }

  std::string joy_left_topic_ = "/joy_left";
  std::string joy_right_topic_ = "/joy_right";
  std::string ackermann_topic_ = "/ackermann_cmd";
  std::string frame_id_ = "base_link";

  double max_steer_rad_ = 0.24;
  double forward_speed_mps_ = 0.2;
  double reverse_speed_mps_ = 0.5;
  int steering_axis_index_ = 0;
  int forward_button_index_ = 1;
  int reverse_button_index_ = -1;
  double steering_deadzone_ = 0.05;
  bool invert_steering_ = false;
  double publish_rate_hz_ = 30.0;
  int joy_timeout_ms_ = 300;
  bool hold_last_steering_on_timeout_ = false;

  bool has_left_joy_ = false;
  bool has_right_joy_ = false;
  double latest_speed_mps_ = 0.0;
  double latest_steer_rad_ = 0.0;
  rclcpp::Time last_left_joy_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_right_joy_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_right_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToAckermannNode>());
  rclcpp::shutdown();
  return 0;
}
