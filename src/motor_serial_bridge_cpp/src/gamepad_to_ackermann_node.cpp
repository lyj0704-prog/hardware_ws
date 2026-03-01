#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

class GamepadToAckermannNode : public rclcpp::Node
{
public:
  GamepadToAckermannNode()
  : Node("gamepad_to_ackermann_node")
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
    ackermann_topic_ = declare_parameter<std::string>("ackermann_topic", "/ackermann_cmd");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    steering_axis_index_ = declare_parameter<int>("steering_axis_index", 0);
    forward_button_index_ = declare_parameter<int>("forward_button_index", 1);
    reverse_button_index_ = declare_parameter<int>("reverse_button_index", 3);

    max_steer_rad_ = declare_parameter<double>("max_steer_rad", 0.24);
    max_forward_speed_mps_ = declare_parameter<double>("max_forward_speed_mps", 0.2);
    max_reverse_speed_mps_ = declare_parameter<double>("max_reverse_speed_mps", 0.1);
    accel_mps2_ = declare_parameter<double>("accel_mps2", 0.4);
    steering_slew_radps_ = std::max(0.0, declare_parameter<double>("steering_slew_radps", 0.4));
    steering_threshold_ = std::clamp(
      declare_parameter<double>("steering_threshold", 0.6), 0.0, 1.0);

    steering_deadzone_ = declare_parameter<double>("steering_deadzone", 0.05);
    invert_steering_ = declare_parameter<bool>("invert_steering", true);

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    joy_timeout_ms_ = declare_parameter<int>("joy_timeout_ms", 300);
    debug_print_ = declare_parameter<bool>("debug_print", false);

    ackermann_pub_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic_, 20);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 20,
      std::bind(&GamepadToAckermannNode::onJoy, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GamepadToAckermannNode::onPublishTimer, this));

    last_publish_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "gamepad_to_ackermann_node ready. joy=%s axis=%d fwd_btn=%d rev_btn=%d "
      "max_fwd=%.3f max_rev=%.3f max_steer=%.3f steer_slew=%.3f "
      "steer_th=%.2f",
      joy_topic_.c_str(),
      steering_axis_index_,
      forward_button_index_,
      reverse_button_index_,
      max_forward_speed_mps_,
      max_reverse_speed_mps_,
      max_steer_rad_,
      steering_slew_radps_,
      steering_threshold_);
  }

private:
  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_ = *msg;
    last_joy_time_ = now();
    has_joy_ = true;
  }

  void onPublishTimer()
  {
    const rclcpp::Time t_now = now();
    const double dt = std::clamp((t_now - last_publish_time_).seconds(), 0.0, 0.2);
    last_publish_time_ = t_now;

    double steer_target_cmd = 0.0;
    bool forward_pressed = false;
    bool reverse_pressed = false;

    if (has_joy_) {
      const auto elapsed_ms = (t_now - last_joy_time_).nanoseconds() / 1000000LL;
      if (elapsed_ms <= joy_timeout_ms_) {
        if (steering_axis_index_ >= 0 &&
          static_cast<size_t>(steering_axis_index_) < last_joy_.axes.size())
        {
          double steer_input = static_cast<double>(last_joy_.axes[steering_axis_index_]);
          if (std::abs(steer_input) < steering_deadzone_) {
            steer_input = 0.0;
          }

          const double sign = invert_steering_ ? -1.0 : 1.0;
          const double abs_input = std::abs(steer_input);
          if (abs_input >= steering_threshold_) {
            const double steer_dir = (steer_input > 0.0) ? 1.0 : -1.0;
            steer_target_cmd = sign * steer_dir * max_steer_rad_;
          } else {
            steer_target_cmd = 0.0;
          }
        }

        if (forward_button_index_ >= 0 &&
          static_cast<size_t>(forward_button_index_) < last_joy_.buttons.size())
        {
          forward_pressed = (last_joy_.buttons[forward_button_index_] != 0);
        }

        if (reverse_button_index_ >= 0 &&
          static_cast<size_t>(reverse_button_index_) < last_joy_.buttons.size())
        {
          reverse_pressed = (last_joy_.buttons[reverse_button_index_] != 0);
        }
      }
    }

    double target_speed_mps = 0.0;
    if (forward_pressed && !reverse_pressed) {
      target_speed_mps = max_forward_speed_mps_;
    } else if (reverse_pressed && !forward_pressed) {
      target_speed_mps = -max_reverse_speed_mps_;
    }

    const double max_delta = accel_mps2_ * dt;
    const double delta = target_speed_mps - speed_cmd_mps_;
    if (delta > max_delta) {
      speed_cmd_mps_ += max_delta;
    } else if (delta < -max_delta) {
      speed_cmd_mps_ -= max_delta;
    } else {
      speed_cmd_mps_ = target_speed_mps;
    }

    if (steering_slew_radps_ > 0.0) {
      const double max_steer_delta = steering_slew_radps_ * dt;
      const double steer_delta = steer_target_cmd - steer_cmd_rad_;
      if (steer_delta > max_steer_delta) {
        steer_cmd_rad_ += max_steer_delta;
      } else if (steer_delta < -max_steer_delta) {
        steer_cmd_rad_ -= max_steer_delta;
      } else {
        steer_cmd_rad_ = steer_target_cmd;
      }
    } else {
      steer_cmd_rad_ = steer_target_cmd;
    }

    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = t_now;
    msg.header.frame_id = frame_id_;
    msg.drive.speed = speed_cmd_mps_;
    msg.drive.steering_angle = steer_cmd_rad_;
    ackermann_pub_->publish(msg);

    if (debug_print_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 200,
        "cmd speed=%.3f steer=%.3f steer_target=%.3f fwd=%d rev=%d target=%.3f",
        speed_cmd_mps_,
        steer_cmd_rad_,
        steer_target_cmd,
        forward_pressed ? 1 : 0,
        reverse_pressed ? 1 : 0,
        target_speed_mps);
    }
  }

  std::string joy_topic_{"/joy"};
  std::string ackermann_topic_{"/ackermann_cmd"};
  std::string frame_id_{"base_link"};

  int steering_axis_index_{0};
  int forward_button_index_{1};
  int reverse_button_index_{3};

  double max_steer_rad_{0.24};
  double max_forward_speed_mps_{0.2};
  double max_reverse_speed_mps_{0.1};
  double accel_mps2_{0.4};
  double steering_slew_radps_{0.4};
  double steering_threshold_{0.6};
  double steering_deadzone_{0.05};
  bool invert_steering_{true};

  double publish_rate_hz_{30.0};
  int joy_timeout_ms_{300};
  bool debug_print_{false};

  bool has_joy_{false};
  double speed_cmd_mps_{0.0};
  double steer_cmd_rad_{0.0};
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::Joy last_joy_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadToAckermannNode>());
  rclcpp::shutdown();
  return 0;
}
