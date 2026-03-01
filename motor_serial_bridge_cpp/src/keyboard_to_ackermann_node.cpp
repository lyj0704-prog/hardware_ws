#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

class KeyboardToAckermannNode : public rclcpp::Node
{
public:
  KeyboardToAckermannNode()
  : Node("keyboard_to_ackermann_node")
  {
    ackermann_topic_ = declare_parameter<std::string>("ackermann_topic", "/ackermann_cmd");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    forward_speed_mps_ = declare_parameter<double>("forward_speed_mps", 0.2);
    reverse_speed_mps_ = declare_parameter<double>("reverse_speed_mps", 0.2);
    max_steer_rad_ = declare_parameter<double>("max_steer_rad", 0.24);
    steer_step_rad_ = declare_parameter<double>("steer_step_rad", 0.02);

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    throttle_timeout_ms_ = declare_parameter<int>("throttle_timeout_ms", 250);
    debug_print_ = declare_parameter<bool>("debug_print", false);

    ackermann_pub_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic_, 20);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&KeyboardToAckermannNode::onPublishTimer, this));

    last_throttle_key_time_ = now();

    if (configureTerminal()) {
      input_thread_ = std::thread(&KeyboardToAckermannNode::inputLoop, this);
      RCLCPP_INFO(
        get_logger(),
        "keyboard_to_ackermann_node ready. Use arrows: Up=+speed Down=-speed Left/Right=steer. "
        "Space=stop+center, C=center, Q=quit.");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "STDIN is not a TTY or raw mode setup failed. Node will publish zero commands.");
    }
  }

  ~KeyboardToAckermannNode() override
  {
    running_.store(false);
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
    restoreTerminal();
  }

private:
  bool configureTerminal()
  {
    if (!isatty(STDIN_FILENO)) {
      return false;
    }

    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
      RCLCPP_WARN(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return false;
    }

    struct termios raw = original_termios_;
    cfmakeraw(&raw);
    raw.c_lflag |= ISIG;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      RCLCPP_WARN(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      return false;
    }

    stdin_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (stdin_flags_ >= 0) {
      (void)fcntl(STDIN_FILENO, F_SETFL, stdin_flags_ | O_NONBLOCK);
    }

    terminal_configured_ = true;
    return true;
  }

  void restoreTerminal()
  {
    if (!terminal_configured_) {
      return;
    }

    (void)tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    if (stdin_flags_ >= 0) {
      (void)fcntl(STDIN_FILENO, F_SETFL, stdin_flags_);
    }
    terminal_configured_ = false;
  }

  void inputLoop()
  {
    while (running_.load()) {
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(STDIN_FILENO, &readfds);

      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 20000;
      const int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
      if (ret <= 0 || !FD_ISSET(STDIN_FILENO, &readfds)) {
        continue;
      }

      char buf[64];
      const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
      if (n <= 0) {
        continue;
      }
      processInputBytes(buf, static_cast<size_t>(n));
    }
  }

  void processInputBytes(const char * data, size_t len)
  {
    key_buffer_.append(data, len);

    size_t idx = 0;
    while (idx < key_buffer_.size()) {
      const unsigned char c = static_cast<unsigned char>(key_buffer_[idx]);

      if (c == 0x1b) {
        if (idx + 1 >= key_buffer_.size()) {
          break;
        }
        if (key_buffer_[idx + 1] != '[') {
          idx += 1;
          continue;
        }
        if (idx + 2 >= key_buffer_.size()) {
          break;
        }

        onArrowKey(key_buffer_[idx + 2]);
        idx += 3;
        continue;
      }

      onSingleKey(static_cast<char>(c));
      idx += 1;
    }

    key_buffer_.erase(0, idx);
    if (key_buffer_.size() > 16) {
      key_buffer_.clear();
    }
  }

  void onArrowKey(char code)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    switch (code) {
      case 'A':  // Up
        speed_cmd_mps_ = std::abs(forward_speed_mps_);
        last_throttle_key_time_ = now();
        has_throttle_key_ = true;
        break;
      case 'B':  // Down
        speed_cmd_mps_ = -std::abs(reverse_speed_mps_);
        last_throttle_key_time_ = now();
        has_throttle_key_ = true;
        break;
      case 'C':  // Right
        steer_cmd_rad_ = std::clamp(
          steer_cmd_rad_ - std::abs(steer_step_rad_), -max_steer_rad_, max_steer_rad_);
        break;
      case 'D':  // Left
        steer_cmd_rad_ = std::clamp(
          steer_cmd_rad_ + std::abs(steer_step_rad_), -max_steer_rad_, max_steer_rad_);
        break;
      default:
        break;
    }

    if (debug_print_) {
      RCLCPP_INFO(
        get_logger(), "cmd speed=%.3f steer=%.3f", speed_cmd_mps_, steer_cmd_rad_);
    }
  }

  void onSingleKey(char key)
  {
    if (key == 'q' || key == 'Q') {
      RCLCPP_INFO(get_logger(), "Quit key received.");
      running_.store(false);
      rclcpp::shutdown();
      return;
    }

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (key == ' ') {
      speed_cmd_mps_ = 0.0;
      steer_cmd_rad_ = 0.0;
      has_throttle_key_ = false;
    } else if (key == 'c' || key == 'C') {
      steer_cmd_rad_ = 0.0;
    } else if (key == 's' || key == 'S') {
      speed_cmd_mps_ = 0.0;
      has_throttle_key_ = false;
    } else {
      return;
    }

    if (debug_print_) {
      RCLCPP_INFO(
        get_logger(), "cmd speed=%.3f steer=%.3f", speed_cmd_mps_, steer_cmd_rad_);
    }
  }

  void onPublishTimer()
  {
    const auto t_now = now();
    double speed_cmd = 0.0;
    double steer_cmd = 0.0;

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (has_throttle_key_) {
        const bool timed_out =
          (t_now - last_throttle_key_time_).nanoseconds() / 1000000LL > throttle_timeout_ms_;
        if (timed_out) {
          speed_cmd_mps_ = 0.0;
        }
      }
      speed_cmd = speed_cmd_mps_;
      steer_cmd = steer_cmd_rad_;
    }

    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = t_now;
    msg.header.frame_id = frame_id_;
    msg.drive.speed = speed_cmd;
    msg.drive.steering_angle = steer_cmd;
    ackermann_pub_->publish(msg);
  }

  std::string ackermann_topic_ = "/ackermann_cmd";
  std::string frame_id_ = "base_link";

  double forward_speed_mps_ = 0.2;
  double reverse_speed_mps_ = 0.2;
  double max_steer_rad_ = 0.24;
  double steer_step_rad_ = 0.02;
  double publish_rate_hz_ = 30.0;
  int throttle_timeout_ms_ = 250;
  bool debug_print_ = false;

  std::atomic<bool> running_{true};
  std::thread input_thread_;
  bool terminal_configured_ = false;
  struct termios original_termios_{};
  int stdin_flags_ = -1;
  std::string key_buffer_;

  std::mutex cmd_mutex_;
  double speed_cmd_mps_ = 0.0;
  double steer_cmd_rad_ = 0.0;
  bool has_throttle_key_ = false;
  rclcpp::Time last_throttle_key_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardToAckermannNode>());
  rclcpp::shutdown();
  return 0;
}
