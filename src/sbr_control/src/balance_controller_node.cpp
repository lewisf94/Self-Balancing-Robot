#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sbr_msgs/msg/balance_state.hpp"

#include "sbr_control/balance_controller.hpp"

namespace sbr_control
{

/// ROS 2 wrapper around BalanceController.
///
/// Subscribes to an IMU and /cmd_vel, runs the balance loop on a fixed-rate
/// timer and publishes normalized wheel efforts plus a BalanceState telemetry
/// message.
class BalanceControllerNode : public rclcpp::Node
{
public:
  BalanceControllerNode()
  : rclcpp::Node("balance_controller_node")
  {
    loop_rate_ = declare_parameter<double>("loop_rate", 200.0);

    BalanceController::Params p;
    p.pitch_gains.kp = declare_parameter<double>("pitch_kp", 6.0);
    p.pitch_gains.ki = declare_parameter<double>("pitch_ki", 0.0);
    p.pitch_gains.kd = declare_parameter<double>("pitch_kd", 0.4);
    p.pitch_gains.integral_limit = declare_parameter<double>("integral_limit", 0.5);
    p.pitch_gains.output_limit = declare_parameter<double>("output_limit", 1.0);
    p.pitch_offset = declare_parameter<double>("pitch_offset", 0.0);
    p.output_scale = declare_parameter<double>("output_scale", 1.0);
    p.fall_threshold = declare_parameter<double>("fall_threshold", 0.78);
    p.lean_per_velocity = declare_parameter<double>("lean_per_velocity", 0.08);
    p.steer_gain = declare_parameter<double>("steer_gain", 0.4);
    controller_.set_params(p);

    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    invert_pitch_ = declare_parameter<bool>("invert_pitch", false);

    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto wheel_cmd_topic = declare_parameter<std::string>("wheel_cmd_topic", "/wheel_cmd");

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&BalanceControllerNode::on_imu, this, std::placeholders::_1));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      std::bind(&BalanceControllerNode::on_cmd_vel, this, std::placeholders::_1));

    wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(wheel_cmd_topic, 10);
    state_pub_ = create_publisher<sbr_msgs::msg::BalanceState>("balance_state", 10);

    last_cmd_time_ = now();
    // Clock-based timer (not create_wall_timer) so the loop honours
    // use_sim_time in Gazebo; on hardware the node clock is system time.
    timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(1.0 / loop_rate_),
      std::bind(&BalanceControllerNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "balance_controller_node started at %.1f Hz", loop_rate_);
  }

private:
  static double pitch_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    // Pitch = rotation about the Y axis.
    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::fabs(sinp) >= 1.0) {
      return std::copysign(M_PI / 2.0, sinp);
    }
    return std::asin(sinp);
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double pitch = pitch_from_quaternion(msg->orientation);
    double pitch_rate = msg->angular_velocity.y;
    if (invert_pitch_) {
      pitch = -pitch;
      pitch_rate = -pitch_rate;
    }
    pitch_ = pitch;
    pitch_rate_ = pitch_rate;
    have_imu_ = true;
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    linear_cmd_ = msg->linear.x;
    angular_cmd_ = msg->angular.z;
    last_cmd_time_ = now();
  }

  void on_timer()
  {
    if (!have_imu_) {
      return;  // wait for the first IMU sample
    }

    // Drop stale velocity commands so the robot does not run away.
    if ((now() - last_cmd_time_).seconds() > cmd_timeout_) {
      linear_cmd_ = 0.0;
      angular_cmd_ = 0.0;
    }

    const double dt = 1.0 / loop_rate_;
    const auto cmd = controller_.update(pitch_, pitch_rate_, linear_cmd_, angular_cmd_, dt);

    std_msgs::msg::Float64MultiArray wheel_msg;
    wheel_msg.data = {cmd.left_effort, cmd.right_effort};
    wheel_pub_->publish(wheel_msg);

    sbr_msgs::msg::BalanceState state;
    state.header.stamp = now();
    state.header.frame_id = "base_link";
    state.pitch = pitch_;
    state.pitch_rate = pitch_rate_;
    state.pitch_setpoint = cmd.pitch_setpoint;
    state.linear_command = linear_cmd_;
    state.angular_command = angular_cmd_;
    state.left_effort = cmd.left_effort;
    state.right_effort = cmd.right_effort;
    state.balancing = cmd.balancing;
    state_pub_->publish(state);
  }

  BalanceController controller_;
  double loop_rate_{200.0};
  double cmd_timeout_{0.5};
  bool invert_pitch_{false};

  double pitch_{0.0};
  double pitch_rate_{0.0};
  double linear_cmd_{0.0};
  double angular_cmd_{0.0};
  bool have_imu_{false};
  rclcpp::Time last_cmd_time_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
  rclcpp::Publisher<sbr_msgs::msg::BalanceState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sbr_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sbr_control::BalanceControllerNode>());
  rclcpp::shutdown();
  return 0;
}
