#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "uav_px4_control/action/go_to_px4_position.hpp"

using namespace std::chrono_literals;

class Px4ControlActionNode : public rclcpp::Node
{
public:
  using GoToPx4Position = uav_px4_control::action::GoToPx4Position;
  using GoalHandleGoToPx4Position =
    rclcpp_action::ServerGoalHandle<GoToPx4Position>;

  Px4ControlActionNode() : Node("px4_control_action_node")
  {
    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);

    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);

    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();
    
    odom_sub_ = 
      create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        qos,
        std::bind(&Px4ControlActionNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_sub_ =
      create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&Px4ControlActionNode::cmdVelCallback, this, std::placeholders::_1));

    action_server_ =
      rclcpp_action::create_server<GoToPx4Position>(
        this,
        "go_to_px4_position",
        std::bind(&Px4ControlActionNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Px4ControlActionNode::handleCancel, this, std::placeholders::_1),
        std::bind(&Px4ControlActionNode::handleAccepted, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      100ms,
      std::bind(&Px4ControlActionNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "PX4 control action node started");
  }

private:
  enum class ControlMode
  {
    POSITION,
    VELOCITY
  };

  void timerCallback()
  {
    publishOffboardControlMode();

    if (control_mode_ == ControlMode::POSITION)
    {
      publishPositionSetpoint();
    }
    else
    {
      publishVelocitySetpoint();
    }

    if (setpoint_counter_ == 10)
    {
      setOffboardMode();
      arm();
    }

    if (setpoint_counter_ < 11)
    {
      setpoint_counter_++;
    }
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    control_mode_ = ControlMode::VELOCITY;

    // ROS convention:
    // linear.z positive = up
    //
    // PX4 NED convention:
    // velocity z negative = up
    velocity_x_ = msg->linear.x;
    velocity_y_ = msg->linear.y;
    velocity_z_ = -msg->linear.z;

    yaw_rate_ = msg->angular.z;

    last_cmd_vel_time_ = now();
  }

  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    current_x_ = msg->position[0];
    current_y_ = msg->position[1];
    current_z_ = msg->position[2];  // NED: negative means above origin
    odom_received_ = true;
  }

  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = nowMicros();

    if (control_mode_ == ControlMode::POSITION)
    {
      msg.position = true;
      msg.velocity = false;
    }
    else
    {
      msg.position = false;
      msg.velocity = true;
    }

    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_pub_->publish(msg);
  }

  void publishPositionSetpoint()
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = nowMicros();

    // Input goal uses ROS-style z:
    // z = +2 means 2 meters up.
    //
    // PX4 uses NED:
    // z = -2 means 2 meters up.
    msg.position = {
      static_cast<float>(target_x_),
      static_cast<float>(target_y_),
      static_cast<float>(-target_z_)
    };

    msg.yaw = static_cast<float>(target_yaw_);

    trajectory_setpoint_pub_->publish(msg);
  }

  void publishVelocitySetpoint()
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = nowMicros();

    // Use NaN for unused position fields.
    msg.position = {
      NAN,
      NAN,
      NAN
    };

    msg.velocity = {
      static_cast<float>(velocity_x_),
      static_cast<float>(velocity_y_),
      static_cast<float>(velocity_z_)
    };

    msg.yawspeed = static_cast<float>(yaw_rate_);

    trajectory_setpoint_pub_->publish(msg);
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GoToPx4Position::Goal> goal)
  {
    RCLCPP_INFO(
      get_logger(),
      "Received PX4 position goal: x=%.2f y=%.2f z=%.2f yaw=%.2f",
      goal->x,
      goal->y,
      goal->z,
      goal->yaw);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleGoToPx4Position>)
  {
    RCLCPP_WARN(get_logger(), "PX4 position goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(
    const std::shared_ptr<GoalHandleGoToPx4Position> goal_handle)
  {
    std::thread{
      std::bind(&Px4ControlActionNode::executeAction, this, std::placeholders::_1),
      goal_handle
    }.detach();
  }

  void executeAction(
    const std::shared_ptr<GoalHandleGoToPx4Position> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    target_x_ = goal->x;
    target_y_ = goal->y;
    target_z_ = goal->z;
    target_yaw_ = goal->yaw;

    control_mode_ = ControlMode::POSITION;

    auto feedback = std::make_shared<GoToPx4Position::Feedback>();
    auto result = std::make_shared<GoToPx4Position::Result>();

    rclcpp::Rate rate(10.0);

    const double position_tolerance = 0.25;

    while (rclcpp::ok())
    {
      if (goal_handle->is_canceling())
      {
        result->success = false;
        result->message = "Goal canceled";
        goal_handle->canceled(result);
        return;
      }

      if (!odom_received_)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Waiting for PX4 odometry...");
        rate.sleep();
        continue;
      }

      double target_z_ned = -target_z_;

      double dx = target_x_ - current_x_;
      double dy = target_y_ - current_y_;
      double dz = target_z_ned - current_z_;

      double distance_error = std::sqrt(dx * dx + dy * dy + dz * dz);

      feedback->current_x = current_x_;
      feedback->current_y = current_y_;
      feedback->current_z = -current_z_;  // Convert back to ROS-style altitude
      feedback->distance_error = distance_error;

      goal_handle->publish_feedback(feedback);

      if (distance_error < position_tolerance)
      {
        result->success = true;
        result->message = "PX4 position reached";
        goal_handle->succeed(result);

        RCLCPP_INFO(get_logger(), "PX4 position goal reached");
        return;
      }

      rate.sleep();
    }

    result->success = false;
    result->message = "ROS shutdown";
    goal_handle->abort(result);
  }

  void arm()
  {
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0,
      0.0);

    RCLCPP_INFO(get_logger(), "Arm command sent");
  }

  void setOffboardMode()
  {
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0,
      6.0);

    RCLCPP_INFO(get_logger(), "Offboard mode command sent");
  }

  void publishVehicleCommand(
    uint16_t command,
    float param1,
    float param2)
  {
    px4_msgs::msg::VehicleCommand msg{};

    msg.timestamp = nowMicros();

    msg.param1 = param1;
    msg.param2 = param2;

    msg.command = command;

    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;

    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
  }

  uint64_t nowMicros()
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp_action::Server<GoToPx4Position>::SharedPtr action_server_;

  rclcpp::TimerBase::SharedPtr timer_;

  ControlMode control_mode_{ControlMode::POSITION};

  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{2.0};
  double target_yaw_{0.0};

  double velocity_x_{0.0};
  double velocity_y_{0.0};
  double velocity_z_{0.0};
  double yaw_rate_{0.0};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};

  bool odom_received_{false};

  int setpoint_counter_{0};

  rclcpp::Time last_cmd_vel_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4ControlActionNode>());
  rclcpp::shutdown();
  return 0;
}