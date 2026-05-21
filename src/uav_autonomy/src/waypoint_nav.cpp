#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "uav_autonomy/action/go_to_waypoint.hpp"

class WaypointActionServer : public rclcpp::Node
{
public:
  using GoToWaypoint = uav_autonomy::action::GoToWaypoint;
  using GoalHandleGoToWaypoint = rclcpp_action::ServerGoalHandle<GoToWaypoint>;

  WaypointActionServer() : Node("waypoint_action_server")
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&WaypointActionServer::odomCallback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<GoToWaypoint>(
      this,
      "go_to_waypoint",
      std::bind(&WaypointActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WaypointActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&WaypointActionServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waypoint action server started");
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GoToWaypoint::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received waypoint request: x=%.2f y=%.2f yaw=%.2f",
      goal->target_x,
      goal->target_y,
      goal->target_yaw);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleGoToWaypoint>)
  {
    RCLCPP_WARN(this->get_logger(), "Waypoint goal canceled");
    stopRobot();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleGoToWaypoint> goal_handle)
  {
    std::thread{
      std::bind(&WaypointActionServer::execute, this, std::placeholders::_1),
      goal_handle
    }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToWaypoint> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<GoToWaypoint::Feedback>();
    auto result = std::make_shared<GoToWaypoint::Result>();

    rclcpp::Rate rate(20.0);

    const double position_tolerance = 0.10;
    const double yaw_tolerance = 0.10;

    const double kp_linear = 0.6;
    const double kp_angular = 1.5;
    const double kp_final_yaw = 1.5;

    const double max_linear_speed = 0.5;
    const double max_angular_speed = 0.8;

    bool linear_reach = false;

    while (rclcpp::ok())
    {
      if (goal_handle->is_canceling())
      {
        stopRobot();
        result->success = false;
        result->message = "Goal canceled";
        goal_handle->canceled(result);
        return;
      }

      if (!odom_received_)
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Waiting for /odom...");
        rate.sleep();
        continue;
      }

      double dx = goal->target_x - current_x_;
      double dy = goal->target_y - current_y_;

      double distance_error = std::sqrt(dx * dx + dy * dy);
      double target_heading = std::atan2(dy, dx);
      double heading_error = normalizeAngle(target_heading - current_yaw_);

      geometry_msgs::msg::Twist cmd;

      if (distance_error > position_tolerance && linear_reach == false)
      {
        cmd.angular.z = clamp(kp_angular * heading_error, -max_angular_speed, max_angular_speed);

        if (std::abs(heading_error) < 0.5)
        {
          cmd.linear.x = clamp(kp_linear * distance_error, 0.0, max_linear_speed);
        }
        else
        {
          cmd.linear.x = 0.0;
        }
      }
      else
      {
        linear_reach = true;
        double yaw_error = normalizeAngle(goal->target_yaw - current_yaw_);

        if (std::abs(yaw_error) > yaw_tolerance)
        {
          cmd.angular.z = clamp(kp_final_yaw * yaw_error, -0.5, 0.5);
          cmd.linear.x = 0.0;
        }
        else
        {
          stopRobot();

          result->success = true;
          result->message = "Waypoint reached";
          goal_handle->succeed(result);

          RCLCPP_INFO(this->get_logger(), "Waypoint reached successfully");
          return;
        }
      }

      cmd_pub_->publish(cmd);

      feedback->current_x = current_x_;
      feedback->current_y = current_y_;
      feedback->distance_error = distance_error;
      feedback->yaw_error = normalizeAngle(goal->target_yaw - current_yaw_);

      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    stopRobot();

    result->success = false;
    result->message = "ROS shutdown before reaching waypoint";
    goal_handle->abort(result);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);
    odom_received_ = true;
  }

  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  double clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(value, max_value));
  }

  void stopRobot()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<GoToWaypoint>::SharedPtr action_server_;

  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};

  bool odom_received_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointActionServer>());
  rclcpp::shutdown();
  return 0;
}