#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class Px4OdometryToTfNode : public rclcpp::Node
{
public:
  Px4OdometryToTfNode() : Node("px4_odometry_to_tf_node")
  {
    this->declare_parameter<std::string>("parent_frame", "odom");
    this->declare_parameter<std::string>("child_frame", "base_link");

    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_  = this->get_parameter("child_frame" ).as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();
    
    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      qos,
      std::bind(&Px4OdometryToTfNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "PX4 odometry to TF node started: %s -> %s",
      parent_frame_.c_str(),
      child_frame_.c_str());
  }

private:
  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;

    /*
      PX4 uses NED:
        x = North
        y = East
        z = Down

      ROS usually uses ENU:
        x = East
        y = North
        z = Up

      Conversion:
        ROS x =  PX4 y
        ROS y =  PX4 x
        ROS z = -PX4 z
    */

    transform.transform.translation.x =  msg->position[1];
    transform.transform.translation.y =  msg->position[0];
    transform.transform.translation.z = -msg->position[2];

    /*
      Basic quaternion conversion from PX4 NED to ROS ENU.

      PX4 attitude quaternion is usually given as:
        q = [w, x, y, z]

      ROS geometry_msgs quaternion:
        x, y, z, w

      Approximate NED-to-ENU conversion:
        q_enu.x =  q_ned.y
        q_enu.y =  q_ned.x
        q_enu.z = -q_ned.z
        q_enu.w =  q_ned.w
    */

    transform.transform.rotation.x =  msg->q[2];
    transform.transform.rotation.y =  msg->q[1];
    transform.transform.rotation.z = -msg->q[3];
    transform.transform.rotation.w =  msg->q[0];

    tf_broadcaster_->sendTransform(transform);

  }

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string parent_frame_;
  std::string child_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4OdometryToTfNode>());
  rclcpp::shutdown();
  return 0;
}