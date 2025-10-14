#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RelativePlanner
{
public:
  RelativePlanner()
  : node_(std::make_shared<rclcpp::Node>("relative_planner",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
    logger_(node_->get_logger()),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_),
    move_group_(node_, "panda_arm")  // Replace with your planning group
  {
    RCLCPP_INFO(logger_, "RelativePlanner initialized");
  }
  

  void run()
  {
    moveit::planning_interface::MoveGroupInterface gripper_group(node_, "hand");

    // Close the gripper
    gripper_group.setNamedTarget("close");
    gripper_group.move();

    // Wait a bit
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Open the gripper
    gripper_group.setNamedTarget("open");
    gripper_group.move();

    
    // Step 1: Move to a pose relative to a frame (e.g. end_effector_link)
    geometry_msgs::msg::PoseStamped relative_pose;
    relative_pose.header.frame_id = "panda_hand_tcp";  // Or any valid frame
    relative_pose.pose.orientation.w = 1.0;
    relative_pose.pose.position.x = 0.0;  // Move 10cm forward
    relative_pose.pose.position.y = 0.1;
    relative_pose.pose.position.z = 0.0;
    
    tf2::Quaternion rotation;
    rotation.setRPY(0, 0, 45);  // 45 degrees around Z
    geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(rotation);
    relative_pose.pose.orientation = q_msg;

    geometry_msgs::msg::PoseStamped target_pose;
    try {
      target_pose = tf_buffer_.transform(relative_pose, move_group_.getPlanningFrame(), tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(logger_, "Transform failed: %s", ex.what());
      return;
    }

    RCLCPP_INFO(logger_, "Moving to pose relative to planning frame: %s", move_group_.getPlanningFrame().c_str());
    move_group_.setPoseTarget(target_pose.pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_.plan(plan));
    if (success) {
      RCLCPP_INFO(logger_, "Plan successful. Executing...");
      move_group_.execute(plan);
    } else {
      RCLCPP_ERROR(logger_, "Planning failed.");
    }

    rclcpp::sleep_for(std::chrono::seconds(2));
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  moveit::planning_interface::MoveGroupInterface move_group_;
};

int main(int argc, char * argv[])
{
  rclcpp::sleep_for(std::chrono::seconds(5));
  rclcpp::init(argc, argv);
  RelativePlanner planner;
  planner.run();
  rclcpp::shutdown();
  return 0;
}
