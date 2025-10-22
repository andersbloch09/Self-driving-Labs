#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>


#include <vector>
#include <cmath>



static const double PLANNING_TIME_S = 25.0;
static const double MAX_VELOCITY_SCALE = 0.1;  // Even slower - 1% of max velocity
static const double MAX_ACCELERATION_SCALE = 0.1;  // Even slower - 1% of max acceleration
static const unsigned int PLANNING_ATTEMPTS = 10;
static const double GOAL_TOLERANCE = 1e-3;
static const std::string PLANNING_GROUP = "panda_arm";
static const std::string base_link = "panda_link0";
static const std::string tcp_frame = "panda_hand_tcp";

std::vector<double> poseToList(const geometry_msgs::msg::Pose& pose) {
  return {
    pose.position.x,
    pose.position.y,
    pose.position.z,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w
  };
}

bool allClose(const std::vector<double>& goal, const std::vector<double>& actual, double tolerance) {
    if (goal.size() != actual.size()) return false;
    for (size_t i = 0; i < goal.size(); ++i) {
        if (std::fabs(actual[i] - goal[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

bool allClose(const geometry_msgs::msg::Pose& goal, const geometry_msgs::msg::Pose& actual, double tolerance) {
    std::vector<double> g = poseToList(goal);
    std::vector<double> a = poseToList(actual);

    double dx = g[0] - a[0];
    double dy = g[1] - a[1];
    double dz = g[2] - a[2];
    double d = std::sqrt(dx * dx + dy * dy + dz * dz);

    double cos_phi_half = std::fabs(g[3] * a[3] + g[4] * a[4] + g[5] * a[5] + g[6] * a[6]);

    return d <= tolerance && cos_phi_half >= std::cos(tolerance / 2.0);
}

bool allClose(const geometry_msgs::msg::PoseStamped& goal, const geometry_msgs::msg::PoseStamped& actual, double tolerance) {
    return allClose(goal.pose, actual.pose, tolerance);
}


class Manipulator
{
public:
  Manipulator()
  : node_(std::make_shared<rclcpp::Node>("manipulator")),
    logger_(node_->get_logger()),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
    RCLCPP_INFO(logger_, "🚀  Manipulator initialized");
    
    // Set up executor (THIS IS THE KEY!)
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });
    
    // Give executor time to start
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Create MoveGroupInterface with shared pointer like move_robot_server
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
    
    // Configure MoveIt exactly like move_robot_server
    move_group_->setPoseReferenceFrame(base_link);
    move_group_->setPlanningTime(PLANNING_TIME_S);
    move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
    move_group_->setGoalTolerance(GOAL_TOLERANCE);
    move_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALE);
    move_group_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALE);
    move_group_->setPlanningPipelineId("ompl");
    move_group_->setEndEffectorLink(tcp_frame);
    
    RCLCPP_INFO(logger_, "End-effector link set to: %s", move_group_->getEndEffectorLink().c_str());
    
    // Wait a bit more for everything to be ready
    RCLCPP_INFO(logger_, "⏱️  Waiting for MoveIt to be ready...");
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    // Now getCurrentPose() should work like in move_robot_server!
    RCLCPP_INFO(logger_, "✅ Getting current pose...");
    auto current_pose = move_group_->getCurrentPose();
    printPose(current_pose);

    auto joints = move_group_->getCurrentJointValues();
    RCLCPP_INFO(logger_, "Current Joint Values:");
    for (size_t i = 0; i < joints.size(); ++i) {
      RCLCPP_INFO(logger_, "  Joint %zu: %.3f", i, joints[i]);
    }
    
    // Do a  movement
    RCLCPP_INFO(logger_, "🤖 Moving robot...");
    moveRelativeToFrame("panda_hand_tcp", {0.1, 0.0, 0.0});  // Move 10cm in X direction
    rclcpp::sleep_for(std::chrono::seconds(1));
    moveRelativeToFrame("panda_hand_tcp", {0.0, 0.0, 0.1});

    // Get pose after movement
    auto final_pose = move_group_->getCurrentPose();
    printPose(final_pose);
    
    RCLCPP_INFO(logger_, "✅  manipulation completed!");
  }
  
  ~Manipulator() {
    // Clean up executor thread
    if (executor_thread_.joinable()) {
      executor_->cancel();
      executor_thread_.join();
    }
  }

private:
  // Print pose helper (like in move_robot_server)
  void printPose(const geometry_msgs::msg::PoseStamped& pose) {
    RCLCPP_INFO(logger_, "Current Pose:");
    RCLCPP_INFO(logger_, "  Position: [%.3f, %.3f, %.3f]", 
               pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    RCLCPP_INFO(logger_, "  Orientation: [%.3f, %.3f, %.3f, %.3f]",
               pose.pose.orientation.x, pose.pose.orientation.y, 
               pose.pose.orientation.z, pose.pose.orientation.w);
  }
  
  std::pair<bool, bool> moveRelativeToFrame(
    const std::string& frame_id,
    const std::vector<double>& pos = {0, 0, 0, 0, 0, 0}) 
  {
    RCLCPP_INFO(logger_, "Moving relative to frame %s: [%.3f, %.3f, %.3f]", 
               frame_id.c_str(), pos[0], pos[1], pos[2]);

    tf2::Quaternion q;
    q.setRPY(pos[3], pos[4], pos[5]);

    geometry_msgs::msg::PoseStamped pose_goal;
    pose_goal.header.frame_id = frame_id;
    pose_goal.pose.position.x = pos[0];
    pose_goal.pose.position.y = pos[1];
    pose_goal.pose.position.z = pos[2];
    pose_goal.pose.orientation = tf2::toMsg(q);

    std::string planning_frame = move_group_->getPlanningFrame();
    geometry_msgs::msg::PoseStamped transformed_pose;

    try {
      transformed_pose = tf_buffer_->transform(pose_goal, planning_frame, tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Transform failed: %s", ex.what());
      return {false, false};
    }

    move_group_->setPoseTarget(transformed_pose);
    
    // Use plan() and execute() instead of move() for better control
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool plan_success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    bool execute_success = false;
    if (plan_success) {
      RCLCPP_INFO(logger_, "Executing movement at %d%% velocity...", 
                 static_cast<int>(MAX_VELOCITY_SCALE * 100));
      auto result = move_group_->execute(my_plan);
      execute_success = (result == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
      RCLCPP_ERROR(logger_, "Planning failed!");
    }

    move_group_->stop();
    move_group_->clearPoseTargets();

    geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
    bool close = allClose(transformed_pose.pose, current_pose.pose, 0.01);

    return {close, execute_success};
  }


  // Member variables 
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Wait for system to be ready
  rclcpp::sleep_for(std::chrono::seconds(5));
  
  {
    Manipulator manipulator;
    // Keep the node alive for a bit to see the results
    rclcpp::sleep_for(std::chrono::seconds(2));
  }
  
  rclcpp::shutdown();
  return 0;
}




