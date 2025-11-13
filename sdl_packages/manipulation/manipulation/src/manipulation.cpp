#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/msg/collision_object.hpp>


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <btcpp_ros2_interfaces/action/cube_visual_calibration.hpp>
#include <aruco_interfaces/srv/aruco_detect.hpp>
#include <franka_msgs/srv/error_recovery.hpp>

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
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "hand");

    // =====================================================
    // Add static environment (base and camera) to the scene
    // =====================================================
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    // ===============================================
    // Create grasp frame from known orientation
    // ===============================================
    std::vector<double> orientation = {1.00, 0.00, 0.00, 0.00};

    // Convert quaternion → RPY
    tf2::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Define position
    std::vector<double> translation = {0.538, -0.024, 0.108};

    // Publish the static frame using your new method
    publishStaticFrame(
      "panda_link0",          // parent frame
      "mir_storage_lookout",          // new child frame name
      translation,            // translation in meters
      {roll, pitch, yaw}      // rotation in radians
    );

    // ===============================================
    // initialize Aruco detection client
    // ===============================================
    aruco_client_ = node_->create_client<aruco_interfaces::srv::ArucoDetect>("aruco_detect");

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

    // Get pose after movement
    auto final_pose = move_group_->getCurrentPose();
    printPose(final_pose);
    
    RCLCPP_INFO(logger_, "manipulation completed!");
  }
  
  ~Manipulator() {
    // Clean up executor thread
    if (executor_thread_.joinable()) {
      //executor_->cancel();
      executor_thread_.join();
    }
  }

// getter methods
rclcpp::Node::SharedPtr getNode() const { return node_; }

rclcpp::Client<aruco_interfaces::srv::ArucoDetect>::SharedPtr getArucoClient() const { 
  return aruco_client_; 
}
bool recover() { return recoverFromError(); }

void addCollisionMesh(
    const std::string& object_id,
    const std::string& mesh_path,
    const geometry_msgs::msg::Pose& pose,
    const std::string& frame_id = "panda_link0")
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject object;
  object.id = object_id;
  object.header.frame_id = frame_id;

  shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);
  if (!mesh)
  {
    RCLCPP_ERROR(logger_, "❌ Failed to load mesh: %s", mesh_path.c_str());
    return;
  }

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(mesh, mesh_msg);
  shape_msgs::msg::Mesh mesh_converted = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  object.meshes.push_back(mesh_converted);
  object.mesh_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_interface.applyCollisionObjects({object});

  RCLCPP_INFO(logger_, "✅ Added object [%s] to planning scene", object_id.c_str());
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

  std::pair<bool, bool> moveToJointPosition(
    const std::vector<double>& target_joints)
{
  const size_t dof = move_group_->getVariableCount();

  if (target_joints.size() != dof)
  {
    RCLCPP_ERROR(logger_,
                 "moveToJointPosition: Expected %zu joints, got %zu",
                 dof, target_joints.size());
    return {false, false};
  }

  // Log goal
  RCLCPP_INFO(logger_, "Moving to absolute joint position:");
  for (size_t i = 0; i < dof; i++)
    RCLCPP_INFO(logger_, "  q[%zu] = %.3f", i, target_joints[i]);

  // Set joint-space goal
  move_group_->setJointValueTarget(target_joints);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success =
      (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  bool exec_success = false;

  if (plan_success)
  {
    RCLCPP_INFO(logger_,
        "Executing joint trajectory at %d%% velocity...",
        static_cast<int>(MAX_VELOCITY_SCALE * 100));

    auto result = move_group_->execute(plan);
    exec_success = (result == moveit::core::MoveItErrorCode::SUCCESS);
  }
  else
  {
    RCLCPP_ERROR(logger_, "Joint trajectory planning FAILED.");
  }

  move_group_->stop();

  // Check closeness of final position vs goal
  std::vector<double> final_joints = move_group_->getCurrentJointValues();
  bool close = true;
  const double tolerance = 0.01;  // rad

  for (size_t i = 0; i < dof; i++)
  {
    if (std::fabs(final_joints[i] - target_joints[i]) > tolerance)
    {
      close = false;
      break;
    }
  }

  if (close)
    RCLCPP_INFO(logger_, "✅ Reached target joint position.");
  else
    RCLCPP_WARN(logger_,
       "⚠ Target joint position NOT reached within tolerance.");

  return {close, exec_success};
}

  bool planCartesianPath(
    const std::string& frame_name,
    const std::vector<double>& pos,
    bool avoid_collisions = true)
{
    // --- Waypoint and transform (same as before) ---
    geometry_msgs::msg::PoseStamped wpose;
    wpose.header.frame_id = frame_name;
    wpose.pose.position.x = pos[0];
    wpose.pose.position.y = pos[1];
    wpose.pose.position.z = pos[2];
    tf2::Quaternion q;
    q.setRPY(pos[3], pos[4], pos[5]);
    wpose.pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::PoseStamped transformed_pose;
    try {
        transformed_pose = tf_buffer_->transform(wpose, move_group_->getPlanningFrame(), tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(logger_, "Transform failed: %s", ex.what());
        return false;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(transformed_pose.pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.005;
    double jump_threshold = 0.0;

    double fraction = move_group_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);

    if (fraction < 0.99) {
        RCLCPP_WARN(logger_, "Cartesian path only %.2f%% complete", fraction * 100.0);
    } else {
        RCLCPP_INFO(logger_, "Cartesian path computed successfully (%.2f%%)", fraction * 100.0);
    }

    // --- Retiming the trajectory ---
    robot_trajectory::RobotTrajectory rt(move_group_->getCurrentState()->getRobotModel(), PLANNING_GROUP);
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    if (!time_param.computeTimeStamps(rt, MAX_VELOCITY_SCALE, MAX_ACCELERATION_SCALE)) {
        RCLCPP_WARN(logger_, "Failed to retime trajectory!");
    }

    // Update plan with retimed trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    rt.getRobotTrajectoryMsg(plan.trajectory_);

    // Execute at scaled velocity
    bool success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
        RCLCPP_ERROR(logger_, "Failed to execute Cartesian path!");
    }

    return success;
}

bool MoveGripper(double left_joint, double right_joint)
{
  std::vector<double> joints = {left_joint, right_joint};
  std::vector<std::string> joint_names = {"panda_finger_joint1", "panda_finger_joint2"};

  gripper_group_->setJointValueTarget(joint_names, joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (gripper_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(logger_, "Gripper move plan %s", success ? "succeeded" : "failed");

  if (success) {
    gripper_group_->execute(plan);
    return true;
  }
  return false;
}

void publishStaticFrame(
    const std::string& parent_frame,
    const std::string& child_frame,
    const std::vector<double>& translation = {0.0, 0.0, 0.0},
    const std::vector<double>& rpy = {0.0, 0.0, 0.0})
{
  if (!static_broadcaster_) {
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node_->get_clock()->now();
  t.header.frame_id = parent_frame;
  t.child_frame_id = child_frame;

  // Translation
  t.transform.translation.x = translation[0];
  t.transform.translation.y = translation[1];
  t.transform.translation.z = translation[2];

  // Rotation
  tf2::Quaternion q;
  q.setRPY(rpy[0], rpy[1], rpy[2]);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  static_broadcaster_->sendTransform(t);

  RCLCPP_INFO(
    logger_,
    "Published static TF [%s -> %s]  (xyz: [%.3f, %.3f, %.3f], rpy: [%.3f, %.3f, %.3f])",
    parent_frame.c_str(), child_frame.c_str(),
    translation[0], translation[1], translation[2],
    rpy[0], rpy[1], rpy[2]);
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


 bool recoverFromError() {
  if (!error_recovery_client_) {
    error_recovery_client_ = node_->create_client<franka_msgs::srv::ErrorRecovery>(
      "/panda_error_recovery_service_server/error_recovery"
    );
  }
  
  if (!error_recovery_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(logger_, "Error recovery service not available");
    return false;
  }
  
  auto request = std::make_shared<franka_msgs::srv::ErrorRecovery::Request>();
  auto future = error_recovery_client_->async_send_request(request);
  
  // Use wait_for instead of spin_until_future_complete
  auto status = future.wait_for(std::chrono::seconds(5));
  
  if (status == std::future_status::ready) {
    auto response = future.get();
    RCLCPP_INFO(logger_, "Error recovery completed");
    rclcpp::sleep_for(std::chrono::seconds(2));
    return true;
  } else {
    RCLCPP_ERROR(logger_, "Error recovery service call timed out");
    return false;
  }
}

  // Member variables
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  rclcpp::Client<franka_msgs::srv::ErrorRecovery>::SharedPtr error_recovery_client_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::thread executor_thread_;
  rclcpp::Client<aruco_interfaces::srv::ArucoDetect>::SharedPtr aruco_client_;
};


class ActionsManager
{
public:
  using GoHome = control_msgs::action::FollowJointTrajectory;
  using GoalHandleGoHome = rclcpp_action::ServerGoalHandle<GoHome>;

  using MoveBoxPosOnRobot = control_msgs::action::FollowJointTrajectory;
  using GoalHandleMoveBoxPosOnRobot = rclcpp_action::ServerGoalHandle<MoveBoxPosOnRobot>;

  using CameraCalibration = control_msgs::action::FollowJointTrajectory;
  using GoalHandleCalibration = rclcpp_action::ServerGoalHandle<CameraCalibration>;

  using CubeVisualCalibration = btcpp_ros2_interfaces::action::CubeVisualCalibration;
  using GoalHandleCubeVisualCalibration = rclcpp_action::ServerGoalHandle<CubeVisualCalibration>;

  explicit ActionsManager(std::shared_ptr<Manipulator> manip)
  : manip_(manip),
    node_(manip->getNode()),
    logger_(node_->get_logger())
  {
    using namespace std::placeholders;

    // ---- Action: move_in_square ----
    go_home_server_ = rclcpp_action::create_server<GoHome>(
      node_,
      "go_home",
      std::bind(&ActionsManager::handle_goal_go_home, this, _1, _2),
      std::bind(&ActionsManager::handle_cancel_go_home, this, _1),
      std::bind(&ActionsManager::handle_accepted_go_home, this, _1)
    );

    cube_visual_calibration_server_ = rclcpp_action::create_server<CubeVisualCalibration>(
      node_,
      "cube_visual_calibration",
      std::bind(&ActionsManager::handle_goal_cube_visual_calibration, this, _1, _2),
      std::bind(&ActionsManager::handle_cancel_cube_visual_calibration, this, _1),
      std::bind(&ActionsManager::handle_accepted_cube_visual_calibration, this, _1)
    );

    camera_calibration_server_  = rclcpp_action::create_server<CameraCalibration>(
      node_,
      "camera_calibration",
      std::bind(&ActionsManager::handle_goal_camera_calibration, this, _1, _2),
      std::bind(&ActionsManager::handle_cancel_camera_calibration, this, _1),
      std::bind(&ActionsManager::handle_accepted_camera_calibration, this, _1)
    );

    // ---- Action: move_box_pos_on_robot ----
    move_box_pos_on_robot_server_ = rclcpp_action::create_server<MoveBoxPosOnRobot>(
      node_,
      "move_box_pos_on_robot",
      std::bind(&ActionsManager::handle_goal_move_box_pos_on_robot, this, _1, _2),
      std::bind(&ActionsManager::handle_cancel_move_box_pos_on_robot, this, _1),
      std::bind(&ActionsManager::handle_accepted_move_box_pos_on_robot, this, _1)
    );

  }

private:
  // =====================================================
  //go_home ACTION
  // =====================================================
  rclcpp_action::GoalResponse handle_goal_go_home(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GoHome::Goal>)
  {
    RCLCPP_INFO(logger_, "Received goal for /go_home");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_go_home(
    const std::shared_ptr<GoalHandleGoHome>)
  {
    RCLCPP_INFO(logger_, "Cancel request received for /go_home");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_go_home(const std::shared_ptr<GoalHandleGoHome> goal_handle)
  {
    std::thread([this, goal_handle]() {
      auto result = std::make_shared<GoHome::Result>();
      bool success = go_home();
      result->error_code = success ? 0 : -1;
      result->error_string = success ? "Trajectory executed successfully" : "Trajectory failed";

      if (success)
        goal_handle->succeed(result);
      else
        goal_handle->abort(result);
    }).detach();
  }

  bool go_home()
  {

    manip_->recover();  

    manip_->moveToJointPosition(std::vector<double>{
      0.20375095067107887, -0.39273988735964127, -0.17276381184038808,
      -2.7220893394764865, -0.0844567528031363, 2.3430847805341086, 
      0.8822717887647267});
    return true;
  }

  // =====================================================
  // cube_visual_calibration ACTION
  // =====================================================
  rclcpp_action::GoalResponse handle_goal_cube_visual_calibration(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const CubeVisualCalibration::Goal> goal)
{
  RCLCPP_INFO(logger_, "Received goal for /cube_visual_calibration with side: %s", goal->side.c_str());
  
  // Optional: validate the side parameter
  if (goal->side != "left" && goal->side != "right") {
    RCLCPP_ERROR(logger_, "Invalid side parameter: %s. Must be 'left' or 'right'", goal->side.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel_cube_visual_calibration(
  const std::shared_ptr<GoalHandleCubeVisualCalibration>)
{
  RCLCPP_INFO(logger_, "Cancel request received for /cube_visual_calibration");
  return rclcpp_action::CancelResponse::ACCEPT;
} 

void handle_accepted_cube_visual_calibration(const std::shared_ptr<GoalHandleCubeVisualCalibration> goal_handle)
{
  std::thread([this, goal_handle]() {
    // Extract the 'side' parameter from the goal
    const auto goal = goal_handle->get_goal();
    std::string side = goal->side;
    
    auto result = std::make_shared<CubeVisualCalibration::Result>();
    bool success = cube_visual_calibration(side);  // Pass side to the function

    result->error_code = success ? 0 : -1;
    result->message = success 
      ? "Cube visual calibration completed successfully for " + side + " side"
      : "Cube visual calibration failed for " + side + " side";

    if (success)
      goal_handle->succeed(result);
    else
      goal_handle->abort(result);
  }).detach();
}

void handle_canceled_cube_visual_calibration(const std::shared_ptr<GoalHandleCubeVisualCalibration>)
{
  RCLCPP_INFO(logger_, "Cube visual calibration canceled");
}

  bool cube_visual_calibration(const std::string& side) {

    if (side == "right") {
      RCLCPP_INFO(logger_, "Executing cube visual calibration...");
      manip_->moveToJointPosition(std::vector<double>{
          1.5956115519456693, 0.8311388652199193, -2.50595094115274, 
          -2.1689134552938896, 0.2011045270697737, 1.893249041398366, 
          1.31055953314449
      });
    } else if (side == "left") {
      RCLCPP_INFO(logger_, "Executing cube visual calibration...");
      manip_->moveToJointPosition(std::vector<double>{
        0.3588204508785312, -0.4363771210260558, -0.3616325221749825,
        -1.900478438790212, 0.28568802635543683, 1.4648013339042663,
        -0.7218936765773428});}
      
    // Call aruco detection service using the getter
    auto request = std::make_shared<aruco_interfaces::srv::ArucoDetect::Request>();
    auto future = manip_->getArucoClient()->async_send_request(request);
    auto response = future.get(); 
    
    // Check if marker was detected
    if (response->id < 0) {
      RCLCPP_WARN(logger_, "No ArUco marker detected.");
      return false;
    }
    
    
    if (response->id != 0) {
      manip_->moveRelativeToFrame(
        "aruco_marker_" + std::to_string(response->id),
        {0, 0, -0.2, 0, 0, 0});}
    
    // Call service again for better frame
    future = manip_->getArucoClient()->async_send_request(request);
    response = future.get();  


    RCLCPP_INFO(logger_, "Detected ArUco marker ID: %d", response->id);
    return true;
  }


  // =====================================================
  // camera_calibration ACTION
  // =====================================================

  rclcpp_action::GoalResponse handle_goal_camera_calibration(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CameraCalibration::Goal>)
  {
    RCLCPP_INFO(logger_, "Received goal for /camera_calibration");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_camera_calibration(
    const std::shared_ptr<GoalHandleCalibration>)
  {
    RCLCPP_INFO(logger_, "Cancel request received for /camera_calibration");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_camera_calibration(const std::shared_ptr<GoalHandleCalibration> goal_handle)
  {
    std::thread([this, goal_handle]() {
      auto result = std::make_shared<CameraCalibration::Result>();
      bool success = calibrate_camera(); 

      result->error_code = success ? 0 : -1;
      result->error_string = success ? "Calibration completed successfully" : "Calibration failed";

      if (success)
        goal_handle->succeed(result);
      else
        goal_handle->abort(result);
    }).detach();
  }
  
  bool calibrate_camera() {
    RCLCPP_INFO(logger_, "Executing camera calibration...");

    
    std::vector<double> initial_joints = {0.040248013796111565, -0.6791726456293383, -0.1395924679635489,
                                        -2.5689465115484365, -0.14891476913616702, 1.7820160946846006,
                                        0.7973367972589201};
    manip_->moveToJointPosition(initial_joints);
    rclcpp::sleep_for(std::chrono::seconds(2));
    // Add more calibration steps as needed
    auto joint_target_list = std::vector<std::vector<double>>{
    {0.04782388471249942, -0.17336348758436476, -0.1201145298083623, -1.8330875176212245, -0.11871012623549507, 1.3905901284623374, 0.7395101646608424},

    {-0.15561781020316984, -0.41171883176232305, 0.3274474033543576, -2.5596163518458233, -0.30950417719413353, 1.78779767370224, 1.2496959300256438},

    {-0.18175793773040433, -1.524229304731938, -0.38794875635791226, -2.7649503838723164, -0.4340341348780526, 1.4808229488096138, 0.8714191941379836},

    {0.5457112110790452, -1.058295548321908, -0.974057254075288, -2.376992587319951, -0.6909174268972457, 1.4805916281373137, 0.7302634253524906},

  {1.2195870974746175, -0.5894056424544509, -1.5358369686227096, -1.9080860292357962, -0.550409801337454, 1.557894101301829, 0.6976626893482457},
  
  {0.934879883222396, -1.2257907931205985, -0.49502804495995506, -2.524067088989393, -0.7400466033510504, 1.3365063954989114, 1.3470015028533007},
  
  {0.9561815292040506, -1.6257404654988072, -0.32334264627823434, -2.5262677478121036, -0.7329317413552985, 0.8668012846163663, 1.3606218857566748},
  };


    for (const auto& joint_target : joint_target_list) {
      manip_->moveToJointPosition(joint_target);
      rclcpp::sleep_for(std::chrono::seconds(2));
    }

    manip_->moveToJointPosition(initial_joints);
    rclcpp::sleep_for(std::chrono::seconds(2));
    return true;
}

  // =====================================================
  // move_box_pos_on_robot ACTION
  // =====================================================
  rclcpp_action::GoalResponse handle_goal_move_box_pos_on_robot(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveBoxPosOnRobot::Goal>)
  {
    RCLCPP_INFO(logger_, "Received goal for /move_box_pos_on_robot");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_move_box_pos_on_robot(
    const std::shared_ptr<GoalHandleMoveBoxPosOnRobot>)
  {
    RCLCPP_INFO(logger_, "Cancel request received for /move_box_pos_on_robot");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_move_box_pos_on_robot(const std::shared_ptr<GoalHandleMoveBoxPosOnRobot> goal_handle)
  {
    std::thread([this, goal_handle]() {
      auto result = std::make_shared<MoveBoxPosOnRobot::Result>();
      bool success = move_box_pos_on_robot();

      result->error_code = success ? 0 : -1;
      result->error_string = success ? "Move completed successfully" : "Move failed";

      if (success)
        goal_handle->succeed(result);
      else
        goal_handle->abort(result);
    }).detach();
  }

  bool move_box_pos_on_robot()
  {
    RCLCPP_INFO(logger_, "📦 Executing move_box_pos_on_robot...");

    std::vector<double> orientation = {1.000, 0.004, 0.001, -0.001};

    // conversion to roll-pitch-yaw
    tf2::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    manip_->MoveGripper(0.04, 0.04); // open
    manip_->moveRelativeToFrame("panda_link0", {0.538, -0.024, 0.108, roll, pitch, yaw});
    manip_->planCartesianPath("panda_hand_tcp", {0.0, 0.0, 0.05, 0, 0, 0}); 
    manip_->MoveGripper(0.033, 0.033); // close
    manip_->planCartesianPath("panda_hand_tcp", {0.0, 0.0, -0.1, 0, 0, 0}); 
    manip_->planCartesianPath("panda_hand_tcp", {0.0, -0.117, 0, 0, 0, 0}); 
    manip_->planCartesianPath("panda_hand_tcp", {0.0, 0.0, 0.1, 0, 0, 0}); 
    manip_->MoveGripper(0.04, 0.04); // open
    manip_->planCartesianPath("panda_hand_tcp", {0.0, 0.0, -0.1, 0, 0, 0}); 
    manip_->moveRelativeToFrame("panda_link0", {0.538, -0.024, 0.108, roll, pitch, yaw});

    return true;
  }

  

  // =====================================================
  // Members
  // =====================================================
  std::shared_ptr<Manipulator> manip_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  rclcpp_action::Server<GoHome>::SharedPtr go_home_server_;
  rclcpp_action::Server<CubeVisualCalibration>::SharedPtr cube_visual_calibration_server_;
  rclcpp_action::Server<CameraCalibration>::SharedPtr camera_calibration_server_;
  rclcpp_action::Server<MoveBoxPosOnRobot>::SharedPtr move_box_pos_on_robot_server_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Wait for system to be ready
  rclcpp::sleep_for(std::chrono::seconds(5));

  
  // Initialize manipulator interface
  auto manipulator = std::make_shared<Manipulator>();


  // Create ActionsManager (action server node)
  auto actions_manager = std::make_shared<ActionsManager>(manipulator);
  

  auto spin_blocker = std::make_shared<rclcpp::Node>("spin_blocker");
  rclcpp::spin(spin_blocker);


  RCLCPP_INFO(manipulator->getNode()->get_logger(), "Shutting down cleanly...");

  // --- stop the manipulator's executor thread properly ---
  manipulator.reset();  // triggers Manipulator destructor → joins executor thread

  rclcpp::shutdown();
  return 0;
}



