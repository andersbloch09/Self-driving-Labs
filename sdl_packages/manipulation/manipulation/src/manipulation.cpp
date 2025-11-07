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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

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
    
    RCLCPP_INFO(logger_, "✅  manipulation completed!");
  }
  
  ~Manipulator() {
    // Clean up executor thread
    if (executor_thread_.joinable()) {
      //executor_->cancel();
      executor_thread_.join();
    }
  }

rclcpp::Node::SharedPtr getNode() const { return node_; }


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

  // Member variables
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};


class ActionsManager
{
public:
  using MoveInSquare = control_msgs::action::FollowJointTrajectory;
  using GoalHandleMoveInSquare = rclcpp_action::ServerGoalHandle<MoveInSquare>;

  using MoveBoxPosOnRobot = control_msgs::action::FollowJointTrajectory;
  using GoalHandleMoveBoxPosOnRobot = rclcpp_action::ServerGoalHandle<MoveBoxPosOnRobot>;

  using CameraCalibration = control_msgs::action::FollowJointTrajectory;
  using GoalHandleCalibration = rclcpp_action::ServerGoalHandle<CameraCalibration>;

  explicit ActionsManager(std::shared_ptr<Manipulator> manip)
  : manip_(manip),
    node_(manip->getNode()),
    logger_(node_->get_logger())
  {
    using namespace std::placeholders;

    // ---- Action: move_in_square ----
    move_in_square_server_ = rclcpp_action::create_server<MoveInSquare>(
      node_,
      "move_in_square",
      std::bind(&ActionsManager::handle_goal_move_in_square, this, _1, _2),
      std::bind(&ActionsManager::handle_cancel_move_in_square, this, _1),
      std::bind(&ActionsManager::handle_accepted_move_in_square, this, _1)
    );

    camera_calibration = rclcpp_action::create_server<CameraCalibration>(
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

    RCLCPP_INFO(logger_, "✅ ActionsManager ready — actions: [/move_in_square], [/pick_object]");
  }

private:
  // =====================================================
  //move_in_square ACTION
  // =====================================================
  rclcpp_action::GoalResponse handle_goal_move_in_square(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveInSquare::Goal>)
  {
    RCLCPP_INFO(logger_, "Received goal for /move_in_square");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_move_in_square(
    const std::shared_ptr<GoalHandleMoveInSquare>)
  {
    RCLCPP_INFO(logger_, "Cancel request received for /move_in_square");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_move_in_square(const std::shared_ptr<GoalHandleMoveInSquare> goal_handle)
  {
    std::thread([this, goal_handle]() {
      auto result = std::make_shared<MoveInSquare::Result>();
      bool success = move_in_square();

      result->error_code = success ? 0 : -1;
      result->error_string = success ? "Trajectory executed successfully" : "Trajectory failed";

      if (success)
        goal_handle->succeed(result);
      else
        goal_handle->abort(result);
    }).detach();
  }

  bool move_in_square()
  {
    RCLCPP_INFO(logger_, "🤖 Executing move_in_square...");
    for (int i = 0; i < 2; ++i)
    {
      manip_->planCartesianPath("panda_hand_tcp", { 0.1,  0.0, 0.0, 0, 0, 0 });
      manip_->MoveGripper(0.1, 0.1); // open
      manip_->planCartesianPath("panda_hand_tcp", { 0.0,  0.1, 0.0, 0, 0, 0 });
      manip_->MoveGripper(0.0, 0.0); // close
      manip_->planCartesianPath("panda_hand_tcp", {-0.1,  0.0, 0.0, 0, 0, 0 });
      manip_->MoveGripper(0.1, 0.1); // open
      manip_->planCartesianPath("panda_hand_tcp", { 0.0, -0.1, 0.0, 0, 0, 0 });
      manip_->MoveGripper(0.0, 0.0); // close
    }
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
  bool calibrate_camera()
  {
    RCLCPP_INFO(logger_, "Executing camera calibration...");
    
    


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
  rclcpp_action::Server<MoveInSquare>::SharedPtr move_in_square_server_;
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


  RCLCPP_INFO(manipulator->getNode()->get_logger(), "🧹 Shutting down cleanly...");

  // --- stop the manipulator's executor thread properly ---
  manipulator.reset();  // triggers Manipulator destructor → joins executor thread

  rclcpp::shutdown();
  return 0;
}


