#include "franka_hardware/sim/robot_sim.hpp"
#include <cstring>
#include <Eigen/Dense>

namespace franka_hardware{ 

bool RobotSim::populateIndices(){
  const mjModel* m_ = franka_hardware_model_->getMjModel();
  // body index loop
  for(int n = 0; n < kNumberOfJoints+2; n++){
    std::string link_name = robot_name_ + "_link" + std::to_string(n);
    int body_id = mj_name2id(m_, mjOBJ_BODY, link_name.c_str());
    if(body_id != -1){
      link_indices_[n] = body_id;
    }
    else{
      std::cerr << "No mjOBJ_BODY called " << link_name << " found" << std::endl;
      return false;
    }
  }

  // joint site loop
  for(int n = 0; n < kNumberOfJoints+2; n++){
    std::string site_name;
    if(n < kNumberOfJoints){
      site_name = robot_name_ + "_joint" + std::to_string(n+1) + "_site";
    }
    else if (n == kNumberOfJoints){
      site_name = robot_name_ + "_flange_site";
    }
    else{
      site_name = robot_name_ + "_ee_site";
    }

    int site_id = mj_name2id(m_, mjOBJ_SITE, site_name.c_str());
    if(site_id != -1){
      joint_site_indices_[n] = site_id;
    }
    else{
      std::cerr << "No mjOBJ_SITE called " << site_name << " found" << std::endl;
      return false;
    }
  }

  // joint index loop
  for(int n = 0; n < kNumberOfJoints; n++){
    std::string joint_name = robot_name_ + "_joint" + std::to_string(n+1);
    int joint_id = mj_name2id(m_, mjOBJ_JOINT, joint_name.c_str());
    if(joint_id != -1){
      joint_indices_[n] = joint_id;
      joint_qvel_indices_[n] = m_->jnt_dofadr[joint_id]; // for qvel (nv x 1) indexing
      joint_qpos_indices_[n] = m_->jnt_qposadr[joint_id]; // for qpos (nq x 1) indexing
    }
    else{
      std::cerr << "No mjOBJ_JOINT called " << joint_name << " found" << std::endl;
      return false;
    }
  }

  // Find the actuator indices for each of the robot's torque and velocity actuators
  for(int n = 0; n < kNumberOfJoints; n++){
    std::string joint_name = robot_name_ + "_joint" + std::to_string(n+1);
    std::string trq_name = robot_name_ + "_act_trq" + std::to_string(n+1);
    std::string vel_name = robot_name_ + "_act_vel" + std::to_string(n+1);
    std::string pos_name = robot_name_ + "_act_pos" + std::to_string(n+1);
    int trq_id = mj_name2id(m_, mjOBJ_ACTUATOR, trq_name.c_str());
    int vel_id = mj_name2id(m_, mjOBJ_ACTUATOR, vel_name.c_str());
    int pos_id = mj_name2id(m_, mjOBJ_ACTUATOR, pos_name.c_str());
    if(trq_id != -1){
      act_trq_indices_[n] = trq_id;
    }
    else{
      std::cerr << "No torque mjOBJ_ACTUATOR " << trq_name << " found for " << joint_name<< std::endl; 
      return false;
    }
    if(vel_id != -1){
      act_vel_indices_[n] = vel_id;
    }
    else{
      std::cerr << "No velocity mjOBJ_ACTUATOR " << vel_name << " found for " << joint_name << std::endl; 
      return false;
    }
    if(pos_id != -1){
      act_pos_indices_[n] = pos_id;
    }
    else{
      std::cerr << "No position mjOBJ_ACTUATOR " << pos_name << " found for " << joint_name << std::endl; 
      return false;
    }
  }

  // Gripper
  if(has_gripper_){
    // gripper joint loop
    for(int n = 0; n < 2; n++){
      std::string joint_name = robot_name_ + "_finger_joint" + std::to_string(n+1); //left, then right
      int joint_id = mj_name2id(m_, mjOBJ_JOINT, joint_name.c_str());
      if(joint_id != -1){
        gripper_joint_indices_[n] = joint_id;
        gripper_joint_qvel_indices_[n] = m_->jnt_dofadr[joint_id]; // for qvel (nv x 1) indexing
        gripper_joint_qpos_indices_[n] = m_->jnt_qposadr[joint_id]; // for qpos (nq x 1) indexing
      }
      else{
        std::cerr << "No joint found for " << joint_name << std::endl; 
        return false;
      }
    }
    // gripper act loop
    std::string act_name = robot_name_ + "_act_gripper";
    int act_id = mj_name2id(m_, mjOBJ_ACTUATOR, act_name.c_str());
    if(act_id != -1){
      gripper_act_idx_ = act_id;
    }
    else{
      std::cerr << "No mjOBJ_ACTUATOR " << act_name << " found for gripper" << std::endl; 
      return false;
    }
  }
  setModelIndices();
  return true;
}

franka::RobotState RobotSim::populateFrankaState(){
  /*
    Incomplete:
    std::array<double, 7UL> q_d_; // desired ...
    std::array<double, 7UL> dq_d_; // desired ...
    std::array<double, 7UL> ddq_d_; // desired ...
    std::array<double, 7UL> tau_J_d_; // desired ...
    std::array<double, 7UL> dtau_J_; // derivative joint torque
    
    franka::RobotState current_state_; // <- pack all of them into franka::RobotState
  */
  // mjModel* m = franka_hardware_model_->getMjModel();
  mjData* d = franka_hardware_model_->getMjData();
  double tau_ext_hat_filtered[7] = {0};
  for(int i=0; i<kNumberOfJoints; i++){
    current_state_.q[i] = d->qpos[joint_qpos_indices_[i]];
    current_state_.dq[i] = d->qvel[joint_qvel_indices_[i]];
    // the actual franka publishes non-zero values when in gravcomp mode, so add the qfrc_gravcomp to match that
    current_state_.tau_J[i] = d->actuator_force[act_trq_indices_[i]] + d->qfrc_gravcomp[joint_qvel_indices_[i]]; 
    tau_ext_hat_filtered[i] = d->qfrc_applied[joint_qvel_indices_[i]];
    current_state_.tau_ext_hat_filtered[i] = tau_ext_hat_filtered[i];
  }

  // calculate end effector jacobian, and then compose the EE force transform
  double basePos[3] = {0};
  double basePos_I[3] = {0};
  double baseQuat[4] = {0};
  double baseQuat_I[4] = {0};
  franka_hardware_model_->getXPosQuatbyLink(basePos, baseQuat, link_indices_[8]);
  mju_negPose(basePos_I, baseQuat_I, basePos, baseQuat);
  double forceTMat[36] = {0};
  franka_hardware_model_->composeForceTransform(forceTMat, basePos_I, baseQuat_I);
  // compose the 6*7
  double Jac[42] = {0};
  franka_hardware_model_->get6x7Jacobian(Jac, joint_site_indices_[8]);

  // multiply the jac
  double rJac[42] = {0};
  mju_mulMatMat(rJac, forceTMat, Jac, 6, 6, 7);
  // finally, get the force
  double eeForce[6] = {0};
  mju_mulMatVec(eeForce, rJac, tau_ext_hat_filtered, 6, 7);
  // forceTMat * Jac * torques
  for(int i=0; i<6; i++){
    // we want the end-effector external forces, so 8th body in the link indices.
    // This needs to be transformed into the base frame.
    current_state_.O_F_ext_hat_K[i] = eeForce[i];
  }

  // xpos is in W;
  // l7_W -> l7_B
  // T^B_W = l0_W
  // 
  // Doesn't really work; set up the inverses and stuff properly later.
  // o_t_ee_ = xpose[link0] * xpose[link7]
  int& s7 = joint_site_indices_[7];
  int& l0 = link_indices_[0];
  double eePosres[3] = {0};
  double eeQuatres[4] = {0};
  franka_hardware_model_->getBSiteInALinkframe(eePosres, eeQuatres, l0, s7);

  // transform data so that we can assign to O_T_EE
  double o_t_ee_rMVec[9] = {0};
  mju_quat2Mat(o_t_ee_rMVec, eeQuatres);
  double TMat[16] = {o_t_ee_rMVec[0], o_t_ee_rMVec[1], o_t_ee_rMVec[2], eePosres[0],
                     o_t_ee_rMVec[3], o_t_ee_rMVec[4], o_t_ee_rMVec[5], eePosres[1],
                     o_t_ee_rMVec[6], o_t_ee_rMVec[7], o_t_ee_rMVec[8], eePosres[2],
                     0,0,0,1};
  // finally, write to the state
  // convert to col major format
  // convertToColMajor
  int k = 0;
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      current_state_.O_T_EE[k] = TMat[row * 4 + col];
      k++;
    }
  }

  return current_state_;
}

franka_hardware::ModelSim* RobotSim::getModel() {
      return franka_hardware_model_.get();
}

bool RobotSim::setModelIndices(){
  franka_hardware_model_->setIndices(link_indices_,
                                     joint_site_indices_,
                                     joint_indices_,
                                     joint_qpos_indices_,
                                     joint_qvel_indices_,
                                     act_trq_indices_,
                                     act_pos_indices_,
                                     act_vel_indices_);
  return true;
}

} // namespace franka_hardware