//
// Created by luohx on 22-12-5.
//
#include "legged_controllers/legged_rl_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged
{
LeggedRLController::~LeggedRLController()
{
  controller_running_ = false;
}

bool LeggedRLController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  // Load Policy from module
  //  controller_nh.getParam("/policy_file", policy_file_);
  policy_file_ =
      "/home/luohx/catkin_ws/src/unitree_ros/reinforce_controller/policy/rough_a1/exported/policies/policy_1.pt";
  try
  {
    policy_ = torch::jit::load(policy_file_);
  }
  catch (const c10::Error& e)
  {
    ROS_ERROR_STREAM("[Legged RL Controller] Error occurred while load torch model");
    return false;
  }
  ROS_INFO_STREAM("[Legged RL Controller] load policy successfully");

  // Hardware interface
  HybridJointInterface* hybrid_joint_interface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{ "LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                        "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE" };
  for (const auto& joint_name : joint_names)
    hybrid_joint_handles_.push_back(hybrid_joint_interface->getHandle(joint_name));

  imu_sensor_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

  // obs variable
  assert(imu_sensor_handle_.getLinearAcceleration()[0] == 0 && imu_sensor_handle_.getLinearAcceleration()[1] == 0 &&
         imu_sensor_handle_.getLinearAcceleration()[2] == 0);
  base_lin_vel_.setZero();
  last_actions_.setZero();
  command_.setZero();
  command_scaler_.setZero();
  command_scaler_(0, 0) = obs_scales_.lin_vel;
  command_scaler_(1, 1) = obs_scales_.lin_vel;
  command_scaler_(2, 2) = obs_scales_.ang_vel;
  std::vector<double> default_joint_angles{ robot_cfg_.init_state.LF_HAA_joint, robot_cfg_.init_state.LF_HFE_joint,
                                            robot_cfg_.init_state.LF_KFE_joint, robot_cfg_.init_state.RF_HAA_joint,
                                            robot_cfg_.init_state.RF_HFE_joint, robot_cfg_.init_state.RF_KFE_joint,
                                            robot_cfg_.init_state.LH_HAA_joint, robot_cfg_.init_state.LH_HFE_joint,
                                            robot_cfg_.init_state.LH_KFE_joint, robot_cfg_.init_state.RH_HAA_joint,
                                            robot_cfg_.init_state.RH_HFE_joint, robot_cfg_.init_state.RH_KFE_joint };
  for (size_t i = 0; i < default_joint_angles.size(); i++)
  {
    default_joint_angles_(i, 0) = default_joint_angles[i];
  }
  obs_tensor_.clear();
  obs_tensor_.emplace_back(torch::tensor(std::vector<float>(235, 0)));
  auto actions = policy_.forward(obs_tensor_).toTensor();
  return true;
}

void LeggedRLController::update(const ros::Time& time, const ros::Duration& period)
{
  // compute observation
//  computeObservation(time, period);

  // get action
//    at::Tensor actions = policy_.forward(obs_tensor_).toTensor();

  //  actions = torch::clip(actions, -robot_cfg_.clip_actions, robot_cfg_.clip_actions);
  //  auto actions_vector = std::vector<double>(actions.data_ptr<double>(), actions.data_ptr<double>() + actions.numel());

  //  // set command
  //  // TODO: check whether is need to set command decimation times each update
  //  for (size_t i = 0; i < robot_cfg_.control_cfg.decimation; i++)
  //  {
  //    at::Tensor actions_scaled = actions * robot_cfg_.control_cfg.action_scale;
  //    for (size_t j = 0; j < hybrid_joint_handles_.size(); j++)
  //    {
  //      double pos_des = actions_vector[j] + default_joint_angles_(j, 0);
  //      hybrid_joint_handles_[j].setCommand(pos_des, 0, robot_cfg_.control_cfg.stiffness, robot_cfg_.control_cfg.damping,
  //                                          0);
  //    }
  //  }
  //
  // record last action
  //  for (size_t i = 0; i < hybrid_joint_handles_.size(); i++)
  //  {
  //    last_actions_(i, 0) = actions_vector[i];
  //  }
}

void LeggedRLController::starting(const ros::Time& time)
{
  controller_running_ = true;
}

void LeggedRLController::stopping(const ros::Time& time)
{
  controller_running_ = false;
}

void LeggedRLController::computeObservation(const ros::Time& time, const ros::Duration& period)
{
  obs_tensor_.clear();

  double dt = period.toSec();

  // Angular from IMU
  Eigen::Quaternion<double> quat(imu_sensor_handle_.getOrientation()[3], imu_sensor_handle_.getOrientation()[0],
                                 imu_sensor_handle_.getOrientation()[1], imu_sensor_handle_.getOrientation()[2]);
  Eigen::Matrix<double, 3, 3> inverse_rot = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat)).inverse();

  // linear velocity (base coordinate)
  Eigen::Matrix<double, 3, 1> g(0, 0, -9.81);
  Eigen::Matrix<double, 3, 1> imu_accel(imu_sensor_handle_.getLinearAcceleration()[0],
                                        imu_sensor_handle_.getLinearAcceleration()[1],
                                        imu_sensor_handle_.getLinearAcceleration()[2]);

  Eigen::Matrix<double, 3, 1> accel = imu_accel + inverse_rot * g;
  Eigen::Matrix<double, 3, 1> base_lin_vel = base_lin_vel_ + accel * dt;

  // Angular velocity
  Eigen::Matrix<double, 3, 1> base_ang_vel(imu_sensor_handle_.getAngularVelocity()[0],
                                           imu_sensor_handle_.getAngularVelocity()[1],
                                           imu_sensor_handle_.getAngularVelocity()[2]);
  std::cout << base_lin_vel << std::endl;
  // Projected gravity
  Eigen::Matrix<double, 3, 1> gravity_vector(0, 0, -1);
  Eigen::Matrix<double, 3, 1> projected_gravity(inverse_rot * gravity_vector);

  // command
  Eigen::Matrix<double, 3, 1> command(0.5, 0, 0);

  // dof position
  Eigen::Matrix<double, 12, 1> dof_pose;
  for (size_t i = 0; i < hybrid_joint_handles_.size(); i++)
  {
    dof_pose(i) = hybrid_joint_handles_[i].getPosition();
  }

  // dof velocity
  Eigen::Matrix<double, 12, 1> dof_vel;
  for (size_t i = 0; i < hybrid_joint_handles_.size(); i++)
  {
    dof_vel(i) = hybrid_joint_handles_[i].getVelocity();
  }

  // actions
  Eigen::Matrix<double, 12, 1> actions(last_actions_);

  // heights
  Eigen::Matrix<double, 187, 1> heights;
  heights.setZero();

  Eigen::Matrix<double, 235, 1> obs;
  // clang-format off
  obs << base_lin_vel * obs_scales_.lin_vel,
         base_ang_vel * obs_scales_.ang_vel,
         projected_gravity,
         command_scaler_ * command,
         (dof_pose - default_joint_angles_) * obs_scales_.dof_pos,
         dof_vel * obs_scales_.dof_vel,
         actions,
         heights * obs_scales_.height_measurements;
  // clang-format on

  std::vector<double> obs_vector(obs.data(), obs.data() + obs.size());
  at::Tensor obs_tensor = torch::clip(torch::tensor(obs_vector), -robot_cfg_.clip_obs, robot_cfg_.clip_obs);
  obs_tensor_.emplace_back(obs_tensor);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedRLController, controller_interface::ControllerBase)
