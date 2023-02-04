//
// Created by luohx on 22-12-5.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/jacobian.hpp>

#include "legged_controllers/legged_rl_controller.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <pluginlib/class_list_macros.hpp>

namespace legged
{
LeggedRLController::~LeggedRLController()
{
  controller_running_ = false;
}

bool LeggedRLController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  // Get config file
  std::string task_file;
  std::string urdf_file;
  std::string reference_file;
  controller_nh.getParam("/task_file", task_file);
  controller_nh.getParam("/urdf_file", urdf_file);
  controller_nh.getParam("/reference_file", reference_file);

  // Get policy model
  std::string policy_file_path;
  if (!controller_nh.getParam("/module/module_path", policy_file_path))
  {
    ROS_ERROR_STREAM("Get policy path fail from param server, some error occur!");
    return false;
  }
  loadPolicyModel(policy_file_path);

  // Get robot config from param server
  if (!this->parseCfg(controller_nh))
  {
    ROS_ERROR_STREAM("Get robot config fail from param server, some error occur!");
    return false;
  }
  actions_.resize(actions_size_);
  observations_.resize(observation_size_);

  // state init
  command_.setZero();
  last_actions_.setZero();
  base_lin_vel_.setZero();
  base_position_.setZero();
  std::vector<scalar_t> default_joint_angles{ robot_cfg_.init_state.LF_HAA_joint, robot_cfg_.init_state.LF_HFE_joint,
                                              robot_cfg_.init_state.LF_KFE_joint, robot_cfg_.init_state.RF_HAA_joint,
                                              robot_cfg_.init_state.RF_HFE_joint, robot_cfg_.init_state.RF_KFE_joint,
                                              robot_cfg_.init_state.LH_HAA_joint, robot_cfg_.init_state.LH_HFE_joint,
                                              robot_cfg_.init_state.LH_KFE_joint, robot_cfg_.init_state.RH_HAA_joint,
                                              robot_cfg_.init_state.RH_HFE_joint, robot_cfg_.init_state.RH_KFE_joint };
  for (int i = 0; i < default_joint_angles.size(); i++)
  {
    default_joint_angles_(i, 0) = default_joint_angles[i];
  }

  // Hardware interface
  auto* hybrid_joint_interface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names = { "LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                           "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE" };
  for (const auto& joint_name : joint_names)
    hybrid_joint_handles_.push_back(hybrid_joint_interface->getHandle(joint_name));

  imu_sensor_handles_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

  auto* contact_interface = robot_hw->get<ContactSensorInterface>();
  std::vector<std::string> contact_names = { "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT" };
  for (const auto& name : contact_names)
  {
    contact_handles_.push_back(contact_interface->getHandle(name));
  }

  // State estimate
  setupStateEstimate(task_file, urdf_file, reference_file);

  // init publisher and subscriber
  cmd_vel_sub_ = controller_nh.subscribe("/cmd_vel", 1, &LeggedRLController::cmdVelCallback, this);
  return true;
}

void LeggedRLController::starting(const ros::Time& time)
{
  controller_running_ = true;

  std::vector<scalar_t> init_joint_angles(12, 0);
  for (auto& hybrid_joint_handle : hybrid_joint_handles_)
    init_joint_angles_.push_back(hybrid_joint_handle.getPosition());

  scalar_t duration_secs = 2.0;
  stand_duration_ = duration_secs * 1000.0;
  stand_percent_ += 1 / stand_duration_;
  mode_ = Mode::LIE;
  loop_count_ = 0;
}

void LeggedRLController::stopping(const ros::Time& time)
{
  controller_running_ = false;
}

void LeggedRLController::update(const ros::Time& time, const ros::Duration& period)
{
  // state estimate
  rbd_state_ = state_estimate_->update(time, period);

  if (mode_ == Mode::LIE)
  {
    if (stand_percent_ < 1)
    {
      for (int j = 0; j < hybrid_joint_handles_.size(); j++)
      {
        scalar_t pos_des = init_joint_angles_[j] * (1 - stand_percent_) + default_joint_angles_[j] * stand_percent_;
        hybrid_joint_handles_[j].setCommand(pos_des, 0, 50, 1, 0);
      }
      stand_percent_ += 1 / stand_duration_;
    }
    else
    {
      mode_ = Mode::STAND;
    }
  }
  else if (mode_ == Mode::STAND)
  {
    if (loop_count_ > 5000)
    {
      mode_ = Mode::WALK;
    }
  }
  else if (mode_ == Mode::WALK)
  {
    // compute observation & actions
    if (loop_count_ % robot_cfg_.control_cfg.decimation == 0)
    {
      computeObservation(time, period);
      computeActions();
      // limit action range
      scalar_t action_min = -robot_cfg_.clip_actions;
      scalar_t action_max = robot_cfg_.clip_actions;
      std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                     [action_min, action_max](scalar_t x) { return std::max(action_min, std::min(action_max, x)); });
    }

    // set action
    for (int i = 0; i < hybrid_joint_handles_.size(); i++)
    {
      scalar_t pos_des = actions_[i] * robot_cfg_.control_cfg.action_scale + default_joint_angles_(i, 0);
      hybrid_joint_handles_[i].setCommand(pos_des, 0, robot_cfg_.control_cfg.stiffness, robot_cfg_.control_cfg.damping,
                                          0);
      last_actions_(i, 0) = actions_[i];
    }
  }
  loop_count_++;
}

void LeggedRLController::setupStateEstimate(const std::string& task_file, const std::string& urdf_file,
                                            const std::string& reference_file)
{
  ModelSettings model_settings;
  model_settings = loadModelSettings(task_file, "model_settings", false);

  // PinocchioInterface
  auto pinocchio_interface_ptr = std::make_unique<PinocchioInterface>(
      centroidal_model::createPinocchioInterface(urdf_file, model_settings.jointNames));

  // CentroidalModelInfo
  CentroidalModelInfo centroidal_model_info = centroidal_model::createCentroidalModelInfo(
      *pinocchio_interface_ptr, centroidal_model::loadCentroidalType(task_file),
      centroidal_model::loadDefaultJointState(pinocchio_interface_ptr->getModel().nq - 6, reference_file),
      model_settings.contactNames3DoF, model_settings.contactNames6DoF);

  // PinocchioEndEffectorKinematics
  CentroidalModelPinocchioMapping pinocchio_mapping(centroidal_model_info);
  auto ee_kinematic_ptr = std::make_shared<PinocchioEndEffectorKinematics>(*pinocchio_interface_ptr, pinocchio_mapping,
                                                                           model_settings.contactNames3DoF);

  state_estimate_ = std::make_shared<KalmanFilterEstimate>(
      std::make_unique<PinocchioInterface>(
          centroidal_model::createPinocchioInterface(urdf_file, model_settings.jointNames)),
      centroidal_model_info, *ee_kinematic_ptr, hybrid_joint_handles_, contact_handles_, imu_sensor_handles_);
}

void LeggedRLController::loadPolicyModel(const std::string& policy_file_path)
{
  policy_file_path_ = policy_file_path;
  ROS_INFO_STREAM("Load Onnx model from path : " << policy_file_path);

  // create env
  onnx_env_prt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
  // create session
  Ort::SessionOptions session_options;
  session_options.SetInterOpNumThreads(1);
  session_ptr_ = std::make_unique<Ort::Session>(*onnx_env_prt_, policy_file_path.c_str(), session_options);
  // get input and output info
  input_names_.clear();
  output_names_.clear();
  input_shapes_.clear();
  output_shapes_.clear();
  Ort::AllocatorWithDefaultOptions allocator;
  for (int i = 0; i < session_ptr_->GetInputCount(); i++)
  {
    input_names_.push_back(session_ptr_->GetInputName(i, allocator));
    input_shapes_.push_back(session_ptr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
  }
  for (int i = 0; i < session_ptr_->GetOutputCount(); i++)
  {
    output_names_.push_back(session_ptr_->GetOutputName(i, allocator));
    output_shapes_.push_back(session_ptr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
  }

  ROS_INFO_STREAM("Load Onnx model from successfully !!!");
}

void LeggedRLController::computeActions()
{
  // create input tensor object
  Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
  std::vector<Ort::Value> input_values;
  input_values.push_back(Ort::Value::CreateTensor<tensor_element_t>(
      memory_info, observations_.data(), observations_.size(), input_shapes_[0].data(), input_shapes_[0].size()));
  // run inference
  Ort::RunOptions run_options;
  std::vector<Ort::Value> output_values =
      session_ptr_->Run(run_options, input_names_.data(), input_values.data(), 1, output_names_.data(), 1);

  for (int i = 0; i < actions_size_; i++)
  {
    actions_[i] = *(output_values[0].GetTensorMutableData<tensor_element_t>() + i);
  }
}

void LeggedRLController::computeObservation(const ros::Time& time, const ros::Duration& period)
{
  // Angular from IMU
  Eigen::Quaternion<scalar_t> quat(imu_sensor_handles_.getOrientation()[3], imu_sensor_handles_.getOrientation()[0],
                                   imu_sensor_handles_.getOrientation()[1], imu_sensor_handles_.getOrientation()[2]);
  Eigen::Matrix<scalar_t, 3, 3> inverse_rot = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat)).inverse();

  // linear velocity (base coordinate)
  Eigen::Matrix<scalar_t, 3, 1> g(0, 0, -9.81);
  Eigen::Matrix<scalar_t, 3, 1> imu_accel(imu_sensor_handles_.getLinearAcceleration()[0],
                                          imu_sensor_handles_.getLinearAcceleration()[1],
                                          imu_sensor_handles_.getLinearAcceleration()[2]);
  base_lin_vel_ = inverse_rot * rbd_state_.segment<3>(21);

  // Angular velocity
  Eigen::Matrix<scalar_t, 3, 1> base_ang_vel(imu_sensor_handles_.getAngularVelocity()[0],
                                             imu_sensor_handles_.getAngularVelocity()[1],
                                             imu_sensor_handles_.getAngularVelocity()[2]);

  // Projected gravity
  Eigen::Matrix<scalar_t, 3, 1> gravity_vector(0, 0, -1);
  Eigen::Matrix<scalar_t, 3, 1> projected_gravity(inverse_rot * gravity_vector);

  // command
  Eigen::Matrix<scalar_t, 3, 1> command = command_;

  // dof position and dof velocity
  Eigen::Matrix<scalar_t, 12, 1> dof_pose;
  Eigen::Matrix<scalar_t, 12, 1> dof_vel;
  // The joint order in the message is different from the definition
  // of the joint order in the model so needs to be exchanged
  for (int i = 0; i < hybrid_joint_handles_.size(); i++)
  {
    dof_vel(i) = hybrid_joint_handles_[i].getVelocity();
    dof_pose(i) = hybrid_joint_handles_[i].getPosition();
  }

  // actions
  Eigen::Matrix<scalar_t, 12, 1> actions(last_actions_);

  // heights
  scalar_t measured_height = 0.0;
  scalar_t base_height = rbd_state_(5);
  Eigen::Matrix<scalar_t, 187, 1> heights;
  heights.fill(base_height - 0.5 - measured_height);

  ObsScales& obs_scales = robot_cfg_.obs_scales;
  Eigen::Matrix<scalar_t, 3, 3> command_scaler =
      Eigen::DiagonalMatrix<scalar_t, 3>(obs_scales.lin_vel, obs_scales.lin_vel, obs_scales.ang_vel);

  Eigen::Matrix<scalar_t, 235, 1> obs;
  // clang-format off
  obs << base_lin_vel_ * obs_scales.lin_vel,
      base_ang_vel * obs_scales.ang_vel,
      projected_gravity,
      command_scaler * command,
      (dof_pose - default_joint_angles_) * obs_scales.dof_pos,
      dof_vel * obs_scales.dof_vel,
      actions,
      heights * obs_scales.height_measurements;
  // clang-format on

  for (Eigen::Matrix<scalar_t, 235, 1>::Index i = 0; i < obs.size(); i++)
  {
    observations_[i] = static_cast<tensor_element_t>(obs(i));
  }
  // Limit observation range
  scalar_t obs_min = -robot_cfg_.clip_obs;
  scalar_t obs_max = robot_cfg_.clip_obs;
  std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                 [obs_min, obs_max](scalar_t x) { return std::max(obs_min, std::min(obs_max, x)); });
}

void LeggedRLController::baseStateRecCallback(const gazebo_msgs::ModelStates& msg)
{
  base_lin_vel_.x() = msg.twist[1].linear.x;
  base_lin_vel_.y() = msg.twist[1].linear.y;
  base_lin_vel_.z() = msg.twist[1].linear.z;

  base_position_.x() = msg.pose[1].position.x;
  base_position_.y() = msg.pose[1].position.y;
  base_position_.z() = msg.pose[1].position.z;
}

void LeggedRLController::cmdVelCallback(const geometry_msgs::Twist& msg)
{
  command_[0] = msg.linear.x;
  command_[1] = msg.linear.y;
  command_[2] = msg.linear.z;
}

bool LeggedRLController::parseCfg(ros::NodeHandle& nh)
{
  InitState& init_state = robot_cfg_.init_state;
  ControlCfg& control_cfg = robot_cfg_.control_cfg;
  ObsScales& obs_scales = robot_cfg_.obs_scales;

  int error = 0;
  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LF_HAA_joint", init_state.LF_HAA_joint));
  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LF_HFE_joint", init_state.LF_HFE_joint));
  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LF_KFE_joint", init_state.LF_KFE_joint));

  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RF_HAA_joint", init_state.RF_HAA_joint));
  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RF_HFE_joint", init_state.RF_HFE_joint));
  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RF_KFE_joint", init_state.RF_KFE_joint));

  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LH_HAA_joint", init_state.LH_HAA_joint));
  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LH_HFE_joint", init_state.LH_HFE_joint));
  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LH_KFE_joint", init_state.LH_KFE_joint));

  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RH_HAA_joint", init_state.RH_HAA_joint));
  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RH_HFE_joint", init_state.RH_HFE_joint));
  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RH_KFE_joint", init_state.RH_KFE_joint));

  error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness", control_cfg.stiffness));
  error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping", control_cfg.damping));
  error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/action_scale", control_cfg.action_scale));
  error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/decimation", control_cfg.decimation));

  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_observations", robot_cfg_.clip_obs));
  error +=
      static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_actions", robot_cfg_.clip_actions));

  error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/lin_vel", obs_scales.lin_vel));
  error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/ang_vel", obs_scales.ang_vel));
  error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_pos", obs_scales.dof_pos));
  error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_vel", obs_scales.dof_vel));
  error += static_cast<int>(
      !nh.getParam("/LeggedRobotCfg/normalization/obs_scales/height_measurements", obs_scales.height_measurements));

  error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/actions_size", actions_size_));
  error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/observations_size", observation_size_));

  return (error == 0);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedRLController, controller_interface::ControllerBase)
