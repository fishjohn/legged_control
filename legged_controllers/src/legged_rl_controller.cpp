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
  // Initialize legged interface
  //  std::string task_file, urdf_file, reference_file;
  //  controller_nh.getParam("/task_file", task_file);
  //  controller_nh.getParam("/urdf_file", urdf_file);
  //  controller_nh.getParam("/reference_file", reference_file);
  //  bool verbose;
  //  loadData::loadCppDataType(task_file, "legged_robot_interface.verbose", verbose);
  //
  //  legged_interface_ = std::make_shared<LeggedInterface>(task_file, urdf_file, reference_file, verbose);

  // get robot config from param server
  if (!this->parseCfg(controller_nh))
  {
    ROS_ERROR_STREAM("Get robot config fail from param server, some error occur!");
    return false;
  }

  // Hardware interface
  HybridJointInterface* hybrid_joint_interface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{ "LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                        "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE" };
  for (const auto& joint_name : joint_names)
    hybrid_joint_handles_.push_back(hybrid_joint_interface->getHandle(joint_name));

  imu_sensor_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

  //  ContactSensorInterface* contact_interface = robot_hw->get<ContactSensorInterface>();
  //  std::vector<ContactSensorHandle> contact_handles;
  //  for (auto& name : legged_interface_->modelSettings().contactNames3DoF)
  //    contact_handles.push_back(contact_interface->getHandle(name));

  // State estimation
  //  setupStateEstimate(*legged_interface_, hybrid_joint_handles_, contact_handles,
  //                     robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu"));

  // state init
  command_.setZero();
  last_actions_.setZero();
  base_lin_vel_.setZero();
  base_position_.setZero();
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

  // init publisher and subscriber
  base_state_sub_ = controller_nh.subscribe("/gazebo/model_states", 1, &LeggedRLController::baseStateRecCallback, this);
  policy_action_sub_ =
      controller_nh.subscribe("/rl_controller/action", 10, &LeggedRLController::policyActionRecCallback, this);
  obs_pub_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>(controller_nh, "/rl_controller/obs", 10));
  return true;
}

void LeggedRLController::starting(const ros::Time& time)
{
  controller_running_ = true;

  std::vector<double> init_joint_angles(12, 0);
  for (auto& hybrid_joint_handle : hybrid_joint_handles_)
    init_joint_angles_.push_back(hybrid_joint_handle.getPosition());

  double duration_secs = 2.0;
  stand_duration_ = duration_secs * 1000.0;
  stand_percent_ += 1 / stand_duration_;
  mode_ = Mode::LIE;
}

void LeggedRLController::stopping(const ros::Time& time)
{
  controller_running_ = false;
}

void LeggedRLController::update(const ros::Time& time, const ros::Duration& period)
{
  if (mode_ == Mode::LIE)
  {
    if (stand_percent_ < 1)
    {
      for (int j = 0; j < hybrid_joint_handles_.size(); j++)
      {
        double pos_des = init_joint_angles_[j] * (1 - stand_percent_) + default_joint_angles_[j] * stand_percent_;
        hybrid_joint_handles_[j].setCommand(pos_des, 0, 50, 1, 0);
      }
      stand_percent_ += 1 / stand_duration_;
    }
    else
    {
      mode_ = Mode::STAND;
      last_obs_pub_ = time;
    }
  }
  else if (mode_ == Mode::STAND)
  {
    mode_ = Mode::WALK;
    return;
  }
  else if (mode_ == Mode::WALK)
  {
    auto msg = computeObservation(time, period);
    publishObsMsg(msg, time);
    if (action_topic_updated_)
    {
      auto actions = actions_buffer_.readFromRT();
      assert(actions->data.size() == hybrid_joint_handles_.size());
      double action_min = -robot_cfg_.clip_actions;
      double action_max = robot_cfg_.clip_actions;
      std::transform(actions->data.begin(), actions->data.end(), actions->data.begin(),
                     [action_min, action_max](double x) { return std::max(action_min, std::min(action_max, x)); });
      for (size_t i = 0; i < hybrid_joint_handles_.size(); i++)
      {
        double pos_des = actions->data[i] * robot_cfg_.control_cfg.action_scale + default_joint_angles_(i, 0);
        hybrid_joint_handles_[i].setCommand(pos_des, 0, robot_cfg_.control_cfg.stiffness,
                                            robot_cfg_.control_cfg.damping, 0);
        last_actions_(i, 0) = actions->data[i];
      }

      action_topic_updated_ = false;
    }
  }
}

void LeggedRLController::publishObsMsg(const std::vector<double>& msg, const ros::Time& time)
{
  double publish_rate = 500;

  if (last_obs_pub_ + ros::Duration(1. / publish_rate) < time)
  {
    if (obs_pub_->trylock())
    {
      obs_pub_->msg_.data.resize(msg.size());
      assert(235 == msg.size());
      for (size_t i = 0; i < msg.size(); i++)
      {
        obs_pub_->msg_.data[i] = msg.at(i);
      }
      obs_pub_->unlockAndPublish();
    }
    last_obs_pub_ = time;
  }
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

void LeggedRLController::policyActionRecCallback(const std_msgs::Float32MultiArray& msg)
{
  actions_buffer_.writeFromNonRT(msg);
  action_topic_updated_ = true;
}

std::vector<double> LeggedRLController::computeObservation(const ros::Time& time, const ros::Duration& period)
{
  // Angular from IMU
  Eigen::Quaternion<scalar_t> quat(imu_sensor_handle_.getOrientation()[3], imu_sensor_handle_.getOrientation()[0],
                                   imu_sensor_handle_.getOrientation()[1], imu_sensor_handle_.getOrientation()[2]);
  Eigen::Matrix<double, 3, 3> inverse_rot = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat)).inverse();

  // linear velocity (base coordinate)
  Eigen::Matrix<double, 3, 1> g(0, 0, -9.81);
  Eigen::Matrix<double, 3, 1> imu_accel(imu_sensor_handle_.getLinearAcceleration()[0],
                                        imu_sensor_handle_.getLinearAcceleration()[1],
                                        imu_sensor_handle_.getLinearAcceleration()[2]);

  // Angular velocity
  Eigen::Matrix<double, 3, 1> base_ang_vel(imu_sensor_handle_.getAngularVelocity()[0],
                                           imu_sensor_handle_.getAngularVelocity()[1],
                                           imu_sensor_handle_.getAngularVelocity()[2]);

  // Projected gravity
  Eigen::Matrix<double, 3, 1> gravity_vector(0, 0, -1);
  Eigen::Matrix<double, 3, 1> projected_gravity(inverse_rot * gravity_vector);

  // command
  Eigen::Matrix<double, 3, 1> command(0.3, 0.0, 0);

  // dof position and dof velocity
  Eigen::Matrix<double, 12, 1> dof_pose;
  Eigen::Matrix<double, 12, 1> dof_vel;
  // The joint order in the message is different from the definition
  // of the joint order in the model so needs to be exchanged
  for (int i = 0; i < hybrid_joint_handles_.size(); i++)
  {
    dof_vel(i) = hybrid_joint_handles_[i].getVelocity();
    dof_pose(i) = hybrid_joint_handles_[i].getPosition();
  }

  // actions
  Eigen::Matrix<double, 12, 1> actions(last_actions_);

  // heights
  double measured_height = 0.0;
  Eigen::Matrix<double, 187, 1> heights;
  heights.fill(base_position_.z() - 0.5 - measured_height);

  ObsScales& obs_scales = robot_cfg_.obs_scales;
  Eigen::Matrix<double, 3, 3> command_scaler =
      Eigen::DiagonalMatrix<double, 3>(obs_scales.lin_vel, obs_scales.lin_vel, obs_scales.ang_vel);

  Eigen::Matrix<double, 235, 1> obs;
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

  std::vector<double> obs_vector(obs.data(), obs.data() + obs.size());
  double obs_min = -robot_cfg_.clip_obs;
  double obs_max = robot_cfg_.clip_obs;
  std::transform(obs_vector.begin(), obs_vector.end(), obs_vector.begin(),
                 [obs_min, obs_max](double x) { return std::max(obs_min, std::min(obs_max, x)); });

  return obs_vector;
}

void LeggedRLController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                              const std::string& reference_file, bool verbose)
{
  legged_interface_ = std::make_shared<LeggedInterface>(task_file, urdf_file, reference_file, verbose);
  legged_interface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void LeggedRLController::setupStateEstimate(LeggedInterface& legged_interface,
                                            const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                            const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                            const hardware_interface::ImuSensorHandle& imu_sensor_handle)
{
  state_estimate_ = std::make_shared<KalmanFilterEstimate>(*legged_interface_, hybrid_joint_handles_,
                                                           contact_sensor_handles, imu_sensor_handle);
  current_observation_.time = 0.;
}

bool LeggedRLController::parseCfg(ros::NodeHandle& nh)
{
  InitState& init_state = robot_cfg_.init_state;
  ControlCfg& control_cfg = robot_cfg_.control_cfg;
  ObsScales& obs_scales = robot_cfg_.obs_scales;

  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LF_HAA_joint", init_state.LF_HAA_joint);
  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LF_HFE_joint", init_state.LF_HFE_joint);
  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LF_KFE_joint", init_state.LF_KFE_joint);

  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RF_HAA_joint", init_state.RF_HAA_joint);
  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RF_HFE_joint", init_state.RF_HFE_joint);
  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RF_KFE_joint", init_state.RF_KFE_joint);

  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LH_HAA_joint", init_state.LH_HAA_joint);
  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LH_HFE_joint", init_state.LH_HFE_joint);
  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/LH_KFE_joint", init_state.LH_KFE_joint);

  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RH_HAA_joint", init_state.RH_HAA_joint);
  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RH_HFE_joint", init_state.RH_HFE_joint);
  nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/RH_KFE_joint", init_state.RH_KFE_joint);

  nh.getParam("/LeggedRobotCfg/control/stiffness", control_cfg.stiffness);
  nh.getParam("/LeggedRobotCfg/control/damping", control_cfg.damping);
  nh.getParam("/LeggedRobotCfg/control/action_scale", control_cfg.action_scale);
  nh.getParam("/LeggedRobotCfg/control/decimation", control_cfg.decimation);

  nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_observations", robot_cfg_.clip_obs);
  nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_actions", robot_cfg_.clip_actions);

  nh.getParam("/LeggedRobotCfg/normalization/obs_scales/lin_vel", obs_scales.lin_vel);
  nh.getParam("/LeggedRobotCfg/normalization/obs_scales/ang_vel", obs_scales.ang_vel);
  nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_pos", obs_scales.dof_pos);
  nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_vel", obs_scales.dof_vel);
  nh.getParam("/LeggedRobotCfg/normalization/obs_scales/height_measurements", obs_scales.height_measurements);

  return true;
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedRLController, controller_interface::ControllerBase)
