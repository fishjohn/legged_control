//
// Created by luohx on 23-2-9.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/jacobian.hpp>

#include "legged_controllers/LeggedCMPCController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <pluginlib/class_list_macros.hpp>

namespace legged {

bool LeggedCMPCController::init(hardware_interface::RobotHW *robot_hw,
                                ros::NodeHandle &controller_nh) {
  // Get config file
  std::string task_file;
  std::string urdf_file;
  std::string reference_file;
  controller_nh.getParam("/task_file", task_file);
  controller_nh.getParam("/urdf_file", urdf_file);
  controller_nh.getParam("/reference_file", reference_file);

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }

  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  std::vector<std::string> contactNames3DoF{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  for (const auto& name : contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }

  auto* imuInterface = robot_hw->get<hardware_interface::ImuSensorInterface>();
  std::string imuName = "unitree_imu";
  imuSensorHandle_ = imuInterface->getHandle(imuName);

  // State estimate
  setupStateEstimate(task_file, urdf_file, reference_file);
  eeKinematicsPtr_->setPinocchioInterface(*pinocchioInterfacePtr_);


  XmlRpc::XmlRpcValue mpc_params;
  legged_control::WeightConfig config;
  controller_nh.getParam("mpc", mpc_params);
  ROS_ASSERT(mpc_params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  if (mpc_params.hasMember("weight"))
  {
    if (mpc_params["weight"].getType() == XmlRpc::XmlRpcValue::TypeArray)
      if (mpc_params["weight"].size() == 12)
      {
        config.ori_roll = xmlRpcGetDouble(mpc_params["weight"], 0);
        config.ori_pitch = xmlRpcGetDouble(mpc_params["weight"], 1);
        config.ori_yaw = xmlRpcGetDouble(mpc_params["weight"], 2);
        config.pos_x = xmlRpcGetDouble(mpc_params["weight"], 3);
        config.pos_y = xmlRpcGetDouble(mpc_params["weight"], 4);
        config.pos_z = xmlRpcGetDouble(mpc_params["weight"], 5);
        config.rate_roll = xmlRpcGetDouble(mpc_params["weight"], 6);
        config.rate_pitch = xmlRpcGetDouble(mpc_params["weight"], 7);
        config.rate_yaw = xmlRpcGetDouble(mpc_params["weight"], 8);
        config.vel_x = xmlRpcGetDouble(mpc_params["weight"], 9);
        config.vel_y = xmlRpcGetDouble(mpc_params["weight"], 10);
        config.vel_z = xmlRpcGetDouble(mpc_params["weight"], 11);
        config.alpha = xmlRpcGetDouble(mpc_params["alpha"]);
        config.horizon = mpc_params["horizon"];
        config.dt = xmlRpcGetDouble(mpc_params["dt"]);
        weight_buffer_.initRT(config);
        dynamic_initialized_ = false;
      }
  }
  dynamic_initialized_ = true;

  setupMPC();

  // Dynamic reconfigure
  ros::NodeHandle nh_mpc = ros::NodeHandle(controller_nh, "mpc");
  dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<legged_control::WeightConfig>>(nh_mpc);
  dynamic_reconfigure::Server<legged_control::WeightConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
    dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  dynamic_srv_->setCallback(cb);


  traj_.resize(12 * horizon_);
  traj_.setZero();

  gait_table_.resize(4 * horizon_);
  gait_table_.setZero();

  return true;
}

void LeggedCMPCController::starting(const ros::Time &time) {
}

void LeggedCMPCController::update(const ros::Time &time,
                                  const ros::Duration &period) {
  // State Estimate
  currentObservation_.time += period.toSec();

  vector_t measuredRbdState = stateEstimate_->update(time, period);
  updateRobotState(measuredRbdState);

  mpcSolver_->solve(time, robot_state_, gait_table_, traj_);
  std::vector<Vec3<double>> solution = mpcSolver_->getSolution();
  std::cout << solution.size() << std::endl;
  for(auto iter = solution.begin(); iter != solution.end(); ++iter) {
    std::cout << "-----------------------------" << std::endl;
    std::cout << *iter << std::endl;
    std::cout << "-----------------------------" << std::endl;
  }
}

LeggedCMPCController::~LeggedCMPCController() {
  controllerRunning_ = false;
}

void LeggedCMPCController::updateRobotState(const vector_t& rbdState) {
  // Orientation
  Eigen::Quaterniond quat(imuSensorHandle_.getOrientation()[3], imuSensorHandle_.getOrientation()[0],
                          imuSensorHandle_.getOrientation()[1], imuSensorHandle_.getOrientation()[2]);
  robot_state_.quat_ = quat;

  // Pose
  robot_state_.pos_ = rbdState.segment<3>(3);

  // Linear velocity
  robot_state_.linear_vel_ = rbdState.segment<3>(21);

  // Anaular velocity
  robot_state_.angular_vel_ = rbdState.segment<3>(18);

  // Accelreation
  Eigen::Vector3d accel(imuSensorHandle_.getLinearAcceleration()[0],
                        imuSensorHandle_.getLinearAcceleration()[1],
                        imuSensorHandle_.getLinearAcceleration()[2]);
  robot_state_.accel_ = accel;

  // Foot pose / Foot velocity
  const auto eePos = eeKinematicsPtr_->getPosition(vector_t());
  const auto eeVel = eeKinematicsPtr_->getVelocity(vector_t(), vector_t());

  std::vector<int> index_map = {0, 2, 1, 3};
  for (int i = 0; i < 4; i++) {
    double footRadius = 0.02;
    robot_state_.foot_pos_[i].segment<3>(0) = -eePos[index_map[i]];
    robot_state_.foot_pos_[i](1) += footRadius;
    robot_state_.foot_vel_[i].segment<3>(0) = -eeVel[index_map[i]];
  }

  // Foot contact
  for (int i = 0; i < 4; i++) {
    robot_state_.contact_state_[i] = contactHandles_[index_map[i]].isContact();
  }
}

void LeggedCMPCController::setupMPC() {
  // TODO : Add Interface
  double mass = 22.5;
  Matrix3d inertia;
  inertia << 0.050874, 0., 0., 0., 0.64036, 0., 0., 0., 0.6565;
  mpcSolver_ = std::make_shared<QpOasesSolver>(mass, -9.81, 0.6, inertia);
}

void LeggedCMPCController::setupStateEstimate(const std::string &taskFile,
                                              const std::string &urdfFile,
                                              const std::string &referenceFile) {
  ModelSettings modelSettings;
  modelSettings = loadModelSettings(taskFile, "model_settings", false);

  // PinocchioInterface
  pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(
      centroidal_model::createPinocchioInterface(urdfFile,
                                                 modelSettings.jointNames));

  // CentroidalModelInfo
  CentroidalModelInfo centroidalModelInfo = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(
              pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
          modelSettings.contactNames3DoF, modelSettings.contactNames6DoF);

  // PinocchioEndEffectorKinematics
  CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo);
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(*pinocchioInterfacePtr_, pinocchioMapping,
      modelSettings.contactNames3DoF);

  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(*pinocchioInterfacePtr_, centroidalModelInfo,
                                                           *eeKinematicsPtr_, hybridJointHandles_, contactHandles_, imuSensorHandle_);


  currentObservation_.time = 0;
}
void LeggedCMPCController::dynamicCallback(legged_control::WeightConfig &config,
                                           uint32_t) {
  if (!dynamic_initialized_)
  {
    ROS_INFO("[Mpc] Dynamic params are set by rosparams at initialization");

    dynamic_initialized_ = true;
    legged_control::WeightConfig init_config = *weight_buffer_.readFromNonRT();
    config.ori_roll = init_config.ori_roll;
    config.ori_pitch = init_config.ori_pitch;
    config.ori_yaw = init_config.ori_yaw;
    config.pos_x = init_config.pos_x;
    config.pos_y = init_config.pos_y;
    config.pos_z = init_config.pos_z;
    config.rate_roll = init_config.rate_roll;
    config.rate_pitch = init_config.rate_pitch;
    config.rate_yaw = init_config.rate_yaw;
    config.vel_x = init_config.vel_x;
    config.vel_y = init_config.vel_y;
    config.vel_z = init_config.vel_z;
    config.alpha = init_config.alpha;
    config.horizon = init_config.horizon;
    config.dt = init_config.dt;
  }
  Matrix<double, 13, 1> weight;
  weight << config.ori_roll, config.ori_pitch, config.ori_yaw, config.pos_x, config.pos_y, config.pos_z,
      config.rate_roll, config.rate_pitch, config.rate_yaw, config.vel_x, config.vel_y, config.vel_z, 0.;
  horizon_ = config.horizon;
  std::cout << "config dt: " << config.dt << "config horizon : " << config.horizon << std::endl;
  mpcSolver_->setup(config.dt, config.horizon, 200., weight, config.alpha, 1.0);
  ROS_INFO("[Mpc] Dynamic params update");
}

} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedCMPCController, controller_interface::ControllerBase)