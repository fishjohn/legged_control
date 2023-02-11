//
// Created by luohx on 23-2-9.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <legged_estimation/StateEstimateBase.h>
#include <legged_estimation/LinearKalmanFilter.h>

#include <ocs2_mpc/SystemObservation.h>

#include <legged_controllers/cheetah_basic/mpc_solver.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_wbc/WbcBase.h>

#include <legged_controllers/cheetah_basic/util/ros_utilities.h>

#include <dynamic_reconfigure/server.h>
#include "legged_controllers/WeightConfig.h"

namespace legged {

class LeggedCMPCController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                                   ContactSensorInterface> {
public:
  LeggedCMPCController() = default;
  ~LeggedCMPCController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

protected:
  void setupMPC();

  virtual void setupStateEstimate(const std::string& taskFile, const std::string& urdfFile,
                                  const std::string&referenceFile);

  void updateRobotState(const vector_t& rbdState);

  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<MpcSolverBase> mpcSolver_;

  SystemObservation currentObservation_;
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

private:
  std::atomic_bool controllerRunning_{}, mpcRunning_{};

  RobotState robot_state_;

  // MPC
  int horizon_;
  VectorXd gait_table_;
  VectorXd traj_;

  // Kinematic
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::shared_ptr<PinocchioInterface> pinocchioInterfacePtr_;

  // Dynamic reconfigure
  void dynamicCallback(legged_control::WeightConfig& config, uint32_t /*level*/);

  std::shared_ptr<dynamic_reconfigure::Server<legged_control::WeightConfig>> dynamic_srv_{};
  realtime_tools::RealtimeBuffer<legged_control::WeightConfig> weight_buffer_;
  bool dynamic_initialized_;

  // Subscribe
  

};

}   // namespace legged