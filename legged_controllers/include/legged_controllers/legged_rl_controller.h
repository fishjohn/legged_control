//
// Created by luohx on 22-12-5.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <legged_common/hardware_interface/hybrid_joint_interface.h>
#include <legged_common/hardware_interface/contact_sensor_interface.h>
#include <legged_estimation/state_estimate_base.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <Eigen/Geometry>

#include <torch/torch.h>
#include <torch/script.h>

namespace legged
{
using namespace ocs2;

struct ObsScales
{
  double lin_vel = 2.0;
  double ang_vel = 0.25;
  double dof_pos = 1.0;
  double dof_vel = 0.05;
  double height_measurements = 5.0;
};

struct A1Cfg {
  struct ControlCfg {
    float stiffness = 20.;
    float damping = 0.5;
    float action_scale = 0.25;
    int decimation = 4;
  };

  struct InitState {
    // default joint angles
    double LF_HAA_joint = 0.1;
    double LF_HFE_joint = 0.8;
    double LF_KFE_joint = -1.5;

    double LH_HAA_joint = 0.1;
    double LH_HFE_joint = 1.0;
    double LH_KFE_joint = -1.5;

    double RF_HAA_joint = -0.1;
    double RF_HFE_joint = 0.8;
    double RF_KFE_joint = -1.5;

    double RH_HAA_joint = -0.1;
    double RH_HFE_joint = 1.0;
    double RH_KFE_joint = -1.5;
  };
  double clip_actions = 100.0;
  double clip_obs = 100.0;
  ControlCfg control_cfg;
  InitState init_state;
};

class LeggedRLController
  : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                          ContactSensorInterface>
{
public:
  LeggedRLController() = default;
  ~LeggedRLController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

protected:
  void computeObservation(const ros::Time& time, const ros::Duration& period);

  std::vector<HybridJointHandle> hybrid_joint_handles_;
  hardware_interface::ImuSensorHandle imu_sensor_handle_;

private:
  std::string policy_file_;
  torch::jit::script::Module policy_;
  std::vector<torch::jit::IValue> obs_tensor_;

  std::atomic_bool controller_running_;

  // temp state
  ObsScales obs_scales_;
  A1Cfg robot_cfg_;
  Eigen::Matrix<double, 3, 1> base_lin_vel_;
  Eigen::Matrix<double, 3, 1> command_;
  Eigen::Matrix<double, 3, 3> command_scaler_;
  Eigen::Matrix<double, 12, 1> last_actions_;
  Eigen::Matrix<double, 12, 1> default_joint_angles_;
};

}  // namespace legged
