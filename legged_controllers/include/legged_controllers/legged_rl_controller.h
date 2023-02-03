//
// Created by luohx on 22-12-5.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_estimation/StateEstimateBase.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_interface/LeggedInterface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float32MultiArray.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <Eigen/Geometry>

namespace legged
{
using namespace ocs2;
using namespace legged_robot;

struct RobotCfg
{
  struct ControlCfg
  {
    float stiffness;
    float damping;
    float action_scale;
    int decimation;
  };

  struct InitState
  {
    // default joint angles
    scalar_t LF_HAA_joint;
    scalar_t LF_HFE_joint;
    scalar_t LF_KFE_joint;

    scalar_t LH_HAA_joint;
    scalar_t LH_HFE_joint;
    scalar_t LH_KFE_joint;

    scalar_t RF_HAA_joint;
    scalar_t RF_HFE_joint;
    scalar_t RF_KFE_joint;

    scalar_t RH_HAA_joint;
    scalar_t RH_HFE_joint;
    scalar_t RH_KFE_joint;
  };

  struct ObsScales
  {
    scalar_t lin_vel;
    scalar_t ang_vel;
    scalar_t dof_pos;
    scalar_t dof_vel;
    scalar_t height_measurements;
  };

  scalar_t clip_actions;
  scalar_t clip_obs;

  InitState init_state;
  ObsScales obs_scales;
  ControlCfg control_cfg;
};

class LeggedRLController
  : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                          ContactSensorInterface>
{
  using tensor_element_t = float;
  using ObsScales = RobotCfg::ObsScales;
  using ControlCfg = RobotCfg::ControlCfg;
  using InitState = RobotCfg::InitState;

  enum class Mode : uint8_t
  {
    LIE,
    STAND,
    WALK
  };

public:
  LeggedRLController() = default;
  ~LeggedRLController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

protected:
  virtual void setupStateEstimate(const std::string& task_file, const std::string& urdf_file,
                                  const std::string& reference_file);
  bool parseCfg(ros::NodeHandle& nh);
  void loadPolicyModel(const std::string& policy_file_path);
  void computeActions();
  void computeObservation(const ros::Time& time, const ros::Duration& period);

  void baseStateRecCallback(const gazebo_msgs::ModelStates& msg);

  std::shared_ptr<StateEstimateBase> state_estimate_;

  std::vector<scalar_t> init_joint_angles_;
  std::vector<HybridJointHandle> hybrid_joint_handles_;
  hardware_interface::ImuSensorHandle imu_sensor_handles_;
  std::vector<ContactSensorHandle> contact_handles_;

private:
  std::atomic_bool controller_running_;
  int loop_count_;
  Mode mode_;

  // publisher & subscriber
  ros::Subscriber base_state_sub_;

  // stand
  scalar_t stand_percent_;
  scalar_t stand_duration_;

  // onnx policy model
  std::string policy_file_path_;
  std::shared_ptr<Ort::Env> onnx_env_prt_;
  std::unique_ptr<Ort::Session> session_ptr_;
  std::vector<const char*> input_names_;
  std::vector<const char*> output_names_;
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::vector<int64_t>> output_shapes_;

  int actions_size_;
  int observation_size_;
  std::vector<tensor_element_t> actions_;
  std::vector<tensor_element_t> observations_;

  // temp state
  RobotCfg robot_cfg_{};
  vector_t rbd_state_;
  Eigen::Matrix<scalar_t, 3, 1> command_;
  Eigen::Matrix<scalar_t, 3, 1> base_lin_vel_;
  Eigen::Matrix<scalar_t, 3, 1> base_position_;
  Eigen::Matrix<scalar_t, 12, 1> last_actions_;
  Eigen::Matrix<scalar_t, 12, 1> default_joint_angles_;
};

}  // namespace legged
