//
// Created by luohx on 22-12-5.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <legged_common/hardware_interface/hybrid_joint_interface.h>
#include <legged_common/hardware_interface/contact_sensor_interface.h>
#include <legged_estimation/state_estimate_base.h>
#include <legged_estimation/linear_kalman_filter.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float32MultiArray.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

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
    double LF_HAA_joint;
    double LF_HFE_joint;
    double LF_KFE_joint;

    double LH_HAA_joint;
    double LH_HFE_joint;
    double LH_KFE_joint;

    double RF_HAA_joint;
    double RF_HFE_joint;
    double RF_KFE_joint;

    double RH_HAA_joint;
    double RH_HFE_joint;
    double RH_KFE_joint;
  };

  struct ObsScales
  {
    double lin_vel;
    double ang_vel;
    double dof_pos;
    double dof_vel;
    double height_measurements;
  };

  double clip_actions;
  double clip_obs;

  InitState init_state;
  ObsScales obs_scales;
  ControlCfg control_cfg;
};

class LeggedRLController
  : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                          ContactSensorInterface>
{
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
  bool parseCfg(ros::NodeHandle& nh);
  std::vector<double> computeObservation(const ros::Time& time, const ros::Duration& period);
  void publishObsMsg(const std::vector<double>& msg, const ros::Time& time);

  void setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                            const std::string& reference_file, bool verbose);
  void setupStateEstimate(LeggedInterface& legged_interface, const std::vector<HybridJointHandle>& hybrid_joint_handles,
                          const std::vector<ContactSensorHandle>& contact_sensor_handles,
                          const hardware_interface::ImuSensorHandle& imu_sensor_handle);

  void baseStateRecCallback(const gazebo_msgs::ModelStates& msg);
  void policyActionRecCallback(const std_msgs::Float32MultiArray& msg);

  std::shared_ptr<LeggedInterface> legged_interface_;
  std::shared_ptr<StateEstimateBase> state_estimate_;

  SystemObservation current_observation_;
  std::vector<HybridJointHandle> hybrid_joint_handles_;
  hardware_interface::ImuSensorHandle imu_sensor_handle_;
  std::vector<double> init_joint_angles_;

private:
  std::atomic_bool controller_running_;
  Mode mode_;

  // publisher & subscriber
  ros::Subscriber base_state_sub_;
  ros::Subscriber policy_action_sub_;
  realtime_tools::RealtimeBuffer<std_msgs::Float32MultiArray> actions_buffer_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>> obs_pub_;
  ros::Time last_obs_pub_;
  std::vector<double> action_vector_;
  bool action_topic_updated_;

  // stand
  double stand_percent_;
  double stand_duration_;

  // temp state
  RobotCfg robot_cfg_{};
  Eigen::Matrix<double, 3, 1> command_;
  Eigen::Matrix<double, 3, 1> base_lin_vel_;
  Eigen::Matrix<double, 3, 1> base_position_;
  Eigen::Matrix<double, 12, 1> last_actions_;
  Eigen::Matrix<double, 12, 1> default_joint_angles_;
};

}  // namespace legged
