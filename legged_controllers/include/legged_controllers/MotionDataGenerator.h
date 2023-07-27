//
// Created by luohx on 23-7-21.
//

#pragma once

#include <geometry_msgs/Twist.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include <ros/ros.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class MotionDataGenerator final {
  using json = nlohmann::json;

 public:
  struct RecordTask {
   public:
    scalar_t trajTime;
    std::string trajName;
    std::vector<vector_t> traj;
    scalar_t frameDuration;
    vector3_t middleTimeVel;  // lin_vel_x, lin_vel_y, yaw_rate

    scalar_t motionWeight;
    std::string recordFilePath;
  };

  MotionDataGenerator(ros::NodeHandle& nh, const std::string& topicPrefix, const std::string& taskFile, const std::string& urdfFile,
                      const std::string& referenceFile, std::vector<RecordTask>& tasks);

  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

  void generate();

  void saveData(const std::vector<vector_t>& dataFrames);

  vector_t convertDataFormat(const SystemObservation& observation);

 private:
  std::vector<vector_t> recordDatas_;
  std::vector<RecordTask> recordTasks_;
  mutable bool beginRecord_;
  scalar_t beginRecordTime_;

  ::ros::Publisher cmdVelPub_;
  ::ros::Subscriber obsSub_;

  mutable std::mutex latestObservationMutex_;
  mutable size_t currentTaskIndex_;
  SystemObservation latestObservation_;

  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
};
}  // namespace legged