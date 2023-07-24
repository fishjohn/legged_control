//
// Created by luohx on 23-7-21.
//

#pragma once

#include <geometry_msgs/Twist.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include <ros/ros.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class MotionDataGenerator final {
  using json = nlohmann::json;

 public:
  MotionDataGenerator(ros::NodeHandle& nh, const std::string& topicPrefix, const std::string& taskFile, const std::string& urdfFile,
                      const std::string& referenceFile);

  void generate(const vector_t& cmdVel, scalar_t duration);

  void saveData(const std::vector<vector_t>& dataFrames);

  vector_t convertDataFormat(const SystemObservation& observation);

 private:
  ::ros::Publisher cmdVelPub_;
  ::ros::Subscriber obsSub_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;

  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
};
}  // namespace legged