//
// Created by luohx on 23-7-21.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <boost/filesystem.hpp>

#include "legged_common/third_party/json.hpp"
#include "legged_controllers/MotionDataGenerator.h"

namespace legged {
using namespace ocs2;
MotionDataGenerator::MotionDataGenerator(ros::NodeHandle& nh, const std::string& topicPrefix, const std::string& taskFile,
                                         const std::string& urdfFile, const std::string& referenceFile, std::vector<RecordTask>& tasks)
    : recordTasks_(std::move(tasks)), currentTaskIndex_(0), beginRecord_(false) {
  ModelSettings modelSettings;
  modelSettings = loadModelSettings(taskFile, "model_settings", false);

  // PinocchioInterface
  pinocchioInterfacePtr_ =
      std::make_unique<PinocchioInterface>(centroidal_model::createPinocchioInterface(urdfFile, modelSettings.jointNames));

  // CentroidalModelInfo
  info_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile), modelSettings.contactNames3DoF,
      modelSettings.contactNames6DoF);

  // PinocchioEndEffectorKinematics
  CentroidalModelPinocchioMapping pinocchioMapping(info_);
  eeKinematics_ =
      std::make_unique<PinocchioEndEffectorKinematics>(*pinocchioInterfacePtr_, pinocchioMapping, modelSettings.contactNames3DoF);
  eeKinematics_->setPinocchioInterface(*pinocchioInterfacePtr_);
  // CentroidalModelRbdConversions
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(*pinocchioInterfacePtr_, info_);

  // command velocity publisher
  cmdVelPub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  // observation subscriber
  obsSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 10, &MotionDataGenerator::observationCallback, this);
}

void MotionDataGenerator::observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  //  std::lock_guard<std::mutex> lock(latestObservationMutex_);

  if (beginRecord_) {
    const auto curObservation = ros_msg_conversions::readObservationMsg(*msg);

    const scalar_t lastObsTime = latestObservation_.time;
    const scalar_t curObsTime = curObservation.time;
    assert(lastObsTime <= curObsTime);

    if (curObsTime - lastObsTime >= recordTasks_[currentTaskIndex_].frameDuration) {
      vector_t dataFrame = convertDataFormat(curObservation);
      recordTasks_[currentTaskIndex_].traj.push_back(dataFrame);
      latestObservation_ = curObservation;
    }
  }
}

void MotionDataGenerator::generate() {
  for (size_t i = 0; i < recordTasks_.size(); i++) {
    currentTaskIndex_ = i;

    auto startTime = std::chrono::steady_clock::now();
    auto endTime = startTime + std::chrono::duration<scalar_t>(recordTasks_[i].trajTime);
    scalar_t frequency = 100.0;  // hz
    scalar_t dt = 1 / frequency;
    std::chrono::microseconds period(static_cast<int>(1e6 / frequency));

    vector3_t accel = recordTasks_[i].middleTimeVel / (recordTasks_[i].trajTime / 2);
    vector3_t cmdVel(0.0, 0.0, 0.0);

    beginRecord_ = true;
    while (std::chrono::steady_clock::now() < endTime) {
      auto loopStartTime = std::chrono::steady_clock::now();
      if (endTime - loopStartTime >= std::chrono::duration<scalar_t>(recordTasks_[i].trajTime / 2)) {
        cmdVel += accel * dt;
      } else {
        cmdVel -= accel * dt;
      }

      geometry_msgs::Twist msg;
      msg.linear.x = cmdVel(0);
      msg.linear.y = cmdVel(1);
      msg.linear.z = 0;
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = cmdVel(2);
      cmdVelPub_.publish(msg);

      ros::spinOnce();
      auto loopEndTime = std::chrono::steady_clock::now();
      auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(loopEndTime - loopStartTime);

      if (elapsedTime < period) {
        std::this_thread::sleep_for(period - elapsedTime);
      }
    }
    beginRecord_ = false;
    saveData(recordTasks_[currentTaskIndex_].traj);
  }
}

vector_t MotionDataGenerator::convertDataFormat(const SystemObservation& observation) {
  const auto& state = observation.state;
  const auto& input = observation.input;

  vector_t basePos = centroidal_model::getBasePose(state, info_).head(3);
  vector_t zyxAngle = centroidal_model::getBasePose(state, info_).tail(3);
  Eigen::Quaternion<scalar_t> quat = getQuaternionFromEulerAnglesZyx<scalar_t>(zyxAngle);
  matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles<scalar_t>(zyxAngle).inverse();
  vector_t rbdState = rbdConversions_->computeRbdStateFromCentroidalModel(state, input);
  vector_t linVel = inverseRot * rbdState.segment<3>(info_.generalizedCoordinatesNum + 3);
  vector_t angVel = inverseRot * rbdState.segment<3>(info_.generalizedCoordinatesNum);
  vector_t jointPos = centroidal_model::getJointAngles(state, info_);
  vector_t jointVel = centroidal_model::getJointVelocities(input, info_);

  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  size_t actuatedDofNum = info_.actuatedDofNum;

  vector_t qPino(info_.generalizedCoordinatesNum);
  vector_t vPino(info_.generalizedCoordinatesNum);
  qPino.setZero();
  qPino.segment<3>(3) = zyxAngle;  // Only set orientation, let position in origin.
  qPino.tail(actuatedDofNum) = jointPos;

  vPino.setZero();
  vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qPino.segment<3>(3), angVel);  // Only set angular velocity, let linear velocity be zero
  vPino.tail(actuatedDofNum) = jointVel;

  pinocchio::forwardKinematics(model, data, qPino, vPino);
  pinocchio::updateFramePlacements(model, data);

  vector_t eePos(12);
  vector_t eeVel(12);
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    eePos.segment<3>(i * 3) = eeKinematics_->getPosition(vector_t())[i];
    eeVel.segment<3>(i * 3) = eeKinematics_->getVelocity(vector_t(), vector_t())[i];
  }

  vector_t dataFrame(61);
  dataFrame << basePos, quat.x(), quat.y(), quat.z(), quat.w(), jointPos, eePos, linVel, angVel, jointVel, eeVel;

  return dataFrame;
}

void MotionDataGenerator::saveData(const std::vector<vector_t>& dataFrames) {
  std::string loopMode = "Wrap";
  double frameDuration = 0.02;
  bool enableCycleOffsetPosition = true;
  bool enableCycleOffsetRotation = true;
  double motionWeight = recordTasks_[currentTaskIndex_].motionWeight;

  json j;
  j["LoopMode"] = loopMode;
  j["FrameDuration"] = frameDuration;
  j["EnableCycleOffsetPosition"] = enableCycleOffsetPosition;
  j["EnableCycleOffsetRotation"] = enableCycleOffsetRotation;
  j["MotionWeight"] = motionWeight;

  json arrayJson;
  for (auto& dataFrame : dataFrames) {
    json singleFrameJson;
    for (int i = 0; i < dataFrame.size(); ++i) {
      singleFrameJson.push_back(dataFrame(i));
    }
    arrayJson.push_back(singleFrameJson);
  }
  j["Frames"] = arrayJson;
  std::ofstream file(recordTasks_[currentTaskIndex_].recordFilePath);
  if (!file) {
    std::cerr << "Error: could not open file for writing.\n";
  } else {
    file << j.dump(4);
    if (!file) {
      std::cerr << "Error: failed to write to file.\n";
    }
  }
  std::cerr << "record file: " << recordTasks_[currentTaskIndex_].recordFilePath << " finsish" << std::endl;
}

}  // namespace legged

using namespace legged;

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_motion_record");
  ::ros::NodeHandle nodeHandle;

  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/referenceFile", referenceFile);

  std::string dataSaveDir;
  nodeHandle.getParam("/dataSetSaveDir", dataSaveDir);

  scalar_t totalTime = 30.0;  // s
  std::vector<std::string> trajNames = {"forward",       "backward",       "lateral_left",       "lateral_right",
                                        "left_steering", "right_steering", "combined_locomotion"};
  size_t trajNum = trajNames.size();
  scalar_t trajTime = totalTime / trajNum;
  std::pair<scalar_t, scalar_t> linxVelRange{-1.0, 1.0};  // min max [m/s]
  std::pair<scalar_t, scalar_t> linyVelRange{-0.5, 0.5};  // min max [m/s]
  std::pair<scalar_t, scalar_t> yawRateRange{-1, 1};      // min max [rad/s]
  std::vector<vector3_t> middleTimeVels = {
      vector3_t{linxVelRange.second, 0, 0},
      vector3_t{linxVelRange.first, 0, 0},
      vector3_t{0, linyVelRange.second, 0},
      vector3_t{0, linyVelRange.first, 0},
      vector3_t{0, 0, yawRateRange.second},
      vector3_t{0, 0, yawRateRange.first},
      vector3_t{linxVelRange.second, linyVelRange.second, yawRateRange.second},
  };

  std::vector<MotionDataGenerator::RecordTask> recordTasks;
  recordTasks.reserve(trajNum);

  for (size_t i = 0; i < trajNum; i++) {
    MotionDataGenerator::RecordTask task;
    task.trajName = trajNames[i];
    task.trajTime = trajTime;
    task.frameDuration = 0.02;
    task.middleTimeVel = middleTimeVels[i];
    task.motionWeight = 0.5;
    task.recordFilePath = dataSaveDir + trajNames[i] + ".json";
    recordTasks.push_back(task);
  }

  MotionDataGenerator motionDataGenerator(nodeHandle, robotName, taskFile, urdfFile, referenceFile, recordTasks);
  motionDataGenerator.generate();

  ::ros::spin();
  return 0;
}
