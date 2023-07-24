//
// Created by luohx on 23-7-21.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <pluginlib/class_list_macros.hpp>

#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "legged_common/third_party/json.hpp"
#include "legged_controllers/MotionDataGenerator.h"

namespace legged {
using namespace ocs2;
MotionDataGenerator::MotionDataGenerator(ros::NodeHandle& nh, const std::string& topicPrefix, const std::string& taskFile,
                                         const std::string& urdfFile, const std::string& referenceFile) {
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

  // command velocity publisher
  cmdVelPub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  obsSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);
}

void MotionDataGenerator::generate(const vector_t& cmdVel, scalar_t duration) {
  while (latestObservation_.time == 0) {
    ros::spinOnce();
  }
  auto startTime = std::chrono::steady_clock::now();
  auto endTime = startTime + std::chrono::duration<scalar_t>(duration);

  scalar_t frequency = 50.0;  // Hz
  std::chrono::microseconds period(static_cast<int>(1e6 / frequency));

  std::vector<vector_t> dataFrames;

  while (std::chrono::steady_clock::now() < endTime) {
    auto loopStartTime = std::chrono::steady_clock::now();

    geometry_msgs::Twist msg;
    msg.linear.x = cmdVel(0);
    msg.linear.y = cmdVel(1);
    msg.linear.z = cmdVel(2);
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = cmdVel(3);
    cmdVelPub_.publish(msg);

    vector_t dataFrame = convertDataFormat(latestObservation_);
    dataFrames.push_back(dataFrame);
    std::cerr << "latest time : " << latestObservation_.time << std::endl;

    ros::spinOnce();

    auto loopEndTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(loopEndTime - loopStartTime);

    if (elapsedTime < period) {
      std::this_thread::sleep_for(period - elapsedTime);
    }
  }

  saveData(dataFrames);
}

vector_t MotionDataGenerator::convertDataFormat(const SystemObservation& observation) {
  vector_t basePos = observation.state.head(3);
  vector_t zyxAngle = observation.state.segment<3>(3);
  Eigen::Quaternion<scalar_t> quat = getQuaternionFromEulerAnglesZyx<scalar_t>(zyxAngle);
  matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles<scalar_t>(zyxAngle).inverse();
  vector_t linVel = inverseRot * observation.state.head(3);
  vector_t angVel = inverseRot * observation.state.segment<3>(3);
  vector_t jointPos = observation.state.segment<12>(12);
  vector_t jointVel = observation.input.segment<12>(12);

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
  double frameDuration = 0.021;
  bool enableCycleOffsetPosition = true;
  bool enableCycleOffsetRotation = true;
  double motionWeight = 0.5;

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
  std::ofstream file("/home/luohx/catkin_ws/output/motion_record_output.json");
  if (!file) {
    std::cerr << "Error: could not open file for writing.\n";
  } else {
    file << j.dump(4);
    if (!file) {
      std::cerr << "Error: failed to write to file.\n";
    }
  }
  std::cerr << "record file finsish" << std::endl;
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

  MotionDataGenerator motionDataGenerator(nodeHandle, robotName, taskFile, urdfFile, referenceFile);
  vector_t cmdVel(4);
  cmdVel << 0.5, 0, 0, 0;
  motionDataGenerator.generate(cmdVel, 30);

  return 0;
}
