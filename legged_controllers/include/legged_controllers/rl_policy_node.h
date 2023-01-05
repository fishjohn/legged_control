//
// Created by luohx on 22-12-21.
//

#pragma once

#include <torch/torch.h>
#include <torch/script.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <realtime_tools/realtime_publisher.h>

namespace legged
{

constexpr int obs_tensor_size = 235;
constexpr int action_size = 12;

class RLPolicyNode final
{
public:
  RLPolicyNode() = default;
  ~RLPolicyNode() = default;
  bool init(ros::NodeHandle& nh);
  void generateAction();

protected:
  void publishAction(const std::vector<float>& actions);
  void updateObsCallback(const std_msgs::Float32MultiArray& msg);
  void generateActionCallback(const ros::TimerEvent& time_obj);

private:
  bool on_walking_;

  std::string policy_file_;
  torch::jit::script::Module policy_;
  std::vector<torch::jit::IValue> obs_tensor_;

  ros::Subscriber obs_sub_;
  ros::Timer action_send_timer_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>> action_pub_;
};
}  // namespace legged