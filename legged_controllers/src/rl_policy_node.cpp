//
// Created by luohx on 22-12-21.
//

#include "legged_controllers/rl_policy_node.h"

using namespace legged;

bool RLPolicyNode::init(ros::NodeHandle& nh)
{
  std::string module_path;
  nh.getParam("/module/module_path", module_path);
  ROS_INFO_STREAM("RL controller load module from path: " << module_path);

  // load policy from module
  try
  {
    policy_ = torch::jit::load(module_path);
  }
  catch (const c10::Error& e)
  {
    ROS_ERROR_STREAM("error loading the model");
    return false;
  }
  // Info module Parameters
  ROS_INFO_STREAM("RL_controller load policy successfully !!!\r\n ");
  ROS_INFO_STREAM("Parameters in the model:");
  ROS_INFO_STREAM("####" << std::string(25, '='));
  for (const auto& pair : policy_.named_parameters())
  {
    ROS_INFO_STREAM(pair.name << " " << pair.value.sizes());
  }
  ROS_INFO_STREAM("####" << std::string(25, '='));

  on_walking_ = false;

  obs_sub_ = nh.subscribe("/rl_controller/obs", 1, &RLPolicyNode::updateObsCallback, this);
  action_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>(nh, "/rl_controller/action", 10));
  action_pub_->msg_.data.resize(action_size);
  double action_sending_freq = 200;
  action_send_timer_ =
      nh.createTimer(ros::Duration(1.0 / action_sending_freq), &RLPolicyNode::generateActionCallback, this);

  return true;
}

void RLPolicyNode::updateObsCallback(const std_msgs::Float32MultiArray& msg)
{
  obs_tensor_.clear();
  obs_tensor_.emplace_back(torch::tensor(msg.data));

  if (!on_walking_)
  {
    on_walking_ = true;
  }
}

void RLPolicyNode::generateActionCallback(const ros::TimerEvent& time_obj)
{
  if (on_walking_)
  {
    generateAction();
  }
}

void RLPolicyNode::generateAction()
{
  auto output = policy_.forward(obs_tensor_).toTensor();
  auto action = std::vector<float>(output.data_ptr<float>(), output.data_ptr<float>() + output.numel());
  publishAction(action);
}

void RLPolicyNode::publishAction(const std::vector<float>& actions)
{
  if (action_pub_->trylock())
  {
    for (size_t i = 0; i < action_size; i++)
    {
      action_pub_->msg_.data[i] = actions.at(i);
    }
    action_pub_->unlockAndPublish();
  }
}

int main(int argc, char* argv[])
{
  // Initialize ros node
  ::ros::init(argc, argv, "rl_policy_node");
  ::ros::NodeHandle nh;

  RLPolicyNode rl_policy_node;
  if (!rl_policy_node.init(nh))
  {
    ROS_ERROR_STREAM("RL Policy Node inits fail, error occur");
    return -1;
  }

  ros::spin();
  // Successful exit
  return 0;
}
