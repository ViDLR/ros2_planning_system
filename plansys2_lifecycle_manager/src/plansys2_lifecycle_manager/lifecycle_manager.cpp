#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

namespace plansys2
{

LifecycleServiceClient::LifecycleServiceClient(
  const std::string & node_name, const std::string & managed_node, const std::string & namespace_)
: Node(node_name, namespace_), managed_node_(managed_node){
  RCLCPP_INFO(this->get_logger(), "Created LifecycleServiceClient with name %s and namespace: %s", node_name.c_str(),namespace_.c_str());
}


void LifecycleServiceClient::init()
{
  RCLCPP_INFO(this->get_logger(), "Initializing LifecycleServiceClient for %s", managed_node_.c_str());
  client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(managed_node_ + "/get_state");
  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(managed_node_ + "/change_state");

  while (!client_get_state_->wait_for_service(std::chrono::seconds(1)) ||
         !client_change_state_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(), "Waiting for lifecycle services...");
  }

  RCLCPP_INFO(this->get_logger(), "Initialized LifecycleServiceClient for %s", managed_node_.c_str());
}

unsigned int LifecycleServiceClient::get_state(std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future_result = client_get_state_->async_send_request(request);

  if (wait_for_result(future_result, time_out) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get state of %s", managed_node_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  return future_result.get()->current_state.id;
}

bool LifecycleServiceClient::change_state(std::uint8_t transition, std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  auto future_result = client_change_state_->async_send_request(request);

  if (wait_for_result(future_result, time_out) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Failed to change state of %s", managed_node_.c_str());
    return false;
  }

  return future_result.get()->success;
}


bool startup_function(
  std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
  std::chrono::seconds timeout)
{
  for (const auto & [name, client] : manager_nodes) {
    RCLCPP_INFO(rclcpp::get_logger("lifecycle_manager"), "Configuring node %s", name.c_str());

    if (!client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, timeout)) {
      RCLCPP_ERROR(rclcpp::get_logger("lifecycle_manager"), "Failed to configure %s", name.c_str());
      return false;
    }

    while (client->get_state(timeout) != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_INFO(rclcpp::get_logger("lifecycle_manager"), "Waiting for INACTIVE state for %s", name.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(rclcpp::get_logger("lifecycle_manager"), "Activating node %s", name.c_str());
    if (!client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, timeout)) {
      RCLCPP_ERROR(rclcpp::get_logger("lifecycle_manager"), "Failed to activate %s", name.c_str());
      return false;
    }
  }
  return true;
}

}  // namespace plansys2
