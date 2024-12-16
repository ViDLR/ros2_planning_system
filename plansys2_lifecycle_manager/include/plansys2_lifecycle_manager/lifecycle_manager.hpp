#ifndef PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= end) {
      return std::future_status::timeout;
    }
  }
  return future.wait_for(std::chrono::seconds(0));
}

class LifecycleServiceClient : public rclcpp::Node
{
public:
  // Updated constructor with namespace support
  explicit LifecycleServiceClient(
    const std::string & node_name,
    const std::string & managed_node,
    const std::string & namespace_ = "");

  void init();

  unsigned int get_state(std::chrono::seconds time_out = std::chrono::seconds(3));

  bool change_state(
    std::uint8_t transition,
    std::chrono::seconds time_out = std::chrono::seconds(3));

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  std::string managed_node_;
};

bool startup_function(
  std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
  std::chrono::seconds timeout);

}  // namespace plansys2

#endif  // PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
