// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_
#define PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <map>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/srv/get_ordered_sub_goals.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace plansys2
{

class ExecutorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ExecutePlan = plansys2_msgs::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ServerGoalHandle<ExecutePlan>;
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ExecutorNode();  // Default constructor
  ExecutorNode(const std::string &node_name, const std::string &namespace_ = ""); // New constructor


  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  void get_ordered_sub_goals_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Response> response);

  void get_plan_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response);

protected:
  bool cancel_plan_requested_;
  std::optional<plansys2_msgs::msg::Plan> current_plan_;
  std::optional<std::vector<plansys2_msgs::msg::Tree>> ordered_sub_goals_;

  std::string action_bt_xml_;
  std::string start_action_bt_xml_;
  std::string end_action_bt_xml_;
  pluginlib::ClassLoader<plansys2::BTBuilder> bt_builder_loader_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr
    execution_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::Plan>::SharedPtr executing_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::Knowledge>::SharedPtr knowledge_pub_;

  rclcpp_action::Server<ExecutePlan>::SharedPtr execute_plan_action_server_;
  rclcpp::Service<plansys2_msgs::srv::GetOrderedSubGoals>::SharedPtr
    get_ordered_sub_goals_service_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr dotgraph_pub_;

  std::optional<std::vector<plansys2_msgs::msg::Tree>> getOrderedSubGoals();

  rclcpp::Service<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_service_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecutePlan::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  void execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);
  
  void notify_manager_node_with_knowledge();

  void handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  std::vector<plansys2_msgs::msg::ActionExecutionInfo> get_feedback_info(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);

  void print_execution_info(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> exec_info);
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_
