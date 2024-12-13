// #include <chrono>
// #include <future>
// #include <map>
// #include <memory>
// #include <string>
// #include <vector>

// #include "rclcpp/rclcpp.hpp"
// #include "plansys2_lifecycle_manager/lifecycle_manager.hpp"
// #include "plansys2_msgs/srv/update_executors.hpp"

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);

//     std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;

//     manager_nodes["domain_expert"] = std::make_shared<plansys2::LifecycleServiceClient>(
//         "domain_expert_lc_mngr", "domain_expert");
//     manager_nodes["problem_expert"] = std::make_shared<plansys2::LifecycleServiceClient>(
//         "problem_expert_lc_mngr", "problem_expert");
//     manager_nodes["planner"] = std::make_shared<plansys2::LifecycleServiceClient>(
//         "planner_lc_mngr", "planner");

//     rclcpp::executors::SingleThreadedExecutor exe;
//     for (auto & manager_node : manager_nodes) {
//       manager_node.second->init();
//       exe.add_node(manager_node.second);
//     }

//     std::shared_future<bool> startup_future = std::async(
//       std::launch::async,
//       std::bind(plansys2::startup_function, manager_nodes, std::chrono::seconds(5)));
//     exe.spin_until_future_complete(startup_future);

//     if (!startup_future.get()) {
//       RCLCPP_ERROR(
//         rclcpp::get_logger("plansys2_lifecycle_manager"),
//         "Failed to start plansys2!");
//       rclcpp::shutdown();
//       return -1;
//     }

//     rclcpp::shutdown();

//     return 0;
//   }


#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create node
    auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_node");

    // Declare managed_nodes parameter
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "List of nodes to manage in the lifecycle manager";
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
    node->declare_parameter<std::vector<std::string>>("managed_nodes", {}, descriptor);

    // Get managed_nodes
    std::vector<std::string> managed_nodes;
    node->get_parameter("managed_nodes", managed_nodes);

    if (managed_nodes.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No managed_nodes specified. Exiting.");
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "Managing the following nodes:");
    for (const auto &node_name : managed_nodes) {
        RCLCPP_INFO(node->get_logger(), " - %s", node_name.c_str());
    }

    // Create LifecycleServiceClients for each managed node
    std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;
    for (const auto &node_name : managed_nodes) {
        RCLCPP_INFO(node->get_logger(), "node name %s", node_name.c_str());
        manager_nodes[node_name] = std::make_shared<plansys2::LifecycleServiceClient>(
            node_name + "_lc_mngr", node_name);
    }

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
    for (auto & manager_node : manager_nodes) {
      manager_node.second->init();
      executor.add_node(manager_node.second);
    }

    std::shared_future<bool> startup_future = std::async(
      std::launch::async,
      std::bind(plansys2::startup_function, manager_nodes, std::chrono::seconds(5)));
    executor.spin_until_future_complete(startup_future);

    if (startup_future.get()) {
      executor.spin();
      rclcpp::shutdown();
      return 0;
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("plansys2_bringup"),
        "Failed to start plansys2!");
      rclcpp::shutdown();
      return -1;
    }

    return 0;
}
