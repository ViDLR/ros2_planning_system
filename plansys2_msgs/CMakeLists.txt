cmake_minimum_required(VERSION 3.5)

project(plansys2_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Action.msg"
  "msg/ActionExecution.msg"
  "msg/ActionExecutionInfo.msg"
  "msg/ActionPerformerStatus.msg"
  "msg/DurativeAction.msg"
  "msg/Knowledge.msg"
  "msg/Node.msg"
  "msg/Param.msg"
  "msg/Plan.msg"
  "msg/Failure.msg"
  "msg/PlanItem.msg"
  "msg/FailureItem.msg"
  "msg/Tree.msg"
  "msg/Team.msg"
  "srv/AddProblem.srv"
  "srv/AddProblemGoal.srv"
  "srv/AffectNode.srv"
  "srv/AffectParam.srv"
  "srv/ExistNode.srv"
  "srv/GetDomain.srv"
  "srv/GetDomainActions.srv"
  "srv/GetDomainActionDetails.srv"
  "srv/GetDomainDurativeActionDetails.srv"
  "srv/GetDomainName.srv"
  "srv/GetDomainTypes.srv"
  "srv/GetDomainConstants.srv"
  "srv/GetNodeDetails.srv"
  "srv/GetPlan.srv"
  "srv/GetOrderedSubGoals.srv"
  "srv/GetProblem.srv"
  "srv/GetProblemGoal.srv"
  "srv/GetProblemInstances.srv"
  "srv/GetProblemInstanceDetails.srv"
  "srv/GetStates.srv"
  "srv/IsProblemGoalSatisfied.srv"
  "srv/RemoveProblemGoal.srv"
  "srv/ClearProblemKnowledge.srv"
  "srv/ValidateDomain.srv"
  "srv/RepairKnowledge.srv"
  "srv/UpdateExecutors.srv"
  "srv/StartTeams.srv"
  "srv/StopTeams.srv"
  "srv/SubscribeKnowledgeTopics.srv"
  "action/ExecutePlan.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
