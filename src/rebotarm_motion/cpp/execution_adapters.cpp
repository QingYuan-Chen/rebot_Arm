/**
 * 文件职责：MoveIt 响应适配器调用纯计算服务，并把计划能力绑定带到最终 FJT。
 * 对外接口：RetimeExecutionCurve、BoundSimpleControllerManager 两个 pluginlib 插件。
 * 安全约束：不重写设备限值，不在执行阶段重定时；服务总期限 2 s，未知计划拒绝。
 *   RobotTrajectory 不保存 JointTrajectory header，所以仅写 header 会丢绑定。
 *   缓存分别保存 MoveIt 重编码匹配值和服务权威纳秒轨迹；匹配后下发后者。
 *   设备重连后的旧绑定仍交后端拒绝；缓存淘汰或进程重启后须重新规划。
 * 文件结构：计划绑定缓存 → 有界服务调用 → 响应插件 → 原控制器管理器装饰器。
 */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <moveit/controller_manager/controller_manager.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <rebotarm_msgs/srv/retime_arm_trajectory.hpp>

namespace rebotarm_motion
{
using Trajectory = trajectory_msgs::msg::JointTrajectory;
using Manager = moveit_controller_manager::MoveItControllerManager;
using Handle = moveit_controller_manager::MoveItControllerHandle;
using Status = moveit_controller_manager::ExecutionStatus;
using Service = rebotarm_msgs::srv::RetimeArmTrajectory;
std::mutex binding_mutex;
struct BoundPlan { Trajectory comparison; Trajectory authoritative; bool executable; };
std::deque<BoundPlan> plans;

bool same_values(const std::vector<double>& a, const std::vector<double>& b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (!std::isfinite(a[i]) || !std::isfinite(b[i]) || std::abs(a[i]-b[i]) > 1e-12) return false;
  return true;
}

bool same_plan(const Trajectory& a, const Trajectory& b)
{
  if (a.joint_names != b.joint_names || a.points.size() != b.points.size()) return false;
  for (size_t i = 0; i < a.points.size(); ++i)
  {
    const auto& x = a.points[i]; const auto& y = b.points[i];
    const int64_t tx = int64_t(x.time_from_start.sec)*1000000000+x.time_from_start.nanosec;
    const int64_t ty = int64_t(y.time_from_start.sec)*1000000000+y.time_from_start.nanosec;
    if (std::abs(tx-ty) > 1 || !same_values(x.positions,y.positions) ||
        !same_values(x.velocities,y.velocities) || !same_values(x.accelerations,y.accelerations) ||
        !x.effort.empty() || !y.effort.empty()) return false;
  }
  return true;
}

class RetimeExecutionCurve : public planning_interface::PlanningResponseAdapter
{
  rclcpp::Node::SharedPtr parent_;
  std::string service_;
  bool reference_ = false;
public:
  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    parent_ = node;
    std::string ns = "rebotarm";
    node->get_parameter_or("arm_namespace", ns, ns);
    service_ = "/"+ns+"/retime_arm_trajectory";
    node->get_parameter_or(parameter_namespace+".execution_reference", reference_, false);
  }
  std::string getDescription() const override { return "Retime checked execution curve"; }
  void adapt(const planning_scene::PlanningSceneConstPtr&,
             const planning_interface::MotionPlanRequest& req,
             planning_interface::MotionPlanResponse& res) const override
  {
    if (!res.trajectory || res.trajectory->empty()) return;
    try
    {
      // 本次调用独占 executor；不加入 move_group 的回调组，避免同步等待自身回调。
      const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(2);
      static std::atomic<unsigned> serial{0};
      rclcpp::NodeOptions options;
      options.context(parent_->get_node_base_interface()->get_context()).use_global_arguments(false);
      auto node = std::make_shared<rclcpp::Node>("execution_retime_client_"+std::to_string(serial++), options);
      auto client = node->create_client<Service>(service_);
      rclcpp::ExecutorOptions executor_options;
      executor_options.context = parent_->get_node_base_interface()->get_context();
      rclcpp::executors::SingleThreadedExecutor executor(executor_options);
      executor.add_node(node);
      auto remaining = [&] { return deadline-std::chrono::steady_clock::now(); };
      if (!client->wait_for_service(remaining())) throw std::runtime_error("retiming service unavailable within 2 s");
      moveit_msgs::msg::RobotTrajectory message;
      res.trajectory->getRobotTrajectoryMsg(message);
      auto request = std::make_shared<Service::Request>();
      request->trajectory = message.joint_trajectory;
      request->planning_purpose = reference_ ? Service::Request::REFERENCE : Service::Request::FINAL_EXECUTION;
      request->velocity_scaling = req.max_velocity_scaling_factor > 0 ? req.max_velocity_scaling_factor : 1.;
      request->acceleration_scaling = req.max_acceleration_scaling_factor > 0 ? req.max_acceleration_scaling_factor : 1.;
      auto future = client->async_send_request(request);
      if (executor.spin_until_future_complete(future,remaining()) != rclcpp::FutureReturnCode::SUCCESS)
      {
        client->remove_pending_request(future);
        throw std::runtime_error("execution curve retiming exceeded total 2 s deadline");
      }
      const auto response = future.get();
      if (!response->success) throw std::runtime_error(response->message);
      res.trajectory->setRobotTrajectoryMsg(res.trajectory->getFirstWayPoint(),response->trajectory);
      // 用 MoveIt 再编码后的值登记，覆盖它的纳秒截断和关节顺序，执行端不重新计算。
      res.trajectory->getRobotTrajectoryMsg(message);
      std::lock_guard<std::mutex> lock(binding_mutex);
      plans.push_back({message.joint_trajectory, response->trajectory, response->executable});
      if (plans.size() > 128) plans.pop_front();
      RCLCPP_INFO(parent_->get_logger(), "execution curve planned: scale=%.6f final_time=%.6f s",
                  response->time_scale,res.trajectory->getDuration());
    }
    catch (const std::exception& error)
    {
      RCLCPP_ERROR(parent_->get_logger(), "execution curve planning rejected: %s",error.what());
      res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      res.trajectory.reset();
    }
  }
};

class BoundHandle : public Handle
{
  moveit_controller_manager::MoveItControllerHandlePtr inner_;
  bool rejected_ = false;
public:
  explicit BoundHandle(moveit_controller_manager::MoveItControllerHandlePtr inner)
    : Handle(inner->getName()), inner_(std::move(inner)) {}
  bool sendTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) override
  {
    auto bound = trajectory;
    rejected_ = true;
    {
      std::lock_guard<std::mutex> lock(binding_mutex);
      for (auto p = plans.rbegin(); p != plans.rend(); ++p)
        if (same_plan(p->comparison,trajectory.joint_trajectory))
        {
          if (p->executable) { bound.joint_trajectory = p->authoritative; rejected_ = false; }
          break;
        }
    }
    if (rejected_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rebotarm_motion"),"no bound final plan; re-plan before Execute");
      return false;
    }
    return inner_->sendTrajectory(bound);
  }
  bool cancelExecution() override { return inner_->cancelExecution(); }
  bool waitForExecution(const rclcpp::Duration& timeout) override { return inner_->waitForExecution(timeout); }
  Status getLastExecutionStatus() override { return rejected_ ? Status(Status::FAILED) : inner_->getLastExecutionStatus(); }
};

class BoundSimpleControllerManager : public Manager
{
  // loader 必须比插件实例晚析构；实际 Action 与控制器映射仍由原 simple manager 拥有。
  std::shared_ptr<pluginlib::ClassLoader<Manager>> loader_;
  std::shared_ptr<Manager> inner_;
  std::map<std::string,moveit_controller_manager::MoveItControllerHandlePtr> handles_;
public:
  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    loader_ = std::make_shared<pluginlib::ClassLoader<Manager>>("moveit_core","moveit_controller_manager::MoveItControllerManager");
    inner_ = loader_->createSharedInstance("moveit_simple_controller_manager/MoveItSimpleControllerManager");
    inner_->initialize(node);
  }
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    auto found = handles_.find(name);
    if (found != handles_.end()) return found->second;
    auto handle = inner_->getControllerHandle(name);
    if (!handle) return nullptr;
    return handles_.emplace(name,std::make_shared<BoundHandle>(handle)).first->second;
  }
  void getControllersList(std::vector<std::string>& names) override { inner_->getControllersList(names); }
  void getActiveControllers(std::vector<std::string>& names) override { inner_->getActiveControllers(names); }
  void getControllerJoints(const std::string& name,std::vector<std::string>& joints) override { inner_->getControllerJoints(name,joints); }
  ControllerState getControllerState(const std::string& name) override { return inner_->getControllerState(name); }
  bool switchControllers(const std::vector<std::string>& a,const std::vector<std::string>& b) override
  { return inner_->switchControllers(a,b); }
};
}
PLUGINLIB_EXPORT_CLASS(rebotarm_motion::RetimeExecutionCurve,planning_interface::PlanningResponseAdapter)
PLUGINLIB_EXPORT_CLASS(rebotarm_motion::BoundSimpleControllerManager,moveit_controller_manager::MoveItControllerManager)
