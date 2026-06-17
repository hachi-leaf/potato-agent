#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdio>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "potato_interface/action/output.hpp"

using namespace std::chrono_literals;
using Output = potato_interface::action::Output;
using GoalHandleOutput = rclcpp_action::ServerGoalHandle<Output>;

class CmdExecNode : public rclcpp::Node {
public:
  explicit CmdExecNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("cmd_exec_node", options) {
    declare_parameter("agent_name", "agent");
    std::string agent_name = get_parameter("agent_name").as_string();
    std::string func_name = "cmd_exec";

    // 发布 help 话题（transient local，确保 output_info 能收到）
    help_pub_ = create_publisher<std_msgs::msg::String>(
      "/" + agent_name + "/output/" + func_name + "/help",
      rclcpp::QoS(1).transient_local());
    auto help_msg = std_msgs::msg::String();
    help_msg.data =
      "TYPE:action\n"
      "Command Executor\n"
      "输入 shell 命令，将执行并返回 stdout/stderr。\n"
      "示例: goal_text = \"ls -l\"\n";
    help_pub_->publish(help_msg);

    // 创建 Action 服务器
    action_server_ = rclcpp_action::create_server<Output>(
      this,
      "/" + agent_name + "/output/" + func_name,
      std::bind(&CmdExecNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CmdExecNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&CmdExecNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "CmdExec node started for agent '%s'", agent_name.c_str());
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr help_pub_;
  rclcpp_action::Server<Output>::SharedPtr action_server_;

  // 简单存储正在执行的 Goal 的取消标志，但此处为并发设计，我们使用每个 Goal 独立的标志
  struct ActiveGoal {
    std::shared_ptr<GoalHandleOutput> handle;
    std::atomic<bool> cancel_flag{false};
  };
  std::mutex goals_mutex_;
  std::unordered_map<std::shared_ptr<GoalHandleOutput>, std::shared_ptr<ActiveGoal>> active_goals_;

  // Goal 回调
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const Output::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received goal: %s", goal->goal_text.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleOutput> goal_handle) {
    RCLCPP_INFO(get_logger(), "Cancel requested");
    std::lock_guard lock(goals_mutex_);
    auto it = active_goals_.find(goal_handle);
    if (it != active_goals_.end()) {
      it->second->cancel_flag.store(true);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<GoalHandleOutput> goal_handle) {
    auto active = std::make_shared<ActiveGoal>();
    active->handle = goal_handle;
    {
      std::lock_guard lock(goals_mutex_);
      active_goals_[goal_handle] = active;
    }
    // 启动执行线程
    std::thread{std::bind(&CmdExecNode::execute, this, goal_handle, active)}.detach();
  }

  void execute(
      std::shared_ptr<GoalHandleOutput> goal_handle,
      std::shared_ptr<ActiveGoal> active) {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Output::Result>();

    std::string cmd = goal->goal_text + " 2>&1";
    std::array<char, 128> buffer;
    std::string full_output;

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
      result->success = false;
      result->error_msg = "Failed to open pipe";
      goal_handle->abort(result);
      cleanup(goal_handle);
      return;
    }

    // 逐行读取，发布反馈，同时检查取消
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
      // 检查取消
      if (active->cancel_flag.load()) {
        // 尝试终止子进程（pclose 会等待结束，但我们可以直接关闭管道，非标准但可行）
        pclose(pipe);
        result->success = false;
        result->error_msg = "Cancelled";
        goal_handle->canceled(result);
        cleanup(goal_handle);
        return;
      }
      std::string line(buffer.data());
      full_output += line;

      // 发布反馈（可选）
      auto feedback = std::make_shared<Output::Feedback>();
      feedback->partial_result = line;
      goal_handle->publish_feedback(feedback);
    }

    int status = pclose(pipe);
    if (status == -1) {
      result->success = false;
      result->error_msg = "pclose error";
      goal_handle->abort(result);
    } else {
      int exit_code = WEXITSTATUS(status);
      result->success = (exit_code == 0);
      result->final_result = full_output;
      result->error_msg = (exit_code == 0) ? "" : "Command failed with exit code " + std::to_string(exit_code);
      goal_handle->succeed(result);
    }
    cleanup(goal_handle);
  }

  void cleanup(const std::shared_ptr<GoalHandleOutput>& handle) {
    std::lock_guard lock(goals_mutex_);
    active_goals_.erase(handle);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto temp_node = std::make_shared<rclcpp::Node>("temp");
  temp_node->declare_parameter("agent_name", "agent");
  std::string agent_name = temp_node->get_parameter("agent_name").as_string();
  temp_node.reset();

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + agent_name + "_output_cmd_exec"});

  auto node = std::make_shared<CmdExecNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}