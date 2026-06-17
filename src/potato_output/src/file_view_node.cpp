#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdio>
#include <array>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "potato_interface/action/output.hpp"

using namespace std::chrono_literals;
using Output = potato_interface::action::Output;
using GoalHandleOutput = rclcpp_action::ServerGoalHandle<Output>;

class FileViewNode : public rclcpp::Node {
public:
  explicit FileViewNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("file_view_node", options) {
    declare_parameter("agent_name", "agent");
    std::string agent_name = get_parameter("agent_name").as_string();
    std::string func_name = "file_view";

    // 发布 help 话题
    help_pub_ = create_publisher<std_msgs::msg::String>(
      "/" + agent_name + "/output/" + func_name + "/help",
      rclcpp::QoS(1).transient_local());
    auto help_msg = std_msgs::msg::String();
    help_msg.data =
      "TYPE:action\n"
      "File Viewer - 查看文件内容\n"
      "输入格式: <范围类型> [参数...] <文件路径>\n"
      "范围类型:\n"
      "  all      : 显示整个文件\n"
      "  head N   : 显示文件前 N 行\n"
      "  tail N   : 显示文件最后 N 行\n"
      "  range A B: 显示第 A 行到第 B 行\n"
      "示例:\n"
      "  all /etc/passwd\n"
      "  head 10 /var/log/syslog\n"
      "  tail 20 /var/log/syslog\n"
      "  range 5 10 /etc/passwd\n";
    help_pub_->publish(help_msg);

    // 创建 Action 服务器
    action_server_ = rclcpp_action::create_server<Output>(
      this,
      "/" + agent_name + "/output/" + func_name,
      std::bind(&FileViewNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FileViewNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&FileViewNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "FileView node started for agent '%s'", agent_name.c_str());
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr help_pub_;
  rclcpp_action::Server<Output>::SharedPtr action_server_;

  struct ActiveGoal {
    std::shared_ptr<GoalHandleOutput> handle;
    std::atomic<bool> cancel_flag{false};
  };
  std::mutex goals_mutex_;
  std::unordered_map<std::shared_ptr<GoalHandleOutput>, std::shared_ptr<ActiveGoal>> active_goals_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const Output::Goal> goal) {
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleOutput> goal_handle) {
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
    std::thread{std::bind(&FileViewNode::execute, this, goal_handle, active)}.detach();
  }

  // 根据 goal_text 构建 shell 命令
  std::string build_cmd(const std::string& input) {
    std::istringstream iss(input);
    std::string type;
    if (!(iss >> type)) return "";

    if (type == "all") {
      std::string filepath;
      if (!(iss >> filepath)) return "";
      return "cat \"" + filepath + "\" 2>&1";
    }
    else if (type == "head") {
      int n;
      std::string filepath;
      if (!(iss >> n >> filepath)) return "";
      return "head -n " + std::to_string(n) + " \"" + filepath + "\" 2>&1";
    }
    else if (type == "tail") {
      int n;
      std::string filepath;
      if (!(iss >> n >> filepath)) return "";
      return "tail -n " + std::to_string(n) + " \"" + filepath + "\" 2>&1";
    }
    else if (type == "range") {
      int a, b;
      std::string filepath;
      if (!(iss >> a >> b >> filepath)) return "";
      return "sed -n '" + std::to_string(a) + "," + std::to_string(b) + "p' \"" + filepath + "\" 2>&1";
    }
    return "";
  }

  void execute(
      std::shared_ptr<GoalHandleOutput> goal_handle,
      std::shared_ptr<ActiveGoal> active) {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Output::Result>();

    std::string cmd = build_cmd(goal->goal_text);
    if (cmd.empty()) {
      result->success = false;
      result->error_msg = "Invalid input format. See help.";
      goal_handle->abort(result);
      cleanup(goal_handle);
      return;
    }

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
      result->success = false;
      result->error_msg = "Failed to open pipe";
      goal_handle->abort(result);
      cleanup(goal_handle);
      return;
    }

    std::array<char, 128> buffer;
    std::string output;
    while (fgets(buffer.data(), buffer.size(), pipe)) {
      if (active->cancel_flag.load()) {
        pclose(pipe);
        result->success = false;
        result->error_msg = "Cancelled";
        goal_handle->canceled(result);
        cleanup(goal_handle);
        return;
      }
      output += buffer.data();
      auto feedback = std::make_shared<Output::Feedback>();
      feedback->partial_result = std::string(buffer.data());
      goal_handle->publish_feedback(feedback);
    }

    int status = pclose(pipe);
    if (status == -1) {
      result->success = false;
      result->error_msg = "pclose error";
      goal_handle->abort(result);
    } else {
      int exit_code = WEXITSTATUS(status);
      if (exit_code == 0) {
        result->success = true;
        result->final_result = output;
      } else {
        result->success = false;
        result->final_result = output;
        result->error_msg = "File view failed (check path or permissions)";
      }
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
  options.arguments({"--ros-args", "-r", "__node:=" + agent_name + "_output_file_view"});

  auto node = std::make_shared<FileViewNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}