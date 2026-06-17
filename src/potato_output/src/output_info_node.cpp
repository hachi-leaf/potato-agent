#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "potato_interface/action/output.hpp"
#include "potato_interface/action/call_output.hpp"
#include "potato_interface/srv/list_outputs.hpp"
#include "potato_interface/srv/get_output_info.hpp"

using namespace std::chrono_literals;
using Output = potato_interface::action::Output;
using CallOutput = potato_interface::action::CallOutput;
using GoalHandleCallOutput = rclcpp_action::ServerGoalHandle<CallOutput>;
using ClientGoalHandleOutput = rclcpp_action::ClientGoalHandle<Output>;

struct OutputEntry {
  std::string name;
  std::string help_text;
  rclcpp_action::Client<Output>::SharedPtr action_client;
};

class OutputInfoNode : public rclcpp::Node {
public:
  explicit OutputInfoNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("output_info_node", options) {
    declare_parameter("agent_name", "agent");
    std::string agent_name = get_parameter("agent_name").as_string();
    agent_name_ = agent_name;

    // 服务：列出所有 output
    list_srv_ = create_service<potato_interface::srv::ListOutputs>(
      "/" + agent_name + "/output/info/list",
      std::bind(&OutputInfoNode::handle_list, this, std::placeholders::_1, std::placeholders::_2));

    // 服务：获取指定 output 的帮助
    info_srv_ = create_service<potato_interface::srv::GetOutputInfo>(
      "/" + agent_name + "/output/info/get",
      std::bind(&OutputInfoNode::handle_get_info, this, std::placeholders::_1, std::placeholders::_2));

    // 动作：通用调用入口 /call
    call_action_server_ = rclcpp_action::create_server<CallOutput>(
      this,
      "/" + agent_name + "/output/call",
      std::bind(&OutputInfoNode::handle_call_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&OutputInfoNode::handle_call_cancel, this, std::placeholders::_1),
      std::bind(&OutputInfoNode::handle_call_accepted, this, std::placeholders::_1));

    // 定时扫描 output 节点
    scan_timer_ = create_wall_timer(1s, std::bind(&OutputInfoNode::scan_outputs, this));

    RCLCPP_INFO(get_logger(), "OutputInfoNode started for agent '%s'", agent_name.c_str());
  }

private:
  std::string agent_name_;
  std::mutex entries_mutex_;
  std::unordered_map<std::string, OutputEntry> output_entries_;
  std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> help_subs_;

  rclcpp::Service<potato_interface::srv::ListOutputs>::SharedPtr list_srv_;
  rclcpp::Service<potato_interface::srv::GetOutputInfo>::SharedPtr info_srv_;
  rclcpp_action::Server<CallOutput>::SharedPtr call_action_server_;
  rclcpp::TimerBase::SharedPtr scan_timer_;

  // 子动作句柄映射，用于取消传递
  std::mutex sub_goal_mutex_;
  std::unordered_map<std::shared_ptr<GoalHandleCallOutput>, std::shared_ptr<ClientGoalHandleOutput>> sub_goal_map_;

  // ========== 服务回调 ==========
  void handle_list(
      const std::shared_ptr<potato_interface::srv::ListOutputs::Request> /*req*/,
      std::shared_ptr<potato_interface::srv::ListOutputs::Response> res) {
    std::lock_guard lock(entries_mutex_);
    for (const auto& [name, entry] : output_entries_) {
      res->names.push_back(name);
    }
  }

  void handle_get_info(
      const std::shared_ptr<potato_interface::srv::GetOutputInfo::Request> req,
      std::shared_ptr<potato_interface::srv::GetOutputInfo::Response> res) {
    std::lock_guard lock(entries_mutex_);
    auto it = output_entries_.find(req->name);
    if (it != output_entries_.end()) {
      res->info = it->second.help_text;
    } else {
      res->info = "Not found";
    }
  }

  // ========== /call 动作回调 ==========
  rclcpp_action::GoalResponse handle_call_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const CallOutput::Goal> goal) {
    std::lock_guard lock(entries_mutex_);
    auto it = output_entries_.find(goal->target_name);
    if (it == output_entries_.end()) {
      RCLCPP_WARN(get_logger(), "Rejected: unknown target '%s'", goal->target_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_call_cancel(
      const std::shared_ptr<GoalHandleCallOutput> goal_handle) {
    RCLCPP_INFO(get_logger(), "Cancel requested for output call");

    // 从原始 Goal 中获取 target_name
    auto goal = goal_handle->get_goal();
    std::string target_name = goal->target_name;

    // 取出子动作句柄
    std::shared_ptr<ClientGoalHandleOutput> sub_handle;
    {
      std::lock_guard lock(sub_goal_mutex_);
      auto it = sub_goal_map_.find(goal_handle);
      if (it != sub_goal_map_.end()) {
        sub_handle = it->second;
      }
    }

    if (sub_handle) {
      // 获取对应的 ActionClient
      OutputEntry entry;
      {
        std::lock_guard lock(entries_mutex_);
        auto it = output_entries_.find(target_name);
        if (it != output_entries_.end()) {
          entry = it->second;
        }
      }

      if (entry.action_client) {
        RCLCPP_INFO(get_logger(), "Forwarding cancel to sub action");
        // 使用 ActionClient 的取消接口
        entry.action_client->async_cancel_goal(sub_handle);
      }
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_call_accepted(
      const std::shared_ptr<GoalHandleCallOutput> goal_handle) {
    std::thread{std::bind(&OutputInfoNode::execute_call, this, goal_handle)}.detach();
  }

  void execute_call(
      std::shared_ptr<GoalHandleCallOutput> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<CallOutput::Result>();

    OutputEntry entry;
    {
      std::lock_guard lock(entries_mutex_);
      auto it = output_entries_.find(goal->target_name);
      if (it == output_entries_.end()) {
        result->success = false;
        result->error_msg = "Output not available";
        goal_handle->abort(result);
        return;
      }
      entry = it->second;
    }

    if (!entry.action_client) {
      result->success = false;
      result->error_msg = "Action client not ready";
      goal_handle->abort(result);
      return;
    }

    if (!entry.action_client->wait_for_action_server(1s)) {
      result->success = false;
      result->error_msg = "Target action server not ready";
      goal_handle->abort(result);
      return;
    }

    // 构造子 Goal（Output 类型，只有 goal_text）
    auto sub_goal = Output::Goal();
    sub_goal.goal_text = goal->goal_text;

    auto send_options = rclcpp_action::Client<Output>::SendGoalOptions();
    send_options.feedback_callback =
      [goal_handle](auto, const std::shared_ptr<const Output::Feedback> fb) {
        auto fwd = std::make_shared<CallOutput::Feedback>();
        fwd->partial_result = fb->partial_result;
        goal_handle->publish_feedback(fwd);
      };

    auto future = entry.action_client->async_send_goal(sub_goal, send_options);
    auto sub_result = future.get();

    // 保存子动作句柄，用于取消传递
    {
      std::lock_guard lock(sub_goal_mutex_);
      sub_goal_map_[goal_handle] = sub_result;   // sub_result 是 ClientGoalHandle 的共享指针
    }

    // 等待子动作完成
    auto result_future = entry.action_client->async_get_result(sub_result);
    auto sub_result_ptr = result_future.get();  // 阻塞等待子动作结束
    if (sub_result_ptr.code == rclcpp_action::ResultCode::SUCCEEDED) {
      result->success = sub_result_ptr.result->success;
      result->final_result = sub_result_ptr.result->final_result;
      result->error_msg = sub_result_ptr.result->error_msg;
      goal_handle->succeed(result);
    } else if (sub_result_ptr.code == rclcpp_action::ResultCode::CANCELED) {
      result->success = false;
      result->error_msg = "Sub action cancelled";
      goal_handle->canceled(result);
    } else {
      result->success = false;
      result->error_msg = "Sub action failed";
      goal_handle->abort(result);
    }

    // 清理映射
    {
      std::lock_guard lock(sub_goal_mutex_);
      sub_goal_map_.erase(goal_handle);
    }
  }

  // ========== 输出发现 ==========
  void scan_outputs() {
    auto topics = get_topic_names_and_types();
    std::string prefix = "/" + agent_name_ + "/output/";
    for (const auto& [topic_name, types] : topics) {
      if (topic_name.rfind(prefix, 0) == 0) {
        std::string suffix = topic_name.substr(prefix.size());
        size_t slash = suffix.find('/');
        if (slash != std::string::npos && suffix.substr(slash) == "/help") {
          std::string output_name = suffix.substr(0, slash);
          if (help_subs_.find(output_name) == help_subs_.end()) {
            create_help_subscription(output_name);
          }
        }
      }
    }
  }

  void create_help_subscription(const std::string& output_name) {
    std::string topic = "/" + agent_name_ + "/output/" + output_name + "/help";
    auto sub = create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(1).transient_local(),
      [this, output_name](std_msgs::msg::String::ConstSharedPtr msg) {
        process_help_message(output_name, msg->data);
      });
    help_subs_[output_name] = sub;
  }

  void process_help_message(const std::string& output_name, const std::string& help_text) {
    std::string first_line = help_text.substr(0, help_text.find('\n'));
    if (first_line.rfind("TYPE:action", 0) != 0) {
      RCLCPP_WARN(get_logger(), "Output '%s' ignored: first line not 'TYPE:action'", output_name.c_str());
      return;
    }

    {
      std::lock_guard lock(entries_mutex_);
      auto& entry = output_entries_[output_name];
      entry.name = output_name;
      entry.help_text = help_text;
      if (!entry.action_client) {
        entry.action_client = rclcpp_action::create_client<Output>(
          this, "/" + agent_name_ + "/output/" + output_name);
      }
    }
    RCLCPP_INFO(get_logger(), "Registered output '%s'", output_name.c_str());
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto temp_node = std::make_shared<rclcpp::Node>("temp");
  temp_node->declare_parameter("agent_name", "agent");
  std::string agent_name = temp_node->get_parameter("agent_name").as_string();
  temp_node.reset();

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + agent_name + "_output_info"});

  auto node = std::make_shared<OutputInfoNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}