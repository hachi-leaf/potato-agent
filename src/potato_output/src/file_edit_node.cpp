#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>
#include <sstream>
#include <vector>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "potato_interface/action/output.hpp"

using namespace std::chrono_literals;
using Output = potato_interface::action::Output;
using GoalHandleOutput = rclcpp_action::ServerGoalHandle<Output>;

namespace fs = std::filesystem;

class FileEditNode : public rclcpp::Node {
public:
  explicit FileEditNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("file_edit_node", options) {
    declare_parameter("agent_name", "agent");
    std::string agent_name = get_parameter("agent_name").as_string();
    std::string func_name = "file_edit";

    // 发布帮助话题
    help_pub_ = create_publisher<std_msgs::msg::String>(
      "/" + agent_name + "/output/" + func_name + "/help",
      rclcpp::QoS(1).transient_local());
    auto help_msg = std_msgs::msg::String();
    help_msg.data =
      "参数为纯文本，格式为：\n"
      "<mode> <lineno> <filepath> <content>\n"
      "mode: w (覆盖), a (追加), i (插入)\n"
      "lineno: 0 表示全文操作; >0 表示从指定行开始 (覆盖/插入)\n"
      "filepath: 目标文件完整路径\n"
      "content: 要写入的内容，支持多行\n"
      "示例:\n"
      "  w 5 /path/to/file.txt new content\n"
      "  a 0 /path/to/file.txt appended text\n"
      "  i 3 /path/to/file.txt inserted line\n";
    help_pub_->publish(help_msg);

    // 创建 Action 服务器
    action_server_ = rclcpp_action::create_server<Output>(
      this,
      "/" + agent_name + "/output/" + func_name,
      std::bind(&FileEditNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FileEditNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&FileEditNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "FileEdit node started for agent '%s'", agent_name.c_str());
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
      std::shared_ptr<const Output::Goal> /*goal*/) {
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

  void handle_accepted(const std::shared_ptr<GoalHandleOutput> goal_handle) {
    auto active = std::make_shared<ActiveGoal>();
    active->handle = goal_handle;
    {
      std::lock_guard lock(goals_mutex_);
      active_goals_[goal_handle] = active;
    }
    std::thread{std::bind(&FileEditNode::execute, this, goal_handle, active)}.detach();
  }

  // 解析输入: <mode> <lineno> <filepath> <content>
  bool parse_input(const std::string& input, char &mode, int &lineno,
                   std::string &filepath, std::string &content) {
    std::istringstream iss(input);
    std::string mode_str;
    if (!(iss >> mode_str)) return false;
    if (mode_str.size() != 1) return false;
    mode = mode_str[0];
    if (mode != 'w' && mode != 'a' && mode != 'i') return false;

    if (!(iss >> lineno)) return false;
    if (!(iss >> filepath)) return false;

    // 剩余部分作为内容（保留前导空格）
    std::getline(iss, content);
    if (!content.empty() && content[0] == ' ') content.erase(0,1);
    return true;
  }

  // 将文本按行分割
  static std::vector<std::string> split_lines(const std::string& text) {
    std::vector<std::string> lines;
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
      lines.push_back(line);
    }
    return lines;
  }

  void execute(std::shared_ptr<GoalHandleOutput> goal_handle,
               std::shared_ptr<ActiveGoal> active) {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Output::Result>();

    // 检查取消
    if (active->cancel_flag.load()) {
      result->success = false;
      result->error_msg = "Cancelled";
      goal_handle->canceled(result);
      cleanup(goal_handle);
      return;
    }

    char mode;
    int lineno;
    std::string filepath, content;
    if (!parse_input(goal->goal_text, mode, lineno, filepath, content)) {
      result->success = false;
      result->error_msg = "Invalid input format. Use: <mode> <lineno> <file> <content>";
      goal_handle->abort(result);
      cleanup(goal_handle);
      return;
    }

    // 将内容转换为行列表
    auto new_lines = split_lines(content);

    try {
      // 模式: w (覆盖)
      if (mode == 'w') {
        if (lineno == 0) {
          // 全文覆盖
          std::ofstream out(filepath, std::ios::trunc);
          if (!out.is_open()) {
            throw std::runtime_error("Cannot open file for writing: " + filepath);
          }
          for (const auto& line : new_lines) {
            out << line << '\n';
          }
        } else {
          // 替换从 lineno 行开始的内容
          std::vector<std::string> lines;
          if (fs::exists(filepath)) {
            std::ifstream in(filepath);
            if (!in.is_open()) {
              throw std::runtime_error("Cannot open file for reading: " + filepath);
            }
            std::string line;
            while (std::getline(in, line)) {
              lines.push_back(line);
            }
          }
          // 补齐空行到 lineno-1
          while (static_cast<int>(lines.size()) < lineno - 1) {
            lines.push_back("");
          }
          // 替换从 lineno-1 开始的若干行
          if (static_cast<int>(lines.size()) >= lineno) {
            lines.erase(lines.begin() + lineno - 1, lines.end());
          }
          // 追加新内容
          for (const auto& line : new_lines) {
            lines.push_back(line);
          }
          // 写回文件
          std::ofstream out(filepath, std::ios::trunc);
          if (!out.is_open()) {
            throw std::runtime_error("Cannot open file for writing: " + filepath);
          }
          for (const auto& line : lines) {
            out << line << '\n';
          }
        }
      }
      // 模式: a (追加)
      else if (mode == 'a') {
        if (lineno == 0) {
          // 全文追加
          std::ofstream out(filepath, std::ios::app);
          if (!out.is_open()) {
            throw std::runtime_error("Cannot open file for appending: " + filepath);
          }
          for (const auto& line : new_lines) {
            out << line << '\n';
          }
        } else {
          // 在指定行后插入内容
          std::vector<std::string> lines;
          if (fs::exists(filepath)) {
            std::ifstream in(filepath);
            if (!in.is_open()) {
              throw std::runtime_error("Cannot open file for reading: " + filepath);
            }
            std::string line;
            while (std::getline(in, line)) {
              lines.push_back(line);
            }
          }
          // 补齐空行到 lineno
          while (static_cast<int>(lines.size()) < lineno) {
            lines.push_back("");
          }
          // 在 lineno 行后插入新行
          lines.insert(lines.begin() + lineno, new_lines.begin(), new_lines.end());
          // 写回
          std::ofstream out(filepath, std::ios::trunc);
          if (!out.is_open()) {
            throw std::runtime_error("Cannot open file for writing: " + filepath);
          }
          for (const auto& line : lines) {
            out << line << '\n';
          }
        }
      }
      // 模式: i (插入)
      else if (mode == 'i') {
        std::vector<std::string> lines;
        if (fs::exists(filepath)) {
          std::ifstream in(filepath);
          if (!in.is_open()) {
            throw std::runtime_error("Cannot open file for reading: " + filepath);
          }
          std::string line;
          while (std::getline(in, line)) {
            lines.push_back(line);
          }
        }
        if (lineno <= 0) {
          // 插入到文件开头
          lines.insert(lines.begin(), new_lines.begin(), new_lines.end());
        } else {
          // 插入到 lineno 行前（lineno-1 位置）
          if (static_cast<int>(lines.size()) < lineno - 1) {
            // 文件不够长，补齐空行
            while (static_cast<int>(lines.size()) < lineno - 1) {
              lines.push_back("");
            }
          }
          lines.insert(lines.begin() + lineno - 1, new_lines.begin(), new_lines.end());
        }
        // 写回文件
        std::ofstream out(filepath, std::ios::trunc);
        if (!out.is_open()) {
          throw std::runtime_error("Cannot open file for writing: " + filepath);
        }
        for (const auto& line : lines) {
          out << line << '\n';
        }
      }
    } catch (const std::exception& e) {
      result->success = false;
      result->final_result = e.what();
      result->error_msg = e.what();
      goal_handle->succeed(result);  // 使用 succeed 将错误信息返回给调用者
      cleanup(goal_handle);
      return;
    }

    // 成功
    result->success = true;
    result->final_result = "File edited successfully.";
    goal_handle->succeed(result);
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
  options.arguments({"--ros-args", "-r", "__node:=" + agent_name + "_output_file_edit"});

  auto node = std::make_shared<FileEditNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}