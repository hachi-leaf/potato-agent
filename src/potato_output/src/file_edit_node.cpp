#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdio>
#include <array>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "potato_interface/action/output.hpp"

using namespace std::chrono_literals;
using Output = potato_interface::action::Output;
using GoalHandleOutput = rclcpp_action::ServerGoalHandle<Output>;

static const char BASE64_CHARS[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static std::string base64_encode(const std::string &in) {
  std::string out;
  int val = 0, valb = -6;
  for (unsigned char c : in) {
    val = (val << 8) + c;
    valb += 8;
    while (valb >= 0) {
      out.push_back(BASE64_CHARS[(val >> valb) & 0x3F]);
      valb -= 6;
    }
  }
  if (valb > -6) out.push_back(BASE64_CHARS[((val << 8) >> (valb + 8)) & 0x3F]);
  while (out.size() % 4) out.push_back('=');
  return out;
}

class FileEditNode : public rclcpp::Node {
public:
  explicit FileEditNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("file_edit_node", options) {
    declare_parameter("agent_name", "agent");
    std::string agent_name = get_parameter("agent_name").as_string();
    std::string func_name = "file_edit";

    // 发布 help 话题
    help_pub_ = create_publisher<std_msgs::msg::String>(
      "/" + agent_name + "/output/" + func_name + "/help",
      rclcpp::QoS(1).transient_local());
    auto help_msg = std_msgs::msg::String();
    help_msg.data =
      "TYPE:action\n"
      "File Editor - 编辑文件内容\n"
      "输入格式：<模式> <行数> <文件路径> <内容>\n"
      "模式: w - 覆盖写入, a - 追加, i - 插入\n"
      "行数: 0 表示全文操作; >0 表示从指定行开始 (覆盖/插入)\n"
      "文件路径: 目标文件的绝对或相对路径\n"
      "内容: 编辑的具体文本 (可多行)\n"
      "示例:\n"
      "  w 0 /tmp/test.txt Hello World\n"
      "  a 5 /tmp/test.txt 新的一行\n"
      "  i 1 /tmp/test.txt 插入到第一行之前\n";
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
    pid_t child_pid = -1;   // 可选：用于真正杀死进程
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
      // 如果有子进程 PID 可尝试 kill，此处省略
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
    std::thread{std::bind(&FileEditNode::execute, this, goal_handle, active)}.detach();
  }

  // 解析 goal_text: <mode> <lineno> <filepath> <content>
  // 模式: w, a, i
  bool parse_input(const std::string& input, char &mode, int &lineno, std::string &filepath, std::string &content) {
    std::istringstream iss(input);
    std::string mode_str;
    if (!(iss >> mode_str)) return false;
    if (mode_str.size() != 1) return false;
    mode = mode_str[0];
    if (mode != 'w' && mode != 'a' && mode != 'i') return false;

    if (!(iss >> lineno)) return false;
    if (!(iss >> filepath)) return false;

    // 剩余部分全作为内容
    std::getline(iss, content);  // 读取空格后面的剩余内容
    if (!content.empty() && content[0] == ' ') content.erase(0,1); // 去除前导空格
    // 如果内容为空，允许，表示清空文件等
    return true;
  }

  void execute(
      std::shared_ptr<GoalHandleOutput> goal_handle,
      std::shared_ptr<ActiveGoal> active) {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Output::Result>();

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

    // 将内容 base64 编码，避免 shell 特殊字符问题
    std::string encoded = base64_encode(content);

    // 构建 Python 脚本
    std::ostringstream py_cmd;
    py_cmd << "python3 -c \""
           << "import sys, base64; "
           << "mode = '" << mode << "'; "
           << "lineno = " << lineno << "; "
           << "fpath = '" << filepath << "'; "
           << "content = base64.b64decode('" << encoded << "').decode('utf-8'); "
           // 实现文件编辑逻辑
           << "lines = []; "
           << "if mode == 'w': "
           << "  if lineno == 0: "
           << "    with open(fpath, 'w') as f: f.write(content) "
           << "  else: "
           // 覆盖模式：替换从 lineno 行开始的内容（若文件不够长则追加）
           << "    with open(fpath, 'r') as f: lines = f.readlines(); "
           << "    while len(lines) < lineno: lines.append('\\n'); "
           << "    lines = lines[:lineno-1] + [content] + lines[lineno-1:]; "
           << "    with open(fpath, 'w') as f: f.writelines(lines) "
           << "elif mode == 'a': "
           << "  with open(fpath, 'a') as f: "
           << "    if lineno == 0: f.write(content) "
           << "    else: "
           << "      with open(fpath, 'r') as fr: lines = fr.readlines(); "
           << "      while len(lines) < lineno: lines.append('\\n'); "
           << "      lines.insert(lineno, content); "
           << "      with open(fpath, 'w') as fw: fw.writelines(lines) "
           << "elif mode == 'i': "
           << "  with open(fpath, 'r') as f: lines = f.readlines(); "
           << "  if lineno > 0: lines.insert(lineno-1, content) "
           << "  else: lines.insert(0, content); "
           << "  with open(fpath, 'w') as f: f.writelines(lines) "
           << "print('OK')\" 2>&1";

    std::string full_cmd = py_cmd.str();

    // 执行命令，读取输出
    FILE* pipe = popen(full_cmd.c_str(), "r");
    if (!pipe) {
      result->success = false;
      result->error_msg = "Failed to execute editor";
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
      if (exit_code == 0 && output.find("OK") != std::string::npos) {
        result->success = true;
        result->final_result = "File edited successfully.";
      } else {
        result->success = false;
        result->final_result = output;
        result->error_msg = "Edit failed (check file path or permissions)";
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
  options.arguments({"--ros-args", "-r", "__node:=" + agent_name + "_output_file_edit"});

  auto node = std::make_shared<FileEditNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}