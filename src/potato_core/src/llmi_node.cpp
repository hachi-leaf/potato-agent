#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "potato_interface/msg/chat_message.hpp"
#include "potato_interface/msg/content_part.hpp"
#include "potato_interface/action/chat.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;
using Chat = potato_interface::action::Chat;
using GoalHandle = rclcpp_action::ServerGoalHandle<Chat>;
using namespace std::chrono_literals;

// ========== 提供者配置结构 ==========
struct ProviderConfig {
  std::string name;
  std::string api_key;
  std::string api_url;
  std::string model_name;
  int priority = 0;
  int max_tokens = 4096;
  std::vector<std::string> modalities;
  bool enabled = true;
};

// ========== 活跃请求上下文 ==========
struct RequestContext {
  std::shared_ptr<GoalHandle> goal_handle;
  std::string accumulated_content;
  std::string accumulated_reasoning;
  std::string buffer;                 // SSE 行缓冲
  std::atomic<bool> cancel_flag{false};
  std::atomic<int> prompt_tokens{0};
  std::atomic<int> completion_tokens{0};
  std::atomic<int> total_tokens{0};
  int feedback_count{0};
};

// ========== 节点类 ==========
class LLMInterfaceNode : public rclcpp::Node {
public:
  explicit LLMInterfaceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("llmi_node", options) {
    declare_parameter("agent_name", "agent");
    declare_parameter("core_chat_action", "core/chat");
    declare_parameter("config_file", "./providers.json");
    declare_parameter("default_timeout_sec", 120);

    std::string config_file = get_parameter("config_file").as_string();
    default_timeout_ = get_parameter("default_timeout_sec").as_int();

    load_providers(config_file);
    if (providers_.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid LLM providers loaded in '%s'. Node will be unusable.",
                   config_file.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Loaded %zu LLM provider(s).", providers_.size());
    }

    std::string agent_name = get_parameter("agent_name").as_string();
    std::string action_suffix = get_parameter("core_chat_action").as_string();
    std::string action_name = "/" + agent_name + "/" + action_suffix;

    action_server_ = rclcpp_action::create_server<Chat>(
      this,
      action_name,
      std::bind(&LLMInterfaceNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LLMInterfaceNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&LLMInterfaceNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "LLM Interface Node started. Action server: %s",
                action_name.c_str());
  }

private:
  std::vector<ProviderConfig> providers_;
  mutable std::shared_mutex providers_mutex_;
  int default_timeout_;

  rclcpp_action::Server<Chat>::SharedPtr action_server_;
  std::mutex contexts_mutex_;
  std::unordered_map<std::shared_ptr<GoalHandle>, std::shared_ptr<RequestContext>> active_contexts_;

  // ========== 从单个 JSON 文件加载配置 ==========
  void load_providers(const std::string& file_path) {
    if (!fs::exists(file_path) || !fs::is_regular_file(file_path)) {
      RCLCPP_ERROR(get_logger(), "Config file '%s' does not exist.", file_path.c_str());
      return;
    }

    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open config file '%s'.", file_path.c_str());
      return;
    }

    try {
      json j = json::parse(file);
      auto process = [&](const json& item) {
        ProviderConfig cfg;
        cfg.name = item.value("name", "unnamed");
        cfg.api_key = item.value("api_key", "");
        cfg.api_url = item.value("api_url", "");
        cfg.model_name = item.value("model_name", "");
        cfg.priority = item.value("priority", 99);
        cfg.max_tokens = item.value("max_tokens", 4096);
        if (item.contains("modalities")) {
          cfg.modalities = item["modalities"].get<std::vector<std::string>>();
        } else {
          cfg.modalities = {"text"};
        }
        cfg.enabled = item.value("enabled", true);
        if (!cfg.api_key.empty() && !cfg.api_url.empty() && !cfg.model_name.empty() && cfg.enabled) {
          providers_.push_back(std::move(cfg));
        }
      };

      if (j.is_array()) {
        for (auto& obj : j) process(obj);
      } else {
        process(j);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Parse error in '%s': %s", file_path.c_str(), e.what());
    }

    // 按优先级排序
    std::sort(providers_.begin(), providers_.end(),
              [](const ProviderConfig& a, const ProviderConfig& b) {
                return a.priority < b.priority;
              });
  }

  // ========== 选择提供者 ==========
  ProviderConfig select_provider(bool requires_image) {
    std::shared_lock lock(providers_mutex_);
    for (const auto& p : providers_) {
      if (!p.enabled) continue;
      bool has_text = false, has_image = false;
      for (const auto& m : p.modalities) {
        if (m == "text") has_text = true;
        if (m == "image") has_image = true;
      }
      if (requires_image && !has_image) continue;
      if (!has_text) continue;
      return p;
    }
    return ProviderConfig{};
  }

  // ========== Action 回调 ==========
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const Chat::Goal> goal) {
    bool requires_image = false;
    for (const auto& msg : goal->messages) {
      for (const auto& part : msg.parts) {
        if (part.type == "image_url") {
          requires_image = true;
          break;
        }
      }
    }

    ProviderConfig cfg = select_provider(requires_image);
    if (cfg.api_key.empty()) {
      RCLCPP_WARN(get_logger(), "No suitable provider available. Rejecting goal.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "Goal accepted, using provider '%s'.", cfg.name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle) {
    std::lock_guard lock(contexts_mutex_);
    auto it = active_contexts_.find(goal_handle);
    if (it != active_contexts_.end()) {
      it->second->cancel_flag.store(true);
      RCLCPP_INFO(get_logger(), "Cancellation requested.");
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&LLMInterfaceNode::execute, this, goal_handle)}.detach();
  }

  // ========== 核心执行 ==========
  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Chat::Result>();

    bool requires_image = false;
    for (const auto& msg : goal->messages) {
      for (const auto& part : msg.parts) {
        if (part.type == "image_url") { requires_image = true; break; }
      }
    }

    ProviderConfig config;
    {
      std::shared_lock lock(providers_mutex_);
      config = select_provider(requires_image);
    }
    if (config.api_key.empty()) {
      result->success = false;
      result->error_msg = "No suitable LLM provider available.";
      goal_handle->abort(result);
      return;
    }

    json messages_json = json::array();
    for (const auto& msg : goal->messages) {
      if (msg.parts.empty()) {
        messages_json.push_back({{"role", msg.role}, {"content", ""}});
        continue;
      }
      if (msg.parts.size() == 1 && msg.parts[0].type == "text") {
        messages_json.push_back({{"role", msg.role}, {"content", msg.parts[0].text}});
      } else {
        json content_array = json::array();
        for (const auto& part : msg.parts) {
          if (part.type == "text") {
            content_array.push_back({{"type", "text"}, {"text", part.text}});
          } else if (part.type == "image_url") {
            content_array.push_back({
              {"type", "image_url"},
              {"image_url", {{"url", part.image_url}}}
            });
          }
        }
        messages_json.push_back({{"role", msg.role}, {"content", content_array}});
      }
    }

    json payload = {
      {"model", config.model_name},
      {"messages", messages_json},
      {"stream", goal->stream}
    };
    if (!goal->reasoning_effort.empty()) {
      payload["reasoning_effort"] = goal->reasoning_effort;
    }

    int timeout = (goal->timeout_sec > 0) ? goal->timeout_sec : default_timeout_;

    CURL* curl = curl_easy_init();
    if (!curl) {
      result->success = false;
      result->error_msg = "Failed to initialize CURL";
      goal_handle->abort(result);
      return;
    }

    std::string url = config.api_url + "/chat/completions";
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    std::string auth = "Authorization: Bearer " + config.api_key;
    headers = curl_slist_append(headers, auth.c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    std::string post_data = payload.dump();
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_data.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, (long)timeout);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);

    auto ctx = std::make_shared<RequestContext>();
    ctx->goal_handle = goal_handle;
    {
      std::lock_guard lock(contexts_mutex_);
      active_contexts_[goal_handle] = ctx;
    }

    curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, progress_callback);
    curl_easy_setopt(curl, CURLOPT_XFERINFODATA, ctx.get());
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);

    if (goal->stream) {
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, ctx.get());

      CURLcode res = curl_easy_perform(curl);
      long http_code = 0;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

      curl_slist_free_all(headers);
      curl_easy_cleanup(curl);

      {
        std::lock_guard lock(contexts_mutex_);
        active_contexts_.erase(goal_handle);
      }

      if (res != CURLE_OK || http_code != 200) {
        result->success = false;
        if (ctx->cancel_flag.load()) {
          result->error_msg = "Request cancelled";
        } else {
          result->error_msg = "HTTP " + std::to_string(http_code) + ": " +
                              curl_easy_strerror(res);
        }
        goal_handle->abort(result);
        RCLCPP_ERROR(get_logger(), "Stream failed: %s", result->error_msg.c_str());
        return;
      }

      result->content = ctx->accumulated_content;
      result->reasoning_content = ctx->accumulated_reasoning;
      result->success = true;
      result->prompt_tokens = ctx->prompt_tokens.load();
      result->completion_tokens = ctx->completion_tokens.load();
      result->total_tokens = ctx->total_tokens.load();
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Stream finished. Content length=%zu, feedbacks sent=%d",
                  ctx->accumulated_content.size(), ctx->feedback_count);

    } else {
      std::string response_str;
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, string_callback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_str);

      CURLcode res = curl_easy_perform(curl);
      long http_code = 0;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

      curl_slist_free_all(headers);
      curl_easy_cleanup(curl);

      {
        std::lock_guard lock(contexts_mutex_);
        active_contexts_.erase(goal_handle);
      }

      if (res != CURLE_OK || http_code != 200) {
        result->success = false;
        if (ctx->cancel_flag.load()) {
          result->error_msg = "Request cancelled";
        } else {
          result->error_msg = "HTTP " + std::to_string(http_code) + ": " +
                              curl_easy_strerror(res);
        }
        goal_handle->abort(result);
        return;
      }

      try {
        json response = json::parse(response_str);
        result->content = response["choices"][0]["message"]["content"];
        if (response["choices"][0]["message"].contains("reasoning_content")) {
          result->reasoning_content =
              response["choices"][0]["message"]["reasoning_content"];
        }
        if (response.contains("usage")) {
          result->prompt_tokens = response["usage"]["prompt_tokens"];
          result->completion_tokens = response["usage"]["completion_tokens"];
          result->total_tokens = response["usage"]["total_tokens"];
        }
        result->success = true;
        goal_handle->succeed(result);
      } catch (const std::exception& e) {
        result->success = false;
        result->error_msg = "JSON parsing error: " + std::string(e.what());
        goal_handle->abort(result);
      }
    }
  }

  // ========== curl 回调 ==========
  static size_t write_callback(void* contents, size_t size, size_t nmemb, void* userp) {
    auto* ctx = static_cast<RequestContext*>(userp);
    if (!ctx || ctx->cancel_flag.load()) return 0;

    size_t total = size * nmemb;
    std::string data(static_cast<char*>(contents), total);
    ctx->buffer += data;
    ctx->feedback_count++;

    size_t pos;
    while ((pos = ctx->buffer.find('\n')) != std::string::npos) {
      std::string line = ctx->buffer.substr(0, pos);
      ctx->buffer.erase(0, pos + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (line.empty()) continue;

      if (line.rfind("data: ", 0) == 0) {
        std::string data_str = line.substr(6);
        if (data_str == "[DONE]") break;
        try {
          json j = json::parse(data_str);
          if (j.contains("choices") && !j["choices"].empty()) {
            auto& delta = j["choices"][0]["delta"];
            auto feedback = std::make_shared<Chat::Feedback>();
            bool has_update = false;

            if (delta.contains("content") && !delta["content"].is_null()) {
              std::string chunk = delta["content"];
              ctx->accumulated_content += chunk;
              feedback->partial_content = chunk;
              has_update = true;
            }
            if (delta.contains("reasoning_content") && !delta["reasoning_content"].is_null()) {
              std::string chunk = delta["reasoning_content"];
              ctx->accumulated_reasoning += chunk;
              feedback->partial_reasoning = chunk;
              has_update = true;
            }
            if (has_update) {
              ctx->goal_handle->publish_feedback(feedback);
            }
          }
          if (j.contains("usage")) {
            ctx->prompt_tokens.store(j["usage"]["prompt_tokens"]);
            ctx->completion_tokens.store(j["usage"]["completion_tokens"]);
            ctx->total_tokens.store(j["usage"]["total_tokens"]);
          }
        } catch (...) {}
      }
    }
    return total;
  }

  static size_t string_callback(void* contents, size_t size, size_t nmemb, void* userp) {
    auto* str = static_cast<std::string*>(userp);
    str->append(static_cast<char*>(contents), size * nmemb);
    return size * nmemb;
  }

  static int progress_callback(void* clientp, curl_off_t, curl_off_t, curl_off_t, curl_off_t) {
    auto* ctx = static_cast<RequestContext*>(clientp);
    return (ctx && ctx->cancel_flag.load()) ? 1 : 0;
  }
};

// ========== main ==========
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // 提取 agent_name 参数以设置节点名
  auto temp_node = std::make_shared<rclcpp::Node>("temp_param_reader");
  temp_node->declare_parameter("agent_name", "agent");
  std::string agent_name = temp_node->get_parameter("agent_name").as_string();
  temp_node.reset();

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + agent_name + "_llmi"});

  auto node = std::make_shared<LLMInterfaceNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
