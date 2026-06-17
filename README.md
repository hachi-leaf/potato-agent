<div align="center">

![License](https://img.shields.io/badge/license-MIT-blue?style=flat-square)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros&style=flat-square)
![C++](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B&style=flat-square)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&style=flat-square)
![Platform](https://img.shields.io/badge/platform-Ubuntu%2024.04-E95420?logo=ubuntu&style=flat-square)

</div>

---

# 🥔 Potato Agent

> *A modular, extensible ROS2-native ReAct Agent framework — batteries included, potato-powered.*

**Potato Agent** 是一个基于 ROS2 的智能体框架，采用 **ReAct（Reasoning + Acting）** 范式，通过模块化架构将 LLM 推理与工具执行无缝集成。

---

## ✨ 特性

| 🧠 | 🔧 | 🔌 | ⚡ |
|:--:|:--:|:--:|:--:|
| **多 Provider** | **可扩展工具** | **流式 SSE** | **并发安全** |
| 支持多个 LLM 后端 | 热插拔式工具节点 | 实时流式输出 | 线程安全设计 |

| 🎯 | ❌ | ⏱️ | 📦 |
|:--:|:--:|:--:|:--:|
| **ReAct 范式** | **取消支持** | **超时控制** | **ROS2 原生** |
| 思考-行动循环 | 请求可随时取消 | 可配置超时 | 完全融入 ROS2 生态 |

---

## 🏗️ 架构

```
┌─────────────────────────────────────────────────────────────┐
│                     User / Application                      │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                  potato_core (核心引擎)                       │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                  llmi_node.cpp                         │  │
│  │  • 多 Provider 管理 (优先级, 多模态支持)                │  │
│  │  • 流式 SSE 解析 & 非流式请求                          │  │
│  │  • 取消 / 超时 / 并发控制                               │  │
│  │  • Chat Action 服务器                                  │  │
│  └───────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │               test_react_agent.py                      │  │
│  │  • ReAct 循环引擎                                      │  │
│  │  • 自定义标签协议解析                                   │  │
│  │  • 工具发现 & 动态调用                                  │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
          │                    │                    │
          ▼                    ▼                    ▼
┌──────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ potato_input  │  │   potato_interface   │  │   potato_output       │
│              │  │                      │  │                       │
│ input_node   │  │ Chat.action          │  │ cmd_exec_node   🖥️    │
│              │  │ Output.action        │  │ file_view_node  📄    │
│              │  │ CallOutput.action    │  │ file_edit_node  ✏️    │
│              │  │ ChatMessage.msg      │  │ output_info_node ℹ️   │
│              │  │ ListOutputs.srv      │  │                       │
│              │  │ GetOutputInfo.srv    │  │  ... 更多工具 ...      │
└──────────────┘  └──────────────────────┘  └──────────────────────┘
```

---

## 📦 模块说明

### 🧠 potato_core — 核心引擎

| 文件 | 说明 |
|------|------|
| `src/llmi_node.cpp` | LLM 接口节点，Action 服务器，处理 Chat 请求 |
| `scripts/test_react_agent.py` | ReAct Agent 客户端，解析标签协议，执行思考-行动循环 |
| `scripts/test_stream.py` | 流式推理测试 |
| `scripts/test_none_stream.py` | 非流式推理测试 |
| `scripts/test_reasoning.py` | 推理能力测试 |
| `scripts/test_concurrency.py` | 并发请求测试 |
| `scripts/test_timeout.py` | 超时处理测试 |
| `scripts/test_cancel.py` | 取消请求测试 |

### 🔌 potato_interface — 通信接口

| 类型 | 名称 | 用途 |
|------|------|------|
| Action | `Chat.action` | LLM 聊天（支持流式、推理、多模态） |
| Action | `Output.action` | 工具输出（含反馈流） |
| Action | `CallOutput.action` | 工具调用代理 |
| Message | `ChatMessage.msg` | 聊天消息（多角色） |
| Message | `ContentPart.msg` | 内容片段（text / image_url） |
| Service | `ListOutputs.srv` | 列出所有可用工具 |
| Service | `GetOutputInfo.srv` | 获取工具帮助信息 |

### 🔧 potato_output — 内置工具

| 工具 | 节点 | 功能 |
|:----:|------|------|
| 🖥️ `cmd_exec` | `cmd_exec_node.cpp` | 执行 Shell 命令，返回 stdout/stderr |
| 📄 `file_view` | `file_view_node.cpp` | 查看文件（all / head / tail / range） |
| ✏️ `file_edit` | `file_edit_node.cpp` | 编辑文件（覆盖 / 追加 / 插入） |
| ℹ️ `output_info` | `output_info_node.cpp` | 工具信息注册与发现 |

> 💡 **扩展工具**：只需实现 `potato_interface::action::Output` 服务端，即可添加你自己的工具！

---

## 🚀 快速开始

### 环境要求

- Ubuntu 24.04
- ROS2 Jazzy
- C++17 编译器
- Python 3.10+
- libcurl, nlohmann-json

### 构建

```bash
cd ~/your_ros2_workspace
colcon build --packages-select potato_interface potato_core potato_input potato_output
source install/setup.bash
```

### 配置 LLM Provider

在项目根目录创建 `providers.json`：

```json
[
  {
    "name": "deepseek",
    "api_key": "sk-your-api-key",
    "api_url": "https://api.deepseek.com/v1",
    "model_name": "deepseek-chat",
    "priority": 1,
    "modalities": ["text"],
    "enabled": true
  }
]
```

### 启动 Agent

```bash
# 启动核心节点
ros2 run potato_core llmi_node --ros-args -p agent_name:=moss

# 启动工具节点
ros2 run potato_output cmd_exec_node --ros-args -p agent_name:=moss &
ros2 run potato_output file_view_node --ros-args -p agent_name:=moss &
ros2 run potato_output file_edit_node --ros-args -p agent_name:=moss &
ros2 run potato_output output_info_node --ros-args -p agent_name:=moss &

# 启动 ReAct 客户端
python3 src/potato_core/scripts/test_react_agent.py moss
```

---

## 🔄 ReAct 协议

Agent 使用自定义标签协议与系统交互：

```
@^moss:命令名^@
参数内容
@#moss:命令名#@
```

| 内置命令 | 说明 |
|----------|------|
| `@^moss:tools_list^@` | 获取可用工具列表 |
| `@^moss:tools_info^@` | 查询工具帮助信息 |
| `@^moss:工具名^@` | 调用指定工具 |
| `@^moss:end^@` | 结束对话，返回最终答案 |

> 💡 转义规则：输出 `@^`、`^@`、`@#`、`#@` 时，在前面加 `@` 变为 `@^`、`^@`、`@#`、`#@`。

---

## 🎯 设计亮点

### 1. 多 Provider 优先级调度
`providers.json` 配置多个 LLM 后端，按 `priority` 排序，自动选择最合适的 Provider。支持多模态判断（有图片时自动选择支持 image 的 Provider）。

### 2. 流式 SSE 解析
`llmi_node.cpp` 实现了完整的 SSE 流式解析，支持 `reasoning_content` 和 `content` 双通道输出，逐 chunk 反馈给客户端。

### 3. 优雅的取消机制
通过 `cancel_flag` 原子变量 + `curl` 进度回调实现零延迟取消，支持 `pthread_cancel` 作为兜底方案。

### 4. 热插拔工具系统
工具节点通过 ROS2 Action 暴露，启动/停止不影响 Agent 运行。`output_info_node` 自动发现并注册所有工具。

---

## 📁 项目结构

```
potato_agent/
├── README.md
├── LICENSE
├── providers.json
├── build/
├── install/
├── log/
└── src/
    ├── potato_core/
    │   ├── src/llmi_node.cpp
    │   └── scripts/
    │       ├── test_react_agent.py
    │       ├── test_stream.py
    │       ├── test_none_stream.py
    │       ├── test_reasoning.py
    │       ├── test_concurrency.py
    │       ├── test_timeout.py
    │       └── test_cancel.py
    ├── potato_interface/
    │   ├── action/
    │   │   ├── Chat.action
    │   │   ├── Output.action
    │   │   └── CallOutput.action
    │   ├── msg/
    │   │   ├── ChatMessage.msg
    │   │   └── ContentPart.msg
    │   └── srv/
    │       ├── ListOutputs.srv
    │       └── GetOutputInfo.srv
    ├── potato_input/
    │   └── src/input_node.cpp
    └── potato_output/
        └── src/
            ├── cmd_exec_node.cpp
            ├── file_view_node.cpp
            ├── file_edit_node.cpp
            └── output_info_node.cpp
```

---

## 📝 License

MIT © [Leaf](mailto:zxy_yys_leaf@163.com)

---

<div align="center">

```
  🥔  Potato Agent — 简单、可靠、可扩展的 ROS2 Agent 框架
```

</div>
README_EOF 2>&1