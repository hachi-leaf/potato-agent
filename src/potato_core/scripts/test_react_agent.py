#!/usr/bin/env python3
"""
ReAct Agent —— 自动发现 output 工具，支持流式推理
"""
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from potato_interface.action import Chat, CallOutput
from potato_interface.msg import ChatMessage, ContentPart
from potato_interface.srv import ListOutputs, GetOutputInfo


# ==================== 工具函数 ====================
def make_msg(role, text):
    msg = ChatMessage(role=role)
    part = ContentPart(type="text", text=text)
    msg.parts = [part]
    return msg


# ==================== Agent 节点 ====================
class ReactAgent(Node):
    def __init__(self, agent_name):
        super().__init__('react_agent')
        self.agent_name = agent_name

        # 创建客户端
        self.chat_client = ActionClient(self, Chat, f'/{agent_name}/core/chat')
        self.call_client = ActionClient(self, CallOutput, f'/{agent_name}/output/call')
        self.list_client = self.create_client(ListOutputs, f'/{agent_name}/output/info/list')
        self.info_client = self.create_client(GetOutputInfo, f'/{agent_name}/output/info/get')

        # 等待服务
        self.chat_client.wait_for_server(timeout_sec=10.0)
        self.call_client.wait_for_server(timeout_sec=10.0)
        self.list_client.wait_for_service(timeout_sec=5.0)
        self.info_client.wait_for_service(timeout_sec=5.0)

        # 系统提示词（不包含具体工具列表）
        self.system_prompt = (
            "你是一个能够执行命令和操作的助手。你可以通过以下方式与系统交互：\n\n"
            "1. 查询可用工具：\n"
            "   动作：info list\n"
            "   （系统会返回所有可用工具的名称列表）\n\n"
            "2. 查看某个工具的详细用法：\n"
            "   动作：info get <工具名>\n"
            "   （系统会返回该工具的帮助文档）\n\n"
            "3. 执行工具：\n"
            "   动作：<工具名> <参数>\n"
            "   （系统会执行工具并返回结果）\n\n"
            "如果你已经能够回答用户的问题，请输出：\n"
            "   最终答案：<你的回答>\n\n"
            "注意：每次只输出一个动作或最终答案。请先使用 info 命令了解有哪些工具可用。"
        )

        # 对话历史
        self.messages = [make_msg("system", self.system_prompt)]

    # ==================== 工具发现（不经过 LLM） ====================
    def list_tools(self):
        """返回工具名称列表"""
        req = ListOutputs.Request()
        future = self.list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            return ["(获取工具列表超时)"]
        res = future.result()
        if res is None:
            return ["(获取工具列表失败)"]
        return list(res.names)

    def get_tool_info(self, name):
        """返回指定工具的帮助文本"""
        req = GetOutputInfo.Request()
        req.name = name
        future = self.info_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            return "(获取工具信息超时)"
        res = future.result()
        if res is None:
            return "(获取工具信息失败)"
        return res.info

    # ==================== LLM 流式推理 ====================
    def feedback_cb(self, fb_msg):
        """实时打印增量内容（包括推理和回答）"""
        fb = fb_msg.feedback
        if fb.partial_reasoning:
            sys.stdout.write(fb.partial_reasoning)
            sys.stdout.flush()
        if fb.partial_content:
            sys.stdout.write(fb.partial_content)
            sys.stdout.flush()

    def think_stream(self, messages):
        goal = Chat.Goal()
        goal.stream = True
        goal.reasoning_effort = "high"
        goal.messages = messages
        goal.timeout_sec = 60

        future = self.chat_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().error("Timeout waiting for goal acceptance")
            return None
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        if not result_future.done():
            self.get_logger().error("Timeout waiting for model response")
            goal_handle.cancel_goal_async()
            return None

        res = result_future.result().result
        if not res.success:
            self.get_logger().error(f"LLM error: {res.error_msg}")
            return None
        return res.content

    # ==================== 工具执行 ====================
    def execute_tool(self, tool_name, args):
        goal = CallOutput.Goal()
        goal.target_name = tool_name
        goal.goal_text = args

        future = self.call_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            return f"工具 {tool_name} 调用超时"
        goal_handle = future.result()
        if not goal_handle.accepted:
            return f"工具 {tool_name} 调用被拒绝"

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        if not result_future.done():
            return f"工具 {tool_name} 结果超时"
        res = result_future.result().result
        if res.success:
            return res.final_result.strip()
        else:
            return f"工具执行失败: {res.error_msg}"

    # ==================== 主循环 ====================
    def run(self, question):
        self.messages.append(make_msg("user", question))

        for step in range(999):
            print(f"\n--- 第 {step+1} 轮 ---")
            reply = self.think_stream(self.messages)
            if reply is None:
                return "错误：模型无响应"
            print()  # 换行

            # 解析最终答案
            final = None
            for line in reply.split("\n"):
                if "最终答案" in line:
                    if "：" in line:
                        final = line.split("：", 1)[-1].strip()
                    elif ":" in line:
                        final = line.split(":", 1)[-1].strip()
            if final:
                self.messages.append(make_msg("assistant", reply))
                return final

            # 解析动作
            action_line = None
            for line in reply.split("\n"):
                line = line.strip()
                if line.startswith("动作：") or line.startswith("Action:"):
                    action_line = line
                    break
            if action_line is None:
                self.messages.append(make_msg("assistant", reply))
                self.messages.append(make_msg("user", "请给出动作或最终答案。"))
                continue

            # 提取动作内容
            if "：" in action_line:
                action_str = action_line.split("：", 1)[-1].strip()
            elif ":" in action_line:
                action_str = action_line.split(":", 1)[-1].strip()
            else:
                action_str = ""
            parts = action_str.split(maxsplit=1)
            tool_name = parts[0] if parts else ""
            args = parts[1] if len(parts) > 1 else ""

            # 特殊处理 info 命令
            if tool_name == "info":
                if args == "list" or args.startswith("list"):
                    tool_list = self.list_tools()
                    obs = "可用工具列表：\n" + "\n".join(tool_list)
                elif args.startswith("get "):
                    target = args[4:].strip()
                    obs = self.get_tool_info(target)
                else:
                    obs = "info 命令格式错误。请使用：info list 或 info get <工具名>"
            else:
                print(f"[执行] {tool_name} {args}")
                obs = self.execute_tool(tool_name, args)
                print(f"[结果] {obs}")

            # 追加历史
            self.messages.append(make_msg("assistant", reply))
            self.messages.append(make_msg("user", f"观察结果：\n{obs}"))

        return "超过最大步数，任务中止。"


def main():
    rclpy.init()
    agent_name = "rem" if len(sys.argv) <= 1 else sys.argv[1]
    node = ReactAgent(agent_name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"===== ReAct Agent ({agent_name}) =====")
    print("输入问题开始对话，输入 'quit' 退出")

    try:
        while True:
            user_input = input("\n>>> ")
            if user_input.strip().lower() in ("quit", "exit"):
                break
            if not user_input.strip():
                continue
            answer = node.run(user_input)
            print(f"\n===== 最终答案 =====\n{answer}")
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()