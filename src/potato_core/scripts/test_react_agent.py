#!/usr/bin/env python3
"""
ReAct Agent —— 自定义标签协议，支持转义，优化流式显示
"""
import re
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from potato_interface.action import Chat, CallOutput
from potato_interface.msg import ChatMessage, ContentPart
from potato_interface.srv import ListOutputs, GetOutputInfo


def make_msg(role, text):
    msg = ChatMessage(role=role)
    part = ContentPart(type="text", text=text)
    msg.parts = [part]
    return msg


class ReactAgent(Node):
    def __init__(self, agent_name):
        super().__init__('react_agent')
        self.agent_name = agent_name

        # 客户端
        self.chat_client = ActionClient(self, Chat, f'/{agent_name}/core/chat')
        self.call_client = ActionClient(self, CallOutput, f'/{agent_name}/output/call')
        self.list_client = self.create_client(ListOutputs, f'/{agent_name}/output/info/list')
        self.info_client = self.create_client(GetOutputInfo, f'/{agent_name}/output/info/get')

        self.chat_client.wait_for_server(timeout_sec=10.0)
        self.call_client.wait_for_server(timeout_sec=10.0)
        self.list_client.wait_for_service(timeout_sec=5.0)
        self.info_client.wait_for_service(timeout_sec=5.0)

        # 动态构建系统提示词（含转义说明）
        self.system_prompt = (
            f"你的名字是 {agent_name}，是一个 Agent。\n"
            f"如果需要与系统交互，请在回复中包含特殊标签：\n"
            f"开始标签：@^{agent_name}:命令名^@\n"
            f"结束标签：@#{agent_name}:命令名#@\n"
            f"命令内容放在两个标签之间。例如：\n"
            f"  @^{agent_name}:tools_list^@None@#{agent_name}:tools_list#@\n"
            f"  @^{agent_name}:tools_info^@工具名@#{agent_name}:tools_info#@\n"
            f"  @^{agent_name}:工具名^@参数@#{agent_name}:工具名#@\n"
            f"注意：每次只能使用一个工具。\n"
            f"当你想要结束本轮 ReAct 时，请使用：\n"
            f"  @^{agent_name}:end^@最终回答内容@#{agent_name}:end#@\n"
            f"当你想在输出内容包括 @^,^@,@#,#@ 时，再加个@来转义，变成：\n"
            f"  @@^,@^@,@@#,@#@\n"
        )

        self.messages = [make_msg("system", self.system_prompt)]

        # 流式显示状态
        self.last_fb_type = None      # 'reasoning' 或 'content'

    # ================== 工具发现 ==================
    def list_tools(self):
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

    # ================== 流式推理（优化显示） ==================
    def feedback_cb(self, fb_msg):
        fb = fb_msg.feedback

        # 处理推理内容
        if fb.partial_reasoning:
            if self.last_fb_type != 'reasoning':
                if self.last_fb_type is not None:
                    sys.stdout.write('\n')
                sys.stdout.write('[思考] ')
                self.last_fb_type = 'reasoning'
            sys.stdout.write(fb.partial_reasoning)
            sys.stdout.flush()

        # 处理正式输出内容
        if fb.partial_content:
            if self.last_fb_type != 'content':
                if self.last_fb_type is not None:
                    sys.stdout.write('\n')
                sys.stdout.write('[回复] ')
                self.last_fb_type = 'content'
            sys.stdout.write(fb.partial_content)
            sys.stdout.flush()

    def think_stream(self, messages):
        # 重置流式状态
        self.last_fb_type = None

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

    # ================== 工具执行 ==================
    def execute_tool(self, tool_name, args):
        goal = CallOutput.Goal()
        goal.target_name = tool_name
        goal.goal_text = args

        print(f"[命令] {tool_name} {args}")
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

    # ================== 解析与循环 ==================
    def run(self, question):
        self.messages.append(make_msg("user", question))

        for step in range(999):
            print(f"\n===== 第 {step+1} 轮 =====")
            reply = self.think_stream(self.messages)
            if reply is None:
                return "错误：模型无响应"
            print()  # 换行

            # 1. 还原转义字符，避免干扰标签识别
            reply_unescaped = reply.replace('@@^', '@^').replace('@^@', '^@').replace('@@#', '@#').replace('@#@', '#@')

            # 2. 匹配标签：@^{agent}:命令^@ 内容 @#{agent}:命令#@
            pattern = rf'@\^{self.agent_name}:(?P<cmd>[^^]+)\^@\s*(?P<content>.*?)\s*@#{self.agent_name}:(?P=cmd)#@'
            matches = list(re.finditer(pattern, reply_unescaped, re.DOTALL))

            if not matches:
                print("[系统] 模型回复格式错误，请使用规定的标签格式。")
                # 将原始回复（未转义）存入历史，以便模型看到自己写的转义字符
                self.messages.append(make_msg("assistant", reply))
                self.messages.append(make_msg("user",
                    f"你的回复格式不正确。请使用 @^{self.agent_name}:命令^@ 内容 @#{self.agent_name}:命令#@ 的格式。"))
                continue

            first_match = matches[0]
            cmd = first_match.group("cmd").strip()
            content = first_match.group("content").strip()

            if cmd == "end":
                self.messages.append(make_msg("assistant", reply))
                return content if content else "对话结束"

            elif cmd == "tools_list":
                tools = self.list_tools()
                obs = "可用工具列表：\n" + "\n".join(tools)
                print(f"[系统] 获取工具列表")

            elif cmd == "tools_info":
                tool_name = content
                obs = self.get_tool_info(tool_name)
                print(f"[系统] 查询工具 '{tool_name}' 的信息")

            else:
                obs = self.execute_tool(cmd, content)

            print(f"[结果] {obs}")
            self.messages.append(make_msg("assistant", reply))
            self.messages.append(make_msg("user", f"系统返回：\n{obs}"))

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