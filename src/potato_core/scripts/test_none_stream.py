#!/usr/bin/env python3
"""非流式对话测试"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from potato_interface.action import Chat
from potato_interface.msg import ChatMessage, ContentPart

class Tester(Node):
    def __init__(self):
        super().__init__('tester')
        self.client = ActionClient(self, Chat, '/rem/core/chat')  # 请改为实际 agent_name
    async def run(self):
        self.client.wait_for_server(timeout_sec=5)
        goal = Chat.Goal()
        goal.stream = False
        goal.messages = [
            make_msg("system", "这是一次测试，简短输出，满足测试者的需求即可。"),
            make_msg("user", "什么是 Agent？")
        ]
        goal_handle = await self.client.send_goal_async(goal)
        result = await goal_handle.get_result_async()
        r = result.result
        print(f"成功: {r.success}, 内容: {r.content}")
        if r.reasoning_content:
            print(f"思考: {r.reasoning_content}")
        print(f"Tokens: {r.prompt_tokens}/{r.completion_tokens}/{r.total_tokens}")

def make_msg(role, text):
    msg = ChatMessage(role=role)
    msg.parts = [ContentPart(type="text", text=text)]
    return msg

def main():
    rclpy.init()
    node = Tester()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    future = executor.create_task(node.run())
    executor.spin_until_future_complete(future, timeout_sec=30)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()