#!/usr/bin/env python3
"""测试 reasoning_effort (low/medium/high)"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from potato_interface.action import Chat
from potato_interface.msg import ChatMessage, ContentPart

class Tester(Node):
    def __init__(self):
        super().__init__('tester')
        self.client = ActionClient(self, Chat, '/rem/core/chat')
    async def run(self):
        self.client.wait_for_server(timeout_sec=5)
        for effort in ["low", "medium", "high"]:
            goal = Chat.Goal()
            goal.stream = False
            goal.reasoning_effort = effort
            goal.messages = [make_msg("user", "1+1=? 请逐步推理")]
            handle = await self.client.send_goal_async(goal)
            result = await handle.get_result_async()
            r = result.result
            print(f"等级:{effort}, 成功:{r.success}, 思考长度:{len(r.reasoning_content)}, 回答:{r.content}")

def make_msg(role, text):
    msg = ChatMessage(role=role)
    msg.parts = [ContentPart(type="text", text=text)]
    return msg

def main():
    rclpy.init()
    node = Tester()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    future = ex.create_task(node.run())
    ex.spin_until_future_complete(future, timeout_sec=60)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()