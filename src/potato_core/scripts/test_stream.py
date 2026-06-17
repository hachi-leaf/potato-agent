#!/usr/bin/env python3
"""流式对话测试"""
import sys, threading, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from potato_interface.action import Chat
from potato_interface.msg import ChatMessage, ContentPart

class StreamTester(Node):
    def __init__(self):
        super().__init__('stream_tester')
        self.client = ActionClient(self, Chat, '/rem/core/chat')
    def feedback_cb(self, fb):
        if fb.feedback.partial_content:
            sys.stdout.write(fb.feedback.partial_content)
            sys.stdout.flush()
    def run(self):
        self.client.wait_for_server(timeout_sec=5)
        goal = Chat.Goal()
        goal.stream = True
        goal.messages = [
            make_msg("system", "这是一次测试，请满足用户的需求。"),
            make_msg("user", "什么是 Agent？")
        ]
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        while not future.done(): pass
        handle = future.result()
        res_fut = handle.get_result_async()
        while not res_fut.done(): pass
        r = res_fut.result().result
        print(f"\n完成，成功:{r.success}, tokens:{r.total_tokens}")

def make_msg(role, text):
    msg = ChatMessage(role=role)
    msg.parts = [ContentPart(type="text", text=text)]
    return msg

def main():
    rclpy.init()
    node = StreamTester()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    spin_t = threading.Thread(target=exec.spin, daemon=True)
    spin_t.start()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()