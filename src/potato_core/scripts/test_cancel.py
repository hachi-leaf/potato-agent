#!/usr/bin/env python3
"""取消请求测试"""
import rclpy, threading, time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from potato_interface.action import Chat
from potato_interface.msg import ChatMessage, ContentPart

class CancelTester(Node):
    def __init__(self):
        super().__init__('cancel_tester')
        self.client = ActionClient(self, Chat, '/rem/core/chat')
    def run(self):
        self.client.wait_for_server(timeout_sec=5)
        goal = Chat.Goal()
        goal.stream = True   # 流式便于中途取消
        goal.messages = [make_msg("user", "写一首很长的诗")]
        future = self.client.send_goal_async(goal)
        while not future.done(): pass
        handle = future.result()
        if not handle.accepted:
            print("Goal rejected")
            return
        def delayed_cancel():
            time.sleep(1)
            print("发送取消...")
            handle.cancel_goal_async()
        t = threading.Thread(target=delayed_cancel)
        t.start()
        res_fut = handle.get_result_async()
        while not res_fut.done(): pass
        r = res_fut.result().result
        print(f"成功:{r.success}, 错误:{r.error_msg}")
        t.join()

def make_msg(role, text):
    msg = ChatMessage(role=role)
    msg.parts = [ContentPart(type="text", text=text)]
    return msg

def main():
    rclpy.init()
    node = CancelTester()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    spin_t = threading.Thread(target=exec.spin, daemon=True)
    spin_t.start()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()