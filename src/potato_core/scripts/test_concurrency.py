#!/usr/bin/env python3
"""并发请求测试"""
import rclpy, asyncio
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from potato_interface.action import Chat
from potato_interface.msg import ChatMessage, ContentPart

class ConcurrentTester(Node):
    def __init__(self):
        super().__init__('conc_tester')
        self.client = ActionClient(self, Chat, '/rem/core/chat')
    async def send_one(self, text):
        goal = Chat.Goal()
        goal.stream = False
        goal.messages = [make_msg("user", text)]
        handle = await self.client.send_goal_async(goal)
        result = await handle.get_result_async()
        r = result.result
        print(f"[{text[:10]}...] 成功:{r.success}, 长度:{len(r.content)}")

def make_msg(role, text):
    msg = ChatMessage(role=role)
    msg.parts = [ContentPart(type="text", text=text)]
    return msg

def main():
    rclpy.init()
    node = ConcurrentTester()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    async def run():
        node.client.wait_for_server(timeout_sec=5)
        tasks = [
            node.send_one("从1数到101"),
            node.send_one("从1数到102"),
            node.send_one("从1数到103"),
            node.send_one("从1数到104"),
            node.send_one("从1数到105"),
            node.send_one("从1数到106"),
            node.send_one("从1数到107"),
            node.send_one("从1数到108"),
            node.send_one("从1数到109"),
        ]
        await asyncio.gather(*tasks)
    # 运行 asyncio 事件循环并集成 ROS spin
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    # 在后台线程 spin
    import threading
    spin_t = threading.Thread(target=lambda: ex.spin(), daemon=True)
    spin_t.start()
    loop.run_until_complete(run())
    rclpy.shutdown()

if __name__ == '__main__':
    main()