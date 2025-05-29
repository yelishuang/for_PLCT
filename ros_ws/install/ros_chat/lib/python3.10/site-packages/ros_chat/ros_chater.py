#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets
import threading
import queue  # 用于线程间通信

class WebSocketToUserInput(Node):
    def __init__(self):
        super().__init__('websocket_to_user_input')
        self.publisher_ = self.create_publisher(String, 'user_input', 10)
        self.subscription_ = self.create_subscription(
            String,
            'random_responde',
            self.response_callback,
            10
        )

        # 配置 WebSocket 地址
        self.declare_parameter('ws_url', 'ws://localhost:8765')
        self.ws_url = self.get_parameter('ws_url').get_parameter_value().string_value

        self.get_logger().info(f'Connecting to WebSocket: {self.ws_url}')

        # 消息队列（线程安全）
        self.send_queue = queue.Queue()

        # 启动 WebSocket 客户端线程
        self.websocket_thread = threading.Thread(target=self.run_websocket, daemon=True)
        self.websocket_thread.start()

    def response_callback(self, msg):
        """当收到 /random_response 消息时，将其放入发送队列"""
        self.get_logger().info(f"Queuing response: {msg.data}")
        self.send_queue.put(msg.data)

    def run_websocket(self):
        asyncio.run(self.websocket_client())

    async def websocket_client(self):
        try:
            async with websockets.connect(self.ws_url) as websocket:
                self.get_logger().info('WebSocket connected')

                # 创建接收和发送任务
                receive_task = asyncio.create_task(self.receive_messages(websocket))
                send_task = asyncio.create_task(self.send_messages(websocket))

                await asyncio.gather(receive_task, send_task)

        except Exception as e:
            self.get_logger().error(f"WebSocket connection failed: {e}")

    async def receive_messages(self, websocket):
        """接收 WebSocket 消息并发布到 /user_input"""
        while rclpy.ok():
            try:
                message = await websocket.recv()
                self.get_logger().info(f"Received: {message}")
                msg = String()
                msg.data = message
                self.publisher_.publish(msg)
            except websockets.exceptions.ConnectionClosed:
                self.get_logger().warn("WebSocket connection closed")
                break

    async def send_messages(self, websocket):
        """从队列中取出消息并通过 WebSocket 发送"""
        while rclpy.ok():
            if not self.send_queue.empty():
                try:
                    message = self.send_queue.get_nowait()
                    await websocket.send(message)
                    self.get_logger().info(f"Sent: {message}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send message: {e}")
            else:
                await asyncio.sleep(0.1)  # 避免 CPU 占用过高

def main(args=None):
    rclpy.init()
    node = WebSocketToUserInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
