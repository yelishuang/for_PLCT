import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, String
import random  

class HelloWorldResponder(Node):
    def __init__(self):
        super().__init__('random_responder')
        self.subscription = self.create_subscription(
            String,  # 接收任意 Int64 消息（忽略内容）
            'user_input',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'random_responde', 10)

    def listener_callback(self, msg):
        # 生成随机数
        random_number = random.randint(0, 100)
        
        # 构建响应字符串
        response_str = f'Hello World{random_number}'
        
        # 构造响应消息
        response = String()
        response.data = response_str
        
        # 发布响应
        self.publisher_.publish(response)
        
        # 打印日志
        self.get_logger().info(f'Received a message. Responding with: {response.data}')

def main(args=None):
    rclpy.init()
    responder = HelloWorldResponder()
    try:
        rclpy.spin(responder)
    except KeyboardInterrupt:
        pass
    finally:
        responder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
