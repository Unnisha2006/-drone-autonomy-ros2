import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AITriggerPublisher(Node):
    def __init__(self):
        super().__init__('ai_trigger_publisher')
        self.publisher_ = self.create_publisher(String, '/ai/trigger', 10)
        self.timer = self.create_timer(8.0, self.send_trigger)

    def send_trigger(self):
        msg = String()
        msg.data = "return"  # Change to "abort" to test abort
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent trigger: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = AITriggerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
