import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class MQTTBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')
        self.pub = self.create_publisher(String, '/ai/trigger', 10)

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect("broker.hivemq.com", 1883, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT Broker!")
        client.subscribe("drone/command")

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        self.get_logger().info(f"Received MQTT: {payload}")
        ros_msg = String()
        ros_msg.data = payload
        self.pub.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
