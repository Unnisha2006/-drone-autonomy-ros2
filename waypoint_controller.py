import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
from std_msgs.msg import String
import json
from datetime import datetime

class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.ai_sub = self.create_subscription(String, '/ai/trigger', self.ai_callback, 10)
        self.timer = self.create_timer(5.0, self.send_waypoint)

    def send_waypoint(self):
        self.get_logger().info("Sending waypoint command...")
        self.send_vehicle_command(
            command=16,  # MAV_CMD_NAV_WAYPOINT
            param5=47.397742,  # Latitude
            param6=8.545594,   # Longitude
            param7=10.0        # Altitude
        )
        self.log_event("MOVE_TO_WAYPOINT", lat=47.397742, lon=8.545594, alt=10.0)

    def send_return_to_base(self):
        self.get_logger().info("Returning to base...")
        self.send_vehicle_command(command=20)  # MAV_CMD_NAV_RETURN_TO_LAUNCH
        self.log_event("RETURN_TO_BASE")

    def send_abort(self):
        self.get_logger().info("Abort triggered!")
        self.send_vehicle_command(command=21)  # MAV_CMD_DO_LAND_IMMEDIATELY
        self.log_event("LAND_IMMEDIATELY")

    def send_vehicle_command(self, command, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def ai_callback(self, msg):
        if msg.data == "abort":
            self.send_abort()
        elif msg.data == "return":
            self.send_return_to_base()

    def log_event(self, event, lat=None, lon=None, alt=None):
        log_data = {
            "time": datetime.utcnow().isoformat() + "Z",
            "event": event
        }
        if lat and lon and alt:
            log_data["location"] = {"lat": lat, "lon": lon, "alt": alt}
        with open("drone_log.json", "a") as f:
            f.write(json.dumps(log_data) + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
