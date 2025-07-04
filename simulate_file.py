import time
import random
from datetime import datetime

# Simulated Drone State
class Drone:
    def __init__(self):
        self.lat = 37.7749
        self.lon = -122.4194
        self.alt = 50.0
        self.speed = 5.0
        self.battery = 100.0
        self.state = "IDLE"

    def update_position(self):
        self.lat += random.uniform(-0.0003, 0.0003)
        self.lon += random.uniform(-0.0003, 0.0003)
        self.alt += random.uniform(-1.0, 1.0)
        self.speed = random.uniform(4.0, 6.0)
        self.battery -= 0.5

    def get_telemetry(self):
        return {
            "timestamp": datetime.utcnow().isoformat(),
            "latitude": round(self.lat, 6),
            "longitude": round(self.lon, 6),
            "altitude": round(self.alt, 2),
            "speed": round(self.speed, 2),
            "battery": round(self.battery, 2),
            "state": self.state
        }

    def trigger_ai_alert(self):
        print("🚨 AI ALERT: Obstacle detected!")
        self.state = "RETURN_TO_BASE"

    def return_to_base(self):
        self.lat = 37.7749
        self.lon = -122.4194
        self.alt = 0.0
        self.speed = 0.0
        self.state = "LANDED"
        print("✅ Drone returned to base and landed.")


def simulate_drone_mission():
    drone = Drone()
    print("🚁 Starting drone mission simulation...")

    for tick in range(30):
        if tick == 15:
            drone.trigger_ai_alert()

        if drone.state == "RETURN_TO_BASE":
            drone.return_to_base()
            break

        drone.update_position()
        telemetry = drone.get_telemetry()
        print(telemetry)
        time.sleep(0.5)

    print("🛑 Simulation complete.")


if __name__ == "__main__":
    simulate_drone_mission()

