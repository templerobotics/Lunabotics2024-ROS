import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import socket
import json
import threading

class JoyReceiver(Node):
    def __init__(self):
        super().__init__('joy_receiver')
        self.pub = self.create_publisher(Joy, 'joy', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 5006))  # Must match the Mac's UDP port
        threading.Thread(target=self.listen_udp, daemon=True).start()
        self.get_logger().info("Listening for joystick data on UDP port 5006...")

    def listen_udp(self):
        while True:
            data, _ = self.sock.recvfrom(4096)
            try:
                parsed = json.loads(data.decode())
                msg = Joy()
                msg.axes = parsed.get("axes", [])
                msg.buttons = parsed.get("buttons", [])
                self.pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to parse message: {e}")

def main():
    rclpy.init()
    node = JoyReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
