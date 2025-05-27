import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

arduinoData = serial.Serial('/dev/ttyUSB0', 9600)

class SerialNode(Node):
    def __init__(self):
        super().__init__("servo_node")
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.get_logger().info("Servo node initialized")

    def joy_callback(self, joy_msg):
        if joy_msg.buttons[11] == 1:
            self.get_logger().info('Open servo button pressed!')
            arduinoData.write(b'1')
        if joy_msg.buttons[12] == 1:
            self.get_logger().info('Close servo button pressed!')
            arduinoData.write(b'0')

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
