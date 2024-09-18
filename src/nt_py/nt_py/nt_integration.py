import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from networktables import NetworkTable


class nt(Node):
    def __init__(self):
        super().__init__("rospy network table bridge")
        


