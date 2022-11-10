#!/usr/bin/python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

if __name__ == "__main__":
    rclpy.init()
    node = Node('start_switch')
    pub = node.create_publisher(Bool, 'start_switch', 5)

    for i in range(5):
        msg = Bool()
        msg.data = True
        pub.publish(msg)
