#!/usr/bin/python3

from hexapod.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DummyNode(Node):
    def __init__(self):
        super().__init__('state_controller_node')
        self.create_subscription(PoseStamped, 'foot/pose' ,self.foot_pose_callback,10)
        
    def foot_pose_callback(self,msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
