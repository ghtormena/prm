#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose_to_odom')
        self.sub = self.create_subscription(
            PoseStamped,
            '/model/prm_robot/pose',
            self.cb_pose,
            10
        )
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.br  = TransformBroadcaster(self)

    def cb_pose(self, msg: PoseStamped):
        odom = Odometry()
        odom.header = msg.header              # usa exatamente o timestamp e frame
        odom.child_frame_id = 'base_link'
        odom.pose.pose  = msg.pose
        self.pub.publish(odom)

        t = TransformStamped()
        t.header        = msg.header
        t.header.frame_id  = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation = msg.pose.position
        t.transform.rotation    = msg.pose.orientation
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
