#!/usr/bin/env python3
# mission_manager.py

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import cv2
import numpy as np
import math
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # suporte a PoseStamped em tf2_buffer.transform

# intervalo HSV da bandeira
HSV_MIN = (86, 0, 6)
HSV_MAX = (94, 255, 255)

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # utilitários
        self.bridge = CvBridge()
        self.flag_seen = False
        self.initial_pose = None
        self.phase = 'explore'
        self.map: OccupancyGrid = None

        # metas já visitadas e última meta enviada
        self.visited_goals = []  # lista de (x, y)
        self.last_goal = None    # PoseStamped

        # publisher para FlagServo
        self.flag_servo_enabled_pub = self.create_publisher(Bool, '/flag_servo_enable', 1)
        # cliente de ação para FollowWaypoints
        self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # TF2
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # assinaturas
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_cb, 10)
        self.create_subscription(Odometry, '/odom',            self.odom_cb,   10)
        self.create_subscription(Bool,      '/flag_servo_arrived', self.flag_arrived_cb, 1)
        self.create_subscription(OccupancyGrid, '/map',        self.map_cb,    1)

        # controle de handles
        self.explore_goal_future = None
        self.explore_goal_handle = None
        self.return_goal_future  = None
        self.return_goal_handle  = None

        # timer de missão
        self.timer = self.create_timer(2.0, self.mission_loop)

    def map_cb(self, msg: OccupancyGrid):
        self.map = msg

    def odom_cb(self, msg: Odometry):
        if self.initial_pose is not None:
            return

        ps_odom = PoseStamped()
        ps_odom.header.frame_id = 'odom'
        ps_odom.header.stamp    = msg.header.stamp
        ps_odom.pose            = msg.pose.pose

        try:
            ps_map = self.tf_buffer.transform(ps_odom, 'map', timeout=Duration(seconds=1.0))
            ps_map.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose = ps_map
            self.get_logger().info(
                f'Pose inicial salva (map): '
                f'{ps_map.pose.position.x:.2f}, '
                f'{ps_map.pose.position.y:.2f}'
            )
        except TransformException as ex:
            self.get_logger().warn(f'Falha no TF ao converter pose: {ex}')

    def camera_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_MIN, HSV_MAX)
        self.flag_seen = np.count_nonzero(mask) > 1750

    def compute_next_frontier_goal(self) -> PoseStamped:
        if self.map is None or self.initial_pose is None:
            return None

        w, h   = self.map.info.width, self.map.info.height
        res    = self.map.info.resolution
        ox, oy = self.map.info.origin.position.x, self.map.info.origin.position.y
        grid   = np.array(self.map.data).reshape((h, w), order='C')

        free    = (grid == 0).astype(np.uint8)
        unknown = (grid == -1).astype(np.uint8)

        # fronteiras: livre adjacente a desconhecido
        unk_dilated = cv2.dilate(unknown, np.ones((3,3),np.uint8), iterations=1)
        frontier    = free & (unk_dilated > 0)

        # folga de 0.2 m de obstáculos
        dt = cv2.distanceTransform((grid != 100).astype(np.uint8), cv2.DIST_L2, 5) * res
        frontier &= (dt > 0.2)

        pts = np.argwhere(frontier)
        if pts.size == 0:
            return None

        rx = self.initial_pose.pose.position.x
        ry = self.initial_pose.pose.position.y

        # candidatos a >0.5 m do robô
        candidates = []
        for i, j in pts:
            x = ox + (j + 0.5) * res
            y = oy + (i + 0.5) * res
            d = math.hypot(x - rx, y - ry)
            if d > 0.5:
                candidates.append((x, y, d))

        if not candidates:
            return None

        # filtra já visitados
        min_repeat = 0.5
        filtered = [
            (x, y, d) for x, y, d in candidates
            if all(math.hypot(x - vx, y - vy) > min_repeat for vx, vy in self.visited_goals)
        ]
        if not filtered:
            return None

        # >>> MUDANÇA AQUI: escolhe o ponto MAIS DISTANTE <<< 
        x, y, _ = max(filtered, key=lambda c: c[2])

        # monta PoseStamped
        yaw = math.atan2(y - ry, x - rx)
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp    = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw/2.0)
        p.pose.orientation.w = math.cos(yaw/2.0)
        return p

    def mission_loop(self):
        if self.phase != 'explore':
            return

        if self.explore_goal_future is None and self.explore_goal_handle is None:
            if not self.waypoint_client.wait_for_server(timeout_sec=0.1):
                return

            goal_pose = self.compute_next_frontier_goal()
            if goal_pose is None:
                self.get_logger().warn('⚠️  Nenhuma fronteira válida encontrada.')
                return

            self.last_goal = goal_pose
            gx = goal_pose.pose.position.x
            gy = goal_pose.pose.position.y
            self.get_logger().info(f'➡️  Explorando ponto em ({gx:.2f}, {gy:.2f})')

            goal = FollowWaypoints.Goal()
            goal.poses = [goal_pose]
            self.explore_goal_future = self.waypoint_client.send_goal_async(goal)
            self.explore_goal_future.add_done_callback(self.explore_response_cb)

        elif self.flag_seen and self.explore_goal_handle:
            self.get_logger().info('🏳️  Bandeira vista! Cancelando exploração.')
            cancel_fut = self.explore_goal_handle.cancel_goal_async()
            cancel_fut.add_done_callback(self._on_explore_cancel)
            self.flag_servo_enabled_pub.publish(Bool(data=True))
            self.phase = 'servo'
            self.flag_seen = False

    def explore_response_cb(self, future):
        handle = future.result()
        self.explore_goal_future = None
        if not handle.accepted:
            self.get_logger().error('Exploração NÃO aceita!')
        else:
            self.get_logger().info('Exploração aceita pelo servidor')
            self.explore_goal_handle = handle
            handle.get_result_async().add_done_callback(self.on_explore_complete)

    def on_explore_complete(self, future):
        self.get_logger().info('✅ Meta concluída, liberando handle.')
        if self.last_goal:
            x = self.last_goal.pose.position.x
            y = self.last_goal.pose.position.y
            self.visited_goals.append((x, y))
            self.last_goal = None
        self.explore_goal_handle = None

    def _on_explore_cancel(self, future):
        self.get_logger().info('Exploração cancelada')
        self.explore_goal_handle = None

    def flag_arrived_cb(self, msg: Bool):
        if msg.data and self.phase == 'servo':
            self.phase = 'return'
            self.flag_servo_enabled_pub.publish(Bool(data=False))
            self.send_return_goal()

    def send_return_goal(self):
        if self.initial_pose is None:
            self.get_logger().error('Pose inicial não disponível!')
            return
        if not self.waypoint_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('FollowWaypoints não respondeu.')
            return

        pose = PoseStamped()
        pose.header.frame_id = self.initial_pose.header.frame_id
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose            = self.initial_pose.pose

        self.get_logger().info('↩️  Regressando à base via FollowWaypoints')
        goal = FollowWaypoints.Goal()
        goal.poses = [pose]
        self.return_goal_future = self.waypoint_client.send_goal_async(goal)
        self.return_goal_future.add_done_callback(self.return_response_cb)

    def return_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Retorno NÃO aceito!')
            return
        self.return_goal_handle = handle
        handle.get_result_async().add_done_callback(self.return_complete_cb)

    def return_complete_cb(self, future):
        self.get_logger().info('🚩 Chegou à base, missão finalizada!')
        self.phase = 'finished'

def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
