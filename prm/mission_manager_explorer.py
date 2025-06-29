#!/usr/bin/env python3
# mission_manager_enhanced_no_repeat_wait.py

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
import math
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs.tf2_geometry_msgs

HSV_MIN = (110, 50, 50)
HSV_MAX = (125, 255, 255)

GOAL_ARRIVED_THRESHOLD = 0.3
IMMOBILITY_TIMEOUT_SEC = 20.0
MOVEMENT_THRESHOLD = 0.05

class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self.bridge = CvBridge()
        self.flag_seen = False
        self.initial_pose = None
        self.current_pose = None
        self.phase = 'explore'
        self.map: OccupancyGrid = None

        self.visited_goals = []
        self.last_goal = None

        self.selection_mode = 'max_x'

        self.goal_start_time = None
        self.last_position = None
        self.stationary_start_time = None
        self.waiting_for_result = False

        self._force_next_goal_skip_visited = False  # ⚠️ NOVO: indica se vai usar "skip visited"

        self.flag_servo_enabled_pub = self.create_publisher(Bool, '/flag_servo_enable', 1)
        self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, '/flag_servo_arrived', self.flag_arrived_cb, 1)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)

        self.explore_goal_future = None
        self.explore_goal_handle = None
        self.return_goal_future = None
        self.return_goal_handle = None

        self.distancia_frontal_frentinha = float('inf')
        self.timer = self.create_timer(2.0, self.mission_loop)

    def map_cb(self, msg: OccupancyGrid):
        self.map = msg

    def scan_callback(self, msg: LaserScan):
        indices = list(range(340, 360)) + list(range(0, 21))
        dists = [msg.ranges[i] for i in indices if not np.isnan(msg.ranges[i])]
        self.distancia_frontal_frentinha = min(dists) if dists else float('inf')

    def odom_cb(self, msg: Odometry):
        if self.initial_pose is None:
            ps_odom = PoseStamped()
            ps_odom.header.frame_id = 'odom'
            ps_odom.header.stamp = msg.header.stamp
            ps_odom.pose = msg.pose.pose
            try:
                ps_map = self.tf_buffer.transform(ps_odom, 'map', timeout=Duration(seconds=1.0))
                ps_map.header.stamp = self.get_clock().now().to_msg()
                self.initial_pose = ps_map
                self.get_logger().info(
                    f'Pose inicial salva (map): ({ps_map.pose.position.x:.2f}, {ps_map.pose.position.y:.2f})'
                )
            except TransformException as ex:
                self.get_logger().warn(f'Falha ao converter pose inicial: {ex}')

        ps_curr = PoseStamped()
        ps_curr.header.frame_id = 'odom'
        ps_curr.header.stamp = msg.header.stamp
        ps_curr.pose = msg.pose.pose
        try:
            ps_map_curr = self.tf_buffer.transform(ps_curr, 'map', timeout=Duration(seconds=0.2))
            ps_map_curr.header.stamp = self.get_clock().now().to_msg()
            self.current_pose = ps_map_curr
        except TransformException:
            pass

        if (
            self.phase == 'explore'
            and self.last_goal is not None
            and self.explore_goal_handle is not None
            and self.current_pose is not None
        ):
            dx = self.current_pose.pose.position.x - self.last_goal.pose.position.x
            dy = self.current_pose.pose.position.y - self.last_goal.pose.position.y
            dist_to_goal = math.hypot(dx, dy)

            if dist_to_goal < GOAL_ARRIVED_THRESHOLD:
                self.get_logger().info(
                    f'✔️ Meta atingida ({dist_to_goal:.2f} m). Cancelando goal.'
                )
                cancel_fut = self.explore_goal_handle.cancel_goal_async()
                cancel_fut.add_done_callback(self._on_explore_force_cancel)
                return

            now = self.get_clock().now()
            x_curr = self.current_pose.pose.position.x
            y_curr = self.current_pose.pose.position.y

            if self.last_position is None:
                self.last_position = (x_curr, y_curr)
                self.stationary_start_time = now
            else:
                moved = math.hypot(x_curr - self.last_position[0], y_curr - self.last_position[1])
                if moved > MOVEMENT_THRESHOLD:
                    self.last_position = (x_curr, y_curr)
                    self.stationary_start_time = now
                else:
                    elapsed = (now - self.stationary_start_time).nanoseconds / 1e9
                    if elapsed > IMMOBILITY_TIMEOUT_SEC:
                        self.get_logger().warn(
                            f'⏰ Imobilidade detectada por {elapsed:.1f}s. Cancelando goal.'
                        )
                        # ⚠️ Marca flag para tentar ponto alternativo
                        self._force_next_goal_skip_visited = True
                        cancel_fut = self.explore_goal_handle.cancel_goal_async()
                        cancel_fut.add_done_callback(self._on_explore_force_cancel)

    def _on_explore_force_cancel(self, future):
        self.get_logger().info('🔥 Goal cancelado.')
        if self.last_goal:
            self.visited_goals.append((
                self.last_goal.pose.position.x,
                self.last_goal.pose.position.y
            ))
            self.last_goal = None
        self.explore_goal_handle = None
        self.last_position = None
        self.stationary_start_time = None
        self.waiting_for_result = False

    def camera_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_MIN, HSV_MAX)
        self.flag_seen = np.count_nonzero(mask) > 1200

    def compute_next_frontier_goal(self) -> PoseStamped:
        if self.map is None or self.initial_pose is None:
            return None

        w, h = self.map.info.width, self.map.info.height
        res = self.map.info.resolution
        ox, oy = self.map.info.origin.position.x, self.map.info.origin.position.y
        grid = np.array(self.map.data).reshape((h, w), order='C')

        free = (grid == 0).astype(np.uint8)
        unknown = (grid == -1).astype(np.uint8)
        unk_dil = cv2.dilate(unknown, np.ones((3, 3), np.uint8), iterations=1)
        frontier = free & (unk_dil > 0)

        # 🔼 AUMENTA margem para paredes
        dt = cv2.distanceTransform((grid != 100).astype(np.uint8), cv2.DIST_L2, 5) * res
        frontier &= (dt > 0.4)

        ys, xs = np.where(frontier)
        if xs.size == 0:
            self.get_logger().warn('⚠️ Nenhuma fronteira encontrada.')
            # ⚠️ Zera lista de visitados
            self.visited_goals.clear()
            return None

        if self.selection_mode == 'max_x':
            idx = int(np.argmax(xs))
            x_best = ox + (xs[idx] + 0.5) * res
            y_best = oy + (ys[idx] + 0.5) * res
            self.get_logger().info(f'📍Modo MAX_X: ({x_best:.2f}, {y_best:.2f})')
            if x_best > 7.0:
                self.selection_mode = 'farthest'
                self.get_logger().info('🔄 Trocando para modo FARTHEST.')
        else:
            MIN_CLUSTER_SIZE = 20
            INFLATION_RADIUS = 0.3

            n_labels, labels = cv2.connectedComponents(frontier.astype(np.uint8))
            centroids = []
            for lbl in range(1, n_labels):
                ys_lbl, xs_lbl = np.where(labels == lbl)
                if len(xs_lbl) < MIN_CLUSTER_SIZE:
                    continue
                cx = xs_lbl.mean()
                cy = ys_lbl.mean()
                x = ox + (cx + 0.5) * res
                y = oy + (cy + 0.5) * res
                if x > 3.7:
                    centroids.append((x, y))

            if not centroids:
                self.get_logger().warn('⚠️ Nenhum cluster válido.')
                self.visited_goals.clear()
                return None

            ref_pts = self.visited_goals or [(self.initial_pose.pose.position.x, self.initial_pose.pose.position.y)]

            def min_dist(pt):
                return min(math.hypot(pt[0] - vx, pt[1] - vy) for vx, vy in ref_pts)

            # Ordena todos por distância crescente
            ordered = sorted(centroids, key=min_dist, reverse=True)

            # Se imobilidade: tenta o primeiro ainda não visitado
            for x_c, y_c in ordered:
                if self._force_next_goal_skip_visited and (x_c, y_c) in self.visited_goals:
                    continue
                x_best, y_best = x_c, y_c
                break
            else:
                # Todos repetidos: pega o mais distante mesmo
                x_best, y_best = ordered[0]

            best_dist = min_dist((x_best, y_best))
            self.get_logger().info(
                f'📍Modo FARTHEST: ({x_best:.2f}, {y_best:.2f}), min_dist = {best_dist:.2f} m'
            )
            self._force_next_goal_skip_visited = False

        if (x_best, y_best) in self.visited_goals:
            x_best += 1.0
            self.get_logger().warn(
                f'⚠️ Ponto já visitado. Novo ponto: ({x_best:.2f}, {y_best:.2f})'
            )

        quat = self.current_pose.pose.orientation if self.current_pose else PoseStamped().pose.orientation

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x_best
        goal.pose.position.y = y_best
        goal.pose.orientation = quat

        return goal

    # resto da classe igual...
