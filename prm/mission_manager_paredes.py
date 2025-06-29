#!/usr/bin/env python3
# mission_manager_predefined_only.py

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
import math
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs.tf2_geometry_msgs

# intervalo HSV da bandeira azul
HSV_MIN = (110, 50, 50)
HSV_MAX = (125, 255, 255)

GOAL_ARRIVED_THRESHOLD = 0.3    # distância para considerar meta alcançada
IMMOBILITY_TIMEOUT_SEC = 20.0   # tempo máximo parado antes de cancelar
MOVEMENT_THRESHOLD = 0.05       # deslocamento mínimo para "resetar" imobilidade


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        # utilitários
        self.bridge = CvBridge()
        self.flag_seen = False
        self.initial_pose = None
        self.current_pose = None

        # fases: 'explore' → coleta bandeira, 'return' → volta base, 'finished'
        self.phase = 'explore'

        # listas de waypoints e índices
        self.predefined_goals = []
        self.return_goals     = []
        self.current_goal_index   = 0
        self.current_return_index = 0

        # para controlar envio/cancelamento
        self.last_goal            = None
        self.last_position        = None
        self.stationary_start_time= None
        self.waiting_for_result   = False
        self.explore_goal_handle  = None  # handle do FollowWaypoints ativo

        # publisher p/ ativar servo de bandeira
        self.flag_servo_enabled_pub = self.create_publisher(Bool, '/flag_servo_enable', 1)

        # cliente de action FollowWaypoints
        self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # TF2
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscriptions
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/flag_servo_arrived', self.flag_arrived_cb, 1)

        # --- constrói lista de pontos da IDA ---
        ida_coords = [
            {'x': -7.899073600769043, 'y': -1.535544514656067, 'z': -0.714083322695916, 'w': 0.7000607175435287},
            {'x': -7.246140956878662, 'y': -3.4308130741119385,'z': -0.07143477008820631,'w': 0.9974452734974711},
            {'x': -6.234116077423096, 'y': -2.8057761192321777,'z':  0.6787648009428306,'w': 0.7343557346416241},
            {'x': -5.531503200531006, 'y': -1.3525712490081787,'z':  0.38592697899479816,'w': 0.9225293311781195},
            {'x': -5.316287994384766, 'y': -0.12701988220214844,'z':  0.008216733210999753,'w': 0.999966242077871},
            {'x': -4.1534423828125,   'y': -0.40097397565841675,'z': -0.6432625600036543,'w': 0.7656456614502201},
            {'x': -3.4451465606689453,'y': -2.8850226402282715,'z': -0.37033234146432364,'w': 0.9288993254737306},
            {'x': -2.604991912841797, 'y': -3.7553935050964355,'z':  0.03217734468143624,'w': 0.9994821751733505},
            {'x': -1.3940773010253906,'y': -3.6543655395507812,'z':  0.0769356938430921,'w': 0.9970360570274688},
            {'x':  0.19152259826660156,'y': -3.783205270767212,'z':  0.07450944179583473,'w': 0.9972203081983806},
            {'x':  2.1191444396972656, 'y': -3.624685764312744,'z':  0.027583654293796344,'w': 0.9996194986172491},
            {'x':  3.1197853088378906, 'y': -1.44779372215271, 'z':  0.3264708001999547,'w': 0.9452072876447797},
            {'x':  4.350732326507568,  'y': -0.3532865643501282,'z':  0.08195244938842597,'w': 0.9966362405809039},
            {'x':  5.902196407318115,  'y': -0.41282281279563904,'z': -0.4917315450096124,'w': 0.8707468562346117},
            {'x':  6.293968200683594,  'y': -3.1980769634246826,'z': -0.4475574718142307,'w': 0.8942551701965463},
            {'x':  7.580601692199707,  'y': -3.631807327270508, 'z':  0.6388236871554824,'w': 0.7693531677513743},
            {'x':  7.931103706359863,  'y': -1.9462791681289673,'z':  0.7025743987328159,'w': 0.7116102966127051},
            {'x':  7.99854850769043,   'y': -1.01876962184906,  'z':  0.741090356760994, 'w': 0.6714053046527578},
        ]
        for p in ida_coords:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = p['x']
            ps.pose.position.y = p['y']
            ps.pose.orientation.z = p['z']
            ps.pose.orientation.w = p['w']
            self.predefined_goals.append(ps)

        # --- constrói lista de pontos da VOLTA ---
        return_coords = [
            {'x':  8.007593154907227,  'y': -1.5081161260604858, 'z': -0.7266620133623654, 'w': 0.6869951370542251},
            {'x':  7.7794108390808105, 'y': -3.248244524002075,  'z': -0.984586994633775,  'w': 0.17489554024625822},
            {'x':  6.778037071228027,  'y': -3.6325254440307617, 'z':  0.998832001614458,  'w': 0.04831803546146669},
            {'x':  6.0378193855285645, 'y': -2.5516371726989746, 'z':  0.7631504512366931, 'w': 0.6462208513946541},
            {'x':  5.550597667694092,  'y': -0.13249456882476807,'z': -0.9999990076685495,'w': 0.0014087802937327156},
            {'x':  3.8934903144836426, 'y': -0.9858461618423462, 'z': -0.8618868462861337,'w': 0.5071006450389732},
            {'x':  2.4036667346954346, 'y': -2.951819658279419,  'z': -0.9989887524577789,'w': 0.04496078805860151},
            {'x':  0.9034090042114258, 'y': -3.636523723602295,  'z':  0.9942985886477755,'w': 0.10663168672135628},
            {'x': -0.9722890853881836, 'y': -3.260240316390991,  'z': -0.9998506447109899,'w': 0.017282600238904964},
            {'x': -1.942845344543457,  'y': -3.1429147720336914, 'z':  0.9989030641629595,'w': 0.0468259373195075},
            {'x': -3.171311855316162,  'y': -2.554518222808838,  'z':  0.8605203805944691,'w': 0.5094160132755449},
            {'x': -4.010497093200684,  'y': -1.578209400177002,  'z':  0.8035915258018305,'w': 0.5951811990137844},
            {'x': -4.500006198883057,  'y':  0.2464061677455902,  'z': -0.9960084539131498,'w': 0.08925894763852522},
            {'x': -5.89094352722168,   'y': -0.09387589991092682,'z': -0.6981104698071203,'w': 0.7159900641389388},
            {'x': -6.052659511566162,  'y': -2.5335893630981445, 'z': -0.7448930260601531,'w': 0.6671839174672514},
            {'x': -6.996476173400879,  'y': -3.5382254123687744, 'z':  0.9983056967359374,'w': 0.058187076439485556},
            {'x': -8.127896308898926,  'y': -2.751328229904175,  'z':  0.6359035383687511,'w': 0.7717685468390781},
            {'x': -8.202762603759766,  'y': -0.6968315839767456, 'z':  0.7127353587852339,'w': 0.7014330390973068},
            {'x': -7.970743179321289,  'y': -0.34670981764793396,'z':  0.4327323409608681,'w': 0.9015224462466406},
        ]
        for p in return_coords:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = p['x']
            ps.pose.position.y = p['y']
            ps.pose.orientation.z = p['z']
            ps.pose.orientation.w = p['w']
            self.return_goals.append(ps)

        # timer principal
        self.timer = self.create_timer(2.0, self.mission_loop)


    def scan_cb(self, msg: LaserScan):
        idx = list(range(340,360)) + list(range(0,21))
        d = [msg.ranges[i] for i in idx if not np.isnan(msg.ranges[i])]
        self.dist_front = min(d) if d else float('inf')


    def odom_cb(self, msg: Odometry):
        # fixa initial_pose uma vez
        if self.initial_pose is None:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.header.stamp    = msg.header.stamp
            ps.pose            = msg.pose.pose
            try:
                mp = self.tf_buffer.transform(ps, 'map', timeout=Duration(seconds=1.0))
                mp.header.stamp = self.get_clock().now().to_msg()
                self.initial_pose = mp
                self.get_logger().info(
                    f'Pose inicial salva: ({mp.pose.position.x:.2f}, {mp.pose.position.y:.2f})'
                )
            except TransformException as e:
                self.get_logger().warn(f'TF falhou: {e}')

        # atualiza current_pose
        ps = PoseStamped()
        ps.header.frame_id = 'odom'
        ps.header.stamp    = msg.header.stamp
        ps.pose            = msg.pose.pose
        try:
            mp = self.tf_buffer.transform(ps, 'map', timeout=Duration(seconds=0.2))
            mp.header.stamp = self.get_clock().now().to_msg()
            self.current_pose = mp
        except TransformException:
            return

        # monitora proximidade e imobilidade em qualquer fase de trajetos
        if self.phase in ('explore','return') and self.last_goal and self.explore_goal_handle and self.current_pose:
            dx = self.current_pose.pose.position.x - self.last_goal.pose.position.x
            dy = self.current_pose.pose.position.y - self.last_goal.pose.position.y
            dist = math.hypot(dx, dy)

            if dist < GOAL_ARRIVED_THRESHOLD:
                self.get_logger().info(f'✔️ Meta ({dist:.2f}m) atingida. Cancelando goal.')
                self._cancel_current_goal()
                return

            # verifica imobilidade
            now = self.get_clock().now()
            x,y = self.current_pose.pose.position.x, self.current_pose.pose.position.y
            if self.last_position is None:
                self.last_position = (x,y)
                self.stationary_start_time = now
            else:
                if math.hypot(x-self.last_position[0], y-self.last_position[1]) > MOVEMENT_THRESHOLD:
                    self.last_position = (x,y)
                    self.stationary_start_time = now
                else:
                    secs = (now - self.stationary_start_time).nanoseconds / 1e9
                    if secs > IMMOBILITY_TIMEOUT_SEC:
                        self.get_logger().warn(f'⏰ Imobilidade {secs:.1f}s. Cancelando goal.')
                        self._cancel_current_goal()


    def camera_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_MIN, HSV_MAX)
        self.flag_seen = np.count_nonzero(mask) > 1200


    def mission_loop(self):
        # 1) Exploração
        if self.phase == 'explore':
            # se viu bandeira, interrompe
            if self.flag_seen and self.explore_goal_handle:
                self.get_logger().info('🏳️ Bandeira detectada → cancelando ida.')
                self._cancel_current_goal()
                self.flag_servo_enabled_pub.publish(Bool(data=True))
                self.phase = 'servo'
                return

            if self.waiting_for_result:
                return

            if self.current_goal_index < len(self.predefined_goals) and not self.explore_goal_handle:
                goal = self.predefined_goals[self.current_goal_index]
                goal.header.stamp = self.get_clock().now().to_msg()
                self.last_goal = goal
                self.last_position = None
                self.stationary_start_time = None
                self.get_logger().info(
                    f'➡️ Ida {self.current_goal_index+1}/{len(self.predefined_goals)}: '
                    f'({goal.pose.position.x:.2f},{goal.pose.position.y:.2f})'
                )
                self._send_goal(goal)

        # 2) Retorno
        elif self.phase == 'return':
            if self.waiting_for_result:
                return

            if self.current_return_index < len(self.return_goals) and not self.explore_goal_handle:
                goal = self.return_goals[self.current_return_index]
                goal.header.stamp = self.get_clock().now().to_msg()
                self.last_goal = goal
                self.last_position = None
                self.stationary_start_time = None
                self.get_logger().info(
                    f'↩️ Volta {self.current_return_index+1}/{len(self.return_goals)}: '
                    f'({goal.pose.position.x:.2f},{goal.pose.position.y:.2f})'
                )
                self._send_goal(goal)
            elif self.current_return_index >= len(self.return_goals):
                self.get_logger().info('✅ Volta concluída. Missão finalizada.')
                self.phase = 'finished'


    def _send_goal(self, pose: PoseStamped):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [pose]
        send_fut = self.waypoint_client.send_goal_async(goal_msg)
        send_fut.add_done_callback(self._on_goal_response)
        self.waiting_for_result = True


    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal NÃO aceito!')
            self.waiting_for_result = False
            return
        self.get_logger().info('Goal aceito.')
        self.explore_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)


    def _on_goal_result(self, future):
        # called when goal finishes (sucesso ou cancelado)
        self.explore_goal_handle = None
        self.waiting_for_result = False

        if self.phase == 'explore':
            self.current_goal_index += 1
        elif self.phase == 'return':
            self.current_return_index += 1


    def _cancel_current_goal(self):
        if self.explore_goal_handle:
            cancel_fut = self.explore_goal_handle.cancel_goal_async()
            # não incrementamos aqui: deixamos que _on_goal_result trate
            cancel_fut.add_done_callback(lambda _: self.get_logger().info('Goal cancelado.'))


    def flag_arrived_cb(self, msg: Bool):
        if msg.data and self.phase == 'servo':
            self.get_logger().info('🏁 Bandeira pega → iniciando retorno.')
            self.phase = 'return'
            self.current_return_index = 0
            self.waiting_for_result = False
            self.flag_servo_enabled_pub.publish(Bool(data=False))


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
