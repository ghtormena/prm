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

        # controle de fases: 'explore' → coleta bandeira, 'return' → volta base, 'finished'
        self.phase = 'explore'

        # para monitorar metas e imobilidade
        self.predefined_goals = []
        self.return_goals     = []
        self.current_goal_index = 0
        self.current_return_index = 0
        self.last_goal = None
        self.last_position = None
        self.stationary_start_time = None
        self.waiting_for_result = False
        self.explore_goal_handle  = None

        # publisher para ativar/desativar servo de bandeira
        self.flag_servo_enabled_pub = self.create_publisher(Bool, '/flag_servo_enable', 1)

        # cliente de action para FollowWaypoints
        self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # TF2
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscriptions
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/flag_servo_arrived', self.flag_arrived_cb, 1)

        # carregar lista de pontos DA IDA
        ida_coords = [
            {'x': -6.667668342590332,  'y': -1.128657579421997,   'z': -0.3011794918070548,  'w': 0.9535674667871404},
            {'x': -5.428117752075195,  'y': -1.662007451057434,   'z':  0.20656852737440487, 'w': 0.9784321353565457},
            {'x': -4.255116939544678,  'y': -3.815044403076172,   'z':  0.3379921788708857,  'w': 0.9411489186213365},
            {'x': -4.026280879974365,  'y': -2.2470624446868896,  'z':  0.11079815058486918, 'w': 0.9938429301589727},
            {'x': -3.017289161682129,  'y': -3.7028467655181885,  'z':  0.10201702770798039, 'w': 0.9947826526722454},
            {'x': -1.2365496158599854, 'y': -3.902912139892578,   'z':  0.1488987567070383,  'w': 0.9888524461470974},
            {'x': -0.09009289741516113,'y': -3.901170253753662,   'z':  0.14931356145487926, 'w': 0.9887898969779475},
            {'x':  0.9365971088409424, 'y': -3.954685688018799,   'z':  0.14949652353719117, 'w': 0.9887622512263977},
            {'x':  1.7995119094848633, 'y': -3.984099864959717,   'z':  0.30582670831099973, 'w': 0.952087193739974},
            {'x':  2.5738415718078613, 'y': -3.9763925075531006,  'z':  0.09043442415158165, 'w': 0.995902412351919},
            {'x':  3.457817554473877,  'y': -3.301541328430176,   'z': -0.306858882466113,   'w': 0.9517550242849513},
            {'x':  4.451894283294678,  'y': -2.3707022666931152,  'z': -0.4586883281524881,  'w': 0.8885972190012049},
            {'x':  4.298230171203613,  'y': -4.019372463226318,   'z':  0.36270022306744315, 'w': 0.9319058687371955},
            {'x':  5.457037925720215,  'y': -3.3053269386291504,  'z': -0.35194062069217225, 'w': 0.9360223285300455},
            {'x':  7.325922966003418,  'y': -3.238062858581543,   'z': -0.4071037024990537,  'w': 0.9133819438830406},
            {'x':  8.301312446594238,  'y': -3.8284752368927,     'z':  0.04122450648476514, 'w': 0.9991499087049388},
            {'x':  7.809718132019043,  'y': -1.5080010890960693,  'z':  0.4399198076877409,  'w': 0.8980370609301049},
            {'x':  6.915520668029785,  'y': -1.4652411937713623,  'z':  0.7329174968218405,  'w': 0.6803175309018631},
            {'x':  8.582658767700195,  'y': -0.7100906372070312,  'z':  0.7888438234696868,  'w': 0.6145937049577759},
            {'x':  8.356708526611328, 'y': -0.5125732421875,    'z':  0.9336018607861866,  'w': 0.35831210632152766},
            {'x':  8.356708526611328, 'y': -0.5125732421875,    'z':  0.9336018607861866,  'w': 0.35831210632152766},
            {'x':  8.356708526611328, 'y': -0.5125732421875,    'z':  0.9336018607861866,  'w': 0.35831210632152766},
            # {'x':  4.3713459968566895, 'y':  3.6725809574127197,  'z': -0.9827477768569974,  'w': 0.18495082341700786},
            # {'x':  5.124448776245117,  'y':  1.8883694410324097,  'z': -0.6727684763568778,  'w': 0.7398530781313579},
            # {'x':  5.173882484436035,  'y': -0.15012018382549286, 'z': -0.8142586958197244,  'w': 0.580502175949377},
            # {'x':  5.137442588806152,  'y': -2.201612949371338,   'z': -0.6666417087232644,  'w': 0.7453783148110268},
            # {'x':  7.08064079284668,   'y': -0.71009361743927,    'z':  0.4193381507584788,  'w': 0.9078301136878305},
            # {'x':  7.08064079284668,   'y': -0.71009361743927,    'z':  0.40377802773832966, 'w': 0.9148569857173003},
            # {'x':  3.8900489807128906, 'y':  2.170248031616211,   'z': -0.9987746642200839,  'w': 0.04948909083887595},
            # {'x':  4.3992438316345215, 'y': -3.0447745323181152,  'z': -0.19588039094933477, 'w': 0.9806277950586225},
            # {'x':  4.36100435256958,   'y': -1.258830189704895,   'z': -0.3404193583754608,  'w': 0.9402737157036984},
            # {'x':  7.375908851623535,  'y':  1.5346821546554565,  'z':  0.4450622986284255,  'w': 0.8954996093464151},
        ]
        for p in ida_coords:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = p['x']
            ps.pose.position.y = p['y']
            ps.pose.orientation.z = p['z']
            ps.pose.orientation.w = p['w']
            self.predefined_goals.append(ps)

        # carregar lista de pontos DA VOLTA (mesmo formato)
        return_coords = [
            {'x':  8.320028305053711,'y': -1.7023584842681885,'z': -0.9410833489878457,'w': 0.3381747037520999},
            {'x':  6.792913436889648,'y': -3.0988731384277344,'z': -0.890980704284877,'w': 0.45404117059141724},
            {'x':  4.571274757385254,'y': -3.6665120124816895,'z': -0.999999409421849,'w': 0.0010868099894770914},
            {'x':  2.6192400455474854,'y': -3.4626288414001465,'z': -0.9994668522014938,'w': 0.03264982925586606},
            {'x':  0.7548511028289795,'y': -3.630343437194824,'z': -0.9999068978260418,'w': 0.013645353784415766},
            {'x': -1.444184422492981,'y': -3.689713478088379, 'z':  0.9999348471056805,'w': 0.011414970159373239},
            {'x': -2.4925363063812256,'y': -3.2741167545318604,'z':  0.9994778693866455,'w': 0.032310812529734494},
            {'x': -3.7896039485931396,'y': -1.7586870193481445,'z':  0.9706503645901756,'w': 0.24049505134401208},
            {'x': -5.46696662902832, 'y': -1.3563283681869507, 'z': -0.922341119051282,'w': 0.3863765781038349},
            {'x': -6.884380340576172,'y': -1.1602972745895386,'z':  0.9706799428265787,'w': 0.24037564060068592},
            {'x': -7.883652210235596,'y': -0.21716314554214478,'z':  0.889148251624754, 'w': 0.45761925946428783},
        ]
        for p in return_coords:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = p['x']
            ps.pose.position.y = p['y']
            ps.pose.orientation.z = p['z']
            ps.pose.orientation.w = p['w']
            self.return_goals.append(ps)

        # temporizador principal
        self.timer = self.create_timer(2.0, self.mission_loop)


    def scan_cb(self, msg: LaserScan):
        idx = list(range(340,360)) + list(range(0,21))
        d = [msg.ranges[i] for i in idx if not np.isnan(msg.ranges[i])]
        self.dist_front = min(d) if d else float('inf')


    def odom_cb(self, msg: Odometry):
        # grava initial_pose uma vez
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

        # monitora proximidade e imobilidade tanto em 'explore' quanto em 'return'
        if self.phase in ('explore','return') and self.last_goal and self.explore_goal_handle and self.current_pose:
            dx = self.current_pose.pose.position.x - self.last_goal.pose.position.x
            dy = self.current_pose.pose.position.y - self.last_goal.pose.position.y
            dist = math.hypot(dx, dy)

            if dist < GOAL_ARRIVED_THRESHOLD:
                # meta alcançada
                self.get_logger().info(f'✔️ Meta atingida ({dist:.2f} m). Cancelando goal.')
                self._cancel_current_goal()
                return

            # imobilidade
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
        # 1) se fase de explorar
        if self.phase == 'explore':
            # checa flag antes de enviar próximo
            if self.flag_seen and self.explore_goal_handle:
                self.get_logger().info('🏳️ Bandeira detectada! Cancelando exploração.')
                self._cancel_current_goal()
                self.flag_servo_enabled_pub.publish(Bool(data=True))
                self.phase = 'servo'
                self.waiting_for_result = False
                self.flag_seen = False
                return

            # bloqueia se esperando resultado
            if self.waiting_for_result:
                return

            # envia próximo ponto de ida
            if not self.explore_goal_handle:
                if self.current_goal_index >= len(self.predefined_goals):
                    # se esgotou, aguarda bandeira
                    return
                goal = self.predefined_goals[self.current_goal_index]
                goal.header.stamp = self.get_clock().now().to_msg()
                self.last_goal = goal
                self.last_position = None
                self.stationary_start_time = None
                self.get_logger().info(f'➡️ Explorando ponto {self.current_goal_index+1}: '
                    f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')
                self._send_goal(goal)

        # 2) se fase de retorno
        elif self.phase == 'return':
            # bloqueia se esperando resultado
            if self.waiting_for_result:
                return

            # envia próximo ponto de volta
            if not self.explore_goal_handle:
                if self.current_return_index >= len(self.return_goals):
                    self.get_logger().info('✅ Volta à base concluída!')
                    self.phase = 'finished'
                    return
                goal = self.return_goals[self.current_return_index]
                goal.header.stamp = self.get_clock().now().to_msg()
                self.last_goal = goal
                self.last_position = None
                self.stationary_start_time = None
                self.get_logger().info(f'↩️ Voltando ponto {self.current_return_index+1}: '
                    f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')
                self._send_goal(goal)

        # demais fases (servo, finished) não entram aqui


    def _send_goal(self, pose: PoseStamped):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [pose]
        fut = self.waypoint_client.send_goal_async(goal_msg)
        fut.add_done_callback(self._on_goal_response)
        self.waiting_for_result = True


    def _on_goal_response(self, future):
        handle = future.result()
        self.waiting_for_result = False
        if not handle.accepted:
            self.get_logger().error('Goal NÃO aceito!')
            return
        self.get_logger().info('Goal aceito pelo servidor.')
        self.explore_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)


    def _on_goal_result(self, future):
        # chamado quando o waypoint é completado ou cancelado
        self.explore_goal_handle = None
        if self.phase == 'explore':
            self.current_goal_index += 1
        elif self.phase == 'return':
            self.current_return_index += 1


    def _cancel_current_goal(self):
        if self.explore_goal_handle:
            fut = self.explore_goal_handle.cancel_goal_async()
            fut.add_done_callback(lambda _: self._on_goal_result(fut))


    def flag_arrived_cb(self, msg: Bool):
        if msg.data and self.phase == 'servo':
            self.get_logger().info('🏁 Bandeira pegou → iniciando retorno')
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
