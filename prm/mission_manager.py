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
import tf2_geometry_msgs  # para poder usar tf2_buffer.transform em PoseStamped


# intervalo HSV da bandeira
HSV_MIN = (86, 0, 6)
HSV_MAX = (94, 255, 255)

# distância (em metros) abaixo da qual consideramos “meta alcançada” e cancelamos o goal
GOAL_ARRIVED_THRESHOLD = 0.3


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # utilitários
        self.bridge = CvBridge()
        self.flag_seen = False
        self.initial_pose = None       # PoseStamped em 'map' onde o robô começou
        self.current_pose = None       # PoseStamped em 'map' da pose atual do robô
        self.phase = 'explore'         # fases: 'explore', 'servo', 'return', 'finished'
        self.map: OccupancyGrid = None # última OccupancyGrid recebida

        # metas já visitadas (lista de (x,y)) e última meta enviada
        self.visited_goals = []
        self.last_goal = None          # PoseStamped do último waypoint enviado

        # publisher para habilitar/desabilitar o FlagServo
        self.flag_servo_enabled_pub = self.create_publisher(Bool, '/flag_servo_enable', 1)

        # cliente de ação para FollowWaypoints do Nav2
        self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # TF2 (buffer e listener) para converter entre 'odom' e 'map'
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # assinaturas
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Bool, '/flag_servo_arrived', self.flag_arrived_cb, 1)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)

        # controle de “futuro / handle” para o goal de exploração e retorno
        self.explore_goal_future = None
        self.explore_goal_handle = None
        self.return_goal_future  = None
        self.return_goal_handle  = None

        # timer principal que roda a cada 2 s
        self.timer = self.create_timer(2.0, self.mission_loop)

    def map_cb(self, msg: OccupancyGrid):
        """Recebe o OccupancyGrid e armazena em self.map."""
        self.map = msg

    def odom_cb(self, msg: Odometry):
        """
        1) Se ainda não definimos a pose inicial, converte a pose em 'odom' para 'map' e salva em initial_pose.
        2) A cada mensagem de odometria, também atualizamos current_pose (pose atual em 'map').
        3) Se estivermos em fase ‘explore’ e houver um goal pendente, checamos a distância para current_pose.
           Se < GOAL_ARRIVED_THRESHOLD, cancelamos o goal e marcamos como “alcançado”.
        """

        # 1) Registra initial_pose se ainda não foi definida
        if self.initial_pose is None:
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
                    f'{ps_map.pose.position.x:.2f}, {ps_map.pose.position.y:.2f}'
                )
            except TransformException as ex:
                self.get_logger().warn(f'Falha no TF ao converter pose inicial: {ex}')

        # 2) Atualiza current_pose a cada odometria, para uso em limiar de distância
        ps_odom_curr = PoseStamped()
        ps_odom_curr.header.frame_id = 'odom'
        ps_odom_curr.header.stamp    = msg.header.stamp
        ps_odom_curr.pose            = msg.pose.pose
        try:
            ps_map_curr = self.tf_buffer.transform(ps_odom_curr, 'map', timeout=Duration(seconds=0.2))
            ps_map_curr.header.stamp = self.get_clock().now().to_msg()
            self.current_pose = ps_map_curr
        except TransformException:
            # Não sobrescreve current_pose se falhar
            pass

        # 3) Se estivermos explorando, já enviamos um goal e current_pose existe,
        #    checa a distância e cancela se < limiar
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
                    f'✔️  Perto o suficiente da meta ({dist_to_goal:.2f} m). '
                    'Marcando como alcançada e cancelando goal.'
                )
                cancel_fut = self.explore_goal_handle.cancel_goal_async()
                cancel_fut.add_done_callback(self._on_explore_force_cancel)

    def _on_explore_force_cancel(self, future):
        """
        Callback quando o cancelamento é confirmado. 
        Marcamos a meta como concluída, igual ao on_explore_complete.
        """
        self.get_logger().info('🔥 Explore goal foi cancelado (por proximidade).')
        if self.last_goal:
            x = self.last_goal.pose.position.x
            y = self.last_goal.pose.position.y
            self.visited_goals.append((x, y))
            self.last_goal = None
        self.explore_goal_handle = None

    def camera_cb(self, msg: Image):
        """Atualiza self.flag_seen (True/False) se a bandeira estiver visível."""
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_MIN, HSV_MAX)
        self.flag_seen = np.count_nonzero(mask) > 1750

    def compute_next_frontier_goal(self) -> PoseStamped:
        """
        1) Busca blocos exatos de 2×2 células ocupadas (==100) em self.map, 
           onde **nenhuma** célula vizinha seja ocupada e **pelo menos 50 %** das vizinhas
           (que existam dentro do grid) sejam livres (==0). Se encontrar, retorna o centroide
           desse bloco como meta.
        2) Caso não encontre, segue a lógica original de frontiers:
           – células livres adjacentes a desconhecido
           – filtrar por distância > 0.5 m da initial_pose e não revisitado
           – escolher a célula MAIS DISTANTE
        """

        if self.map is None or self.initial_pose is None:
            return None

        w, h   = self.map.info.width, self.map.info.height
        res    = self.map.info.resolution
        ox, oy = self.map.info.origin.position.x, self.map.info.origin.position.y
        grid   = np.array(self.map.data).reshape((h, w), order='C')

        # ===== Passo 1: procurar blocos 2×2 exatos de ocupação =====
        # Percorre o grid procurando cada submatriz 2×2 == 100.
        for i in range(h - 1):
            for j in range(w - 1):
                count = 0
                # verifica se as quatro células (i,j), (i,j+1), (i+1,j), (i+1,j+1) são == 100
                if (
                    grid[i,   j]   == 100
                ):
                    if (grid[i,   j+1] == 100): count += 1
                    if (grid[i+1, j]   == 100): count += 1
                    if (grid[i+1, j+1] == 100): count += 1

                    if count >= 2:
                        # Agora vamos checar as células vizinhas à volta desse 2×2:
                        # Definimos “vizinhanca” como todo retângulo [i-1 … i+2] × [j-1 … j+2], 
                        # exceto as próprias quatro centrais (o bloco 2×2).
                        free_count = 0
                        unknown_count = 0
                        total_neighbors = 0
                        any_occupied_neighbor = False

                        for ii in range(i - 1, i + 3):
                            for jj in range(j - 1, j + 3):
                                # pula fora do grid
                                if ii < 0 or ii >= h or jj < 0 or jj >= w:
                                    continue
                                # pula as quatro centrais do bloco 2×2
                                if (ii == i   and jj == j)   or (ii == i   and jj == j + 1) \
                                or (ii == i+1 and jj == j)   or (ii == i+1 and jj == j + 1):
                                    continue

                                total_neighbors += 1
                                val = grid[ii, jj]
                                if val == 100:
                                    # se ANY vizinho for ocupado, descartamos esse bloco 2×2
                                    any_occupied_neighbor = True
                                    break
                                elif val == 0:
                                    free_count += 1
                                else:  # val == -1 (desconhecido)
                                    unknown_count += 1
                            if any_occupied_neighbor:
                                break

                        # Se houver algum vizinho ocupado, já descartamos este bloco.
                        if any_occupied_neighbor:
                            continue

                        # Agora exigimos que *pelo menos metade* dos vizinhos sejam livres.
                        # total_neighbors pode variar de 3 até 8, dependendo de estar na borda do mapa.
                        if total_neighbors > 0 and free_count >= (total_neighbors / 2.0):
                            # encontramos um bloco 2×2 válido. Calculamos seu centroide:
                            mean_i = i + 0.5  # (i + (i+1)) / 2
                            mean_j = j + 0.5
                            x = ox + (mean_j + 0.5) * res  # +0.5 para centro do pixel
                            y = oy + (mean_i + 0.5) * res

                            # orientação: copiamos a orientação atual do robô para que o Nav2 
                            # não tente acerta yaw ao chegar.
                            if self.current_pose is not None:
                                quat = self.current_pose.pose.orientation
                            else:
                                quat = PoseStamped().pose.orientation
                                quat.x = 0.0; quat.y = 0.0; quat.z = 0.0; quat.w = 1.0

                            p = PoseStamped()
                            p.header.frame_id = 'map'
                            p.header.stamp    = self.get_clock().now().to_msg()
                            p.pose.position.x = x
                            p.pose.position.y = y
                            p.pose.orientation = quat

                            self.get_logger().info(
                                f'🔍 Encontrado bloco 2×2 de obstáculos em [(i,j)=({i},{j})], '
                                f'{free_count}/{total_neighbors} vizinhos livres. '
                                f'Meta=({x:.2f}, {y:.2f}).'
                            )
                            return p

        # ===== Passo 2: se não achou bloco 2×2 válido, segue lógica original =====

        free    = (grid == 0).astype(np.uint8)
        unknown = (grid == -1).astype(np.uint8)

        # 2.1) fronteira = célula livre adjacente a pelo menos um desconhecido
        unk_dilated = cv2.dilate(unknown, np.ones((3, 3), np.uint8), iterations=1)
        frontier    = free & (unk_dilated > 0)

        # 2.2) folga de 0.2 m em relação a qualquer obstáculo (grid != 100)
        dt = cv2.distanceTransform((grid != 100).astype(np.uint8), cv2.DIST_L2, 5) * res
        frontier &= (dt > 0.2)

        pts = np.argwhere(frontier)
        if pts.size == 0:
            return None

        # posição do robô (estática, pega de initial_pose)
        rx = self.initial_pose.pose.position.x
        ry = self.initial_pose.pose.position.y

        # 2.3) converte cada célula de frontier em coordenada (x, y) e filtra distância > 0.5 m
        candidates = []
        for i, j in pts:
            x = ox + (j + 0.5) * res
            y = oy + (i + 0.5) * res
            d = math.hypot(x - rx, y - ry)
            if d > 0.5:
                candidates.append((x, y, d))

        if not candidates:
            return None

        # 2.4) filtra metas já visitadas (distância mínima de 0.5 m)
        min_repeat = 0.5
        filtered = [
            (x, y, d) for x, y, d in candidates
            if all(math.hypot(x - vx, y - vy) > min_repeat for vx, vy in self.visited_goals)
        ]
        if not filtered:
            return None

        # 2.5) escolhe a célula de frontier MAIS DISTANTE do robô
        x, y, _ = max(filtered, key=lambda c: c[2])

        # orientação: copiamos a orientação atual do robô (ignorar rotação)
        if self.current_pose is not None:
            quat = self.current_pose.pose.orientation
        else:
            quat = PoseStamped().pose.orientation
            quat.x = 0.0; quat.y = 0.0; quat.z = 0.0; quat.w = 1.0

        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp    = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation = quat

        return p

    def mission_loop(self):
        if self.phase != 'explore':
            return

        # se ainda não enviamos um objetivo
        if self.explore_goal_future is None and self.explore_goal_handle is None:
            if not self.waypoint_client.wait_for_server(timeout_sec=0.1):
                return

            goal_pose = self.compute_next_frontier_goal()
            if goal_pose is None:
                self.get_logger().warn('⚠️  Nenhuma fronteira válida ou bloco 2×2 encontrado.')
                return

            self.last_goal = goal_pose
            gx = goal_pose.pose.position.x
            gy = goal_pose.pose.position.y
            self.get_logger().info(f'➡️  Explorando ponto em ({gx:.2f}, {gy:.2f})')

            goal = FollowWaypoints.Goal()
            goal.poses = [goal_pose]
            self.explore_goal_future = self.waypoint_client.send_goal_async(goal)
            self.explore_goal_future.add_done_callback(self.explore_response_cb)

        # se virmos a bandeira enquanto exploramos, cancelamos exploração
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
            self.get_logger().error('Exploração NÃO aceita pelo servidor!')
        else:
            self.get_logger().info('Exploração aceita pelo servidor')
            self.explore_goal_handle = handle
            handle.get_result_async().add_done_callback(self.on_explore_complete)

    def on_explore_complete(self, future):
        self.get_logger().info('✅ Meta de exploração concluída, liberando handle.')
        if self.last_goal:
            x = self.last_goal.pose.position.x
            y = self.last_goal.pose.position.y
            self.visited_goals.append((x, y))
            self.last_goal = None
        self.explore_goal_handle = None

    def _on_explore_cancel(self, future):
        self.get_logger().info('Exploração cancelada pelo usuário (flag_servo).')
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
        pose.pose            = self.initial_pose.pose  # retorna à pose inicial (orientação idêntica)

        self.get_logger().info('↩️  Regressando à base via FollowWaypoints')
        goal = FollowWaypoints.Goal()
        goal.poses = [pose]
        self.return_goal_future = self.waypoint_client.send_goal_async(goal)
        self.return_goal_future.add_done_callback(self.return_response_cb)

    def return_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Retorno NÃO aceito pelo servidor!')
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
