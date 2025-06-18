#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray

# --- ajuste aqui se mudar a cor da bandeira ---
# HSV_MIN = (86, 0, 6)      # azul–esverdeado no seu range
# HSV_MAX = (94, 255, 255)
# HSV_MIN = (0, 200, 200)  # Ajuste para o tom de azul mais realista
# HSV_MAX = (16, 255, 255)  # Faixa maior para saturação e valor
HSV_MIN = (110, 50, 50)  # Faixa mínima para capturar o azul
HSV_MAX = (125, 255, 255)  # Faixa máxima para o azul mais intenso

# ----------------------------------------------


class ControleRobo(Node):
    def __init__(self):
        super().__init__('controle_robo_novo')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )

        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)

        self.timer = self.create_timer(0.1, self.move_robot)

        self.bridge = CvBridge()
        self.obstaculo_a_frente = False
        self.girar_para_esquerda = True
        self.bandeira_x = None
        self.largura_img = None
        self.distancia_frontal = float('inf')
        self.chegou_na_bandeira = False
        self.area_bandeira = 0
        self.alinhando_bandeira = False


        # Para evitar troca rápida de direção
        self.direcao_obstaculo = None
        self.timestamp_ultima_direcao = self.get_clock().now()

        self.tempo_ultima_bandeira = self.get_clock().now()
        self.bandeira_ja_foi_vista = False


    def scan_callback(self, msg: LaserScan):
        # if (
        #     self.bandeira_x is not None and
        #     self.area_bandeira > 1300
        # ):
        #     self.get_logger().info("✅ Ignorando LiDAR: bandeira visível e próxima")
        #     self.obstaculo_a_frente = False  # Garante que nenhum desvio seja acionado
        #     return

        # if self.alinhando_bandeira:
        #     return
        
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        indices_esq_frente = list(range(325, 360))
        indices_dir_frente = list(range(0, 36))

        dist_esq = [msg.ranges[i] for i in indices_esq_frente if not np.isnan(msg.ranges[i])] #coloca as distâncias à esquerda em um vetor
        dist_dir = [msg.ranges[i] for i in indices_dir_frente if not np.isnan(msg.ranges[i])] #coloca as distâncias à direita em um vetor

        min_esq = min(dist_esq) if dist_esq else float('inf') # minima distância à esquerda
        min_dir = min(dist_dir) if dist_dir else float('inf') # mínima distância à direita
        media_esq = np.mean(dist_esq) if dist_esq else 0
        media_dir = np.mean(dist_dir) if dist_dir else 0

        if (
            self.bandeira_x is not None and
            self.area_bandeira > 1300
        ):
            self.obstaculo_a_frente = False
            return  # mas não ativa desvio

        todos_frontal = dist_esq + dist_dir #distancia total no leque determinado
        self.distancia_frontal = min(todos_frontal) if todos_frontal else float('inf') #minima distancia em um leque de 180

        if self.distancia_frontal < 0.8: #se algum ponto a 180 esta a menos de 1m
            self.obstaculo_a_frente = True

            agora = self.get_clock().now()
            tempo_desde_ultima_troca = (agora - self.timestamp_ultima_direcao).nanoseconds / 1e9

            if (
                self.direcao_obstaculo is None or 
                tempo_desde_ultima_troca > 2.0 or
                abs(media_esq - media_dir) > 0.2
            ):
                self.girar_para_esquerda = media_esq < media_dir
                self.direcao_obstaculo = 'ESQ' if self.girar_para_esquerda else 'DIR'
                self.timestamp_ultima_direcao = agora

            direcao = "ESQUERDA" if self.girar_para_esquerda else "DIREITA"
            self.get_logger().info(
                f"[Desvio Frontal] Objeto a {self.distancia_frontal:.2f} m → Virando para {direcao}"
            )
        else:
            self.obstaculo_a_frente = False
            self.direcao_obstaculo = None

    def camera_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.largura_img = cv_image.shape[1]

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_MIN, HSV_MAX)

        # Ignora os 30% de cima:
        cutoff = int(0.3 * cv_image.shape[0])
        mask[:cutoff, :] = 0   # tudo acima de cutoff vira 0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            maior = max(contours, key=cv2.contourArea)
            M = cv2.moments(maior)
            area = cv2.contourArea(maior)

            if M["m00"] > 0:
                self.bandeira_x = int(M["m10"] / M["m00"])
            else:
                self.bandeira_x = None

            self.area_bandeira = area
        else:
            self.bandeira_x = None
            self.area_bandeira = 0

        if self.bandeira_x is not None:
            self.bandeira_ja_foi_vista = True
            self.tempo_ultima_bandeira = self.get_clock().now()
        else:
            self.bandeira_x = None


    def imu_callback(self, msg: Imu):
        pass

    def odom_callback(self, msg: Odometry):
        pass

    def move_gripper(self, extension_pos: float, gripper_pos: float):
        msg = Float64MultiArray()
        msg.data = [extension_pos, gripper_pos]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'📤 Enviando gripper_extension={extension_pos}, right_gripper_joint={gripper_pos}')

    def move_robot(self):
        twist = Twist()

        #✅ Ignorar obstáculo se a bandeira estiver centralizada e próxima
        if (
            self.bandeira_x is not None and
            self.largura_img and
            self.area_bandeira > 1500 and
            self.distancia_frontal < 1.3  # Algo está à frente, provavelmente a própria bandeira
        ):
            centro = self.largura_img // 2
            erro = self.bandeira_x - centro

            if abs(erro) > 10:  # ainda está desalinhado
                self.alinhando_bandeira = True
                self.obstaculo_a_frente = False
                twist.linear.x = 0.0
                twist.angular.z = -0.001 * erro  # gira mais rápido
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("🔄 Girando para centralizar a bandeira próxima (sem andar)")
                return
            else:
                self.alinhando_bandeira = False

        if (not self.chegou_na_bandeira
            and self.bandeira_x is not None
            and self.largura_img
            and self.area_bandeira > 1750  # 👈 depende da sua câmera e distância
            #and self.distancia_frontal < 0.7
        ):
            centro = self.largura_img // 2
            erro = abs(self.bandeira_x - centro)

            if erro < 10:
                self.chegou_na_bandeira = True
                self.get_logger().info(f"🏁 Missão cumprida! Bandeira centralizada e próxima (área={self.area_bandeira:.0f})")

        if self.chegou_na_bandeira:
            twist = Twist()
            self.move_gripper(0.0, -0.1)
            time.sleep(2.0)
            # Fecha a garra
            self.move_gripper(-0.8, 0.0)
            time.sleep(6.0)
            self.cmd_vel_pub.publish(twist)
            return

        if self.bandeira_x is not None and self.largura_img:
            centro = self.largura_img // 2
            erro = self.bandeira_x - centro
            if self.area_bandeira > 1200 and self.distancia_frontal < 2.0:
                self.obstaculo_a_frente = False  # Ignorar obstáculos
                twist.linear.x = 0.1
                twist.angular.z = -0.0005 * erro
                self.get_logger().info("Movendo em direção à bandeira 🎯")
            elif not self.obstaculo_a_frente:
                twist.linear.x = 0.1
                twist.angular.z = -0.0005* erro
                self.get_logger().info("Movendo em direção à bandeira 🎯")
            else:
                if self.distancia_frontal < 0.2:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.2 if self.girar_para_esquerda else -0.2
                    self.get_logger().info("⚠️ Objeto MUITO próximo! Girando parado")
                else:
                    twist.linear.x = 0.05
                    twist.angular.z = 0.1 if self.girar_para_esquerda else -0.1
                    self.get_logger().info("Desviando de obstáculo durante perseguição 🚧")
        else:
            if not self.obstaculo_a_frente:
                twist.linear.x = 0.1
                self.get_logger().info("Movendo reto 🚶")
            else:
                if self.distancia_frontal < 0.2:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.2 if self.girar_para_esquerda else -0.2
                    self.get_logger().info("⚠️ Objeto MUITO próximo! Girando parado (sem bandeira)")
                else:
                    twist.linear.x = 0.1
                    twist.angular.z = 0.2 if self.girar_para_esquerda else -0.2
                    self.get_logger().info("Desviando de obstáculo sem bandeira 👀")

        self.cmd_vel_pub.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
