#!/usr/bin/env python3
"""
flag_servo.py  –  Controle visual + LiDAR para aproximar do alvo colorido.

Executa somente quando /flag_servo_enable == True.
Quando chega à bandeira publica Bool(True) em /flag_servo_arrived.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType



# --- ajuste aqui se mudar a cor da bandeira ---
# HSV_MIN = (86, 0, 6)      # azul–esverdeado no seu range
# HSV_MAX = (94, 255, 255)
HSV_MIN = (110, 50, 50)  # Faixa mínima para capturar o azul
HSV_MAX = (125, 255, 255)
# ----------------------------------------------


class FlagServo(Node):
    def __init__(self):
        super().__init__('flag_servo')

        # publishers / subscribers -------------------------------------------------
        self.cmd_vel_pub     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arrived_pub     = self.create_publisher(Bool, '/flag_servo_arrived', 1)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )


        self.create_subscription(
            Bool, '/flag_servo_enable', self.enable_cb, 1)

        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)

        self.create_subscription(
            Image, '/robot_cam/colored_map', self.image_cb, qos_profile_sensor_data)

        # timer 10 Hz --------------------------------------------------------------
        self.timer = self.create_timer(0.1, self.control_loop)

        # estado interno -----------------------------------------------------------
        self.bridge = CvBridge()
        self.enabled = False
        self.already_stopped = False

        self.dist_front = float('inf')
        self.flag_x     = None          # pixel x do alvo
        self.img_w      = None
        self.arrived    = False
        self.area_bandeira = 0

        # para alternar lado de desvio
        self.turn_left  = True
        self.last_turn_stamp = self.get_clock().now()

    def disable_obstacle_layer(self):
        client = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('❌ Serviço de parâmetros não disponível!')
            return

        req = SetParameters.Request()
        param = ParameterMsg()
        param.name = 'voxel_layer.enabled'
        param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
        req.parameters = [param]

        future = client.call_async(req)
        # self.get_logger().info('🚧 Desativando obstacle_layer...')
        # rclpy.spin_until_future_complete(self, future)

        # if future.result() is not None:
        #     self.get_logger().info('✅ obstacle_layer desativada com sucesso!')
        # else:
        #     self.get_logger().error('❌ Falha ao desativar obstacle_layer.')
        def callback(future):
            try:
                result = future.result()
                self.get_logger().info("✅ obstacle_layer desativada com sucesso!")
            except Exception as e:
                self.get_logger().error(f"❌ Erro ao desativar obstacle_layer: {e}")

        future.add_done_callback(callback)
    # --------------------------------------------------------------------------- #
    #                                Callbacks
    # --------------------------------------------------------------------------- #
    def enable_cb(self, msg: Bool):
        self.enabled = msg.data
        if not self.enabled:
            self.get_logger().info('[FlagServo] Desligado')
            self.stop_robot()
        else:
            self.get_logger().info('[FlagServo] Habilitado')

    def scan_cb(self, msg: LaserScan):
        if len(msg.ranges) == 0:
            return

        idx_left = list(range(350, 360)) #mudei, pegava 90 graus da frente
        idx_right = list(range(0, 11))

        distances = [msg.ranges[i] for i in idx_left + idx_right
                     if not np.isnan(msg.ranges[i])]
        self.dist_front = min(distances) if distances else float('inf')

        # escolhe lado mais livre para desvio
        left_vals  = [msg.ranges[i] for i in idx_left  if not np.isnan(msg.ranges[i])]
        right_vals = [msg.ranges[i] for i in idx_right if not np.isnan(msg.ranges[i])]
        mean_left  = np.mean(left_vals)  if left_vals  else 0.0
        mean_right = np.mean(right_vals) if right_vals else 0.0

        if self.dist_front < 1.0:  # obstáculo perto
            now = self.get_clock().now()
            if (now - self.last_turn_stamp).nanoseconds / 1e9 > 2.0:
                self.turn_left = mean_left < mean_right
                self.last_turn_stamp = now

    def image_cb(self, msg: Image):
        if not self.enabled:
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = cv_img.shape[:2]
        self.img_w = w

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_MIN, HSV_MAX)

        # Ignora os 50% de cima:
        cutoff = int(0.5 * h)
        mask[:cutoff, :] = 0   # tudo acima de cutoff vira 0

        # continua como antes
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            maior = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(maior)
            if M["m00"] > 0:
                self.flag_x = int(M["m10"] / M["m00"])
                return
            self.area_bandeira=area
        # nada encontrado
        self.flag_x = None


    # --------------------------------------------------------------------------- #
    #                                 Main loop
    # --------------------------------------------------------------------------- #
    def move_gripper(self, extension_pos: float, gripper_pos_left: float, gripper_pos_right: float):
        msg = Float64MultiArray()
        msg.data = [extension_pos, gripper_pos_left, gripper_pos_right]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'📤 Enviando gripper_extension={extension_pos}, right_gripper_joint={gripper_pos_right}, left_gripper_joint={gripper_pos_left}')

    
    def control_loop(self):
        """Loop de controle: aproximação visual + LiDAR, garantindo alinhamento antes de finalizar."""
        if not self.enabled:
            if not self.already_stopped:
                self.stop_robot()
                self.already_stopped = True
            return
        self.already_stopped = False

        # se já sinalizamos chegada, apenas para
        if self.arrived:
            self.stop_robot()
            return

        twist = Twist()

        # quando a bandeira está detectada na imagem
        if self.flag_x is not None and self.img_w:
            # cálculo do erro de centralização
            center = self.img_w // 2
            error  = self.flag_x - center
            Kp     = 0.003

            # controle proporcional para alinhar
            twist.angular.z = -Kp * error

            # parâmetros
            ALIGN_TOL = 10     # pixels de tolerância para centralização
            DIST_STOP = 0.30  # m para considerar "perto"
            DIST_ABRIR_GARRA = 0.69
            ALIGN_TOL_GARRA = 25

            if self.dist_front < DIST_ABRIR_GARRA: #and self.area_bandeira > 900:
                # está suficientemente perto → verificar alinhamento
                if abs(error) < ALIGN_TOL_GARRA:
                    # alinhado e perto → missão concluída
                    #self.stop_robot()
                    self.get_logger().info('Abrindo garra!')
                    # Estende o braço e abre a garra
                    self.move_gripper(0.6, -0.06, 0.06)
                    #time.sleep(2.0)


            if self.dist_front < DIST_STOP: # and self.area_bandeira > 900:
                # está suficientemente perto → verificar alinhamento
                if abs(error) < ALIGN_TOL:
                    # alinhado e perto → missão concluída
                    
                    self.get_logger().info('🏁 Bandeira alcançada e alinhada!')
                    self.stop_robot()
                    # Estende o braço e abre a garra
                    # self.move_gripper(0.0, -0.1)
                    # time.sleep(2.0)
                    # Fecha a garra
                    self.get_logger().info('Fechando garra!')
                    self.move_gripper(-1.3, 0.0, 0.0)
                    time.sleep(6.0)
                    
                    # Após pegar a bandeira
                    self.disable_obstacle_layer()

                    self.arrived = True
                    self.arrived_pub.publish(Bool(data=True))
                    return
                else:
                    # precisa ajustar alinhamento antes de parar
                    twist.linear.x = 0.0
            else:
                # ainda longe → avança com velocidade reduzida se estiver intermediário
                twist.linear.x = 0.075 if self.dist_front < 1.0 else 0.15

        else:
            # se não vê a bandeira, gira para buscá-la
            self.get_logger().info('🔍 Bandeira não vista, girando...')
            twist.linear.x  = 0.0
            twist.angular.z = 0.2 

        # log e publicação do comando
        self.get_logger().info(
            f'Dist:{self.dist_front:.2f}  flag_x:{self.flag_x}  '
            f'lin:{twist.linear.x:.2f}  ang:{twist.angular.z:.3f}'
        )
        self.cmd_vel_pub.publish(twist)


    # --------------------------------------------------------------------------- #
    def stop_robot(self):
        """Envia Twist zero uma única vez."""
        self.cmd_vel_pub.publish(Twist())

# --------------------------------------------------------------------------- #
def main():
    rclpy.init()
    node = FlagServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
