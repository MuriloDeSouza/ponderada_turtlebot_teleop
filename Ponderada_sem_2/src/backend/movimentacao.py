#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import sys, select, tty, termios

# Definição dos valores máximos de velocidade linear e angular do TurtleBot3 Burger
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

# Mensagem de instruções para o usuário
msg = """
Control Your TurtleBot3 Burger!
---------------------------
Moving around:
    ↑
←       →
    ↓

↑ : increase linear velocity (Burger : ~ 0.22)
↓ : decrease linear velocity (Burger : ~ 0.22)
← : increase angular velocity (Burger : ~ 2.84)
→ : decrease angular velocity (Burger : ~ 2.84)

tecla 's' : parada de Emergência
tecla de espaço : encerrar o nó

CTRL-C to quit
"""

# Mensagem de erro para falha de comunicação
e = """
Communications Failed
"""

# Função para capturar a tecla pressionada pelo usuário
def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)  # Lê os próximos dois caracteres para setas direcionais
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Função para formatar a string de velocidades
def vels(target_linear_vel, target_angular_vel):
    return f"currently:\tlinear vel {target_linear_vel}\t angular vel {target_angular_vel}"

# Função para criar um perfil simples de mudança de velocidade
def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

# Função para restringir o valor dentro de um intervalo
def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    return input

# Função para verificar e ajustar a velocidade linear ao limite
def checkLinearLimitVelocity(vel):
    return constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

# Função para verificar e ajustar a velocidade angular ao limite
def checkAngularLimitVelocity(vel):
    return constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

# Classe que representa o nó de teleoperação do TurtleBot3
class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 30)
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info(msg)
        self.timer = self.create_timer(0.1, self.update)

        # Criação do serviço para parar o robô
        self.stop_service = self.create_service(Empty, 'stop_robot', self.stop_robot_callback)

    # Função de callback para o serviço de parar o robô
    def stop_robot_callback(self, request, response):
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Robot stopped via service call")
        rclpy.shutdown()  # Encerra o processo de operação
        return response

    # Função chamada periodicamente para atualizar a velocidade do robô
    def update(self):
        key = getKey(self.settings)
        if key == '\x1b[A':  # Setinha para cima
            self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
            self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == '\x1b[B':  # Setinha para baixo
            self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel - LIN_VEL_STEP_SIZE)
            self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == '\x1b[D':  # Setinha para esquerda
            self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE)
            self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == '\x1b[C':  # Setinha para direita
            self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE)
            self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == 's':  # Parada de emergência
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            self.control_linear_vel = 0.0
            self.control_angular_vel = 0.0
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.get_logger().info("Robot stopped via key press 's'")
        elif key == ' ':  # Encerrar o nó
            rclpy.shutdown()  # Encerra o processo de operação
            self.get_logger().info("Node shutdown via key press 'space'")
        else:
            if key == '\x03':  # Ctrl-C
                rclpy.shutdown()
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                return

        twist = Twist()
        self.control_linear_vel = makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
        twist.linear.x = self.control_linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        self.control_angular_vel = makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.control_angular_vel

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)  # Inicializa a comunicação ROS
    node = TeleopNode()  # Cria uma instância do nó de teleoperação
    try:
        rclpy.spin(node)  # Mantém o nó em execução
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Destroi o nó
        rclpy.shutdown()  # Encerra a comunicação ROS
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)  # Restaura as configurações do terminal

if __name__ == '__main__':
    main()  # Executa a função principal se o script for executado diretamente
