#!/usr/bin/env python3

import sys
import select
import termios
import tty
import atexit
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_turtlesim.srv import Spawn, SetBgColor, ChangeSprite, Kill, GetTurtlePosition
import random
import math

# --- Mapeamento de teclas para ações ---
KEY_BINDINGS = {
    # Movimento
    'w': {'move': (2.0, 0.0)},
    's': {'move': (-2.0, 0.0)},
    'a': {'rotate': -120.0},
    'd': {'rotate': 120.0},

    # Cores de fundo
    'r': {'color': (255, 0, 0)},
    'g': {'color': (0, 255, 0)},
    'b': {'color': (0, 0, 255)},

    # Sprite aleatório
    't': {'action': 'change_sprite'},

    # Sprites específicos
    '1': {'sprite': 'ardent'},
    '2': {'sprite': 'bouncy'},
    '3': {'sprite': 'crystal'},
    '4': {'sprite': 'dashing'},
    '5': {'sprite': 'eloquent'},
    '6': {'sprite': 'foxy'},
    '7': {'sprite': 'galactic'},
    '8': {'sprite': 'humble'},
    '9': {'sprite': 'iron'},
    '0': {'sprite': 'jazzy'},
    '-': {'sprite': 'rolling'},

    #polygons
    'p' : {'polygon': 'regular_polygon'},
    '+' : {'polygon': 'increase_sides'},
    '_' : {'polygon': 'decrease_sides'},
    'i' : {'polygon': 'increase_size'},
    'k' : {'polygon': 'decrease_size'},
}

# --- Classe utilitária para leitura de teclado em terminal ---
class KeyboardInput:
    def __init__(self):
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
            self.is_tty = True
            atexit.register(self.restore_terminal)
        else:
            self.is_tty = False

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def restore_terminal(self):
        if self.is_tty:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

# --- Nó principal de teleoperação ---
class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_custom_turtle')

        # --- Parâmetros e inicialização ---
        self.declare_parameter('turtle_name', f'turtle_{random.randint(1000, 9999)}')
        self.turtle_name = self.get_parameter('turtle_name').value

        self.keyboard = KeyboardInput()
        if not self.keyboard.is_tty:
            self.get_logger().warning("Não está rodando em modo TTY. Teclado não funcionará.")
            return

        # --- Serviços ROS 2 ---
        self.cli_spawn = self.create_client(Spawn, 'spawn_turtle')
        self.cli_bg = self.create_client(SetBgColor, 'set_background_color')
        self.cli_kill = self.create_client(Kill, 'kill_turtle')
        self.cli_sprite = self.create_client(ChangeSprite, 'change_turtle_sprite')
        self.cli_get_position = self.create_client(GetTurtlePosition, 'get_turtle_position')

        # Aguarda serviços necessários
        for cli, name in [
            (self.cli_spawn, 'spawn_turtle'),
            (self.cli_get_position, 'get_turtle_position'),
            (self.cli_bg, 'set_background_color'),
            (self.cli_sprite, 'change_turtle_sprite'),
            (self.cli_kill, 'kill_turtle')
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Serviço "{name}" não disponível, esperando...')

        # --- Parâmetros de spawn ---
        self.declare_parameter('x', 5.0)
        self.declare_parameter('y', 5.0)
        self.declare_parameter('theta', 0.0)

        # --- Spawna a tartaruga controlada ---
        req = Spawn.Request()
        req.name = self.turtle_name
        req.x = self.get_parameter('x').value
        req.y = self.get_parameter('y').value
        req.theta = self.get_parameter('theta').value
        future = self.cli_spawn.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.turtle_id = future.result().turtle_id
        else:
            self.get_logger().error("Failed to spawn turtle!")
            rclpy.shutdown()
            return

        self.active_timer = None
        self.action_in_progress = False

        self.polygon_sides = 3
        self.side_length = 1.0
        self.polygon_in_progress = False


        self.get_logger().info(f"Spawned my turtle: {self.turtle_name} (ID={self.turtle_id})")
        self.publisher = self.create_publisher(Twist, f'/{self.turtle_name}/cmd_vel', 10)

        print("""
--- Teleop Custom Turtle ---
Use:
  w/s - mover frente/trás
  a/d - girar esquerda/direita
  p - desenhar polígono
  + - aumentar lados do polígono
  _ - diminuir lados do polígono
  i - aumentar tamanho do polígono
  k - diminuir tamanho do polígono
  r/g/b - mudar cor de fundo
  t - trocar sprite
  1-0/- - sprites específicos
CTRL+C - sair
""")

    # --- Métodos de controle de movimento ---
    def cancel_active_action(self):
        if self.active_timer:
            self.get_logger().info("Cancelling active action timer.")
            self.active_timer.cancel()
            self.active_timer = None
        self.action_in_progress = False
        self.publisher.publish(Twist())

    def start_move(self, linear, duration):
        self.cancel_active_action()
        self.action_in_progress = True
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info(f"Started move: linear={linear}, duration={duration}")
        self.active_timer = self.create_timer(duration, self.stop_turtle)

    def start_rotate(self, angle_deg, duration):
        self.cancel_active_action()
        self.action_in_progress = True
        angle_rad = math.radians(angle_deg)
        angular_speed = angle_rad / duration
        twist = Twist()
        twist.angular.z = angular_speed
        self.publisher.publish(twist)
        self.get_logger().info(f"Started rotate: angular={angular_speed}, duration={duration}")
        self.active_timer = self.create_timer(duration, self.stop_turtle)

    def stop_turtle(self):
        self.get_logger().info("Turtle stopped by timer.")
        self.publisher.publish(Twist())
        if self.active_timer:
            self.active_timer.cancel()
            self.active_timer = None
        self.action_in_progress = False

    def send_polygon_request(self):
        req = GetTurtlePosition.Request()
        req.name = self.turtle_name
        req.sides = self.polygon_sides
        req.side_length = self.side_length
        future = self.cli_get_position.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Failed to get turtle position!")
            return None
        
    def draw_polygon(self):
        if self.action_in_progress:
            self.get_logger().info("Action in progress, cannot draw polygon now.")
            return
        response = self.send_polygon_request()
        if response is None or not response.success:
            self.get_logger().error("Failed to get valid polygon parameters.")
            self.polygon_in_progress = False
            return

        turn_angle = response.turn_angle_deg
        self.get_logger().info(f"Drawing polygon with {self.polygon_sides} sides of length {self.side_length}")

        if response.dmin < response.radius:
            self.get_logger().warning("Not enough space to draw the polygon without hitting walls. Repositioning...")

            if hasattr(response, 'x') and hasattr(response, 'y') and hasattr(response, 'theta'):
                new_x = response.x
                new_y = response.y
                new_theta = response.theta
            else:
                new_x = 5.0
                new_y = 5.0
                new_theta = 0.0

            world = 10.0
            r = response.radius

            tx = max(r, min(new_x, world - r))
            ty = max(r, min(new_y, world - r))
            dist = math.hypot(tx - new_x, ty - new_y)
            if dist > 0.01:
                bearing = math.degrees(math.atan2(ty - new_y, tx - new_x))

                cur_deg = math.degrees(new_theta)
                rel = ((bearing - cur_deg + 180) % 360) - 180

                rot_duration = max(0.1, abs(rel) / 120.0)
                self.start_rotate(rel, rot_duration)
                while self.active_timer is not None and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)

                speed = 2.0
                self.start_move(speed, dist / speed)
                while self.active_timer is not None and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)

                response = self.send_polygon_request()
                if response is None or not response.success:
                    self.get_logger().error("Failed to get valid polygon parameters after repositioning.")
                    return

        def wait_for_active_timer(node):
            while node.active_timer is not None and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)

        for side in range(self.polygon_sides):
            self.start_move(2.0, self.side_length / 2.0)
            wait_for_active_timer(self)
            self.start_rotate(turn_angle, 2.0)
            wait_for_active_timer(self)

        self.polygon_in_progress = False


    def set_background(self, r, g, b):
        req = SetBgColor.Request()
        req.red = r
        req.green = g
        req.blue = b
        self.cli_bg.call_async(req)
        self.get_logger().info(f'Sent background color: RGB({r}, {g}, {b})')

    def change_sprite(self):
        sprites = [
            'ardent', 'bouncy', 'crystal', 'dashing', 'eloquent',
            'foxy', 'galactic', 'humble', 'iron', 'jazzy', 'rolling'
        ]
        new_sprite = random.choice(sprites)
        self.change_sprite_to(new_sprite)

    def change_sprite_to(self, sprite_name):
        req = ChangeSprite.Request()
        req.turtle_id = self.turtle_id
        req.sprite_name = sprite_name
        self.cli_sprite.call_async(req)
        self.get_logger().info(f'Requested sprite change to: "{sprite_name}"')

    # --- Loop principal de teleoperação ---
    def run(self):
        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.01)
                key = self.keyboard.get_key()
                if key == '\x03':  # CTRL+C
                    break
                if key in KEY_BINDINGS:
                    action = KEY_BINDINGS[key]
                    # primeiro: ações relacionadas a polígonos (podem alterar parâmetros ou iniciar desenho)
                    if 'polygon' in action:
                        kind = action['polygon']
                        if kind == 'regular_polygon':
                            if not self.action_in_progress and not self.polygon_in_progress:
                                self.polygon_in_progress = True
                                self.draw_polygon()
                        elif kind == 'increase_sides':
                            if self.polygon_sides < 12:
                                self.polygon_sides += 1
                                self.get_logger().info(f"Increased polygon sides to {self.polygon_sides}")
                            else:
                                self.get_logger().info("Maximum sides reached (12).")
                        elif kind == 'decrease_sides':
                            if self.polygon_sides > 3:
                                self.polygon_sides -= 1
                                self.get_logger().info(f"Decreased polygon sides to {self.polygon_sides}")
                            else:
                                self.get_logger().info("Minimum sides reached (3).")
                        elif kind == 'increase_size':
                            if self.side_length < 5.0:
                                self.side_length += 0.5
                                self.get_logger().info(f"Increased polygon side length to {self.side_length}")
                            else:
                                self.get_logger().info("Maximum side length reached (5.0).")
                        elif kind == 'decrease_size':
                            if self.side_length > 0.5:
                                self.side_length -= 0.5
                                self.get_logger().info(f"Decreased polygon side length to {self.side_length}")
                            else:
                                self.get_logger().info("Minimum side length reached (0.5).")
                        continue  # já tratou a tecla

                    # então: ações de movimento / sprite / cor — só quando não estiver desenhando polígono
                    if not self.polygon_in_progress:
                        if 'move' in action:
                            linear, _ = action['move']
                            self.start_move(linear, 1.0)
                        elif 'rotate' in action:
                            angle_deg = action['rotate']
                            self.start_rotate(angle_deg, 1.0)
                        elif 'color' in action:
                            r, g, b = action['color']
                            self.set_background(r, g, b)
                        elif 'sprite' in action:
                            new_sprite = action['sprite']
                            self.change_sprite_to(new_sprite)
                        elif 'action' in action and action['action'] == 'change_sprite':
                            self.change_sprite()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.cancel_active_action()
            self.publisher.publish(Twist())
            # Mata a tartaruga ao sair
            req = Kill.Request()
            req.name = self.turtle_name
            future = self.cli_kill.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f"Killed turtle: {self.turtle_name}")

# --- Função principal ---
def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    if hasattr(node, 'keyboard') and node.keyboard.is_tty:
        try:
            node.run()
        except KeyboardInterrupt:
            print('Nó interrompido pelo usuario')
        finally:
            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()
    else:
        print("Erro: Não está em modo TTY. Não é possível ler teclado.")

if __name__ == '__main__':
    main()