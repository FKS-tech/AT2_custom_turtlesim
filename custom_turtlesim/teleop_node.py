#!/usr/bin/env python3


import sys
import select
import termios
import tty
import atexit
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_turtlesim.srv import Spawn, SetBgColor, ChangeSprite, Kill
import random
import math
import time
from ament_index_python.packages import get_package_share_directory


key_bindings = {
    # movimento
    'w': {'move': (2.0, 0.0)},
    's': {'move': (-2.0, 0.0)},
    'a': {'rotate': -120.0},  
    'd': {'rotate': 120.0}, 

    # cores fixas
    'r': {'color': (255, 0, 0)},
    'g': {'color': (0, 255, 0)},
    'b': {'color': (0, 0, 255)},

    # sprite(aleatório)
    't': {'action': 'change_sprite'},

    # Sprites específicos (opcional)
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
}


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_custom_turtle')
        self.declare_parameter('turtle_name', f'turtle_{random.randint(1000, 9999)}')
        self.turtle_name = self.get_parameter('turtle_name').value

        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
            self.is_tty = True
        else:
            self.get_logger().warn("Não está rodando em modo TTY. Teclado não funcionará.")
            self.is_tty = False
            return

        self.settings = termios.tcgetattr(sys.stdin)
        atexit.register(lambda: termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings))

        self.cli_spawn = self.create_client(Spawn, 'spawn_turtle')
        self.cli_bg = self.create_client(SetBgColor, 'set_background_color')
        self.cli_kill = self.create_client(Kill, 'kill_turtle')

        while not self.cli_kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill_turtle service...')

        self.cli_sprite = self.create_client(ChangeSprite, 'change_turtle_sprite')

        while not self.cli_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn_turtle service...')

        self.declare_parameter('x', 5.0)
        self.declare_parameter('y', 5.0)
        self.declare_parameter('theta', 0.0)


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


        self.get_logger().info(f"Spawned my turtle: {self.turtle_name} (ID={self.turtle_id})")
        
        self.publisher = self.create_publisher(Twist, f'/{self.turtle_name}/cmd_vel', 10)

        print("""
--- Teleop Custom Turtle ---
Use:
  w/s - mover frente/trás
  a/d - girar esquerda/direita
  r/g/b - mudar cor de fundo
  t - trocar sprite
CTRL+C - sair
""")

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


        self.active_timer = self.create_timer(duration, lambda: self.stop_turtle())

    def start_rotate(self, angle_deg, duration):

        self.cancel_active_action()
        self.action_in_progress = True

        angle_rad = math.radians(angle_deg)
        angular_speed = angle_rad / duration

        twist = Twist()
        twist.angular.z = angular_speed
        self.publisher.publish(twist)
        self.get_logger().info(f"Started rotate: angular={angular_speed}, duration={duration}")
        self.active_timer = self.create_timer(duration, lambda: self.stop_turtle())

    def stop_turtle(self):
        self.get_logger().info("Turtle stopped by timer.")
        self.publisher.publish(Twist())
        if self.active_timer:
            self.active_timer.cancel()
            self.active_timer = None
        self.action_in_progress = False


    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while True:

                rclpy.spin_once(self, timeout_sec=0.01)  

                
                key = self.get_key()
                
                if key == '\x03':
                    break

                if key in key_bindings:
                    action = key_bindings[key]

                    if 'move' in action:
                        linear, _ = action['move']
                        self.start_move(linear, 1.0)

                    elif 'rotate' in action:
                        angle_deg = action['rotate']
                        self.start_rotate(angle_deg, 1.0)


                    elif 'color' in action:
                        r, g, b = action['color']
                        self.set_background(r, g, b)
                        self.get_logger().info(f"Key pressed: {key}")

                    elif 'sprite' in action:
                        new_sprite = action['sprite']
                        self.change_sprite_to(new_sprite)

                    elif 'action' in action and action['action'] == 'change_sprite':
                        self.change_sprite()

        except Exception as e:
            self.get_logger().info(f'Error: {e}')

        finally:

            self.cancel_active_action()
            self.publisher.publish(Twist())

            
            req = Kill.Request()
            req.name = self.turtle_name
            future = self.cli_kill.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f"Killed turtle: {self.turtle_name}")

    def set_background(self, r, g, b):
        req = SetBgColor.Request()
        req.red = r
        req.green = g
        req.blue = b
        future = self.cli_bg.call_async(req)

        self.get_logger().info(f'Sent  background color: RGB({r}, {g}, {b})')

    def change_sprite(self):
        sprites = ['ardent', 'bouncy', 'crystal', 'dashing', 'eloquent',
                   'foxy', 'galactic', 'humble', 'iron', 'jazzy', 'rolling']
        new_sprite = random.choice(sprites)
        self.change_sprite_to(new_sprite)

    def change_sprite_to(self, sprite_name):
        req = ChangeSprite.Request()
        req.turtle_id = self.turtle_id
        req.sprite_name = sprite_name
        future = self.cli_sprite.call_async(req)
        self.get_logger().info(f'Requested sprite change to: "{sprite_name}"')

def main(args = None):
    rclpy.init(args = args)
    node = TeleopNode()
    if hasattr(node, 'is_tty') and node.is_tty:
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








