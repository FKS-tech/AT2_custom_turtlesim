#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_turtlesim.srv import Spawn, SetBgColor, ChangeSprite, Kill
import random
import time
import math
import sys
import pygame
from ament_index_python.packages import get_package_share_directory  
import os


WINDOW_WIDTH = 500
WINDOW_HEIGHT = 500
WORLD_SIZE = 10.0


class Turtle():
    def __init__(self, identificacao, name, x, y, theta):
        self.id = identificacao
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = 0.0
        self.vtheta = 0.0
        self.sprite = 'jazzy'
        self.trail = []
        self.max_trail_lenght = 50


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('custom_turtlesim_simulator')

        self.sprite_images = {}
        self.load_sprite_images()

        self.turtles = {}
        self.next_id = 1
        self.bg_r, self.bg_g, self.bg_b = 255, 255, 255

        self.spawn_srv = self.create_service(Spawn, 'spawn_turtle', self.spawn_callback)
        self.bg_srv = self.create_service(SetBgColor, 'set_background_color', self.bg_callback)
        self.sprite_srv = self.create_service(ChangeSprite, 'change_turtle_sprite', self.sprite_callback)
        self.kill_srv = self.create_service(Kill, 'kill_turtle', self.kill_callback)

        self.timer = self.create_timer(0.1, self.update_simulation)

        self.pygame_initialized = False
        self.screen = None

        self.get_logger().info('Custom Turtlesim Simulator ready!')

    def ensure_pygame(self):
        """Abre ou fecha a janela do PyGame conforme presença de tartarugas."""
        if self.turtles and not self.pygame_initialized:
            pygame.init()
            self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
            pygame.display.set_caption("Custom Turtlesim")
            self.pygame_initialized = True
            self.get_logger().info("Janela gráfica aberta.")

        elif not self.turtles and self.pygame_initialized:
            pygame.quit()
            self.pygame_initialized = False
            self.screen = None
            self.get_logger().info("Janela gráfica fechada.")

    def spawn_callback(self, request, response):
        name = request.name if request.name else f"turtle{self.next_id}"
        turtle_id = self.next_id
        self.next_id += 1

        self.turtles[turtle_id] = Turtle(turtle_id, name, request.x, request.y, request.theta)

        topic = f'/{name}/cmd_vel'

        self.create_subscription(Twist, topic, lambda msg, tid=turtle_id: self.cmd_vel_callback(msg, tid), 10)

        self.get_logger().info(f"Spawned {name} (ID={turtle_id}) at ({request.x}, {request.y})")

        response.turtle_id = turtle_id
        response.name = name
        return response

    def cmd_vel_callback(self, msg, turtle_id):
        if turtle_id in self.turtles:
            self.turtles[turtle_id].vx = msg.linear.x
            self.turtles[turtle_id].vtheta = msg.angular.z

    def bg_callback(self, request, response):
        self.bg_r= request.red
        self.bg_g= request.green
        self.bg_b= request.blue

        self.get_logger().info(f"Background color set to RGB({self.bg_r}, {self.bg_g}, {self.bg_b})")
        response.success = True
        return response

    def sprite_callback(self, request, response):
        response.success = False
        if request.turtle_id in self.turtles:
            self.turtles[request.turtle_id].sprite = request.sprite_name
            self.get_logger().info(f"Turtle {request.turtle_id} sprite changed to '{request.sprite_name}'")
            response.success = True
        return response

    def kill_callback(self, request, response):
        turtle_to_remove = None
        for ide, turtle in self.turtles.items():
            if turtle.name == request.name:
                turtle_to_remove = ide
                break

        if turtle_to_remove is not None:
            del self.turtles[turtle_to_remove]
            self.get_logger().info(f"Turtle '{request.name}' killed.")
            response.success = True
        else:
            self.get_logger().warn(f"Turtle '{request.name}' not found.")
            response.success = False

        return response

    def load_sprite_images(self):
        pkg_share = get_package_share_directory('custom_turtlesim')
        resources_path = os.path.join(pkg_share, 'resources')

        sprite_names = [
            'ardent', 'bouncy', 'crystal', 'dashing', 'eloquent',
            'foxy', 'galactic', 'humble', 'iron', 'jazzy', 'rolling'
        ]

        for sprite_name in sprite_names:
            filename = f"{sprite_name}.png"
            img_path = os.path.join(resources_path, filename)
            if os.path.exists(img_path):
                try:
                    img = pygame.image.load(img_path)
                    img = pygame.transform.scale(img, (64, 64))
                    self.sprite_images[sprite_name] = img
                except pygame.error:
                    self.get_logger().warn(f"Não foi possível carregar imagem: {img_path}")
                    self.sprite_images[sprite_name] = None
            else:
                self.get_logger().warn(f"Imagem não encontrada: {img_path}")
                self.sprite_images[sprite_name] = None


    def update_simulation(self):
        dt = 0.1


        for turtle in self.turtles.values():

            turtle.x += turtle.vx * dt * math.cos(turtle.theta)
            turtle.y += turtle.vx * dt * math.sin(turtle.theta)
            turtle.theta += turtle.vtheta * dt

            turtle.x = max(0.0, min(10.0, turtle.x))
            turtle.y = max(0.0, min(10.0, turtle.y))

            turtle.trail.append((turtle.x, turtle.y))
            if len(turtle.trail) > turtle.max_trail_lenght:
                turtle.trail.pop(0)

        self.ensure_pygame()
        if self.pygame_initialized:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rclpy.shutdown

            self.screen.fill((self.bg_r, self.bg_g, self.bg_b))

            for turtle in self.turtles.values():
                px = int((turtle.x / WORLD_SIZE) * WINDOW_WIDTH)
                py = int((turtle.y / WORLD_SIZE) * WINDOW_HEIGHT)

                if turtle.sprite == "robot":
                    color = (255, 0, 0)     
                elif turtle.sprite == "cat":
                    color = (255, 165, 0)   
                elif turtle.sprite == "spaceship":
                    color = (0, 255, 255)   
                else:
                    color = (0, 0, 0)

                if len(turtle.trail) > 1:
                    trail_points = []
                    for x, y in turtle.trail:
                        px =int((x/WORLD_SIZE) * WINDOW_WIDTH)
                        py =int((y/WORLD_SIZE) * WINDOW_HEIGHT)
                        trail_points.append((px, py))

                    if len(trail_points) > 1:
                        pygame.draw.lines(self.screen, color, False, trail_points, 2)

                img = self.sprite_images.get(turtle.sprite)
                if img:
                    angle_deg = math.degrees(turtle.theta)
                    corrected_angle = -angle_deg - 90  
                    rotated_img = pygame.transform.rotate(img, corrected_angle)
                    rect = rotated_img.get_rect(center=(px, py))
                    self.screen.blit(rotated_img, rect)
                else:
                    pygame.draw.circle(self.screen, (0, 0, 0), (px, py), 12)

            pygame.display.flip()

        if self.turtles:
            status = ", ".join([f"{t.name}: ({t.x:.1f}, {t.y:.1f})" for t in self.turtles.values()])
            self.get_logger().debug(f"Turtles: {status}")

def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('nó interrompido pelo o usuario!')
    finally:
        if hasattr(node, 'pygame_initialized') and node.pygame_initialized:
            pygame.quit()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()














